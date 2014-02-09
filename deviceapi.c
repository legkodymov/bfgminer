/*
 * Copyright 2011-2013 Luke Dashjr
 * Copyright 2011-2012 Con Kolivas
 * Copyright 2012-2013 Andrew Smith
 * Copyright 2010 Jeff Garzik
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "config.h"

#ifdef WIN32
#include <winsock2.h>
#else
#include <sys/select.h>
#endif
#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "compat.h"
#include "deviceapi.h"
#include "logging.h"
#include "lowlevel.h"
#ifdef NEED_BFG_LOWL_VCOM
#include "lowl-vcom.h"
#endif
#include "miner.h"
#include "util.h"

struct driver_registration *_bfg_drvreg1;
struct driver_registration *_bfg_drvreg2;

void _bfg_register_driver(const struct device_drv *drv)
{
	static struct driver_registration *initlist;
	struct driver_registration *ndr;
	
	if (!drv)
	{
		// Move initlist to hashtables
		LL_FOREACH(initlist, ndr)
		{
			drv = ndr->drv;
			if (drv->drv_init)
				drv->drv_init();
			HASH_ADD_KEYPTR(hh , _bfg_drvreg1, drv->dname, strlen(drv->dname), ndr);
			HASH_ADD_KEYPTR(hh2, _bfg_drvreg2, drv->name , strlen(drv->name ), ndr);
		}
		initlist = NULL;
		return;
	}
	
	ndr = malloc(sizeof(*ndr));
	*ndr = (struct driver_registration){
		.drv = drv,
	};
	LL_PREPEND(initlist, ndr);
}

static
int sort_drv_by_dname(struct driver_registration * const a, struct driver_registration * const b)
{
	return strcmp(a->drv->dname, b->drv->dname);
};

static
int sort_drv_by_priority(struct driver_registration * const a, struct driver_registration * const b)
{
	return a->drv->probe_priority - b->drv->probe_priority;
};

void bfg_devapi_init()
{
	_bfg_register_driver(NULL);
	HASH_SRT(hh , _bfg_drvreg1, sort_drv_by_dname   );
	HASH_SRT(hh2, _bfg_drvreg2, sort_drv_by_priority);
}


bool hashes_done(struct thr_info *thr, int64_t hashes, struct timeval *tvp_hashes, uint32_t *max_nonce)
{
	struct cgpu_info *cgpu = thr->cgpu;
	const long cycle = opt_log_interval / 5 ? : 1;
	
	if (unlikely(hashes == -1)) {
		if (timer_elapsed(&cgpu->tv_device_last_not_well, NULL) > 0)
			dev_error(cgpu, REASON_THREAD_ZERO_HASH);
		
		if (thr->scanhash_working && opt_restart) {
			applog(LOG_ERR, "%"PRIpreprv" failure, attempting to reinitialize", cgpu->proc_repr);
			thr->scanhash_working = false;
			cgpu->reinit_backoff = 5.2734375;
			hashes = 0;
		} else {
			applog(LOG_ERR, "%"PRIpreprv" failure, disabling!", cgpu->proc_repr);
			cgpu->deven = DEV_RECOVER_ERR;
			run_cmd(cmd_idle);
			return false;
		}
	}
	else
		thr->scanhash_working = true;
	
	thr->hashes_done += hashes;
	if (hashes > cgpu->max_hashes)
		cgpu->max_hashes = hashes;
	
	timeradd(&thr->tv_hashes_done, tvp_hashes, &thr->tv_hashes_done);
	
	// max_nonce management (optional)
	if (unlikely((long)thr->tv_hashes_done.tv_sec < cycle)) {
		int mult;
		
		if (likely(!max_nonce || *max_nonce == 0xffffffff))
			return true;
		
		mult = 1000000 / ((thr->tv_hashes_done.tv_usec + 0x400) / 0x400) + 0x10;
		mult *= cycle;
		if (*max_nonce > (0xffffffff * 0x400) / mult)
			*max_nonce = 0xffffffff;
		else
			*max_nonce = (*max_nonce * mult) / 0x400;
	} else if (unlikely(thr->tv_hashes_done.tv_sec > cycle) && max_nonce)
		*max_nonce = *max_nonce * cycle / thr->tv_hashes_done.tv_sec;
	else if (unlikely(thr->tv_hashes_done.tv_usec > 100000) && max_nonce)
		*max_nonce = *max_nonce * 0x400 / (((cycle * 1000000) + thr->tv_hashes_done.tv_usec) / (cycle * 1000000 / 0x400));
	
	hashmeter2(thr);
	
	return true;
}

bool hashes_done2(struct thr_info *thr, int64_t hashes, uint32_t *max_nonce)
{
	struct timeval tv_now, tv_delta;
	timer_set_now(&tv_now);
	timersub(&tv_now, &thr->_tv_last_hashes_done_call, &tv_delta);
	thr->_tv_last_hashes_done_call = tv_now;
	return hashes_done(thr, hashes, &tv_delta, max_nonce);
}

/* A generic wait function for threads that poll that will wait a specified
 * time tdiff waiting on a work restart request. Returns zero if the condition
 * was met (work restart requested) or ETIMEDOUT if not.
 */
int restart_wait(struct thr_info *thr, unsigned int mstime)
{
	struct timeval tv_timer, tv_now, tv_timeout;
	fd_set rfds;
	SOCKETTYPE wrn = thr->work_restart_notifier[0];
	int rv;
	
	if (unlikely(thr->work_restart_notifier[1] == INVSOCK))
	{
		// This is a bug!
		applog(LOG_ERR, "%"PRIpreprv": restart_wait called without a work_restart_notifier", thr->cgpu->proc_repr);
		cgsleep_ms(mstime);
		return (thr->work_restart ? 0 : ETIMEDOUT);
	}
	
	timer_set_now(&tv_now);
	timer_set_delay(&tv_timer, &tv_now, mstime * 1000);
	while (true)
	{
		FD_ZERO(&rfds);
		FD_SET(wrn, &rfds);
		tv_timeout = tv_timer;
		rv = select(wrn + 1, &rfds, NULL, NULL, select_timeout(&tv_timeout, &tv_now));
		if (rv == 0)
			return ETIMEDOUT;
		if (rv > 0)
		{
			if (thr->work_restart)
				return 0;
			notifier_read(thr->work_restart_notifier);
		}
		timer_set_now(&tv_now);
	}
}

static
struct work *get_and_prepare_work(struct thr_info *thr)
{
	struct cgpu_info *proc = thr->cgpu;
	struct device_drv *api = proc->drv;
	struct work *work;
	
	work = get_work(thr);
	if (!work)
		return NULL;
	if (api->prepare_work && !api->prepare_work(thr, work)) {
		free_work(work);
		applog(LOG_ERR, "%"PRIpreprv": Work prepare failed, disabling!", proc->proc_repr);
		proc->deven = DEV_RECOVER_ERR;
		run_cmd(cmd_idle);
		return NULL;
	}
	return work;
}

// Miner loop to manage a single processor (with possibly multiple threads per processor)
void minerloop_scanhash(struct thr_info *mythr)
{
	struct cgpu_info *cgpu = mythr->cgpu;
	struct device_drv *api = cgpu->drv;
	struct timeval tv_start, tv_end;
	struct timeval tv_hashes, tv_worktime;
	uint32_t max_nonce = api->can_limit_work ? api->can_limit_work(mythr) : 0xffffffff;
	int64_t hashes;
	struct work *work;
	const bool primary = (!mythr->device_thread) || mythr->primary_thread;
	
#ifdef HAVE_PTHREAD_CANCEL
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);
#endif
	
	if (cgpu->deven != DEV_ENABLED)
		mt_disable(mythr);
	
	while (likely(!cgpu->shutdown)) {
		mythr->work_restart = false;
		request_work(mythr);
		work = get_and_prepare_work(mythr);
		if (!work)
			break;
		timer_set_now(&work->tv_work_start);
		
		do {
			thread_reportin(mythr);
			/* Only allow the mining thread to be cancelled when
			* it is not in the driver code. */
			pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);
			timer_set_now(&tv_start);
			hashes = api->scanhash(mythr, work, work->blk.nonce + max_nonce);
			timer_set_now(&tv_end);
			pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
			pthread_testcancel();
			thread_reportin(mythr);
			
			timersub(&tv_end, &tv_start, &tv_hashes);
			if (!hashes_done(mythr, hashes, &tv_hashes, api->can_limit_work ? &max_nonce : NULL))
				goto disabled;
			
			if (unlikely(mythr->work_restart)) {
				/* Apart from device_thread 0, we stagger the
				 * starting of every next thread to try and get
				 * all devices busy before worrying about
				 * getting work for their extra threads */
				if (!primary) {
					struct timespec rgtp;

					rgtp.tv_sec = 0;
					rgtp.tv_nsec = 250 * mythr->device_thread * 1000000;
					nanosleep(&rgtp, NULL);
				}
				break;
			}
			
			if (unlikely(mythr->pause || cgpu->deven != DEV_ENABLED))
disabled:
				mt_disable(mythr);
			
			timersub(&tv_end, &work->tv_work_start, &tv_worktime);
		} while (!abandon_work(work, &tv_worktime, cgpu->max_hashes));
		free_work(work);
	}
}

bool do_job_prepare(struct thr_info *mythr, struct timeval *tvp_now)
{
	struct cgpu_info *proc = mythr->cgpu;
	struct device_drv *api = proc->drv;
	struct timeval tv_worktime;
	
	mythr->tv_morework.tv_sec = -1;
	mythr->_job_transition_in_progress = true;
	if (mythr->work)
		timersub(tvp_now, &mythr->work->tv_work_start, &tv_worktime);
	if ((!mythr->work) || abandon_work(mythr->work, &tv_worktime, proc->max_hashes))
	{
		mythr->work_restart = false;
		request_work(mythr);
		// FIXME: Allow get_work to return NULL to retry on notification
		if (mythr->next_work)
			free_work(mythr->next_work);
		mythr->next_work = get_and_prepare_work(mythr);
		if (!mythr->next_work)
			return false;
		mythr->starting_next_work = true;
		api->job_prepare(mythr, mythr->next_work, mythr->_max_nonce);
	}
	else
	{
		mythr->starting_next_work = false;
		api->job_prepare(mythr, mythr->work, mythr->_max_nonce);
	}
	job_prepare_complete(mythr);
	return true;
}

void job_prepare_complete(struct thr_info *mythr)
{
	if (unlikely(mythr->busy_state == TBS_GETTING_RESULTS))
		return;
	if (mythr->work)
	{
		if (true /* TODO: job is near complete */ || unlikely(mythr->work_restart))
			do_get_results(mythr, true);
		else
		{}  // TODO: Set a timer to call do_get_results when job is near complete
	}
	else  // no job currently running
		do_job_start(mythr);
}

void do_get_results(struct thr_info *mythr, bool proceed_with_new_job)
{
	struct cgpu_info *proc = mythr->cgpu;
	struct device_drv *api = proc->drv;
	struct work *work = mythr->work;
	
	mythr->_job_transition_in_progress = true;
	mythr->tv_results_jobstart = mythr->tv_jobstart;
	mythr->_proceed_with_new_job = proceed_with_new_job;
	if (api->job_get_results)
		api->job_get_results(mythr, work);
	else
		job_results_fetched(mythr);
}

void job_results_fetched(struct thr_info *mythr)
{
	if (mythr->_proceed_with_new_job)
		do_job_start(mythr);
	else
	{
		if (likely(mythr->prev_work))
		{
			struct timeval tv_now;
			
			timer_set_now(&tv_now);
			
			do_process_results(mythr, &tv_now, mythr->prev_work, true);
		}
		mt_disable_start(mythr);
	}
}

void do_job_start(struct thr_info *mythr)
{
	struct cgpu_info *proc = mythr->cgpu;
	struct device_drv *api = proc->drv;
	
	thread_reportin(mythr);
	api->job_start(mythr);
}

void mt_job_transition(struct thr_info *mythr)
{
	struct timeval tv_now;
	
	timer_set_now(&tv_now);
	
	if (mythr->starting_next_work)
	{
		mythr->next_work->tv_work_start = tv_now;
		if (mythr->prev4_work)
			free_work(mythr->prev4_work);
		mythr->prev4_work = mythr->prev3_work;
		mythr->prev3_work = mythr->prev2_work;
		mythr->prev2_work = mythr->prev_work;
		mythr->prev_work = mythr->work;
		mythr->work = mythr->next_work;
		mythr->next_work = NULL;
	}
	mythr->tv_jobstart = tv_now;
	mythr->_job_transition_in_progress = false;
}

void job_start_complete(struct thr_info *mythr)
{
	struct timeval tv_now;
	
	if (unlikely(!mythr->prev_work))
		return;
	
	timer_set_now(&tv_now);
	
	do_process_results(mythr, &tv_now, mythr->prev_work, false);
}

void job_start_abort(struct thr_info *mythr, bool failure)
{
	struct cgpu_info *proc = mythr->cgpu;
	
	if (failure)
	{
		proc->deven = DEV_RECOVER_ERR;
		run_cmd(cmd_idle);
	}
	mythr->work = NULL;
	mythr->_job_transition_in_progress = false;
}

bool do_process_results(struct thr_info *mythr, struct timeval *tvp_now, struct work *work, bool stopping)
{
	struct cgpu_info *proc = mythr->cgpu;
	struct device_drv *api = proc->drv;
	struct timeval tv_hashes;
	int64_t hashes = 0;
	
	if (api->job_process_results)
		hashes = api->job_process_results(mythr, work, stopping);
	thread_reportin(mythr);
	
	if (hashes)
	{
		timersub(tvp_now, &mythr->tv_results_jobstart, &tv_hashes);
		if (!hashes_done(mythr, hashes, &tv_hashes, api->can_limit_work ? &mythr->_max_nonce : NULL))
			return false;
	}
	
	return true;
}

static
void do_notifier_select(struct thr_info *thr, struct timeval *tvp_timeout)
{
	struct cgpu_info *cgpu = thr->cgpu;
	struct timeval tv_now;
	int maxfd;
	fd_set rfds;
	
	timer_set_now(&tv_now);
	FD_ZERO(&rfds);
	FD_SET(thr->notifier[0], &rfds);
	maxfd = thr->notifier[0];
	FD_SET(thr->work_restart_notifier[0], &rfds);
	set_maxfd(&maxfd, thr->work_restart_notifier[0]);
	if (thr->mutex_request[1] != INVSOCK)
	{
		FD_SET(thr->mutex_request[0], &rfds);
		set_maxfd(&maxfd, thr->mutex_request[0]);
	}
	if (select(maxfd + 1, &rfds, NULL, NULL, select_timeout(tvp_timeout, &tv_now)) < 0)
		return;
	if (thr->mutex_request[1] != INVSOCK && FD_ISSET(thr->mutex_request[0], &rfds))
	{
		// FIXME: This can only handle one request at a time!
		pthread_mutex_t *mutexp = &cgpu->device_mutex;
		notifier_read(thr->mutex_request);
		mutex_lock(mutexp);
		pthread_cond_signal(&cgpu->device_cond);
		pthread_cond_wait(&cgpu->device_cond, mutexp);
		mutex_unlock(mutexp);
	}
	if (FD_ISSET(thr->notifier[0], &rfds)) {
		notifier_read(thr->notifier);
	}
	if (FD_ISSET(thr->work_restart_notifier[0], &rfds))
		notifier_read(thr->work_restart_notifier);
}

static
void _minerloop_setup(struct thr_info *mythr)
{
	struct cgpu_info * const cgpu = mythr->cgpu, *proc;
	
	if (mythr->work_restart_notifier[1] == -1)
		notifier_init(mythr->work_restart_notifier);
	
	for (proc = cgpu; proc; proc = proc->next_proc)
	{
		mythr = proc->thr[0];
		timer_set_now(&mythr->tv_watchdog);
		proc->disable_watchdog = true;
	}
}

void minerloop_async(struct thr_info *mythr)
{
	struct thr_info *thr = mythr;
	struct cgpu_info *cgpu = mythr->cgpu;
	struct device_drv *api = cgpu->drv;
	struct timeval tv_now;
	struct timeval tv_timeout;
	struct cgpu_info *proc;
	bool is_running, should_be_running;
	
	_minerloop_setup(mythr);
	
	while (likely(!cgpu->shutdown)) {
		tv_timeout.tv_sec = -1;
		timer_set_now(&tv_now);
		for (proc = cgpu; proc; proc = proc->next_proc)
		{
			mythr = proc->thr[0];
			
			// Nothing should happen while we're starting a job
			if (unlikely(mythr->busy_state == TBS_STARTING_JOB))
				goto defer_events;
			
			is_running = mythr->work;
			should_be_running = (proc->deven == DEV_ENABLED && !mythr->pause);
			
			if (should_be_running)
			{
				if (unlikely(!(is_running || mythr->_job_transition_in_progress)))
				{
					mt_disable_finish(mythr);
					goto djp;
				}
				if (unlikely(mythr->work_restart))
					goto djp;
			}
			else  // ! should_be_running
			{
				if (unlikely((is_running || !mythr->_mt_disable_called) && !mythr->_job_transition_in_progress))
				{
disabled: ;
					timer_unset(&mythr->tv_morework);
					if (is_running)
					{
						if (mythr->busy_state != TBS_GETTING_RESULTS)
							do_get_results(mythr, false);
						else
							// Avoid starting job when pending result fetch completes
							mythr->_proceed_with_new_job = false;
					}
					else  // !mythr->_mt_disable_called
						mt_disable_start(mythr);
				}
			}
			
			if (timer_passed(&mythr->tv_morework, &tv_now))
			{
djp: ;
				if (!do_job_prepare(mythr, &tv_now))
					goto disabled;
			}
			
defer_events:
			if (timer_passed(&mythr->tv_poll, &tv_now))
				api->poll(mythr);
			
			if (timer_passed(&mythr->tv_watchdog, &tv_now))
			{
				timer_set_delay(&mythr->tv_watchdog, &tv_now, WATCHDOG_INTERVAL * 1000000);
				bfg_watchdog(proc, &tv_now);
			}
			
			reduce_timeout_to(&tv_timeout, &mythr->tv_morework);
			reduce_timeout_to(&tv_timeout, &mythr->tv_poll);
			reduce_timeout_to(&tv_timeout, &mythr->tv_watchdog);
		}
		
		do_notifier_select(thr, &tv_timeout);
	}
}

static
void do_queue_flush(struct thr_info *mythr)
{
	struct cgpu_info *proc = mythr->cgpu;
	struct device_drv *api = proc->drv;
	
	api->queue_flush(mythr);
	if (mythr->next_work)
	{
		free_work(mythr->next_work);
		mythr->next_work = NULL;
	}
}

void minerloop_queue(struct thr_info *thr)
{
	struct thr_info *mythr;
	struct cgpu_info *cgpu = thr->cgpu;
	struct device_drv *api = cgpu->drv;
	struct timeval tv_now;
	struct timeval tv_timeout;
	struct cgpu_info *proc;
	bool should_be_running;
	struct work *work;
	
	_minerloop_setup(thr);
	
	while (likely(!cgpu->shutdown)) {
		tv_timeout.tv_sec = -1;
		timer_set_now(&tv_now);
		for (proc = cgpu; proc; proc = proc->next_proc)
		{
			mythr = proc->thr[0];
			
			should_be_running = (proc->deven == DEV_ENABLED && !mythr->pause);
redo:
			if (should_be_running)
			{
				if (unlikely(mythr->_mt_disable_called))
					mt_disable_finish(mythr);
				
				if (unlikely(mythr->work_restart))
				{
					mythr->work_restart = false;
					do_queue_flush(mythr);
				}
				
				while (!mythr->queue_full)
				{
					if (mythr->next_work)
					{
						work = mythr->next_work;
						mythr->next_work = NULL;
					}
					else
					{
						request_work(mythr);
						// FIXME: Allow get_work to return NULL to retry on notification
						work = get_and_prepare_work(mythr);
					}
					if (!work)
						break;
					if (!api->queue_append(mythr, work))
						mythr->next_work = work;
				}
			}
			else
			if (unlikely(!mythr->_mt_disable_called))
			{
				do_queue_flush(mythr);
				mt_disable_start(mythr);
			}
			
			if (timer_passed(&mythr->tv_poll, &tv_now))
				api->poll(mythr);
			
			if (timer_passed(&mythr->tv_watchdog, &tv_now))
			{
				timer_set_delay(&mythr->tv_watchdog, &tv_now, WATCHDOG_INTERVAL * 1000000);
				bfg_watchdog(proc, &tv_now);
			}
			
			should_be_running = (proc->deven == DEV_ENABLED && !mythr->pause);
			if (should_be_running && !mythr->queue_full)
				goto redo;
			
			reduce_timeout_to(&tv_timeout, &mythr->tv_poll);
			reduce_timeout_to(&tv_timeout, &mythr->tv_watchdog);
		}
		
		do_notifier_select(thr, &tv_timeout);
	}
}

void *miner_thread(void *userdata)
{
	struct thr_info *mythr = userdata;
	struct cgpu_info *cgpu = mythr->cgpu;
	struct device_drv *drv = cgpu->drv;

	pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

	char threadname[20];
	snprintf(threadname, 20, "miner_%s", cgpu->proc_repr_ns);
	RenameThread(threadname);

	if (drv->thread_init && !drv->thread_init(mythr)) {
		dev_error(cgpu, REASON_THREAD_FAIL_INIT);
		for (struct cgpu_info *slave = cgpu->next_proc; slave && !slave->threads; slave = slave->next_proc)
			dev_error(slave, REASON_THREAD_FAIL_INIT);
		__thr_being_msg(LOG_ERR, mythr, "failure, exiting");
		goto out;
	}

	if (drv_ready(cgpu))
		cgpu_set_defaults(cgpu);
	
	thread_reportout(mythr);
	applog(LOG_DEBUG, "Popping ping in miner thread");
	notifier_read(mythr->notifier);  // Wait for a notification to start
	
	cgtime(&cgpu->cgminer_stats.start_tv);
	if (drv->minerloop)
		drv->minerloop(mythr);
	else
		minerloop_scanhash(mythr);
	__thr_being_msg(LOG_NOTICE, mythr, "shutting down");

out: ;
	struct cgpu_info *proc = cgpu;
	do
	{
		proc->deven = DEV_DISABLED;
		proc->status = LIFE_DEAD2;
	}
	while ( (proc = proc->next_proc) && !proc->threads);
	mythr->getwork = 0;
	mythr->has_pth = false;
	cgsleep_ms(1);
	
	if (drv->thread_shutdown)
		drv->thread_shutdown(mythr);

	notifier_destroy(mythr->notifier);

	return NULL;
}

static pthread_mutex_t _add_cgpu_mutex = PTHREAD_MUTEX_INITIALIZER;

static
bool _add_cgpu(struct cgpu_info *cgpu)
{
	int lpcount;
	
	renumber_cgpu(cgpu);
	if (!cgpu->procs)
		cgpu->procs = 1;
	lpcount = cgpu->procs;
	cgpu->device = cgpu;
	
	cgpu->dev_repr = malloc(6);
	sprintf(cgpu->dev_repr, "%s%2u", cgpu->drv->name, cgpu->device_id % 100);
	cgpu->dev_repr_ns = malloc(6);
	sprintf(cgpu->dev_repr_ns, "%s%u", cgpu->drv->name, cgpu->device_id % 100);
	strcpy(cgpu->proc_repr, cgpu->dev_repr);
	sprintf(cgpu->proc_repr_ns, "%s%u", cgpu->drv->name, cgpu->device_id);
	
#ifdef NEED_BFG_LOWL_VCOM
	maybe_strdup_if_null(&cgpu->dev_manufacturer, detectone_meta_info.manufacturer);
	maybe_strdup_if_null(&cgpu->dev_product,      detectone_meta_info.product);
	maybe_strdup_if_null(&cgpu->dev_serial,       detectone_meta_info.serial);
#endif
	
	devices_new = realloc(devices_new, sizeof(struct cgpu_info *) * (total_devices_new + lpcount + 1));
	devices_new[total_devices_new++] = cgpu;
	
	if (lpcount > 1)
	{
		int ns;
		int tpp = cgpu->threads / lpcount;
		struct cgpu_info **nlp_p, *slave;
		const bool manylp = (lpcount > 26);
		const char *as = (manylp ? "aa" : "a");
		
		// Note, strcpy instead of assigning a byte to get the \0 too
		strcpy(&cgpu->proc_repr[5], as);
		ns = strlen(cgpu->proc_repr_ns);
		strcpy(&cgpu->proc_repr_ns[ns], as);
		
		nlp_p = &cgpu->next_proc;
		for (int i = 1; i < lpcount; ++i)
		{
			slave = malloc(sizeof(*slave));
			*slave = *cgpu;
			slave->proc_id = i;
			if (manylp)
			{
				slave->proc_repr[5] += i / 26;
				slave->proc_repr[6] += i % 26;
				slave->proc_repr_ns[ns    ] += i / 26;
				slave->proc_repr_ns[ns + 1] += i % 26;
			}
			else
			{
				slave->proc_repr[5] += i;
				slave->proc_repr_ns[ns] += i;
			}
			slave->threads = tpp;
			devices_new[total_devices_new++] = slave;
			*nlp_p = slave;
			nlp_p = &slave->next_proc;
		}
		*nlp_p = NULL;
		cgpu->proc_id = 0;
		cgpu->threads -= (tpp * (lpcount - 1));
	}

	cgpu->last_device_valid_work = time(NULL);
	
	return true;
}

bool add_cgpu(struct cgpu_info *cgpu)
{
	mutex_lock(&_add_cgpu_mutex);
	const bool rv = _add_cgpu(cgpu);
	mutex_unlock(&_add_cgpu_mutex);
	return rv;
}

void add_cgpu_live(void *p)
{
	add_cgpu(p);
}

bool add_cgpu_slave(struct cgpu_info *cgpu, struct cgpu_info *prev_cgpu)
{
	if (!prev_cgpu)
		return add_cgpu(cgpu);
	
	while (prev_cgpu->next_proc)
		prev_cgpu = prev_cgpu->next_proc;
	
	mutex_lock(&_add_cgpu_mutex);
	
	int old_total_devices = total_devices_new;
	if (!_add_cgpu(cgpu))
	{
		mutex_unlock(&_add_cgpu_mutex);
		return false;
	}
	prev_cgpu->next_proc = devices_new[old_total_devices];
	
	mutex_unlock(&_add_cgpu_mutex);
	
	return true;
}

#ifdef NEED_BFG_LOWL_VCOM
bool _serial_detect_all(struct lowlevel_device_info * const info, void * const userp)
{
	detectone_func_t detectone = userp;
	
	if (serial_claim(info->path, NULL))
		applogr(false, LOG_DEBUG, "%s is already claimed... skipping probes", info->path);
	
	return detectone(info->path);
}
#endif

int _serial_detect(struct device_drv *api, detectone_func_t detectone, autoscan_func_t autoscan, int flags)
{
	struct string_elist *iter, *tmp;
	const char *dev, *colon;
	bool inhibitauto = flags & 4;
	char found = 0;
	bool forceauto = flags & 1;
	bool hasname;
	bool doall = false;
	size_t namel = strlen(api->name);
	size_t dnamel = strlen(api->dname);

#ifdef NEED_BFG_LOWL_VCOM
	clear_detectone_meta_info();
#endif
	DL_FOREACH_SAFE(scan_devices, iter, tmp) {
		dev = iter->string;
		if ((colon = strchr(dev, ':')) && colon[1] != '\0') {
			size_t idlen = colon - dev;

			// allow either name:device or dname:device
			if ((idlen != namel || strncasecmp(dev, api->name, idlen))
			&&  (idlen != dnamel || strncasecmp(dev, api->dname, idlen)))
				continue;

			dev = colon + 1;
			hasname = true;
		}
		else
			hasname = false;
		if (!strcmp(dev, "auto"))
			forceauto = true;
		else if (!strcmp(dev, "noauto"))
			inhibitauto = true;
		else
		if ((flags & 2) && !hasname)
			continue;
		else
		if (!detectone)
		{}  // do nothing
		else
		if (!strcmp(dev, "all"))
			doall = true;
#ifdef NEED_BFG_LOWL_VCOM
		else
		if (serial_claim(dev, NULL))
		{
			applog(LOG_DEBUG, "%s is already claimed... skipping probes", dev);
			string_elist_del(&scan_devices, iter);
		}
#endif
		else if (detectone(dev)) {
			string_elist_del(&scan_devices, iter);
			++found;
		}
	}

#ifdef NEED_BFG_LOWL_VCOM
	if (doall && detectone)
		found += lowlevel_detect_id(_serial_detect_all, detectone, &lowl_vcom, 0, 0);
#endif
	
	if ((forceauto || !(inhibitauto || found)) && autoscan)
		found += autoscan();

	return found;
}

static
FILE *_open_bitstream(const char *path, const char *subdir, const char *sub2, const char *filename)
{
	char fullpath[PATH_MAX];
	strcpy(fullpath, path);
	strcat(fullpath, "/");
	if (subdir) {
		strcat(fullpath, subdir);
		strcat(fullpath, "/");
	}
	if (sub2) {
		strcat(fullpath, sub2);
		strcat(fullpath, "/");
	}
	strcat(fullpath, filename);
	return fopen(fullpath, "rb");
}
#define _open_bitstream(path, subdir, sub2)  do {  \
	f = _open_bitstream(path, subdir, sub2, filename);  \
	if (f)  \
		return f;  \
} while(0)

#define _open_bitstream2(path, path3)  do {  \
	_open_bitstream(path, NULL, path3);  \
	_open_bitstream(path, "../share/" PACKAGE, path3);  \
	_open_bitstream(path, "../" PACKAGE, path3);  \
} while(0)

#define _open_bitstream3(path)  do {  \
	_open_bitstream2(path, dname);  \
	_open_bitstream2(path, "bitstreams");  \
	_open_bitstream2(path, NULL);  \
} while(0)

FILE *open_bitstream(const char *dname, const char *filename)
{
	FILE *f;

	_open_bitstream3(opt_kernel_path);
	_open_bitstream3(cgminer_path);
	_open_bitstream3(".");

	return NULL;
}

void close_device_fd(struct thr_info * const thr)
{
	struct cgpu_info * const proc = thr->cgpu;
	const int fd = proc->device_fd;
	
	if (fd == -1)
		return;
	
	if (close(fd))
		applog(LOG_WARNING, "%"PRIpreprv": Error closing device fd", proc->proc_repr);
	else
	{
		proc->device_fd = -1;
		applog(LOG_DEBUG, "%"PRIpreprv": Closed device fd", proc->proc_repr);
	}
}
