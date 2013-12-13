#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "tm_i2c.h"
#include <sys/mman.h>
#include <stdio.h>
#include <unistd.h>

static volatile unsigned *gpiom;

#define INP_GPIO(g) *(gpiom+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpiom+((g)/10)) |=  (1<<(((g)%10)*3))
#define GPIO_SET *(gpiom+7)
#define GPIO_CLR *(gpiom+10)

float tm_i2c_Data2Temp(unsigned int ans) {
	float t = ans;
	return (t / 1023.0 * 3.3 * 2-2.73) * 100.0;
}

float tm_i2c_Data2Core(unsigned int ans) {
	float t = ans;
	return t / 1023.0 * 3.3;
}

int tm_i2c_init() {
	int i;
	int fd;

	fd = open("/dev/mem",O_RDWR|O_SYNC);
	if (fd < 0) { perror("/dev/mem trouble"); exit(1); }
	gpiom = mmap(0,4096,PROT_READ|PROT_WRITE,MAP_SHARED,fd,0x20200000);
	if (gpiom == MAP_FAILED) { perror("gpio mmap trouble"); exit(1); }
	close(fd);

	INP_GPIO(17); OUT_GPIO(17);
	INP_GPIO(18); OUT_GPIO(18);
	for(i = 0; i < 4 ; i++) {
		INP_GPIO(i + 22); OUT_GPIO(i + 22);
		GPIO_CLR = 1 << (i + 22);
	}
	INP_GPIO(27); OUT_GPIO(27);

	GPIO_CLR = 1 << 27;
	usleep(1);
	GPIO_SET = 1 << 27;


}

void tm_i2c_close() {
	close(tm_i2c_fd);
}

unsigned int tm_i2c_req(int fd, unsigned char addr, unsigned char cmd, unsigned int data) {
	int i;
	unsigned char buf[16];
	struct i2c_msg msg;
	tm_struct *tm = (tm_struct *) buf;
	struct i2c_rdwr_ioctl_data msg_rdwr;
	unsigned int ret;

	//printf("REQ from %02X cmd: %02X\n", addr, cmd);

	tm->cmd = cmd;
	tm->data_lsb = data & 0xFF;
	tm->data_msb = (data & 0xFF00) >> 8;

	/* Write CMD */
	msg.addr = addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = buf;
	msg_rdwr.msgs = &msg;
	msg_rdwr.nmsgs = 1;
	if ((i = ioctl(fd, I2C_RDWR, &msg_rdwr)) < 0) {
//		perror("ioctl error");
		return -1;
	}

	/* Read result */
	msg.addr = addr;
	msg.flags = I2C_M_RD;
	msg.len = 3;
	msg.buf = buf;
	msg_rdwr.msgs = &msg;
	msg_rdwr.nmsgs = 1;
	if ((i = ioctl(fd, I2C_RDWR, &msg_rdwr)) < 0) {
//		perror("ioctl error");
		return -1;
	}

	//hexdump(buf, 10);
	ret = (tm->data_msb << 8) + tm->data_lsb;
	if (tm->cmd == cmd) return ret;
	return 0;
}

int tm_i2c_detect(unsigned char slot) {
	if (slot < 0 || slot > 15) return 0;
	//return tm_i2c_req(tm_i2c_fd, (TM_ADDR >> 1) + (slot >> 1), TM_GET_CORE0, 0);
	return 1;
}

float tm_i2c_getcore0(unsigned char slot) {
	if (slot < 0 || slot > 31) return 0;
	return tm_i2c_Data2Core(tm_i2c_req(tm_i2c_fd, (TM_ADDR >> 1) + slot, TM_GET_CORE0, 0));
}

float tm_i2c_getcore1(unsigned char slot) {
	if (slot < 0 || slot > 31) return 0;
	return tm_i2c_Data2Core(tm_i2c_req(tm_i2c_fd, (TM_ADDR >> 1) + slot, TM_GET_CORE1, 0));
}

float tm_i2c_gettemp(unsigned char slot) {
	if (slot < 0 || slot > 31) return 0;
	return tm_i2c_Data2Temp(tm_i2c_req(tm_i2c_fd, (TM_ADDR >> 1) + slot, TM_GET_TEMP, 0));
}
/*
void tm_i2c_set_oe(unsigned char slot) {
	if (slot < 0 || slot > 31) return;
	tm_i2c_req(tm_i2c_fd, (TM_ADDR >> 1) + slot, TM_SET_OE, 0);
}

void tm_i2c_clear_oe(unsigned char slot) {
	if (slot < 0 || slot > 31) return;
	tm_i2c_req(tm_i2c_fd, (TM_ADDR >> 1) + slot, TM_SET_OE, 1);
}
*/
void tm_i2c_set_oe(unsigned char slot) {
	int i;

	if (slot < 0 || slot > 15) return;
	for(i = 0 ; i < 4 ; i++) {
		if (slot & (1 << i)) GPIO_SET = 1 << (i + 22);
		else GPIO_CLR = 1 << (i + 22);
	}
	GPIO_CLR = 1 << 27;
}

void tm_i2c_clear_oe(unsigned char slot) {
	if (slot < 0 || slot > 15) return;
	GPIO_SET = 1 << 27;
}

unsigned char tm_i2c_slot2addr(unsigned char slot) {
	if (slot < 0 || slot > 31) return 0;
	return ((TM_ADDR >> 1) + slot);
}

void out_led(int * led_c) {
	int i, j;

	GPIO_SET = 1 << 17;
	GPIO_SET = 1 << 18;
	GPIO_CLR = 1 << 18;
	for (j = 0; j < 8; j++) {
		if (led_c[7 - j]) {
			GPIO_CLR = 1 << 17;
		} else {
			GPIO_SET = 1 << 17;
		}
		GPIO_SET = 1 << 18;
		GPIO_CLR = 1 << 18;
	}
		GPIO_SET = 1 << 18;
	return;
}
