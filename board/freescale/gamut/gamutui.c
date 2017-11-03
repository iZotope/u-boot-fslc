/*
 * Copyright (C) 2016 iZotope, Inc.
 *
 * Author: Matthew Campbell <mcampbell@izotope.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 * This is a reduced scope implementation of the i.MX6 <-> Atmel protocol
 * in iZotope's Gamut. This is not meant to be fully functinal, just enough
 * for pre-boot behavior in u-boot.
 *
 */

#include <common.h>
#include "gamutui.h"
#include <spi.h>
#include <crc.h>
#include <asm/gpio.h>
/* we don't want appmsg_protocol.h including any std libs */
#define APPMSG_PROTOCOL_NO_INCLUDES
#include "hardware-shared-open/gamut/appmsg_protocol.h"

#define ATMEL_INT_GPIO IMX_GPIO_NR(2, 15)
#define ATMEL_RESET_GPIO IMX_GPIO_NR(2, 18)
#define ATMEL_SENSE_GPIO IMX_GPIO_NR(2, 19)
#define RECOVERY_BUTTON_HOLD BTN_LEARN
#define RECOVERY_BUTTON_RECOVERY BTN_REC
#define RECOVERY_BUTTON_UBOOT BTN_PLAY

struct gamutui_prv {
	bool inited;
	struct spi_slave *spi;
	uint8_t tx[128];
	uint8_t rx[128];
};
static struct gamutui_prv prv;

static ulong get_ms(void)
{
	unsigned long freq = get_tbclk()/1000;
	unsigned long ticks = get_ticks();

	return ticks / freq;
}

/* this transfers what's in the tx in the prv.
 * it will calculate and poplulate the CRC for you
 * returns 0 on success
 * returns -1 on tx error
 * returns -2 on rx error
 */
static int xfer_frame(void)
{
	uint8_t * tx = prv.tx;
	uint8_t * rx = prv.rx;

	/* calculate the CRC */
	uint16_t crc = cyg_crc16(tx, (tx[0] & 0x7F)+1);
	tx[123] = crc >> 8;
	tx[124] = crc & 0xFF;

	/* beacuse each byte is a sepearte transaction, we need to assert CS here */
	spi_xfer(prv.spi, 0, NULL, NULL, SPI_XFER_BEGIN);

	/* Note: this must be done in 8-bit bursts to ensure timing between the
	 * bytes for the Atmel. Do not try to do a xfer larger than 8 bits
	 */
	for (int i=0; i<125; i++) {
		spi_xfer(prv.spi, 8, &tx[i], &rx[i], 0);
	}
	/* don't bother checking the atmel CRC, just ack everything */
	tx[127] = rx[123] ^ 0xFF;
	tx[127] = (tx[127] & 0x3F) | (0x80);
	/* see note about about 8-bit burst transfers */
	for (int i=125; i<128; i++) {
		spi_xfer(prv.spi, 8, &tx[i], &rx[i], 0);
	}
	/* there is a problem with the Atmel not getting the TXCOMPLETE interrupt
	 * for the SS release if it is too close to the last byte. The docs say
	 * it should be ((1/(2*Fsclk))-9ns), which is ~1.3us for our 375kHz clock.
	 * Experimentally we need at least 10uS for reliable operation. We should
	 * This was seen on Rev C silicon, we should revist this with newer silicon
	 * to see if it's still there.
	 */
	udelay(30);
	/* Dummy transfer to de-assert SS */
	spi_xfer(prv.spi, 0, NULL, NULL, SPI_XFER_END);

	/* We also need to give the atmel some time to process the packet before
	 * another one could get sent. This time was found by exprimentally
	 * running a lot of transfers and giving 
	 */
	udelay(200);

	/* make sure the Atmel ack'd our packet */
	if ( ((rx[127] ^ 0xFF) & 0x3F) != ((crc >> 8) & 0x3F) ) {
		return -1;
	}

	/* check the Atmel CRC */
	uint8_t sz = (rx[2] & (~0x80)) + 1;
	if (sz > 119) {
		return -2;
	}
	crc = cyg_crc16(&rx[2], sz);
	uint16_t atmel_crc = rx[124] | rx[123] << 8;
	if(crc != atmel_crc) {
		return -2;
	}


	return 0;
}

/* TODO: need a timeout on this incase the int line is floating (atmel not preset/errored)! */
/* maybe call get_appmsg with an invalid msg_id? */
static void drain_atmel(void)
{
	uint8_t * tx = prv.tx;
	uint16_t iter = 0;
	tx[0] = 0x80;

	while(!gpio_get_value(ATMEL_INT_GPIO) && iter < 1000) {
		xfer_frame();
		iter++;
	}
}

static int send_appmsg(uint8_t msg_id, uint8_t *msg, int msg_size)
{
	uint8_t * tx = prv.tx;

	if (!prv.inited) {
		return -1;
	}

	memset(tx, 0, sizeof(tx));

	/* Always set the 'canrx' bit, we will just throw the data away, but we
	 * want to free up some space in the Atmel TX fifo for any possible reply.
	 */
	tx[0] = (msg_size + 2) | (0x80);
	tx[1] = msg_size + 1;
	tx[2] = msg_id;
	memcpy(&tx[3], msg, msg_size);

	if (xfer_frame() == -1) {
		return -1;
	}

	return 0;
}

/* returns zero if no errors, but message not found
 * returns number of bytes written into buf if message found
 * return negative if error
 */
static int read_single_appmsg(uint8_t msg_id, uint8_t *buf)
{
	uint8_t * tx = prv.tx;
	uint8_t * rx = prv.rx;

	if (!prv.inited) {
		return -1;
	}

	memset(tx, 0, sizeof(tx));

	/* send nothing, recieve it all */
	tx[0] = 0x80;

	if (xfer_frame() == -2) {
		/* ignore malformed frames */
		return 0;
	}

	/* loop over data looking for our appmsg */
	for (int i=3; i<123;) {
		uint8_t sz = rx[i];
		if (sz) {
			uint8_t payload_sz = sz -1;
			uint8_t id = rx[i+1];
			if(id == msg_id) {
				if (payload_sz > 119) {
					return -1;
				}
				memcpy(buf, &rx[i+2], payload_sz);
				return payload_sz;
			}
		}
		i += sz + 1;
	}

	return 0;
}

/* attmepts to get a specific app message until it comes, or a timeout 
 * returns 0 if timed out
 * returns negative if error
 * returns number of bytes in appmsge if found
 */
static int get_appmsg(uint8_t msg_id, uint8_t *msg, ulong ms_timeout)
{
	if (!prv.inited) {
		return -1;
	}

	ulong ms_end = get_ms() + ms_timeout;

	while(1) {
		if(!gpio_get_value(ATMEL_INT_GPIO)) {
			int ret = read_single_appmsg(msg_id, msg);
			if (ret < 0) {
				return -1;
			} else if (ret > 0) {
				return ret;
			}
		}

		if (get_ms() > ms_end) {
			return 0;
		}
	}
	/* can't get here */
	return -1;
}

int check_recovery(void) {
	uint8_t buf[128];
	int ret;

	if (getenv_yesno("skip_recovery") == 1) {
		return BOOT_NORMAL;
	}

	printf("Checking UI board\n");
	printf("\tdraining the atmel...\n");
	drain_atmel();

	printf("\trequesting button state...\n");
	ret = send_appmsg(MSG_BUTTON, 0, 0);
	if (ret) {
		printf("\tBad packet from atmel, recovery boot\n");
		return BOOT_RECOVERY_ERROR;
	}
	printf("\twait for response...\n");
	ret = get_appmsg(MSG_BUTTON, buf, 50);
	if (ret <= 0) {
		printf("\tBad packet from atmel, recovery boot\n");
		return BOOT_RECOVERY_ERROR;
	}

	uint32_t * btn = (void *)buf;
	printf("\tResponse is 0x%08X\n", *btn);
	if(*btn & BIT(RECOVERY_BUTTON_HOLD)) {
		/* Set single LED red to indicate ready for second press */
		gamutui_set_single_led(0xFF0000);

		printf("\twait for second button event\n");
		ret = get_appmsg(MSG_BUTTON, buf, 5000);
		if (ret > 0) {
			btn = (void *)buf;
			printf("\tResponse is 0x%08X\n", *btn);
			if (*btn & (BIT(RECOVERY_BUTTON_HOLD))) {
				if (*btn & BIT(RECOVERY_BUTTON_RECOVERY)) {
					return BOOT_RECOVERY_REQUESTED;
				} else if (*btn & BIT(RECOVERY_BUTTON_UBOOT)) {
					return BOOT_BREAK_UBOOT;
				}
			}
		} else if (ret < 0) {
			printf("\tBad packet from atmel, recovery boot\n");
			return BOOT_RECOVERY_ERROR;
		}
	}

	printf("\tAll is well, do a normal boot\n");
	return BOOT_NORMAL;
}

/* checks if we should do a recovery boot 
 * returns:
 * BOOT_NORMAL - do a normal boot
 * BOOT_RECOVERY_REQUESTED - do a recovery boot (UI requested)
 * BOOT_RECOVERY_ERROR - do a recovery boot (UI error)
 * BOOT_BREAK_UBOOT - break into u-boot
 */

int gamutui_check_recovery(void)
{
	int first_result = check_recovery();
	if (first_result != BOOT_RECOVERY_ERROR) {
		return first_result;
	}

	printf("Atmel came up in a bad state. Trying a reset.\n");
	mdelay(500); // Wait a bit in case the atmel is still booting
	gamutui_reset();
	mdelay(500); // Wait another bit for the atmel to come up again.
	return check_recovery();
}

/** set the whole LED ring to one color
 * color - color to set the LED ring to in the following format:
 *         0x00RRGGBB
 * return:
 *  0 - succes
 *  other - failure
 */
int gamutui_set_leds(uint32_t color)
{
	struct msg_led_update led;
	memset(&led, 0, sizeof(led));

	for (int i=0; i<LED_RING_PIXEL_COUNT; i++) {
		led.pixels[i].red =   (color >> 16) & 0xFF;
		led.pixels[i].green = (color >>  8) & 0xFF;
		led.pixels[i].blue =  (color >>  0) & 0xFF;
	}

	return send_appmsg(MSG_LED_RING_UPDATE, (void*)&led, sizeof(led));
}

/** set the center LED in the ring to one color
 * color - color to set the LED ring to in the following format:
 *         0x00RRGGBB
 * return:
 *  0 - succes
 *  other - failure
 */
int gamutui_set_single_led(uint32_t color)
{
	struct msg_led_update led;
	memset(&led, 0, sizeof(led));

	led.pixels[15].red =   (color >> 16) & 0xFF;
	led.pixels[15].green = (color >>  8) & 0xFF;
	led.pixels[15].blue =  (color >>  0) & 0xFF;

	return send_appmsg(MSG_LED_RING_UPDATE, (void*)&led, sizeof(led));
}

int gamutui_start_boot_animation(void)
{
	struct msg_animation animation;
	memset(&animation, 0, sizeof(animation));

	animation.id = ANIMATION_BOOT;
	animation.state = 1;
	return send_appmsg(MSG_ANIMATION, (void*)&animation, sizeof(animation));
}

int gamutui_start_critical_battery_animation(void)
{
	struct msg_animation animation;
	memset(&animation, 0, sizeof(animation));

	animation.id = ANIMATION_CRITICAL_BATTERY;
	animation.state = 1;
	return send_appmsg(MSG_ANIMATION, (void*)&animation, sizeof(animation));
}

int gamutui_stop_animation(void) {
	struct msg_animation animation;
	memset(&animation, 0, sizeof(animation));

	animation.state = 0;
	return send_appmsg(MSG_ANIMATION, (void*)&animation, sizeof(animation));
}

int gamutui_reset(void) {
	printf("Resetting the atmel...\n");
	if (gpio_direction_output(ATMEL_RESET_GPIO, 1) != 0) {
		return -1;
	}

	udelay(1000);

	if (gpio_direction_output(ATMEL_RESET_GPIO, 0) != 0) {
		return -1;
	}

	return 0;
}

int gamutui_get_status(struct msg_status_reply* status) {
	if (send_appmsg(MSG_STATUS, NULL, 0) != 0) {
		return -1;
	}

	if (get_appmsg(MSG_STATUS, (uint8_t*) status, 1000) <= 0) {
		return -1;
	}

	return 0;
}

void gamutui_wait_for_sense(void) {
	printf("Waiting for the UI board to be plugged in...\n");

	/* No delays if the first poll senses the atmel */
	if (gpio_get_value(ATMEL_SENSE_GPIO) == 0) {
		return;
	}

	/* Poll until the board is plugged in, then wait a lil to ensure the PCIE is fully inserted */
	while (gpio_get_value(ATMEL_SENSE_GPIO) != 0) {
		mdelay(10);
	}

	mdelay(1000);
}

int gamutui_init(void)
{
	int ret;

	prv.inited = false;
	printf("Initializing gamut ui...\n");

	/* get the GPIO for the Atmel interrupt line */
	ret = gpio_direction_input(ATMEL_INT_GPIO);
	if (ret) {
		printf("Error: couldn't setup Atmel Int GPIO\n");
		return -1;
	}

	ret = gpio_direction_input(ATMEL_SENSE_GPIO);
	if (ret) {
		printf("Error: couldn't setup Atmel sense GPIO\n");
		return -1;
	}

	prv.spi = spi_setup_slave(3, 0, 400000, SPI_MODE_0);
	ret = spi_claim_bus(prv.spi);
	if (ret) {
		printf("Error claming spi bus for gamut ui\n");
		return -1;
	}

	prv.inited = true;
	return 0;
}

static int do_ui(cmd_tbl_t *cmdtp, int flag, int argc,
		char * const argv[])
{
	if (!strcmp(argv[1], "led")) {
		int ret;
		struct msg_led_update led;
		memset(&led, 0, sizeof(led));

		for (int i=0; i<LED_RING_PIXEL_COUNT; i++) {
			switch (argv[2][0]) {
			case 'r': led.pixels[i].red = 255; break;
			case 'g': led.pixels[i].green = 255; break;
			case 'b': led.pixels[i].blue = 255; break;
			}
		}

		ret = send_appmsg(MSG_LED_RING_UPDATE, (void*)&led, sizeof(led));
		printf("Did an spi xfer, ret = %d\n", ret);
	} else if (!strcmp(argv[1], "btn")) {
		uint8_t buf[128];
		int ret;

		struct msg_button btn = { 0 };
		ret = send_appmsg(MSG_BUTTON, (void*)&btn, sizeof(btn));
		if (ret) {
			printf("Error sending request packet\n");
			return 0;
		}
		ret = get_appmsg(MSG_BUTTON, buf, 50);
		printf("ret is %d\n", ret);
		for (int i=0; i<ret; i++) {
			printf("0x%02X : 0x%02X\n", i, buf[i]);
		}
	} else if (!strcmp(argv[1], "time")) {
		printf ("Time: %lu\n", get_ms());
	} else if(!strcmp(argv[1], "boot")) {
		printf("bootmode = %d\n", gamutui_check_recovery());
	} else if (!strcmp(argv[1], "reset")) {
		return gamutui_reset();
	} else if (!strcmp(argv[1], "status")) {
		struct msg_status_reply status = {0};
		if (gamutui_get_status(&status)) {
			printf("Could not get status.\n");
			return 0;
		}

		printf("Status: power_3p3v_value: %d\n", status.power_3p3v_value);
		printf("Status: power_vsys_value: %d\n", status.power_vsys_value);
		printf("Status: battery_charging: %d\n", status.battery_charging);
		printf("Status: battery_power_ok: %d\n", status.battery_power_ok);
	} else if (!strcmp(argv[1], "i2c")) {
		struct msg_i2c msg = { 0 };
		switch (argv[2][0]) {
			case 'r':
				if(argc < 5) {
					printf("Usage: ui i2c r <device_address> <reg_address>\n");
					return 0;
				}
				msg.op = I2C_OP_READ;
				msg.device_address = simple_strtol(argv[3], NULL, 16);
				msg.reg_address = simple_strtol(argv[4], NULL, 16);
				if (send_appmsg(MSG_I2C, (void*)&msg, sizeof(msg))) {
					printf("Error sending request packet\n");
					return 0;
				}
				if (!get_appmsg(MSG_I2C, (uint8_t*)&msg, 50)) {
					printf("Error receiving response packet\n");
					return 0;
				}
				if (!msg.success) {
					printf("Error reading from i2c");
					return 0;
				}
				printf("Reg 0x%02X at device 0x%02X is: 0x%02X\n", msg.reg_address, msg.device_address, msg.value);
				break;
			case 'w':
				if(argc < 6) {
					printf("Usage: ui i2c w <device_address> <reg_address> <value>\n");
					return 0;
				}
				msg.op = I2C_OP_WRITE;
				msg.device_address = simple_strtol(argv[3], NULL, 16);
				msg.reg_address = simple_strtol(argv[4], NULL, 16);
				msg.value = simple_strtol(argv[5], NULL, 16);
				if (send_appmsg(MSG_I2C, (void*)&msg, sizeof(msg))) {
					printf("Error sending request packet\n");
					return 0;
				}
				if (!get_appmsg(MSG_I2C, (uint8_t*)&msg, 50)) {
					printf("Error receiving response packet\n");
					return 0;
				}
				if (!msg.success) {
					printf("Error writing to i2c");
					return 0;
				}
				printf("Wrote 0x%02X to reg 0x%02X at device 0x%02X\n", msg.value, msg.reg_address, msg.device_address);
				break;
		}
	} else if (!strcmp(argv[1], "trace_report")) {
		struct msg_trace_report msg = { 0 };
		if (argc < 3) {
			msg.id = -1;
		} else {
			msg.id = simple_strtol(argv[2], NULL, 10);
		}

		if (send_appmsg(MSG_TRACE_REPORT, (void*)&msg, sizeof(msg))) {
			printf("Error sending request packet\n");
			return 0;
		}

		while (get_appmsg(MSG_TRACE_REPORT, (uint8_t*)&msg, 200)) {
			if (strlen(msg.msg) == 0) {
				break;
			}

			printf("%llu\t%d\t%s\n", msg.usec, msg.id, msg.msg);
		}

		return 0;

	} else {
		printf("Command not known...\n");
	}


	return 0;
}

U_BOOT_CMD(ui, 6, 1, do_ui,"Interact with Gamut UI board",
	"This is mainly for devlopment, you should stay away...");
