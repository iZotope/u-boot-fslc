/*
 * Copyright (C) 2016 iZotope, Inc.
 *
 * Author: Matthew Campbell <mcampbell@izotope.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _GAMUTUI_H
#define _GAMUTUI_H

enum recovery_type {
	BOOT_NORMAL,
	BOOT_RECOVERY_REQUESTED,
	BOOT_RECOVERY_ERROR,
	BOOT_BREAK_UBOOT,
};

struct msg_status_reply;

int gamutui_init(void);
int gamutui_check_recovery(void);
int gamutui_set_leds(uint32_t color);
int gamutui_set_single_led(uint32_t color);
int gamutui_start_boot_animation(void);
int gamutui_start_critical_battery_animation(void);
int gamutui_stop_animation(void);
int gamutui_reset(void);
int gamutui_get_status(struct msg_status_reply* status);
void gamutui_wait_for_sense(void);

#endif /* _GAMUTUI_H */