/*
 * Copyright (C) 2016 iZotope, Inc.
 *
 * Author: Matthew Campbell <mcampbell@izotope.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <ext_common.h>
#include <command.h>
#include <errno.h>
#include <mmc.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include "gamutui.h"
#include "cmd_bootgamut.h"

#define APPMSG_PROTOCOL_NO_INCLUDES
#include "hardware-shared-open/gamut/appmsg_protocol.h"

#define ENV_BOOT_SRC "boot_src"
#define ENV_BOOT_SCRIPT "boot_script"
#define ENV_SKIP_UI_SENSE "skip_ui_sense"

#define BOOTSCRIPT_DEFAULT "/boot/boot.scr"
#define BOOTSCRIPT_DEFAULT_NET "boot.scr"

#define BOOTSRC_DEFAULT BOOTSRC_EMMC

#define GAMUT_SD_DEV 0
#define GAMUT_EMMC_DEV 1

#define GAMUT_RECOVERY_FOLDER "/recovery/"
#define GAMUT_RECOVERY_RAMDISK GAMUT_RECOVERY_FOLDER "gamut-recovery.cpio.gz.u-boot"
#define GAMUT_RECOVERY_KERNEL GAMUT_RECOVERY_FOLDER "zImage-recovery.bin"
#define GAMUT_RECOVERY_DTB GAMUT_RECOVERY_FOLDER "zImage-recovery.dtb"

#define GAMUT_BOOT_COUNT_FILE "/boot_count.txt"

#define GAMUT_RECOVERY_DEV GAMUT_EMMC_DEV
#define GAMUT_RECOVERY_PART 1
#define GAMUT_VOLATILE_DEV GAMUT_EMMC_DEV
#define GAMUT_VOLATILE_PART 2
#define GAMUT_ROOTFS1_PART 3
#define GAMUT_ROOTFS2_PART 4

#define ADDR_SCRATCH 0x84000000
#define ADDR_DTB     0x86000000
#define ADDR_RAMDISK 0x88000000

#define ARG_DEVPART (2)
#define ARG_ADDR (3)
#define ARG_FILE (4)
#define ARG_BYTES (5)

/* Regulated system voltage is 3.2V, so VSYS should be above that level,
*  plus margin accounting for the ~200mV voltage drop when the system is running.
*  This value was obtained by observing what boot-time Vsys results in a runtime
*  Vsys of ~3.25V. */
#define MIN_BOOTABLE_VSYS 3430

#define POWEROFF_OFFSET 0x38
#define POWEROFF_MASK 0x60

/* This is a general purpose register in the system reset controller that we use to
 * communicated betwenn the OS and u-boot on warm boots.
 *
 * A shared header would be too cumbersome to implment here so keep these two locations
 * up to date.
 */
#define SRC_GPR2_ADDRESS 0x20d8024
#define GPR2_RCV_REQ_SHIFT (0)
#define GPR2_RCV_REQ_MASK (0xFF << GPR2_RCV_REQ_SHIFT)
#define REQUEST_RECOVERY_VALUE 0xFF
#define GPR2_BOOT_CNT_SHIFT (8)
#define GPR2_BOOT_CNT_MASK (0xFF << GPR2_BOOT_CNT_SHIFT)


#define LED_COLOR_INITIAL 0x000000
#define LED_COLOR_UBOOT 0x0000FF
#define LED_COLOR_USB 0xFFFF00
#define LED_COLOR_RECOVERY 0x00FFFF

/* set 'rootfs_dev' and 'rootfs_part' based on the active eMMC boot parition
 */
static int select_boot_part(cmd_tbl_t *cmdtp, int flag)
{
	int part = mmc_get_boot_part(GAMUT_EMMC_DEV);

	if (part == BOOT_PART_1) {
		printf("Selecting rootfs 1 of 2 for boot!\n");
		setenv("rootfs_part", __stringify(GAMUT_ROOTFS1_PART));
	} else if (part == BOOT_PART_2) {
		printf("Selecting rootfs 2 of 2 for boot!\n");
		setenv("rootfs_part", __stringify(GAMUT_ROOTFS2_PART));
	} else {
		return 1;
	}

	setenv("rootfs_dev", __stringify(GAMUT_EMMC_DEV));
	return 0;
}

/* this loads the boot script into memory at ADDR_SCRATCH
 * Note that the evn vars 'rootfs_dev' and 'rootfs_part' must be set properly
 */
static int source_boot_script(cmd_tbl_t *cmdtp, int flag)
{
	char * ext4_args[] = {"ext4load", "mmc", NULL, NULL, NULL};
	char dev_part[] = "-:-";
	char addr[] = "0x00000000";
	char * boot_script = NULL;

	/* clear at least the size of u-boot image header so it's invalid
	 * incase we fail to load the script
	 */
	memset((void *)ADDR_SCRATCH, 0, image_get_header_size());

	boot_script = getenv(ENV_BOOT_SCRIPT);
	if (!boot_script) {
		boot_script = BOOTSCRIPT_DEFAULT;
	}

	snprintf(dev_part, sizeof(dev_part), "%s:%s", getenv("rootfs_dev"),
		 getenv("rootfs_part"));
	printf("load from 'mmc %s'\n", dev_part);
	ext4_args[ARG_DEVPART] = dev_part;
	snprintf(addr, sizeof(addr), "0x%X", ADDR_SCRATCH);
	ext4_args[ARG_ADDR] = addr;
	ext4_args[ARG_FILE] = boot_script;
	do_ext4_load(cmdtp, flag, ARRAY_SIZE(ext4_args), ext4_args);
	if (source(ADDR_SCRATCH, NULL)) {
		return 1;
	}

	return 0;
}

/* This will attempt to load a recovery image from mmc 1:1
 * it loads the kernel, dtb and ramdisk and launches
 */
static int recovery_boot(cmd_tbl_t *cmdtp, int flag)
{
	char * ext4_args[] = {"ext4load", "mmc", NULL, NULL, NULL};
	char dev_part[] = "-:-";
	char * bootz_args[] = {"bootz", NULL, __stringify(ADDR_RAMDISK), __stringify(ADDR_DTB)};
	char *kernel_addr = getenv("loadaddr");

	snprintf(dev_part, sizeof(dev_part), "%s:%s", __stringify(GAMUT_RECOVERY_DEV),
				__stringify(GAMUT_RECOVERY_PART));
	ext4_args[ARG_DEVPART] = dev_part;

	ext4_args[ARG_FILE] = GAMUT_RECOVERY_RAMDISK;
	ext4_args[ARG_ADDR] = __stringify(ADDR_RAMDISK);
	do_ext4_load(cmdtp, flag, ARRAY_SIZE(ext4_args), ext4_args);

	ext4_args[ARG_FILE] = GAMUT_RECOVERY_KERNEL;
	ext4_args[ARG_ADDR] = kernel_addr;
	do_ext4_load(cmdtp, flag, ARRAY_SIZE(ext4_args), ext4_args);

	ext4_args[ARG_FILE] = GAMUT_RECOVERY_DTB;
	ext4_args[ARG_ADDR] = __stringify(ADDR_DTB);
	do_ext4_load(cmdtp, flag, ARRAY_SIZE(ext4_args), ext4_args);

	bootz_args[1] = kernel_addr;
	do_bootz(cmdtp, flag, ARRAY_SIZE(bootz_args), bootz_args);

	return 0;
}

/* This will do a USB boot strap. It assumes that the USB boot loader on the PC
 * has placed the kernel, ramdisk and dtb in the proper location otherwise it
 * will fail.
 */
static int usb_boot(cmd_tbl_t *cmdtp, int flag)
{
	char * bootz_args[] = {"bootz", NULL, __stringify(ADDR_RAMDISK), __stringify(ADDR_DTB)};
	char *kernel_addr = getenv("loadaddr");

	bootz_args[1] = kernel_addr;
	do_bootz(cmdtp, flag, ARRAY_SIZE(bootz_args), bootz_args);
	return 0;
}

static int check_boot_count(cmd_tbl_t *cmdtp, int flag)
{
	u32 boot_count = 0;

	ulong max_boot_attempts = getenv_ulong("boot_attempts", 10, 3);
	if (!max_boot_attempts) {
		printf("'boot_attempts' env var either invalid, not set, or '0'. Skipping boot attempt counting\n");
		return 0;
	}

	boot_count = readl(SRC_GPR2_ADDRESS);
	boot_count = (boot_count & GPR2_BOOT_CNT_MASK) >> GPR2_BOOT_CNT_SHIFT;
	if (boot_count >= max_boot_attempts) {
		/* we are past our boot count, go right to recovery */
		printf("Too many boot attempts (%lu)\n", max_boot_attempts);
		gamutui_set_single_led(LED_COLOR_RECOVERY);
		setenv(ENV_BOOT_SRC, BOOTSRC_RECOVERY);
		return 1;
	}

	/* write back the boot counter */
	boot_count++;
	u32 val = readl(SRC_GPR2_ADDRESS);
	val &= ~GPR2_BOOT_CNT_MASK;
	val |= (boot_count << GPR2_BOOT_CNT_SHIFT) & GPR2_BOOT_CNT_MASK;
	writel(val, SRC_GPR2_ADDRESS);

	printf("Boot attempt #%d of %lu\n", boot_count, max_boot_attempts);

	return 0;
}

static void poweroff(void) {
	u32 val = readl(SNVS_BASE_ADDR + POWEROFF_OFFSET);
	val |= POWEROFF_MASK;
	writel(val, SNVS_BASE_ADDR + POWEROFF_OFFSET);
	mdelay(5000);
}

/* Check whether we have sufficient battery to boot
* Return: true if sufficient battery or couldn't retrieve voltage,
* 		  false if insufficient battery.
*/
static bool check_battery_level(void) {
	struct msg_status_reply system_status = {0};
	int ret = gamutui_get_status(&system_status);
	if (ret == 0) {
		printf("Vsys is %d mV\n", system_status.power_vsys_value);
		if (system_status.power_vsys_value < MIN_BOOTABLE_VSYS) {
			printf("Vsys is below the %d mV required. Cannot boot.\n",
				MIN_BOOTABLE_VSYS);

			return 0;
		}
	} else {
		printf("Could not retrieve power status from atmel. Returned %d. Beware of a brownout.\n", ret);
	}

	return 1;
}

static bool is_recovery_requested(void) {
	u32 recovery_flag = readl(SRC_GPR2_ADDRESS);
	recovery_flag = (recovery_flag & GPR2_RCV_REQ_MASK) >> GPR2_RCV_REQ_SHIFT;

	printf("Recovery flag is 0x%x\n", recovery_flag);
	if (recovery_flag == REQUEST_RECOVERY_VALUE) {
		return true;
	}

	return false;
}

static void clear_recovery_flag(void) {
	u32 val = readl(SRC_GPR2_ADDRESS);
	val &= ~GPR2_RCV_REQ_MASK;
	writel(val, SRC_GPR2_ADDRESS);
}

/* Do some checks to determine if we should change boot modes or halt boot altogether.
*  Return true if we should break into u-boot.*/
static bool do_startup_check(void) {	
	bool check_atmel = strcmp(getenv(ENV_BOOT_SRC), BOOTSRC_USB);
	if (!check_atmel) {
		/* set the LEDs to indicate a USB boot */
		gamutui_set_single_led(LED_COLOR_USB);
		return false;
	}

	/* only check for atmel status if not a USB boot */
	/* Wait for the UI board to be plugged in */
	if (getenv_yesno(ENV_SKIP_UI_SENSE) != 1) {
		gamutui_wait_for_sense();
	}

	/* Check whether we have enough battery to boot. */
	if (!check_battery_level()) {
		gamutui_start_critical_battery_animation();
		mdelay(3000);
		gamutui_stop_animation();
		poweroff();

		/* Shouldn't get here */
		return false;
	}

	/* Check if we should go into recovery based on magic recovery flag */
	if (is_recovery_requested()) {
		gamutui_set_single_led(LED_COLOR_RECOVERY);
		printf("Found recovery request flag. Doing recovery boot\n");
		setenv(ENV_BOOT_SRC, BOOTSRC_RECOVERY);
		return false;
	}

	/* Check if we should go into recovery mode based on UI board state */
	int ret = gamutui_check_recovery();
	if (ret == BOOT_BREAK_UBOOT) {
		printf("UI interrupt boot '%d', breaking into u-boot\n",
			ret);
		gamutui_set_single_led(LED_COLOR_UBOOT);
		return true;
	} else if (ret == BOOT_RECOVERY_REQUESTED ||
				ret == BOOT_RECOVERY_ERROR) {
		gamutui_set_single_led(LED_COLOR_RECOVERY);
		printf("UI interrupt boot '%d', doing recovery\n", ret);
		setenv(ENV_BOOT_SRC, BOOTSRC_RECOVERY);
	}

	return false;
}

static int do_gamutboot(cmd_tbl_t *cmdtp, int flag, int argc,
		char * const argv[])
{
	char * boot_script = NULL;

	if (!getenv(ENV_BOOT_SRC)) {
		printf("'" ENV_BOOT_SRC "' not set, assuming '"
		       BOOTSRC_DEFAULT "'.\n");
		setenv(ENV_BOOT_SRC, BOOTSRC_DEFAULT);
	}

	gamutui_set_leds(LED_COLOR_INITIAL);

	if (do_startup_check()) {
		return 0;
	}

	boot_script = getenv(ENV_BOOT_SCRIPT);
	if (!boot_script) {
		char * default_boot_script = BOOTSCRIPT_DEFAULT;
		if (!strcmp(getenv(ENV_BOOT_SRC), BOOTSRC_NET))
			default_boot_script = BOOTSCRIPT_DEFAULT_NET;

		printf("'" ENV_BOOT_SCRIPT "' not set, assuming '%s'.\n",
				default_boot_script);
		boot_script = default_boot_script;
	}

	/* Check/increment the boot attempt counter. Do recovery if needed */
	if (check_boot_count(cmdtp, flag)) {
		printf("Problems found, forcing recovery boot\n");
	}

	char * boot_src = getenv(ENV_BOOT_SRC);
	/* do the boot process based on boot_src */
	if (!strcmp(boot_src, BOOTSRC_NET)) {
		printf("Doing a network boot...\n");
		char *tftpb_args[] = {"tftpboot", __stringify(ADDR_SCRATCH), boot_script};

		if (select_boot_part(cmdtp, flag)) {
			printf("Warning: couldn't infer eMMC rootfs, not setting rootfs_dev or rootfs_part.");
		}

		if (do_tftpb(cmdtp, flag, 3, tftpb_args)) {
			return -ENOENT;
		}

		source(ADDR_SCRATCH, NULL);
	} else if (!strcmp(boot_src, BOOTSRC_SD)) {
		printf("Doing an SD card boot...\n");
		setenv("rootfs_dev", __stringify(GAMUT_SD_DEV));
		setenv("rootfs_part", "1"); /* always use first partition */

		source_boot_script(cmdtp, flag);
	} else if (!strcmp(boot_src, BOOTSRC_ARB)) {
		printf("Doing arb boot...\n");
		if(!getenv("rootfs_dev") || !getenv("rootfs_part")) {
			printf("Error: both rootfs_dev and rootfs_part env varns need to be set\n");
			return 1;
		}
		source_boot_script(cmdtp, flag);
	} else if (!strcmp(boot_src, BOOTSRC_EMMC)) {
		printf("Doing a standard eMMC boot...\n");

		gamutui_start_boot_animation();

		if (select_boot_part(cmdtp, flag)) {
			// TODO: error handling? or let attempted boot count handle?
			printf("Need error handling on selecting boot partition\n");
		} else {
			source_boot_script(cmdtp, flag);
		}

		printf("Error: we shouldn't get here..., likely should force a reboot\n");
		/* do a reset to try again, if it keeps failing the boot attempt
		 * count will force us into a recovery boot
		 */
		do_reset(cmdtp, flag, 0, NULL);
		return 1;
	} else if (!strcmp(boot_src, BOOTSRC_RECOVERY)) {
		printf("Doing recovery boot...\n");
		clear_recovery_flag();
		recovery_boot(cmdtp, flag);
	} else if (!strcmp(boot_src, BOOTSRC_USB)) {
		printf("Doing a USB bootstrap...\n");
		usb_boot(cmdtp, flag);
	} else {
		printf("Error: unsupport boot_src\n");
	}

	/* Shouldn't get here, always set LEDs to red */
	gamutui_set_single_led(0xFF0000);
	printf("Error: Boot failed!\n");
	return 1;
}
U_BOOT_CMD(
	bootg, 1, 0, do_gamutboot,
	"Boot gamut with failsafe modes",
	"There are some environment vars effect this command.\n"
	"If they are not set, there is an assumed default.\n"
	"  boot_src: sets the boot source for gamut\n"
	"    emmc - (default) boot from internal eMMC\n"
	"    sd - boot from first partition of the sdcard\n"
	"    net - do a tftp boot\n"
	"    arb - arbitrary boot source, see rootfs_dev and roofs_part below.\n"
	"    recovery - boot a ramdisk based recovery from mmc " __stringify(GAMUT_RECOVERY_DEV) ":" __stringify(GAMUT_RECOVERY_PART) "\n"
	"    usb - do a usb based boot. Assumes you loaded rootfs, zImage, and DTBto memory."
	"  boot_script: filename of bootscript to load. Note: bootg will\n"
	"               automatically set 'rootfs_dev' and 'rootfs_part'\n"
	"               envrionment vars before sourcing the boot_script.\n"
	"    /boot/boot.scr - (default)\n"
	"  skip_recovery: skips and recovery mode checks\n"
	"    If this is set (1,y,Y), skip all recovery mode checks.\n"
	"    If not set (0,n,N) or not present the recovery checks are done\n"
	"  skip_ui_sense: skips waiting for the ui board sense to go heigh before booting\n"
	"    If this is set (1,y,Y), skip sensing UI board before boot.\n"
	"    If not set (0,n,N) or not present the unit will wait to boot until the UI board is sensed\n"
	"  boot_attempts: number of attempted boots before doing an automatic recovery boot.\n"
	"    if set the system will try to boot this number of times before falling\n"
	"    back on a recovery boot. If unset, or set to '0' boot attempt counting is\n"
	"    disabled. The boot count is kept in '" GAMUT_BOOT_COUNT_FILE "' on mmc " __stringify(GAMUT_RECOVERY_DEV) ":" __stringify(GAMUT_RECOVERY_PART) "\n"
	"    It is stored as a uint32_t in native endian. The Linux application is\n"
	"    responsible for clearing this on a successful boot."
	"\n"
	"Bootg also provides some env vars that the boot script can use.\n"
	"  rootfs_dev: the mmc dev number to look for rootfs on.\n"
	"    This should be the eMMC device (" __stringify(GAMUT_EMMC_DEV) ").\n"
	"    For 'arb' boot_src, this must be set by the user before running.\n"
	"  rootfs_part: the partition number to boot from.\n"
	"    This should be one of the two ping-ponged roofts parts on eMMC.\n"
	"    For 'arb' boot_src, this must be set by the user before running.\n"
	"  Use these two env vars to contruct a Linux root device as follows:\n"
	"    /dev/mmcblk${rootfs_dev}p${rootfs_part}\n"
);
