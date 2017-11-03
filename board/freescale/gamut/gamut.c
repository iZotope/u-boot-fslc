/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2015 iZotope, Inc.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-ddr.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/spi.h>
#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <i2c.h>
#include <mmc.h>
#include <netdev.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>
#include "../common/pfuze.h"
#include <usb.h>
#include <usb/ehci-fsl.h>
#include <mmc.h>
#include "gamutui.h"
#include "cmd_bootgamut.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_22K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |             \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED   |             \
	PAD_CTL_DSE_40ohm   | PAD_CTL_HYS)

#define SPI_PAD_CTRL (PAD_CTL_HYS | PAD_CTL_SPEED_MED | \
		      PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
		      PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |	\
		      PAD_CTL_DSE_40ohm | PAD_CTL_HYS |		\
		      PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define OTGID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
			PAD_CTL_PUS_47K_UP | PAD_CTL_SPEED_LOW |\
			PAD_CTL_DSE_80ohm | PAD_CTL_HYS |	\
			PAD_CTL_SRE_FAST)

#define ETH_PHY_RESET	IMX_GPIO_NR(4, 21)

#define SNVS_LPCR (SNVS_BASE_ADDR + 0x38)
#define SNVS_LPSR (SNVS_BASE_ADDR + 0x4c)
#define SNVS_LPPGDR (SNVS_BASE_ADDR + 0x64)

#define SNVS_GLITCH_MAGIC 0x41736166
#define SNVS_GLITCH_EN 0x8
#define SNVS_SRTC_EN 0x1

#define DEFAULT_ETH_MAC_ADDR "00:01:02:11:11:11"

int dram_init(void)
{
	gd->ram_size = get_ram_size((void *)PHYS_SDRAM, PHYS_SDRAM_SIZE);

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TXD__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RXD__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc1_pads[] = {
	/* 4 bit SD for uSD slot */
	MX6_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT0__USDHC1_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT1__USDHC1_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT2__USDHC1_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD1_DAT3__USDHC1_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),

	/*CD pin*/
	MX6_PAD_EPDC_SDCLK__GPIO_1_23 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	/* 8 bit SD for eMMC */
	/* TODO: we should setup the RST pin and have it always pulled high(inactive) */
	MX6_PAD_SD2_RESET__USDHC2_RESET | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	/* MX6_PAD_SD2_RESET__GPIO_4_27 | MUX_PAD_CTRL(NO_PAD_CTRL), */
	MX6_PAD_SD2_CLK__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__USDHC2_DAT0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__USDHC2_DAT1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__USDHC2_DAT2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__USDHC2_DAT3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT4__USDHC2_DAT4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT5__USDHC2_DAT5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT6__USDHC2_DAT6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT7__USDHC2_DAT7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};

static iomux_v3_cfg_t const fec_pads[] = {
	MX6_PAD_FEC_MDC__FEC_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_FEC_MDIO__FEC_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_FEC_CRS_DV__FEC_RX_DV | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_FEC_RXD0__FEC_RX_DATA0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_FEC_RXD1__FEC_RX_DATA1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_FEC_TX_EN__FEC_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_FEC_TXD0__FEC_TX_DATA0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_FEC_TXD1__FEC_TX_DATA1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_FEC_REF_CLK__FEC_REF_OUT | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_FEC_RX_ER__GPIO_4_19 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_FEC_TX_CLK__GPIO_4_21 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static iomux_v3_cfg_t const atmel_pads[] = {
	MX6_PAD_LCD1_VSYNC__GPIO2_IO_18 | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_LCD1_RESET__GPIO2_IO_19 | MUX_PAD_CTRL(NO_PAD_CTRL)
};

#ifdef CONFIG_MXC_SPI
static iomux_v3_cfg_t ecspi1_pads[] = {
	MX6_PAD_ECSPI1_MISO__ECSPI_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_ECSPI1_MOSI__ECSPI_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_ECSPI1_SCLK__ECSPI_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_ECSPI1_SS0__GPIO4_IO11  | MUX_PAD_CTRL(NO_PAD_CTRL),
	/* for Gamut front panel UI board */
	MX6_PAD_EPDC_D0__ECSPI4_MOSI | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EPDC_D1__ECSPI4_MISO | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EPDC_D3__ECSPI4_SCLK | MUX_PAD_CTRL(SPI_PAD_CTRL),
	MX6_PAD_EPDC_D2__GPIO_1_9    | MUX_PAD_CTRL(NO_PAD_CTRL),
};

int board_spi_cs_gpio(unsigned bus, unsigned cs)
{
	if (bus == 0 && cs == 0) {
		return IMX_GPIO_NR(4, 11);
	} else if (bus == 3 && cs == 0) {
		return IMX_GPIO_NR(1, 9);
	}
	
	return -1;
}

static void setup_spi(void)
{
	imx_iomux_v3_setup_multiple_pads(ecspi1_pads, ARRAY_SIZE(ecspi1_pads));
}
#endif

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

static void setup_iomux_fec(void)
{
	imx_iomux_v3_setup_multiple_pads(fec_pads, ARRAY_SIZE(fec_pads));

	/* Reset LAN8720 PHY */
	gpio_direction_output(ETH_PHY_RESET , 0);
	udelay(1000);
	gpio_set_value(ETH_PHY_RESET, 1);
}

static void setup_atmel(void)
{
	imx_iomux_v3_setup_multiple_pads(atmel_pads, ARRAY_SIZE(atmel_pads));
}

static void setup_poweroff(void)
{
	/* This magical sequence of register writes comes from rtc-snvs.c,
	* and are required to stabilize the PMIC_ON_REQ signal and enable
	* CPU poweroff

	Details on this registers can be found in the IMX6SL Security Reference Manual.
	*/

	/* Enable power glitch detection (must do this before all SNVS writes) */
	writel(SNVS_GLITCH_MAGIC, SNVS_LPPGDR);

	/* Set the power-glitch detected bit
	*  (honestly not sure why, but it is definitely necessary) */
	u32 snvs_lpsr = readl(SNVS_LPSR);
	writel(snvs_lpsr | SNVS_GLITCH_EN, SNVS_LPSR);

	/* Enable the low-power secure real-time counter */

	/* This counter is used in a wake-up mechanism that can automatically
	*  turn on high-power components (V3_3), so if we don't start it,
	*  the SNVS will immediately power on the CPU immediately after we shut
	*  it down.
	*/
	u32 snvs_lpcr = readl(SNVS_LPCR);
	writel(snvs_lpcr | SNVS_SRTC_EN, SNVS_LPCR);

	/* It takes several clock cycles to finish writing to this register, so
	*  hold tight until the value is written. */
	u32 iter = 0;
	while (iter < 1000 && !(readl(SNVS_LPCR) & SNVS_SRTC_EN)) {
		iter++;
	}

	if (!(readl(SNVS_LPCR) & SNVS_SRTC_EN)) {
		printf("Could not set up CPU poweroff. Expect brownouts if battery is too low.");
	}
}

#define USDHC1_CD_GPIO	IMX_GPIO_NR(1, 23)

static struct fsl_esdhc_cfg usdhc_cfg[2] = {
	{USDHC1_BASE_ADDR, 0, 4},
	{USDHC2_BASE_ADDR, 0, 8},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
#warning Workaround for broken card detect, fix this when hardware is fixed
		//ret = !gpio_get_value(USDHC1_CD_GPIO);
		return 1;
		break;
	case USDHC2_BASE_ADDR:
		/* MMC2 is EMMC so always present */
		ret = 1;
		break;
	}

	return ret;
}

int board_mmc_init(bd_t *bis)
{
#ifndef CONFIG_SPL_BUILD
	int i, ret;

	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-boot device node)    (Physical Port)
	 * mmc0                    USDHC1 (microsd card slot)
	 * mmc1                    USDHC2 (eMMC)
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(
				usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
			gpio_direction_input(USDHC1_CD_GPIO);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(
				usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
			usdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret) {
			printf("Warning: failed to initialize "
				"mmc dev %d\n", i);
			return ret;
		}
	}

	return 0;
#else
	struct src *src_regs = (struct src *)SRC_BASE_ADDR;
	u32 val;
	u32 port;

	val = readl(&src_regs->sbmr1);

	/* Boot from USDHC */
	port = (val >> 11) & 0x3;
	switch (port) {
	case 0:
		imx_iomux_v3_setup_multiple_pads(usdhc1_pads,
						 ARRAY_SIZE(usdhc1_pads));
		gpio_direction_input(USDHC1_CD_GPIO);
		usdhc_cfg[0].esdhc_base = USDHC1_BASE_ADDR;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
		break;
	case 1:
		imx_iomux_v3_setup_multiple_pads(usdhc2_pads,
						 ARRAY_SIZE(usdhc2_pads));
		gpio_direction_input(USDHC2_CD_GPIO);
		usdhc_cfg[0].esdhc_base = USDHC2_BASE_ADDR;
		usdhc_cfg[0].max_bus_width = 4;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
		break;
	case 2:
		imx_iomux_v3_setup_multiple_pads(usdhc3_pads,
						 ARRAY_SIZE(usdhc3_pads));
		gpio_direction_input(USDHC3_CD_GPIO);
		usdhc_cfg[0].esdhc_base = USDHC3_BASE_ADDR;
		usdhc_cfg[0].max_bus_width = 4;
		usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		break;
	}

	gd->arch.sdhc_clk = usdhc_cfg[0].sdhc_clk;
	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
#endif
}

#ifdef CONFIG_SYS_I2C_MXC
#define PC	MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C1 for PMIC */
struct i2c_pads_info i2c_pad_info1 = {
	.sda = {
		.i2c_mode = MX6_PAD_I2C1_SDA__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_I2C1_SDA__GPIO_3_13 | PC,
		.gp = IMX_GPIO_NR(3, 13),
	},
	.scl = {
		.i2c_mode = MX6_PAD_I2C1_SCL__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_I2C1_SCL__GPIO_3_12 | PC,
		.gp = IMX_GPIO_NR(3, 12),
	},
};

int power_init_board(void)
{
	struct pmic *p;
	int ret;
	unsigned int reg, rev_id;

	ret = power_pfuze3000_init(I2C_PMIC);
	if (ret)
		return ret;

	p = pmic_get("PFUZE3000");
	ret = pmic_probe(p);
	if (ret)
		return ret;

	pmic_reg_read(p, PFUZE3000_DEVICEID, &reg);
	pmic_reg_read(p, PFUZE3000_REVID, &rev_id);
	printf("PMIC:  PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n", reg, rev_id);

	/* disable Low Power Mode during standby mode */
	pmic_reg_read(p, PFUZE3000_LDOGCTL, &reg);
	reg |= 0x1;
	pmic_reg_write(p, PFUZE3000_LDOGCTL, reg);

	/* Configure SW2 to use 3.2v rather than the default 3.3v. */
	pmic_reg_write(p, PFUZE3000_SW2VOLT, 0x20 | 0x6);  // High voltage settings, 3.2v

	return 0;
}
#endif

#ifdef CONFIG_FEC_MXC
int board_eth_init(bd_t *bis)
{
	setup_iomux_fec();

	return cpu_eth_init(bis);
}

static int setup_fec(void)
{
	struct iomuxc *iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	/* clear gpr1[14], gpr1[18:17] to select anatop clock */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC_MASK, 0);

	return enable_fec_anatop_clock(0, ENET_50MHZ);
}
#endif

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

static iomux_v3_cfg_t const usb_otg_pads[] = {
	/* OTG1 */
	MX6_PAD_KEY_COL4__USB_USBOTG1_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_EPDC_PWRCOM__ANATOP_USBOTG1_ID | MUX_PAD_CTRL(OTGID_PAD_CTRL),
	/* OTG2 */
	MX6_PAD_KEY_COL5__USB_USBOTG2_PWR | MUX_PAD_CTRL(NO_PAD_CTRL)
};

static void setup_usb(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
					 ARRAY_SIZE(usb_otg_pads));
}

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return usb_phy_mode(port);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	/* Set Power polarity */
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}
#endif

int board_late_init(void)
{
	int part = mmc_get_boot_part(1);

	gamutui_init();

	printf("eMMC:  ");
	if (part >= 0) {
		switch (part) {
		case BOOT_PART_DISABLED:
			printf("boot not enabled");
			break;
		case BOOT_PART_1:
			printf("boot part 1 of 2 used");
			break;
		case BOOT_PART_2:
			printf("boot part 2 of 2 used");
			break;
		case BOOT_PART_USER:
			printf("User area used for boot");
			break;
		default:
			printf("unknow boot part '0x%X'", part);
		}
	} else {
		printf("not detected");
	}
	printf("\n");

	/* Set the MAC address of the ethernet port. Since ethernet is only used in 
	*  manufacturing, we use a static MAC taken from the old 3COM block, 00:01:02:XX:XX:XX. 
	*  Do this before setting any other env vars, since we don't want to save those other ones. */
	if (!getenv("ethaddr")) {
		printf("Setting the ethernet MAC address to %s\n", DEFAULT_ETH_MAC_ADDR);
		setenv("ethaddr", DEFAULT_ETH_MAC_ADDR);
		saveenv();
	}

	/* All magic numbers below can be explained in the IMX6SL reference
	 * manual. (Document number IMX6SLRM) See "Chapter 8: System Boot" and
	 * "Chapter 46: System Reset Controllor (SRC)".
	 */

	/* First read if we are doing a USB serial boot. This is for
	 * bootstrapping, recovery and manufacturing.
	 * USB boot trumps all.
	 */
	uint32_t bootmode = *(volatile uint32_t *)(0x20D801C);
	bootmode = (bootmode >> 24) & 0x3;
	if (bootmode == 1) {
		printf("USB bootstrap");
		setenv("boot_src", BOOTSRC_USB);
	} else {
		/* if we aren't USB, read if we are booting from eMMC or SD card
		 * Set boot_src to sd if we booted from SD.
		 * This can be manually overidden by the user if they want.
		 */
		uint32_t bootcfg = *(volatile uint32_t *)(0x20D8004);
		printf("boot cfg: 0x%08X\n", bootcfg);
		printf("boot src: ");
		if (bootcfg & (1 << 5)) {
			printf("eMMC");
		} else {
			printf("SD card");
			setenv("boot_src",BOOTSRC_SD);
		}
		printf("\n");
	}

	return 0;
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	setup_atmel();
	setup_poweroff();
#ifdef CONFIG_MXC_SPI
	setup_spi();
#endif
	return 0;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
#endif

#ifdef	CONFIG_FEC_MXC
	setup_fec();
#endif

#ifdef CONFIG_USB_EHCI_MX6
	#error If you want to turn on USB support in Gamut, you must fix the iomux configuation
	setup_usb();
#endif

	return 0;
}

int checkboard(void)
{
	puts("Board: Gamut\n");

	return 0;
}

#ifdef CONFIG_SPL_BUILD
#include <spl.h>
#include <libfdt.h>

const struct mx6sl_iomux_ddr_regs mx6_ddr_ioregs = {
	.dram_sdqs0 = 0x00003030,
	.dram_sdqs1 = 0x00003030,
	.dram_sdqs2 = 0x00003030,
	.dram_sdqs3 = 0x00003030,
	.dram_dqm0 = 0x00000030,
	.dram_dqm1 = 0x00000030,
	.dram_dqm2 = 0x00000030,
	.dram_dqm3 = 0x00000030,
	.dram_cas  = 0x00000030,
	.dram_ras  = 0x00000030,
	.dram_sdclk_0 = 0x00000028,
	.dram_reset = 0x00000030,
	.dram_sdba2 = 0x00000000,
	.dram_odt0 = 0x00000008,
	.dram_odt1 = 0x00000008,
};

const struct mx6sl_iomux_grp_regs mx6_grp_ioregs = {
	.grp_b0ds = 0x00000030,
	.grp_b1ds = 0x00000030,
	.grp_b2ds = 0x00000030,
	.grp_b3ds = 0x00000030,
	.grp_addds = 0x00000030,
	.grp_ctlds = 0x00000030,
	.grp_ddrmode_ctl = 0x00020000,
	.grp_ddrpke = 0x00000000,
	.grp_ddrmode = 0x00020000,
	.grp_ddr_type = 0x00080000,
};

const struct mx6_mmdc_calibration mx6_mmcd_calib = {
	.p0_mpdgctrl0 =  0x20000000,
	.p0_mpdgctrl1 =  0x00000000,
	.p0_mprddlctl =  0x4241444a,
	.p0_mpwrdlctl =  0x3030312b,
	.mpzqlp2ctl = 0x1b4700c7,
};

static struct mx6_lpddr2_cfg mem_ddr = {
	.mem_speed = 800,
	.density = 4,
	.width = 32,
	.banks = 8,
	.rowaddr = 14,
	.coladdr = 10,
	.trcd_lp = 2000,
	.trppb_lp = 2000,
	.trpab_lp = 2250,
	.trasmin = 4200,
};

static void ccgr_init(void)
{
	struct mxc_ccm_reg *ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;

	writel(0xFFFFFFFF, &ccm->CCGR0);
	writel(0xFFFFFFFF, &ccm->CCGR1);
	writel(0xFFFFFFFF, &ccm->CCGR2);
	writel(0xFFFFFFFF, &ccm->CCGR3);
	writel(0xFFFFFFFF, &ccm->CCGR4);
	writel(0xFFFFFFFF, &ccm->CCGR5);
	writel(0xFFFFFFFF, &ccm->CCGR6);

	writel(0x00260324, &ccm->cbcmr);
}

static void spl_dram_init(void)
{
	struct mx6_ddr_sysinfo sysinfo = {
		.dsize = mem_ddr.width / 32,
		.cs_density = 20,
		.ncs = 2,
		.cs1_mirror = 0,
		.walat = 0,
		.ralat = 2,
		.mif3_mode = 3,
		.bi_on = 1,
		.rtt_wr = 0,        /* LPDDR2 does not need rtt_wr rtt_nom */
		.rtt_nom = 0,
		.sde_to_rst = 0,    /* LPDDR2 does not need this field */
		.rst_to_cke = 0x10, /* JEDEC value for LPDDR2: 200us */
		.ddr_type = DDR_TYPE_LPDDR2,
	};
	mx6sl_dram_iocfg(32, &mx6_ddr_ioregs, &mx6_grp_ioregs);
	mx6_dram_cfg(&sysinfo, &mx6_mmcd_calib, &mem_ddr);
}

void board_init_f(ulong dummy)
{
	/* setup AIPS and disable watchdog */
	arch_cpu_init();

	ccgr_init();

	/* iomux and setup of i2c */
	board_early_init_f();

	/* setup GP timer */
	timer_init();

	/* UART clocks enabled and gd valid - init serial console */
	preloader_console_init();

	/* DDR initialization */
	spl_dram_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* load/boot image from boot device */
	board_init_r(NULL, 0);
}

void reset_cpu(ulong addr)
{
}
#endif
