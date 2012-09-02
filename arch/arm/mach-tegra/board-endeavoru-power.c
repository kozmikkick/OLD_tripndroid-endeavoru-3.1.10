/*
 * arch/arm/mach-tegra/board-endeavoru-power.c
 *
 * Copyright (C) 2012 NVIDIA, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */
#include <linux/i2c.h>
#include <linux/pda_power.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/tps80031.h>
#include <linux/regulator/tps80031-regulator.h>
#include <linux/tps80031-charger.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/cpumask.h>
#include <linux/platform_data/tegra_bpc_mgmt.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/gpio-regulator.h>

#include <linux/tps80032_adc.h>
#include <linux/tps80032_vsys_alarm.h>

#include <asm/mach-types.h>

#include <mach/edp.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>

#include "gpio-names.h"
#include "board.h"
#include "board-endeavoru.h"
#include "pm.h"
#include "wakeups-t3.h"
#include "tegra3_tsensor.h"

#define PMC_CTRL		0x0
#define PMC_CTRL_INTR_LOW	(1 << 17)

#define PMC_DPD_PADS_ORIDE		0x01c
#define PMC_DPD_PADS_ORIDE_BLINK	(1 << 20)

/************************ TPS80031 based regulator ****************/
static struct regulator_consumer_supply tps80031_vio_supply_a03[] = {
	REGULATOR_SUPPLY("vio_1v8", NULL),
	REGULATOR_SUPPLY("vddio_gmi", NULL),
	REGULATOR_SUPPLY("vddio_lcd", NULL),
	REGULATOR_SUPPLY("pwrdet_lcd", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc1", NULL),
	REGULATOR_SUPPLY("pwrdet_sdmmc1", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc4", NULL),
	REGULATOR_SUPPLY("pwrdet_sdmmc4", NULL),
	REGULATOR_SUPPLY("vddio_sys", NULL),
	REGULATOR_SUPPLY("vddio_cam", NULL),
	REGULATOR_SUPPLY("pwrdet_cam", NULL),
	REGULATOR_SUPPLY("vddio_bb", NULL),
	REGULATOR_SUPPLY("pwrdet_bb", NULL),
	REGULATOR_SUPPLY("vddio_uart", NULL),
	REGULATOR_SUPPLY("pwrdet_uart", NULL),
	REGULATOR_SUPPLY("vddio_audio", NULL),
	REGULATOR_SUPPLY("pwrdet_audio", NULL),
	REGULATOR_SUPPLY("avdd_osc", NULL),
	REGULATOR_SUPPLY("avdd_usb_pll", NULL),
	REGULATOR_SUPPLY("avdd_hdmi_pll", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc3", NULL),
};

static struct regulator_consumer_supply tps80031_smps1_supply_common[] = {
	REGULATOR_SUPPLY("vdd_cpu", NULL),
};

static struct regulator_consumer_supply tps80031_smps2_supply_common[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_consumer_supply tps80031_smps3_supply_common[] = {
	REGULATOR_SUPPLY("v_lpddr2_1v2", NULL),
	REGULATOR_SUPPLY("vddio_ddr", NULL),
	REGULATOR_SUPPLY("ddr_comp_pu", NULL),
};

static struct regulator_consumer_supply tps80031_smps4_supply_a03[] = {
	REGULATOR_SUPPLY("v_prereg_2v6", NULL),
};

static struct regulator_consumer_supply tps80031_vana_supply_common[] = {
	REGULATOR_SUPPLY("unused_vana", NULL),
};

static struct regulator_consumer_supply tps80031_ldo1_supply_a03[] = {
	REGULATOR_SUPPLY("v_dsi_csi_1v2", NULL),
	REGULATOR_SUPPLY("avdd_dsi_csi", NULL),
	REGULATOR_SUPPLY("vddio_hsic", NULL),
};

static struct regulator_consumer_supply tps80031_ldo2_supply_common[] = {
	REGULATOR_SUPPLY("v_vrtc_1v2", NULL),
	REGULATOR_SUPPLY("vdd_rtc", NULL),
};

static struct regulator_consumer_supply tps80031_ldo3_supply_common[] = {
	REGULATOR_SUPPLY("v_fuse_src_3v3", NULL),
	REGULATOR_SUPPLY("vpp_fuse", NULL),
};

static struct regulator_consumer_supply tps80031_ldo4_supply_a03[] = {
	REGULATOR_SUPPLY("v_lcm_3v", NULL),
};

static struct regulator_consumer_supply tps80031_ldo5_supply_common[] = {
	REGULATOR_SUPPLY("v_sr_2v85", NULL),
};

static struct regulator_consumer_supply tps80031_ldo6_supply_a03[] = {
	REGULATOR_SUPPLY("v_mmc_rx_2v85", NULL),
	REGULATOR_SUPPLY("vdd_ddr_rx", NULL),
};

static struct regulator_consumer_supply tps80031_ldo7_supply_a03[] = {
	REGULATOR_SUPPLY("v_pllmeux_1v2", NULL),
	REGULATOR_SUPPLY("vdd_plla_p_c", NULL),
	REGULATOR_SUPPLY("vdd_pllm", NULL),
	REGULATOR_SUPPLY("vdd_pllu_d", NULL),
	REGULATOR_SUPPLY("vdd_pllx", NULL),
};

static struct regulator_consumer_supply tps80031_ldoln_supply_a03[] = {
	REGULATOR_SUPPLY("v_ddr_hs_1v0", NULL),
	REGULATOR_SUPPLY("vdd_ddr_hs", NULL),
};

static struct regulator_consumer_supply tps80031_ldousb_supply_a03[] = {
	REGULATOR_SUPPLY("v_usb_3v3", NULL),
	REGULATOR_SUPPLY("avdd_usb", NULL),
	REGULATOR_SUPPLY("avdd_hdmi", NULL),
/*	REGULATOR_SUPPLY("vdd_nct1008", "4-004c"), */
};

static struct regulator_consumer_supply tps80031_vbus_supply_common[] = {
	REGULATOR_SUPPLY("usb_vbus", NULL),
};

#define TPS_PDATA_INIT(_id, _sname, _minmv, _maxmv, _supply_reg, _always_on,		\
	_boot_on, _apply_uv, _init_uV, _init_enable, _init_apply,			\
	_flags, _ectrl, _delay)								\
	static struct tps80031_regulator_platform_data pdata_##_id##_##_sname = {	\
		.regulator = {								\
			.constraints = {						\
				.min_uV = (_minmv)*1000,				\
				.max_uV = (_maxmv)*1000,				\
				.valid_modes_mask = (REGULATOR_MODE_NORMAL |		\
						REGULATOR_MODE_STANDBY),		\
				.valid_ops_mask = (REGULATOR_CHANGE_MODE |		\
						REGULATOR_CHANGE_STATUS |		\
						REGULATOR_CHANGE_VOLTAGE),		\
				.always_on = _always_on,				\
				.boot_on = _boot_on,					\
				.apply_uV = _apply_uv,					\
			},								\
			.num_consumer_supplies =					\
				ARRAY_SIZE(tps80031_##_id##_supply_##_sname),		\
			.consumer_supplies = tps80031_##_id##_supply_##_sname,		\
			.supply_regulator = _supply_reg,				\
		},									\
		.init_uV =  _init_uV * 1000,						\
		.init_enable = _init_enable,						\
		.init_apply = _init_apply,						\
		.flags = _flags,							\
		.ext_ctrl_flag = _ectrl,						\
		.delay_us = _delay,							\
	}

TPS_PDATA_INIT(vio, a03,   600, 2100, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(smps1, common, 600, 2600, 0, 0, 0, 0, -1, 0, 0, 0, PWR_REQ_INPUT_PREQ1, 0);
TPS_PDATA_INIT(smps2, common, 600, 2600, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(smps3, common, 600, 2100, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(smps4, a03, 600, 2100, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo1, a03, 1000, 3300, tps80031_rails(VIO), 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo2, common, 1000, 3300, 0, 1, 1, 1, 1000, 1, 1, 0, 0, 0);
TPS_PDATA_INIT(ldo3, common, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo4, a03, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo5, common, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo6, a03, 1000, 3300, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldo7, a03, 1000, 3300, tps80031_rails(VIO), 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(ldoln, a03, 1000, 3300, tps80031_rails(SMPS3), 0, 0, 0, -1, 0, 0, 0, PWR_REQ_INPUT_PREQ1, 0);
TPS_PDATA_INIT(ldousb, a03, 1000, 3300, 0, 0, 0, 0, -1, 0, 0, USBLDO_INPUT_VSYS, PWR_OFF_ON_SLEEP,0);
TPS_PDATA_INIT(vana, common,  1000, 3300, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0);
TPS_PDATA_INIT(vbus, common,  0, 5000, 0, 0, 0, 0, -1, 0, 0, (VBUS_SW_ONLY | VBUS_DISCHRG_EN_PDN), 0, 100000);

static struct tps80031_rtc_platform_data rtc_data = {
	.irq = ENT_TPS80031_IRQ_BASE + TPS80031_INT_RTC_ALARM,
	.time = {
		.tm_year = 2011,
		.tm_mon = 0,
		.tm_mday = 1,
		.tm_hour = 1,
		.tm_min = 2,
		.tm_sec = 3,
	},
};

#define TPS_RTC()				\
	{						\
		.id	= 0,		\
		.name	= "rtc_tps80031",	\
		.platform_data = &rtc_data,	\
	}

static struct tps80032_adc_platform_data pdata_adc = {
	.adc_vref = 1250,
	.gpadc_rt_irq = ENT_TPS80031_IRQ_BASE + TPS80031_INT_GPADC_RT,
	.gpadc_sw_irq = ENT_TPS80031_IRQ_BASE + TPS80031_INT_GPADC_SW2_EOC,
	.calib_bit_map = 0x7F,
};

struct tps80032_vsys_alarm_platform_data pdata_vsys_alarm = {
	.vsys_low_irq = ENT_TPS80031_IRQ_BASE + TPS80031_INT_SYS_VLOW,
};

#define TPS_DEV(_data)					\
	{						\
		.id	 = -1,				\
		.name   = "tps80032-"#_data,		\
		.platform_data  = &pdata_##_data,	\
	}

#define TPS_REG(_id, _data, _sname)				\
	{							\
		.id	 = TPS80031_ID_##_id,			\
		.name   = "tps80031-regulator",			\
		.platform_data  = &pdata_##_data##_##_sname,	\
	}
#define TPS_GPADC()					\
	{						\
		.name	= "tps80031-gpadc",		\
	}

#define TPS80031_DEVS_COMMON		\
	TPS_REG(SMPS1, smps1, common),	\
	TPS_REG(SMPS2, smps2, common),	\
	TPS_REG(SMPS3, smps3, common),	\
	TPS_REG(VANA, vana, common),	\
	TPS_REG(LDO2, ldo2, common),	\
	TPS_REG(LDO3, ldo3, common),	\
	TPS_REG(LDO5, ldo5, common),	\
	TPS_REG(VBUS, vbus, common),	\
	TPS_RTC(),			\
	TPS_DEV(adc),			\
	TPS_DEV(vsys_alarm),		\
	TPS_GPADC()

static struct tps80031_subdev_info tps80031_devs_a03[] = {
	TPS_REG(VIO, vio, a03),
	TPS80031_DEVS_COMMON,
	TPS_REG(SMPS4, smps4, a03),
	TPS_REG(LDO1, ldo1, a03),
	TPS_REG(LDO4, ldo4, a03),
	TPS_REG(LDO6, ldo6, a03),
	TPS_REG(LDO7, ldo7, a03),
	TPS_REG(LDOLN, ldoln, a03),
	TPS_REG(LDOUSB, ldousb, a03),

};

struct tps80031_clk32k_init_data clk32k_idata[] = {
	{
		.clk32k_nr = TPS80031_CLOCK32K_G,
		.enable = true,
		.ext_ctrl_flag = 0,
	},
	{
		.clk32k_nr = TPS80031_CLOCK32K_AUDIO,
		.enable = true,
		.ext_ctrl_flag = PWR_REQ_INPUT_PREQ1,
	},
};

static struct tps80031_pupd_init_data pupd_idata[] = {
	{
		.input_pin = TPS80031_PREQ1,
		.setting = TPS80031_PUPD_PULLUP,
	},
};

static struct tps80031_platform_data tps_platform = {
	.irq_base	= ENT_TPS80031_IRQ_BASE,
	.gpio_base	= ENT_TPS80031_GPIO_BASE,
	.clk32k_init_data	= clk32k_idata,
	.clk32k_init_data_size	= ARRAY_SIZE(clk32k_idata),
	.use_power_off	= true,
};

static struct i2c_board_info __initdata enterprise_regulators[] = {
	{
		I2C_BOARD_INFO("tps80031", 0x4A),
		.irq		= INT_EXTERNAL_PMU,
		.platform_data	= &tps_platform,
	},
};

/************************ GPIO based fixed regulator ****************/
static struct regulator_consumer_supply fixed_reg_led_3v3_en_supply[] = {
	REGULATOR_SUPPLY("v_led_3v3", NULL),
};

static struct regulator_consumer_supply fixed_reg_aud_a1v8_en_supply[] = {
        REGULATOR_SUPPLY("v_aud_a1v8", NULL),
};

static struct regulator_consumer_supply fixed_reg_vib_3v_en_supply[] = {
        REGULATOR_SUPPLY("v_vib_3v", NULL),
};

static struct regulator_consumer_supply fixed_reg_mhl_3v3_en_supply[] = {
        REGULATOR_SUPPLY("v_tp_3v3", NULL),
};

static struct regulator_consumer_supply fixed_reg_aud_3v3_en_supply[] = {
        REGULATOR_SUPPLY("v_aud_3v3", NULL),
};

static struct regulator_consumer_supply fixed_reg_cam_vcm_2v85_en_supply[] = {
        REGULATOR_SUPPLY("v_cam_vcm_2v85", NULL),
};

static struct regulator_consumer_supply fixed_reg_lcmio_1v8_en_supply[] = {
        REGULATOR_SUPPLY("v_lcmio_1v8", NULL),
};

static struct regulator_consumer_supply fixed_reg_lcm_3v3_en_supply[] = {
        REGULATOR_SUPPLY("v_lcm_3v3", NULL),
};

static struct regulator_consumer_supply fixed_reg_srio_1v8_en_supply[] = {
        REGULATOR_SUPPLY("v_srio_1v8", NULL),
};

static struct regulator_consumer_supply fixed_reg_cam2_d1v2_en_supply[] = {
        REGULATOR_SUPPLY("v_cam2_d1v2", NULL),
};

static struct regulator_consumer_supply fixed_reg_cam_d1v2_en_supply[] = {
        REGULATOR_SUPPLY("v_cam_d1v2", NULL),
};

static struct regulator_consumer_supply fixed_reg_camio_1v8_en_supply[] = {
        REGULATOR_SUPPLY("v_camio_1v8", NULL),
};

static struct regulator_consumer_supply fixed_reg_cam_a2v85_en_supply[] = {
        REGULATOR_SUPPLY("v_cam_a2v85", NULL),
};

static struct regulator_consumer_supply fixed_reg_mhl_1v2_en_supply[] = {
        REGULATOR_SUPPLY("v_mhl_1v2", NULL),
};

static struct regulator_consumer_supply fixed_reg_sdmmc_2v85_en_supply[] = {
        REGULATOR_SUPPLY("v_sdmmc_2v85", NULL),
};

static struct regulator_consumer_supply fixed_reg_vdd_fuse_en_supply[] = {
        REGULATOR_SUPPLY("vdd_fuse", NULL),
};

/* LCD_PWR0 (GPIO B2) from T30*/
//static struct regulator_consumer_supply fixed_reg_lcd_1v8_en_supply[] = {
//	REGULATOR_SUPPLY("lcd_vddio_en", NULL),
//};

/* LCD-D17 (GPIO M1) from T30*/
static struct regulator_consumer_supply gpio_reg_sdmmc3_vdd_sel_supply[] = {
//	REGULATOR_SUPPLY("vddio_sdmmc3_2v85_1v8", NULL),
//	REGULATOR_SUPPLY("sdmmc3_compu_pu", NULL),
	REGULATOR_SUPPLY("vddio_sdmmc", "NULL"),
};

static struct gpio_regulator_state gpio_reg_sdmmc3_vdd_sel_states[] = {
	{
		.gpios = 0,
		.value = 2850000,
	},
	{
		.gpios = 1,
		.value = 1800000,
	},
};

static struct gpio gpio_reg_sdmmc3_vdd_sel_gpios[] = {
	{
		.gpio = TEGRA_GPIO_PM1,
		.flags = 0,
		.label = "sdmmc3_vdd_sel",
	},
};

/* Macro for defining gpio regulator device data */
#define GPIO_REG(_id, _name, _input_supply, _active_high,		\
	_boot_state, _delay_us, _minmv, _maxmv)				\
	static struct regulator_init_data ri_data_##_name = 		\
	{								\
		.supply_regulator = _input_supply,			\
		.num_consumer_supplies =				\
			ARRAY_SIZE(gpio_reg_##_name##_supply),		\
		.consumer_supplies = gpio_reg_##_name##_supply,		\
		.constraints = {					\
			.name = "gpio_reg_"#_name,			\
			.min_uV = (_minmv)*1000,			\
			.max_uV = (_maxmv)*1000,			\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
		},							\
	};								\
	static struct gpio_regulator_config gpio_reg_##_name##_pdata =	\
	{								\
		.supply_name = _input_supply,				\
		.enable_gpio = -EINVAL,					\
		.enable_high = _active_high,				\
		.enabled_at_boot = _boot_state,				\
		.startup_delay = _delay_us,				\
		.gpios = gpio_reg_##_name##_gpios,			\
		.nr_gpios = ARRAY_SIZE(gpio_reg_##_name##_gpios),	\
		.states = gpio_reg_##_name##_states,			\
		.nr_states = ARRAY_SIZE(gpio_reg_##_name##_states),	\
		.type = REGULATOR_VOLTAGE,				\
		.init_data = &ri_data_##_name,				\
	};								\
	static struct platform_device gpio_reg_##_name##_dev = {	\
		.name	= "gpio-regulator",				\
		.id = _id,						\
		.dev	= { 						\
			.platform_data = &gpio_reg_##_name##_pdata,	\
		},							\
	}

GPIO_REG(4, sdmmc3_vdd_sel,  tps80031_rails(SMPS4),
		false, false, 0, 1000, 3300);

/* Macro for defining fixed regulator sub device data */
#define FIXED_REG(_id, _name, _input_supply, _gpio_nr, _active_high,	\
			_millivolts, _boot_state, _sdelay)		\
	static struct regulator_init_data ri_data_##_name =		\
	{								\
		.supply_regulator = _input_supply,			\
		.num_consumer_supplies =				\
			ARRAY_SIZE(fixed_reg_##_name##_supply),		\
		.consumer_supplies = fixed_reg_##_name##_supply,	\
		.constraints = {					\
			.valid_modes_mask = (REGULATOR_MODE_NORMAL |	\
					REGULATOR_MODE_STANDBY),	\
			.valid_ops_mask = (REGULATOR_CHANGE_MODE |	\
					REGULATOR_CHANGE_STATUS |	\
					REGULATOR_CHANGE_VOLTAGE),	\
		},							\
	};								\
	static struct fixed_voltage_config fixed_reg_##_name##_pdata =	\
	{								\
		.supply_name = "fixed_reg_"#_name,			\
		.microvolts = _millivolts * 1000,			\
		.gpio = _gpio_nr,					\
		.enable_high = _active_high,				\
		.enabled_at_boot = _boot_state,				\
		.init_data = &ri_data_##_name,				\
		.startup_delay = _sdelay,				\
	};								\
	static struct platform_device fixed_reg_##_name##_dev = {	\
		.name	= "reg-fixed-voltage",				\
		.id	= _id,						\
		.dev	= {						\
			.platform_data = &fixed_reg_##_name##_pdata,	\
		},							\
	}

FIXED_REG(0, led_3v3_en, NULL,
		ENT_TPS80031_GPIO_REGEN2, true, 3300, 1, 0);
FIXED_REG(1, aud_a1v8_en, NULL,
		TEGRA_GPIO_PD2, true, 1800, 1, 0);
FIXED_REG(2, vib_3v_en, NULL,
		TEGRA_GPIO_PE7, true, 3000, 1, 0);
FIXED_REG(3, mhl_3v3_en, NULL,
		TEGRA_GPIO_PY2, true, 3300, 0, 0);
FIXED_REG(4, aud_3v3_en,  NULL,
		TEGRA_GPIO_PB2, true, 3300, 0, 0);
FIXED_REG(5, cam_vcm_2v85_en,  NULL,
		TEGRA_GPIO_PM7, true, 2800, 0, 0);
FIXED_REG(6, lcm_3v3_en,  NULL,
		TEGRA_GPIO_PE2, true, 3300, 1, 0);
FIXED_REG(7, lcmio_1v8_en,  NULL,
		TEGRA_GPIO_PE5, true, 1800, 1, 0);
FIXED_REG(8, srio_1v8_en,  NULL,
		TEGRA_GPIO_PY3, true, 1800, 1, 0);
FIXED_REG(9, cam2_d1v2_en,  NULL,
		TEGRA_GPIO_PF6, true, 1800, 0, 0);
FIXED_REG(10, cam_d1v2_en,  NULL,
		TEGRA_GPIO_PF5, true, 1800, 0, 0);
FIXED_REG(11, camio_1v8_en, NULL,
		TEGRA_GPIO_PBB4, true, 1800, 0, 0);
FIXED_REG(12, cam_a2v85_en,  NULL,
		TEGRA_GPIO_PE3, true, 2800, 0, 0);
FIXED_REG(13, mhl_1v2_en,  NULL,
		TEGRA_GPIO_PE4, true, 1200, 0, 0);
FIXED_REG(14, sdmmc_2v85_en,  NULL,
		TEGRA_GPIO_PM3, true, 2800, 1, 0);
FIXED_REG(15, vdd_fuse_en,  NULL,
		TEGRA_GPIO_PM0, true, 3300, 1, 0);

#define ADD_FIXED_REG(_name)	(&fixed_reg_##_name##_dev)

#define FIXED_REGS_COMMON		\
	ADD_FIXED_REG(sdmmc_2v85_en),	\
	ADD_FIXED_REG(vib_3v_en),	\
	ADD_FIXED_REG(led_3v3_en)

static struct platform_device *fixed_regs_devices_a03[] = {
	FIXED_REGS_COMMON,
	ADD_FIXED_REG(aud_a1v8_en),
	ADD_FIXED_REG(mhl_3v3_en),
	ADD_FIXED_REG(aud_3v3_en),
	ADD_FIXED_REG(cam_vcm_2v85_en),
	ADD_FIXED_REG(lcm_3v3_en),
	ADD_FIXED_REG(lcmio_1v8_en),
	ADD_FIXED_REG(srio_1v8_en),
	ADD_FIXED_REG(cam2_d1v2_en),
	ADD_FIXED_REG(cam_d1v2_en),
	ADD_FIXED_REG(camio_1v8_en),
	ADD_FIXED_REG(cam_a2v85_en),
	ADD_FIXED_REG(mhl_1v2_en),
	ADD_FIXED_REG(vdd_fuse_en),
};

#define ADD_GPIO_REG(_name) (&gpio_reg_##_name##_dev)
static struct platform_device *gpio_regs_devices[] = {
	ADD_GPIO_REG(sdmmc3_vdd_sel),
};

static int __init enterprise_fixed_regulator_init(void)
{
	int i;
	struct board_info board_info;
	struct platform_device **fixed_regs_devices;
	int nfixreg_devs;

	tegra_get_board_info(&board_info);

		fixed_regs_devices = fixed_regs_devices_a03;
		nfixreg_devs = ARRAY_SIZE(fixed_regs_devices_a03);

	for (i = 0; i < nfixreg_devs; ++i) {
		struct fixed_voltage_config *fixed_reg_pdata =
				fixed_regs_devices[i]->dev.platform_data;
		if (fixed_reg_pdata->gpio < TEGRA_NR_GPIOS)
			tegra_gpio_enable(fixed_reg_pdata->gpio);
	}
	return platform_add_devices(fixed_regs_devices, nfixreg_devs);
}

static int __init enterprise_gpio_regulator_init(void)
{
	int i, j;

	for (i = 0; i < ARRAY_SIZE(gpio_regs_devices); ++i) {
		struct gpio_regulator_config *gpio_reg_pdata =
			gpio_regs_devices[i]->dev.platform_data;
		for (j = 0; j < gpio_reg_pdata->nr_gpios; ++j) {
			if (gpio_reg_pdata->gpios[j].gpio < TEGRA_NR_GPIOS)
				tegra_gpio_enable(gpio_reg_pdata->gpios[j].gpio);
		}
	}
	return platform_add_devices(gpio_regs_devices,
				    ARRAY_SIZE(gpio_regs_devices));
}

static int __init enterprise_regulators_fixed_gpio_init(void)
{
	int ret;

	ret = enterprise_fixed_regulator_init();
	  if (ret)
	    return ret;

	ret = enterprise_gpio_regulator_init();
	return ret;
}
subsys_initcall_sync(enterprise_regulators_fixed_gpio_init);

#ifdef CONFIG_PM
static void endeavor_suspend_work(void)
{
	int ret = gpio_direction_output(TEGRA_GPIO_PF7, 0);
	if (ret)
		pr_err("%s: fail to output low gpio PF7\n", __func__);
}

static void endeavor_resume_work(void)
{
	int ret = gpio_direction_output(TEGRA_GPIO_PF7, 1);
	if (ret)
		pr_err("%s: fail to output high gpio PF7\n", __func__);
}
#endif

static int __init endeavor_gpio_rtc_init(void)
{
	int ret = gpio_request(TEGRA_GPIO_PF7, "pmu_msecure");
	if (ret) {
		pr_err("%s unable to request gpio PF7\n", __func__);
		goto fail;
	}
	ret = gpio_direction_output(TEGRA_GPIO_PF7, 1);
	if (ret) {
		pr_err("%s unable to set output gpio PF7\n", __func__);
		goto fail;
	}
	tegra_gpio_enable(TEGRA_GPIO_PF7);
#ifdef CONFIG_PM
	tps_platform.suspend_work = endeavor_suspend_work;
	tps_platform.resume_work = endeavor_resume_work;
#endif
	return 0;
fail:
	gpio_free(TEGRA_GPIO_PF7);
	return ret;
}

static void enterprise_power_off(void)
{
	int ret;

        pr_info("enterprise: Powering off the device or"
                " enter offmode charging\n");
        ret = tps80031_power_off_or_reboot();

	if (ret)
		pr_err("enterprise: failed to power off\n");
	while(1);
}

int __init enterprise_regulator_init(void)
{
	void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
	u32 pmc_ctrl;
	u32 pmc_dpd_pads;
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	/* configure the power management controller to trigger PMU
	 * interrupts when low */

	pmc_ctrl = readl(pmc + PMC_CTRL);
	writel(pmc_ctrl | PMC_CTRL_INTR_LOW, pmc + PMC_CTRL);

	pmc_dpd_pads = readl(pmc + PMC_DPD_PADS_ORIDE);
	writel(pmc_dpd_pads & ~PMC_DPD_PADS_ORIDE_BLINK , pmc + PMC_DPD_PADS_ORIDE);

	endeavor_gpio_rtc_init();

	tps_platform.num_subdevs = ARRAY_SIZE(tps80031_devs_a03);
	tps_platform.subdevs = tps80031_devs_a03;
	tps_platform.pupd_init_data = pupd_idata;
	tps_platform.pupd_init_data_size = ARRAY_SIZE(pupd_idata);

	i2c_register_board_info(4, enterprise_regulators, 1);
	pm_power_off = enterprise_power_off;

	return 0;
}

static void enterprise_board_suspend(int lp_state, enum suspend_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_SUSPEND_BEFORE_CPU))
		tegra_console_uart_suspend();
}

static void enterprise_board_resume(int lp_state, enum resume_stage stg)
{
	if ((lp_state == TEGRA_SUSPEND_LP1) && (stg == TEGRA_RESUME_AFTER_CPU))
		tegra_console_uart_resume();
}

static struct tegra_suspend_platform_data enterprise_suspend_data = {
	.cpu_timer	= 2000,
	.cpu_off_timer	= 200,
	.suspend_mode	= TEGRA_SUSPEND_LP0,
	.core_timer	= 0x7e7e,
	.core_off_timer = 0,
	.corereq_high	= true,
	.sysclkreq_high	= true,
	.board_suspend = enterprise_board_suspend,
	.board_resume = enterprise_board_resume,
};

static void enterprise_init_deep_sleep_mode(void)
{
	struct board_info bi;
	tegra_get_board_info(&bi);

	if (bi.board_id == BOARD_E1205 && bi.fab == BOARD_FAB_A01)
		enterprise_suspend_data.suspend_mode = TEGRA_SUSPEND_LP1;

	if ((bi.board_id == BOARD_E1205 && (bi.sku & BOARD_SKU_VF_BIT) == 0) ||
	    (bi.board_id == BOARD_E1197 && (bi.sku & BOARD_SKU_VF_BIT)))
		enterprise_suspend_data.cpu_timer = 8000;
}

int __init enterprise_suspend_init(void)
{
	enterprise_init_deep_sleep_mode();
	tegra_init_suspend(&enterprise_suspend_data);
	return 0;
}

#ifdef CONFIG_TEGRA_EDP_LIMITS
int __init enterprise_edp_init(void)
{
	unsigned int regulator_mA;

	regulator_mA = get_maximum_cpu_current_supported();
	if (!regulator_mA) {
		regulator_mA = 5000; /* regular AP30 */
	}
	pr_info("%s: CPU regulator %d mA\n", __func__, regulator_mA);

	tegra_init_cpu_edp_limits(regulator_mA);
	tegra_init_system_edp_limits(TEGRA_BPC_CPU_PWR_LIMIT);
	return 0;
}
#endif

static struct tegra_bpc_mgmt_platform_data bpc_mgmt_platform_data = {
	.gpio_trigger = TEGRA_BPC_TRIGGER,
	.bpc_mgmt_timeout = TEGRA_BPC_TIMEOUT,
};

static struct platform_device enterprise_bpc_mgmt_device = {
	.name		= "tegra-bpc-mgmt",
	.id		= -1,
	.dev		= {
		.platform_data = &bpc_mgmt_platform_data,
	},
};

void __init enterprise_bpc_mgmt_init(void)
{
	int int_gpio = TEGRA_GPIO_TO_IRQ(TEGRA_BPC_TRIGGER);
	tegra_gpio_enable(TEGRA_BPC_TRIGGER);

#ifdef CONFIG_SMP
	cpumask_setall(&(bpc_mgmt_platform_data.affinity_mask));
	irq_set_affinity_hint(int_gpio,
				&(bpc_mgmt_platform_data.affinity_mask));
	irq_set_affinity(int_gpio, &(bpc_mgmt_platform_data.affinity_mask));
#endif
	platform_device_register(&enterprise_bpc_mgmt_device);

	return;
}
