/*
 * arch/arm/mach-tegra/include/mach/board_htc.c
 *
 * Copyright (c) 2011-2012, NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __ASM_ARCH_MSM_BOARD_HTC_H
#define __ASM_ARCH_MSM_BOARD_HTC_H

#include <linux/types.h>
#include <linux/list.h>
#include <asm/setup.h>

#define SHIP_BUILD  0
#define MFG_BUILD   1
#define ENG_BUILD   2

int board_mfg_mode(void);
int board_zchg_mode(void);

void htc_gpio_set_diag_gpio_table(unsigned char* dwMFG_gpio_table);

int __init board_mcp_monodie(void);
int __init parse_tag_smi(const struct tag *tags);
int __init parse_tag_hwid(const struct tag *tags);
int __init parse_tag_monodie(const struct tag *tags);

int board_get_sku_tag(void);
void board_get_keycaps_tag(char **);
void board_get_cid_tag(char **);
void board_get_carrier_tag(char **);
void board_get_mid_tag(char **);
int board_emmc_boot(void);

void notify_usb_connected(int online);

char *board_serialno(void);

unsigned int get_kernel_flag(void);
unsigned int get_extra_kernel_flag(void);
unsigned int get_bl_ac_in_flag(void);

extern int panel_type;
extern int usb_phy_error;

extern unsigned long tegra_bootloader_panel_lsb;
extern unsigned long tegra_bootloader_panel_msb;

extern const int htc_get_pcbid_info(void);
const bool is_modem_rework_phase(void);

enum {
	PROJECT_PHASE_INVALID = -2,
	PROJECT_PHASE_EVM = -1,
	PROJECT_PHASE_XA  =  0,
	PROJECT_PHASE_XB  =  1,
	PROJECT_PHASE_XC  =  2,
	PROJECT_PHASE_XD  =  3,
	PROJECT_PHASE_XE  =  4,
	PROJECT_PHASE_XF  =  5,
	PROJECT_PHASE_XG  =  6,
	PROJECT_PHASE_XH  =  7,
	PROJECT_PHASE_A   =  0x80,
	PROJECT_PHASE_B   =  0x81,
	PROJECT_PHASE_C   =  0x82,
	PROJECT_PHASE_D   =  0x83,
	PROJECT_PHASE_E   =  0x84,
	PROJECT_PHASE_F   =  0x85,
	PROJECT_PHASE_G   =  0x86,
	PROJECT_PHASE_H   =  0x87,
};

enum {
	KERNEL_FLAG_PM_MONITOR = BIT(25),
};

unsigned int get_radio_flag(void);

#endif
