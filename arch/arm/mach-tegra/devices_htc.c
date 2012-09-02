/* linux/arch/arm/mach-tegra/devices_htc.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (C) 2007-2009 HTC Corporation.
 * Author: Thomas Tsai <thomas_tsai@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/reboot.h>
#include <asm/mach/flash.h>
#include <asm/setup.h>

#include <mach/dma.h>
#include <mach/board_htc.h>
#include <mach/gpio.h>
#include <mach/restart.h>

#include "devices.h"
#include "board.h"
#include "gpio-names.h"

#define MFG_GPIO_TABLE_MAX_SIZE 0x400
#define EMMC_FREQ_533 533
#define EMMC_FREQ_400 400

#define NVDUMPER_CLEAN 		0xf000caf3U
#define NVDUMPER_DIRTY 		0xdeadbeefU

#define ATAG_GS         	0x5441001d
#define ATAG_PS         	0x5441001c
#define ATAG_CSA		0x5441001f
#define ATAG_GRYO_GSENSOR	0x54410020
#define ATAG_PS_TYPE 		0x4d534D77
#define ATAG_TP_TYPE 		0x4d534D78
#define ATAG_MFG_GPIO_TABLE 	0x59504551

bool enable_debug_ll = false;

static int zchg_mode = 0;
static int build_flag;
static int sku_id;
static int mfg_mode;

static unsigned long kernel_flag;
static unsigned long extra_kernel_flag;
static unsigned int bl_ac_flag = 0;
static unsigned char mfg_gpio_table[MFG_GPIO_TABLE_MAX_SIZE];

static char *emmc_tag;
static char *board_sn;
static char *board_mb_sn;
static char *keycap_tag = NULL;
static char *cid_tag = NULL;
static char *carrier_tag = NULL;

unsigned char pcbid = PROJECT_PHASE_INVALID;

static uint32_t *nvdumper_ptr;
struct htc_reboot_params *reboot_params;

unsigned reboot_battery_first_level = 0;
EXPORT_SYMBOL_GPL(reboot_battery_first_level);

static int __init board_keycaps_tag(char *get_keypads)
{
	if (strlen(get_keypads))
		keycap_tag = get_keypads;
	else
		keycap_tag = NULL;
	return 1;
}
__setup("androidboot.keycaps=", board_keycaps_tag);

void board_get_keycaps_tag(char **ret_data)
{
	*ret_data = keycap_tag;
}
EXPORT_SYMBOL(board_get_keycaps_tag);

static int __init board_set_cid_tag(char *get_hboot_cid)
{
	if (strlen(get_hboot_cid))
		cid_tag = get_hboot_cid;
	else
		cid_tag = NULL;
	return 1;
}
__setup("androidboot.cid=", board_set_cid_tag);

void board_get_cid_tag(char **ret_data)
{
	*ret_data = cid_tag;
}
EXPORT_SYMBOL(board_get_cid_tag);

static int __init board_set_carrier_tag(char *get_hboot_carrier)
{
	if (strlen(get_hboot_carrier))
		carrier_tag = get_hboot_carrier;
	else
		carrier_tag = NULL;
	return 1;
}
__setup("androidboot.carrier=", board_set_carrier_tag);

void board_get_carrier_tag(char **ret_data)
{
	*ret_data = carrier_tag;
}
EXPORT_SYMBOL(board_get_carrier_tag);

unsigned int gs_kvalue;
EXPORT_SYMBOL(gs_kvalue);

static int __init parse_tag_gs_calibration(const struct tag *tag)
{
	gs_kvalue = tag->u.revision.rev;
	printk(KERN_DEBUG "%s: gs_kvalue = 0x%x\n", __func__, gs_kvalue);
	return 0;
}

__tagtable(ATAG_GS, parse_tag_gs_calibration);

unsigned int ps_kparam1;
EXPORT_SYMBOL(ps_kparam1);

unsigned int ps_kparam2;
EXPORT_SYMBOL(ps_kparam2);

static int __init parse_tag_ps_calibration(const struct tag *tag)
{
	ps_kparam1 = tag->u.serialnr.low;
	ps_kparam2 = tag->u.serialnr.high;

	printk(KERN_INFO "%s: ps_kparam1 = 0x%x, ps_kparam2 = 0x%x\n",
		__func__, ps_kparam1, ps_kparam2);

	return 0;
}

__tagtable(ATAG_PS, parse_tag_ps_calibration);

unsigned int als_kadc;
EXPORT_SYMBOL(als_kadc);

static int __init parse_tag_als_calibration(const struct tag *tag)
{
	als_kadc = tag->u.als_kadc.kadc;

	printk(KERN_INFO "%s: als_kadc = 0x%x\n",
		__func__, als_kadc);

	return 0;
}
__tagtable(ATAG_ALS, parse_tag_als_calibration);

unsigned int csa_kvalue1;
EXPORT_SYMBOL(csa_kvalue1);

unsigned int csa_kvalue2;
EXPORT_SYMBOL(csa_kvalue2);

unsigned int csa_kvalue3;
EXPORT_SYMBOL(csa_kvalue3);

unsigned int csa_kvalue4;
EXPORT_SYMBOL(csa_kvalue4);

unsigned int csa_kvalue5;
EXPORT_SYMBOL(csa_kvalue5);

static int __init parse_tag_csa_calibration(const struct tag *tag)
{
	unsigned int *ptr = (unsigned int *)&tag->u;
	csa_kvalue1 = ptr[0];
	csa_kvalue2 = ptr[1];
	csa_kvalue3 = ptr[2];
	csa_kvalue4 = ptr[3];
	csa_kvalue5 = ptr[4];

	printk(KERN_DEBUG "csa_kvalue1 = 0x%x, csa_kvalue2 = 0x%x, "
		"csa_kvalue3 = 0x%x, csa_kvalue4 = 0x%x, csa_kvalue5 = 0x%x\n", 
		csa_kvalue1, csa_kvalue2, csa_kvalue3, csa_kvalue4, csa_kvalue5);

	return 0;
}
__tagtable(ATAG_CSA, parse_tag_csa_calibration);

#ifdef CAMERA_CALIBRATION
#define ATAG_CAM_AWB    0x59504550
unsigned char awb_kvalues[2048];
EXPORT_SYMBOL(awb_kvalues);

static int __init parse_tag_awb_calibration(const struct tag *tag)
{
    printk(KERN_INFO "[CAM] %s: read MFG calibration data\n", __func__);
    unsigned char *ptr = (unsigned char *)&tag->u;

    memcpy(&awb_kvalues[0], ptr, sizeof(awb_kvalues));

    return 0;
}
__tagtable(ATAG_CAM_AWB, parse_tag_awb_calibration);
#endif

unsigned char gyro_gsensor_kvalue[37];
EXPORT_SYMBOL(gyro_gsensor_kvalue);

static int __init parse_tag_gyro_gsensor_calibration(const struct tag *tag)
{
	unsigned char *ptr = (unsigned char *)&tag->u;
	memcpy(&gyro_gsensor_kvalue[0], ptr, sizeof(gyro_gsensor_kvalue));
	return 0;
}
__tagtable(ATAG_GRYO_GSENSOR, parse_tag_gyro_gsensor_calibration);

int __init board_mfg_mode_init(char *s)
{
	if (!strcmp(s, "normal"))
		mfg_mode = 0;
	else if (!strcmp(s, "factory2"))
		mfg_mode = 1;
	else if (!strcmp(s, "recovery"))
		mfg_mode = 2;
	else if (!strcmp(s, "charge"))
		mfg_mode = 3;
	else if (!strcmp(s, "power_test"))
		mfg_mode = 4;
	else if (!strcmp(s, "offmode_charging"))
		mfg_mode = 5;

	return 1;
}

int board_mfg_mode(void)
{
	return mfg_mode;
}

EXPORT_SYMBOL(board_mfg_mode);

__setup("androidboot.mode=", board_mfg_mode_init);

int __init board_zchg_mode_init(char *s)
{
	if (!strcmp(s, "1"))
		zchg_mode = 1;
	else if (!strcmp(s, "2"))
		zchg_mode = 2;
	else if (!strcmp(s, "3"))
		zchg_mode = 3;

	return 1;
}

int board_zchg_mode(void)
{
	return zchg_mode;
}

EXPORT_SYMBOL(board_zchg_mode);
__setup("enable_zcharge=", board_zchg_mode_init);

static int __init board_bootloader_setup(char *str)
{
	char temp[strlen(str) + 1];
	char *p = NULL;
	char *build = NULL;
	char *args = temp;

	printk(KERN_INFO "%s: %s\n", __func__, str);

	strcpy(temp, str);

	/*parse the last parameter*/
	while ((p = strsep(&args, ".")) != NULL) build = p;

	if (build) {
		if (strcmp(build, "0000") == 0) {
			printk(KERN_INFO "%s: SHIP BUILD\n", __func__);
			build_flag = SHIP_BUILD;
		} else if (strcmp(build, "2000") == 0) {
			printk(KERN_INFO "%s: ENG BUILD\n", __func__);
			build_flag = ENG_BUILD;
		} else {
			printk(KERN_INFO "%s: default ENG BUILD\n", __func__);
			build_flag = ENG_BUILD;
		}
	}
	return 1;
}
__setup("androidboot.bootloader=", board_bootloader_setup);

int board_build_flag(void)
{
	return build_flag;
}
EXPORT_SYMBOL(board_build_flag);

static int __init board_serialno_setup(char *serialno)
{
	char *str;

	str = serialno;
#ifdef CONFIG_USB_FUNCTION
	msm_hsusb_pdata.serial_number = str;
#endif
	board_sn = str;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);

static int __init board_mb_serialno_setup(char *serialno)
{
	char *str;

	str = serialno;
	board_mb_sn = str;
	return 1;
}
__setup("androidboot.mb_serialno=", board_mb_serialno_setup);

char *board_serialno(void)
{
	return board_sn;
}

char *board_mb_serialno(void)
{
	return board_mb_sn;
}

int board_get_sku_tag()
{
	return sku_id;
}

int ps_type;
EXPORT_SYMBOL(ps_type);

int __init tag_ps_parsing(const struct tag *tags)
{
	ps_type = tags->u.revision.rev;

	printk(KERN_DEBUG "%s: PS type = 0x%x\n", __func__,
		ps_type);

	return ps_type;
}
__tagtable(ATAG_PS_TYPE, tag_ps_parsing);

int tp_type;
EXPORT_SYMBOL(tp_type);

int __init tag_tp_parsing(const struct tag *tags)
{
	tp_type = tags->u.revision.rev;

	printk(KERN_DEBUG "%s: TS type = 0x%x\n", __func__,
		tp_type);

	return tp_type;
}
__tagtable(ATAG_TP_TYPE, tag_tp_parsing);

int __init parse_tag_mfg_gpio_table(const struct tag *tags)
{
	   unsigned char *dptr = (unsigned char *)(&tags->u);
	   __u32 size;

	   size = min((__u32)(tags->hdr.size - 2) * sizeof(__u32), (__u32)MFG_GPIO_TABLE_MAX_SIZE);
	   memcpy(mfg_gpio_table, dptr, size);
	   return 0;
}
__tagtable(ATAG_MFG_GPIO_TABLE, parse_tag_mfg_gpio_table);

char *board_get_mfg_sleep_gpio_table(void)
{
		return mfg_gpio_table;
}
EXPORT_SYMBOL(board_get_mfg_sleep_gpio_table);

static int __init board_set_emmc_tag(char *get_hboot_emmc)
{
	if (strlen(get_hboot_emmc))
		emmc_tag = get_hboot_emmc;
	else
		emmc_tag = NULL;
	return 1;
}
__setup("androidboot.emmc=", board_set_emmc_tag);

int board_emmc_boot(void)
{
	if (emmc_tag) {
		if (!strcmp(emmc_tag, "true"))
	return 1;
}

	return 0;
}

int __init parse_tag_extdiag(const struct tag *tags)
{
	const struct tag *t = tags;

	for (; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == 0x54410021)
			return t->u.revision.rev;
	}
	return 0;
}

int __init kernel_flag_init(char *s)
{
	int ret;
	ret = strict_strtoul(s, 16, &kernel_flag);
	return 1;
}
__setup("kernelflag=", kernel_flag_init);

unsigned int get_kernel_flag(void)
{
	return kernel_flag;
}

int __init extra_kernel_flag_init(char *s)
{
	int ret;
	ret = strict_strtoul(s, 16, &extra_kernel_flag);
	return 1;
}
__setup("kernelflagex=", extra_kernel_flag_init);

unsigned int get_extra_kernel_flag(void)
{
	return extra_kernel_flag;
}

BLOCKING_NOTIFIER_HEAD(psensor_notifier_list);

int register_notifier_by_psensor(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&psensor_notifier_list, nb);
}

int unregister_notifier_by_psensor(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&psensor_notifier_list, nb);
}

/* should call only one time */
static int __htc_get_pcbid_info(void)
{
	return pcbid;
}

static char* __pcbid_to_name(signed int id)
{
	return "<Latest HW phase>";
}

const int htc_get_pcbid_info(void)
{
	static int __pcbid = PROJECT_PHASE_INVALID;
	if (__pcbid == PROJECT_PHASE_INVALID)
	{
		__pcbid = __htc_get_pcbid_info();
		pr_info("project phase: %s (id=%d)\n",
				__pcbid_to_name(__pcbid), __pcbid);
	}
	return __pcbid;
}

#ifdef CONFIG_DEBUG_LL_DYNAMIC
static int __init board_set_debug_ll(char *val)
{
       pr_debug("%s: low level debug: on\n", __func__);
       enable_debug_ll = true;
       return 1;
}
__setup("debug_ll", board_set_debug_ll);
#endif

int __init bl_ac_flag_init(char *s)
{
	bl_ac_flag=1;
	return 1;
}
__setup("bl_ac_in", bl_ac_flag_init);

unsigned int get_bl_ac_in_flag(void)
{
	return bl_ac_flag;
}

int get_dirty_state(void)
{
	uint32_t val;

	val = ioread32(nvdumper_ptr);
	if (val == NVDUMPER_DIRTY)
		return 1;
	else if (val == NVDUMPER_CLEAN)
		return 0;
	else
		return -1;
}

void set_dirty_state(int dirty)
{
	if (dirty)
		iowrite32(NVDUMPER_DIRTY, nvdumper_ptr);
	else
		iowrite32(NVDUMPER_CLEAN, nvdumper_ptr);
}

static int nvdumper_reboot_cb(struct notifier_block *nb,
		unsigned long event, void *unused)
{
	printk(KERN_INFO "nvdumper: rebooting cleanly.\n");
	set_dirty_state(0);
	return NOTIFY_DONE;
}

struct notifier_block nvdumper_reboot_notifier = {
	.notifier_call = nvdumper_reboot_cb,
};

static int __init nvdumper_init(void)
{
	int ret, dirty;

	printk(KERN_INFO "nvdumper: nvdumper_reserved:0x%08lx\n", nvdumper_reserved);
	if (!nvdumper_reserved) {
		printk(KERN_INFO "nvdumper: not configured\n");
		return -ENOTSUPP;
	}
	nvdumper_ptr = ioremap_nocache(nvdumper_reserved,
			NVDUMPER_RESERVED_LEN);
	printk(KERN_INFO "nvdumper: nvdumper_ptr:0x%p\n", nvdumper_ptr);
	if (!nvdumper_ptr) {
		printk(KERN_INFO "nvdumper: failed to ioremap memory "
			"at 0x%08lx\n", nvdumper_reserved);
		return -EIO;
	}
	reboot_params = ioremap_nocache(nvdumper_reserved - NVDUMPER_RESERVED_LEN,
			NVDUMPER_RESERVED_LEN);
	printk(KERN_INFO "nvdumper: reboot_params:0x%p\n", reboot_params);
	if (!reboot_params) {
		printk(KERN_INFO "nvdumper: failed to ioremap memory "
			"at 0x%08lx\n", nvdumper_reserved - NVDUMPER_RESERVED_LEN);
		return -EIO;
	}
	reboot_battery_first_level = reboot_params->battery_level;
	memset(reboot_params, 0x0, sizeof(struct htc_reboot_params));
	ret = register_reboot_notifier(&nvdumper_reboot_notifier);
	printk(KERN_INFO "nvdumper: ret:%d\n", ret);
	if (ret)
		return ret;
	dirty = get_dirty_state();
	printk(KERN_INFO "nvdumper: dirty:%d\n", dirty);
	switch (dirty) {
	case 0:
		printk(KERN_INFO "nvdumper: last reboot was clean\n");
		break;
	case 1:
		printk(KERN_INFO "nvdumper: last reboot was dirty\n");
		break;
	default:
		printk(KERN_INFO "nvdumper: last reboot was unknown\n");
		break;
	}
	set_dirty_state(1);
	return 0;
}
module_init(nvdumper_init);

static int __exit nvdumper_exit(void)
{
	unregister_reboot_notifier(&nvdumper_reboot_notifier);
	set_dirty_state(0);
	iounmap(nvdumper_ptr);
	iounmap(reboot_params);
	return 0;
}
module_exit(nvdumper_exit);
