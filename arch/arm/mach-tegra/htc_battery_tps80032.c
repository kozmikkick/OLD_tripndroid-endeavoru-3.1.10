/* arch/arm/mach-tegra/htc_battery_tps80032.c
 *
 * Copyright (C) 2011 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>

#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/tps65200.h>
#include <linux/tps80032_adc.h>
#include <linux/tps80032_vsys_alarm.h>
#include <linux/reboot.h>
#include <linux/miscdevice.h>
#include <linux/android_alarm.h>
#include <linux/suspend.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>

#include <asm/mach-types.h>

#include <mach/board_htc.h>
#include <mach/cable_detect.h>
#include <mach/restart.h>
#include <mach/htc_battery_core.h>
#include <mach/htc_battery_tps80032.h>

/* used for debug if function called */
#define BATT_LOG(fmt, ...) do { \
	struct timespec ts; \
	struct rtc_time tm; \
	getnstimeofday(&ts); \
	rtc_time_to_tm(ts.tv_sec, &tm); \
	printk(KERN_INFO "[BATT] " fmt \
	" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", ##__VA_ARGS__, \
	ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
	tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

#define BATT_ERR(fmt, ...) do { \
	struct timespec ts; \
	struct rtc_time tm; \
	getnstimeofday(&ts); \
	rtc_time_to_tm(ts.tv_sec, &tm); \
	printk(KERN_ERR "[BATT] err:" fmt \
	" at %lld (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", ##__VA_ARGS__, \
	ktime_to_ns(ktime_get()), tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, \
	tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec); \
} while (0)

#ifdef PWR_DEVICE_TAG
#undef PWR_DEVICE_TAG
#endif
#define PWR_DEVICE_TAG "CHAR"

#define WRITE_PWR_SAVE_DISABLE	0x1000
#define ALL_AC_CHARGING		0x4000000

#define BATT_SUSPEND_CHECK_TIME			3600
#define BATT_SUSPEND_PHONE_CALL_CHECK_TIME	300
#define BATT_LATE_RESUME_CHECK_TIME		300
#define BATT_TIMER_CHECK_TIME			180
#define FIRST_ADC_READ_DELAY			5

#define BATT_VOLT_CHANNEL	0
#define BATT_TEMP_CHANNEL	1
#define BATT_ID_CHANNEL		3
#define BATT_CURR_CHANNEL	5

#define MV_TO_ADC_BITS(batt_vol) ((batt_vol) * 4095 / 1250)

#define QB_LPB_SHUTDOWN_VOLTAGE 3100

enum {
	ATTR_REBOOT_LEVEL = 0,
	ATTR_QB_REASON,
};

static void mbat_in_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(mbat_in_struct, mbat_in_func);
static struct kset *htc_batt_kset;

static void reverse_current_func(struct work_struct *work);
static DECLARE_DELAYED_WORK(reverse_current_struct, reverse_current_func);

static int htc_batt_phone_call;
static int is_alarm_not_work = 0;

struct htc_battery_info {
	int device_id;

	/* lock to protect the battery info */
	struct mutex info_lock;

	spinlock_t batt_lock;
	int is_open;

	struct kobject batt_timer_kobj;
	struct kobject batt_cable_kobj;

	struct wake_lock vbus_wake_lock;
	char debug_log[DEBUG_LOG_LENGTH];
	char power_meter[POWER_METER_LENGTH];

	struct battery_info_reply rep;

	struct battery_adc_reply adc_data;
	int adc_vref[ADC_REPLY_ARRAY_SIZE];

	int guage_driver;
	int charger;
	int vzero_clb_channel;
	int volt_adc_offset;
	int power_off_by_id;
	int check2_value;
	int first_level_ready;
	int is_cable_in;
	int online;
#ifdef CONFIG_HAS_EARLYSUSPEND
 	struct early_suspend early_suspend;
#endif

};
static struct htc_battery_info htc_batt_info;

struct htc_battery_timer {
	unsigned long batt_system_jiffies;
	unsigned long batt_suspend_ms;
	unsigned long total_time_ms;
	unsigned int batt_alarm_status;
	unsigned int batt_alarm_enabled;
	unsigned int alarm_timer_flag;
	unsigned int time_out;
	struct work_struct batt_work;
	struct work_struct batt_first_work;
	struct alarm batt_check_wakeup_alarm;
	struct timer_list batt_timer;
	struct timer_list batt_first_timer;
	struct workqueue_struct *batt_wq;
	struct wake_lock battery_lock;
};
static struct htc_battery_timer htc_batt_timer;

static void usb_status_notifier_func(int online);
static struct t_usb_status_notifier usb_status_notifier = {
	.name = "htc_battery_tps80032",
	.func = usb_status_notifier_func,
};

static int htc_battery_initial;
static int htc_full_level_flag;
static int htc_battery_set_charging(int ctl);

static void tps_int_notifier_func(int int_reg, int value);
static struct tps65200_chg_int_notifier tps_int_notifier = {
	.name = "htc_battery_tps80032",
	.func = tps_int_notifier_func,
};

static ssize_t tps80032_first_batt_show_attributes(struct device *dev,
					struct device_attribute *attr,
					char *buf);

static int tps80032_qb_store_attributes(struct device *dev,
 		struct device_attribute *attr, const char *buf, size_t size);

static void tps_int_notifier_func(int int_reg, int value)
{
	if (int_reg == CHECK_INT1) {
		htc_batt_info.rep.over_vchg = (unsigned int)value;
		htc_battery_core_update();
	} else if (int_reg == CHECK_INT2) {
		htc_batt_info.check2_value = value;
		schedule_delayed_work(&reverse_current_struct
					, msecs_to_jiffies(5000));
	}
}

static int battery_alarm_notifier_func(struct notifier_block *nfb,
					unsigned long value, void *data);
static struct notifier_block battery_alarm_notifier = {
	.notifier_call = battery_alarm_notifier_func,
};

static int battery_alarm_notifier_func(struct notifier_block *nfb,
		unsigned long status, void *data)
{
	htc_batt_timer.batt_alarm_status++;
	BATT_LOG("%s: batt alarm status %u", __func__, htc_batt_timer.batt_alarm_status);
	return 0;
}


static void update_wake_lock(int status)
{
	if (status == CHARGER_USB)
		wake_lock(&htc_batt_info.vbus_wake_lock);
	else
		/* give userspace some time to see the uevent and update LED state or whatnot...*/
		wake_lock_timeout(&htc_batt_info.vbus_wake_lock, HZ * 5);
}

static void reverse_current_func(struct work_struct *work)
{
	char message[16];
	char *envp[] = { message, NULL };

	scnprintf(message, 16, "REVERSE_CURR=%d", htc_batt_info.check2_value);

	update_wake_lock(htc_batt_info.rep.charging_source);

	kobject_uevent_env(&htc_batt_info.batt_cable_kobj, KOBJ_CHANGE, envp);
}

static void usb_status_notifier_func(int online)
{
	char message[16];
	char *envp[] = { message, NULL };

	mutex_lock(&htc_batt_info.info_lock);

	htc_batt_info.online = online;
	switch (online) {
	case CONNECT_TYPE_USB:
		BATT_LOG("cable USB");
		if ( !!(get_kernel_flag() & ALL_AC_CHARGING) ) {
			BATT_LOG("Force AC charging, fake as AC");
			htc_batt_info.rep.charging_source = CHARGER_AC;
		} else
			htc_batt_info.rep.charging_source = CHARGER_USB;
		htc_batt_info.is_cable_in = 1;
		break;
	case CONNECT_TYPE_AC:
		BATT_LOG("cable AC");
		htc_batt_info.rep.charging_source = CHARGER_AC;
		htc_batt_info.is_cable_in = 1;
		break;
	case CONNECT_TYPE_UNKNOWN:
		BATT_ERR("unknown cable");
		htc_batt_info.rep.charging_source = CHARGER_USB;
		htc_batt_info.is_cable_in = 1;
		break;
	case CONNECT_TYPE_INTERNAL:
		BATT_LOG("delivers power to VBUS from battery");
		htc_battery_set_charging(POWER_SUPPLY_ENABLE_INTERNAL);
		mutex_unlock(&htc_batt_info.info_lock);
		return;
	case CONNECT_TYPE_NONE:
		if ( !!(get_kernel_flag() & WRITE_PWR_SAVE_DISABLE) ) {
			htc_batt_info.is_cable_in = 0;
			mutex_unlock(&htc_batt_info.info_lock);
			kernel_power_off();
			break;
		}
	/* if not writing kernel flag for usb plug-out , handled by default */
	default:
		BATT_LOG("No cable exists");
		htc_batt_info.rep.charging_source = CHARGER_BATTERY;
		htc_batt_info.is_cable_in = 0;
		break;
	}
	is_alarm_not_work = 0;
	htc_batt_timer.alarm_timer_flag =
			(unsigned int)htc_batt_info.rep.charging_source;

	scnprintf(message, 16, "CHG_SOURCE=%d", htc_batt_info.rep.charging_source);

	update_wake_lock(htc_batt_info.rep.charging_source);
	mutex_unlock(&htc_batt_info.info_lock);

	kobject_uevent_env(&htc_batt_info.batt_cable_kobj, KOBJ_CHANGE, envp);
}

static int htc_battery_set_charging(int ctl)
{
	int rc = 0;

	if (htc_batt_info.charger == SWITCH_CHARGER_TPS65200)
		rc = tps_set_charger_ctrl(ctl);

	return rc;
}

static int htc_batt_charger_control(enum charger_control_flag control)
{
	char message[16] = "CHARGERSWITCH=";
	char *envp[] = { message, NULL };

	switch (control) {
	case STOP_CHARGER:
		strncat(message, "0", 1);
		break;
	case ENABLE_CHARGER:
		strncat(message, "1", 1);
		break;
	default:
		return -1;
	};

	kobject_uevent_env(&htc_batt_info.batt_cable_kobj, KOBJ_CHANGE, envp);

	return 0;
}

static void htc_batt_phone_call_notification(int phone_call)
{
	BATT_LOG("%s:Phone call notified with value %d", __func__, phone_call);
	htc_batt_phone_call = phone_call;
}

static void htc_batt_set_full_level(int percent)
{
	char message[16];
	char *envp[] = { message, NULL };

	BATT_LOG("%s: set full level as %d", __func__, percent);

	scnprintf(message, sizeof(message), "FULL_LEVEL=%d", percent);

	kobject_uevent_env(&htc_batt_info.batt_cable_kobj, KOBJ_CHANGE, envp);

	return;
}

static ssize_t htc_battery_show_batt_attr(struct device_attribute *attr,
					char *buf)
{
	int len = 0;

	len += scnprintf(buf + len, PAGE_SIZE - len,
			"%s", htc_batt_info.debug_log);
	return len;
}

static ssize_t htc_battery_show_batt_power_meter(struct device_attribute *attr,
					char *buf)
{
	int len = 0;

	len += scnprintf(buf + len, PAGE_SIZE - len,
			"%s", htc_batt_info.power_meter);
	return len;
}

static int htc_batt_open(struct inode *inode, struct file *filp)
{
	int ret = 0;

	spin_lock(&htc_batt_info.batt_lock);

	if (!htc_batt_info.first_level_ready) {
		ret = -EBUSY;
		goto not_ready;
	}

	if (!htc_batt_info.is_open)
		htc_batt_info.is_open = 1;
	else
		ret = -EBUSY;
not_ready:
	spin_unlock(&htc_batt_info.batt_lock);
	return ret;
}

static int htc_batt_release(struct inode *inode, struct file *filp)
{
	spin_lock(&htc_batt_info.batt_lock);
	htc_batt_info.is_open = 0;
	spin_unlock(&htc_batt_info.batt_lock);

	return 0;
}

static int htc_batt_get_battery_info(struct battery_info_reply *htc_batt_update)
{

	htc_batt_update->batt_vol = htc_batt_info.rep.batt_vol;
	htc_batt_update->batt_id = htc_batt_info.rep.batt_id;
	htc_batt_update->batt_temp = htc_batt_info.rep.batt_temp;
	htc_batt_update->batt_current = htc_batt_info.rep.batt_current;
	htc_batt_update->batt_discharg_current =
				htc_batt_info.rep.batt_discharg_current;
	htc_batt_update->level = htc_batt_info.rep.level;
	htc_batt_update->charging_source =
				htc_batt_info.rep.charging_source;
	htc_batt_update->charging_enabled =
				htc_batt_info.rep.charging_enabled;
	htc_batt_update->full_bat = htc_batt_info.rep.full_bat;
	htc_batt_update->full_level = htc_batt_info.rep.full_level;
	htc_batt_update->over_vchg = htc_batt_info.rep.over_vchg;
	htc_batt_update->temp_fault = htc_batt_info.rep.temp_fault;
	htc_batt_update->batt_state = htc_batt_info.rep.batt_state;
	htc_batt_update->overload = htc_batt_info.rep.overload;

	return 0;
}

static void batt_set_check_timer(u32 seconds)
{

	mod_timer(&htc_batt_timer.batt_timer,
			jiffies + msecs_to_jiffies(seconds * 1000));
}

static int32_t htc_batt_get_battery_adc(void)
{
	int ret = 0;
	struct battery_adc_reply adc;
	int i, temp;

	for (i = 0; i < ADC_REPLY_ARRAY_SIZE; i++) {
		/* Read battery voltage adc data. */
		ret = tps80032_adc_select_and_read(
				adc.adc_voltage + i,
				BATT_VOLT_CHANNEL);
		if (ret)
			goto get_adc_failed;

		/* Read battery current adc data. */
		ret = tps80032_adc_select_and_read(
				adc.adc_current + i,
				BATT_CURR_CHANNEL);
		if (ret)
			goto get_adc_failed;

		/* Read battery vzero clb adc data. */
		if (htc_batt_info.vzero_clb_channel >= 0) {
			ret = tps80032_adc_select_and_read(
					adc.adc_vzero_clb + i,
					htc_batt_info.vzero_clb_channel);
			if (ret)
				goto get_adc_failed;
		} else
			adc.adc_vzero_clb[i] = TPS80032_GPADC_FAKE_VALUE;

		mdelay(1);
	}

	for (i = 0; i < ADC_REPLY_ARRAY_SIZE; i++) {
		/* an adjustment for board no calibratioin */
		temp = ((int) adc.adc_voltage[i]) + htc_batt_info.volt_adc_offset;
		if (temp < 0)
			adc.adc_voltage[i] = 0;
		else if (temp > 4095)
			adc.adc_voltage[i] = 4095;
		else
			adc.adc_voltage[i] = temp;

			/* Read battery temperature adc data. */
			ret = tps80032_adc_select_and_read(
					adc.adc_temperature + i,
					BATT_TEMP_CHANNEL);
			if (ret)
				goto get_adc_failed;
			/* Read battery id adc data. */
			ret = tps80032_adc_select_and_read(
					adc.adc_battid + i,
					BATT_ID_CHANNEL);
			if (ret)
				goto get_adc_failed;
		mdelay(1);
	}

	memcpy(&htc_batt_info.adc_data, &adc,
		sizeof(struct battery_adc_reply));

get_adc_failed:
	return ret;
}

static void batt_regular_timer_handler(unsigned long data)
{
	wake_lock(&htc_batt_timer.battery_lock);
	queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_work);
}

static void batt_first_timer_handler(unsigned long data)
{
	queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_first_work);
}

static void batt_check_alarm_handler(struct alarm *alarm)
{
	BATT_LOG("alarm handler, but do nothing.");
	return;
}

static void batt_work_func(struct work_struct *work)
{
	int rc = 0;
	int notify_cable_gone = 0;
	int has_vbus;
	char total_time[32];
	char battery_alarm[16];
	char *envp[] = { total_time, battery_alarm, NULL };

	rc = htc_batt_get_battery_adc();
	if (rc)
		BATT_ERR("Read ADC failed!");

	/* read tps80032 VBUS_DETECT state */
	has_vbus = tps80032_read_vbus_detection();
	BATT_LOG("tps80032 VBUS_DETECT:%d OPA_MODE:%d BOOST_HW_PWR:%d", has_vbus,
			tps80032_read_opa_mode(),tps80032_read_boots_hw_pwr());

	scnprintf(total_time, sizeof(total_time), "TOTAL_TIME=%lu", htc_batt_timer.total_time_ms);

	scnprintf(battery_alarm, sizeof(battery_alarm), "BATT_ALARM=%u", htc_batt_timer.batt_alarm_status);

	kobject_uevent_env(&htc_batt_info.batt_timer_kobj, KOBJ_CHANGE, envp);
	htc_batt_timer.total_time_ms = 0;
	htc_batt_timer.batt_system_jiffies = jiffies;
	htc_batt_timer.batt_alarm_status = 0;
	batt_set_check_timer(htc_batt_timer.time_out);
	wake_unlock(&htc_batt_timer.battery_lock);

	if (!has_vbus) {
		/* vbus gone, check if need to send no cable event to battery  */
		mutex_lock(&htc_batt_info.info_lock);
		if (htc_batt_info.online != CONNECT_TYPE_NONE)
			notify_cable_gone = 1;
		mutex_unlock(&htc_batt_info.info_lock);

		if (!!notify_cable_gone) {
			BATT_LOG("VBUS is not present, notify cable gone!!");
			usb_status_notifier_func(CONNECT_TYPE_NONE);
		}
	}

	if (is_alarm_not_work == 1) {
		/* force wake lock once when charging*/
		mutex_lock(&htc_batt_info.info_lock);
		if (htc_batt_info.rep.charging_source != CHARGER_BATTERY) {
			is_alarm_not_work = 2;
			wake_lock(&htc_batt_info.vbus_wake_lock);
		} else
			is_alarm_not_work = 0;
		mutex_unlock(&htc_batt_info.info_lock);
	}
	return;
}

extern unsigned (*get_battery_level_cb)(void);
static unsigned tps80032_batt_get_battery_level(void)
{
	if (htc_batt_info.rep.batt_state != 0)
		return htc_batt_info.rep.level;

	return BATTERY_LEVEL_NO_VALUE;
}

static void batt_first_work_func(struct work_struct *work)
{
	int rc;
	unsigned long flags;

	rc = htc_batt_get_battery_adc();
	if (rc)
		return;

	get_battery_level_cb = tps80032_batt_get_battery_level;

	spin_lock_irqsave(&htc_batt_info.batt_lock, flags);
	htc_batt_info.first_level_ready = 1;
	spin_unlock_irqrestore(&htc_batt_info.batt_lock, flags);
}

static void battery_power_story_board(void)
{
	/* do nothing */
}

static unsigned int quickboot_low_power_boot;

static long htc_batt_ioctl(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	wake_lock(&htc_batt_timer.battery_lock);

	switch (cmd) {
	case HTC_BATT_IOCTL_READ_SOURCE: {
		if (copy_to_user((void __user *)arg,
			&htc_batt_info.rep.charging_source, sizeof(u32)))
			ret = -EFAULT;
		break;
	}
	case HTC_BATT_IOCTL_SET_BATT_ALARM: {
		u32 time_out = 0;
		if (copy_from_user(&time_out, (void *)arg, sizeof(u32))) {
			ret = -EFAULT;
			break;
		}

		htc_batt_timer.time_out = time_out;
		if (!htc_battery_initial) {
			htc_battery_initial = 1;
			batt_set_check_timer(htc_batt_timer.time_out);
		}
		break;
	}
	case HTC_BATT_IOCTL_GET_ADC_VREF: {
		if (copy_to_user((void __user *)arg, &htc_batt_info.adc_vref,
				sizeof(htc_batt_info.adc_vref))) {
			BATT_ERR("copy_to_user failed!");
			ret = -EFAULT;
		}
		break;
	}
	case HTC_BATT_IOCTL_GET_ADC_ALL: {
		if (copy_to_user((void __user *)arg, &htc_batt_info.adc_data,
					sizeof(struct battery_adc_reply))) {
			BATT_ERR("copy_to_user failed!");
			ret = -EFAULT;
		}
		break;
	}
	case HTC_BATT_IOCTL_CHARGER_CONTROL: {
		u32 charger_mode = 0;
		if (copy_from_user(&charger_mode, (void *)arg, sizeof(u32))) {
			BATT_ERR("copy_from_user failed!");
			ret = -EFAULT;
			break;
		}
		BATT_LOG("do charger control = %u", charger_mode);
		htc_battery_set_charging(charger_mode);
		break;
	}
	case HTC_BATT_IOCTL_UPDATE_BATT_INFO: {
		mutex_lock(&htc_batt_info.info_lock);
		if (copy_from_user(&htc_batt_info.rep, (void *)arg,
					sizeof(struct battery_info_reply))) {
			BATT_ERR("copy_from_user failed!");
			ret = -EFAULT;
			mutex_unlock(&htc_batt_info.info_lock);
			break;
		}

		if (htc_batt_info.rep.batt_temp > 150) {
			if (htc_batt_info.rep.batt_vol <= 2700) {
				BATT_LOG("Normal temperature, critical shutdown, level 0");
				htc_batt_info.rep.charging_source = CHARGER_BATTERY;
				htc_batt_info.rep.level = 0;
			}
		}

		if (quickboot_low_power_boot && htc_batt_info.rep.level >= 4)
			quickboot_low_power_boot = 0;

		if (quickboot_low_power_boot &&
			htc_batt_info.rep.batt_vol <= QB_LPB_SHUTDOWN_VOLTAGE) {
			BATT_LOG("QuickBoot once, and voltage lower than %d.  "
				"Shutdown System\n", QB_LPB_SHUTDOWN_VOLTAGE);
			mutex_unlock(&htc_batt_info.info_lock);
			kernel_power_off();
		}

		battery_power_story_board();

		mutex_unlock(&htc_batt_info.info_lock);

		htc_battery_core_update();
		break;
	}
	case HTC_BATT_IOCTL_BATT_DEBUG_LOG:
		if (copy_from_user(htc_batt_info.debug_log, (void *)arg,
					DEBUG_LOG_LENGTH)) {
			BATT_ERR("copy debug log from user failed!");
			ret = -EFAULT;
		}
		break;
	case HTC_BATT_IOCTL_SET_VOLTAGE_ALARM: {
		break;
	}
	case HTC_BATT_IOCTL_SET_ALARM_TIMER_FLAG: {
		/* alarm flag could be reset by cable. */
		unsigned int flag;
		if (copy_from_user(&flag, (void *)arg, sizeof(unsigned int))) {
			BATT_ERR("Set timer type into alarm failed!");
			ret = -EFAULT;
			break;
		}
		htc_batt_timer.alarm_timer_flag = flag;
		BATT_LOG("Set alarm timer flag:%u", flag);
		break;
	}
	case HTC_BATT_IOCTL_BATT_POWER_METER:
		if (copy_from_user(htc_batt_info.power_meter, (void *)arg,
					POWER_METER_LENGTH)) {
			BATT_ERR("copy power meter from user failed!");
			ret = -EFAULT;
		}
		break;
	default:
		BATT_ERR("%s: no matched ioctl cmd", __func__);
		break;
	}

	wake_unlock(&htc_batt_timer.battery_lock);

	return ret;
}

/*  MBAT_IN interrupt handler	*/
static void mbat_in_func(struct work_struct *work)
{
	int is_power_off;

	mutex_lock(&htc_batt_info.info_lock);
	is_power_off = htc_batt_info.power_off_by_id;
	mutex_unlock(&htc_batt_info.info_lock);
	if (is_power_off) {
		BATT_LOG("shut down device due to MBAT_IN interrupt");
		htc_battery_set_charging(0);
		kernel_power_off();
	}
}

static irqreturn_t mbat_int_handler(int irq, void *data)
{
	struct htc_battery_platform_data *pdata = data;

	disable_irq_nosync(pdata->gpio_mbat_in);

	schedule_delayed_work(&mbat_in_struct, msecs_to_jiffies(50)); 
	return IRQ_HANDLED;
}
/*  MBAT_IN interrupt handler end   */

const struct file_operations htc_batt_fops = {
	.owner = THIS_MODULE,
	.open = htc_batt_open,
	.release = htc_batt_release,
	.unlocked_ioctl = htc_batt_ioctl,
};

static struct miscdevice htc_batt_device_node = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc_batt",
	.fops = &htc_batt_fops,
};

static void htc_batt_kobject_release(struct kobject *kobj)
{
	BATT_ERR("htc_batt_kobject_release.\n");
	return;
}

static struct kobj_type htc_batt_ktype = {
	.release = htc_batt_kobject_release,
};

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void htc_battery_late_resume(struct early_suspend *h)
{
	htc_batt_timer.total_time_ms += (jiffies -
			htc_batt_timer.batt_system_jiffies) * MSEC_PER_SEC / HZ;
	htc_batt_timer.batt_system_jiffies = jiffies;

	if (htc_batt_timer.total_time_ms >= BATT_LATE_RESUME_CHECK_TIME * MSEC_PER_SEC) {
		BATT_LOG("late resume with check time up, update battery level");
		del_timer_sync(&htc_batt_timer.batt_timer);
		cancel_work_sync(&htc_batt_timer.batt_work);
		wake_lock(&htc_batt_timer.battery_lock);
		queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_work);
	}
}
#endif

static unsigned long target_interval_ms = 0;
static int htc_battery_prepare(struct device *dev)
{
	int time_diff;
	int time_out;
	struct timespec xtime;
	ktime_t interval;
	ktime_t slack = ktime_set(0, 0);
	ktime_t next_alarm;

	target_interval_ms = 0;

	del_timer_sync(&htc_batt_timer.batt_timer);
	cancel_work_sync(&htc_batt_timer.batt_work);

	htc_batt_timer.total_time_ms += (jiffies -
			htc_batt_timer.batt_system_jiffies) * MSEC_PER_SEC / HZ;
	htc_batt_timer.batt_system_jiffies = jiffies;
	getnstimeofday(&xtime);
	htc_batt_timer.batt_suspend_ms = xtime.tv_sec * MSEC_PER_SEC + xtime.tv_nsec / NSEC_PER_MSEC;

	if (htc_batt_timer.alarm_timer_flag)
		time_out = htc_batt_timer.time_out;
	else if (htc_batt_phone_call)
		time_out = BATT_SUSPEND_PHONE_CALL_CHECK_TIME;
	else
		time_out = BATT_SUSPEND_CHECK_TIME;

	time_diff = time_out * MSEC_PER_SEC - htc_batt_timer.total_time_ms;

	if (time_diff <= 5 * MSEC_PER_SEC) {
		wake_lock(&htc_batt_timer.battery_lock);
		queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_work);
		return -EBUSY;
	} else if (htc_batt_timer.alarm_timer_flag)
		batt_set_check_timer((time_diff / MSEC_PER_SEC) + 1);
	else
		batt_set_check_timer(htc_batt_timer.time_out);

	interval = ktime_set(time_diff / MSEC_PER_SEC,
			(time_diff % MSEC_PER_SEC) * NSEC_PER_MSEC);

	xtime = ktime_to_timespec(interval);
	BATT_LOG("%s: passing time:%lu, status:%u%u, "
		"alarm will be triggered after %ld.%ld seconds",
		__func__, htc_batt_timer.total_time_ms,
		htc_batt_phone_call,
		htc_batt_timer.alarm_timer_flag,
		xtime.tv_sec, xtime.tv_nsec);
	next_alarm = ktime_add(alarm_get_elapsed_realtime(), interval);
	alarm_start_range(&htc_batt_timer.batt_check_wakeup_alarm,
				next_alarm, ktime_add(next_alarm, slack));
	target_interval_ms = xtime.tv_sec * MSEC_PER_SEC;
	return 0;
}

static void htc_battery_complete(struct device *dev)
{
	unsigned long resume_ms;
	unsigned long check_time;
	struct timespec xtime;

	del_timer_sync(&htc_batt_timer.batt_timer);
	cancel_work_sync(&htc_batt_timer.batt_work);

	getnstimeofday(&xtime);
	resume_ms = xtime.tv_sec * MSEC_PER_SEC + xtime.tv_nsec / NSEC_PER_MSEC;
	htc_batt_timer.total_time_ms += resume_ms -
					htc_batt_timer.batt_suspend_ms;
	htc_batt_timer.batt_system_jiffies = jiffies;

	BATT_LOG("%s: total suspend time:%lu, the total passing time:%lu",
			__func__, (resume_ms - htc_batt_timer.batt_suspend_ms),
			htc_batt_timer.total_time_ms);

	/* XXX We found rtc-alarm will have chance not work.  If rtc-alarm not
	 * work, the capacity accumulate will have error, and other security
	 * effect.  This check will dump RTC register in kernel.  And also set
	 * flag to keep device not to suspend.
	 * If suspend time over target interval 20sec twice, set the flag.
	 * Clean when cable status change.
	 * However if resume take too long, the workaround may also happened.
	 */
	if (!is_alarm_not_work &&
		target_interval_ms != 0 &&
		target_interval_ms + 20000 <
			(resume_ms - htc_batt_timer.batt_suspend_ms)) {
		pr_info("[BATT] Alarm not work!!\n");
		is_alarm_not_work = 1;
	}

	if (htc_batt_timer.alarm_timer_flag)
		/* 500 msecs check buffer time */
		check_time = (htc_batt_timer.time_out * MSEC_PER_SEC)
				- (MSEC_PER_SEC / 2);
	else if (htc_batt_phone_call)
		check_time = BATT_SUSPEND_PHONE_CALL_CHECK_TIME * MSEC_PER_SEC;
	else
		check_time = BATT_SUSPEND_CHECK_TIME * MSEC_PER_SEC;

	/*  - When kernel resumes, battery driver should check total time to decide if do battery algorithm or just ignore.
	    - If kernel resumes due to battery voltage alarm, do battery algorithm forcibly. */
	if ((htc_batt_timer.total_time_ms >= check_time) ||
	    (!!htc_batt_timer.batt_alarm_status)) {
		wake_lock(&htc_batt_timer.battery_lock);
		queue_work(htc_batt_timer.batt_wq, &htc_batt_timer.batt_work);
	} else if (htc_batt_timer.alarm_timer_flag)
		batt_set_check_timer(((check_time - htc_batt_timer.total_time_ms) / MSEC_PER_SEC) + 1);
	else
		batt_set_check_timer(htc_batt_timer.time_out);
}

static struct dev_pm_ops htc_battery_tps80032_pm_ops = {
	.prepare = htc_battery_prepare,
	.complete = htc_battery_complete,
};

extern int (*tps80031_is_cable_in)(void);
static int is_cable_in(void)
{
	int value;
	mutex_lock(&htc_batt_info.info_lock);
	value = htc_batt_info.is_cable_in;
	mutex_unlock(&htc_batt_info.info_lock);
	return value;
}

static struct device_attribute tps80032_batt_attrs[] = {
	__ATTR(reboot_level, S_IRUGO, tps80032_first_batt_show_attributes, NULL),
	__ATTR(quickboot_low_power_boot, S_IWUSR, NULL, tps80032_qb_store_attributes),
	};

static ssize_t tps80032_first_batt_show_attributes(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	int i = 0;
	const ptrdiff_t off = attr - tps80032_batt_attrs;

	if (off < sizeof(tps80032_batt_attrs)) {
		if (off == ATTR_REBOOT_LEVEL)
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
				get_reboot_battery_level());
		else
			i = -EINVAL;
	} else
		i = -EINVAL;

	if (i < 0)
		pr_err("%s: attribute %d is not supported\n",
			__func__, i);

	return i;
}

static int tps80032_qb_store_attributes(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	quickboot_low_power_boot = sysfs_streq(buf, "0") ? 0 : 1;
	pr_info("[BATT] set quickboot_low_power_boot %d", quickboot_low_power_boot);
	return size;
}

static int tps80032_batt_create_attrs(struct device *dev)
{
	int i = 0, rc = 0;

	for (i = 0; i < ARRAY_SIZE(tps80032_batt_attrs); i++) {
		rc = device_create_file(dev, &tps80032_batt_attrs[i]);
		if (rc)
			goto batt_attrs_failed;
	}

	goto succeed;

batt_attrs_failed:
	while (i--)
		device_remove_file(dev, &tps80032_batt_attrs[i]);
succeed:
	return rc;
}

static int htc_battery_probe(struct platform_device *pdev)
{
	int i, rc = 0;
	struct htc_battery_platform_data *pdata = pdev->dev.platform_data;
	struct htc_battery_core *htc_battery_core_ptr;

	htc_battery_core_ptr = kmalloc(sizeof(struct htc_battery_core),
					GFP_KERNEL);
	if (!htc_battery_core_ptr) {
		BATT_ERR("%s: kmalloc failed for htc_battery_core_ptr.",
				__func__);
		return -ENOMEM;
	}

	mutex_lock(&htc_batt_info.info_lock);
	htc_batt_info.power_off_by_id = pdata->power_off_by_id;
	mutex_unlock(&htc_batt_info.info_lock);

	INIT_DELAYED_WORK(&mbat_in_struct, mbat_in_func);
	if (pdata->gpio_mbat_in_trigger_level == MBAT_IN_HIGH_TRIGGER)
		rc = request_irq(pdata->gpio_mbat_in,
				mbat_int_handler, IRQF_TRIGGER_HIGH,
				"mbat_in", pdata);
	else if (pdata->gpio_mbat_in_trigger_level == MBAT_IN_LOW_TRIGGER)
		rc = request_irq(pdata->gpio_mbat_in,
				mbat_int_handler, IRQF_TRIGGER_LOW,
				"mbat_in", pdata);
	if (rc)
		BATT_ERR("request mbat_in irq failed!");
	else
		irq_set_irq_wake(pdata->gpio_mbat_in, 1);

	htc_battery_core_ptr->func_show_batt_attr = htc_battery_show_batt_attr;
	htc_battery_core_ptr->func_show_batt_power_meter = htc_battery_show_batt_power_meter;
	htc_battery_core_ptr->func_get_battery_info = htc_batt_get_battery_info;
	htc_battery_core_ptr->func_charger_control = htc_batt_charger_control;
	htc_battery_core_ptr->func_set_full_level = htc_batt_set_full_level;
	htc_battery_core_ptr->func_phone_call_notification = htc_batt_phone_call_notification;
	htc_battery_core_register(&pdev->dev, htc_battery_core_ptr);

	htc_batt_info.device_id = pdev->id;
	htc_batt_info.guage_driver = pdata->guage_driver;
	htc_batt_info.charger = pdata->charger;
	htc_batt_info.vzero_clb_channel = pdata->vzero_clb_channel;
	htc_batt_info.volt_adc_offset = pdata->volt_adc_offset;
	htc_batt_info.check2_value = 0;
	htc_batt_info.first_level_ready = 0;

	htc_batt_info.rep.full_level = 100;

	htc_batt_info.is_open = 0;

	for (i = 0; i < ADC_REPLY_ARRAY_SIZE; i++)
		htc_batt_info.adc_vref[i] = (1 << 12) - 1;

	htc_batt_info.is_cable_in = 0;
	tps80031_is_cable_in = is_cable_in;

	htc_batt_info.online = -1;

	INIT_WORK(&htc_batt_timer.batt_work, batt_work_func);
	init_timer(&htc_batt_timer.batt_timer);
	htc_batt_timer.batt_timer.function = batt_regular_timer_handler;
	alarm_init(&htc_batt_timer.batt_check_wakeup_alarm,
			ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			batt_check_alarm_handler);
	INIT_WORK(&htc_batt_timer.batt_first_work, batt_first_work_func);
	init_timer(&htc_batt_timer.batt_first_timer);
	htc_batt_timer.batt_first_timer.function = batt_first_timer_handler;
	htc_batt_timer.batt_wq = create_singlethread_workqueue("batt_timer");

	rc = misc_register(&htc_batt_device_node);
	if (rc) {
		BATT_ERR("Unable to register misc device %d",
			MISC_DYNAMIC_MINOR);
		goto fail;
	}

	htc_batt_kset = kset_create_and_add("event_to_daemon", NULL,
			kobject_get(&htc_batt_device_node.this_device->kobj));
	if (!htc_batt_kset) {
		rc = -ENOMEM;
		goto fail;
	}

	htc_batt_info.batt_timer_kobj.kset = htc_batt_kset;
	rc = kobject_init_and_add(&htc_batt_info.batt_timer_kobj,
				&htc_batt_ktype, NULL, "htc_batt_timer");
	if (rc) {
		BATT_ERR("init kobject htc_batt_timer failed.");
		kobject_put(&htc_batt_info.batt_timer_kobj);
		goto fail;
	}

	htc_batt_info.batt_cable_kobj.kset = htc_batt_kset;
	rc = kobject_init_and_add(&htc_batt_info.batt_cable_kobj,
				&htc_batt_ktype, NULL, "htc_cable_detect");
	if (rc) {
		BATT_ERR("init kobject htc_cable_timer failed.");
		kobject_put(&htc_batt_info.batt_timer_kobj);
		goto fail;
	}

	if (pdata->charger == SWITCH_CHARGER_TPS65200)
		tps_register_notifier(&tps_int_notifier);

	tps80032_vsys_alarm_register_notifier(&battery_alarm_notifier);

#ifdef CONFIG_HAS_EARLYSUSPEND
	htc_batt_info.early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING;
	htc_batt_info.early_suspend.resume = htc_battery_late_resume;
	register_early_suspend(&htc_batt_info.early_suspend);
#endif

	tps80032_batt_create_attrs(&pdev->dev);

	mod_timer(&htc_batt_timer.batt_first_timer,
			jiffies + msecs_to_jiffies(FIRST_ADC_READ_DELAY * 1000));

	BATT_LOG("htc_battery_probe(): finish");

fail:
	kfree(htc_battery_core_ptr);
	return rc;
}

static struct platform_driver htc_battery_driver = {
	.probe	= htc_battery_probe,
	.driver	= {
		.name	= "htc_battery",
		.owner	= THIS_MODULE,
		.pm = &htc_battery_tps80032_pm_ops,
	},
};

static int __init htc_battery_init(void)
{

	htc_batt_phone_call = 0;
	htc_battery_initial = 0;
	htc_full_level_flag = 0;
	spin_lock_init(&htc_batt_info.batt_lock);
	wake_lock_init(&htc_batt_info.vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_present");
	wake_lock_init(&htc_batt_timer.battery_lock, WAKE_LOCK_SUSPEND, "htc_battery_tps80032");
	mutex_init(&htc_batt_info.info_lock);

	usb_register_notifier(&usb_status_notifier);
	platform_driver_register(&htc_battery_driver);

	/* init battery parameters. */
	htc_batt_info.rep.batt_vol = 3300;
	htc_batt_info.rep.batt_id = 1;
	htc_batt_info.rep.batt_temp = 300;
	htc_batt_info.rep.level = 10;
	htc_batt_info.rep.full_bat = 1580000;
	htc_batt_info.rep.full_level = 100;
	htc_batt_info.rep.batt_state = 0;
	htc_batt_info.rep.temp_fault = -1;
	htc_batt_info.rep.overload = 0;
	htc_batt_timer.total_time_ms = 0;
	htc_batt_timer.batt_system_jiffies = jiffies;
	htc_batt_timer.batt_alarm_status = 0;
	htc_batt_timer.alarm_timer_flag = 0;

	return 0;
}

module_init(htc_battery_init);
MODULE_DESCRIPTION("HTC Battery Driver");
MODULE_LICENSE("GPL");
