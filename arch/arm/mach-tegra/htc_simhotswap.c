/*
 * arch/arm/mach-tegra/htc_simhotswap.c
 *
 * Copyright (C) 2011 NVIDIA Corporation
 * Copyright (C) 2011 HTC Corporation
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
#include <linux/miscdevice.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <mach/board_htc.h>

#include "board-endeavoru.h"
#include "gpio-names.h"
#include "devices.h"
#include "board.h"

#define SIM_DETECT_LOW_ACTIVE TEGRA_GPIO_PI5
#define SIM_DETECT SIM_DETECT_LOW_ACTIVE

static int oldStatus=-1, newStatus=-1;//-1=init 0=insert 1=remove

static int htc_hotswap_open( struct inode * inode, struct file * file )
{
	return 0;
}

const struct file_operations htc_hotswap_fops = {
	.open = htc_hotswap_open,
};

static struct miscdevice sim_hotswap_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "htc_simhotswap",
	.fops = &htc_hotswap_fops,
};

struct htc_simhotswap_info {
	struct kobject simhotswap_kobj;
	struct delayed_work hotswap_work;
	struct workqueue_struct *hotswap_wq;
	struct mutex lock;
};

static struct htc_simhotswap_info htc_hotswap_info;
static struct kset *htc_hotswap_kset;

static void htc_simhotswap_kobject_release(struct kobject *kobj)
{
	printk(KERN_ERR "htc_hotswap_kobject_release.\n");
	return;
}

static struct kobj_type htc_hotswap_ktype = {
	.release = htc_simhotswap_kobject_release,
};

static void hotswap_work_func(struct work_struct *work)
{
	int status;
	char message[20] = "SIMHOTSWAP=";
	char *envp[] = { message, NULL };

	mutex_lock(&htc_hotswap_info.lock);
	status = gpio_get_value(SIM_DETECT);
	if (status){
		strncat(message, "REMOVE", 6);	
		newStatus=status;
		}else{
		strncat(message, "INSERT", 6);
		newStatus=status;
		}
		pr_info("SIM_STATUS = %d oldStatus=%d --> newStatus=%d\n", status,oldStatus,newStatus);
	if(oldStatus!=newStatus){
		kobject_uevent_env(&htc_hotswap_info.simhotswap_kobj, KOBJ_CHANGE, envp);
		pr_info("SIM_DETECT %s\n",status==1?"REMOVE":"INSERT");
		oldStatus=newStatus;
	}
	mutex_unlock(&htc_hotswap_info.lock);

	return;
}

static irqreturn_t sim_detect_irq(int irq, void *dev_id)
{
	schedule_delayed_work(&htc_hotswap_info.hotswap_work, msecs_to_jiffies(90));
	return IRQ_HANDLED;
}

static int htc_simhotswap_probe(struct platform_device *pdev)
{
	int ret = 0;
	printk(KERN_INFO"delay work version %s\n",__func__);

	mutex_init(&htc_hotswap_info.lock);

	INIT_DELAYED_WORK(&htc_hotswap_info.hotswap_work, hotswap_work_func);
	htc_hotswap_info.hotswap_wq = create_singlethread_workqueue("htc_simhotswap");

	ret = misc_register(&sim_hotswap_misc);
	if (ret) {
		pr_err("failed to register misc device!\n");
		goto fail;
	}

	htc_hotswap_kset = kset_create_and_add("event", NULL,
			kobject_get(&sim_hotswap_misc.this_device->kobj));
	if (!htc_hotswap_kset) {
		ret = -ENOMEM;
		goto fail;
	}

	htc_hotswap_info.simhotswap_kobj.kset = htc_hotswap_kset;

	ret = kobject_init_and_add(&htc_hotswap_info.simhotswap_kobj,
			&htc_hotswap_ktype, NULL, "simhotswap");
	if (ret) {
		kobject_put(&htc_hotswap_info.simhotswap_kobj);
		goto fail;
	}	
	oldStatus = gpio_get_value(SIM_DETECT);
	pr_info("htc_simhotswap_probe(): finish SIM init status=%d\n",oldStatus);

	ret = request_irq(gpio_to_irq(SIM_DETECT),
			sim_detect_irq,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"sim_detect", NULL);
	if (ret) {
		pr_err("%s:Failed to request irq, ret=%d\n", __func__, ret);
	}

fail:
	return ret;
}

static struct platform_driver htc_simhotswap_driver = {
	.probe	= htc_simhotswap_probe,
	.driver	= {
		.name	= "htc_simhotswap",
	},
};

static int __init sim_hotswap_init(void)
{
	platform_driver_register(&htc_simhotswap_driver);
	return 0;
}
module_init(sim_hotswap_init);
