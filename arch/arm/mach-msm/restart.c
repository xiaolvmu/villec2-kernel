/* Copyright (c) 2010-2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/pm.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/pmic8901.h>
#include <linux/mfd/pm8xxx/misc.h>
#include <linux/gpio.h>
#include <linux/console.h>

#include <asm/cacheflush.h>
#include <asm/mach-types.h>

#include <mach/msm_iomap.h>
#include <mach/restart.h>
#include <mach/socinfo.h>
#include <mach/irqs.h>
#include <mach/scm.h>
#include <mach/msm_watchdog.h>
#include <mach/board_htc.h>

#include "smd_private.h"
#include "timer.h"
#include <mach/htc_restart_handler.h>

#define WDT0_RST	0x38
#define WDT0_EN		0x40
#define WDT0_BARK_TIME	0x4C
#define WDT0_BITE_TIME	0x5C

#define PSHOLD_CTL_SU (MSM_TLMM_BASE + 0x820)

#define RESTART_REASON_ADDR 0xF00
#define DLOAD_MODE_ADDR     0x0

#define SCM_IO_DISABLE_PMIC_ARBITER	1

static int restart_mode;
int ramdump_source=0;

int pmic_reset_irq;
static void __iomem *msm_tmr0_base;

static int notify_efs_sync = 0;
static int notify_efs_sync_set(const char *val, struct kernel_param *kp);
module_param_call(notify_efs_sync, notify_efs_sync_set, param_get_int,
		&notify_efs_sync, 0644);

static void check_efs_sync_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(checkwork_struct, check_efs_sync_work);

static int in_panic;
static int panic_prep_restart(struct notifier_block *this,
			      unsigned long event, void *ptr)
{
	in_panic = 1;
	return NOTIFY_DONE;
}
static struct notifier_block panic_blk = {
	.notifier_call	= panic_prep_restart,
};
#ifdef CONFIG_MSM_DLOAD_MODE
static void *dload_mode_addr;

static int dload_set(const char *val, struct kernel_param *kp);
static int download_mode = 1;
module_param_call(download_mode, dload_set, param_get_int,
			&download_mode, 0644);

static void set_dload_mode(int on)
{
	if (dload_mode_addr) {
		__raw_writel(on ? 0xE47B337D : 0, dload_mode_addr);
		__raw_writel(on ? 0xCE14091A : 0,
		       dload_mode_addr + sizeof(unsigned int));
		mb();
	}
}

static int dload_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = download_mode;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	
	if (download_mode >> 1) {
		download_mode = old_val;
		return -EINVAL;
	}

	set_dload_mode(download_mode);

	return 0;
}
#else
#define set_dload_mode(x) do {} while (0)
#endif

void msm_set_restart_mode(int mode)
{
	restart_mode = mode;
}
EXPORT_SYMBOL(msm_set_restart_mode);

static void msm_flush_console(void)
{
	unsigned long flags;

	printk("\n");
	printk(KERN_EMERG "[K] Restarting %s\n", linux_banner);
	if (console_trylock()) {
		console_unlock();
		return;
	}

	mdelay(50);

	local_irq_save(flags);

	if (console_trylock())
		printk(KERN_EMERG "[K] restart: Console was locked! Busting\n");
	else
		printk(KERN_EMERG "[K] restart: Console was locked!\n");
	console_unlock();

	local_irq_restore(flags);
}
static void set_modem_efs_sync(void)
{
	smsm_change_state(SMSM_APPS_STATE, SMSM_APPS_REBOOT, SMSM_APPS_REBOOT);
	printk(KERN_INFO "[K] %s: wait for modem efs_sync\n", __func__);
}

static int check_modem_efs_sync(void)
{
	return (smsm_get_state(SMSM_MODEM_STATE) & SMSM_SYSTEM_PWRDWN_USR);
}

static int efs_sync_work_timout;

static void check_efs_sync_work(struct work_struct *work)
{
	if (--efs_sync_work_timout > 0 && !check_modem_efs_sync()) {
		schedule_delayed_work(&checkwork_struct, msecs_to_jiffies(1000));
	} else {
		notify_efs_sync = 0;
		if (efs_sync_work_timout <= 0)
			pr_notice("%s: modem efs_sync timeout.\n", __func__);
		else
			pr_info("%s: modem efs_sync done.\n", __func__);
	}
}

static void notify_modem_efs_sync_schedule(unsigned timeout)
{
	efs_sync_work_timout = timeout;
	set_modem_efs_sync();
	schedule_delayed_work(&checkwork_struct, msecs_to_jiffies(1000));
}

static void check_modem_efs_sync_timeout(unsigned timeout)
{
	while (timeout > 0 && !check_modem_efs_sync()) {
		writel(1, msm_tmr0_base + WDT0_RST);
		mdelay(1000);
		timeout--;
	}
	if (timeout <= 0)
		pr_notice("%s: modem efs_sync timeout.\n", __func__);
	else
		pr_info("%s: modem efs_sync done.\n", __func__);
}

static int notify_efs_sync_set(const char *val, struct kernel_param *kp)
{
	int ret;
	int old_val = notify_efs_sync;

	
	if (old_val == 1)
		return -EINVAL;

	ret = param_set_int(val, kp);

	if (ret)
		return ret;

	
	if (notify_efs_sync != 1) {
		notify_efs_sync = 0;
		return -EINVAL;
	}

	notify_modem_efs_sync_schedule(10);

	return 0;
}

static int notify_efs_sync_call
	(struct notifier_block *this, unsigned long code, void *_cmd)
{
	unsigned long oem_code = 0;

	switch (code) {
	case SYS_RESTART:
		if (_cmd && !strncmp(_cmd, "oem-", 4)) {
			oem_code = simple_strtoul(_cmd + 4, 0, 16) & 0xff;
		}

		
		if (board_mfg_mode() <= 2) {
			
			if (oem_code != 0x11) {
				set_modem_efs_sync();
				check_modem_efs_sync_timeout(10);
			}
		}
		

	case SYS_POWER_OFF:
		
		if (notify_efs_sync) {
			
			pr_info("%s: userspace initiated efs_sync not finished...\n", __func__);
			cancel_delayed_work(&checkwork_struct);
			
			check_modem_efs_sync_timeout(efs_sync_work_timout - 1);
		}
		break;
	}

	return NOTIFY_DONE;
}

static struct notifier_block notify_efs_sync_notifier = {
	.notifier_call = notify_efs_sync_call,
};

static void __msm_power_off(int lower_pshold)
{
	printk(KERN_CRIT "[K] Powering off the SoC\n");
#ifdef CONFIG_MSM_DLOAD_MODE
	set_dload_mode(0);
#endif
	pm8xxx_reset_pwr_off(0);

	if (lower_pshold) {
		__raw_writel(0, PSHOLD_CTL_SU);
		mdelay(10000);
		printk(KERN_ERR "[K] Powering off has failed\n");
	}
	return;
}

static void msm_power_off(void)
{
	
	__msm_power_off(1);
}

static void cpu_power_off(void *data)
{
	int rc;

	pr_err("PMIC Initiated shutdown %s cpu=%d\n", __func__,
						smp_processor_id());
	if (smp_processor_id() == 0) {
		__msm_power_off(0);

		pet_watchdog();
		pr_err("Calling scm to disable arbiter\n");
		
		rc = scm_call_atomic1(SCM_SVC_PWR,
						SCM_IO_DISABLE_PMIC_ARBITER, 1);

		pr_err("SCM returned even when asked to busy loop rc=%d\n", rc);
		pr_err("waiting on pmic to shut msm down\n");
	}

	preempt_disable();
	while (1)
		;
}

static irqreturn_t resout_irq_handler(int irq, void *dev_id)
{
	pr_warn("%s PMIC Initiated shutdown\n", __func__);
	oops_in_progress = 1;
	smp_call_function_many(cpu_online_mask, cpu_power_off, NULL, 0);
	if (smp_processor_id() == 0)
		cpu_power_off(NULL);
	preempt_disable();
	while (1)
		;
	return IRQ_HANDLED;
}

void msm_restart(char mode, const char *cmd)
{
	unsigned long oem_code = 0;

#ifdef CONFIG_MSM_DLOAD_MODE

	
	set_dload_mode(0);

	
	set_dload_mode(in_panic);

	
	if (restart_mode == RESTART_DLOAD)
		set_dload_mode(1);

	
	if (!download_mode)
		set_dload_mode(0);
#endif

	printk(KERN_NOTICE "[K] Going down for restart now\n");

	printk(KERN_NOTICE "%s: Kernel command line: %s\n", __func__, saved_command_line);

	pm8xxx_reset_pwr_off(1);

	pr_info("[K] %s: restart by command: [%s]\r\n", __func__, (cmd) ? cmd : "");

	if (in_panic) {
		
	} else if (!cmd) {
		set_restart_action(RESTART_REASON_REBOOT, NULL);
	} else if (!strncmp(cmd, "bootloader", 10)) {
		set_restart_action(RESTART_REASON_BOOTLOADER, NULL);
	} else if (!strncmp(cmd, "recovery", 8)) {
		set_restart_action(RESTART_REASON_RECOVERY, NULL);
    } else if (!strcmp(cmd, "eraseflash")) {
        set_restart_action(RESTART_REASON_ERASE_FLASH, NULL);
	} else if (!strncmp(cmd, "oem-", 4)) {
		oem_code = simple_strtoul(cmd + 4, NULL, 16) & 0xff;
		set_restart_to_oem(oem_code, NULL);
	} else if (!strcmp(cmd, "force-hard") ||
			(RESTART_MODE_LEGECY < mode && mode < RESTART_MODE_MAX)
			) {
		
		if (mode == RESTART_MODE_MODEM_USER_INVOKED)
			set_restart_action(RESTART_REASON_REBOOT, NULL);
		else if (mode == RESTART_MODE_ERASE_EFS)
			set_restart_action(RESTART_REASON_ERASE_EFS, NULL);
		else {
			set_restart_action(RESTART_REASON_RAMDUMP, cmd);
		}

	} else {
		set_restart_action(RESTART_REASON_REBOOT, NULL);
	}

	msm_flush_console();
	flush_cache_all();

	__raw_writel(0, msm_tmr0_base + WDT0_EN);
	if (!(machine_is_msm8x60_fusion() || machine_is_msm8x60_fusn_ffa())) {
		mb();
		__raw_writel(0, PSHOLD_CTL_SU); 
		mdelay(5000);
		pr_notice("[K] PS_HOLD didn't work, falling back to watchdog\n");
	}

	pr_info("[K] %s: Restarting by watchdog\r\n", __func__);

	__raw_writel(1, msm_tmr0_base + WDT0_RST);
	__raw_writel(5*0x31F3, msm_tmr0_base + WDT0_BARK_TIME);
	__raw_writel(0x31F3, msm_tmr0_base + WDT0_BITE_TIME);
	__raw_writel(1, msm_tmr0_base + WDT0_EN);

	mdelay(10000);
	printk(KERN_ERR "[K] Restarting has failed\n");
}

static int __init msm_restart_init(void)
{
	int rc;

	htc_restart_handler_init();

	atomic_notifier_chain_register(&panic_notifier_list, &panic_blk);
	register_reboot_notifier(&notify_efs_sync_notifier);

#ifdef CONFIG_MSM_DLOAD_MODE
	dload_mode_addr = MSM_IMEM_BASE + DLOAD_MODE_ADDR;

	
	set_dload_mode(1);
#endif
	msm_tmr0_base = msm_timer_get_timer0_base();
	pm_power_off = msm_power_off;

	if (pmic_reset_irq != 0) {
		rc = request_any_context_irq(pmic_reset_irq,
					resout_irq_handler, IRQF_TRIGGER_HIGH,
					"restart_from_pmic", NULL);
		if (rc < 0)
			pr_err("pmic restart irq fail rc = %d\n", rc);
	} else {
		pr_warn("no pmic restart interrupt specified\n");
	}

	return 0;
}

late_initcall(msm_restart_init);
