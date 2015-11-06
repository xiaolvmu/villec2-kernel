/* linux/kernel/power/htc_pnpmgr.c
 *
 * Copyright (C) 2012 HTC Corporation.
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

#include <linux/module.h>
#include <linux/string.h>
#include <linux/cpu.h>

#include "power.h"

#define MAX_BUF 100
#define MAX_ATTR_LEN 40

static wait_queue_head_t sysfs_state_wq;

struct kobject *cpufreq_kobj;
static struct kobject *hotplug_kobj;
static struct kobject *thermal_kobj;
static struct kobject *apps_kobj;
static struct kobject *pnpmgr_kobj;
static struct kobject *adaptive_policy_kobj;

#define define_string_show(_name, str_buf)				\
static ssize_t _name##_show						\
(struct kobject *kobj, struct kobj_attribute *attr, char *buf)		\
{									\
	return snprintf(buf, strnlen(str_buf, MAX_BUF) + 1, str_buf);	\
}

#define define_string_store(_name, str_buf, store_cb)		\
static ssize_t _name##_store					\
(struct kobject *kobj, struct kobj_attribute *attr,		\
 const char *buf, size_t n)					\
{								\
	strncpy(str_buf, buf, MAX_BUF);				\
	str_buf[MAX_BUF-1] = '\0';				\
	(store_cb)(#_name);					\
	sysfs_notify(kobj, NULL, #_name);			\
	return n;						\
}

#define define_int_show(_name, int_val)				\
static ssize_t _name##_show					\
(struct kobject *kobj, struct kobj_attribute *attr, char *buf)	\
{								\
	return sprintf(buf, "%d", int_val);			\
}

#define define_int_store(_name, int_val, store_cb)		\
static ssize_t _name##_store					\
(struct kobject *kobj, struct kobj_attribute *attr,		\
 const char *buf, size_t n)					\
{								\
	int val;						\
	if (sscanf(buf, "%d", &val) > 0) {			\
		int_val = val;					\
		(store_cb)(#_name);				\
		sysfs_notify(kobj, NULL, #_name);		\
		return n;					\
	}							\
	return -EINVAL;						\
}

static char activity_buf[MAX_BUF];
static char media_mode_buf[MAX_BUF];

static void null_cb(const char *attr) {
	do { } while (0);
}

define_string_show(activity_trigger, activity_buf);
define_string_store(activity_trigger, activity_buf, null_cb);
power_attr(activity_trigger);

define_string_show(media_mode, media_mode_buf);
define_string_store(media_mode, media_mode_buf, null_cb);
power_attr(media_mode);

static int thermal_c0_value;
#if (CONFIG_NR_CPUS >= 2)
static int thermal_c1_value;
#if (CONFIG_NR_CPUS == 4)
static int thermal_c2_value;
static int thermal_c3_value;
#endif
#endif
static int thermal_final_value;
static int thermal_g0_value;
static int thermal_batt_value;

define_int_show(thermal_c0, thermal_c0_value);
define_int_store(thermal_c0, thermal_c0_value, null_cb);
power_attr(thermal_c0);

#if (CONFIG_NR_CPUS >= 2)
define_int_show(thermal_c1, thermal_c1_value);
define_int_store(thermal_c1, thermal_c1_value, null_cb);
power_attr(thermal_c1);
#if (CONFIG_NR_CPUS == 4)
define_int_show(thermal_c2, thermal_c2_value);
define_int_store(thermal_c2, thermal_c2_value, null_cb);
power_attr(thermal_c2);

define_int_show(thermal_c3, thermal_c3_value);
define_int_store(thermal_c3, thermal_c3_value, null_cb);
power_attr(thermal_c3);
#endif
#endif

define_int_show(thermal_final, thermal_final_value);
define_int_store(thermal_final, thermal_final_value, null_cb);
power_attr(thermal_final);

define_int_show(thermal_g0, thermal_g0_value);
define_int_store(thermal_g0, thermal_g0_value, null_cb);
power_attr(thermal_g0);

define_int_show(thermal_batt, thermal_batt_value);
define_int_store(thermal_batt, thermal_batt_value, null_cb);
power_attr(thermal_batt);

#ifdef CONFIG_HOTPLUG_CPU
static int mp_args_changed = 0;
static char mp_changed_attr[MAX_ATTR_LEN] = {0};
static DEFINE_SPINLOCK(mp_args_lock);

static char mp_nw_arg[MAX_BUF];
static char mp_tw_arg[MAX_BUF];
static char mp_ns_arg[MAX_BUF];
static char mp_ts_arg[MAX_BUF];
static int mp_decision_ms_value;
static int mp_min_cpus_value;
static int mp_max_cpus_value;

static void update_mp_args(const char *attr)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&mp_args_lock, irq_flags);
	if (mp_args_changed == 0) {
		mp_args_changed = 1;
		wake_up(&sysfs_state_wq);
	}
	snprintf(mp_changed_attr, strnlen(attr, MAX_ATTR_LEN) + 1, attr);
	pr_debug("[PnPMgr]: update mp arg \"%s\"\n", mp_changed_attr);
	spin_unlock_irqrestore(&mp_args_lock, irq_flags);
}

define_string_show(mp_nw, mp_nw_arg);
define_string_store(mp_nw, mp_nw_arg, update_mp_args);
power_attr(mp_nw);

define_string_show(mp_tw, mp_tw_arg);
define_string_store(mp_tw, mp_tw_arg, update_mp_args);
power_attr(mp_tw);

define_string_show(mp_ns, mp_ns_arg);
define_string_store(mp_ns, mp_ns_arg, update_mp_args);
power_attr(mp_ns);

define_string_show(mp_ts, mp_ts_arg);
define_string_store(mp_ts, mp_ts_arg, update_mp_args);
power_attr(mp_ts);

define_int_show(mp_decision_ms, mp_decision_ms_value);
define_int_store(mp_decision_ms, mp_decision_ms_value, update_mp_args);
power_attr(mp_decision_ms);

define_int_show(mp_min_cpus, mp_min_cpus_value);
define_int_store(mp_min_cpus, mp_min_cpus_value, update_mp_args);
power_attr(mp_min_cpus);

define_int_show(mp_max_cpus, mp_max_cpus_value);
define_int_store(mp_max_cpus, mp_max_cpus_value, update_mp_args);
power_attr(mp_max_cpus);

static ssize_t wait_for_mp_args_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	int ret;
	unsigned long irq_flags;

	ret = wait_event_interruptible(sysfs_state_wq, mp_args_changed == 1);
	if (ret && mp_args_changed == 0)
		return ret;
	else {
		spin_lock_irqsave(&mp_args_lock, irq_flags);
		if(mp_args_changed == 1)
			mp_args_changed = 0;
		else {
			spin_unlock_irqrestore(&mp_args_lock, irq_flags);
			return ret;
		}
		ret = sprintf(buf, "%s", mp_changed_attr);
		spin_unlock_irqrestore(&mp_args_lock, irq_flags);

		sysfs_notify(hotplug_kobj, NULL, "wait_for_mp_args");
	}
	return ret;
}
power_ro_attr(wait_for_mp_args);
#endif 

#ifdef CONFIG_PERFLOCK
extern ssize_t
perflock_scaling_max_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf);
extern ssize_t
perflock_scaling_max_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n);
extern ssize_t
perflock_scaling_min_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf);
extern ssize_t
perflock_scaling_min_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n);
power_attr(perflock_scaling_max);
power_attr(perflock_scaling_min);
#endif

#ifdef CONFIG_HOTPLUG_CPU
ssize_t
cpu_hotplug_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int ret = 0;
	ret = sprintf(buf, "%u", num_online_cpus());
	return ret;
}
ssize_t
cpu_hotplug_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	return 0;
}
power_attr(cpu_hotplug);
#endif

static struct attribute *cpufreq_g[] = {
#ifdef CONFIG_PERFLOCK
	&perflock_scaling_max_attr.attr,
	&perflock_scaling_min_attr.attr,
#endif
	NULL,
};

static struct attribute *hotplug_g[] = {
#ifdef CONFIG_HOTPLUG_CPU
	&mp_nw_attr.attr,
	&mp_tw_attr.attr,
	&mp_ns_attr.attr,
	&mp_ts_attr.attr,
	&mp_decision_ms_attr.attr,
	&mp_min_cpus_attr.attr,
	&mp_max_cpus_attr.attr,
	&wait_for_mp_args_attr.attr,
	&cpu_hotplug_attr.attr,
#endif
	NULL,
};

static struct attribute *thermal_g[] = {
	&thermal_c0_attr.attr,
#if (CONFIG_NR_CPUS >= 2)
	&thermal_c1_attr.attr,
#if (CONFIG_NR_CPUS == 4)
	&thermal_c2_attr.attr,
	&thermal_c3_attr.attr,
#endif
#endif
	&thermal_final_attr.attr,
	&thermal_g0_attr.attr,
	&thermal_batt_attr.attr,
	NULL,
};

static struct attribute *apps_g[] = {
	&activity_trigger_attr.attr,
	&media_mode_attr.attr,
	NULL,
};

static struct attribute_group cpufreq_attr_group = {
	.attrs = cpufreq_g,
};

static struct attribute_group hotplug_attr_group = {
	.attrs = hotplug_g,
};

static struct attribute_group thermal_attr_group = {
	.attrs = thermal_g,
};

static struct attribute_group apps_attr_group = {
	.attrs = apps_g,
};

#ifdef CONFIG_HOTPLUG_CPU
static int __cpuinit cpu_hotplug_callback(struct notifier_block *nfb, unsigned long action, void *hcpu)
{
	switch (action) {
		
		case CPU_ONLINE:
		case CPU_ONLINE_FROZEN:
			sysfs_notify(hotplug_kobj, NULL, "cpu_hotplug");
			break;
		case CPU_DEAD:
		case CPU_DEAD_FROZEN:
			break;
	}
	return NOTIFY_OK;
}

static struct notifier_block __refdata cpu_hotplug_notifier = {
	.notifier_call = cpu_hotplug_callback,
	.priority = -10, 
};
#endif

static unsigned int slack_time_ms;
static unsigned int step_time_ms;
static unsigned int max_powersave_bias;
static unsigned int powersave_bias_step;
static unsigned int parameter_changed;
static unsigned int adaptive_policy_enabled = 1;

define_int_show(slack_time_ms, slack_time_ms);
define_int_store(slack_time_ms, slack_time_ms, null_cb);
power_attr(slack_time_ms);

define_int_show(step_time_ms, step_time_ms);
define_int_store(step_time_ms, step_time_ms, null_cb);
power_attr(step_time_ms);

define_int_show(max_powersave_bias, max_powersave_bias);
define_int_store(max_powersave_bias, max_powersave_bias, null_cb);
power_attr(max_powersave_bias);

define_int_show(powersave_bias_step, powersave_bias_step);
define_int_store(powersave_bias_step, powersave_bias_step, null_cb);
power_attr(powersave_bias_step);


define_int_show(parameter_changed, parameter_changed);
define_int_store(parameter_changed, parameter_changed, null_cb);
power_attr(parameter_changed);

define_int_show(enabled, adaptive_policy_enabled);
define_int_store(enabled, adaptive_policy_enabled, null_cb);
power_attr(enabled);


static struct attribute *adaptive_attr[] = {
	&slack_time_ms_attr.attr,
	&step_time_ms_attr.attr,
	&max_powersave_bias_attr.attr,
	&powersave_bias_step_attr.attr,
	&parameter_changed_attr.attr,
	&enabled_attr.attr,
	NULL,
};

static struct attribute_group adaptive_attr_group = {
	.attrs = adaptive_attr,
};

static int __init pnpmgr_init(void)
{
	int ret;

	init_waitqueue_head(&sysfs_state_wq);

	pnpmgr_kobj = kobject_create_and_add("pnpmgr", power_kobj);

	if (!pnpmgr_kobj) {
		pr_err("%s: Can not allocate enough memory for pnpmgr.\n", __func__);
		return -ENOMEM;
	}

	cpufreq_kobj = kobject_create_and_add("cpufreq", pnpmgr_kobj);
	hotplug_kobj = kobject_create_and_add("hotplug", pnpmgr_kobj);
	thermal_kobj = kobject_create_and_add("thermal", pnpmgr_kobj);
	apps_kobj = kobject_create_and_add("apps", pnpmgr_kobj);
	adaptive_policy_kobj = kobject_create_and_add("adaptive_policy", power_kobj);

	if (!cpufreq_kobj || !hotplug_kobj || !thermal_kobj || !apps_kobj || !adaptive_policy_kobj) {
		pr_err("%s: Can not allocate enough memory.\n", __func__);
		return -ENOMEM;
	}

	ret = sysfs_create_group(cpufreq_kobj, &cpufreq_attr_group);
	ret |= sysfs_create_group(hotplug_kobj, &hotplug_attr_group);
	ret |= sysfs_create_group(thermal_kobj, &thermal_attr_group);
	ret |= sysfs_create_group(apps_kobj, &apps_attr_group);
	ret |= sysfs_create_group(adaptive_policy_kobj, &adaptive_attr_group);

	if (ret) {
		pr_err("%s: sysfs_create_group failed\n", __func__);
		return ret;
	}

#ifdef CONFIG_HOTPLUG_CPU
	register_hotcpu_notifier(&cpu_hotplug_notifier);
#endif

	return 0;
}

static void  __exit pnpmgr_exit(void)
{
	sysfs_remove_group(cpufreq_kobj, &cpufreq_attr_group);
	sysfs_remove_group(hotplug_kobj, &hotplug_attr_group);
	sysfs_remove_group(thermal_kobj, &thermal_attr_group);
	sysfs_remove_group(apps_kobj, &apps_attr_group);
	sysfs_remove_group(adaptive_policy_kobj, &adaptive_attr_group);
#ifdef CONFIG_HOTPLUG_CPU
	unregister_hotcpu_notifier(&cpu_hotplug_notifier);
#endif
}

module_init(pnpmgr_init);
module_exit(pnpmgr_exit);
