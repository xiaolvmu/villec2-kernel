/*
 * kernel/power/main.c - PM subsystem core functionality.
 *
 * Copyright (c) 2003 Patrick Mochel
 * Copyright (c) 2003 Open Source Development Lab
 *
 * This file is released under the GPLv2
 *
 */

#include <linux/export.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/resume-trace.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/hrtimer.h>

#include "power.h"

#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif

#define MAX_BUF 100

DEFINE_MUTEX(pm_mutex);

#ifdef CONFIG_PM_SLEEP


static BLOCKING_NOTIFIER_HEAD(pm_chain_head);

static void touch_event_fn(struct work_struct *work);
static DECLARE_WORK(touch_event_struct, touch_event_fn);

static struct hrtimer tc_ev_timer;
static int tc_ev_processed;
static ktime_t touch_evt_timer_val;

int register_pm_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&pm_chain_head, nb);
}
EXPORT_SYMBOL_GPL(register_pm_notifier);

int unregister_pm_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&pm_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_pm_notifier);

int pm_notifier_call_chain(unsigned long val)
{
	int ret = blocking_notifier_call_chain(&pm_chain_head, val, NULL);

	return notifier_to_errno(ret);
}

int pm_async_enabled = 1;

static ssize_t pm_async_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%d\n", pm_async_enabled);
}

static ssize_t pm_async_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 1)
		return -EINVAL;

	pm_async_enabled = val;
	return n;
}

power_attr(pm_async);

static ssize_t
touch_event_show(struct kobject *kobj,
		 struct kobj_attribute *attr, char *buf)
{
	if (tc_ev_processed == 0)
		return snprintf(buf, strnlen("touch_event", MAX_BUF) + 1,
				"touch_event");
	else
		return snprintf(buf, strnlen("null", MAX_BUF) + 1,
				"null");
}

static ssize_t
touch_event_store(struct kobject *kobj,
		  struct kobj_attribute *attr,
		  const char *buf, size_t n)
{

	hrtimer_cancel(&tc_ev_timer);
	tc_ev_processed = 0;

	hrtimer_start(&tc_ev_timer, touch_evt_timer_val, HRTIMER_MODE_REL);

	
	sysfs_notify(kobj, NULL, "touch_event");

	return n;
}

power_attr(touch_event);

static ssize_t
touch_event_timer_show(struct kobject *kobj,
		 struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, MAX_BUF, "%lld", touch_evt_timer_val.tv64);
}

static ssize_t
touch_event_timer_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	touch_evt_timer_val = ktime_set(0, val*1000);

	return n;
}

power_attr(touch_event_timer);

static void touch_event_fn(struct work_struct *work)
{
	
	tc_ev_processed = 1;
	sysfs_notify(power_kobj, NULL, "touch_event");

	return;
}

static enum hrtimer_restart tc_ev_stop(struct hrtimer *hrtimer)
{

	schedule_work(&touch_event_struct);

	return HRTIMER_NORESTART;
}

#ifdef CONFIG_PM_DEBUG
int pm_test_level = TEST_NONE;

static const char * const pm_tests[__TEST_AFTER_LAST] = {
	[TEST_NONE] = "none",
	[TEST_CORE] = "core",
	[TEST_CPUS] = "processors",
	[TEST_PLATFORM] = "platform",
	[TEST_DEVICES] = "devices",
	[TEST_FREEZER] = "freezer",
};

static ssize_t pm_test_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	char *s = buf;
	int level;

	for (level = TEST_FIRST; level <= TEST_MAX; level++)
		if (pm_tests[level]) {
			if (level == pm_test_level)
				s += sprintf(s, "[%s] ", pm_tests[level]);
			else
				s += sprintf(s, "%s ", pm_tests[level]);
		}

	if (s != buf)
		
		*(s-1) = '\n';

	return (s - buf);
}

static ssize_t pm_test_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	const char * const *s;
	int level;
	char *p;
	int len;
	int error = -EINVAL;

	p = memchr(buf, '\n', n);
	len = p ? p - buf : n;

	lock_system_sleep();

	level = TEST_FIRST;
	for (s = &pm_tests[level]; level <= TEST_MAX; s++, level++)
		if (*s && len == strlen(*s) && !strncmp(buf, *s, len)) {
			pm_test_level = level;
			error = 0;
			break;
		}

	unlock_system_sleep();

	return error ? error : n;
}

power_attr(pm_test);
#endif 

#ifdef CONFIG_DEBUG_FS
static char *suspend_step_name(enum suspend_stat_step step)
{
	switch (step) {
	case SUSPEND_FREEZE:
		return "freeze";
	case SUSPEND_PREPARE:
		return "prepare";
	case SUSPEND_SUSPEND:
		return "suspend";
	case SUSPEND_SUSPEND_NOIRQ:
		return "suspend_noirq";
	case SUSPEND_RESUME_NOIRQ:
		return "resume_noirq";
	case SUSPEND_RESUME:
		return "resume";
	default:
		return "";
	}
}

static int suspend_stats_show(struct seq_file *s, void *unused)
{
	int i, index, last_dev, last_errno, last_step;

	last_dev = suspend_stats.last_failed_dev + REC_FAILED_NUM - 1;
	last_dev %= REC_FAILED_NUM;
	last_errno = suspend_stats.last_failed_errno + REC_FAILED_NUM - 1;
	last_errno %= REC_FAILED_NUM;
	last_step = suspend_stats.last_failed_step + REC_FAILED_NUM - 1;
	last_step %= REC_FAILED_NUM;
	seq_printf(s, "%s: %d\n%s: %d\n%s: %d\n%s: %d\n%s: %d\n"
			"%s: %d\n%s: %d\n%s: %d\n%s: %d\n%s: %d\n",
			"success", suspend_stats.success,
			"fail", suspend_stats.fail,
			"failed_freeze", suspend_stats.failed_freeze,
			"failed_prepare", suspend_stats.failed_prepare,
			"failed_suspend", suspend_stats.failed_suspend,
			"failed_suspend_late",
				suspend_stats.failed_suspend_late,
			"failed_suspend_noirq",
				suspend_stats.failed_suspend_noirq,
			"failed_resume", suspend_stats.failed_resume,
			"failed_resume_early",
				suspend_stats.failed_resume_early,
			"failed_resume_noirq",
				suspend_stats.failed_resume_noirq);
	seq_printf(s,	"failures:\n  last_failed_dev:\t%-s\n",
			suspend_stats.failed_devs[last_dev]);
	for (i = 1; i < REC_FAILED_NUM; i++) {
		index = last_dev + REC_FAILED_NUM - i;
		index %= REC_FAILED_NUM;
		seq_printf(s, "\t\t\t%-s\n",
			suspend_stats.failed_devs[index]);
	}
	seq_printf(s,	"  last_failed_errno:\t%-d\n",
			suspend_stats.errno[last_errno]);
	for (i = 1; i < REC_FAILED_NUM; i++) {
		index = last_errno + REC_FAILED_NUM - i;
		index %= REC_FAILED_NUM;
		seq_printf(s, "\t\t\t%-d\n",
			suspend_stats.errno[index]);
	}
	seq_printf(s,	"  last_failed_step:\t%-s\n",
			suspend_step_name(
				suspend_stats.failed_steps[last_step]));
	for (i = 1; i < REC_FAILED_NUM; i++) {
		index = last_step + REC_FAILED_NUM - i;
		index %= REC_FAILED_NUM;
		seq_printf(s, "\t\t\t%-s\n",
			suspend_step_name(
				suspend_stats.failed_steps[index]));
	}

	return 0;
}

static int suspend_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, suspend_stats_show, NULL);
}

static const struct file_operations suspend_stats_operations = {
	.open           = suspend_stats_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int __init pm_debugfs_init(void)
{
	debugfs_create_file("suspend_stats", S_IFREG | S_IRUGO,
			NULL, NULL, &suspend_stats_operations);
	return 0;
}

late_initcall(pm_debugfs_init);
#endif 

#endif 

struct kobject *power_kobj;

static ssize_t state_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char *s = buf;
#ifdef CONFIG_SUSPEND
	int i;

	for (i = 0; i < PM_SUSPEND_MAX; i++) {
		if (pm_states[i] && valid_state(i))
			s += sprintf(s,"%s ", pm_states[i]);
	}
#endif
#ifdef CONFIG_HIBERNATION
	s += sprintf(s, "%s\n", "disk");
#else
	if (s != buf)
		
		*(s-1) = '\n';
#endif
	return (s - buf);
}

static ssize_t state_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
#ifdef CONFIG_SUSPEND
#ifdef CONFIG_EARLYSUSPEND
	suspend_state_t state = PM_SUSPEND_ON;
#else
	suspend_state_t state = PM_SUSPEND_STANDBY;
#endif
	const char * const *s;
#endif
	char *p;
	int len;
	int error = -EINVAL;

	p = memchr(buf, '\n', n);
	len = p ? p - buf : n;

	
	if (len == 4 && !strncmp(buf, "disk", len)) {
		error = hibernate();
		goto Exit;
	}

#ifdef CONFIG_SUSPEND
	for (s = &pm_states[state]; state < PM_SUSPEND_MAX; s++, state++) {
		if (*s && len == strlen(*s) && !strncmp(buf, *s, len)) {
#ifdef CONFIG_EARLYSUSPEND
			if (state == PM_SUSPEND_ON || valid_state(state)) {
				error = 0;
				request_suspend_state(state);
				break;
			}
#else
			error = pm_suspend(state);
#endif
		}
	}
#endif

 Exit:
	return error ? error : n;
}

power_attr(state);

#ifdef CONFIG_PM_SLEEP

static ssize_t wakeup_count_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	unsigned int val;

	return pm_get_wakeup_count(&val) ? sprintf(buf, "%u\n", val) : -EINTR;
}

static ssize_t wakeup_count_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	unsigned int val;

	if (sscanf(buf, "%u", &val) == 1) {
		if (pm_save_wakeup_count(val))
			return n;
	}
	return -EINVAL;
}

power_attr(wakeup_count);
#endif 

#ifdef CONFIG_PM_TRACE
int pm_trace_enabled;

static ssize_t pm_trace_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%d\n", pm_trace_enabled);
}

static ssize_t
pm_trace_store(struct kobject *kobj, struct kobj_attribute *attr,
	       const char *buf, size_t n)
{
	int val;

	if (sscanf(buf, "%d", &val) == 1) {
		pm_trace_enabled = !!val;
		return n;
	}
	return -EINVAL;
}

power_attr(pm_trace);

static ssize_t pm_trace_dev_match_show(struct kobject *kobj,
				       struct kobj_attribute *attr,
				       char *buf)
{
	return show_trace_dev_match(buf, PAGE_SIZE);
}

static ssize_t
pm_trace_dev_match_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t n)
{
	return -EINVAL;
}

power_attr(pm_trace_dev_match);

#endif 

#ifdef CONFIG_USER_WAKELOCK
power_attr(wake_lock);
power_attr(wake_unlock);
#endif

#ifdef CONFIG_PERFLOCK
static struct perf_lock user_cpu_perf_lock;
static struct perf_lock user_cpu_ceiling_lock;
static struct perf_lock user_perf_lock[PERF_LOCK_INVALID];
static struct perf_lock user_ceiling_lock[PERF_LOCK_INVALID];
static ssize_t
perflock_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	
	int i, perf_enable = 0;

	for (i = 0; i < PERF_LOCK_INVALID; i++)
		if (is_perf_lock_active(&user_perf_lock[i]) != 0)
			perf_enable |= (1 << i);

	if (is_perf_lock_active(&user_cpu_perf_lock) != 0)
		perf_enable |= (1 << PERF_LOCK_INVALID);

	return sprintf(buf, "%d\n", perf_enable);
}

static inline void user_cpufreq_perf_lock(int level, int val)
{
	if (val == 1 && !is_perf_lock_active(&user_perf_lock[level]))
		perf_lock(&user_perf_lock[level]);
	if (val == 0 && is_perf_lock_active(&user_perf_lock[level]))
		perf_unlock(&user_perf_lock[level]);
}

#define perf_level_wrapper(off, on, level) \
	case off:\
		user_cpufreq_perf_lock(level, 0);\
		break;\
	case on:\
		user_cpufreq_perf_lock(level, 1);\
		break;


static ssize_t
perflock_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int val , ret = -EINVAL;

	if (sscanf(buf, "%d", &val) > 0) {
		if (val == 11 && !is_perf_lock_active(&user_cpu_perf_lock)) {
			perf_lock(&user_cpu_perf_lock);
			ret = n;
		} else if (val == 10 && is_perf_lock_active(&user_cpu_perf_lock)) {
			perf_unlock(&user_cpu_perf_lock);
			ret = n;
		} else {
			switch (val) {
			perf_level_wrapper(0, 1, PERF_LOCK_LOWEST);
			perf_level_wrapper(2, 3, PERF_LOCK_LOW);
			perf_level_wrapper(4, 5, PERF_LOCK_MEDIUM);
			perf_level_wrapper(6, 7, PERF_LOCK_HIGH);
			perf_level_wrapper(8, 9, PERF_LOCK_HIGHEST);
			default:
				
				break;
			}
			return n;
		}
	}
	return ret;
}
power_attr(perflock);

int launch_event_enabled = 0;
static ssize_t
launch_event_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", launch_event_enabled);
}

static ssize_t
launch_event_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 1)
		return -EINVAL;

	launch_event_enabled = val;
	sysfs_notify(kobj, NULL, "launch_event");
	return n;
}
power_attr(launch_event);

int powersave_enabled = 0;
static ssize_t
powersave_show(struct kobject *kobj, struct kobj_attribute *attr,
                char *buf)
{
        return sprintf(buf, "%d\n", powersave_enabled);
}

static ssize_t
powersave_store(struct kobject *kobj, struct kobj_attribute *attr,
                const char *buf, size_t n)
{
        unsigned long val;

        if (strict_strtoul(buf, 10, &val))
                return -EINVAL;

        if (val > 1)
                return -EINVAL;

        powersave_enabled = val;
        sysfs_notify(kobj, NULL, "powersave");
        return n;
}
power_attr(powersave);

static ssize_t
cpufreq_ceiling_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	
	int i, ceiling_enable = 0;

	for (i = 0; i < PERF_LOCK_INVALID; i++)
		if(is_perf_lock_active(&user_ceiling_lock[i]) != 0)
			ceiling_enable |= (1 << i);

	if (is_perf_lock_active(&user_cpu_ceiling_lock) != 0)
		ceiling_enable |= (1 << PERF_LOCK_INVALID);

	return sprintf(buf, "%d\n", ceiling_enable);
}

static inline void user_cpufreq_ceiling_lock(int level, int val)
{
	if (val == 1 && !is_perf_lock_active(&user_ceiling_lock[level]))
		perf_lock(&user_ceiling_lock[level]);
	if (val == 0 && is_perf_lock_active(&user_ceiling_lock[level]))
		perf_unlock(&user_ceiling_lock[level]);
}

#define ceiling_level_wrapper(off, on, level) \
	case off:\
		user_cpufreq_ceiling_lock(level, 0);\
		break;\
	case on:\
		user_cpufreq_ceiling_lock(level, 1);\
		break;

static ssize_t
cpufreq_ceiling_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int val, ret = -EINVAL;

	if (sscanf(buf, "%d", &val) > 0) {
		if (val == 11 && !is_perf_lock_active(&user_cpu_ceiling_lock)) {
			perf_lock(&user_cpu_ceiling_lock);
			ret = n;
		} else if (val == 10 && is_perf_lock_active(&user_cpu_ceiling_lock)) {
			perf_unlock(&user_cpu_ceiling_lock);
			ret = n;
		} else {
			switch (val){
			ceiling_level_wrapper(0, 1, PERF_LOCK_LOWEST);
			ceiling_level_wrapper(2, 3, PERF_LOCK_LOW);
			ceiling_level_wrapper(4, 5, PERF_LOCK_MEDIUM);
			ceiling_level_wrapper(6, 7, PERF_LOCK_HIGH);
			ceiling_level_wrapper(8, 9, PERF_LOCK_HIGHEST);
			default:
				
				break;
			}
			ret = n;
		}
	}

	return ret;
}
power_attr(cpufreq_ceiling);

#ifdef CONFIG_HTC_ONMODE_CHARGING
static ssize_t state_onchg_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	char *s = buf;
	if (get_onchg_state())
		s += sprintf(s, "chgoff ");
	else
		s += sprintf(s, "chgon ");

	if (s != buf)
		
		*(s-1) = '\n';

	return (s - buf);
}

static ssize_t
state_onchg_store(struct kobject *kobj, struct kobj_attribute *attr,
	       const char *buf, size_t n)
{
	char *p;
	int len;

	p = memchr(buf, '\n', n);
	len = p ? p - buf : n;

	if (len == 5 || len == 6 || len == 7) {
		if (!strncmp(buf, "chgon", len))
			request_onchg_state(1);
		else if (!strncmp(buf, "chgoff", len))
			request_onchg_state(0);
	}

	return 0;
}

power_attr(state_onchg);
#endif

static int cpunum_max;
static int cpunum_min;

static ssize_t
cpunum_floor_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int i;
	int all_cpus = num_possible_cpus();
	for (i = all_cpus-1 ; i >= 0 ; i--) {
		if (cpunum_min & (1 << i))
		    break;
	}
	if (i < 0)
	    i = 0;
	else
	    i++;

	return sprintf(buf, "%d\n", i);
}

static ssize_t
cpunum_floor_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int val, bit, on;

	if (sscanf(buf, "%d", &val) > 0) {
		bit = val / 2;
		on = val % 2;
		if (bit >= num_possible_cpus() || bit < 0)
		    return -EINVAL;
		if (on)
		    cpunum_min |= (1 << bit);
		else
		    cpunum_min &= ~(1 << bit);
		sysfs_notify(kobj, NULL, "cpunum_floor");
		return n;
	}
	return -EINVAL;
}

static ssize_t
cpunum_ceiling_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int i;
	int all_cpus = num_possible_cpus();
	for (i = 0 ; i < all_cpus ; i++) {
		if (cpunum_max & (1 << i))
		    break;
	}
	if (i >= all_cpus)
	    i = 0;
	else
	    i++;

	return sprintf(buf, "%d\n", i);
}

static ssize_t
cpunum_ceiling_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int val, bit, on;

	if (sscanf(buf, "%d", &val) > 0) {
		bit = val / 2;
		on = val % 2;
		if (bit >= num_possible_cpus() || bit < 0)
		    return -EINVAL;
		if (on)
		    cpunum_max |= (1 << bit);
		else
		    cpunum_max &= ~(1 << bit);
		sysfs_notify(kobj, NULL, "cpunum_ceiling");
		return n;
	}
	return -EINVAL;
}

power_attr(cpunum_floor);
power_attr(cpunum_ceiling);
#endif

static struct attribute *g[] = {
	&state_attr.attr,
#ifdef CONFIG_PM_TRACE
	&pm_trace_attr.attr,
	&pm_trace_dev_match_attr.attr,
#endif
#ifdef CONFIG_PM_SLEEP
	&pm_async_attr.attr,
	&wakeup_count_attr.attr,
	&touch_event_attr.attr,
	&touch_event_timer_attr.attr,
#ifdef CONFIG_PM_DEBUG
	&pm_test_attr.attr,
#endif
#ifdef CONFIG_USER_WAKELOCK
	&wake_lock_attr.attr,
	&wake_unlock_attr.attr,
#endif
#ifdef CONFIG_HTC_ONMODE_CHARGING
	&state_onchg_attr.attr,
#endif
#endif
#ifdef CONFIG_PERFLOCK
	&perflock_attr.attr,
	&cpufreq_ceiling_attr.attr,
	&launch_event_attr.attr,
	&powersave_attr.attr,
	&cpunum_floor_attr.attr,
	&cpunum_ceiling_attr.attr,
#endif
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

#ifdef CONFIG_PM_RUNTIME
struct workqueue_struct *pm_wq;
struct workqueue_struct *pm_rt_wq;
EXPORT_SYMBOL_GPL(pm_wq);

static int __init pm_start_workqueue(void)
{
	pm_wq = alloc_workqueue("pm", WQ_FREEZABLE, 0);
	
	pm_rt_wq = alloc_workqueue("pm_rt", WQ_FREEZABLE | WQ_HIGHPRI , 0);
	

	return pm_wq ? 0 : -ENOMEM;
}
#else
static inline int pm_start_workqueue(void) { return 0; }
#endif

static int __init pm_init(void)
{
	int error = pm_start_workqueue();
#ifdef CONFIG_PERFLOCK
	int i;
	static char ceil_buf[PERF_LOCK_INVALID][38];
	static char perf_buf[PERF_LOCK_INVALID][24];
#endif
	if (error)
		return error;
	hibernate_image_size_init();
	hibernate_reserved_size_init();

	touch_evt_timer_val = ktime_set(2, 0);
	hrtimer_init(&tc_ev_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tc_ev_timer.function = &tc_ev_stop;
	tc_ev_processed = 1;


	power_kobj = kobject_create_and_add("power", NULL);
#ifdef CONFIG_PERFLOCK
	perf_lock_init(&user_cpu_perf_lock, TYPE_PERF_LOCK, PERF_LOCK_HIGHEST, "User CPU Highest Perflock"); 
	perf_lock_init(&user_cpu_ceiling_lock, TYPE_CPUFREQ_CEILING, PERF_LOCK_HIGH, "User CPU High cpufreq_ceiling lock"); 
	for (i = PERF_LOCK_LOWEST; i < PERF_LOCK_INVALID; i++) {
		snprintf(perf_buf[i], 23, "User Perflock level(%d)", i);
		perf_buf[i][23] = '\0';
		perf_lock_init(&user_perf_lock[i], TYPE_PERF_LOCK, i, perf_buf[i]);

		snprintf(ceil_buf[i], 37, "User cpufreq_ceiling lock level(%d)", i);
		ceil_buf[i][37] = '\0';
		perf_lock_init(&user_ceiling_lock[i], TYPE_CPUFREQ_CEILING, i, ceil_buf[i]);
	}
#endif
	if (!power_kobj)
		return -ENOMEM;
	return sysfs_create_group(power_kobj, &attr_group);
}

core_initcall(pm_init);
