/*
 * linux/kernel/posix-timers.c
 *
 *
 * 2002-10-15  Posix Clocks & timers
 *                           by George Anzinger george@mvista.com
 *
 *			     Copyright (C) 2002 2003 by MontaVista Software.
 *
 * 2004-06-01  Fix CLOCK_REALTIME clock/timer TIMER_ABSTIME bug.
 *			     Copyright (C) 2004 Boris Hu
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * MontaVista Software | 1237 East Arques Avenue | Sunnyvale | CA 94085 | USA
 */

#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/mutex.h>

#include <asm/uaccess.h>
#include <linux/list.h>
#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/idr.h>
#include <linux/posix-clock.h>
#include <linux/posix-timers.h>
#include <linux/syscalls.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/export.h>


static struct kmem_cache *posix_timers_cache;
static struct idr posix_timers_id;
static DEFINE_SPINLOCK(idr_lock);

#if SIGEV_THREAD_ID != (SIGEV_THREAD_ID & \
                       ~(SIGEV_SIGNAL | SIGEV_NONE | SIGEV_THREAD))
#error "SIGEV_THREAD_ID must not share bit with other SIGEV values!"
#endif

#ifndef ENOTSUP
# define ENANOSLEEP_NOTSUP EOPNOTSUPP
#else
# define ENANOSLEEP_NOTSUP ENOTSUP
#endif



static struct k_clock posix_clocks[MAX_CLOCKS];

static int common_nsleep(const clockid_t, int flags, struct timespec *t,
			 struct timespec __user *rmtp);
static int common_timer_create(struct k_itimer *new_timer);
static void common_timer_get(struct k_itimer *, struct itimerspec *);
static int common_timer_set(struct k_itimer *, int,
			    struct itimerspec *, struct itimerspec *);
static int common_timer_del(struct k_itimer *timer);

static enum hrtimer_restart posix_timer_fn(struct hrtimer *data);

static struct k_itimer *__lock_timer(timer_t timer_id, unsigned long *flags);

#define lock_timer(tid, flags)						   \
({	struct k_itimer *__timr;					   \
	__cond_lock(&__timr->it_lock, __timr = __lock_timer(tid, flags));  \
	__timr;								   \
})

static inline void unlock_timer(struct k_itimer *timr, unsigned long flags)
{
	spin_unlock_irqrestore(&timr->it_lock, flags);
}

static int posix_clock_realtime_get(clockid_t which_clock, struct timespec *tp)
{
	ktime_get_real_ts(tp);
	return 0;
}

static int posix_clock_realtime_set(const clockid_t which_clock,
				    const struct timespec *tp)
{
	return do_sys_settimeofday(tp, NULL);
}

static int posix_clock_realtime_adj(const clockid_t which_clock,
				    struct timex *t)
{
	return do_adjtimex(t);
}

static int posix_ktime_get_ts(clockid_t which_clock, struct timespec *tp)
{
	ktime_get_ts(tp);
	return 0;
}

static int posix_get_monotonic_raw(clockid_t which_clock, struct timespec *tp)
{
	getrawmonotonic(tp);
	return 0;
}


static int posix_get_realtime_coarse(clockid_t which_clock, struct timespec *tp)
{
	*tp = current_kernel_time();
	return 0;
}

static int posix_get_monotonic_coarse(clockid_t which_clock,
						struct timespec *tp)
{
	*tp = get_monotonic_coarse();
	return 0;
}

static int posix_get_coarse_res(const clockid_t which_clock, struct timespec *tp)
{
	*tp = ktime_to_timespec(KTIME_LOW_RES);
	return 0;
}

static int posix_get_boottime(const clockid_t which_clock, struct timespec *tp)
{
	get_monotonic_boottime(tp);
	return 0;
}


static __init int init_posix_timers(void)
{
	struct k_clock clock_realtime = {
		.clock_getres	= hrtimer_get_res,
		.clock_get	= posix_clock_realtime_get,
		.clock_set	= posix_clock_realtime_set,
		.clock_adj	= posix_clock_realtime_adj,
		.nsleep		= common_nsleep,
		.nsleep_restart	= hrtimer_nanosleep_restart,
		.timer_create	= common_timer_create,
		.timer_set	= common_timer_set,
		.timer_get	= common_timer_get,
		.timer_del	= common_timer_del,
	};
	struct k_clock clock_monotonic = {
		.clock_getres	= hrtimer_get_res,
		.clock_get	= posix_ktime_get_ts,
		.nsleep		= common_nsleep,
		.nsleep_restart	= hrtimer_nanosleep_restart,
		.timer_create	= common_timer_create,
		.timer_set	= common_timer_set,
		.timer_get	= common_timer_get,
		.timer_del	= common_timer_del,
	};
	struct k_clock clock_monotonic_raw = {
		.clock_getres	= hrtimer_get_res,
		.clock_get	= posix_get_monotonic_raw,
	};
	struct k_clock clock_realtime_coarse = {
		.clock_getres	= posix_get_coarse_res,
		.clock_get	= posix_get_realtime_coarse,
	};
	struct k_clock clock_monotonic_coarse = {
		.clock_getres	= posix_get_coarse_res,
		.clock_get	= posix_get_monotonic_coarse,
	};
	struct k_clock clock_boottime = {
		.clock_getres	= hrtimer_get_res,
		.clock_get	= posix_get_boottime,
		.nsleep		= common_nsleep,
		.nsleep_restart	= hrtimer_nanosleep_restart,
		.timer_create	= common_timer_create,
		.timer_set	= common_timer_set,
		.timer_get	= common_timer_get,
		.timer_del	= common_timer_del,
	};

	posix_timers_register_clock(CLOCK_REALTIME, &clock_realtime);
	posix_timers_register_clock(CLOCK_MONOTONIC, &clock_monotonic);
	posix_timers_register_clock(CLOCK_MONOTONIC_RAW, &clock_monotonic_raw);
	posix_timers_register_clock(CLOCK_REALTIME_COARSE, &clock_realtime_coarse);
	posix_timers_register_clock(CLOCK_MONOTONIC_COARSE, &clock_monotonic_coarse);
	posix_timers_register_clock(CLOCK_BOOTTIME, &clock_boottime);

	posix_timers_cache = kmem_cache_create("posix_timers_cache",
					sizeof (struct k_itimer), 0, SLAB_PANIC,
					NULL);
	idr_init(&posix_timers_id);
	return 0;
}

__initcall(init_posix_timers);

static void schedule_next_timer(struct k_itimer *timr)
{
	struct hrtimer *timer = &timr->it.real.timer;

	if (timr->it.real.interval.tv64 == 0)
		return;

	timr->it_overrun += (unsigned int) hrtimer_forward(timer,
						timer->base->get_time(),
						timr->it.real.interval);

	timr->it_overrun_last = timr->it_overrun;
	timr->it_overrun = -1;
	++timr->it_requeue_pending;
	hrtimer_restart(timer);
}

void do_schedule_next_timer(struct siginfo *info)
{
	struct k_itimer *timr;
	unsigned long flags;

	timr = lock_timer(info->si_tid, &flags);

	if (timr && timr->it_requeue_pending == info->si_sys_private) {
		if (timr->it_clock < 0)
			posix_cpu_timer_schedule(timr);
		else
			schedule_next_timer(timr);

		info->si_overrun += timr->it_overrun_last;
	}

	if (timr)
		unlock_timer(timr, flags);
}

int posix_timer_event(struct k_itimer *timr, int si_private)
{
	struct task_struct *task;
	int shared, ret = -1;
	timr->sigq->info.si_sys_private = si_private;

	rcu_read_lock();
	task = pid_task(timr->it_pid, PIDTYPE_PID);
	if (task) {
		shared = !(timr->it_sigev_notify & SIGEV_THREAD_ID);
		ret = send_sigqueue(timr->sigq, task, shared);
	}
	rcu_read_unlock();
	
	return ret > 0;
}
EXPORT_SYMBOL_GPL(posix_timer_event);

static enum hrtimer_restart posix_timer_fn(struct hrtimer *timer)
{
	struct k_itimer *timr;
	unsigned long flags;
	int si_private = 0;
	enum hrtimer_restart ret = HRTIMER_NORESTART;

	timr = container_of(timer, struct k_itimer, it.real.timer);
	spin_lock_irqsave(&timr->it_lock, flags);

	if (timr->it.real.interval.tv64 != 0)
		si_private = ++timr->it_requeue_pending;

	if (posix_timer_event(timr, si_private)) {
		if (timr->it.real.interval.tv64 != 0) {
			ktime_t now = hrtimer_cb_get_time(timer);

#ifdef CONFIG_HIGH_RES_TIMERS
			{
				ktime_t kj = ktime_set(0, NSEC_PER_SEC / HZ);

				if (timr->it.real.interval.tv64 < kj.tv64)
					now = ktime_add(now, kj);
			}
#endif
			timr->it_overrun += (unsigned int)
				hrtimer_forward(timer, now,
						timr->it.real.interval);
			ret = HRTIMER_RESTART;
			++timr->it_requeue_pending;
		}
	}

	unlock_timer(timr, flags);
	return ret;
}

static struct pid *good_sigevent(sigevent_t * event)
{
	struct task_struct *rtn = current->group_leader;

	if ((event->sigev_notify & SIGEV_THREAD_ID ) &&
		(!(rtn = find_task_by_vpid(event->sigev_notify_thread_id)) ||
		 !same_thread_group(rtn, current) ||
		 (event->sigev_notify & ~SIGEV_THREAD_ID) != SIGEV_SIGNAL))
		return NULL;

	if (((event->sigev_notify & ~SIGEV_THREAD_ID) != SIGEV_NONE) &&
	    ((event->sigev_signo <= 0) || (event->sigev_signo > SIGRTMAX)))
		return NULL;

	return task_pid(rtn);
}

void posix_timers_register_clock(const clockid_t clock_id,
				 struct k_clock *new_clock)
{
	if ((unsigned) clock_id >= MAX_CLOCKS) {
		printk(KERN_WARNING "POSIX clock register failed for clock_id %d\n",
		       clock_id);
		return;
	}

	if (!new_clock->clock_get) {
		printk(KERN_WARNING "POSIX clock id %d lacks clock_get()\n",
		       clock_id);
		return;
	}
	if (!new_clock->clock_getres) {
		printk(KERN_WARNING "POSIX clock id %d lacks clock_getres()\n",
		       clock_id);
		return;
	}

	posix_clocks[clock_id] = *new_clock;
}
EXPORT_SYMBOL_GPL(posix_timers_register_clock);

static struct k_itimer * alloc_posix_timer(void)
{
	struct k_itimer *tmr;
	tmr = kmem_cache_zalloc(posix_timers_cache, GFP_KERNEL);
	if (!tmr)
		return tmr;
	if (unlikely(!(tmr->sigq = sigqueue_alloc()))) {
		kmem_cache_free(posix_timers_cache, tmr);
		return NULL;
	}
	memset(&tmr->sigq->info, 0, sizeof(siginfo_t));
	return tmr;
}

static void k_itimer_rcu_free(struct rcu_head *head)
{
	struct k_itimer *tmr = container_of(head, struct k_itimer, it.rcu);

	kmem_cache_free(posix_timers_cache, tmr);
}

#define IT_ID_SET	1
#define IT_ID_NOT_SET	0
static void release_posix_timer(struct k_itimer *tmr, int it_id_set)
{
	if (it_id_set) {
		unsigned long flags;
		spin_lock_irqsave(&idr_lock, flags);
		idr_remove(&posix_timers_id, tmr->it_id);
		spin_unlock_irqrestore(&idr_lock, flags);
	}
	put_pid(tmr->it_pid);
	sigqueue_free(tmr->sigq);
	call_rcu(&tmr->it.rcu, k_itimer_rcu_free);
}

static struct k_clock *clockid_to_kclock(const clockid_t id)
{
	if (id < 0)
		return (id & CLOCKFD_MASK) == CLOCKFD ?
			&clock_posix_dynamic : &clock_posix_cpu;

	if (id >= MAX_CLOCKS || !posix_clocks[id].clock_getres)
		return NULL;
	return &posix_clocks[id];
}

static int common_timer_create(struct k_itimer *new_timer)
{
	hrtimer_init(&new_timer->it.real.timer, new_timer->it_clock, 0);
	return 0;
}


SYSCALL_DEFINE3(timer_create, const clockid_t, which_clock,
		struct sigevent __user *, timer_event_spec,
		timer_t __user *, created_timer_id)
{
	struct k_clock *kc = clockid_to_kclock(which_clock);
	struct k_itimer *new_timer;
	int error, new_timer_id;
	sigevent_t event;
	int it_id_set = IT_ID_NOT_SET;

	if (!kc)
		return -EINVAL;
	if (!kc->timer_create)
		return -EOPNOTSUPP;

	new_timer = alloc_posix_timer();
	if (unlikely(!new_timer))
		return -EAGAIN;

	spin_lock_init(&new_timer->it_lock);
 retry:
	if (unlikely(!idr_pre_get(&posix_timers_id, GFP_KERNEL))) {
		error = -EAGAIN;
		goto out;
	}
	spin_lock_irq(&idr_lock);
	error = idr_get_new(&posix_timers_id, new_timer, &new_timer_id);
	spin_unlock_irq(&idr_lock);
	if (error) {
		if (error == -EAGAIN)
			goto retry;
		error = -EAGAIN;
		goto out;
	}

	it_id_set = IT_ID_SET;
	new_timer->it_id = (timer_t) new_timer_id;
	new_timer->it_clock = which_clock;
	new_timer->it_overrun = -1;

	if (timer_event_spec) {
		if (copy_from_user(&event, timer_event_spec, sizeof (event))) {
			error = -EFAULT;
			goto out;
		}
		rcu_read_lock();
		new_timer->it_pid = get_pid(good_sigevent(&event));
		rcu_read_unlock();
		if (!new_timer->it_pid) {
			error = -EINVAL;
			goto out;
		}
	} else {
		event.sigev_notify = SIGEV_SIGNAL;
		event.sigev_signo = SIGALRM;
		event.sigev_value.sival_int = new_timer->it_id;
		new_timer->it_pid = get_pid(task_tgid(current));
	}

	new_timer->it_sigev_notify     = event.sigev_notify;
	new_timer->sigq->info.si_signo = event.sigev_signo;
	new_timer->sigq->info.si_value = event.sigev_value;
	new_timer->sigq->info.si_tid   = new_timer->it_id;
	new_timer->sigq->info.si_code  = SI_TIMER;

	if (copy_to_user(created_timer_id,
			 &new_timer_id, sizeof (new_timer_id))) {
		error = -EFAULT;
		goto out;
	}

	error = kc->timer_create(new_timer);
	if (error)
		goto out;

	spin_lock_irq(&current->sighand->siglock);
	new_timer->it_signal = current->signal;
	list_add(&new_timer->list, &current->signal->posix_timers);
	spin_unlock_irq(&current->sighand->siglock);

	return 0;
out:
	release_posix_timer(new_timer, it_id_set);
	return error;
}

static struct k_itimer *__lock_timer(timer_t timer_id, unsigned long *flags)
{
	struct k_itimer *timr;

	rcu_read_lock();
	timr = idr_find(&posix_timers_id, (int)timer_id);
	if (timr) {
		spin_lock_irqsave(&timr->it_lock, *flags);
		if (timr->it_signal == current->signal) {
			rcu_read_unlock();
			return timr;
		}
		spin_unlock_irqrestore(&timr->it_lock, *flags);
	}
	rcu_read_unlock();

	return NULL;
}

static void
common_timer_get(struct k_itimer *timr, struct itimerspec *cur_setting)
{
	ktime_t now, remaining, iv;
	struct hrtimer *timer = &timr->it.real.timer;

	memset(cur_setting, 0, sizeof(struct itimerspec));

	iv = timr->it.real.interval;

	
	if (iv.tv64)
		cur_setting->it_interval = ktime_to_timespec(iv);
	else if (!hrtimer_active(timer) &&
		 (timr->it_sigev_notify & ~SIGEV_THREAD_ID) != SIGEV_NONE)
		return;

	now = timer->base->get_time();

	if (iv.tv64 && (timr->it_requeue_pending & REQUEUE_PENDING ||
	    (timr->it_sigev_notify & ~SIGEV_THREAD_ID) == SIGEV_NONE))
		timr->it_overrun += (unsigned int) hrtimer_forward(timer, now, iv);

	remaining = ktime_sub(hrtimer_get_expires(timer), now);
	
	if (remaining.tv64 <= 0) {
		if ((timr->it_sigev_notify & ~SIGEV_THREAD_ID) != SIGEV_NONE)
			cur_setting->it_value.tv_nsec = 1;
	} else
		cur_setting->it_value = ktime_to_timespec(remaining);
}

SYSCALL_DEFINE2(timer_gettime, timer_t, timer_id,
		struct itimerspec __user *, setting)
{
	struct itimerspec cur_setting;
	struct k_itimer *timr;
	struct k_clock *kc;
	unsigned long flags;
	int ret = 0;

	timr = lock_timer(timer_id, &flags);
	if (!timr)
		return -EINVAL;

	kc = clockid_to_kclock(timr->it_clock);
	if (WARN_ON_ONCE(!kc || !kc->timer_get))
		ret = -EINVAL;
	else
		kc->timer_get(timr, &cur_setting);

	unlock_timer(timr, flags);

	if (!ret && copy_to_user(setting, &cur_setting, sizeof (cur_setting)))
		return -EFAULT;

	return ret;
}

SYSCALL_DEFINE1(timer_getoverrun, timer_t, timer_id)
{
	struct k_itimer *timr;
	int overrun;
	unsigned long flags;

	timr = lock_timer(timer_id, &flags);
	if (!timr)
		return -EINVAL;

	overrun = timr->it_overrun_last;
	unlock_timer(timr, flags);

	return overrun;
}

static int
common_timer_set(struct k_itimer *timr, int flags,
		 struct itimerspec *new_setting, struct itimerspec *old_setting)
{
	struct hrtimer *timer = &timr->it.real.timer;
	enum hrtimer_mode mode;

	if (old_setting)
		common_timer_get(timr, old_setting);

	
	timr->it.real.interval.tv64 = 0;
	if (hrtimer_try_to_cancel(timer) < 0)
		return TIMER_RETRY;

	timr->it_requeue_pending = (timr->it_requeue_pending + 2) & 
		~REQUEUE_PENDING;
	timr->it_overrun_last = 0;

	
	if (!new_setting->it_value.tv_sec && !new_setting->it_value.tv_nsec)
		return 0;

	mode = flags & TIMER_ABSTIME ? HRTIMER_MODE_ABS : HRTIMER_MODE_REL;
	hrtimer_init(&timr->it.real.timer, timr->it_clock, mode);
	timr->it.real.timer.function = posix_timer_fn;

	hrtimer_set_expires(timer, timespec_to_ktime(new_setting->it_value));

	
	timr->it.real.interval = timespec_to_ktime(new_setting->it_interval);

	
	if (((timr->it_sigev_notify & ~SIGEV_THREAD_ID) == SIGEV_NONE)) {
		
		if (mode == HRTIMER_MODE_REL) {
			hrtimer_add_expires(timer, timer->base->get_time());
		}
		return 0;
	}

	hrtimer_start_expires(timer, mode);
	return 0;
}

SYSCALL_DEFINE4(timer_settime, timer_t, timer_id, int, flags,
		const struct itimerspec __user *, new_setting,
		struct itimerspec __user *, old_setting)
{
	struct k_itimer *timr;
	struct itimerspec new_spec, old_spec;
	int error = 0;
	unsigned long flag;
	struct itimerspec *rtn = old_setting ? &old_spec : NULL;
	struct k_clock *kc;

	if (!new_setting)
		return -EINVAL;

	if (copy_from_user(&new_spec, new_setting, sizeof (new_spec)))
		return -EFAULT;

	if (!timespec_valid(&new_spec.it_interval) ||
	    !timespec_valid(&new_spec.it_value))
		return -EINVAL;
retry:
	timr = lock_timer(timer_id, &flag);
	if (!timr)
		return -EINVAL;

	kc = clockid_to_kclock(timr->it_clock);
	if (WARN_ON_ONCE(!kc || !kc->timer_set))
		error = -EINVAL;
	else
		error = kc->timer_set(timr, flags, &new_spec, rtn);

	unlock_timer(timr, flag);
	if (error == TIMER_RETRY) {
		rtn = NULL;	
		goto retry;
	}

	if (old_setting && !error &&
	    copy_to_user(old_setting, &old_spec, sizeof (old_spec)))
		error = -EFAULT;

	return error;
}

static int common_timer_del(struct k_itimer *timer)
{
	timer->it.real.interval.tv64 = 0;

	if (hrtimer_try_to_cancel(&timer->it.real.timer) < 0)
		return TIMER_RETRY;
	return 0;
}

static inline int timer_delete_hook(struct k_itimer *timer)
{
	struct k_clock *kc = clockid_to_kclock(timer->it_clock);

	if (WARN_ON_ONCE(!kc || !kc->timer_del))
		return -EINVAL;
	return kc->timer_del(timer);
}

SYSCALL_DEFINE1(timer_delete, timer_t, timer_id)
{
	struct k_itimer *timer;
	unsigned long flags;

retry_delete:
	timer = lock_timer(timer_id, &flags);
	if (!timer)
		return -EINVAL;

	if (timer_delete_hook(timer) == TIMER_RETRY) {
		unlock_timer(timer, flags);
		goto retry_delete;
	}

	spin_lock(&current->sighand->siglock);
	list_del(&timer->list);
	spin_unlock(&current->sighand->siglock);
	timer->it_signal = NULL;

	unlock_timer(timer, flags);
	release_posix_timer(timer, IT_ID_SET);
	return 0;
}

static void itimer_delete(struct k_itimer *timer)
{
	unsigned long flags;

retry_delete:
	spin_lock_irqsave(&timer->it_lock, flags);

	if (timer_delete_hook(timer) == TIMER_RETRY) {
		unlock_timer(timer, flags);
		goto retry_delete;
	}
	list_del(&timer->list);
	timer->it_signal = NULL;

	unlock_timer(timer, flags);
	release_posix_timer(timer, IT_ID_SET);
}

void exit_itimers(struct signal_struct *sig)
{
	struct k_itimer *tmr;

	while (!list_empty(&sig->posix_timers)) {
		tmr = list_entry(sig->posix_timers.next, struct k_itimer, list);
		itimer_delete(tmr);
	}
}

SYSCALL_DEFINE2(clock_settime, const clockid_t, which_clock,
		const struct timespec __user *, tp)
{
	struct k_clock *kc = clockid_to_kclock(which_clock);
	struct timespec new_tp;

	if (!kc || !kc->clock_set)
		return -EINVAL;

	if (copy_from_user(&new_tp, tp, sizeof (*tp)))
		return -EFAULT;

	return kc->clock_set(which_clock, &new_tp);
}

SYSCALL_DEFINE2(clock_gettime, const clockid_t, which_clock,
		struct timespec __user *,tp)
{
	struct k_clock *kc = clockid_to_kclock(which_clock);
	struct timespec kernel_tp;
	int error;

	if (!kc)
		return -EINVAL;

	error = kc->clock_get(which_clock, &kernel_tp);

	if (!error && copy_to_user(tp, &kernel_tp, sizeof (kernel_tp)))
		error = -EFAULT;

	return error;
}

SYSCALL_DEFINE2(clock_adjtime, const clockid_t, which_clock,
		struct timex __user *, utx)
{
	struct k_clock *kc = clockid_to_kclock(which_clock);
	struct timex ktx;
	int err;

	if (!kc)
		return -EINVAL;
	if (!kc->clock_adj)
		return -EOPNOTSUPP;

	if (copy_from_user(&ktx, utx, sizeof(ktx)))
		return -EFAULT;

	err = kc->clock_adj(which_clock, &ktx);

	if (!err && copy_to_user(utx, &ktx, sizeof(ktx)))
		return -EFAULT;

	return err;
}

SYSCALL_DEFINE2(clock_getres, const clockid_t, which_clock,
		struct timespec __user *, tp)
{
	struct k_clock *kc = clockid_to_kclock(which_clock);
	struct timespec rtn_tp;
	int error;

	if (!kc)
		return -EINVAL;

	error = kc->clock_getres(which_clock, &rtn_tp);

	if (!error && tp && copy_to_user(tp, &rtn_tp, sizeof (rtn_tp)))
		error = -EFAULT;

	return error;
}

static int common_nsleep(const clockid_t which_clock, int flags,
			 struct timespec *tsave, struct timespec __user *rmtp)
{
	return hrtimer_nanosleep(tsave, rmtp, flags & TIMER_ABSTIME ?
				 HRTIMER_MODE_ABS : HRTIMER_MODE_REL,
				 which_clock);
}

SYSCALL_DEFINE4(clock_nanosleep, const clockid_t, which_clock, int, flags,
		const struct timespec __user *, rqtp,
		struct timespec __user *, rmtp)
{
	struct k_clock *kc = clockid_to_kclock(which_clock);
	struct timespec t;

	if (!kc)
		return -EINVAL;
	if (!kc->nsleep)
		return -ENANOSLEEP_NOTSUP;

	if (copy_from_user(&t, rqtp, sizeof (struct timespec)))
		return -EFAULT;

	if (!timespec_valid(&t))
		return -EINVAL;

	return kc->nsleep(which_clock, flags, &t, rmtp);
}

long clock_nanosleep_restart(struct restart_block *restart_block)
{
	clockid_t which_clock = restart_block->nanosleep.clockid;
	struct k_clock *kc = clockid_to_kclock(which_clock);

	if (WARN_ON_ONCE(!kc || !kc->nsleep_restart))
		return -EINVAL;

	return kc->nsleep_restart(restart_block);
}
