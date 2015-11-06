#ifndef _linux_POSIX_TIMERS_H
#define _linux_POSIX_TIMERS_H

#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/timex.h>
#include <linux/alarmtimer.h>

union cpu_time_count {
	cputime_t cpu;
	unsigned long long sched;
};

struct cpu_timer_list {
	struct list_head entry;
	union cpu_time_count expires, incr;
	struct task_struct *task;
	int firing;
};

#define CPUCLOCK_PID(clock)		((pid_t) ~((clock) >> 3))
#define CPUCLOCK_PERTHREAD(clock) \
	(((clock) & (clockid_t) CPUCLOCK_PERTHREAD_MASK) != 0)

#define CPUCLOCK_PERTHREAD_MASK	4
#define CPUCLOCK_WHICH(clock)	((clock) & (clockid_t) CPUCLOCK_CLOCK_MASK)
#define CPUCLOCK_CLOCK_MASK	3
#define CPUCLOCK_PROF		0
#define CPUCLOCK_VIRT		1
#define CPUCLOCK_SCHED		2
#define CPUCLOCK_MAX		3
#define CLOCKFD			CPUCLOCK_MAX
#define CLOCKFD_MASK		(CPUCLOCK_PERTHREAD_MASK|CPUCLOCK_CLOCK_MASK)

#define MAKE_PROCESS_CPUCLOCK(pid, clock) \
	((~(clockid_t) (pid) << 3) | (clockid_t) (clock))
#define MAKE_THREAD_CPUCLOCK(tid, clock) \
	MAKE_PROCESS_CPUCLOCK((tid), (clock) | CPUCLOCK_PERTHREAD_MASK)

#define FD_TO_CLOCKID(fd)	((~(clockid_t) (fd) << 3) | CLOCKFD)
#define CLOCKID_TO_FD(clk)	((unsigned int) ~((clk) >> 3))

struct k_itimer {
	struct list_head list;		
	spinlock_t it_lock;
	clockid_t it_clock;		
	timer_t it_id;			
	int it_overrun;			
	int it_overrun_last;		
	int it_requeue_pending;		
#define REQUEUE_PENDING 1
	int it_sigev_notify;		
	struct signal_struct *it_signal;
	union {
		struct pid *it_pid;	
		struct task_struct *it_process;	
	};
	struct sigqueue *sigq;		
	union {
		struct {
			struct hrtimer timer;
			ktime_t interval;
		} real;
		struct cpu_timer_list cpu;
		struct {
			unsigned int clock;
			unsigned int node;
			unsigned long incr;
			unsigned long expires;
		} mmtimer;
		struct {
			struct alarm alarmtimer;
			ktime_t interval;
		} alarm;
		struct rcu_head rcu;
	} it;
};

struct k_clock {
	int (*clock_getres) (const clockid_t which_clock, struct timespec *tp);
	int (*clock_set) (const clockid_t which_clock,
			  const struct timespec *tp);
	int (*clock_get) (const clockid_t which_clock, struct timespec * tp);
	int (*clock_adj) (const clockid_t which_clock, struct timex *tx);
	int (*timer_create) (struct k_itimer *timer);
	int (*nsleep) (const clockid_t which_clock, int flags,
		       struct timespec *, struct timespec __user *);
	long (*nsleep_restart) (struct restart_block *restart_block);
	int (*timer_set) (struct k_itimer * timr, int flags,
			  struct itimerspec * new_setting,
			  struct itimerspec * old_setting);
	int (*timer_del) (struct k_itimer * timr);
#define TIMER_RETRY 1
	void (*timer_get) (struct k_itimer * timr,
			   struct itimerspec * cur_setting);
};

extern struct k_clock clock_posix_cpu;
extern struct k_clock clock_posix_dynamic;

void posix_timers_register_clock(const clockid_t clock_id, struct k_clock *new_clock);

int posix_timer_event(struct k_itimer *timr, int si_private);

void posix_cpu_timer_schedule(struct k_itimer *timer);

void run_posix_cpu_timers(struct task_struct *task);
void posix_cpu_timers_exit(struct task_struct *task);
void posix_cpu_timers_exit_group(struct task_struct *task);

void set_process_cpu_timer(struct task_struct *task, unsigned int clock_idx,
			   cputime_t *newval, cputime_t *oldval);

long clock_nanosleep_restart(struct restart_block *restart_block);

void update_rlimit_cpu(struct task_struct *task, unsigned long rlim_new);

#endif
