#undef TRACE_SYSTEM
#define TRACE_SYSTEM signal

#if !defined(_TRACE_SIGNAL_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_SIGNAL_H

#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/tracepoint.h>

#define TP_STORE_SIGINFO(__entry, info)				\
	do {							\
		if (info == SEND_SIG_NOINFO ||			\
		    info == SEND_SIG_FORCED) {			\
			__entry->errno	= 0;			\
			__entry->code	= SI_USER;		\
		} else if (info == SEND_SIG_PRIV) {		\
			__entry->errno	= 0;			\
			__entry->code	= SI_KERNEL;		\
		} else {					\
			__entry->errno	= info->si_errno;	\
			__entry->code	= info->si_code;	\
		}						\
	} while (0)

#ifndef TRACE_HEADER_MULTI_READ
enum {
	TRACE_SIGNAL_DELIVERED,
	TRACE_SIGNAL_IGNORED,
	TRACE_SIGNAL_ALREADY_PENDING,
	TRACE_SIGNAL_OVERFLOW_FAIL,
	TRACE_SIGNAL_LOSE_INFO,
};
#endif

TRACE_EVENT(signal_generate,

	TP_PROTO(int sig, struct siginfo *info, struct task_struct *task,
			int group, int result),

	TP_ARGS(sig, info, task, group, result),

	TP_STRUCT__entry(
		__field(	int,	sig			)
		__field(	int,	errno			)
		__field(	int,	code			)
		__array(	char,	comm,	TASK_COMM_LEN	)
		__field(	pid_t,	pid			)
		__field(	int,	group			)
		__field(	int,	result			)
	),

	TP_fast_assign(
		__entry->sig	= sig;
		TP_STORE_SIGINFO(__entry, info);
		memcpy(__entry->comm, task->comm, TASK_COMM_LEN);
		__entry->pid	= task->pid;
		__entry->group	= group;
		__entry->result	= result;
	),

	TP_printk("sig=%d errno=%d code=%d comm=%s pid=%d grp=%d res=%d",
		  __entry->sig, __entry->errno, __entry->code,
		  __entry->comm, __entry->pid, __entry->group,
		  __entry->result)
);

TRACE_EVENT(signal_deliver,

	TP_PROTO(int sig, struct siginfo *info, struct k_sigaction *ka),

	TP_ARGS(sig, info, ka),

	TP_STRUCT__entry(
		__field(	int,		sig		)
		__field(	int,		errno		)
		__field(	int,		code		)
		__field(	unsigned long,	sa_handler	)
		__field(	unsigned long,	sa_flags	)
	),

	TP_fast_assign(
		__entry->sig	= sig;
		TP_STORE_SIGINFO(__entry, info);
		__entry->sa_handler	= (unsigned long)ka->sa.sa_handler;
		__entry->sa_flags	= ka->sa.sa_flags;
	),

	TP_printk("sig=%d errno=%d code=%d sa_handler=%lx sa_flags=%lx",
		  __entry->sig, __entry->errno, __entry->code,
		  __entry->sa_handler, __entry->sa_flags)
);

#endif 

#include <trace/define_trace.h>
