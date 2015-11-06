#ifndef IOPRIO_H
#define IOPRIO_H

#include <linux/sched.h>
#include <linux/iocontext.h>

#define IOPRIO_BITS		(16)
#define IOPRIO_CLASS_SHIFT	(13)
#define IOPRIO_PRIO_MASK	((1UL << IOPRIO_CLASS_SHIFT) - 1)

#define IOPRIO_PRIO_CLASS(mask)	((mask) >> IOPRIO_CLASS_SHIFT)
#define IOPRIO_PRIO_DATA(mask)	((mask) & IOPRIO_PRIO_MASK)
#define IOPRIO_PRIO_VALUE(class, data)	(((class) << IOPRIO_CLASS_SHIFT) | data)

#define ioprio_valid(mask)	(IOPRIO_PRIO_CLASS((mask)) != IOPRIO_CLASS_NONE)

enum {
	IOPRIO_CLASS_NONE,
	IOPRIO_CLASS_RT,
	IOPRIO_CLASS_BE,
	IOPRIO_CLASS_IDLE,
};

#define IOPRIO_BE_NR	(8)

enum {
	IOPRIO_WHO_PROCESS = 1,
	IOPRIO_WHO_PGRP,
	IOPRIO_WHO_USER,
};

#define IOPRIO_NORM	(4)
static inline int task_ioprio(struct io_context *ioc)
{
	if (ioprio_valid(ioc->ioprio))
		return IOPRIO_PRIO_DATA(ioc->ioprio);

	return IOPRIO_NORM;
}

static inline int task_ioprio_class(struct io_context *ioc)
{
	if (ioprio_valid(ioc->ioprio))
		return IOPRIO_PRIO_CLASS(ioc->ioprio);

	return IOPRIO_CLASS_BE;
}

static inline int task_nice_ioprio(struct task_struct *task)
{
	return (task_nice(task) + 20) / 5;
}

static inline int task_nice_ioclass(struct task_struct *task)
{
	if (task->policy == SCHED_IDLE)
		return IOPRIO_CLASS_IDLE;
	else if (task->policy == SCHED_FIFO || task->policy == SCHED_RR)
		return IOPRIO_CLASS_RT;
	else
		return IOPRIO_CLASS_BE;
}

extern int ioprio_best(unsigned short aprio, unsigned short bprio);

extern int set_task_ioprio(struct task_struct *task, int ioprio);

#endif
