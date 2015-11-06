
#ifndef __ASM_FIQ_H
#define __ASM_FIQ_H

#include <asm/ptrace.h>

struct fiq_handler {
	struct fiq_handler *next;
	const char *name;
	int (*fiq_op)(void *, int relinquish);
	void *dev_id;
};

#ifdef CONFIG_FIQ
extern int claim_fiq(struct fiq_handler *f);
extern void release_fiq(struct fiq_handler *f);
extern void set_fiq_handler(void *start, unsigned int length);
extern void enable_fiq(int fiq);
extern void disable_fiq(int fiq);
#else
static inline int claim_fiq(struct fiq_handler *f)
{
	return 0;
}
static inline void release_fiq(struct fiq_handler *f) { }
static inline void set_fiq_handler(void *start, unsigned int length) { }
static inline void enable_fiq(int fiq) { }
static inline void disable_fiq(int fiq) { }
#endif

extern void __set_fiq_regs(unsigned long const *regs);
extern void __get_fiq_regs(unsigned long *regs);

static inline void set_fiq_regs(struct pt_regs const *regs)
{
	__set_fiq_regs(&regs->ARM_r8);
}

static inline void get_fiq_regs(struct pt_regs *regs)
{
	__get_fiq_regs(&regs->ARM_r8);
}

#endif
