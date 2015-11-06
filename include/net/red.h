#ifndef __NET_SCHED_RED_H
#define __NET_SCHED_RED_H

#include <linux/types.h>
#include <linux/bug.h>
#include <net/pkt_sched.h>
#include <net/inet_ecn.h>
#include <net/dsfield.h>
#include <linux/reciprocal_div.h>


#define RED_ONE_PERCENT ((u32)DIV_ROUND_CLOSEST(1ULL<<32, 100))

#define MAX_P_MIN (1 * RED_ONE_PERCENT)
#define MAX_P_MAX (50 * RED_ONE_PERCENT)
#define MAX_P_ALPHA(val) min(MAX_P_MIN, val / 4)

#define RED_STAB_SIZE	256
#define RED_STAB_MASK	(RED_STAB_SIZE - 1)

struct red_stats {
	u32		prob_drop;	
	u32		prob_mark;	
	u32		forced_drop;	
	u32		forced_mark;	
	u32		pdrop;          
	u32		other;          
};

struct red_parms {
	
	u32		qth_min;	
	u32		qth_max;	
	u32		Scell_max;
	u32		max_P;		
	u32		max_P_reciprocal; 
	u32		qth_delta;	
	u32		target_min;	
	u32		target_max;	
	u8		Scell_log;
	u8		Wlog;		
	u8		Plog;		
	u8		Stab[RED_STAB_SIZE];
};

struct red_vars {
	
	int		qcount;		
	u32		qR;		

	unsigned long	qavg;		
	ktime_t		qidlestart;	
};

static inline u32 red_maxp(u8 Plog)
{
	return Plog < 32 ? (~0U >> Plog) : ~0U;
}

static inline void red_set_vars(struct red_vars *v)
{
	v->qavg		= 0;

	v->qcount	= -1;
}

static inline void red_set_parms(struct red_parms *p,
				 u32 qth_min, u32 qth_max, u8 Wlog, u8 Plog,
				 u8 Scell_log, u8 *stab, u32 max_P)
{
	int delta = qth_max - qth_min;
	u32 max_p_delta;

	p->qth_min	= qth_min << Wlog;
	p->qth_max	= qth_max << Wlog;
	p->Wlog		= Wlog;
	p->Plog		= Plog;
	if (delta < 0)
		delta = 1;
	p->qth_delta	= delta;
	if (!max_P) {
		max_P = red_maxp(Plog);
		max_P *= delta; 
	}
	p->max_P = max_P;
	max_p_delta = max_P / delta;
	max_p_delta = max(max_p_delta, 1U);
	p->max_P_reciprocal  = reciprocal_value(max_p_delta);

	delta /= 5;
	p->target_min = qth_min + 2*delta;
	p->target_max = qth_min + 3*delta;

	p->Scell_log	= Scell_log;
	p->Scell_max	= (255 << Scell_log);

	if (stab)
		memcpy(p->Stab, stab, sizeof(p->Stab));
}

static inline int red_is_idling(const struct red_vars *v)
{
	return v->qidlestart.tv64 != 0;
}

static inline void red_start_of_idle_period(struct red_vars *v)
{
	v->qidlestart = ktime_get();
}

static inline void red_end_of_idle_period(struct red_vars *v)
{
	v->qidlestart.tv64 = 0;
}

static inline void red_restart(struct red_vars *v)
{
	red_end_of_idle_period(v);
	v->qavg = 0;
	v->qcount = -1;
}

static inline unsigned long red_calc_qavg_from_idle_time(const struct red_parms *p,
							 const struct red_vars *v)
{
	s64 delta = ktime_us_delta(ktime_get(), v->qidlestart);
	long us_idle = min_t(s64, delta, p->Scell_max);
	int  shift;


	shift = p->Stab[(us_idle >> p->Scell_log) & RED_STAB_MASK];

	if (shift)
		return v->qavg >> shift;
	else {
		us_idle = (v->qavg * (u64)us_idle) >> p->Scell_log;

		if (us_idle < (v->qavg >> 1))
			return v->qavg - us_idle;
		else
			return v->qavg >> 1;
	}
}

static inline unsigned long red_calc_qavg_no_idle_time(const struct red_parms *p,
						       const struct red_vars *v,
						       unsigned int backlog)
{
	return v->qavg + (backlog - (v->qavg >> p->Wlog));
}

static inline unsigned long red_calc_qavg(const struct red_parms *p,
					  const struct red_vars *v,
					  unsigned int backlog)
{
	if (!red_is_idling(v))
		return red_calc_qavg_no_idle_time(p, v, backlog);
	else
		return red_calc_qavg_from_idle_time(p, v);
}


static inline u32 red_random(const struct red_parms *p)
{
	return reciprocal_divide(net_random(), p->max_P_reciprocal);
}

static inline int red_mark_probability(const struct red_parms *p,
				       const struct red_vars *v,
				       unsigned long qavg)
{
	return !(((qavg - p->qth_min) >> p->Wlog) * v->qcount < v->qR);
}

enum {
	RED_BELOW_MIN_THRESH,
	RED_BETWEEN_TRESH,
	RED_ABOVE_MAX_TRESH,
};

static inline int red_cmp_thresh(const struct red_parms *p, unsigned long qavg)
{
	if (qavg < p->qth_min)
		return RED_BELOW_MIN_THRESH;
	else if (qavg >= p->qth_max)
		return RED_ABOVE_MAX_TRESH;
	else
		return RED_BETWEEN_TRESH;
}

enum {
	RED_DONT_MARK,
	RED_PROB_MARK,
	RED_HARD_MARK,
};

static inline int red_action(const struct red_parms *p,
			     struct red_vars *v,
			     unsigned long qavg)
{
	switch (red_cmp_thresh(p, qavg)) {
		case RED_BELOW_MIN_THRESH:
			v->qcount = -1;
			return RED_DONT_MARK;

		case RED_BETWEEN_TRESH:
			if (++v->qcount) {
				if (red_mark_probability(p, v, qavg)) {
					v->qcount = 0;
					v->qR = red_random(p);
					return RED_PROB_MARK;
				}
			} else
				v->qR = red_random(p);

			return RED_DONT_MARK;

		case RED_ABOVE_MAX_TRESH:
			v->qcount = -1;
			return RED_HARD_MARK;
	}

	BUG();
	return RED_DONT_MARK;
}

static inline void red_adaptative_algo(struct red_parms *p, struct red_vars *v)
{
	unsigned long qavg;
	u32 max_p_delta;

	qavg = v->qavg;
	if (red_is_idling(v))
		qavg = red_calc_qavg_from_idle_time(p, v);

	
	qavg >>= p->Wlog;

	if (qavg > p->target_max && p->max_P <= MAX_P_MAX)
		p->max_P += MAX_P_ALPHA(p->max_P); 
	else if (qavg < p->target_min && p->max_P >= MAX_P_MIN)
		p->max_P = (p->max_P/10)*9; 

	max_p_delta = DIV_ROUND_CLOSEST(p->max_P, p->qth_delta);
	max_p_delta = max(max_p_delta, 1U);
	p->max_P_reciprocal = reciprocal_value(max_p_delta);
}
#endif
