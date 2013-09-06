#ifndef _LINUX_RCULIST_NULLS_H
#define _LINUX_RCULIST_NULLS_H

#ifdef __KERNEL__

#include <linux/list_nulls.h>
#include <linux/rcupdate.h>

static inline void hlist_nulls_del_init_rcu(struct hlist_nulls_node *n)
{
	if (!hlist_nulls_unhashed(n)) {
		__hlist_nulls_del(n);
		n->pprev = NULL;
	}
}

#define hlist_nulls_first_rcu(head) \
	(*((struct hlist_nulls_node __rcu __force **)&(head)->first))

#define hlist_nulls_next_rcu(node) \
	(*((struct hlist_nulls_node __rcu __force **)&(node)->next))

static inline void hlist_nulls_del_rcu(struct hlist_nulls_node *n)
{
	__hlist_nulls_del(n);
	n->pprev = LIST_POISON2;
}

static inline void hlist_nulls_add_head_rcu(struct hlist_nulls_node *n,
					struct hlist_nulls_head *h)
{
	struct hlist_nulls_node *first = h->first;

	n->next = first;
	n->pprev = &h->first;
	rcu_assign_pointer(hlist_nulls_first_rcu(h), n);
	if (!is_a_nulls(first))
		first->pprev = &n->next;
}
/**
 * hlist_nulls_for_each_entry_rcu - iterate over rcu list of given type
 * @tpos:	the type * to use as a loop cursor.
 * @pos:	the &struct hlist_nulls_node to use as a loop cursor.
 * @head:	the head for your list.
 * @member:	the name of the hlist_nulls_node within the struct.
 *
 * The barrier() is needed to make sure compiler doesn't cache first element [1],
 * as this loop can be restarted [2]
 * [1] Documentation/atomic_ops.txt around line 114
 * [2] Documentation/RCU/rculist_nulls.txt around line 146
 */
#define hlist_nulls_for_each_entry_rcu(tpos, pos, head, member)			\
	for (({barrier();}),							\
	     pos = rcu_dereference_raw(hlist_nulls_first_rcu(head));		\
		(!is_a_nulls(pos)) &&						\
		({ tpos = hlist_nulls_entry(pos, typeof(*tpos), member); 1; }); \
		pos = rcu_dereference_raw(hlist_nulls_next_rcu(pos)))

#endif
#endif
