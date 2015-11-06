#ifndef _LINUX_MODULE_PARAMS_H
#define _LINUX_MODULE_PARAMS_H
/* (C) Copyright 2001, 2002 Rusty Russell IBM Corporation */
#include <linux/init.h>
#include <linux/stringify.h>
#include <linux/kernel.h>

#ifdef MODULE
#define MODULE_PARAM_PREFIX 
#else
#define MODULE_PARAM_PREFIX KBUILD_MODNAME "."
#endif

#define MAX_PARAM_PREFIX_LEN (64 - sizeof(unsigned long))

#define ___module_cat(a,b) __mod_ ## a ## b
#define __module_cat(a,b) ___module_cat(a,b)
#ifdef MODULE
#define __MODULE_INFO(tag, name, info)					  \
static const char __module_cat(name,__LINE__)[]				  \
  __used __attribute__((section(".modinfo"), unused, aligned(1)))	  \
  = __stringify(tag) "=" info
#else  
#define __MODULE_INFO(tag, name, info)					  \
  struct __module_cat(name,__LINE__) {}
#endif
#define __MODULE_PARM_TYPE(name, _type)					  \
  __MODULE_INFO(parmtype, name##type, #name ":" _type)

#define MODULE_PARM_DESC(_parm, desc) \
	__MODULE_INFO(parm, _parm, #_parm ":" desc)

struct kernel_param;

struct kernel_param_ops {
	
	int (*set)(const char *val, const struct kernel_param *kp);
	
	int (*get)(char *buffer, const struct kernel_param *kp);
	
	void (*free)(void *arg);
};

struct kernel_param {
	const char *name;
	const struct kernel_param_ops *ops;
	u16 perm;
	s16 level;
	union {
		void *arg;
		const struct kparam_string *str;
		const struct kparam_array *arr;
	};
};

struct kparam_string {
	unsigned int maxlen;
	char *string;
};

struct kparam_array
{
	unsigned int max;
	unsigned int elemsize;
	unsigned int *num;
	const struct kernel_param_ops *ops;
	void *elem;
};

#define module_param(name, type, perm)				\
	module_param_named(name, name, type, perm)

#define module_param_named(name, value, type, perm)			   \
	param_check_##type(name, &(value));				   \
	module_param_cb(name, &param_ops_##type, &value, perm);		   \
	__MODULE_PARM_TYPE(name, #type)

#define module_param_cb(name, ops, arg, perm)				      \
	__module_param_call(MODULE_PARAM_PREFIX, name, ops, arg, perm, -1)

#define __level_param_cb(name, ops, arg, perm, level)			\
	__module_param_call(MODULE_PARAM_PREFIX, name, ops, arg, perm, level)

#define core_param_cb(name, ops, arg, perm)		\
	__level_param_cb(name, ops, arg, perm, 1)

#define postcore_param_cb(name, ops, arg, perm)		\
	__level_param_cb(name, ops, arg, perm, 2)

#define arch_param_cb(name, ops, arg, perm)		\
	__level_param_cb(name, ops, arg, perm, 3)

#define subsys_param_cb(name, ops, arg, perm)		\
	__level_param_cb(name, ops, arg, perm, 4)

#define fs_param_cb(name, ops, arg, perm)		\
	__level_param_cb(name, ops, arg, perm, 5)

#define device_param_cb(name, ops, arg, perm)		\
	__level_param_cb(name, ops, arg, perm, 6)

#define late_param_cb(name, ops, arg, perm)		\
	__level_param_cb(name, ops, arg, perm, 7)

#if defined(CONFIG_ALPHA) || defined(CONFIG_IA64) || defined(CONFIG_PPC64)
#define __moduleparam_const
#else
#define __moduleparam_const const
#endif

#define __module_param_call(prefix, name, ops, arg, perm, level)	\
				\
	static int __param_perm_check_##name __attribute__((unused)) =	\
	BUILD_BUG_ON_ZERO((perm) < 0 || (perm) > 0777 || ((perm) & 2))	\
	+ BUILD_BUG_ON_ZERO(sizeof(""prefix) > MAX_PARAM_PREFIX_LEN);	\
	static const char __param_str_##name[] = prefix #name;		\
	static struct kernel_param __moduleparam_const __param_##name	\
	__used								\
    __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) \
	= { __param_str_##name, ops, perm, level, { arg } }

#define module_param_call(name, set, get, arg, perm)			\
	static struct kernel_param_ops __param_ops_##name =		\
		 { (void *)set, (void *)get };				\
	__module_param_call(MODULE_PARAM_PREFIX,			\
			    name, &__param_ops_##name, arg,		\
			    (perm) + sizeof(__check_old_set_param(set))*0, -1)

static inline int
__check_old_set_param(int (*oldset)(const char *, struct kernel_param *))
{
	return 0;
}

#define kparam_block_sysfs_write(name)			\
	do {						\
		BUG_ON(!(__param_##name.perm & 0222));	\
		__kernel_param_lock();			\
	} while (0)

#define kparam_unblock_sysfs_write(name)		\
	do {						\
		BUG_ON(!(__param_##name.perm & 0222));	\
		__kernel_param_unlock();		\
	} while (0)

#define kparam_block_sysfs_read(name)			\
	do {						\
		BUG_ON(!(__param_##name.perm & 0444));	\
		__kernel_param_lock();			\
	} while (0)

#define kparam_unblock_sysfs_read(name)			\
	do {						\
		BUG_ON(!(__param_##name.perm & 0444));	\
		__kernel_param_unlock();		\
	} while (0)

#ifdef CONFIG_SYSFS
extern void __kernel_param_lock(void);
extern void __kernel_param_unlock(void);
#else
static inline void __kernel_param_lock(void)
{
}
static inline void __kernel_param_unlock(void)
{
}
#endif

#ifndef MODULE
#define core_param(name, var, type, perm)				\
	param_check_##type(name, &(var));				\
	__module_param_call("", name, &param_ops_##type, &var, perm, -1)
#endif 

#define module_param_string(name, string, len, perm)			\
	static const struct kparam_string __param_string_##name		\
		= { len, string };					\
	__module_param_call(MODULE_PARAM_PREFIX, name,			\
			    &param_ops_string,				\
			    .str = &__param_string_##name, perm, -1);	\
	__MODULE_PARM_TYPE(name, "string")

extern bool parameq(const char *name1, const char *name2);

extern bool parameqn(const char *name1, const char *name2, size_t n);

extern int parse_args(const char *name,
		      char *args,
		      const struct kernel_param *params,
		      unsigned num,
		      s16 level_min,
		      s16 level_max,
		      int (*unknown)(char *param, char *val));

#ifdef CONFIG_SYSFS
extern void destroy_params(const struct kernel_param *params, unsigned num);
#else
static inline void destroy_params(const struct kernel_param *params,
				  unsigned num)
{
}
#endif 

#define __param_check(name, p, type) \
	static inline type *__check_##name(void) { return(p); }

extern struct kernel_param_ops param_ops_byte;
extern int param_set_byte(const char *val, const struct kernel_param *kp);
extern int param_get_byte(char *buffer, const struct kernel_param *kp);
#define param_check_byte(name, p) __param_check(name, p, unsigned char)

extern struct kernel_param_ops param_ops_short;
extern int param_set_short(const char *val, const struct kernel_param *kp);
extern int param_get_short(char *buffer, const struct kernel_param *kp);
#define param_check_short(name, p) __param_check(name, p, short)

extern struct kernel_param_ops param_ops_ushort;
extern int param_set_ushort(const char *val, const struct kernel_param *kp);
extern int param_get_ushort(char *buffer, const struct kernel_param *kp);
#define param_check_ushort(name, p) __param_check(name, p, unsigned short)

extern struct kernel_param_ops param_ops_int;
extern int param_set_int(const char *val, const struct kernel_param *kp);
extern int param_get_int(char *buffer, const struct kernel_param *kp);
#define param_check_int(name, p) __param_check(name, p, int)

extern struct kernel_param_ops param_ops_uint;
extern int param_set_uint(const char *val, const struct kernel_param *kp);
extern int param_get_uint(char *buffer, const struct kernel_param *kp);
#define param_check_uint(name, p) __param_check(name, p, unsigned int)

extern struct kernel_param_ops param_ops_long;
extern int param_set_long(const char *val, const struct kernel_param *kp);
extern int param_get_long(char *buffer, const struct kernel_param *kp);
#define param_check_long(name, p) __param_check(name, p, long)

extern struct kernel_param_ops param_ops_ulong;
extern int param_set_ulong(const char *val, const struct kernel_param *kp);
extern int param_get_ulong(char *buffer, const struct kernel_param *kp);
#define param_check_ulong(name, p) __param_check(name, p, unsigned long)

extern struct kernel_param_ops param_ops_charp;
extern int param_set_charp(const char *val, const struct kernel_param *kp);
extern int param_get_charp(char *buffer, const struct kernel_param *kp);
#define param_check_charp(name, p) __param_check(name, p, char *)

extern struct kernel_param_ops param_ops_bool;
extern int param_set_bool(const char *val, const struct kernel_param *kp);
extern int param_get_bool(char *buffer, const struct kernel_param *kp);
#define param_check_bool(name, p) __param_check(name, p, bool)

extern struct kernel_param_ops param_ops_invbool;
extern int param_set_invbool(const char *val, const struct kernel_param *kp);
extern int param_get_invbool(char *buffer, const struct kernel_param *kp);
#define param_check_invbool(name, p) __param_check(name, p, bool)

extern struct kernel_param_ops param_ops_bint;
extern int param_set_bint(const char *val, const struct kernel_param *kp);
#define param_get_bint param_get_int
#define param_check_bint param_check_int

#define module_param_array(name, type, nump, perm)		\
	module_param_array_named(name, name, type, nump, perm)

#define module_param_array_named(name, array, type, nump, perm)		\
	param_check_##type(name, &(array)[0]);				\
	static const struct kparam_array __param_arr_##name		\
	= { .max = ARRAY_SIZE(array), .num = nump,                      \
	    .ops = &param_ops_##type,					\
	    .elemsize = sizeof(array[0]), .elem = array };		\
	__module_param_call(MODULE_PARAM_PREFIX, name,			\
			    &param_array_ops,				\
			    .arr = &__param_arr_##name,			\
			    perm, -1);					\
	__MODULE_PARM_TYPE(name, "array of " #type)

extern struct kernel_param_ops param_array_ops;

extern struct kernel_param_ops param_ops_string;
extern int param_set_copystring(const char *val, const struct kernel_param *);
extern int param_get_string(char *buffer, const struct kernel_param *kp);


struct module;

#if defined(CONFIG_SYSFS) && defined(CONFIG_MODULES)
extern int module_param_sysfs_setup(struct module *mod,
				    const struct kernel_param *kparam,
				    unsigned int num_params);

extern void module_param_sysfs_remove(struct module *mod);
#else
static inline int module_param_sysfs_setup(struct module *mod,
			     const struct kernel_param *kparam,
			     unsigned int num_params)
{
	return 0;
}

static inline void module_param_sysfs_remove(struct module *mod)
{ }
#endif

#endif 
