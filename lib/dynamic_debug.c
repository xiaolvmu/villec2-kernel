/*
 * lib/dynamic_debug.c
 *
 * make pr_debug()/dev_dbg() calls runtime configurable based upon their
 * source module.
 *
 * Copyright (C) 2008 Jason Baron <jbaron@redhat.com>
 * By Greg Banks <gnb@melbourne.sgi.com>
 * Copyright (c) 2008 Silicon Graphics Inc.  All Rights Reserved.
 * Copyright (C) 2011 Bart Van Assche.  All Rights Reserved.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kallsyms.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/list.h>
#include <linux/sysctl.h>
#include <linux/ctype.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/dynamic_debug.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/jump_label.h>
#include <linux/hardirq.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/netdevice.h>

extern struct _ddebug __start___verbose[];
extern struct _ddebug __stop___verbose[];

struct ddebug_table {
	struct list_head link;
	char *mod_name;
	unsigned int num_ddebugs;
	struct _ddebug *ddebugs;
};

struct ddebug_query {
	const char *filename;
	const char *module;
	const char *function;
	const char *format;
	unsigned int first_lineno, last_lineno;
};

struct ddebug_iter {
	struct ddebug_table *table;
	unsigned int idx;
};

static DEFINE_MUTEX(ddebug_lock);
static LIST_HEAD(ddebug_tables);
static int verbose = 0;
module_param(verbose, int, 0644);

static inline const char *basename(const char *path)
{
	const char *tail = strrchr(path, '/');
	return tail ? tail+1 : path;
}

static inline const char *trim_prefix(const char *path)
{
	int skip = strlen(__FILE__) - strlen("lib/dynamic_debug.c");

	if (strncmp(path, __FILE__, skip))
		skip = 0; 

	return path + skip;
}

static struct { unsigned flag:8; char opt_char; } opt_array[] = {
	{ _DPRINTK_FLAGS_PRINT, 'p' },
	{ _DPRINTK_FLAGS_INCL_MODNAME, 'm' },
	{ _DPRINTK_FLAGS_INCL_FUNCNAME, 'f' },
	{ _DPRINTK_FLAGS_INCL_LINENO, 'l' },
	{ _DPRINTK_FLAGS_INCL_TID, 't' },
	{ _DPRINTK_FLAGS_NONE, '_' },
};

static char *ddebug_describe_flags(struct _ddebug *dp, char *buf,
				    size_t maxlen)
{
	char *p = buf;
	int i;

	BUG_ON(maxlen < 6);
	for (i = 0; i < ARRAY_SIZE(opt_array); ++i)
		if (dp->flags & opt_array[i].flag)
			*p++ = opt_array[i].opt_char;
	if (p == buf)
		*p++ = '_';
	*p = '\0';

	return buf;
}

#define vpr_info_dq(q, msg)						\
do {									\
	if (verbose)							\
					\
		pr_info("%s: func=\"%s\" file=\"%s\" "			\
			"module=\"%s\" format=\"%.*s\" "		\
			"lineno=%u-%u",					\
			msg,						\
			q->function ? q->function : "",			\
			q->filename ? q->filename : "",			\
			q->module ? q->module : "",			\
			(int)(q->format ? strlen(q->format) - 1 : 0),	\
			q->format ? q->format : "",			\
			q->first_lineno, q->last_lineno);		\
} while (0)

static int ddebug_change(const struct ddebug_query *query,
			unsigned int flags, unsigned int mask)
{
	int i;
	struct ddebug_table *dt;
	unsigned int newflags;
	unsigned int nfound = 0;
	char flagbuf[10];

	
	mutex_lock(&ddebug_lock);
	list_for_each_entry(dt, &ddebug_tables, link) {

		
		if (query->module && strcmp(query->module, dt->mod_name))
			continue;

		for (i = 0 ; i < dt->num_ddebugs ; i++) {
			struct _ddebug *dp = &dt->ddebugs[i];

			
			if (query->filename &&
			    strcmp(query->filename, dp->filename) &&
			    strcmp(query->filename, basename(dp->filename)) &&
			    strcmp(query->filename, trim_prefix(dp->filename)))
				continue;

			
			if (query->function &&
			    strcmp(query->function, dp->function))
				continue;

			
			if (query->format &&
			    !strstr(dp->format, query->format))
				continue;

			
			if (query->first_lineno &&
			    dp->lineno < query->first_lineno)
				continue;
			if (query->last_lineno &&
			    dp->lineno > query->last_lineno)
				continue;

			nfound++;

			newflags = (dp->flags & mask) | flags;
			if (newflags == dp->flags)
				continue;
			dp->flags = newflags;
			if (verbose)
				pr_info("changed %s:%d [%s]%s =%s\n",
					trim_prefix(dp->filename), dp->lineno,
					dt->mod_name, dp->function,
					ddebug_describe_flags(dp, flagbuf,
							sizeof(flagbuf)));
		}
	}
	mutex_unlock(&ddebug_lock);

	if (!nfound && verbose)
		pr_info("no matches for query\n");

	return nfound;
}

static int ddebug_tokenize(char *buf, char *words[], int maxwords)
{
	int nwords = 0;

	while (*buf) {
		char *end;

		
		buf = skip_spaces(buf);
		if (!*buf)
			break;	
		if (*buf == '#')
			break;	

		
		if (*buf == '"' || *buf == '\'') {
			int quote = *buf++;
			for (end = buf ; *end && *end != quote ; end++)
				;
			if (!*end)
				return -EINVAL;	
		} else {
			for (end = buf ; *end && !isspace(*end) ; end++)
				;
			BUG_ON(end == buf);
		}

		
		if (nwords == maxwords)
			return -EINVAL;	
		if (*end)
			*end++ = '\0';	
		words[nwords++] = buf;
		buf = end;
	}

	if (verbose) {
		int i;
		pr_info("split into words:");
		for (i = 0 ; i < nwords ; i++)
			pr_cont(" \"%s\"", words[i]);
		pr_cont("\n");
	}

	return nwords;
}

static inline int parse_lineno(const char *str, unsigned int *val)
{
	char *end = NULL;
	BUG_ON(str == NULL);
	if (*str == '\0') {
		*val = 0;
		return 0;
	}
	*val = simple_strtoul(str, &end, 10);
	return end == NULL || end == str || *end != '\0' ? -EINVAL : 0;
}

#define isodigit(c)		((c) >= '0' && (c) <= '7')
static char *unescape(char *str)
{
	char *in = str;
	char *out = str;

	while (*in) {
		if (*in == '\\') {
			if (in[1] == '\\') {
				*out++ = '\\';
				in += 2;
				continue;
			} else if (in[1] == 't') {
				*out++ = '\t';
				in += 2;
				continue;
			} else if (in[1] == 'n') {
				*out++ = '\n';
				in += 2;
				continue;
			} else if (isodigit(in[1]) &&
			         isodigit(in[2]) &&
			         isodigit(in[3])) {
				*out++ = ((in[1] - '0')<<6) |
				          ((in[2] - '0')<<3) |
				          (in[3] - '0');
				in += 4;
				continue;
			}
		}
		*out++ = *in++;
	}
	*out = '\0';

	return str;
}

static int check_set(const char **dest, char *src, char *name)
{
	int rc = 0;

	if (*dest) {
		rc = -EINVAL;
		pr_err("match-spec:%s val:%s overridden by %s",
			name, *dest, src);
	}
	*dest = src;
	return rc;
}

static int ddebug_parse_query(char *words[], int nwords,
			       struct ddebug_query *query)
{
	unsigned int i;
	int rc;

	
	if (nwords % 2 != 0)
		return -EINVAL;
	memset(query, 0, sizeof(*query));

	for (i = 0 ; i < nwords ; i += 2) {
		if (!strcmp(words[i], "func"))
			rc = check_set(&query->function, words[i+1], "func");
		else if (!strcmp(words[i], "file"))
			rc = check_set(&query->filename, words[i+1], "file");
		else if (!strcmp(words[i], "module"))
			rc = check_set(&query->module, words[i+1], "module");
		else if (!strcmp(words[i], "format"))
			rc = check_set(&query->format, unescape(words[i+1]),
				"format");
		else if (!strcmp(words[i], "line")) {
			char *first = words[i+1];
			char *last = strchr(first, '-');
			if (query->first_lineno || query->last_lineno) {
				pr_err("match-spec:line given 2 times\n");
				return -EINVAL;
			}
			if (last)
				*last++ = '\0';
			if (parse_lineno(first, &query->first_lineno) < 0)
				return -EINVAL;
			if (last) {
				
				if (parse_lineno(last, &query->last_lineno)
				    < query->first_lineno) {
					pr_err("last-line < 1st-line\n");
					return -EINVAL;
				}
			} else {
				query->last_lineno = query->first_lineno;
			}
		} else {
			pr_err("unknown keyword \"%s\"\n", words[i]);
			return -EINVAL;
		}
		if (rc)
			return rc;
	}
	vpr_info_dq(query, "parsed");
	return 0;
}

static int ddebug_parse_flags(const char *str, unsigned int *flagsp,
			       unsigned int *maskp)
{
	unsigned flags = 0;
	int op = '=', i;

	switch (*str) {
	case '+':
	case '-':
	case '=':
		op = *str++;
		break;
	default:
		return -EINVAL;
	}
	if (verbose)
		pr_info("op='%c'\n", op);

	for ( ; *str ; ++str) {
		for (i = ARRAY_SIZE(opt_array) - 1; i >= 0; i--) {
			if (*str == opt_array[i].opt_char) {
				flags |= opt_array[i].flag;
				break;
			}
		}
		if (i < 0)
			return -EINVAL;
	}
	if (verbose)
		pr_info("flags=0x%x\n", flags);

	
	switch (op) {
	case '=':
		*maskp = 0;
		*flagsp = flags;
		break;
	case '+':
		*maskp = ~0U;
		*flagsp = flags;
		break;
	case '-':
		*maskp = ~flags;
		*flagsp = 0;
		break;
	}
	if (verbose)
		pr_info("*flagsp=0x%x *maskp=0x%x\n", *flagsp, *maskp);
	return 0;
}

static int ddebug_exec_query(char *query_string)
{
	unsigned int flags = 0, mask = 0;
	struct ddebug_query query;
#define MAXWORDS 9
	int nwords, nfound;
	char *words[MAXWORDS];

	nwords = ddebug_tokenize(query_string, words, MAXWORDS);
	if (nwords <= 0)
		return -EINVAL;
	if (ddebug_parse_query(words, nwords-1, &query))
		return -EINVAL;
	if (ddebug_parse_flags(words[nwords-1], &flags, &mask))
		return -EINVAL;

	
	nfound = ddebug_change(&query, flags, mask);
	vpr_info_dq((&query), (nfound) ? "applied" : "no-match");

	return nfound;
}

static int ddebug_exec_queries(char *query)
{
	char *split;
	int i, errs = 0, exitcode = 0, rc, nfound = 0;

	for (i = 0; query; query = split) {
		split = strpbrk(query, ";\n");
		if (split)
			*split++ = '\0';

		query = skip_spaces(query);
		if (!query || !*query || *query == '#')
			continue;

		if (verbose)
			pr_info("query %d: \"%s\"\n", i, query);

		rc = ddebug_exec_query(query);
		if (rc < 0) {
			errs++;
			exitcode = rc;
		} else
			nfound += rc;
		i++;
	}
	pr_info("processed %d queries, with %d matches, %d errs\n",
		 i, nfound, errs);

	if (exitcode)
		return exitcode;
	return nfound;
}

#define PREFIX_SIZE 64

static int remaining(int wrote)
{
	if (PREFIX_SIZE - wrote > 0)
		return PREFIX_SIZE - wrote;
	return 0;
}

static char *dynamic_emit_prefix(const struct _ddebug *desc, char *buf)
{
	int pos_after_tid;
	int pos = 0;

	pos += snprintf(buf + pos, remaining(pos), "%s", KERN_DEBUG);
	if (desc->flags & _DPRINTK_FLAGS_INCL_TID) {
		if (in_interrupt())
			pos += snprintf(buf + pos, remaining(pos), "%s ",
						"<intr>");
		else
			pos += snprintf(buf + pos, remaining(pos), "[%d] ",
						task_pid_vnr(current));
	}
	pos_after_tid = pos;
	if (desc->flags & _DPRINTK_FLAGS_INCL_MODNAME)
		pos += snprintf(buf + pos, remaining(pos), "%s:",
					desc->modname);
	if (desc->flags & _DPRINTK_FLAGS_INCL_FUNCNAME)
		pos += snprintf(buf + pos, remaining(pos), "%s:",
					desc->function);
	if (desc->flags & _DPRINTK_FLAGS_INCL_LINENO)
		pos += snprintf(buf + pos, remaining(pos), "%d:",
					desc->lineno);
	if (pos - pos_after_tid)
		pos += snprintf(buf + pos, remaining(pos), " ");
	if (pos >= PREFIX_SIZE)
		buf[PREFIX_SIZE - 1] = '\0';

	return buf;
}

int __dynamic_pr_debug(struct _ddebug *descriptor, const char *fmt, ...)
{
	va_list args;
	int res;
	struct va_format vaf;
	char buf[PREFIX_SIZE];

	BUG_ON(!descriptor);
	BUG_ON(!fmt);

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	res = printk("%s%pV", dynamic_emit_prefix(descriptor, buf), &vaf);
	va_end(args);

	return res;
}
EXPORT_SYMBOL(__dynamic_pr_debug);

int __dynamic_dev_dbg(struct _ddebug *descriptor,
		      const struct device *dev, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;
	int res;
	char buf[PREFIX_SIZE];

	BUG_ON(!descriptor);
	BUG_ON(!fmt);

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	res = __dev_printk(dynamic_emit_prefix(descriptor, buf), dev, &vaf);
	va_end(args);

	return res;
}
EXPORT_SYMBOL(__dynamic_dev_dbg);

#ifdef CONFIG_NET

int __dynamic_netdev_dbg(struct _ddebug *descriptor,
		      const struct net_device *dev, const char *fmt, ...)
{
	struct va_format vaf;
	va_list args;
	int res;
	char buf[PREFIX_SIZE];

	BUG_ON(!descriptor);
	BUG_ON(!fmt);

	va_start(args, fmt);
	vaf.fmt = fmt;
	vaf.va = &args;
	res = __netdev_printk(dynamic_emit_prefix(descriptor, buf), dev, &vaf);
	va_end(args);

	return res;
}
EXPORT_SYMBOL(__dynamic_netdev_dbg);

#endif

#define DDEBUG_STRING_SIZE 1024
static __initdata char ddebug_setup_string[DDEBUG_STRING_SIZE];

static __init int ddebug_setup_query(char *str)
{
	if (strlen(str) >= DDEBUG_STRING_SIZE) {
		pr_warn("ddebug boot param string too large\n");
		return 0;
	}
	strlcpy(ddebug_setup_string, str, DDEBUG_STRING_SIZE);
	return 1;
}

__setup("ddebug_query=", ddebug_setup_query);

#define USER_BUF_PAGE 4096
static ssize_t ddebug_proc_write(struct file *file, const char __user *ubuf,
				  size_t len, loff_t *offp)
{
	char *tmpbuf;
	int ret;

	if (len == 0)
		return 0;
	if (len > USER_BUF_PAGE - 1) {
		pr_warn("expected <%d bytes into control\n", USER_BUF_PAGE);
		return -E2BIG;
	}
	tmpbuf = kmalloc(len + 1, GFP_KERNEL);
	if (!tmpbuf)
		return -ENOMEM;
	if (copy_from_user(tmpbuf, ubuf, len)) {
		kfree(tmpbuf);
		return -EFAULT;
	}
	tmpbuf[len] = '\0';
	if (verbose)
		pr_info("read %d bytes from userspace\n", (int)len);

	ret = ddebug_exec_queries(tmpbuf);
	kfree(tmpbuf);
	if (ret < 0)
		return ret;

	*offp += len;
	return len;
}

static struct _ddebug *ddebug_iter_first(struct ddebug_iter *iter)
{
	if (list_empty(&ddebug_tables)) {
		iter->table = NULL;
		iter->idx = 0;
		return NULL;
	}
	iter->table = list_entry(ddebug_tables.next,
				 struct ddebug_table, link);
	iter->idx = 0;
	return &iter->table->ddebugs[iter->idx];
}

static struct _ddebug *ddebug_iter_next(struct ddebug_iter *iter)
{
	if (iter->table == NULL)
		return NULL;
	if (++iter->idx == iter->table->num_ddebugs) {
		
		iter->idx = 0;
		if (list_is_last(&iter->table->link, &ddebug_tables)) {
			iter->table = NULL;
			return NULL;
		}
		iter->table = list_entry(iter->table->link.next,
					 struct ddebug_table, link);
	}
	return &iter->table->ddebugs[iter->idx];
}

static void *ddebug_proc_start(struct seq_file *m, loff_t *pos)
{
	struct ddebug_iter *iter = m->private;
	struct _ddebug *dp;
	int n = *pos;

	if (verbose)
		pr_info("called m=%p *pos=%lld\n", m, (unsigned long long)*pos);

	mutex_lock(&ddebug_lock);

	if (!n)
		return SEQ_START_TOKEN;
	if (n < 0)
		return NULL;
	dp = ddebug_iter_first(iter);
	while (dp != NULL && --n > 0)
		dp = ddebug_iter_next(iter);
	return dp;
}

static void *ddebug_proc_next(struct seq_file *m, void *p, loff_t *pos)
{
	struct ddebug_iter *iter = m->private;
	struct _ddebug *dp;

	if (verbose)
		pr_info("called m=%p p=%p *pos=%lld\n",
			m, p, (unsigned long long)*pos);

	if (p == SEQ_START_TOKEN)
		dp = ddebug_iter_first(iter);
	else
		dp = ddebug_iter_next(iter);
	++*pos;
	return dp;
}

static int ddebug_proc_show(struct seq_file *m, void *p)
{
	struct ddebug_iter *iter = m->private;
	struct _ddebug *dp = p;
	char flagsbuf[10];

	if (verbose)
		pr_info("called m=%p p=%p\n", m, p);

	if (p == SEQ_START_TOKEN) {
		seq_puts(m,
			"# filename:lineno [module]function flags format\n");
		return 0;
	}

	seq_printf(m, "%s:%u [%s]%s =%s \"",
		trim_prefix(dp->filename), dp->lineno,
		iter->table->mod_name, dp->function,
		ddebug_describe_flags(dp, flagsbuf, sizeof(flagsbuf)));
	seq_escape(m, dp->format, "\t\r\n\"");
	seq_puts(m, "\"\n");

	return 0;
}

static void ddebug_proc_stop(struct seq_file *m, void *p)
{
	if (verbose)
		pr_info("called m=%p p=%p\n", m, p);
	mutex_unlock(&ddebug_lock);
}

static const struct seq_operations ddebug_proc_seqops = {
	.start = ddebug_proc_start,
	.next = ddebug_proc_next,
	.show = ddebug_proc_show,
	.stop = ddebug_proc_stop
};

static int ddebug_proc_open(struct inode *inode, struct file *file)
{
	struct ddebug_iter *iter;
	int err;

	if (verbose)
		pr_info("called\n");

	iter = kzalloc(sizeof(*iter), GFP_KERNEL);
	if (iter == NULL)
		return -ENOMEM;

	err = seq_open(file, &ddebug_proc_seqops);
	if (err) {
		kfree(iter);
		return err;
	}
	((struct seq_file *) file->private_data)->private = iter;
	return 0;
}

static const struct file_operations ddebug_proc_fops = {
	.owner = THIS_MODULE,
	.open = ddebug_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release_private,
	.write = ddebug_proc_write
};

int ddebug_add_module(struct _ddebug *tab, unsigned int n,
			     const char *name)
{
	struct ddebug_table *dt;
	char *new_name;

	dt = kzalloc(sizeof(*dt), GFP_KERNEL);
	if (dt == NULL)
		return -ENOMEM;
	new_name = kstrdup(name, GFP_KERNEL);
	if (new_name == NULL) {
		kfree(dt);
		return -ENOMEM;
	}
	dt->mod_name = new_name;
	dt->num_ddebugs = n;
	dt->ddebugs = tab;

	mutex_lock(&ddebug_lock);
	list_add_tail(&dt->link, &ddebug_tables);
	mutex_unlock(&ddebug_lock);

	if (verbose)
		pr_info("%u debug prints in module %s\n", n, dt->mod_name);
	return 0;
}
EXPORT_SYMBOL_GPL(ddebug_add_module);

static void ddebug_table_free(struct ddebug_table *dt)
{
	list_del_init(&dt->link);
	kfree(dt->mod_name);
	kfree(dt);
}

int ddebug_remove_module(const char *mod_name)
{
	struct ddebug_table *dt, *nextdt;
	int ret = -ENOENT;

	if (verbose)
		pr_info("removing module \"%s\"\n", mod_name);

	mutex_lock(&ddebug_lock);
	list_for_each_entry_safe(dt, nextdt, &ddebug_tables, link) {
		if (!strcmp(dt->mod_name, mod_name)) {
			ddebug_table_free(dt);
			ret = 0;
		}
	}
	mutex_unlock(&ddebug_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(ddebug_remove_module);

static void ddebug_remove_all_tables(void)
{
	mutex_lock(&ddebug_lock);
	while (!list_empty(&ddebug_tables)) {
		struct ddebug_table *dt = list_entry(ddebug_tables.next,
						      struct ddebug_table,
						      link);
		ddebug_table_free(dt);
	}
	mutex_unlock(&ddebug_lock);
}

static __initdata int ddebug_init_success;

static int __init dynamic_debug_init_debugfs(void)
{
	struct dentry *dir, *file;

	if (!ddebug_init_success)
		return -ENODEV;

	dir = debugfs_create_dir("dynamic_debug", NULL);
	if (!dir)
		return -ENOMEM;
	file = debugfs_create_file("control", 0644, dir, NULL,
					&ddebug_proc_fops);
	if (!file) {
		debugfs_remove(dir);
		return -ENOMEM;
	}
	return 0;
}

static int __init dynamic_debug_init(void)
{
	struct _ddebug *iter, *iter_start;
	const char *modname = NULL;
	int ret = 0;
	int n = 0;

	if (__start___verbose == __stop___verbose) {
		pr_warn("_ddebug table is empty in a "
			"CONFIG_DYNAMIC_DEBUG build");
		return 1;
	}
	iter = __start___verbose;
	modname = iter->modname;
	iter_start = iter;
	for (; iter < __stop___verbose; iter++) {
		if (strcmp(modname, iter->modname)) {
			ret = ddebug_add_module(iter_start, n, modname);
			if (ret)
				goto out_free;
			n = 0;
			modname = iter->modname;
			iter_start = iter;
		}
		n++;
	}
	ret = ddebug_add_module(iter_start, n, modname);
	if (ret)
		goto out_free;

	
	if (ddebug_setup_string[0] != '\0') {
		ret = ddebug_exec_queries(ddebug_setup_string);
		if (ret < 0)
			pr_warn("Invalid ddebug boot param %s",
				ddebug_setup_string);
		else
			pr_info("%d changes by ddebug_query\n", ret);

		
		ret = 0;
	}

out_free:
	if (ret)
		ddebug_remove_all_tables();
	else
		ddebug_init_success = 1;
	return 0;
}
arch_initcall(dynamic_debug_init);
module_init(dynamic_debug_init_debugfs);
