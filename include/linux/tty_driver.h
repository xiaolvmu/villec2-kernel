#ifndef _LINUX_TTY_DRIVER_H
#define _LINUX_TTY_DRIVER_H


#include <linux/export.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/termios.h>

struct tty_struct;
struct tty_driver;
struct serial_icounter_struct;

struct tty_operations {
	struct tty_struct * (*lookup)(struct tty_driver *driver,
			struct inode *inode, int idx);
	int  (*install)(struct tty_driver *driver, struct tty_struct *tty);
	void (*remove)(struct tty_driver *driver, struct tty_struct *tty);
	int  (*open)(struct tty_struct * tty, struct file * filp);
	void (*close)(struct tty_struct * tty, struct file * filp);
	void (*shutdown)(struct tty_struct *tty);
	void (*cleanup)(struct tty_struct *tty);
	int  (*write)(struct tty_struct * tty,
		      const unsigned char *buf, int count);
	int  (*put_char)(struct tty_struct *tty, unsigned char ch);
	void (*flush_chars)(struct tty_struct *tty);
	int  (*write_room)(struct tty_struct *tty);
	int  (*chars_in_buffer)(struct tty_struct *tty);
	int  (*ioctl)(struct tty_struct *tty,
		    unsigned int cmd, unsigned long arg);
	long (*compat_ioctl)(struct tty_struct *tty,
			     unsigned int cmd, unsigned long arg);
	void (*set_termios)(struct tty_struct *tty, struct ktermios * old);
	void (*throttle)(struct tty_struct * tty);
	void (*unthrottle)(struct tty_struct * tty);
	void (*stop)(struct tty_struct *tty);
	void (*start)(struct tty_struct *tty);
	void (*hangup)(struct tty_struct *tty);
	int (*break_ctl)(struct tty_struct *tty, int state);
	void (*flush_buffer)(struct tty_struct *tty);
	void (*set_ldisc)(struct tty_struct *tty);
	void (*wait_until_sent)(struct tty_struct *tty, int timeout);
	void (*send_xchar)(struct tty_struct *tty, char ch);
	int (*tiocmget)(struct tty_struct *tty);
	int (*tiocmset)(struct tty_struct *tty,
			unsigned int set, unsigned int clear);
	int (*resize)(struct tty_struct *tty, struct winsize *ws);
	int (*set_termiox)(struct tty_struct *tty, struct termiox *tnew);
	int (*get_icount)(struct tty_struct *tty,
				struct serial_icounter_struct *icount);
#ifdef CONFIG_CONSOLE_POLL
	int (*poll_init)(struct tty_driver *driver, int line, char *options);
	int (*poll_get_char)(struct tty_driver *driver, int line);
	void (*poll_put_char)(struct tty_driver *driver, int line, char ch);
#endif
	const struct file_operations *proc_fops;
};

struct tty_driver {
	int	magic;		
	struct kref kref;	
	struct cdev cdev;
	struct module	*owner;
	const char	*driver_name;
	const char	*name;
	int	name_base;	
	int	major;		
	int	minor_start;	
	int	num;		
	short	type;		
	short	subtype;	
	struct ktermios init_termios; 
	int	flags;		
	struct proc_dir_entry *proc_entry; 
	struct tty_driver *other; 

	struct tty_struct **ttys;
	struct ktermios **termios;
	void *driver_state;


	const struct tty_operations *ops;
	struct list_head tty_drivers;
};

extern struct list_head tty_drivers;

extern struct tty_driver *__alloc_tty_driver(int lines, struct module *owner);
extern void put_tty_driver(struct tty_driver *driver);
extern void tty_set_operations(struct tty_driver *driver,
			const struct tty_operations *op);
extern struct tty_driver *tty_find_polling_driver(char *name, int *line);

extern void tty_driver_kref_put(struct tty_driver *driver);

#define alloc_tty_driver(lines) __alloc_tty_driver(lines, THIS_MODULE)

static inline struct tty_driver *tty_driver_kref_get(struct tty_driver *d)
{
	kref_get(&d->kref);
	return d;
}

#define TTY_DRIVER_MAGIC		0x5402

#define TTY_DRIVER_INSTALLED		0x0001
#define TTY_DRIVER_RESET_TERMIOS	0x0002
#define TTY_DRIVER_REAL_RAW		0x0004
#define TTY_DRIVER_DYNAMIC_DEV		0x0008
#define TTY_DRIVER_DEVPTS_MEM		0x0010
#define TTY_DRIVER_HARDWARE_BREAK	0x0020

#define TTY_DRIVER_TYPE_SYSTEM		0x0001
#define TTY_DRIVER_TYPE_CONSOLE		0x0002
#define TTY_DRIVER_TYPE_SERIAL		0x0003
#define TTY_DRIVER_TYPE_PTY		0x0004
#define TTY_DRIVER_TYPE_SCC		0x0005	
#define TTY_DRIVER_TYPE_SYSCONS		0x0006

#define SYSTEM_TYPE_TTY			0x0001
#define SYSTEM_TYPE_CONSOLE		0x0002
#define SYSTEM_TYPE_SYSCONS		0x0003
#define SYSTEM_TYPE_SYSPTMX		0x0004

#define PTY_TYPE_MASTER			0x0001
#define PTY_TYPE_SLAVE			0x0002

#define SERIAL_TYPE_NORMAL	1

#endif 
