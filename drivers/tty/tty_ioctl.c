/*
 *  Copyright (C) 1991, 1992, 1993, 1994  Linus Torvalds
 *
 * Modified by Fred N. van Kempen, 01/29/93, to add line disciplines
 * which can be dynamically activated and de-activated by the line
 * discipline handling modules (like SLIP).
 */

#include <linux/types.h>
#include <linux/termios.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/tty.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/compat.h>

#include <asm/io.h>
#include <asm/uaccess.h>

#undef TTY_DEBUG_WAIT_UNTIL_SENT

#undef	DEBUG

#define TERMIOS_FLUSH	1
#define TERMIOS_WAIT	2
#define TERMIOS_TERMIO	4
#define TERMIOS_OLD	8



int tty_chars_in_buffer(struct tty_struct *tty)
{
	if (tty->ops->chars_in_buffer)
		return tty->ops->chars_in_buffer(tty);
	else
		return 0;
}
EXPORT_SYMBOL(tty_chars_in_buffer);

 
int tty_write_room(struct tty_struct *tty)
{
	if (tty->ops->write_room)
		return tty->ops->write_room(tty);
	return 2048;
}
EXPORT_SYMBOL(tty_write_room);

void tty_driver_flush_buffer(struct tty_struct *tty)
{
	if (tty->ops->flush_buffer)
		tty->ops->flush_buffer(tty);
}
EXPORT_SYMBOL(tty_driver_flush_buffer);


void tty_throttle(struct tty_struct *tty)
{
	mutex_lock(&tty->termios_mutex);
	
	if (!test_and_set_bit(TTY_THROTTLED, &tty->flags) &&
	    tty->ops->throttle)
		tty->ops->throttle(tty);
	mutex_unlock(&tty->termios_mutex);
}
EXPORT_SYMBOL(tty_throttle);


void tty_unthrottle(struct tty_struct *tty)
{
	mutex_lock(&tty->termios_mutex);
	if (test_and_clear_bit(TTY_THROTTLED, &tty->flags) &&
	    tty->ops->unthrottle)
		tty->ops->unthrottle(tty);
	mutex_unlock(&tty->termios_mutex);
}
EXPORT_SYMBOL(tty_unthrottle);


void tty_wait_until_sent(struct tty_struct *tty, long timeout)
{
#ifdef TTY_DEBUG_WAIT_UNTIL_SENT
	char buf[64];

	printk(KERN_DEBUG "%s wait until sent...\n", tty_name(tty, buf));
#endif
	if (!timeout)
		timeout = MAX_SCHEDULE_TIMEOUT;
	if (wait_event_interruptible_timeout(tty->write_wait,
			!tty_chars_in_buffer(tty), timeout) >= 0) {
		if (tty->ops->wait_until_sent)
			tty->ops->wait_until_sent(tty, timeout);
	}
}
EXPORT_SYMBOL(tty_wait_until_sent);



static void unset_locked_termios(struct ktermios *termios,
				 struct ktermios *old,
				 struct ktermios *locked)
{
	int	i;

#define NOSET_MASK(x, y, z) (x = ((x) & ~(z)) | ((y) & (z)))

	if (!locked) {
		printk(KERN_WARNING "Warning?!? termios_locked is NULL.\n");
		return;
	}

	NOSET_MASK(termios->c_iflag, old->c_iflag, locked->c_iflag);
	NOSET_MASK(termios->c_oflag, old->c_oflag, locked->c_oflag);
	NOSET_MASK(termios->c_cflag, old->c_cflag, locked->c_cflag);
	NOSET_MASK(termios->c_lflag, old->c_lflag, locked->c_lflag);
	termios->c_line = locked->c_line ? old->c_line : termios->c_line;
	for (i = 0; i < NCCS; i++)
		termios->c_cc[i] = locked->c_cc[i] ?
			old->c_cc[i] : termios->c_cc[i];
	
}

static const speed_t baud_table[] = {
	0, 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800,
	9600, 19200, 38400, 57600, 115200, 230400, 460800,
#ifdef __sparc__
	76800, 153600, 307200, 614400, 921600
#else
	500000, 576000, 921600, 1000000, 1152000, 1500000, 2000000,
	2500000, 3000000, 3500000, 4000000
#endif
};

#ifndef __sparc__
static const tcflag_t baud_bits[] = {
	B0, B50, B75, B110, B134, B150, B200, B300, B600,
	B1200, B1800, B2400, B4800, B9600, B19200, B38400,
	B57600, B115200, B230400, B460800, B500000, B576000,
	B921600, B1000000, B1152000, B1500000, B2000000, B2500000,
	B3000000, B3500000, B4000000
};
#else
static const tcflag_t baud_bits[] = {
	B0, B50, B75, B110, B134, B150, B200, B300, B600,
	B1200, B1800, B2400, B4800, B9600, B19200, B38400,
	B57600, B115200, B230400, B460800, B76800, B153600,
	B307200, B614400, B921600
};
#endif

static int n_baud_table = ARRAY_SIZE(baud_table);


speed_t tty_termios_baud_rate(struct ktermios *termios)
{
	unsigned int cbaud;

	cbaud = termios->c_cflag & CBAUD;

#ifdef BOTHER
	
	if (cbaud == BOTHER)
		return termios->c_ospeed;
#endif
	if (cbaud & CBAUDEX) {
		cbaud &= ~CBAUDEX;

		if (cbaud < 1 || cbaud + 15 > n_baud_table)
			termios->c_cflag &= ~CBAUDEX;
		else
			cbaud += 15;
	}
	return baud_table[cbaud];
}
EXPORT_SYMBOL(tty_termios_baud_rate);


speed_t tty_termios_input_baud_rate(struct ktermios *termios)
{
#ifdef IBSHIFT
	unsigned int cbaud = (termios->c_cflag >> IBSHIFT) & CBAUD;

	if (cbaud == B0)
		return tty_termios_baud_rate(termios);

	
	if (cbaud == BOTHER)
		return termios->c_ispeed;

	if (cbaud & CBAUDEX) {
		cbaud &= ~CBAUDEX;

		if (cbaud < 1 || cbaud + 15 > n_baud_table)
			termios->c_cflag &= ~(CBAUDEX << IBSHIFT);
		else
			cbaud += 15;
	}
	return baud_table[cbaud];
#else
	return tty_termios_baud_rate(termios);
#endif
}
EXPORT_SYMBOL(tty_termios_input_baud_rate);


void tty_termios_encode_baud_rate(struct ktermios *termios,
				  speed_t ibaud, speed_t obaud)
{
	int i = 0;
	int ifound = -1, ofound = -1;
	int iclose = ibaud/50, oclose = obaud/50;
	int ibinput = 0;

	if (obaud == 0)			
		ibaud = 0;		

	termios->c_ispeed = ibaud;
	termios->c_ospeed = obaud;

#ifdef BOTHER

	if ((termios->c_cflag & CBAUD) == BOTHER)
		oclose = 0;
	if (((termios->c_cflag >> IBSHIFT) & CBAUD) == BOTHER)
		iclose = 0;
	if ((termios->c_cflag >> IBSHIFT) & CBAUD)
		ibinput = 1;	
#endif
	termios->c_cflag &= ~CBAUD;


	do {
		if (obaud - oclose <= baud_table[i] &&
		    obaud + oclose >= baud_table[i]) {
			termios->c_cflag |= baud_bits[i];
			ofound = i;
		}
		if (ibaud - iclose <= baud_table[i] &&
		    ibaud + iclose >= baud_table[i]) {
			if (ofound == i && !ibinput)
				ifound  = i;
#ifdef IBSHIFT
			else {
				ifound = i;
				termios->c_cflag |= (baud_bits[i] << IBSHIFT);
			}
#endif
		}
	} while (++i < n_baud_table);

#ifdef BOTHER
	if (ofound == -1)
		termios->c_cflag |= BOTHER;
	if (ifound == -1 && (ibaud != obaud || ibinput))
		termios->c_cflag |= (BOTHER << IBSHIFT);
#else
	if (ifound == -1 || ofound == -1) {
		printk_once(KERN_WARNING "tty: Unable to return correct "
			  "speed data as your architecture needs updating.\n");
	}
#endif
}
EXPORT_SYMBOL_GPL(tty_termios_encode_baud_rate);


void tty_encode_baud_rate(struct tty_struct *tty, speed_t ibaud, speed_t obaud)
{
	tty_termios_encode_baud_rate(tty->termios, ibaud, obaud);
}
EXPORT_SYMBOL_GPL(tty_encode_baud_rate);


speed_t tty_get_baud_rate(struct tty_struct *tty)
{
	speed_t baud = tty_termios_baud_rate(tty->termios);

	if (baud == 38400 && tty->alt_speed) {
		if (!tty->warned) {
			printk(KERN_WARNING "Use of setserial/setrocket to "
					    "set SPD_* flags is deprecated\n");
			tty->warned = 1;
		}
		baud = tty->alt_speed;
	}

	return baud;
}
EXPORT_SYMBOL(tty_get_baud_rate);


void tty_termios_copy_hw(struct ktermios *new, struct ktermios *old)
{
	new->c_cflag &= HUPCL | CREAD | CLOCAL;
	new->c_cflag |= old->c_cflag & ~(HUPCL | CREAD | CLOCAL);
	new->c_ispeed = old->c_ispeed;
	new->c_ospeed = old->c_ospeed;
}
EXPORT_SYMBOL(tty_termios_copy_hw);


int tty_termios_hw_change(struct ktermios *a, struct ktermios *b)
{
	if (a->c_ispeed != b->c_ispeed || a->c_ospeed != b->c_ospeed)
		return 1;
	if ((a->c_cflag ^ b->c_cflag) & ~(HUPCL | CREAD | CLOCAL))
		return 1;
	return 0;
}
EXPORT_SYMBOL(tty_termios_hw_change);


int tty_set_termios(struct tty_struct *tty, struct ktermios *new_termios)
{
	struct ktermios old_termios;
	struct tty_ldisc *ld;
	unsigned long flags;



	mutex_lock(&tty->termios_mutex);
	old_termios = *tty->termios;
	*tty->termios = *new_termios;
	unset_locked_termios(tty->termios, &old_termios, tty->termios_locked);

	
	if (tty->link && tty->link->packet) {
		int extproc = (old_termios.c_lflag & EXTPROC) |
				(tty->termios->c_lflag & EXTPROC);
		int old_flow = ((old_termios.c_iflag & IXON) &&
				(old_termios.c_cc[VSTOP] == '\023') &&
				(old_termios.c_cc[VSTART] == '\021'));
		int new_flow = (I_IXON(tty) &&
				STOP_CHAR(tty) == '\023' &&
				START_CHAR(tty) == '\021');
		if ((old_flow != new_flow) || extproc) {
			spin_lock_irqsave(&tty->ctrl_lock, flags);
			if (old_flow != new_flow) {
				tty->ctrl_status &= ~(TIOCPKT_DOSTOP | TIOCPKT_NOSTOP);
				if (new_flow)
					tty->ctrl_status |= TIOCPKT_DOSTOP;
				else
					tty->ctrl_status |= TIOCPKT_NOSTOP;
			}
			if (extproc)
				tty->ctrl_status |= TIOCPKT_IOCTL;
			spin_unlock_irqrestore(&tty->ctrl_lock, flags);
			wake_up_interruptible(&tty->link->read_wait);
		}
	}

	if (tty->ops->set_termios)
		(*tty->ops->set_termios)(tty, &old_termios);
	else
		tty_termios_copy_hw(tty->termios, &old_termios);

	ld = tty_ldisc_ref(tty);
	if (ld != NULL) {
		if (ld->ops->set_termios)
			(ld->ops->set_termios)(tty, &old_termios);
		tty_ldisc_deref(ld);
	}
	mutex_unlock(&tty->termios_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(tty_set_termios);


static int set_termios(struct tty_struct *tty, void __user *arg, int opt)
{
	struct ktermios tmp_termios;
	struct tty_ldisc *ld;
	int retval = tty_check_change(tty);

	if (retval)
		return retval;

	mutex_lock(&tty->termios_mutex);
	memcpy(&tmp_termios, tty->termios, sizeof(struct ktermios));
	mutex_unlock(&tty->termios_mutex);

	if (opt & TERMIOS_TERMIO) {
		if (user_termio_to_kernel_termios(&tmp_termios,
						(struct termio __user *)arg))
			return -EFAULT;
#ifdef TCGETS2
	} else if (opt & TERMIOS_OLD) {
		if (user_termios_to_kernel_termios_1(&tmp_termios,
						(struct termios __user *)arg))
			return -EFAULT;
	} else {
		if (user_termios_to_kernel_termios(&tmp_termios,
						(struct termios2 __user *)arg))
			return -EFAULT;
	}
#else
	} else if (user_termios_to_kernel_termios(&tmp_termios,
					(struct termios __user *)arg))
		return -EFAULT;
#endif

	tmp_termios.c_ispeed = tty_termios_input_baud_rate(&tmp_termios);
	tmp_termios.c_ospeed = tty_termios_baud_rate(&tmp_termios);

	ld = tty_ldisc_ref(tty);

	if (ld != NULL) {
		if ((opt & TERMIOS_FLUSH) && ld->ops->flush_buffer)
			ld->ops->flush_buffer(tty);
		tty_ldisc_deref(ld);
	}

	if (opt & TERMIOS_WAIT) {
		tty_wait_until_sent(tty, 0);
		if (signal_pending(current))
			return -EINTR;
	}

	tty_set_termios(tty, &tmp_termios);

	return 0;
}

static void copy_termios(struct tty_struct *tty, struct ktermios *kterm)
{
	mutex_lock(&tty->termios_mutex);
	memcpy(kterm, tty->termios, sizeof(struct ktermios));
	mutex_unlock(&tty->termios_mutex);
}

static void copy_termios_locked(struct tty_struct *tty, struct ktermios *kterm)
{
	mutex_lock(&tty->termios_mutex);
	memcpy(kterm, tty->termios_locked, sizeof(struct ktermios));
	mutex_unlock(&tty->termios_mutex);
}

static int get_termio(struct tty_struct *tty, struct termio __user *termio)
{
	struct ktermios kterm;
	copy_termios(tty, &kterm);
	if (kernel_termios_to_user_termio(termio, &kterm))
		return -EFAULT;
	return 0;
}


#ifdef TCGETX


static int set_termiox(struct tty_struct *tty, void __user *arg, int opt)
{
	struct termiox tnew;
	struct tty_ldisc *ld;

	if (tty->termiox == NULL)
		return -EINVAL;
	if (copy_from_user(&tnew, arg, sizeof(struct termiox)))
		return -EFAULT;

	ld = tty_ldisc_ref(tty);
	if (ld != NULL) {
		if ((opt & TERMIOS_FLUSH) && ld->ops->flush_buffer)
			ld->ops->flush_buffer(tty);
		tty_ldisc_deref(ld);
	}
	if (opt & TERMIOS_WAIT) {
		tty_wait_until_sent(tty, 0);
		if (signal_pending(current))
			return -EINTR;
	}

	mutex_lock(&tty->termios_mutex);
	if (tty->ops->set_termiox)
		tty->ops->set_termiox(tty, &tnew);
	mutex_unlock(&tty->termios_mutex);
	return 0;
}

#endif


#ifdef TIOCGETP
static int get_sgflags(struct tty_struct *tty)
{
	int flags = 0;

	if (!(tty->termios->c_lflag & ICANON)) {
		if (tty->termios->c_lflag & ISIG)
			flags |= 0x02;		
		else
			flags |= 0x20;		
	}
	if (tty->termios->c_lflag & ECHO)
		flags |= 0x08;			
	if (tty->termios->c_oflag & OPOST)
		if (tty->termios->c_oflag & ONLCR)
			flags |= 0x10;		
	return flags;
}

static int get_sgttyb(struct tty_struct *tty, struct sgttyb __user *sgttyb)
{
	struct sgttyb tmp;

	mutex_lock(&tty->termios_mutex);
	tmp.sg_ispeed = tty->termios->c_ispeed;
	tmp.sg_ospeed = tty->termios->c_ospeed;
	tmp.sg_erase = tty->termios->c_cc[VERASE];
	tmp.sg_kill = tty->termios->c_cc[VKILL];
	tmp.sg_flags = get_sgflags(tty);
	mutex_unlock(&tty->termios_mutex);

	return copy_to_user(sgttyb, &tmp, sizeof(tmp)) ? -EFAULT : 0;
}

static void set_sgflags(struct ktermios *termios, int flags)
{
	termios->c_iflag = ICRNL | IXON;
	termios->c_oflag = 0;
	termios->c_lflag = ISIG | ICANON;
	if (flags & 0x02) {	
		termios->c_iflag = 0;
		termios->c_lflag &= ~ICANON;
	}
	if (flags & 0x08) {		
		termios->c_lflag |= ECHO | ECHOE | ECHOK |
				    ECHOCTL | ECHOKE | IEXTEN;
	}
	if (flags & 0x10) {		
		termios->c_oflag |= OPOST | ONLCR;
	}
	if (flags & 0x20) {	
		termios->c_iflag = 0;
		termios->c_lflag &= ~(ISIG | ICANON);
	}
	if (!(termios->c_lflag & ICANON)) {
		termios->c_cc[VMIN] = 1;
		termios->c_cc[VTIME] = 0;
	}
}


static int set_sgttyb(struct tty_struct *tty, struct sgttyb __user *sgttyb)
{
	int retval;
	struct sgttyb tmp;
	struct ktermios termios;

	retval = tty_check_change(tty);
	if (retval)
		return retval;

	if (copy_from_user(&tmp, sgttyb, sizeof(tmp)))
		return -EFAULT;

	mutex_lock(&tty->termios_mutex);
	termios = *tty->termios;
	termios.c_cc[VERASE] = tmp.sg_erase;
	termios.c_cc[VKILL] = tmp.sg_kill;
	set_sgflags(&termios, tmp.sg_flags);
	
#ifdef BOTHER
	tty_termios_encode_baud_rate(&termios, termios.c_ispeed,
						termios.c_ospeed);
#endif
	mutex_unlock(&tty->termios_mutex);
	tty_set_termios(tty, &termios);
	return 0;
}
#endif

#ifdef TIOCGETC
static int get_tchars(struct tty_struct *tty, struct tchars __user *tchars)
{
	struct tchars tmp;

	mutex_lock(&tty->termios_mutex);
	tmp.t_intrc = tty->termios->c_cc[VINTR];
	tmp.t_quitc = tty->termios->c_cc[VQUIT];
	tmp.t_startc = tty->termios->c_cc[VSTART];
	tmp.t_stopc = tty->termios->c_cc[VSTOP];
	tmp.t_eofc = tty->termios->c_cc[VEOF];
	tmp.t_brkc = tty->termios->c_cc[VEOL2];	
	mutex_unlock(&tty->termios_mutex);
	return copy_to_user(tchars, &tmp, sizeof(tmp)) ? -EFAULT : 0;
}

static int set_tchars(struct tty_struct *tty, struct tchars __user *tchars)
{
	struct tchars tmp;

	if (copy_from_user(&tmp, tchars, sizeof(tmp)))
		return -EFAULT;
	mutex_lock(&tty->termios_mutex);
	tty->termios->c_cc[VINTR] = tmp.t_intrc;
	tty->termios->c_cc[VQUIT] = tmp.t_quitc;
	tty->termios->c_cc[VSTART] = tmp.t_startc;
	tty->termios->c_cc[VSTOP] = tmp.t_stopc;
	tty->termios->c_cc[VEOF] = tmp.t_eofc;
	tty->termios->c_cc[VEOL2] = tmp.t_brkc;	
	mutex_unlock(&tty->termios_mutex);
	return 0;
}
#endif

#ifdef TIOCGLTC
static int get_ltchars(struct tty_struct *tty, struct ltchars __user *ltchars)
{
	struct ltchars tmp;

	mutex_lock(&tty->termios_mutex);
	tmp.t_suspc = tty->termios->c_cc[VSUSP];
	
	tmp.t_dsuspc = tty->termios->c_cc[VSUSP];
	tmp.t_rprntc = tty->termios->c_cc[VREPRINT];
	
	tmp.t_flushc = tty->termios->c_cc[VEOL2];
	tmp.t_werasc = tty->termios->c_cc[VWERASE];
	tmp.t_lnextc = tty->termios->c_cc[VLNEXT];
	mutex_unlock(&tty->termios_mutex);
	return copy_to_user(ltchars, &tmp, sizeof(tmp)) ? -EFAULT : 0;
}

static int set_ltchars(struct tty_struct *tty, struct ltchars __user *ltchars)
{
	struct ltchars tmp;

	if (copy_from_user(&tmp, ltchars, sizeof(tmp)))
		return -EFAULT;

	mutex_lock(&tty->termios_mutex);
	tty->termios->c_cc[VSUSP] = tmp.t_suspc;
	
	tty->termios->c_cc[VEOL2] = tmp.t_dsuspc;
	tty->termios->c_cc[VREPRINT] = tmp.t_rprntc;
	
	tty->termios->c_cc[VEOL2] = tmp.t_flushc;
	tty->termios->c_cc[VWERASE] = tmp.t_werasc;
	tty->termios->c_cc[VLNEXT] = tmp.t_lnextc;
	mutex_unlock(&tty->termios_mutex);
	return 0;
}
#endif


static int send_prio_char(struct tty_struct *tty, char ch)
{
	int	was_stopped = tty->stopped;

	if (tty->ops->send_xchar) {
		tty->ops->send_xchar(tty, ch);
		return 0;
	}

	if (tty_write_lock(tty, 0) < 0)
		return -ERESTARTSYS;

	if (was_stopped)
		start_tty(tty);
	tty->ops->write(tty, &ch, 1);
	if (was_stopped)
		stop_tty(tty);
	tty_write_unlock(tty);
	return 0;
}


static int tty_change_softcar(struct tty_struct *tty, int arg)
{
	int ret = 0;
	int bit = arg ? CLOCAL : 0;
	struct ktermios old;

	mutex_lock(&tty->termios_mutex);
	old = *tty->termios;
	tty->termios->c_cflag &= ~CLOCAL;
	tty->termios->c_cflag |= bit;
	if (tty->ops->set_termios)
		tty->ops->set_termios(tty, &old);
	if ((tty->termios->c_cflag & CLOCAL) != bit)
		ret = -EINVAL;
	mutex_unlock(&tty->termios_mutex);
	return ret;
}


int tty_mode_ioctl(struct tty_struct *tty, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct tty_struct *real_tty;
	void __user *p = (void __user *)arg;
	int ret = 0;
	struct ktermios kterm;

	BUG_ON(file == NULL);

	if (tty->driver->type == TTY_DRIVER_TYPE_PTY &&
	    tty->driver->subtype == PTY_TYPE_MASTER)
		real_tty = tty->link;
	else
		real_tty = tty;

	switch (cmd) {
#ifdef TIOCGETP
	case TIOCGETP:
		return get_sgttyb(real_tty, (struct sgttyb __user *) arg);
	case TIOCSETP:
	case TIOCSETN:
		return set_sgttyb(real_tty, (struct sgttyb __user *) arg);
#endif
#ifdef TIOCGETC
	case TIOCGETC:
		return get_tchars(real_tty, p);
	case TIOCSETC:
		return set_tchars(real_tty, p);
#endif
#ifdef TIOCGLTC
	case TIOCGLTC:
		return get_ltchars(real_tty, p);
	case TIOCSLTC:
		return set_ltchars(real_tty, p);
#endif
	case TCSETSF:
		return set_termios(real_tty, p,  TERMIOS_FLUSH | TERMIOS_WAIT | TERMIOS_OLD);
	case TCSETSW:
		return set_termios(real_tty, p, TERMIOS_WAIT | TERMIOS_OLD);
	case TCSETS:
		return set_termios(real_tty, p, TERMIOS_OLD);
#ifndef TCGETS2
	case TCGETS:
		copy_termios(real_tty, &kterm);
		if (kernel_termios_to_user_termios((struct termios __user *)arg, &kterm))
			ret = -EFAULT;
		return ret;
#else
	case TCGETS:
		copy_termios(real_tty, &kterm);
		if (kernel_termios_to_user_termios_1((struct termios __user *)arg, &kterm))
			ret = -EFAULT;
		return ret;
	case TCGETS2:
		copy_termios(real_tty, &kterm);
		if (kernel_termios_to_user_termios((struct termios2 __user *)arg, &kterm))
			ret = -EFAULT;
		return ret;
	case TCSETSF2:
		return set_termios(real_tty, p,  TERMIOS_FLUSH | TERMIOS_WAIT);
	case TCSETSW2:
		return set_termios(real_tty, p, TERMIOS_WAIT);
	case TCSETS2:
		return set_termios(real_tty, p, 0);
#endif
	case TCGETA:
		return get_termio(real_tty, p);
	case TCSETAF:
		return set_termios(real_tty, p, TERMIOS_FLUSH | TERMIOS_WAIT | TERMIOS_TERMIO);
	case TCSETAW:
		return set_termios(real_tty, p, TERMIOS_WAIT | TERMIOS_TERMIO);
	case TCSETA:
		return set_termios(real_tty, p, TERMIOS_TERMIO);
#ifndef TCGETS2
	case TIOCGLCKTRMIOS:
		copy_termios_locked(real_tty, &kterm);
		if (kernel_termios_to_user_termios((struct termios __user *)arg, &kterm))
			ret = -EFAULT;
		return ret;
	case TIOCSLCKTRMIOS:
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		copy_termios_locked(real_tty, &kterm);
		if (user_termios_to_kernel_termios(&kterm,
					       (struct termios __user *) arg))
			return -EFAULT;
		mutex_lock(&real_tty->termios_mutex);
		memcpy(real_tty->termios_locked, &kterm, sizeof(struct ktermios));
		mutex_unlock(&real_tty->termios_mutex);
		return 0;
#else
	case TIOCGLCKTRMIOS:
		copy_termios_locked(real_tty, &kterm);
		if (kernel_termios_to_user_termios_1((struct termios __user *)arg, &kterm))
			ret = -EFAULT;
		return ret;
	case TIOCSLCKTRMIOS:
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;
		copy_termios_locked(real_tty, &kterm);
		if (user_termios_to_kernel_termios_1(&kterm,
					       (struct termios __user *) arg))
			return -EFAULT;
		mutex_lock(&real_tty->termios_mutex);
		memcpy(real_tty->termios_locked, &kterm, sizeof(struct ktermios));
		mutex_unlock(&real_tty->termios_mutex);
		return ret;
#endif
#ifdef TCGETX
	case TCGETX: {
		struct termiox ktermx;
		if (real_tty->termiox == NULL)
			return -EINVAL;
		mutex_lock(&real_tty->termios_mutex);
		memcpy(&ktermx, real_tty->termiox, sizeof(struct termiox));
		mutex_unlock(&real_tty->termios_mutex);
		if (copy_to_user(p, &ktermx, sizeof(struct termiox)))
			ret = -EFAULT;
		return ret;
	}
	case TCSETX:
		return set_termiox(real_tty, p, 0);
	case TCSETXW:
		return set_termiox(real_tty, p, TERMIOS_WAIT);
	case TCSETXF:
		return set_termiox(real_tty, p, TERMIOS_FLUSH);
#endif		
	case TIOCGSOFTCAR:
		copy_termios(real_tty, &kterm);
		ret = put_user((kterm.c_cflag & CLOCAL) ? 1 : 0,
						(int __user *)arg);
		return ret;
	case TIOCSSOFTCAR:
		if (get_user(arg, (unsigned int __user *) arg))
			return -EFAULT;
		return tty_change_softcar(real_tty, arg);
	default:
		return -ENOIOCTLCMD;
	}
}
EXPORT_SYMBOL_GPL(tty_mode_ioctl);

int tty_perform_flush(struct tty_struct *tty, unsigned long arg)
{
	struct tty_ldisc *ld;
	int retval = tty_check_change(tty);
	if (retval)
		return retval;

	ld = tty_ldisc_ref_wait(tty);
	switch (arg) {
	case TCIFLUSH:
		if (ld && ld->ops->flush_buffer)
			ld->ops->flush_buffer(tty);
		break;
	case TCIOFLUSH:
		if (ld && ld->ops->flush_buffer)
			ld->ops->flush_buffer(tty);
		
	case TCOFLUSH:
		tty_driver_flush_buffer(tty);
		break;
	default:
		tty_ldisc_deref(ld);
		return -EINVAL;
	}
	tty_ldisc_deref(ld);
	return 0;
}
EXPORT_SYMBOL_GPL(tty_perform_flush);

int n_tty_ioctl_helper(struct tty_struct *tty, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	unsigned long flags;
	int retval;

	switch (cmd) {
	case TCXONC:
		retval = tty_check_change(tty);
		if (retval)
			return retval;
		switch (arg) {
		case TCOOFF:
			if (!tty->flow_stopped) {
				tty->flow_stopped = 1;
				stop_tty(tty);
			}
			break;
		case TCOON:
			if (tty->flow_stopped) {
				tty->flow_stopped = 0;
				start_tty(tty);
			}
			break;
		case TCIOFF:
			if (STOP_CHAR(tty) != __DISABLED_CHAR)
				return send_prio_char(tty, STOP_CHAR(tty));
			break;
		case TCION:
			if (START_CHAR(tty) != __DISABLED_CHAR)
				return send_prio_char(tty, START_CHAR(tty));
			break;
		default:
			return -EINVAL;
		}
		return 0;
	case TCFLSH:
		return tty_perform_flush(tty, arg);
	case TIOCPKT:
	{
		int pktmode;

		if (tty->driver->type != TTY_DRIVER_TYPE_PTY ||
		    tty->driver->subtype != PTY_TYPE_MASTER)
			return -ENOTTY;
		if (get_user(pktmode, (int __user *) arg))
			return -EFAULT;
		spin_lock_irqsave(&tty->ctrl_lock, flags);
		if (pktmode) {
			if (!tty->packet) {
				tty->packet = 1;
				tty->link->ctrl_status = 0;
			}
		} else
			tty->packet = 0;
		spin_unlock_irqrestore(&tty->ctrl_lock, flags);
		return 0;
	}
	default:
		
		return tty_mode_ioctl(tty, file, cmd, arg);
	}
}
EXPORT_SYMBOL(n_tty_ioctl_helper);

#ifdef CONFIG_COMPAT
long n_tty_compat_ioctl_helper(struct tty_struct *tty, struct file *file,
					unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
	case TIOCGLCKTRMIOS:
	case TIOCSLCKTRMIOS:
		return tty_mode_ioctl(tty, file, cmd, (unsigned long) compat_ptr(arg));
	default:
		return -ENOIOCTLCMD;
	}
}
EXPORT_SYMBOL(n_tty_compat_ioctl_helper);
#endif

