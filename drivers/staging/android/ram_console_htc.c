/* drivers/android/ram_console.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/console.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/platform_data/ram_console.h>
#include <asm/setup.h>

#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
#include <linux/rslib.h>
#endif

struct ram_console_buffer {
	uint32_t    sig;
	uint32_t    start;
	uint32_t    size;
	uint8_t     data[0];
};

#define RAM_CONSOLE_SIG (0x43474244) 

#ifdef CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT
static char __initdata
	ram_console_old_log_init_buffer[CONFIG_ANDROID_RAM_CONSOLE_EARLY_SIZE];
#endif
static char *ram_console_old_log;
static size_t ram_console_old_log_size;

static struct ram_console_buffer *ram_console_buffer;
static size_t ram_console_buffer_size;
#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
static char *ram_console_par_buffer;
static struct rs_control *ram_console_rs_decoder;
static int ram_console_corrected_bytes;
static int ram_console_bad_blocks;
#define ECC_BLOCK_SIZE CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION_DATA_SIZE
#define ECC_SIZE CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION_ECC_SIZE
#define ECC_SYMSIZE CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION_SYMBOL_SIZE
#define ECC_POLY CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION_POLYNOMIAL
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
static void ram_console_encode_rs8(uint8_t *data, size_t len, uint8_t *ecc)
{
	int i;
	uint16_t par[ECC_SIZE];
	
	memset(par, 0, sizeof(par));
	encode_rs8(ram_console_rs_decoder, data, len, par, 0);
	for (i = 0; i < ECC_SIZE; i++)
		ecc[i] = par[i];
}

static int ram_console_decode_rs8(void *data, size_t len, uint8_t *ecc)
{
	int i;
	uint16_t par[ECC_SIZE];
	for (i = 0; i < ECC_SIZE; i++)
		par[i] = ecc[i];
	return decode_rs8(ram_console_rs_decoder, data, par, len,
				NULL, 0, NULL, 0, NULL);
}
#endif

static void ram_console_update(const char *s, unsigned int count)
{
	struct ram_console_buffer *buffer = ram_console_buffer;
#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
	uint8_t *buffer_end = buffer->data + ram_console_buffer_size;
	uint8_t *block;
	uint8_t *par;
	int size = ECC_BLOCK_SIZE;
#endif
	memcpy(buffer->data + buffer->start, s, count);
#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
	block = buffer->data + (buffer->start & ~(ECC_BLOCK_SIZE - 1));
	par = ram_console_par_buffer +
	      (buffer->start / ECC_BLOCK_SIZE) * ECC_SIZE;
	do {
		if (block + ECC_BLOCK_SIZE > buffer_end)
			size = buffer_end - block;
		ram_console_encode_rs8(block, size, par);
		block += ECC_BLOCK_SIZE;
		par += ECC_SIZE;
	} while (block < buffer->data + buffer->start + count);
#endif
}

static void ram_console_update_header(void)
{
#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
	struct ram_console_buffer *buffer = ram_console_buffer;
	uint8_t *par;
	par = ram_console_par_buffer +
	      DIV_ROUND_UP(ram_console_buffer_size, ECC_BLOCK_SIZE) * ECC_SIZE;
	ram_console_encode_rs8((uint8_t *)buffer, sizeof(*buffer), par);
#endif
}

static void
ram_console_write(struct console *console, const char *s, unsigned int count)
{
	int rem;
	struct ram_console_buffer *buffer = ram_console_buffer;

	if (count > ram_console_buffer_size) {
		s += count - ram_console_buffer_size;
		count = ram_console_buffer_size;
	}
	rem = ram_console_buffer_size - buffer->start;
	if (rem < count) {
		ram_console_update(s, rem);
		s += rem;
		count -= rem;
		buffer->start = 0;
		buffer->size = ram_console_buffer_size;
	}
	ram_console_update(s, count);

	buffer->start += count;
	if (buffer->size < ram_console_buffer_size)
		buffer->size += count;
	ram_console_update_header();
}

static struct console ram_console = {
	.name	= "ram",
	.write	= ram_console_write,
	.flags	= CON_PRINTBUFFER | CON_ENABLED | CON_ANYTIME,
	.index	= -1,
};

void ram_console_enable_console(int enabled)
{
	if (enabled)
		ram_console.flags |= CON_ENABLED;
	else
		ram_console.flags &= ~CON_ENABLED;
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE_APPEND_PMIC_STATUS_BITS
static unsigned int atoi(const char *name)
{
	unsigned int val = 0;

	for (;; name++) {
		switch (*name) {
		case '0' ... '9':
			val = 10*val+(*name-'0');
			break;
		default:
			return val;
		}
	}
}
static unsigned int last_off_event = 0;
static int __init pmic_last_off_event(char *opt)
{
	if (!opt || !*opt || *opt == '\0')
		return 1;
	pr_debug("[K] ram_console: last_off_event=%s", opt);
	last_off_event = atoi(opt);
	return 1;
}
__setup("reset_status=", pmic_last_off_event);

static unsigned int start_on_event = 0;
static int __init pmic_start_on_event(char *opt)
{
	if (!opt || !*opt || *opt == '\0')
		return 1;
	pr_debug("[K] ram_console: start_on_event=%s", opt);
	start_on_event = atoi(opt);
	return 1;
}
__setup("poweron_status=", pmic_start_on_event);
#endif


#ifdef CONFIG_DEBUG_BLDR_LOG
static char *bldr_log;
static unsigned long bldr_log_start = 0;
static unsigned long bldr_log_size = 0;
static int __init parse_tag_bldr_log(const struct tag *tag)
{
	bldr_log_start = tag->u.bldr_log.addr;
	bldr_log_size =  tag->u.bldr_log.size;
	return 0;
}

__tagtable(ATAG_BLDR_LOG, parse_tag_bldr_log);
#endif

static void __init
ram_console_save_old(struct ram_console_buffer *buffer, const char *bootinfo,
	char *dest)
{
	size_t old_log_size = buffer->size;
	size_t bootinfo_size = 0;
	size_t total_size = old_log_size;
	char *ptr;
	const char *bootinfo_label = "Boot info:\n";
#ifdef CONFIG_ANDROID_RAM_CONSOLE_APPEND_PMIC_STATUS_BITS
#define PMIC_STATUS_MAX 100
	char pmic_status_buffer[PMIC_STATUS_MAX];
	size_t pmic_status_buffer_len;
#endif

#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
	uint8_t *block;
	uint8_t *par;
	char strbuf[80];
	int strbuf_len = 0;

	block = buffer->data;
	par = ram_console_par_buffer;
	while (block < buffer->data + buffer->size) {
		int numerr;
		int size = ECC_BLOCK_SIZE;
		if (block + size > buffer->data + ram_console_buffer_size)
			size = buffer->data + ram_console_buffer_size - block;
		numerr = ram_console_decode_rs8(block, size, par);
		if (numerr > 0) {
#if 0
			printk(KERN_INFO "[K] ram_console: error in block %p, %d\n",
			       block, numerr);
#endif
			ram_console_corrected_bytes += numerr;
		} else if (numerr < 0) {
#if 0
			printk(KERN_INFO "[K] ram_console: uncorrectable error in "
			       "block %p\n", block);
#endif
			ram_console_bad_blocks++;
		}
		block += ECC_BLOCK_SIZE;
		par += ECC_SIZE;
	}
	if (ram_console_corrected_bytes || ram_console_bad_blocks)
		strbuf_len = snprintf(strbuf, sizeof(strbuf),
			"\n%d Corrected bytes, %d unrecoverable blocks\n",
			ram_console_corrected_bytes, ram_console_bad_blocks);
	else
		strbuf_len = snprintf(strbuf, sizeof(strbuf),
				      "\nNo errors detected\n");
	if (strbuf_len >= sizeof(strbuf))
		strbuf_len = sizeof(strbuf) - 1;
	total_size += strbuf_len;
#endif

	if (bootinfo)
		bootinfo_size = strlen(bootinfo) + strlen(bootinfo_label);
	total_size += bootinfo_size;

#ifdef CONFIG_ANDROID_RAM_CONSOLE_APPEND_PMIC_STATUS_BITS
	pmic_status_buffer_len =
		snprintf(pmic_status_buffer, sizeof(pmic_status_buffer),
			"\n[QCT] PMIC status: start_on_event=0x%x, last_off_event=0x%x\n",
			start_on_event, last_off_event);
	pmic_status_buffer_len =
		min(pmic_status_buffer_len, sizeof(pmic_status_buffer) - 1);
	total_size += pmic_status_buffer_len;
#endif

	if (dest == NULL) {
		dest = kmalloc(total_size, GFP_KERNEL);
		if (dest == NULL) {
			printk(KERN_ERR
			       "[K] ram_console: failed to allocate buffer\n");
			return;
		}
	}

	ram_console_old_log = dest;
	ram_console_old_log_size = total_size;
	memcpy(ram_console_old_log,
	       &buffer->data[buffer->start], buffer->size - buffer->start);
	memcpy(ram_console_old_log + buffer->size - buffer->start,
	       &buffer->data[0], buffer->start);
	ptr = ram_console_old_log + old_log_size;
#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
	memcpy(ptr, strbuf, strbuf_len);
	ptr += strbuf_len;
#endif
	if (bootinfo) {
		memcpy(ptr, bootinfo_label, strlen(bootinfo_label));
		ptr += strlen(bootinfo_label);
		memcpy(ptr, bootinfo, bootinfo_size);
		ptr += bootinfo_size;
	}
#ifdef CONFIG_ANDROID_RAM_CONSOLE_APPEND_PMIC_STATUS_BITS
	memcpy(ptr,
		   pmic_status_buffer, pmic_status_buffer_len);
	ptr += pmic_status_buffer_len;
#endif
}

static int __init ram_console_init(struct ram_console_buffer *buffer,
				   size_t buffer_size, const char *bootinfo,
				   char *old_buf)
{
#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
	int numerr;
	uint8_t *par;
#endif
	ram_console_buffer = buffer;
	ram_console_buffer_size =
		buffer_size - sizeof(struct ram_console_buffer);

	if (ram_console_buffer_size > buffer_size) {
		pr_err("[K] ram_console: buffer %p, invalid size %zu, "
		       "datasize %zu\n", buffer, buffer_size,
		       ram_console_buffer_size);
		return 0;
	}

#ifdef CONFIG_ANDROID_RAM_CONSOLE_ERROR_CORRECTION
	ram_console_buffer_size -= (DIV_ROUND_UP(ram_console_buffer_size,
						ECC_BLOCK_SIZE) + 1) * ECC_SIZE;

	if (ram_console_buffer_size > buffer_size) {
		pr_err("[K] ram_console: buffer %p, invalid size %zu, "
		       "non-ecc datasize %zu\n",
		       buffer, buffer_size, ram_console_buffer_size);
		return 0;
	}

	ram_console_par_buffer = buffer->data + ram_console_buffer_size;


	ram_console_rs_decoder = init_rs(ECC_SYMSIZE, ECC_POLY, 0, 1, ECC_SIZE);
	if (ram_console_rs_decoder == NULL) {
		printk(KERN_INFO "[K] ram_console: init_rs failed\n");
		return 0;
	}

	ram_console_corrected_bytes = 0;
	ram_console_bad_blocks = 0;

	par = ram_console_par_buffer +
	      DIV_ROUND_UP(ram_console_buffer_size, ECC_BLOCK_SIZE) * ECC_SIZE;

	numerr = ram_console_decode_rs8(buffer, sizeof(*buffer), par);
	if (numerr > 0) {
		printk(KERN_INFO "[K] ram_console: error in header, %d\n", numerr);
		ram_console_corrected_bytes += numerr;
	} else if (numerr < 0) {
		printk(KERN_INFO
		       "[K] ram_console: uncorrectable error in header\n");
		ram_console_bad_blocks++;
	}
#endif

	if (buffer->sig == RAM_CONSOLE_SIG) {
		if (buffer->size > ram_console_buffer_size
		    || buffer->start > buffer->size)
			printk(KERN_INFO "[K] ram_console: found existing invalid "
			       "buffer, size %d, start %d\n",
			       buffer->size, buffer->start);
		else {
			printk(KERN_INFO "[K] ram_console: found existing buffer, "
			       "size %d, start %d\n",
			       buffer->size, buffer->start);
			ram_console_save_old(buffer, bootinfo, old_buf);
		}
	} else {
		printk(KERN_INFO "[K] ram_console: no valid data in buffer "
		       "(sig = 0x%08x)\n", buffer->sig);
	}

	buffer->sig = RAM_CONSOLE_SIG;
	buffer->start = 0;
	buffer->size = 0;

	register_console(&ram_console);
#ifdef CONFIG_ANDROID_RAM_CONSOLE_ENABLE_VERBOSE
	console_verbose();
#endif
	return 0;
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT
static int __init ram_console_early_init(void)
{
	return ram_console_init((struct ram_console_buffer *)
		CONFIG_ANDROID_RAM_CONSOLE_EARLY_ADDR,
		CONFIG_ANDROID_RAM_CONSOLE_EARLY_SIZE,
		NULL,
		ram_console_old_log_init_buffer);
}
#else
static int ram_console_driver_probe(struct platform_device *pdev)
{
	struct resource *res = pdev->resource;
	size_t start;
	size_t buffer_size;
	void *buffer;
	const char *bootinfo = NULL;
	struct ram_console_platform_data *pdata = pdev->dev.platform_data;

#ifdef CONFIG_DEBUG_BLDR_LOG
	if (bldr_log_start && bldr_log_size) {
		bldr_log = ioremap(bldr_log_start, bldr_log_size);
		if (bldr_log == NULL) {
			printk(KERN_ERR "[K] hboot log: failed to map memory\n");
			return -ENOMEM;
		}

		printk(KERN_INFO "[K] hboot log buffer: got buffer at %lx, size %lx\n", bldr_log_start, bldr_log_size);
	}
#endif

	if (res == NULL || pdev->num_resources != 1 ||
	    !(res->flags & IORESOURCE_MEM)) {
		printk(KERN_ERR "[K] ram_console: invalid resource, %p %d flags "
		       "%lx\n", res, pdev->num_resources, res ? res->flags : 0);
		return -ENXIO;
	}
	buffer_size = res->end - res->start + 1;
	start = res->start;
	printk(KERN_INFO "[K] ram_console: got buffer at %zx, size %zx\n",
	       start, buffer_size);
	buffer = ioremap(res->start, buffer_size);
	if (buffer == NULL) {
		printk(KERN_ERR "[K] ram_console: failed to map memory\n");
		return -ENOMEM;
	}

	if (pdata)
		bootinfo = pdata->bootinfo;

	return ram_console_init(buffer, buffer_size, bootinfo, NULL);
}

static struct platform_driver ram_console_driver = {
	.probe = ram_console_driver_probe,
	.driver		= {
		.name	= "ram_console",
	},
};

static int __init ram_console_module_init(void)
{
	int err;
	err = platform_driver_register(&ram_console_driver);
	return err;
}
#endif

static ssize_t ram_console_read_old(struct file *file, char __user *buf,
				    size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;

#ifdef CONFIG_DEBUG_BLDR_LOG
	loff_t bootloader_pos;
	if (pos >= ram_console_old_log_size + bldr_log_size)
		return 0;

	if (pos >= ram_console_old_log_size) {

		bootloader_pos = pos - ram_console_old_log_size;
		count = min(len, (size_t)(bldr_log_size - bootloader_pos));
		if (copy_to_user(buf, bldr_log + bootloader_pos, count))
			return -EFAULT;

		*offset += count;
		return count;
	}
#else
	if (pos >= ram_console_old_log_size)
		return 0;
#endif
	count = min(len, (size_t)(ram_console_old_log_size - pos));
	if (copy_to_user(buf, ram_console_old_log + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static const struct file_operations ram_console_file_ops = {
	.owner = THIS_MODULE,
	.read = ram_console_read_old,
};

static int __init ram_console_late_init(void)
{
	struct proc_dir_entry *entry;

	if (ram_console_old_log == NULL)
		return 0;
#ifdef CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT
	ram_console_old_log = kmalloc(ram_console_old_log_size, GFP_KERNEL);
	if (ram_console_old_log == NULL) {
		printk(KERN_ERR
		       "[K] ram_console: failed to allocate buffer for old log\n");
		ram_console_old_log_size = 0;
		return 0;
	}
	memcpy(ram_console_old_log,
	       ram_console_old_log_init_buffer, ram_console_old_log_size);
#endif
	entry = create_proc_entry("last_kmsg", S_IFREG | S_IRUGO, NULL);
	if (!entry) {
		printk(KERN_ERR "[K] ram_console: failed to create proc entry\n");
		kfree(ram_console_old_log);
		ram_console_old_log = NULL;
		return 0;
	}

	entry->proc_fops = &ram_console_file_ops;
	entry->size = ram_console_old_log_size;
	return 0;
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE_EARLY_INIT
console_initcall(ram_console_early_init);
#else
postcore_initcall(ram_console_module_init);
#endif
late_initcall(ram_console_late_init);

