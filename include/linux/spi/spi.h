/*
 * Copyright (C) 2005 David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __LINUX_SPI_H
#define __LINUX_SPI_H

#include <linux/device.h>
#include <linux/mod_devicetable.h>
#include <linux/slab.h>
#include <linux/kthread.h>

extern struct bus_type spi_bus_type;

struct spi_device {
	struct device		dev;
	struct spi_master	*master;
	u32			max_speed_hz;
	u8			chip_select;
	u8			mode;
#define	SPI_CPHA	0x01			
#define	SPI_CPOL	0x02			
#define	SPI_MODE_0	(0|0)			
#define	SPI_MODE_1	(0|SPI_CPHA)
#define	SPI_MODE_2	(SPI_CPOL|0)
#define	SPI_MODE_3	(SPI_CPOL|SPI_CPHA)
#define	SPI_CS_HIGH	0x04			
#define	SPI_LSB_FIRST	0x08			
#define	SPI_3WIRE	0x10			
#define	SPI_LOOP	0x20			
#define	SPI_NO_CS	0x40			
#define	SPI_READY	0x80			
	u8			bits_per_word;
	int			irq;
	void			*controller_state;
	void			*controller_data;
	char			modalias[SPI_NAME_SIZE];

};

static inline struct spi_device *to_spi_device(struct device *dev)
{
	return dev ? container_of(dev, struct spi_device, dev) : NULL;
}

static inline struct spi_device *spi_dev_get(struct spi_device *spi)
{
	return (spi && get_device(&spi->dev)) ? spi : NULL;
}

static inline void spi_dev_put(struct spi_device *spi)
{
	if (spi)
		put_device(&spi->dev);
}

static inline void *spi_get_ctldata(struct spi_device *spi)
{
	return spi->controller_state;
}

static inline void spi_set_ctldata(struct spi_device *spi, void *state)
{
	spi->controller_state = state;
}


static inline void spi_set_drvdata(struct spi_device *spi, void *data)
{
	dev_set_drvdata(&spi->dev, data);
}

static inline void *spi_get_drvdata(struct spi_device *spi)
{
	return dev_get_drvdata(&spi->dev);
}

struct spi_message;



struct spi_driver {
	const struct spi_device_id *id_table;
	int			(*probe)(struct spi_device *spi);
	int			(*remove)(struct spi_device *spi);
	void			(*shutdown)(struct spi_device *spi);
	int			(*suspend)(struct spi_device *spi, pm_message_t mesg);
	int			(*resume)(struct spi_device *spi);
	struct device_driver	driver;
};

static inline struct spi_driver *to_spi_driver(struct device_driver *drv)
{
	return drv ? container_of(drv, struct spi_driver, driver) : NULL;
}

extern int spi_register_driver(struct spi_driver *sdrv);

static inline void spi_unregister_driver(struct spi_driver *sdrv)
{
	if (sdrv)
		driver_unregister(&sdrv->driver);
}

#define module_spi_driver(__spi_driver) \
	module_driver(__spi_driver, spi_register_driver, \
			spi_unregister_driver)

struct spi_master {
	struct device	dev;

	struct list_head list;

	s16			bus_num;

	u16			num_chipselect;

	u16			dma_alignment;

	
	u16			mode_bits;

	
	u16			flags;
#define SPI_MASTER_HALF_DUPLEX	BIT(0)		
#define SPI_MASTER_NO_RX	BIT(1)		
#define SPI_MASTER_NO_TX	BIT(2)		

	
	spinlock_t		bus_lock_spinlock;
	struct mutex		bus_lock_mutex;

	
	bool			bus_lock_flag;

	int			(*setup)(struct spi_device *spi);

	int			(*transfer)(struct spi_device *spi,
						struct spi_message *mesg);

	
	void			(*cleanup)(struct spi_device *spi);

	bool				queued;
	struct kthread_worker		kworker;
	struct task_struct		*kworker_task;
	struct kthread_work		pump_messages;
	spinlock_t			queue_lock;
	struct list_head		queue;
	struct spi_message		*cur_msg;
	bool				busy;
	bool				running;
	bool				rt;

	int (*prepare_transfer_hardware)(struct spi_master *master);
	int (*transfer_one_message)(struct spi_master *master,
				    struct spi_message *mesg);
	int (*unprepare_transfer_hardware)(struct spi_master *master);
};

static inline void *spi_master_get_devdata(struct spi_master *master)
{
	return dev_get_drvdata(&master->dev);
}

static inline void spi_master_set_devdata(struct spi_master *master, void *data)
{
	dev_set_drvdata(&master->dev, data);
}

static inline struct spi_master *spi_master_get(struct spi_master *master)
{
	if (!master || !get_device(&master->dev))
		return NULL;
	return master;
}

static inline void spi_master_put(struct spi_master *master)
{
	if (master)
		put_device(&master->dev);
}

extern int spi_master_suspend(struct spi_master *master);
extern int spi_master_resume(struct spi_master *master);

extern struct spi_message *spi_get_next_queued_message(struct spi_master *master);
extern void spi_finalize_current_message(struct spi_master *master);

extern struct spi_master *
spi_alloc_master(struct device *host, unsigned size);

extern int spi_register_master(struct spi_master *master);
extern void spi_unregister_master(struct spi_master *master);

extern struct spi_master *spi_busnum_to_master(u16 busnum);



struct spi_transfer {
	const void	*tx_buf;
	void		*rx_buf;
	unsigned	len;

	dma_addr_t	tx_dma;
	dma_addr_t	rx_dma;

	unsigned	cs_change:1;
	u8		bits_per_word;
	u16		delay_usecs;
	u32		speed_hz;

	struct list_head transfer_list;
};

struct spi_message {
	struct list_head	transfers;

	struct spi_device	*spi;

	unsigned		is_dma_mapped:1;


	
	void			(*complete)(void *context);
	void			*context;
	unsigned		actual_length;
	int			status;

	struct list_head	queue;
	void			*state;
};

static inline void spi_message_init(struct spi_message *m)
{
	memset(m, 0, sizeof *m);
	INIT_LIST_HEAD(&m->transfers);
}

static inline void
spi_message_add_tail(struct spi_transfer *t, struct spi_message *m)
{
	list_add_tail(&t->transfer_list, &m->transfers);
}

static inline void
spi_transfer_del(struct spi_transfer *t)
{
	list_del(&t->transfer_list);
}


static inline struct spi_message *spi_message_alloc(unsigned ntrans, gfp_t flags)
{
	struct spi_message *m;

	m = kzalloc(sizeof(struct spi_message)
			+ ntrans * sizeof(struct spi_transfer),
			flags);
	if (m) {
		unsigned i;
		struct spi_transfer *t = (struct spi_transfer *)(m + 1);

		INIT_LIST_HEAD(&m->transfers);
		for (i = 0; i < ntrans; i++, t++)
			spi_message_add_tail(t, m);
	}
	return m;
}

static inline void spi_message_free(struct spi_message *m)
{
	kfree(m);
}

extern int spi_setup(struct spi_device *spi);
extern int spi_async(struct spi_device *spi, struct spi_message *message);
extern int spi_async_locked(struct spi_device *spi,
			    struct spi_message *message);



extern int spi_sync(struct spi_device *spi, struct spi_message *message);
extern int spi_sync_locked(struct spi_device *spi, struct spi_message *message);
extern int spi_bus_lock(struct spi_master *master);
extern int spi_bus_unlock(struct spi_master *master);

static inline int
spi_write(struct spi_device *spi, const void *buf, size_t len)
{
	struct spi_transfer	t = {
			.tx_buf		= buf,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(spi, &m);
}

static inline int
spi_read(struct spi_device *spi, void *buf, size_t len)
{
	struct spi_transfer	t = {
			.rx_buf		= buf,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(spi, &m);
}

extern int spi_write_then_read(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx);

extern int spi_write_and_read(struct spi_device *spi,
		u8 *txbuf, u8 *rxbuf, unsigned size);

static inline ssize_t spi_w8r8(struct spi_device *spi, u8 cmd)
{
	ssize_t			status;
	u8			result;

	status = spi_write_then_read(spi, &cmd, 1, &result, 1);

	
	return (status < 0) ? status : result;
}

static inline ssize_t spi_w8r16(struct spi_device *spi, u8 cmd)
{
	ssize_t			status;
	u16			result;

	status = spi_write_then_read(spi, &cmd, 1, (u8 *) &result, 2);

	
	return (status < 0) ? status : result;
}



struct spi_board_info {
	char		modalias[SPI_NAME_SIZE];
	const void	*platform_data;
	void		*controller_data;
	int		irq;

	
	u32		max_speed_hz;


	u16		bus_num;
	u16		chip_select;

	u8		mode;

};

#ifdef	CONFIG_SPI
extern int
spi_register_board_info(struct spi_board_info const *info, unsigned n);
#else
static inline int
spi_register_board_info(struct spi_board_info const *info, unsigned n)
	{ return 0; }
#endif


extern struct spi_device *
spi_alloc_device(struct spi_master *master);

extern int
spi_add_device(struct spi_device *spi);

extern struct spi_device *
spi_new_device(struct spi_master *, struct spi_board_info *);

static inline void
spi_unregister_device(struct spi_device *spi)
{
	if (spi)
		device_unregister(&spi->dev);
}

extern const struct spi_device_id *
spi_get_device_id(const struct spi_device *sdev);

#endif 
