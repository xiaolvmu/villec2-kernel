/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "msm_camera_i2c.h"

#if defined(CONFIG_MACH_MONARUDO) || defined(CONFIG_MACH_DELUXE_J) || defined(CONFIG_MACH_DELUXE_R) || defined(CONFIG_MACH_IMPRESSION_J)\
			|| defined(CONFIG_MACH_DELUXE_U) || defined(CONFIG_MACH_DELUXE_UL) || defined(CONFIG_MACH_DELUXE_UB1)\
			|| defined(CONFIG_MACH_T6_TL) || defined(CONFIG_MACH_T6_DUG) || defined(CONFIG_MACH_T6_DWG)\
			|| defined(CONFIG_MACH_T6_UL) || defined(CONFIG_MACH_T6_ULA) || defined(CONFIG_MACH_T6_WL)\
			|| defined(CONFIG_MACH_T6_WHL) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_T6_U)


#if defined(CONFIG_MACH_T6_TL) || defined(CONFIG_MACH_T6_DUG) || defined(CONFIG_MACH_T6_DWG)\
|| defined(CONFIG_MACH_T6_UL) || defined(CONFIG_MACH_T6_ULA) || defined(CONFIG_MACH_T6_WL)\
|| defined(CONFIG_MACH_T6_WHL) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_T6_U)

#define MAX_I2C_RETRIES 2
#else
#define MAX_I2C_RETRIES 20
#endif
static int i2c_transfer_retry(struct i2c_adapter *adap,
			struct i2c_msg *msgs,
			int len)
{
	int i2c_retry = 0;
	int ns; 

	while (i2c_retry++ < MAX_I2C_RETRIES) {
		ns = i2c_transfer(adap, msgs, len);
		if (ns == len)
			break;
		pr_err("[CAM]%s: try %d/%d: i2c_transfer sent: %d, len %d\n",
			__func__,
			i2c_retry, MAX_I2C_RETRIES, ns, len);
		msleep(10);
	}

	return ns == len ? 0 : -EIO;
}
#endif

int32_t msm_camera_i2c_rxdata(struct msm_camera_i2c_client *dev_client,
	unsigned char *rxdata, int data_length)
{
	int32_t rc = 0;
	uint16_t saddr = dev_client->client->addr >> 1;
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = dev_client->addr_type,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = data_length,
			.buf   = rxdata,
		},
	};
#if defined(CONFIG_MACH_MONARUDO) || defined(CONFIG_MACH_DELUXE_J) || defined(CONFIG_MACH_DELUXE_R) || defined(CONFIG_MACH_IMPRESSION_J)\
		|| defined(CONFIG_MACH_DELUXE_U) || defined(CONFIG_MACH_DELUXE_UL) || defined(CONFIG_MACH_DELUXE_UB1)\
        || defined(CONFIG_MACH_T6_TL) || defined(CONFIG_MACH_T6_DUG) || defined(CONFIG_MACH_T6_DWG)\
        || defined(CONFIG_MACH_T6_UL) || defined(CONFIG_MACH_T6_ULA) || defined(CONFIG_MACH_T6_WL)\
        || defined(CONFIG_MACH_T6_WHL) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_T6_U)
	rc = i2c_transfer_retry(dev_client->client->adapter, msgs, 2);
#else
	rc = i2c_transfer(dev_client->client->adapter, msgs, 2);
#endif
	if (rc < 0)
		S_I2C_DBG("msm_camera_i2c_rxdata failed 0x%x\n", saddr);
	return rc;
}

int32_t msm_camera_i2c_txdata(struct msm_camera_i2c_client *dev_client,
				unsigned char *txdata, int length)
{
	int32_t rc = 0;
	uint16_t saddr = dev_client->client->addr >> 1;
	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		 },
	};
#if defined(CONFIG_MACH_MONARUDO) || defined(CONFIG_MACH_DELUXE_J) || defined(CONFIG_MACH_DELUXE_R) || defined(CONFIG_MACH_IMPRESSION_J)\
			|| defined(CONFIG_MACH_DELUXE_U) || defined(CONFIG_MACH_DELUXE_UL) || defined(CONFIG_MACH_DELUXE_UB1)\
            || defined(CONFIG_MACH_T6_TL) || defined(CONFIG_MACH_T6_DUG) || defined(CONFIG_MACH_T6_DWG)\
            || defined(CONFIG_MACH_T6_UL) || defined(CONFIG_MACH_T6_ULA) || defined(CONFIG_MACH_T6_WL)\
            || defined(CONFIG_MACH_T6_WHL) || defined(CONFIG_MACH_DUMMY) || defined(CONFIG_MACH_T6_U)
	rc = i2c_transfer_retry(dev_client->client->adapter, msg, 1);
#else
	rc = i2c_transfer(dev_client->client->adapter, msg, 1);
#endif

	if (rc < 0)
		S_I2C_DBG("msm_camera_i2c_txdata faild 0x%x\n", saddr);
	return 0;
}

int32_t msm_camera_i2c_write_b(struct msm_camera_i2c_client *client,
	uint16_t addr, uint16_t data )
{
	int32_t rc = -EFAULT;
	uint8_t len = 0;
	enum msm_camera_i2c_data_type data_type = MSM_CAMERA_I2C_BYTE_DATA;
	unsigned char buf[client->addr_type+data_type];
	client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	S_I2C_DBG("%s reg addr = 0x%x data type: %d\n",
			  __func__, addr, data_type);
	if (client->addr_type == MSM_CAMERA_I2C_BYTE_ADDR) {
		buf[0] = addr;
		S_I2C_DBG("%s byte %d: 0x%x\n", __func__, len, buf[len]);
		len = 1;
	} else if (client->addr_type == MSM_CAMERA_I2C_WORD_ADDR) {
		buf[0] = addr >> BITS_PER_BYTE;
		buf[1] = addr;
		S_I2C_DBG("%s byte %d: 0x%x\n", __func__, len, buf[len]);
		S_I2C_DBG("%s byte %d: 0x%x\n", __func__, len+1, buf[len+1]);
		len = 2;
	}
	S_I2C_DBG("Data: 0x%x\n", data);
	if (data_type == MSM_CAMERA_I2C_BYTE_DATA) {
		buf[len] = data;
		S_I2C_DBG("Byte %d: 0x%x\n", len, buf[len]);
		len += 1;
	} else if (data_type == MSM_CAMERA_I2C_WORD_DATA) {
		buf[len] = data >> BITS_PER_BYTE;
		buf[len+1] = data;
		S_I2C_DBG("Byte %d: 0x%x\n", len, buf[len]);
		S_I2C_DBG("Byte %d: 0x%x\n", len+1, buf[len+1]);
		len += 2;
	}

	rc = msm_camera_i2c_txdata(client, buf, len);
	if (rc < 0)
		S_I2C_DBG("%s fail\n", __func__);
	return rc;
}

int32_t msm_camera_i2c_write(struct msm_camera_i2c_client *client,
	uint16_t addr, uint16_t data,
	enum msm_camera_i2c_data_type data_type)
{
	int32_t rc = -EFAULT;
	unsigned char buf[client->addr_type+data_type];
	uint8_t len = 0;
	if ((client->addr_type != MSM_CAMERA_I2C_BYTE_ADDR
		&& client->addr_type != MSM_CAMERA_I2C_WORD_ADDR)
		|| (data_type != MSM_CAMERA_I2C_BYTE_DATA
		&& data_type != MSM_CAMERA_I2C_WORD_DATA))
		return rc;

	S_I2C_DBG("%s reg addr = 0x%x data type: %d\n",
			  __func__, addr, data_type);
	if (client->addr_type == MSM_CAMERA_I2C_BYTE_ADDR) {
		buf[0] = addr;
		S_I2C_DBG("%s byte %d: 0x%x\n", __func__, len, buf[len]);
		len = 1;
	} else if (client->addr_type == MSM_CAMERA_I2C_WORD_ADDR) {
		buf[0] = addr >> BITS_PER_BYTE;
		buf[1] = addr;
		S_I2C_DBG("%s byte %d: 0x%x\n", __func__, len, buf[len]);
		S_I2C_DBG("%s byte %d: 0x%x\n", __func__, len+1, buf[len+1]);
		len = 2;
	}
	S_I2C_DBG("Data: 0x%x\n", data);
	if (data_type == MSM_CAMERA_I2C_BYTE_DATA) {
		buf[len] = data;
		S_I2C_DBG("Byte %d: 0x%x\n", len, buf[len]);
		len += 1;
	} else if (data_type == MSM_CAMERA_I2C_WORD_DATA) {
		buf[len] = data >> BITS_PER_BYTE;
		buf[len+1] = data;
		S_I2C_DBG("Byte %d: 0x%x\n", len, buf[len]);
		S_I2C_DBG("Byte %d: 0x%x\n", len+1, buf[len+1]);
		len += 2;
	}

 if (addr==0xffff) {
  msleep(data);
  return 0;
 }
  
	rc = msm_camera_i2c_txdata(client, buf, len);
	if (rc < 0)
		S_I2C_DBG("%s fail\n", __func__);
	return rc;
}

int32_t msm_camera_i2c_write_seq(struct msm_camera_i2c_client *client,
	uint16_t addr, uint8_t *data, uint16_t num_byte)
{
	int32_t rc = -EFAULT;
	unsigned char buf[client->addr_type+num_byte];
	uint8_t len = 0, i = 0;

	if ((client->addr_type != MSM_CAMERA_I2C_BYTE_ADDR
		&& client->addr_type != MSM_CAMERA_I2C_WORD_ADDR)
		|| num_byte == 0)
		return rc;

	S_I2C_DBG("%s reg addr = 0x%x num bytes: %d\n",
			  __func__, addr, num_byte);
	if (client->addr_type == MSM_CAMERA_I2C_BYTE_ADDR) {
		buf[0] = addr;
		S_I2C_DBG("%s byte %d: 0x%x\n", __func__, len, buf[len]);
		len = 1;
	} else if (client->addr_type == MSM_CAMERA_I2C_WORD_ADDR) {
		buf[0] = addr >> BITS_PER_BYTE;
		buf[1] = addr;
		S_I2C_DBG("%s byte %d: 0x%x\n", __func__, len, buf[len]);
		S_I2C_DBG("%s byte %d: 0x%x\n", __func__, len+1, buf[len+1]);
		len = 2;
	}
	for (i = 0; i < num_byte; i++) {
		buf[i+len] = data[i];
		S_I2C_DBG("Byte %d: 0x%x\n", i+len, buf[i+len]);
		S_I2C_DBG("Data: 0x%x\n", data[i]);
	}

	rc = msm_camera_i2c_txdata(client, buf, len+num_byte);
	if (rc < 0)
		S_I2C_DBG("%s fail\n", __func__);
	return rc;
}

int32_t msm_camera_i2c_set_mask(struct msm_camera_i2c_client *client,
	uint16_t addr, uint16_t mask,
	enum msm_camera_i2c_data_type data_type, uint16_t set_mask)
{
	int32_t rc;
	uint16_t reg_data;

	rc = msm_camera_i2c_read(client, addr, &reg_data, data_type);
	if (rc < 0) {
		S_I2C_DBG("%s read fail\n", __func__);
		return rc;
	}
	S_I2C_DBG("%s addr: 0x%x data: 0x%x setmask: 0x%x\n",
			__func__, addr, reg_data, mask);

	if (set_mask)
		reg_data |= mask;
	else
		reg_data &= ~mask;
	S_I2C_DBG("%s write: 0x%x\n", __func__, reg_data);

	rc = msm_camera_i2c_write(client, addr, reg_data, data_type);
	if (rc < 0)
		S_I2C_DBG("%s write fail\n", __func__);

	return rc;
}

int32_t msm_camera_i2c_set_write_mask_data(struct msm_camera_i2c_client *client,
	uint16_t addr, uint16_t data, int16_t mask,
	enum msm_camera_i2c_data_type data_type)
{
	int32_t rc;
	uint16_t reg_data;
	CDBG("%s\n", __func__);
	if (mask == -1)
		return 0;
	if (mask == 0)
		rc = msm_camera_i2c_write(client, addr, data, data_type);
	else{
		rc = msm_camera_i2c_read(client, addr, &reg_data, data_type);
		if (rc < 0) {
			CDBG("%s read fail\n", __func__);
			return rc;
		}
		reg_data  = reg_data & mask;
		reg_data  = (reg_data | (data & (~mask)));
		rc = msm_camera_i2c_write(client, addr, reg_data, data_type);
		if (rc < 0)
			CDBG("%s write fail\n", __func__);
	}
	return rc;
}


int32_t msm_camera_i2c_compare(struct msm_camera_i2c_client *client,
	uint16_t addr, uint16_t data,
	enum msm_camera_i2c_data_type data_type)
{
	int32_t rc = -EIO;
	uint16_t reg_data = 0;
	int data_len = 0;
	switch (data_type) {
	case MSM_CAMERA_I2C_BYTE_DATA:
	case MSM_CAMERA_I2C_WORD_DATA:
		data_len = data_type;
		break;
	case MSM_CAMERA_I2C_SET_BYTE_MASK:
	case MSM_CAMERA_I2C_UNSET_BYTE_MASK:
		data_len = MSM_CAMERA_I2C_BYTE_DATA;
		break;
	case MSM_CAMERA_I2C_SET_WORD_MASK:
	case MSM_CAMERA_I2C_UNSET_WORD_MASK:
		data_len = MSM_CAMERA_I2C_WORD_DATA;
		break;
	default:
		pr_err("%s: Unsupport data type: %d\n", __func__, data_type);
		break;
	}

	rc = msm_camera_i2c_read(client,
		addr, &reg_data, data_len);
	if (rc < 0)
		return rc;

	rc = 0;
	switch (data_type) {
	case MSM_CAMERA_I2C_BYTE_DATA:
	case MSM_CAMERA_I2C_WORD_DATA:
		if (data == reg_data)
			return rc;
		break;
	case MSM_CAMERA_I2C_SET_BYTE_MASK:
	case MSM_CAMERA_I2C_SET_WORD_MASK:
		if ((reg_data & data) == data)
			return rc;
		break;
	case MSM_CAMERA_I2C_UNSET_BYTE_MASK:
	case MSM_CAMERA_I2C_UNSET_WORD_MASK:
		if (!(reg_data & data))
			return rc;
		break;
	default:
		pr_err("%s: Unsupport data type: %d\n", __func__, data_type);
		break;
	}

	S_I2C_DBG("%s: Register and data does not match\n", __func__);
	rc = 1;
	return rc;
}

int32_t msm_camera_i2c_poll(struct msm_camera_i2c_client *client,
	uint16_t addr, uint16_t data,
	enum msm_camera_i2c_data_type data_type)
{
	int32_t rc = -EIO;
	int i;
	S_I2C_DBG("%s: addr: 0x%x data: 0x%x dt: %d\n",
		__func__, addr, data, data_type);

	for (i = 0; i < 20; i++) {
		rc = msm_camera_i2c_compare(client,
			addr, data, data_type);
		if (rc == 0 || rc < 0)
			break;
		usleep_range(10000, 11000);
	}
	return rc;
}

int32_t msm_camera_i2c_poll2(struct msm_camera_i2c_client *client,
	struct msm_camera_i2c_reg_conf *reg_conf_tbl)
{
	int32_t rc = -EIO;
	int i,done=0;
	uint16_t addr = reg_conf_tbl->reg_addr;
	uint16_t value = reg_conf_tbl->reg_data;
	uint16_t mask = reg_conf_tbl->mask;
	uint16_t readValue=0;

	
	for (i=0;i<20 && !done ;++i)
	{
		rc = msm_camera_i2c_read (client, addr, &readValue, 2); 
		if (rc < 0) {
			printk("i2c read error\n");
			
			return rc;
		}
		
		

		switch (reg_conf_tbl->cmd_type)
		{
			case MSM_CAMERA_I2C_CMD_POLL_EQUAL:
				done = !((readValue&mask) == value);
				break;

			case MSM_CAMERA_I2C_CMD_POLL_NOT_EQUAL:
				done = !((readValue&mask) != value);
				break;

			case MSM_CAMERA_I2C_CMD_POLL_LESS:
				done = !((readValue&mask) < value);
				break;
				
			default:
				break;
		}

		if (!done)
			msleep(5);
	}

	return rc;
}

int32_t msm_camera_i2c_write_tbl(struct msm_camera_i2c_client *client,
	struct msm_camera_i2c_reg_conf *reg_conf_tbl, uint16_t size,
	enum msm_camera_i2c_data_type data_type)
{
#define MIN(a,b) ((a)>(b)?(b):(a))
	int i,write_len,burst_size=128,remain_len=0,addr=0;
	uint8_t* buf=0;
	int32_t rc = -EFAULT;
	for (i = 0; i < size; i++) {
		enum msm_camera_i2c_data_type dt;
		if (reg_conf_tbl->cmd_type == MSM_CAMERA_I2C_CMD_POLL) {
			rc = msm_camera_i2c_poll(client, reg_conf_tbl->reg_addr,
				reg_conf_tbl->reg_data, reg_conf_tbl->dt);
		} 
		else if (reg_conf_tbl->cmd_type == MSM_CAMERA_I2C_CMD_POLL_EQUAL ||
				 reg_conf_tbl->cmd_type == MSM_CAMERA_I2C_CMD_POLL_NOT_EQUAL ||
				 reg_conf_tbl->cmd_type == MSM_CAMERA_I2C_CMD_POLL_LESS) {
		
			rc = msm_camera_i2c_poll2(client, reg_conf_tbl);
		
		}
        else if (reg_conf_tbl->cmd_type == MSM_CAMERA_I2C_CMD_WRITE_BURST) {
			buf = reg_conf_tbl->burst_data;
			remain_len = reg_conf_tbl->burst_count;
			write_len = MIN (reg_conf_tbl->burst_count,burst_size);
			addr = reg_conf_tbl->reg_addr;

			do {
				rc = msm_camera_i2c_write_seq(client,addr,buf,write_len);

				addr += write_len;
				buf += write_len;
				remain_len -= write_len;
				write_len = MIN(remain_len ,burst_size);
			}
			while (rc>=0 && write_len>0);
        }
		else {
			if (reg_conf_tbl->dt == 0)
				dt = data_type;
			else
				dt = reg_conf_tbl->dt;

			switch (dt) {
			case MSM_CAMERA_I2C_BYTE_DATA:
			case MSM_CAMERA_I2C_WORD_DATA:
				rc = msm_camera_i2c_write(
					client,
					reg_conf_tbl->reg_addr,
					reg_conf_tbl->reg_data, dt);
				break;
			case MSM_CAMERA_I2C_SET_BYTE_MASK:
				rc = msm_camera_i2c_set_mask(client,
					reg_conf_tbl->reg_addr,
					reg_conf_tbl->reg_data,
					MSM_CAMERA_I2C_BYTE_DATA, 1);
				break;
			case MSM_CAMERA_I2C_UNSET_BYTE_MASK:
				rc = msm_camera_i2c_set_mask(client,
					reg_conf_tbl->reg_addr,
					reg_conf_tbl->reg_data,
					MSM_CAMERA_I2C_BYTE_DATA, 0);
				break;
			case MSM_CAMERA_I2C_SET_WORD_MASK:
				rc = msm_camera_i2c_set_mask(client,
					reg_conf_tbl->reg_addr,
					reg_conf_tbl->reg_data,
					MSM_CAMERA_I2C_WORD_DATA, 1);
				break;
			case MSM_CAMERA_I2C_UNSET_WORD_MASK:
				rc = msm_camera_i2c_set_mask(client,
					reg_conf_tbl->reg_addr,
					reg_conf_tbl->reg_data,
					MSM_CAMERA_I2C_WORD_DATA, 0);
				break;
			case MSM_CAMERA_I2C_SET_BYTE_WRITE_MASK_DATA:
				rc = msm_camera_i2c_set_write_mask_data(client,
					reg_conf_tbl->reg_addr,
					reg_conf_tbl->reg_data,
					reg_conf_tbl->mask,
					MSM_CAMERA_I2C_BYTE_DATA);
				break;
			default:
				pr_err("%s: Unsupport data type: %d\n",
					__func__, dt);
				break;
			}
		}
		if (rc < 0)
			break;
		reg_conf_tbl++;
	}
	return rc;
}

int32_t msm_camera_i2c_read_b(struct msm_camera_i2c_client *client,
	uint16_t addr, uint16_t *data)
{
	int32_t rc = -EFAULT;
	enum msm_camera_i2c_data_type data_type = MSM_CAMERA_I2C_BYTE_DATA;
	unsigned char buf[client->addr_type+data_type];

	client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	if (client->addr_type == MSM_CAMERA_I2C_BYTE_ADDR) {
		buf[0] = addr;
	} else if (client->addr_type == MSM_CAMERA_I2C_WORD_ADDR) {
		buf[0] = addr >> BITS_PER_BYTE;
		buf[1] = addr;
	}
	rc = msm_camera_i2c_rxdata(client, buf, data_type);
	if (rc < 0) {
		S_I2C_DBG("%s fail\n", __func__);
		return rc;
	}
	if (data_type == MSM_CAMERA_I2C_BYTE_DATA)
		*data = buf[0];
	else
		*data = buf[0] << 8 | buf[1];

	S_I2C_DBG("%s addr = 0x%x data: 0x%x", __func__, addr, *data);
	return rc;
}

int32_t msm_camera_i2c_read(struct msm_camera_i2c_client *client,
	uint16_t addr, uint16_t *data,
	enum msm_camera_i2c_data_type data_type)
{
	int32_t rc = -EFAULT;
	unsigned char buf[client->addr_type+data_type];

	if ((client->addr_type != MSM_CAMERA_I2C_BYTE_ADDR
		&& client->addr_type != MSM_CAMERA_I2C_WORD_ADDR)
		|| (data_type != MSM_CAMERA_I2C_BYTE_DATA
		&& data_type != MSM_CAMERA_I2C_WORD_DATA))
		return rc;

	if (client->addr_type == MSM_CAMERA_I2C_BYTE_ADDR) {
		buf[0] = addr;
	} else if (client->addr_type == MSM_CAMERA_I2C_WORD_ADDR) {
		buf[0] = addr >> BITS_PER_BYTE;
		buf[1] = addr;
	}
	rc = msm_camera_i2c_rxdata(client, buf, data_type);
	if (rc < 0) {
		S_I2C_DBG("%s fail\n", __func__);
		return rc;
	}
	if (data_type == MSM_CAMERA_I2C_BYTE_DATA)
		*data = buf[0];
	else
		*data = buf[0] << 8 | buf[1];

	S_I2C_DBG("%s addr = 0x%x data: 0x%x\n", __func__, addr, *data);
	return rc;
}

int32_t msm_camera_i2c_read_seq(struct msm_camera_i2c_client *client,
	uint16_t addr, uint8_t *data, uint16_t num_byte)
{
	int32_t rc = -EFAULT;
	unsigned char buf[client->addr_type+num_byte];
	int i;

	memset(buf, 0, sizeof(buf));
	if ((client->addr_type != MSM_CAMERA_I2C_BYTE_ADDR
		&& client->addr_type != MSM_CAMERA_I2C_WORD_ADDR)
		|| num_byte == 0)
		return rc;

	if (client->addr_type == MSM_CAMERA_I2C_BYTE_ADDR) {
		buf[0] = addr;
	} else if (client->addr_type == MSM_CAMERA_I2C_WORD_ADDR) {
		buf[0] = addr >> BITS_PER_BYTE;
		buf[1] = addr;
	}
	rc = msm_camera_i2c_rxdata(client, buf, num_byte);
	if (rc < 0) {
		S_I2C_DBG("%s fail\n", __func__);
		return rc;
	}

	S_I2C_DBG("%s addr = 0x%x", __func__, addr);
	for (i = 0; i < num_byte; i++) {
		data[i] = buf[i];
		S_I2C_DBG("Byte %d: 0x%x\n", i, buf[i]);
		S_I2C_DBG("Data: 0x%x\n", data[i]);
	}
	return rc;
}

int32_t msm_sensor_write_conf_array(struct msm_camera_i2c_client *client,
			struct msm_camera_i2c_conf_array *array, uint16_t index)
{
	int32_t rc;

	rc = msm_camera_i2c_write_tbl(client,
		(struct msm_camera_i2c_reg_conf *) array[index].conf,
		array[index].size, array[index].data_type);
	if (array[index].delay > 20)
		msleep(array[index].delay);
	else
		usleep_range(array[index].delay*1000,
					(array[index].delay+1)*1000);
	return rc;
}

int32_t msm_sensor_write_enum_conf_array(struct msm_camera_i2c_client *client,
			struct msm_camera_i2c_enum_conf_array *conf,
			uint16_t enum_val)
{
	int32_t rc = -1, i;
	for (i = 0; i < conf->num_enum; i++) {
		if (conf->conf_enum[i] == enum_val)
			break;
		if (conf->conf_enum[i] > enum_val)
			break;
	}
	if (i == conf->num_enum)
		i = conf->num_enum - 1;

	if (i >= conf->num_index)
		return rc;

	rc = msm_sensor_write_all_conf_array(client,
		&conf->conf[i*conf->num_conf], conf->num_conf);

	if (conf->delay > 20)
		msleep(conf->delay);
	else
		usleep_range(conf->delay*1000,
					(conf->delay+1)*1000);
	return rc;
}

int32_t msm_sensor_write_all_conf_array(struct msm_camera_i2c_client *client,
			struct msm_camera_i2c_conf_array *array, uint16_t size)
{
	int32_t rc = 0, i;
	for (i = 0; i < size; i++) {
		rc = msm_sensor_write_conf_array(client, array, i);
		if (rc < 0)
			break;
	}
	return rc;
}

int32_t msm_camera_i2c_rxdata_rumbas(struct msm_camera_i2c_client *dev_client,
	unsigned char *rxdata, int data_length)
{
	int32_t rc = 0;
	uint16_t saddr = dev_client->client->addr >> 1;
	struct i2c_msg msgs[] = {
		{
			.addr  = saddr,
			.flags = 0,
			.len   = data_length,
			.buf   = rxdata,
		},
		{
			.addr  = saddr,
			.flags = I2C_M_RD,
			.len   = data_length,
			.buf   = rxdata,
		},
	};
	rc = i2c_transfer(dev_client->client->adapter, msgs, 2);
	if (rc < 0)
		S_I2C_DBG("msm_camera_i2c_rxdata failed 0x%x\n", saddr);
	return (rc < 0) ? rc : 0;
}


int32_t msm_camera_i2c_read_seq_rumbas(struct msm_camera_i2c_client *client,
	uint16_t addr, uint8_t *data, uint16_t num_byte)
{
	int32_t rc = -EFAULT;
	unsigned char buf[client->addr_type+num_byte];
	int i;

	memset(buf, 0, sizeof(buf));
	if ((client->addr_type != MSM_CAMERA_I2C_BYTE_ADDR
		&& client->addr_type != MSM_CAMERA_I2C_WORD_ADDR)
		|| num_byte == 0)
		return rc;

	if (client->addr_type == MSM_CAMERA_I2C_BYTE_ADDR) {
		buf[0] = addr;
	} else if (client->addr_type == MSM_CAMERA_I2C_WORD_ADDR) {
		buf[0] = addr >> BITS_PER_BYTE;
		buf[1] = addr;
	}

	rc = msm_camera_i2c_rxdata_rumbas(client, buf, num_byte);
	if (rc < 0) {
		S_I2C_DBG("%s fail\n", __func__);
		return rc;
	}

	S_I2C_DBG("%s addr = 0x%x", __func__, addr);
	for (i = 0; i < num_byte; i++) {
		data[i] = buf[i];
		S_I2C_DBG("Byte %d: 0x%x\n", i, buf[i]);
		S_I2C_DBG("Data: 0x%x\n", data[i]);
	}
	return rc;
}

int32_t msm_camera_i2c_txdata_rumbas(struct msm_camera_i2c_client *dev_client, unsigned char *txdata, int length)
{
	int32_t rc = 0;
	uint16_t saddr = dev_client->client->addr >> 1;

	struct i2c_msg msg[] = {
		{
			.addr = saddr,
			.flags = 0,
			.len = length,
			.buf = txdata,
		 },
	};
	rc = i2c_transfer(dev_client->client->adapter, msg, 1);
	
	return rc;
}
