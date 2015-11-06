/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  $
 */



#include <string.h>

#ifdef __KERNEL__
#include <linux/module.h>
#endif

#include "mpu.h"
#include "mlsl.h"
#include "mlos.h"

#include <log.h>
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-compass"


#define AK8975_REG_ST1  (0x02)
#define AK8975_REG_HXL  (0x03)
#define AK8975_REG_ST2  (0x09)

#define AK8975_REG_CNTL (0x0A)

#define AK8975_CNTL_MODE_POWER_DOWN         (0x00)
#define AK8975_CNTL_MODE_SINGLE_MEASUREMENT (0x01)

int ak8975_suspend(void *mlsl_handle,
		   struct ext_slave_descr *slave,
		   struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  AK8975_REG_CNTL,
				  AK8975_CNTL_MODE_POWER_DOWN);
	MLOSSleep(1);		
	ERROR_CHECK(result);
	return result;
}

int ak8975_resume(void *mlsl_handle,
		  struct ext_slave_descr *slave,
		  struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  AK8975_REG_CNTL,
				  AK8975_CNTL_MODE_SINGLE_MEASUREMENT);
	ERROR_CHECK(result);
	return result;
}

int ak8975_read(void *mlsl_handle,
		struct ext_slave_descr *slave,
		struct ext_slave_platform_data *pdata, unsigned char *data)
{
	unsigned char regs[8];
	unsigned char *stat = &regs[0];
	unsigned char *stat2 = &regs[7];
	int result = ML_SUCCESS;
	int status = ML_SUCCESS;

	result =
	    MLSLSerialRead(mlsl_handle, pdata->address, AK8975_REG_ST1,
			   8, regs);
	ERROR_CHECK(result);

	if (*stat & 0x01) {
		memcpy(data, &regs[1], 6);
		status = ML_SUCCESS;
	}

	if (*stat2 & 0x04)
		status = ML_ERROR_COMPASS_DATA_ERROR;
	if (*stat2 & 0x08)
		status = ML_ERROR_COMPASS_DATA_OVERFLOW;
	if (*stat & 0x02) {
		
		status = ML_SUCCESS;
	}

	if (*stat != 0x00 || *stat2 != 0x00) {
		result =
		    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					  AK8975_REG_CNTL,
					  AK8975_CNTL_MODE_SINGLE_MEASUREMENT);
		ERROR_CHECK(result);
	}

	return status;
}

static int ak8975_config(void *mlsl_handle,
			struct ext_slave_descr *slave,
			struct ext_slave_platform_data *pdata,
			struct ext_slave_config *data)
{
	int result;
	if (!data->data)
		return ML_ERROR_INVALID_PARAMETER;

	switch (data->key) {
	case MPU_SLAVE_WRITE_REGISTERS:
		result = MLSLSerialWrite(mlsl_handle, pdata->address,
					data->len,
					(unsigned char *)data->data);
		ERROR_CHECK(result);
		break;
	case MPU_SLAVE_CONFIG_ODR_SUSPEND:
	case MPU_SLAVE_CONFIG_ODR_RESUME:
	case MPU_SLAVE_CONFIG_FSR_SUSPEND:
	case MPU_SLAVE_CONFIG_FSR_RESUME:
	case MPU_SLAVE_CONFIG_MOT_THS:
	case MPU_SLAVE_CONFIG_NMOT_THS:
	case MPU_SLAVE_CONFIG_MOT_DUR:
	case MPU_SLAVE_CONFIG_NMOT_DUR:
	case MPU_SLAVE_CONFIG_IRQ_SUSPEND:
	case MPU_SLAVE_CONFIG_IRQ_RESUME:
	default:
		return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
	};

	return ML_SUCCESS;
}

static int ak8975_get_config(void *mlsl_handle,
				struct ext_slave_descr *slave,
				struct ext_slave_platform_data *pdata,
				struct ext_slave_config *data)
{
	int result;
	if (!data->data)
		return ML_ERROR_INVALID_PARAMETER;

	switch (data->key) {
	case MPU_SLAVE_READ_REGISTERS:
	{
		unsigned char *serial_data = (unsigned char *)data->data;
		result = MLSLSerialRead(mlsl_handle, pdata->address,
					serial_data[0],
					data->len - 1,
					&serial_data[1]);
		ERROR_CHECK(result);
		break;
	}
	case MPU_SLAVE_CONFIG_ODR_SUSPEND:
	case MPU_SLAVE_CONFIG_ODR_RESUME:
	case MPU_SLAVE_CONFIG_FSR_SUSPEND:
	case MPU_SLAVE_CONFIG_FSR_RESUME:
	case MPU_SLAVE_CONFIG_MOT_THS:
	case MPU_SLAVE_CONFIG_NMOT_THS:
	case MPU_SLAVE_CONFIG_MOT_DUR:
	case MPU_SLAVE_CONFIG_NMOT_DUR:
	case MPU_SLAVE_CONFIG_IRQ_SUSPEND:
	case MPU_SLAVE_CONFIG_IRQ_RESUME:
	default:
		return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
	};

	return ML_SUCCESS;
}

struct ext_slave_descr ak8975_descr = {
	 NULL,
	 NULL,
	 ak8975_suspend,
	 ak8975_resume,
	 ak8975_read,
	 ak8975_config,
	 ak8975_get_config,
	 "ak8975",
	 EXT_SLAVE_TYPE_COMPASS,
	 COMPASS_ID_AKM,
	 0x01,
	 9,
	 EXT_SLAVE_LITTLE_ENDIAN,
	 {9830, 4000}
};

struct ext_slave_descr *ak8975_get_slave_descr(void)
{
	return &ak8975_descr;
}
EXPORT_SYMBOL(ak8975_get_slave_descr);

