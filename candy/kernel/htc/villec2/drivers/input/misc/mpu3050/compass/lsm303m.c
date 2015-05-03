/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.
 $
 */

/**
 *  @brief      Provides the interface to setup and handle a compass
 *              connected to the primary I2C interface of the gyroscope.
 *
 *  @{
 *      @file   lsm303m.c
 *      @brief  Magnetometer setup and handling methods for ST LSM303.
 */

/* ------------------ */
/* - Include Files. - */
/* ------------------ */

#ifdef __KERNEL__
#include <linux/module.h>
#endif

#include "mpu.h"
#include "mlsl.h"
#include "mlos.h"

#include <linux/i2c/lsm303dlh.h>

#include <log.h>
#undef MPL_LOG_TAG
#define MPL_LOG_TAG "MPL-compass"

/*----- ST LSM303 Registers ------*/
enum LSM_REG {
	LSM_REG_CONF_A = 0x0,
	LSM_REG_CONF_B = 0x1,
	LSM_REG_MODE = 0x2,
	LSM_REG_X_M = 0x3,
	LSM_REG_X_L = 0x4,
	LSM_REG_Z_M = 0x5,
	LSM_REG_Z_L = 0x6,
	LSM_REG_Y_M = 0x7,
	LSM_REG_Y_L = 0x8,
	LSM_REG_STATUS = 0x9,
	LSM_REG_ID_A = 0xA,
	LSM_REG_ID_B = 0xB,
	LSM_REG_ID_C = 0xC
};

enum LSM_CONF_A {
	LSM_CONF_A_DRATE_MASK = 0x1C,
	LSM_CONF_A_DRATE_0_75 = 0x00,
	LSM_CONF_A_DRATE_1_5 = 0x04,
	LSM_CONF_A_DRATE_3 = 0x08,
	LSM_CONF_A_DRATE_7_5 = 0x0C,
	LSM_CONF_A_DRATE_15 = 0x10,
	LSM_CONF_A_DRATE_30 = 0x14,
	LSM_CONF_A_DRATE_75 = 0x18,
	LSM_CONF_A_MEAS_MASK = 0x3,
	LSM_CONF_A_MEAS_NORM = 0x0,
	LSM_CONF_A_MEAS_POS = 0x1,
	LSM_CONF_A_MEAS_NEG = 0x2
};

enum LSM_CONF_B {
	LSM_CONF_B_GAIN_MASK = 0xE0,
	LSM_CONF_B_GAIN_0_9 = 0x00,
	LSM_CONF_B_GAIN_1_2 = 0x20,
	LSM_CONF_B_GAIN_1_9 = 0x40,
	LSM_CONF_B_GAIN_2_5 = 0x60,
	LSM_CONF_B_GAIN_4_0 = 0x80,
	LSM_CONF_B_GAIN_4_6 = 0xA0,
	LSM_CONF_B_GAIN_5_5 = 0xC0,
	LSM_CONF_B_GAIN_7_9 = 0xE0
};

enum LSM_MODE {
	LSM_MODE_MASK = 0x3,
	LSM_MODE_CONT = 0x0,
	LSM_MODE_SINGLE = 0x1,
	LSM_MODE_IDLE = 0x2,
	LSM_MODE_SLEEP = 0x3
};

/* --------------------- */
/* -    Variables.     - */
/* --------------------- */

/*****************************************
    Accelerometer Initialization Functions
*****************************************/

#ifdef CONFIG_INPUT_LSM303DLH
static struct lsm303dlh_mag_data* lsm303dlh_mag_misc_data = NULL;

extern struct lsm303dlh_mag_data * lsm303dlh_mag_get_instance_ext(void);
extern int lsm303dlh_mag_enable_ext(struct lsm303dlh_mag_data *mag);
extern int lsm303dlh_mag_disable_ext(struct lsm303dlh_mag_data *mag);
#endif

int lsm303dlhm_suspend(void *mlsl_handle,
		       struct ext_slave_descr *slave,
		       struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;
#ifdef CONFIG_INPUT_LSM303DLH
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  LSM_REG_MODE, LSM_MODE_SLEEP);
	ERROR_CHECK(result);
	MLOSSleep(3);
#endif
	return result;
}

int lsm303dlhm_resume(void *mlsl_handle,
		      struct ext_slave_descr *slave,
		      struct ext_slave_platform_data *pdata)
{
	int result = ML_SUCCESS;

#ifdef CONFIG_INPUT_LSM303DLH
	if (!lsm303dlh_mag_misc_data)
	{
		lsm303dlh_mag_misc_data = lsm303dlh_mag_get_instance_ext();
	}
	if (lsm303dlh_mag_misc_data)
	{
		lsm303dlh_mag_misc_data->ext_handle=mlsl_handle;
		result = lsm303dlh_mag_enable_ext(lsm303dlh_mag_misc_data);
		MLOSSleep(50);
	}
	else
	{
		pr_err("%s: lsm303dlh_mag_misc_data is NULL\n", __func__);
	}
#endif

	/* Use single measurement mode. Start at sleep state. */
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  LSM_REG_MODE, LSM_MODE_SLEEP);
	ERROR_CHECK(result);
	/* Config normal measurement */
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  LSM_REG_CONF_A, 0);
	ERROR_CHECK(result);
	/* Adjust gain to 320 LSB/Gauss */
	result =
	    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
				  LSM_REG_CONF_B, LSM_CONF_B_GAIN_5_5);
	ERROR_CHECK(result);

	return result;
}

int lsm303dlhm_read(void *mlsl_handle,
		    struct ext_slave_descr *slave,
		    struct ext_slave_platform_data *pdata,
		    unsigned char *data)
{
	unsigned char stat;
	tMLError result = ML_SUCCESS;
	short zAxisfixed;

	/* Read status reg. to check if data is ready */
	result =
	    MLSLSerialRead(mlsl_handle, pdata->address, LSM_REG_STATUS, 1,
			   &stat);
	ERROR_CHECK(result);
	if (stat & 0x01) {
		result =
		    MLSLSerialRead(mlsl_handle, pdata->address,
				   LSM_REG_X_M, 6, (unsigned char *) data);
		ERROR_CHECK(result);

		/*drop data if overflows */
		if ((data[0] == 0xf0) || (data[2] == 0xf0)
		    || (data[4] == 0xf0)) {
			return ML_ERROR_COMPASS_DATA_OVERFLOW;
		}
		/* convert to fixed point and apply sensitivity correction for
		   Z-axis */
		zAxisfixed =
		    (short) ((unsigned short) data[5] +
			     (unsigned short) data[4] * 256);
		/* scale up by 1.122 (320/285) */
		zAxisfixed = (short) (zAxisfixed * 9) >> 3;
		data[4] = zAxisfixed >> 8;
		data[5] = zAxisfixed & 0xFF;

		/* trigger next measurement read */
		result =
		    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					  LSM_REG_MODE, LSM_MODE_SINGLE);
		ERROR_CHECK(result);

		return ML_SUCCESS;
	} else {
		/* trigger next measurement read */
		result =
		    MLSLSerialWriteSingle(mlsl_handle, pdata->address,
					  LSM_REG_MODE, LSM_MODE_SINGLE);
		ERROR_CHECK(result);

		return ML_ERROR_COMPASS_DATA_NOT_READY;
	}
}

struct ext_slave_descr lsm303dlhm_descr = {
	/*.init             = */ NULL,
	/*.exit             = */ NULL,
	/*.suspend          = */ lsm303dlhm_suspend,
	/*.resume           = */ lsm303dlhm_resume,
	/*.read             = */ lsm303dlhm_read,
	/*.config           = */ NULL,
	/*.get_config       = */ NULL,
	/*.name             = */ "lsm303dlhm",
	/*.type             = */ EXT_SLAVE_TYPE_COMPASS,
	/*.id               = */ COMPASS_ID_LSM303,
	/*.reg              = */ 0x06,
	/*.len              = */ 6,
	/*.endian           = */ EXT_SLAVE_BIG_ENDIAN,
	/*.range            = */ {10240, 0},
};

struct ext_slave_descr *lsm303dlhm_get_slave_descr(void)
{
	return &lsm303dlhm_descr;
}
EXPORT_SYMBOL(lsm303dlhm_get_slave_descr);

/**
 *  @}
**/
