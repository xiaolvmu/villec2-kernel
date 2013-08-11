/* Copyright (c) 2012, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/mach-types.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <mach/board.h>
#include <mach/msm_bus_board.h>
#include <mach/gpiomux.h>
#include <asm/setup.h>
#include "devices-msm8x60.h"
#include "devices.h"
#include "board-villec2.h"

#include <linux/spi/spi.h>
#include <mach/rpm-regulator.h>

#include "board-mahimahi-flashlight.h"
#ifdef CONFIG_MSM_CAMERA_FLASH
#include <linux/htc_flashlight.h>
#include <linux/leds.h>
#endif

static int camera_sensor_power_enable(char *power, unsigned volt, struct regulator **sensor_power);
static int camera_sensor_power_disable(struct regulator *sensor_power);
static struct platform_device msm_camera_server = {
	.name = "msm_cam_server",
	.id = 0,
};

#ifdef CONFIG_MSM_CAMERA
static struct msm_bus_vectors cam_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_zsl_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 566231040,
		.ib  = 905969664,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 706199040,
		.ib  = 1129918464,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 320864256,
		.ib  = 513382810,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 320864256,
		.ib  = 513382810,
	},
};

static struct msm_bus_vectors cam_stereo_video_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 212336640,
		.ib  = 339738624,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 25090560,
		.ib  = 40144896,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 239708160,
		.ib  = 383533056,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 79902720,
		.ib  = 127844352,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_stereo_snapshot_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 300902400,
		.ib  = 481443840,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 230307840,
		.ib  = 368492544,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 245113344,
		.ib  = 392181351,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 106536960,
		.ib  = 170459136,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 106536960,
		.ib  = 170459136,
	},
};

static struct msm_bus_paths cam_bus_client_config[] = {
	{
		ARRAY_SIZE(cam_init_vectors),
		cam_init_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_zsl_vectors),
		cam_zsl_vectors,
	},
	{
		ARRAY_SIZE(cam_stereo_video_vectors),
		cam_stereo_video_vectors,
	},
	{
		ARRAY_SIZE(cam_stereo_snapshot_vectors),
		cam_stereo_snapshot_vectors,
	},
};

static struct msm_bus_scale_pdata cam_bus_client_pdata = {
		cam_bus_client_config,
		ARRAY_SIZE(cam_bus_client_config),
		.name = "msm_camera",
};

struct msm_camera_device_platform_data msm_camera_csi_device_data[] = {
	{
	.ioext.csiphy = 0x04800000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_0_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 266667000,
	.csid_core = 0,
	.is_csic = 1,
	.is_vpe = 1,
	.cam_bus_scale_table = &cam_bus_client_pdata,

	},
	{
	.ioext.csiphy = 0x04900000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = CSI_1_IRQ,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 266667000,
	.csid_core = 1,
	.is_csic = 1,
	.is_vpe = 1,
	.cam_bus_scale_table = &cam_bus_client_pdata,

	},
};

#ifdef CONFIG_MSM_CAMERA_FLASH
int flashlight_control(int mode)
{
#ifdef CONFIG_FLASHLIGHT_TPS61310
	int	rc;
	static int brightness = 255;
	static int backlight_off = 0;

	pr_info("[CAM] %s, linear led, mode %d backlight_off %d", __func__, mode, backlight_off);

	if (mode != FL_MODE_PRE_FLASH && mode != FL_MODE_OFF) {
		if (!backlight_off) {
			
			brightness = led_brightness_value_get("lcd-backlight");
			if (brightness >= 0 && brightness <= 255) {
				pr_info("[CAM] %s, Turn off backlight before flashlight, brightness %d", __func__, brightness);
				led_brightness_value_set("lcd-backlight", 0);
				backlight_off = 1;
			} else
				pr_err("[CAM] %s, Invalid brightness value!! brightness %d", __func__, brightness);
		}
	}

	rc = tps61310_flashlight_control(mode);

	if (mode == FL_MODE_PRE_FLASH || mode == FL_MODE_OFF) {
		if(backlight_off) {
			pr_info("[CAM] %s, Turn on backlight after flashlight, brightness %d", __func__, brightness);
			led_brightness_value_set("lcd-backlight", brightness);
			backlight_off = 0;
		}
	}

	return rc;
#else
	return 0;
#endif
}


static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	.camera_flash = flashlight_control,
};
#endif  

static struct regulator *villec2_reg_8058_l8 = NULL;
static struct regulator *villec2_reg_8058_l9 = NULL;
static struct regulator *villec2_reg_8058_l14 = NULL;
static struct regulator *villec2_reg_8058_l15 = NULL;
static struct regulator *votg_2_8v_switch = NULL;

#ifdef CONFIG_RAWCHIP
static int villec2_use_ext_1v2(void)
{
	return 0;
}
static int config_rawchip_on_gpios(void);

static int villec2_rawchip_vreg_on(void)
{
	int rc;
	pr_info("[CAM] %s\n", __func__);

	
	rc = camera_sensor_power_enable("8058_l14", 2850000, &villec2_reg_8058_l14);
	pr_info("[CAM] sensor_power_enable(\"8058_l14\", 2.85V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] rawchip_power_enable(\"8058_l14\", 2.8V) FAILED %d\n", rc);
		goto enable_VCM_fail;
	}

	
	rc = camera_sensor_power_enable("8058_l8", 1800000, &villec2_reg_8058_l8);
	pr_info("[CAM] sensor_power_enable(\"8058_l8\", 1.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] rawchip_power_enable(\"8058_l8\", 1.8V) FAILED %d\n", rc);
		goto enable_1v8_fail;
	}


	

	rc = gpio_request(VILLEC2_GPIO_V_CAM_D1V2_EN, "CAM_D1V2");
	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"gpio %d\", 1.2V) FAILED %d\n", VILLEC2_GPIO_V_CAM_D1V2_EN, rc);
		goto enable_1v2_fail;
	}
	gpio_direction_output(VILLEC2_GPIO_V_CAM_D1V2_EN, 1);
	gpio_free(VILLEC2_GPIO_V_CAM_D1V2_EN);

	
	
	rc = camera_sensor_power_enable("8058_l15", 2800000, &villec2_reg_8058_l15);
	pr_info("[CAM] sensor_power_enable(\"8058_l15\", 2.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"8058_l15\", 2.8V) FAILED %d\n", rc);
		goto enable_analog_fail;
	}

	votg_2_8v_switch = regulator_get(NULL, "8901_usb_otg");
	if (IS_ERR(votg_2_8v_switch)) {
		pr_err("[CAM] %s: unable to get votg_2_8v_switch\n", __func__);
		goto enable_analog_fail;
	}
	if (regulator_enable(votg_2_8v_switch)) {
		pr_err("[CAM] %s: Unable to enable the regulator: votg_2_8v_switch\n", __func__);
		regulator_put(votg_2_8v_switch);
		votg_2_8v_switch = NULL;
		goto enable_analog_fail;
	}

	msleep(1);

	
	rc = camera_sensor_power_enable("8058_l9", 1800000, &villec2_reg_8058_l9);
	pr_info("[CAM] sensor_power_enable(\"8058_l9\", 1.8V) == %d\n", rc);

	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"8058_l9\", 2.8V) FAILED %d\n", rc);
		goto lcmio_hi_fail;
	}

	config_rawchip_on_gpios();

	
	rc = gpio_request(VILLEC2_GPIO_MCLK_SWITCH, "CAM_SEL");
	if (rc < 0)
	{
		pr_err("[CAM] GPIO (%d) request fail\n", VILLEC2_GPIO_MCLK_SWITCH);
		goto lcmio_hi_fail;
	}
	gpio_direction_output(VILLEC2_GPIO_MCLK_SWITCH, 0);
	gpio_free(VILLEC2_GPIO_MCLK_SWITCH);

	return rc;

lcmio_hi_fail:
	camera_sensor_power_disable(villec2_reg_8058_l9);
enable_analog_fail:
	gpio_request(VILLEC2_GPIO_V_CAM_D1V2_EN, "CAM_D1V2_EN");
	gpio_direction_output(VILLEC2_GPIO_V_CAM_D1V2_EN, 0);
	gpio_free(VILLEC2_GPIO_V_CAM_D1V2_EN);
enable_1v2_fail:
	camera_sensor_power_disable(villec2_reg_8058_l8);
enable_1v8_fail:
	camera_sensor_power_disable(villec2_reg_8058_l14);
enable_VCM_fail:
	return rc;
}
static void config_rawchip_off_gpios(void);
static int villec2_rawchip_vreg_off(void)
{
	int rc = 0;

	pr_info("[CAM] %s\n", __func__);

	if ((votg_2_8v_switch == NULL) || IS_ERR(votg_2_8v_switch)) {
		pr_err("[CAM] %s: unable to get votg_2_8v_switch\n", __func__);
		goto ville_rawchip_vreg_off_fail;
	}
	if (regulator_disable(votg_2_8v_switch)) {
		pr_err("[CAM] %s: Unable to disable the regulator: votg_2_8v_switch\n", __func__);
		regulator_put(votg_2_8v_switch);
		votg_2_8v_switch = NULL;
		goto ville_rawchip_vreg_off_fail;
	}
	regulator_put(votg_2_8v_switch);
	votg_2_8v_switch = NULL;
	mdelay(1);

	rc = camera_sensor_power_disable(villec2_reg_8058_l15);
	pr_info("[CAM] sensor_power_disable(\"8058_l15\") == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] rawchip_power_disable(\"8058_l15\", 1.8V) FAILED %d\n", rc);
		goto ville_rawchip_vreg_off_fail;
	}


	rc = gpio_request(VILLEC2_GPIO_V_CAM_D1V2_EN, "RAW_1V2_EN");
	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"gpio %d\", 1.2V) FAILED %d\n", VILLEC2_GPIO_V_CAM_D1V2_EN, rc);
		goto ville_rawchip_vreg_off_fail;
	}
	gpio_direction_output(VILLEC2_GPIO_V_CAM_D1V2_EN, 0);
	gpio_free(VILLEC2_GPIO_V_CAM_D1V2_EN);


	rc = camera_sensor_power_disable(villec2_reg_8058_l9);
	pr_info("[CAM] sensor_power_disable(\"8058_l9\") == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] rawchip_power_disable(\"8058_l9\", 1.8V) FAILED %d\n", rc);
		goto ville_rawchip_vreg_off_fail;
	}


	rc = camera_sensor_power_disable(villec2_reg_8058_l8);
	pr_info("[CAM] sensor_power_disable(\"8058_l8\") == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_disable(\"8058_l9\") FAILED %d\n", rc);
		goto ville_rawchip_vreg_off_fail;
	}

	
	
	rc = camera_sensor_power_disable(villec2_reg_8058_l14);
	pr_info("[CAM] sensor_power_disable(\"8058_l14\") == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_disable(\"8058_l14\") FAILED %d\n", rc);
		goto ville_rawchip_vreg_off_fail;
	}
	config_rawchip_off_gpios();
	return rc;

ville_rawchip_vreg_off_fail:
	return rc;
}

static uint32_t rawchip_on_gpio_table[] = {
	GPIO_CFG(VILLEC2_GPIO_RAW_RSTN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	GPIO_CFG(VILLEC2_GPIO_RAW_INTR0, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	GPIO_CFG(VILLEC2_GPIO_RAW_INTR1, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	GPIO_CFG(VILLEC2_GPIO_MCAM_SPI_DO, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_GPIO_MCAM_SPI_DI, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_GPIO_MCAM_SPI_CS, 1 , GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_GPIO_MCAM_SPI_CLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_GPIO_CAM_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
	GPIO_CFG(VILLEC2_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
};

static uint32_t rawchip_off_gpio_table[] = {
	GPIO_CFG(VILLEC2_GPIO_MCAM_SPI_DO, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_GPIO_MCAM_SPI_DI, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_GPIO_MCAM_SPI_CS, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_GPIO_MCAM_SPI_CLK, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_CAM_I2C_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_CAM_I2C_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_CAM_ID, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(VILLEC2_GPIO_RAW_INTR0, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 
	GPIO_CFG(VILLEC2_GPIO_RAW_INTR1, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 
	GPIO_CFG(VILLEC2_GPIO_CAM_MCLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), 
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[CAM] %s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static int config_rawchip_on_gpios(void)
{
	pr_info("[CAM] config_rawchip_on_gpios\n");
	config_gpio_table(rawchip_on_gpio_table,
		ARRAY_SIZE(rawchip_on_gpio_table));
	return 0;
}
static void config_rawchip_off_gpios(void)
{
	pr_info("[CAM] config_rawchip_off_gpios\n");
	config_gpio_table(rawchip_off_gpio_table,
		ARRAY_SIZE(rawchip_off_gpio_table));
}

static struct msm_camera_rawchip_info msm_rawchip_board_info = {
	.rawchip_reset	= VILLEC2_GPIO_RAW_RSTN,
	.rawchip_intr0	= MSM_GPIO_TO_INT(VILLEC2_GPIO_RAW_INTR0),
	.rawchip_intr1	= MSM_GPIO_TO_INT(VILLEC2_GPIO_RAW_INTR1),
	.rawchip_spi_freq = 27, 
	.rawchip_mclk_freq = 24, 
	.camera_rawchip_power_on = villec2_rawchip_vreg_on,
	.camera_rawchip_power_off = villec2_rawchip_vreg_off,
	.rawchip_use_ext_1v2 = villec2_use_ext_1v2,
};

struct platform_device msm_rawchip_device = {
	.name	= "rawchip",
	.dev	= {
		.platform_data = &msm_rawchip_board_info,
	},
};

static struct spi_board_info spi_rawchip_board_info[] __initdata = {
	{
		.modalias	= "spi_rawchip",
		.mode           = SPI_MODE_0,
		.bus_num        = 0,
		.chip_select    = 0,
		.max_speed_hz   = 27000000,
	}
};

#endif  

static uint16_t msm_cam_gpio_tbl[] = {
	VILLEC2_GPIO_CAM_MCLK, 
	VILLEC2_GPIO_RAW_INTR0,
	VILLEC2_GPIO_RAW_INTR1,
	VILLEC2_GPIO_MCAM_SPI_CLK,
	VILLEC2_GPIO_MCAM_SPI_CS,
	VILLEC2_GPIO_MCAM_SPI_DI,
	VILLEC2_GPIO_MCAM_SPI_DO,
};


static struct msm_camera_gpio_conf gpio_conf = {
	.cam_gpiomux_conf_tbl = NULL,
	.cam_gpiomux_conf_tbl_size = 0,
	.cam_gpio_tbl = msm_cam_gpio_tbl,
	.cam_gpio_tbl_size = ARRAY_SIZE(msm_cam_gpio_tbl),
};

static int camera_sensor_power_enable(char *power, unsigned volt, struct regulator **sensor_power)
{
	int rc;

	if (power == NULL)
		return -ENODEV;

	*sensor_power = regulator_get(NULL, power);

	if (IS_ERR(*sensor_power)) {
		pr_info("%s: failed to Unable to get %s\n", __func__, power);
		return -ENODEV;
	}

	if (volt != 1800000) {
		rc = regulator_set_voltage(*sensor_power, volt, volt);
		if (rc < 0) {
			pr_info("%s: failed to unable to set %s voltage to %d rc:%d\n",
					__func__, power, volt, rc);
			regulator_put(*sensor_power);
			*sensor_power = NULL;
			return -ENODEV;
		}
	}

	rc = regulator_enable(*sensor_power);
	if (rc < 0) {
		pr_info("%s: failed to Enable regulator %s failed\n", __func__, power);
		regulator_put(*sensor_power);
		*sensor_power = NULL;
		return -ENODEV;
	}

	return rc;
}

static int camera_sensor_power_disable(struct regulator *sensor_power)
{

	int rc;
	if (sensor_power == NULL)
		return -ENODEV;

	if (IS_ERR(sensor_power)) {
		pr_info("%s: failed to Invalid requlator ptr\n", __func__);
		return -ENODEV;
	}

	rc = regulator_disable(sensor_power);
	if (rc < 0)
		pr_info("%s: disable regulator failed\n", __func__);

	regulator_put(sensor_power);
	sensor_power = NULL;
	return rc;
}

#ifdef CONFIG_S5K3H2YX
static int Villec2_s5k3h2yx_vreg_on(void)
{
	int rc = 0;
	pr_info("%s\n", __func__);

	return rc;

}

static int Villec2_s5k3h2yx_vreg_off(void)
{
	int rc = 0;

	pr_info("%s\n", __func__);

	return rc;
}

#ifdef CONFIG_S5K3H2YX_ACT
static struct i2c_board_info s5k3h2yx_actuator_i2c_info = {
	I2C_BOARD_INFO("s5k3h2yx_act", 0x11),
};

static struct msm_actuator_info s5k3h2yx_actuator_info = {
	.board_info     = &s5k3h2yx_actuator_i2c_info,
	.bus_id         = MSM_GSBI4_QUP_I2C_BUS_ID,
	.vcm_pwd        = VILLEC2_GPIO_CAM_VCM_PD,
	.vcm_enable     = 1,
};
#endif
static struct msm_camera_sensor_platform_info sensor_s5k3h2yx_board_info = {
	.mount_angle = 90,
	.mirror_flip = CAMERA_SENSOR_NONE,
	.sensor_reset_enable = 0,
	.sensor_reset	= 0,
	.sensor_pwd	= VILLEC2_GPIO_CAM_PWDN,
	.vcm_pwd	= VILLEC2_GPIO_CAM_VCM_PD,
	.vcm_enable	= 1,
};

static struct camera_led_est msm_camera_sensor_s5k3h2yx_led_table[] = {
		{
		.enable = 1,
		.led_state = FL_MODE_FLASH_LEVEL2,
		.current_ma = 200,
		.lumen_value = 250,
		.min_step = 64,
		.max_step = 256
	},
		{
		.enable = 1,
		.led_state = FL_MODE_FLASH_LEVEL3,
		.current_ma = 300,
		.lumen_value = 350,
		.min_step = 60,
		.max_step = 63
	},
		{
		.enable = 1,
		.led_state = FL_MODE_FLASH_LEVEL4,
		.current_ma = 400,
		.lumen_value = 440,
		.min_step = 56,
		.max_step = 59
	},
		{
		.enable = 1,
		.led_state = FL_MODE_FLASH_LEVEL6,
		.current_ma = 600,
		.lumen_value = 625,
		.min_step = 52,
		.max_step = 55
	},
		{
		.enable = 1,
		.led_state = FL_MODE_FLASH,
		.current_ma = 750,
		.lumen_value = 745,
		.min_step = 0,
		.max_step = 51
	},

		{
		.enable = 2,
		.led_state = FL_MODE_FLASH_LEVEL2,
		.current_ma = 200,
		.lumen_value = 250,
		.min_step = 0,
		.max_step = 270
	},
		{
		.enable = 0,
		.led_state = FL_MODE_FLASH_LEVEL3,
		.current_ma = 300,
		.lumen_value = 300,
		.min_step = 0,
		.max_step = 100
	},
	{
		.enable = 0,
		.led_state = FL_MODE_FLASH_LEVEL4,
		.current_ma = 400,
		.lumen_value = 400,
		.min_step = 101,
		.max_step = 200
	},
	{
		.enable = 0,
		.led_state = FL_MODE_FLASH_LEVEL7,
		.current_ma = 700,
		.lumen_value = 700,
		.min_step = 101,
		.max_step = 200
	},
		{
		.enable = 2,
		.led_state = FL_MODE_FLASH,
		.current_ma = 750,
		.lumen_value = 745,
		.min_step = 271,
		.max_step = 325
	},
	{
		.enable = 0,
		.led_state = FL_MODE_FLASH_LEVEL5,
		.current_ma = 500,
		.lumen_value = 500,
		.min_step = 25,
		.max_step = 26
	},
		{
		.enable = 0,
		.led_state = FL_MODE_FLASH,
		.current_ma = 750,
		.lumen_value = 750,
		.min_step = 271,
		.max_step = 325
	},
};

static struct camera_led_info msm_camera_sensor_s5k3h2yx_led_info = {
	.enable = 1,
	.low_limit_led_state = FL_MODE_TORCH,
	.max_led_current_ma = 750,
	.num_led_est_table = ARRAY_SIZE(msm_camera_sensor_s5k3h2yx_led_table),
};

static struct camera_flash_info msm_camera_sensor_s5k3h2yx_flash_info = {
	.led_info = &msm_camera_sensor_s5k3h2yx_led_info,
	.led_est_table = msm_camera_sensor_s5k3h2yx_led_table,
};

static struct camera_flash_cfg msm_camera_sensor_s5k3h2yx_flash_cfg = {
	.low_temp_limit		= 5,
	.low_cap_limit		= 15,
	.flash_info             = &msm_camera_sensor_s5k3h2yx_flash_info,
};

static struct msm_camera_sensor_flash_data flash_s5k3h2yx = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
#ifdef CONFIG_MSM_CAMERA_FLASH
	.flash_src	= &msm_flash_src,
#endif

};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3h2yx_data = {
	.sensor_name	= "s5k3h2yx",
	.camera_power_on = Villec2_s5k3h2yx_vreg_on,
	.camera_power_off = Villec2_s5k3h2yx_vreg_off,
	.pdata	= &msm_camera_csi_device_data[0],
	.flash_data	= &flash_s5k3h2yx,
	.sensor_platform_info = &sensor_s5k3h2yx_board_info,
	.gpio_conf = &gpio_conf,
	.csi_if	= 1,
	.camera_type = BACK_CAMERA_2D,
#ifdef CONFIG_S5K3H2YX_ACT
	.actuator_info = &s5k3h2yx_actuator_info,
#endif
	.use_rawchip = RAWCHIP_ENABLE, 
	.flash_cfg = &msm_camera_sensor_s5k3h2yx_flash_cfg, 
};

#endif	

#ifdef CONFIG_MT9V113
static uint32_t camera_off_gpio_table[] = {
	GPIO_CFG(VILLEC2_CAM_I2C_SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_CAM_I2C_SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_CAM_ID, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	GPIO_CFG(VILLEC2_GPIO_CAM_MCLK, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), 
};

static uint32_t camera_on_gpio_table[] = {
	GPIO_CFG(VILLEC2_CAM_I2C_SDA, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_CAM_I2C_SCL, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
	GPIO_CFG(VILLEC2_CAM_ID, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(VILLEC2_GPIO_CAM_MCLK, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
};
static int villec2_config_camera_on_gpios(void)
{
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
	return 0;
}

static void villec2_config_camera_off_gpios(void)
{
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static int Villec2_mt9v113_vreg_on(void)
{
	int rc = 0;

	pr_info("[CAM] %s\n", __func__);

	
	
	rc = camera_sensor_power_enable("8058_l14", 2850000, &villec2_reg_8058_l14);
	pr_info("[CAM] sensor_power_enable(\"8058_l14\", 2.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"8058_l14\", 2.8V) FAILED %d\n", rc);
		goto init_fail;
	}
	udelay(50);

	
	rc = camera_sensor_power_enable("8058_l8", 1800000, &villec2_reg_8058_l8);
	pr_info("[CAM] sensor_power_enable(\"8058_l8\", 1.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"8058_l8\", 1.8V) FAILED %d\n", rc);
		goto init_fail;
	}
	udelay(50);

	
	rc = camera_sensor_power_enable("8058_l15", 2800000, &villec2_reg_8058_l15);
	pr_info("[CAM] sensor_power_enable(\"8058_l15\", 2.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"8058_l15\", 2.8V) FAILED %d\n", rc);
		goto init_fail;
	}
	udelay(50);

	
	rc = camera_sensor_power_enable("8058_l9", 1800000, &villec2_reg_8058_l9);
	pr_info("[CAM] sensor_power_enable(\"8058_l9\", 1.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_enable(\"8058_l9\", 1.8V) FAILED %d\n", rc);
		goto init_fail;
	}
	udelay(50);

	
	pr_info("[CAM] Do Reset pin On\n");
	rc = gpio_request(VILLEC2_GPIO_CAM2_RSTz, "mt9v113");
	if (rc < 0) {
		pr_err("[CAM] %s:VILLEC2_GPIO_CAM2_RSTz gpio %d request failed, rc=%d\n", __func__,  VILLEC2_GPIO_CAM2_RSTz, rc);
		goto init_fail;
	}
	gpio_direction_output(VILLEC2_GPIO_CAM2_RSTz, 1);
	msleep(2);
	gpio_free(VILLEC2_GPIO_CAM2_RSTz);

	udelay(50);
	villec2_config_camera_on_gpios();

	rc = gpio_request(VILLEC2_GPIO_MCLK_SWITCH, "CAM_SEL");
	if (rc < 0)
	{
		pr_err("[CAM] GPIO (%d) request fail\n", VILLEC2_GPIO_MCLK_SWITCH);
		goto init_fail;
	}
	gpio_direction_output(VILLEC2_GPIO_MCLK_SWITCH, 1);
	gpio_free(VILLEC2_GPIO_MCLK_SWITCH);


init_fail:
	return rc;
}

static int Villec2_mt9v113_vreg_off(void)
{
	int rc = 0;

	pr_info("[CAM] %s\n", __func__);

	
	pr_info("[CAM] Do Reset pin Off\n");
	rc = gpio_request(VILLEC2_GPIO_CAM2_RSTz, "mt9v113");
	if (rc < 0) {
		pr_err("[CAM] %s:VILLEC2_GPIO_CAM2_RSTz gpio %d request failed, rc=%d\n", __func__,	VILLEC2_GPIO_CAM2_RSTz, rc);
		goto init_fail;
	}
	gpio_direction_output(VILLEC2_GPIO_CAM2_RSTz, 0);
	msleep(2);
	gpio_free(VILLEC2_GPIO_CAM2_RSTz);

	udelay(50);

	
	rc = camera_sensor_power_disable(villec2_reg_8058_l9);
	pr_info("[CAM] camera_sensor_power_disable(\"8058_l9\", 1.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_disable(\"8058_l9\") FAILED %d\n", rc);
		goto init_fail;
	}
	udelay(50);

	
	rc = camera_sensor_power_disable(villec2_reg_8058_l15);
	pr_info("[CAM] camera_sensor_power_disable(\"8058_l15\", 2.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_disable(\"8058_l15\") FAILED %d\n", rc);
		goto init_fail;
	}
	udelay(50);

	
	rc = camera_sensor_power_disable(villec2_reg_8058_l8);
	pr_info("[CAM] camera_sensor_power_disable(\"8058_l8\", 1.8V) == %d\n", rc);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_disable(\"8058_l8\") FAILED %d\n", rc);
		goto init_fail;
	}
	udelay(50);

	
	pr_info("[CAM] camera_sensor_power_disable(\"8058_l14\", 2.8V) == %d\n", rc);
	rc = camera_sensor_power_disable(villec2_reg_8058_l14);
	if (rc < 0) {
		pr_err("[CAM] sensor_power_disable(\"8058_l14\") FAILED %d\n", rc);
		goto init_fail;
	}

	villec2_config_camera_off_gpios();

	
	pr_info("[CAM] Doing clk switch to sleep state\n");
	rc = gpio_request(VILLEC2_GPIO_MCLK_SWITCH, "CAM_SEL");
	if (rc < 0)
	{
		pr_err("[CAM] GPIO (%d) request fail\n", VILLEC2_GPIO_MCLK_SWITCH);
		goto init_fail;
	}
	gpio_direction_output(VILLEC2_GPIO_MCLK_SWITCH, 0);
	gpio_free(VILLEC2_GPIO_MCLK_SWITCH);

init_fail:
		return rc;
}

static struct msm_camera_sensor_platform_info sensor_mt9v113_board_info = {
	.mount_angle = 270,
	.mirror_flip = CAMERA_SENSOR_NONE,
	.sensor_reset_enable = 1,
	.sensor_reset	= VILLEC2_GPIO_CAM2_RSTz,
	.sensor_pwd = VILLEC2_GPIO_CAM2_PWDN,
	.vcm_pwd	= 0,
	.vcm_enable	= 1,
};

static struct msm_camera_sensor_flash_data flash_mt9v113 = {
	.flash_type	= MSM_CAMERA_FLASH_NONE,
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9v113_data = {
	.sensor_name	= "mt9v113",
	.sensor_reset	= VILLEC2_GPIO_CAM2_RSTz,
	.camera_power_on = Villec2_mt9v113_vreg_on,
	.camera_power_off = Villec2_mt9v113_vreg_off,
	.pdata	= &msm_camera_csi_device_data[1],
	.flash_data	= &flash_mt9v113,
	.sensor_platform_info = &sensor_mt9v113_board_info,
	.gpio_conf = &gpio_conf,
	.csi_if	= 1,
	.camera_type = FRONT_CAMERA_2D,
	.use_rawchip = RAWCHIP_DISABLE, 
};
#endif  

#ifdef CONFIG_I2C
static struct i2c_board_info msm_camera_boardinfo[] = {
#ifdef CONFIG_S5K3H2YX
	{
	I2C_BOARD_INFO("s5k3h2yx", 0x20 >> 1),
	.platform_data = &msm_camera_sensor_s5k3h2yx_data,
	},
#endif
#ifdef CONFIG_MT9V113
	{
	I2C_BOARD_INFO("mt9v113", 0x3C),
	.platform_data = &msm_camera_sensor_mt9v113_data,
	},
#endif
};

#endif  


void __init msm8x60_init_cam(void)
{
	pr_info("%s", __func__);

	i2c_register_board_info(MSM_GSBI4_QUP_I2C_BUS_ID,
		msm_camera_boardinfo,
		ARRAY_SIZE(msm_camera_boardinfo));

#ifdef CONFIG_RAWCHIP
	spi_register_board_info(spi_rawchip_board_info,
				ARRAY_SIZE(spi_rawchip_board_info));
	platform_device_register(&msm_rawchip_device);
#endif

	platform_device_register(&msm_camera_server);

	platform_device_register(&msm_device_csic0);
	platform_device_register(&msm_device_csic1);
	platform_device_register(&msm_device_vfe);
	platform_device_register(&msm_device_vpe);
}

#endif	
