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

#include <linux/leds.h>
#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_himax.h"
#include "mdp4.h"

#include <mach/debug_display.h>

struct dcs_cmd_req cmdreq_himax;
static struct mipi_dsi_panel_platform_data *mipi_himax_pdata;

static struct dsi_buf himax_tx_buf;
static struct dsi_buf himax_rx_buf;
static int mipi_himax_lcd_init(void);
static void mipi_himax_set_backlight(struct msm_fb_data_type *mfd);

static int wled_trigger_initialized;
static atomic_t lcd_power_state;
static int bl_off;

static char enter_sleep[2] = {0x10, 0x00}; 
static char exit_sleep[2] = {0x11, 0x00}; 
static char display_off[2] = {0x28, 0x00}; 
static char display_on[2] = {0x29, 0x00}; 

static char led_pwm1[] = {0x51, 0xff}; 
static char led_pwm2[] = {0x53, 0x24}; 
static char pwm_off[]  = {0x51, 0x00};

static char himax_b9[] = {0xB9, 0xFF, 0x83, 0x92}; 
static char himax_d4[] = {0xD4, 0x00}; 
static char himax_ba[] = {0xBA, 0x12, 0x83, 0x00, 0xD6, 0xC6, 0x00, 0x0A}; 
static char himax_c0[] = {0xC0, 0x01, 0x94}; 
static char himax_c6[] = {0xC6, 0x35, 0x00, 0x00, 0x04}; 
static char himax_d5[] = {0xD5, 0x00, 0x00, 0x02}; 
static char himax_bf[] = {0xBF, 0x05, 0x60, 0x02}; 
static char himax_b2[] = {0xB2, 0x0F, 0xC8, 0x04, 0x0C, 0x04}; 
static char himax_35[] = {0x35, 0x00}; 
static char himax_c2[] = {0xC2, 0x08}; 
static char himax_36[] = {0x36, 0x03};
static char himax_c9[] = {0xC9, 0x0F, 0x00, 0x1E, 0x3F, 0x00, 0x80}; 


static struct dsi_cmd_desc himax_video_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc himax_cmd_on_rotation_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_b9), himax_b9},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,  sizeof(himax_d4), himax_d4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_ba), himax_ba},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_c0), himax_c0},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_c6), himax_c6},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_d5), himax_d5},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_bf), himax_bf},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_b2), himax_b2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,  sizeof(himax_35), himax_35},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,  sizeof(himax_c2), himax_c2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,  sizeof(himax_36), himax_36},

	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,  sizeof(led_pwm2), led_pwm2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_c9), himax_c9},
};

static struct dsi_cmd_desc himax_cmd_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_b9), himax_b9},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,  sizeof(himax_d4), himax_d4},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_ba), himax_ba},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_c0), himax_c0},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_c6), himax_c6},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_d5), himax_d5},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_bf), himax_bf},

	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_b2), himax_b2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,  sizeof(himax_35), himax_35},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,  sizeof(himax_c2), himax_c2},

	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,  sizeof(led_pwm2), led_pwm2},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 1,  sizeof(himax_c9), himax_c9},
};

static struct dsi_cmd_desc himax_display_off_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 1,
		sizeof(pwm_off), pwm_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 1,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 130,
		sizeof(enter_sleep), enter_sleep},
};
static struct dsi_cmd_desc himax_display_on_cmds[] = {  
	{DTYPE_DCS_WRITE, 1, 0, 0, 40, sizeof(display_on), display_on},
};


static struct dsi_cmd_desc himax_cmd_backlight_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(led_pwm1), led_pwm1},
};

static int mipi_himax_lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct msm_panel_info *pinfo;

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	pinfo = &mfd->panel_info;
	mipi  = &mfd->panel_info.mipi;

	if (mfd->init_mipi_lcd == 0) {
		mfd->init_mipi_lcd = 1;
		return 0;
	}

	PR_DISP_DEBUG("Display On \n");

	if (mipi->mode == DSI_VIDEO_MODE) {
		cmdreq_himax.cmds = himax_video_on_cmds;
		cmdreq_himax.cmds_cnt = ARRAY_SIZE(himax_video_on_cmds);
	} else {
		if (system_rev == 1) {
			cmdreq_himax.cmds = himax_cmd_on_rotation_cmds;
			cmdreq_himax.cmds_cnt = ARRAY_SIZE(himax_cmd_on_rotation_cmds);
		}
		else {
			cmdreq_himax.cmds = himax_cmd_on_cmds;
			cmdreq_himax.cmds_cnt = ARRAY_SIZE(himax_cmd_on_cmds);
		}
	}

	cmdreq_himax.flags = CMD_REQ_COMMIT;
	cmdreq_himax.rlen = 0;
	cmdreq_himax.cb = NULL;
	mipi_dsi_cmdlist_put(&cmdreq_himax);

	atomic_set(&lcd_power_state, 1);

	return 0;
}

static int mipi_himax_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	cmdreq_himax.cmds = himax_display_off_cmds;
	cmdreq_himax.cmds_cnt = ARRAY_SIZE(himax_display_off_cmds);
	cmdreq_himax.flags = CMD_REQ_COMMIT;
	cmdreq_himax.rlen = 0;
	cmdreq_himax.cb = NULL;
	mipi_dsi_cmdlist_put(&cmdreq_himax);

	
	bl_off = 1;
	mipi_himax_set_backlight(mfd);
	

	atomic_set(&lcd_power_state, 0);

	return 0;
}

DEFINE_LED_TRIGGER(bkl_led_trigger);

static void mipi_himax_set_backlight(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;

	
	if (bl_off) {
		mfd->bl_level = 0;
		bl_off = 0;
	}
	
	if (mipi_himax_pdata && mipi_himax_pdata->shrink_pwm)
		led_pwm1[1] = mipi_himax_pdata->shrink_pwm(mfd->bl_level);
	else
		led_pwm1[1] = (unsigned char)(mfd->bl_level);

	if (mipi_himax_pdata && (mipi_himax_pdata->enable_wled_bl_ctrl)
	    && (wled_trigger_initialized)) {
		led_trigger_event(bkl_led_trigger, led_pwm1[1]);
		return;
	}
	mipi  = &mfd->panel_info.mipi;
	pr_debug("%s+:bl=%d \n", __func__, mfd->bl_level);

	
	if (atomic_read(&lcd_power_state) == 0) {
		PR_DISP_DEBUG("%s: LCD is off. Skip backlight setting\n", __func__);
		return;
	}

	if (mipi->mode == DSI_VIDEO_MODE && mdp4_overlay_dsi_state_get() <= ST_DSI_SUSPEND) {
		return;
	}
	

	if (mipi->mode == DSI_CMD_MODE) {
		mipi_dsi_op_mode_config(DSI_CMD_MODE);
	}

	cmdreq_himax.cmds = himax_cmd_backlight_cmds;
	cmdreq_himax.cmds_cnt = ARRAY_SIZE(himax_cmd_backlight_cmds);
	cmdreq_himax.flags = CMD_REQ_COMMIT;
	cmdreq_himax.rlen = 0;
	cmdreq_himax.cb = NULL;
	mipi_dsi_cmdlist_put(&cmdreq_himax);

#ifdef CONFIG_BACKLIGHT_WLED_CABC
	
	if (wled_trigger_initialized) {
		led_trigger_event(bkl_led_trigger, mfd->bl_level);
	}
#endif
	return;
}

static void mipi_himax_display_on(struct msm_fb_data_type *mfd)
{

    
    PR_DISP_DEBUG("%s\n",  __FUNCTION__);

	mipi_dsi_op_mode_config(DSI_CMD_MODE);

	cmdreq_himax.cmds = himax_display_on_cmds;
	cmdreq_himax.cmds_cnt = ARRAY_SIZE(himax_display_on_cmds);
	cmdreq_himax.flags = CMD_REQ_COMMIT;
	cmdreq_himax.rlen = 0;
	cmdreq_himax.cb = NULL;
	mipi_dsi_cmdlist_put(&cmdreq_himax);
}



static int __devinit mipi_himax_lcd_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct platform_device *current_pdev;
	static struct mipi_dsi_phy_ctrl *phy_settings;
	static char dlane_swap;

	if (pdev->id == 0) {
		mipi_himax_pdata = pdev->dev.platform_data;

		if (mipi_himax_pdata
			&& mipi_himax_pdata->phy_ctrl_settings) {
			phy_settings = (mipi_himax_pdata->phy_ctrl_settings);
		}

		if (mipi_himax_pdata
			&& mipi_himax_pdata->dlane_swap) {
			dlane_swap = (mipi_himax_pdata->dlane_swap);
		}

		return 0;
	}

	current_pdev = msm_fb_add_device(pdev);

	if (current_pdev) {
		mfd = platform_get_drvdata(current_pdev);
		if (!mfd)
			return -ENODEV;
		if (mfd->key != MFD_KEY)
			return -EINVAL;

		mipi  = &mfd->panel_info.mipi;

		if (phy_settings != NULL)
			mipi->dsi_phy_db = phy_settings;

		if (dlane_swap)
			mipi->dlane_swap = dlane_swap;
	}
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = mipi_himax_lcd_probe,
	.driver = {
		.name   = "mipi_himax",
	},
};

static struct msm_fb_panel_data himax_panel_data = {
	.on		= mipi_himax_lcd_on,
	.off		= mipi_himax_lcd_off,
	.set_backlight  = mipi_himax_set_backlight,
	.display_on = mipi_himax_display_on,
};

static int ch_used[3];

int mipi_himax_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	ret = mipi_himax_lcd_init();
	if (ret) {
		pr_err("mipi_himax_lcd_init() failed with ret %u\n", ret);
		return ret;
	}

	pdev = platform_device_alloc("mipi_himax", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	himax_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &himax_panel_data,
		sizeof(himax_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int mipi_himax_lcd_init(void)
{
	led_trigger_register_simple("bkl_trigger", &bkl_led_trigger);
	pr_info("%s: SUCCESS (WLED TRIGGER)\n", __func__);
	wled_trigger_initialized = 1;
	atomic_set(&lcd_power_state, 1);

	mipi_dsi_buf_alloc(&himax_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&himax_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

