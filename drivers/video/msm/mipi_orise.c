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
#include "mipi_orise.h"
#include "mdp4.h"

#include <mach/debug_display.h>

struct dcs_cmd_req cmdreq_orise;
static struct mipi_dsi_panel_platform_data *mipi_orise_pdata;

static struct dsi_buf orise_tx_buf;
static struct dsi_buf orise_rx_buf;
static int mipi_orise_lcd_init(void);

static int wled_trigger_initialized;
static atomic_t lcd_power_state;

static char sony_orise_001[] ={0x00, 0x00}; 
static char sony_orise_002[] = {
        0xFF, 0x96, 0x01, 0x01}; 
static char sony_orise_003[] ={0x00, 0x80}; 
static char sony_orise_004[] = {
        0xFF, 0x96, 0x01};
static char sony_inv_01[] = {0x00, 0xB3};
static char sony_inv_02[] = {0xC0, 0x50};
static char sony_timing1_01[] = {0x00, 0x80};
static char sony_timing1_02[] = {0xF3, 0x04};
static char sony_timing2_01[] = {0x00, 0xC0};
static char sony_timing2_02[] = {0xC2, 0xB0};
static char sony_pwrctl2_01[] = {0x00, 0xA0};
static char sony_pwrctl2_02[] = {
	0xC5, 0x04, 0x3A, 0x56,
	0x44, 0x44, 0x44, 0x44};
static char sony_pwrctl3_01[] = {0x00, 0xB0};
static char sony_pwrctl3_02[] = {
	0xC5, 0x04, 0x3A, 0x56,
	0x44, 0x44, 0x44, 0x44};

static char sony_gamma28_00[] ={0x00, 0x00}; 
static char sony_gamma28_01[] = {
	0xe1, 0x07, 0x10, 0x16,
	0x0F, 0x08, 0x0F, 0x0D,
	0x0C, 0x02, 0x06, 0x0F,
	0x0B, 0x11, 0x0D, 0x07,
	0x00
}; 

static char sony_gamma28_02[] ={0x00, 0x00}; 
static char sony_gamma28_03[] = {
	0xe2, 0x07, 0x10, 0x16,
	0x0F, 0x08, 0x0F, 0x0D,
	0x0C, 0x02, 0x06, 0x0F,
	0x0B, 0x11, 0x0D, 0x07,
	0x00
}; 

static char sony_gamma28_04[] ={0x00, 0x00}; 
static unsigned char sony_gamma28_05[] = {
	0xe3, 0x19, 0x1D, 0x20,
	0x0C, 0x04, 0x0B, 0x0B,
	0x0A, 0x03, 0x07, 0x12,
	0x0B, 0x11, 0x0D, 0x07,
	0x00
}; 

static char sony_gamma28_06[] ={0x00, 0x00}; 
static char sony_gamma28_07[] = {
	0xe4, 0x19, 0x1D, 0x20,
	0x0C, 0x04, 0x0B, 0x0B,
	0x0A, 0x03, 0x07, 0x12,
	0x0B, 0x11, 0x0D, 0x07,
	0x00
}; 

static char sony_gamma28_08[] ={0x00, 0x00}; 
static char sony_gamma28_09[] = {
	0xe5, 0x07, 0x0F, 0x15,
	0x0D, 0x06, 0x0E, 0x0D,
	0x0C, 0x02, 0x06, 0x0F,
	0x09, 0x0D, 0x0D, 0x06,
	0x00
}; 

static char sony_gamma28_10[] ={0x00, 0x00}; 
static char sony_gamma28_11[] = {
	0xe6, 0x07, 0x0F, 0x15,
	0x0D, 0x06, 0x0E, 0x0D,
	0x0C, 0x02, 0x06, 0x0F,
	0x09, 0x0D, 0x0D, 0x06,
	0x00
}; 

static char pwm_freq_sel_cmds1[] = {0x00, 0xB4}; 
static char pwm_freq_sel_cmds2[] = {0xC6, 0x00}; 

static char pwm_dbf_cmds1[] = {0x00, 0xB1}; 
static char pwm_dbf_cmds2[] = {0xC6, 0x04}; 
#if 0
static char orise_panel_Set_TE_Line[] = {
        0x44, 0x01, 0x68, 0xFF}; 
static char orise_panel_TE_Enable[] = {0x35, 0x00}; 

static char max_pktsize[] = {0x04, 0x00}; 

static char no_video_mode1[] = {0x00, 0x93}; 
static char no_video_mode2[] = {0xB0, 0xB7}; 
static char no_wait_te1[] = {0x00, 0xA0}; 
static char no_wait_te2[] = {0xC1, 0x00}; 
#endif
static char dsi_orise_pwm2[] = {0x53, 0x24};
static char dsi_orise_pwm3[] = {0x55, 0x00};

static char enter_sleep[2] = {0x10, 0x00}; 
static char exit_sleep[2] = {0x11, 0x00}; 
static char display_off[2] = {0x28, 0x00}; 
static char display_on[2] = {0x29, 0x00}; 

static char led_pwm1[] = {0x51, 0x00}; 

static struct dsi_cmd_desc orise_video_on_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(exit_sleep), exit_sleep},
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(display_on), display_on},
};

static struct dsi_cmd_desc orise_cmd_on_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_orise_001), sony_orise_001},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(sony_orise_002), sony_orise_002},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_orise_003), sony_orise_003},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(sony_orise_004), sony_orise_004},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_inv_01), sony_inv_01},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_inv_02), sony_inv_02},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_timing1_01), sony_timing1_01},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_timing1_02), sony_timing1_02},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_timing2_01), sony_timing2_01},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_timing2_02), sony_timing2_02},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_pwrctl2_01), sony_pwrctl2_01},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(sony_pwrctl2_02), sony_pwrctl2_02},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_pwrctl3_01), sony_pwrctl3_01},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(sony_pwrctl3_02), sony_pwrctl3_02},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_orise_001), sony_orise_001},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_gamma28_00), sony_gamma28_00},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(sony_gamma28_01), sony_gamma28_01},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_gamma28_02), sony_gamma28_02},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(sony_gamma28_03), sony_gamma28_03},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_gamma28_04), sony_gamma28_04},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(sony_gamma28_05), sony_gamma28_05},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_gamma28_06), sony_gamma28_06},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(sony_gamma28_07), sony_gamma28_07},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_gamma28_08), sony_gamma28_08},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(sony_gamma28_09), sony_gamma28_09},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_gamma28_10), sony_gamma28_10},
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(sony_gamma28_11), sony_gamma28_11},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(sony_orise_001), sony_orise_001},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(pwm_freq_sel_cmds1), pwm_freq_sel_cmds1},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(pwm_freq_sel_cmds2), pwm_freq_sel_cmds2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(pwm_dbf_cmds1), pwm_dbf_cmds1},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(pwm_dbf_cmds2), pwm_dbf_cmds2},
#if 0
	{DTYPE_DCS_LWRITE, 1, 0, 0, 0, sizeof(orise_panel_Set_TE_Line), orise_panel_Set_TE_Line},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(orise_panel_TE_Enable), orise_panel_TE_Enable},
	{DTYPE_MAX_PKTSIZE, 1, 0, 0, 0, sizeof(max_pktsize), max_pktsize},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(no_video_mode1), no_video_mode1},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(no_video_mode2), no_video_mode2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(no_wait_te1), no_wait_te1},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(no_wait_te2), no_wait_te2},
#endif
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(dsi_orise_pwm2), dsi_orise_pwm2},
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(dsi_orise_pwm3), dsi_orise_pwm3},
	{DTYPE_DCS_WRITE,  1, 0, 0, 120, sizeof(exit_sleep), exit_sleep},
};

static struct dsi_cmd_desc orise_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 10,
		sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120,
		sizeof(enter_sleep), enter_sleep}
};
static struct dsi_cmd_desc orise_display_on_cmds[] = {  
	{DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(display_on), display_on},
};


static struct dsi_cmd_desc orise_cmd_backlight_cmds[] = {
	{DTYPE_DCS_WRITE1, 1, 0, 0, 0,
		sizeof(led_pwm1), led_pwm1},
};

static int mipi_orise_lcd_on(struct platform_device *pdev)
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
		cmdreq_orise.cmds = orise_video_on_cmds;
		cmdreq_orise.cmds_cnt = ARRAY_SIZE(orise_video_on_cmds);
	} else {
		cmdreq_orise.cmds = orise_cmd_on_cmds;
		cmdreq_orise.cmds_cnt = ARRAY_SIZE(orise_cmd_on_cmds);
	}

	cmdreq_orise.flags = CMD_REQ_COMMIT;
	cmdreq_orise.rlen = 0;
	cmdreq_orise.cb = NULL;
	mipi_dsi_cmdlist_put(&cmdreq_orise);

	atomic_set(&lcd_power_state, 1);

	return 0;
}

static int mipi_orise_lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	mfd = platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;

	cmdreq_orise.cmds = orise_display_off_cmds;
	cmdreq_orise.cmds_cnt = ARRAY_SIZE(orise_display_off_cmds);
	cmdreq_orise.flags = CMD_REQ_COMMIT;
	cmdreq_orise.rlen = 0;
	cmdreq_orise.cb = NULL;
	mipi_dsi_cmdlist_put(&cmdreq_orise);

	atomic_set(&lcd_power_state, 0);

	return 0;
}

DEFINE_LED_TRIGGER(bkl_led_trigger);

static void mipi_orise_set_backlight(struct msm_fb_data_type *mfd)
{
	struct mipi_panel_info *mipi;
	if (mipi_orise_pdata && mipi_orise_pdata->shrink_pwm)
		led_pwm1[1] = mipi_orise_pdata->shrink_pwm(mfd->bl_level);
	else
		led_pwm1[1] = (unsigned char)(mfd->bl_level);

	if (mipi_orise_pdata && (mipi_orise_pdata->enable_wled_bl_ctrl)
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

	cmdreq_orise.cmds = orise_cmd_backlight_cmds;
	cmdreq_orise.cmds_cnt = ARRAY_SIZE(orise_cmd_backlight_cmds);
	cmdreq_orise.flags = CMD_REQ_COMMIT;
	cmdreq_orise.rlen = 0;
	cmdreq_orise.cb = NULL;
	mipi_dsi_cmdlist_put(&cmdreq_orise);

#ifdef CONFIG_BACKLIGHT_WLED_CABC
	
	if (wled_trigger_initialized) {
		led_trigger_event(bkl_led_trigger, mfd->bl_level);
	}
#endif
	return;
}

static void mipi_orise_display_on(struct msm_fb_data_type *mfd)
{

    
    PR_DISP_DEBUG("%s\n",  __FUNCTION__);

	mipi_dsi_op_mode_config(DSI_CMD_MODE);

	cmdreq_orise.cmds = orise_display_on_cmds;
	cmdreq_orise.cmds_cnt = ARRAY_SIZE(orise_display_on_cmds);
	cmdreq_orise.flags = CMD_REQ_COMMIT;
	cmdreq_orise.rlen = 0;
	cmdreq_orise.cb = NULL;
	mipi_dsi_cmdlist_put(&cmdreq_orise);
}



static int __devinit mipi_orise_lcd_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct mipi_panel_info *mipi;
	struct platform_device *current_pdev;
	static struct mipi_dsi_phy_ctrl *phy_settings;
	static char dlane_swap;

	if (pdev->id == 0) {
		mipi_orise_pdata = pdev->dev.platform_data;

		if (mipi_orise_pdata
			&& mipi_orise_pdata->phy_ctrl_settings) {
			phy_settings = (mipi_orise_pdata->phy_ctrl_settings);
		}

		if (mipi_orise_pdata
			&& mipi_orise_pdata->dlane_swap) {
			dlane_swap = (mipi_orise_pdata->dlane_swap);
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
	.probe  = mipi_orise_lcd_probe,
	.driver = {
		.name   = "mipi_orise",
	},
};

static struct msm_fb_panel_data orise_panel_data = {
	.on		= mipi_orise_lcd_on,
	.off		= mipi_orise_lcd_off,
	.set_backlight  = mipi_orise_set_backlight,
	.display_on = mipi_orise_display_on,
};

static int ch_used[3];

int mipi_orise_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	ret = mipi_orise_lcd_init();
	if (ret) {
		pr_err("mipi_orise_lcd_init() failed with ret %u\n", ret);
		return ret;
	}

	pdev = platform_device_alloc("mipi_orise", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	orise_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &orise_panel_data,
		sizeof(orise_panel_data));
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

static int mipi_orise_lcd_init(void)
{
	led_trigger_register_simple("bkl_trigger", &bkl_led_trigger);
	pr_info("%s: SUCCESS (WLED TRIGGER)\n", __func__);
	wled_trigger_initialized = 1;
	atomic_set(&lcd_power_state, 1);

	mipi_dsi_buf_alloc(&orise_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&orise_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

