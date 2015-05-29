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

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_orise.h"
#include <mach/debug_display.h>

static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl dsi_cmd_mode_phy_db = {
#if 0
	{0x03, 0x0a, 0x04, 0x00, 0x20},
	
	{0x8c, 0x34, 0x15, 0x00, 0x46, 0x50, 0x1a, 0x38,
	0x24, 0x03, 0x04, 0xa0},
    
	{0x5f, 0x00, 0x00, 0x10},
    
	{0xff, 0x00, 0x06, 0x00},
	
		{0x0, 0xf9, 0x30, 0xda, 0x00, 0x40, 0x03, 0x62,
	0x40, 0x07, 0x03,
	0x00, 0x1a, 0x00, 0x00, 0x02, 0x00, 0x20, 0x00, 0x01 },
#else
	
	
	{0x09, 0x08, 0x05, 0x00, 0x20},
	
	
	{0xb9, 0x2A, 0x20, 0x00, 0x24, 0x50, 0x1D, 0x2A, 0x24,
	0x03, 0x04, 0xa0},
	
	{0x5f, 0x00, 0x00, 0x10},
	
	{0xff, 0x00, 0x06, 0x00},
	
	{0x0, 0xe, 0x30, 0xda, 0x00, 0x10, 0x0f, 0x61,
	0x40, 0x07, 0x03,
	0x00, 0x1a, 0x00, 0x00, 0x02, 0x00, 0x20, 0x00, 0x02},
#endif
};

static int __init mipi_cmd_orise_qhd_pt_init(void)
{
	int ret;

	if (msm_fb_detect_client("mipi_cmd_orise_qhd"))
		return 0;
	PR_DISP_INFO("panel: mipi_cmd_orise_qhd\n");

	pinfo.xres = 540;
	pinfo.yres = 960;
	pinfo.type = MIPI_CMD_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.width = 56;
	pinfo.height = 99;

	pinfo.lcdc.h_back_porch = 22;
	pinfo.lcdc.h_front_porch = 22;
	pinfo.lcdc.h_pulse_width = 1;
	pinfo.lcdc.v_back_porch = 3;
	pinfo.lcdc.v_front_porch = 3;
	pinfo.lcdc.v_pulse_width = 1;
	pinfo.lcdc.border_clr = 0;	
	pinfo.lcdc.underflow_clr = 0xff;	
	pinfo.lcdc.hsync_skew = 0;

	pinfo.bl_max = 255;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	pinfo.clk_rate = 482000000;

	pinfo.lcd.vsync_enable = TRUE;
	pinfo.lcd.hw_vsync_mode = TRUE;
	pinfo.lcd.refx100 = 6096; 

	pinfo.lcd.v_back_porch = 2;
	pinfo.lcd.v_front_porch = 2;
	pinfo.lcd.v_pulse_width = 2;

	pinfo.mipi.mode = DSI_CMD_MODE;
	pinfo.mipi.dst_format = DSI_CMD_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.esc_byte_ratio = 4;

	pinfo.mipi.t_clk_post = 0x0a;
	pinfo.mipi.t_clk_pre = 0x21;
	pinfo.mipi.stream = 0;	
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.te_sel = 1; 
	pinfo.mipi.interleave_max = 1;
	pinfo.mipi.insert_dcs_cmd = TRUE;
	pinfo.mipi.wr_mem_continue = 0x3c;
	pinfo.mipi.wr_mem_start = 0x2c;
	pinfo.mipi.tx_eot_append = 1; 
	pinfo.mipi.dsi_phy_db = &dsi_cmd_mode_phy_db;

	ret = mipi_orise_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_QHD_PT);

	if (ret)
		PR_DISP_ERR("%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_cmd_orise_qhd_pt_init);
