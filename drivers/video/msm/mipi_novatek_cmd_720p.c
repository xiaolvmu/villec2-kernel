/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
#include "mipi_novatek.h"

static struct msm_panel_info pinfo;

static struct mipi_dsi_phy_ctrl mipi_dsi_sony_panel_id28103_phy_ctrl_720p = {
	
	
	{0x03, 0x08, 0x05, 0x00, 0x20},
	
	{0x9B, 0x38, 0x18, 0x00, 0x4B, 0x51, 0x1C,
	0x3B, 0x29, 0x03, 0x04, 0xA0},
	
	{0x5F, 0x00, 0x00, 0x10},
	
	{0xFF, 0x00, 0x06, 0x00},
	
	{0x0, 0x38, 0x32, 0xDA, 0x00, 0x10, 0x0F, 0x61,
	0x41, 0x0F, 0x01,
	0x00, 0x1A, 0x00, 0x00, 0x02, 0x00, 0x20, 0x00, 0x02 },
};

static int __init mipi_cmd_novatek_sony_hd720p_init(void)
{
	int ret;

	if (msm_fb_detect_client("mipi_cmd_novatek_sony_720p"))
		return 0;

	pinfo.xres = 720;
	pinfo.yres = 1280;
	pinfo.type = MIPI_CMD_PANEL;
	pinfo.pdest = DISPLAY_1;
	pinfo.wait_cycle = 0;
	pinfo.bpp = 24;
	pinfo.lcdc.h_back_porch = 104;
	pinfo.lcdc.h_front_porch = 95;
	pinfo.lcdc.h_pulse_width = 1;
	pinfo.lcdc.v_back_porch = 2;
	pinfo.lcdc.v_front_porch = 6;
	pinfo.lcdc.v_pulse_width = 1;
	pinfo.lcdc.border_clr = 0;	
	pinfo.lcdc.underflow_clr = 0xff;	
	pinfo.lcdc.hsync_skew = 0;
	pinfo.bl_max = 255;
	pinfo.bl_min = 1;
	pinfo.fb_num = 2;
	pinfo.clk_rate = 454000000;
	pinfo.is_3d_panel = FB_TYPE_3D_PANEL;
	pinfo.lcd.vsync_enable = TRUE;
	pinfo.lcd.hw_vsync_mode = TRUE;
	pinfo.lcd.refx100 = 6000; 
	pinfo.lcd.v_back_porch = 2;
	pinfo.lcd.v_front_porch = 6;
	pinfo.lcd.v_pulse_width = 1;

	pinfo.mipi.mode = DSI_CMD_MODE;
	pinfo.mipi.dst_format = DSI_CMD_DST_FORMAT_RGB888;
	pinfo.mipi.vc = 0;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.esc_byte_ratio = 4;
	pinfo.mipi.data_lane0 = TRUE;
	pinfo.mipi.data_lane1 = TRUE;
	pinfo.mipi.data_lane2 = TRUE;
	pinfo.mipi.t_clk_post = 0x10;
	pinfo.mipi.t_clk_pre = 0x21;
	pinfo.mipi.stream = 0;	
	pinfo.mipi.mdp_trigger = DSI_CMD_TRIGGER_NONE;
	pinfo.mipi.dma_trigger = DSI_CMD_TRIGGER_SW;
	pinfo.mipi.te_sel = 1; 
	pinfo.mipi.interleave_max = 1;
	pinfo.mipi.insert_dcs_cmd = TRUE;
	pinfo.mipi.wr_mem_continue = 0x3c;
	pinfo.mipi.wr_mem_start = 0x2c;
	pinfo.mipi.dsi_phy_db = &mipi_dsi_sony_panel_id28103_phy_ctrl_720p;

	ret = mipi_novatek_device_register(&pinfo, MIPI_DSI_PRIM,
						MIPI_DSI_PANEL_720P_PT);
	if (ret)
		pr_err("%s: failed to register device!\n", __func__);

	return ret;
}

module_init(mipi_cmd_novatek_sony_hd720p_init);
