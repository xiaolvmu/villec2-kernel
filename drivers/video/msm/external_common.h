/* Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
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
#ifndef __EXTERNAL_COMMON_H__
#define __EXTERNAL_COMMON_H__
#include <linux/switch.h>
#include <video/msm_hdmi_modes.h>

#ifdef DEBUG
#ifndef DEV_DBG_PREFIX
#define DEV_DBG_PREFIX "EXT_INTERFACE: "
#endif
#define DEV_DBG(args...)	pr_debug(DEV_DBG_PREFIX args)
#else
#define DEV_DBG(args...)	(void)0
#endif /* DEBUG */
#define DEV_INFO(args...)	dev_info(external_common_state->dev, args)
#define DEV_WARN(args...)	dev_warn(external_common_state->dev, args)
#define DEV_ERR(args...)	dev_err(external_common_state->dev, args)

/* A lookup table for all the supported display modes by the HDMI
 * hardware and driver.  Use HDMI_SETUP_LUT in the module init to
 * setup the LUT with the supported modes. */
extern struct msm_hdmi_mode_timing_info
	hdmi_common_supported_video_mode_lut[HDMI_VFRMT_MAX];

/* Structure that encapsulates all the supported display modes by the HDMI sink
 * device */
struct hdmi_disp_mode_list_type {
	uint32	disp_mode_list[HDMI_VFRMT_MAX];
#define TOP_AND_BOTTOM		0x10
#define FRAME_PACKING		0x20
#define SIDE_BY_SIDE_HALF	0x40
	uint32	disp_3d_mode_list[HDMI_VFRMT_MAX];
	uint32	disp_multi_3d_mode_list[16];
	uint32	disp_multi_3d_mode_list_cnt;
	uint32	num_of_elements;
};

struct external_common_state_type {
	boolean hpd_state;
	struct kobject *uevent_kobj;
	uint32 video_resolution;
	bool vcdb_support;
	struct device *dev;
	struct switch_dev sdev;
#ifdef CONFIG_FB_MSM_HDMI_3D
	boolean format_3d;
	void (*switch_3d)(boolean on);
#endif
#ifdef CONFIG_FB_MSM_HDMI_COMMON
	boolean hdcp_active;
	boolean hpd_feature_on;
	boolean hdmi_sink;
	struct hdmi_disp_mode_list_type disp_mode_list;
	uint8 speaker_allocation_block;
	uint16 video_latency, audio_latency;
	uint8 audio_data_block_cnt;
	uint16 physical_address;
	uint32 preferred_video_format;
	uint8 pt_scan_info;
	uint8 it_scan_info;
	uint8 ce_scan_info;
	uint8 spd_vendor_name[9];
	uint8 spd_product_description[17];
	boolean present_3d;
	boolean present_hdcp;
	uint32 audio_data_blocks[16];
	int (*read_edid_block)(int block, uint8 *edid_buf);
	int (*hpd_feature)(int on);
#endif
};

/* The external interface driver needs to initialize the common state. */
extern struct external_common_state_type *external_common_state;
extern struct mutex external_common_state_hpd_mutex;
extern struct mutex hdmi_msm_state_mutex;

#ifdef CONFIG_FB_MSM_HDMI_COMMON
int hdmi_common_read_edid(void);
const char *video_format_2string(uint32 format);
bool hdmi_common_get_video_format_from_drv_data(struct msm_fb_data_type *mfd);
const struct msm_hdmi_mode_timing_info *hdmi_common_get_mode(uint32 mode);
const struct msm_hdmi_mode_timing_info *hdmi_common_get_supported_mode(
	uint32 mode);
const struct msm_hdmi_mode_timing_info *hdmi_mhl_get_mode(uint32 mode);
const struct msm_hdmi_mode_timing_info *hdmi_mhl_get_supported_mode(
	uint32 mode);
void hdmi_common_init_panel_info(struct msm_panel_info *pinfo);

ssize_t video_3d_format_2string(uint32 format, char *buf, int size);
#endif

int external_common_state_create(struct platform_device *pdev);
void external_common_state_remove(void);

#endif /* __EXTERNAL_COMMON_H__ */
