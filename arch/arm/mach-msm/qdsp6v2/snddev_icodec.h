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
#ifndef __MACH_QDSP6V2_SNDDEV_ICODEC_H
#define __MACH_QDSP6V2_SNDDEV_ICODEC_H
#include <linux/mfd/msm-adie-codec.h>
#include <mach/qdsp5v2/audio_def.h>
#include <mach/pmic.h>

struct snddev_icodec_data {
	u32 capability; 
	const char *name;
	u32 copp_id; 
	
	struct adie_codec_dev_profile *profile;
	
	u8 channel_mode;
	u32 default_sample_rate;
	int (*pamp_on) (void);
	void (*pamp_off) (void);
	int (*voltage_on) (void);
	void (*voltage_off) (void);
	u32 dev_vol_type;
#ifdef CONFIG_MACH_VILLEC2
	u32 aic3254_id;
#endif
};

#ifdef CONFIG_MACH_VILLEC2 
struct snddev_icodec_state {
	struct snddev_icodec_data *data;
	struct adie_codec_path *adie_path;
	u32 sample_rate;
	u32 enabled;
};

struct q6v2audio_analog_ops {
	void (*speaker_enable)(int en);
	void (*headset_enable)(int en);
	void (*handset_enable)(int en);
	void (*headset_speaker_enable)(int en);
	void (*int_mic_enable)(int en);
	void (*back_mic_enable)(int en);
	void (*ext_mic_enable)(int en);
	void (*stereo_mic_enable)(int en);
	void (*usb_headset_enable)(int en);
	void (*fm_headset_enable)(int en);
	void (*fm_speaker_enable)(int en);
	void (*voltage_on) (int on);
};

struct q6v2audio_icodec_ops {
	int (*support_adie) (void);
	int (*is_msm_i2s_slave) (void);
};

struct aic3254_info {
    u32 dev_id;
    u32 path_id;
};

int update_aic3254_info(struct aic3254_info *info);
void htc_8x60_register_analog_ops(struct q6v2audio_analog_ops *ops);
void htc_8x60_register_icodec_ops(struct q6v2audio_icodec_ops *ops);

#endif 

#endif
