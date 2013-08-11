/* linux/arch/arm/mach-msm/board-villec2-audio.c
 *
 * Copyright (C) 2010-2011 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/android_pmem.h>
#include <linux/mfd/pmic8058.h>
#include <linux/mfd/marimba.h>
#include <linux/delay.h>
#include <linux/pmic8058-othc.h>
#include <linux/regulator/consumer.h>

#include <mach/gpio.h>
#include <mach/dal.h>
#include "qdsp6v2/snddev_icodec.h"
#include "qdsp6v2/snddev_ecodec.h"
#include "qdsp6v2/snddev_hdmi.h"
#include <mach/qdsp6v2/audio_dev_ctl.h>
#include <sound/apr_audio.h>
#include <sound/q6asm.h>
#include <mach/htc_acoustic_8x60.h>
#include <mach/board_htc.h>

#include "board-villec2.h"

#define PM8058_GPIO_BASE					NR_MSM_GPIOS
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)		(pm_gpio + PM8058_GPIO_BASE)

static struct mutex mic_lock;
static atomic_t q6_effect_mode = ATOMIC_INIT(-1);

static uint32_t msm_snddev_gpio[] = {
	GPIO_CFG(108, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	GPIO_CFG(109, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
	GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 
};


void villec2_mic_bias_on(int en)
{
#if 0
	int rc;
	pr_aud_info("%s\n", __func__);

	if (en) {
		snddev_reg_l11 = regulator_get(NULL, "8058_l11");
		if (IS_ERR(snddev_reg_l11)) {
			pr_aud_err("%s: regulator_get(%s) failed (%ld)\n",
				__func__, "8058_l11", PTR_ERR(snddev_reg_l11));
			return;
		}

		rc = regulator_set_voltage(snddev_reg_l11, 2850000, 2850000);
		if (rc < 0)
			pr_aud_err("%s: regulator_set_voltage(8058_l11) failed (%d)\n",
				__func__, rc);

		rc = regulator_enable(snddev_reg_l11);
		if (rc < 0)
			pr_aud_err("%s: regulator_enable(8058_l11) failed (%d)\n",
				__func__, rc);
	} else {

		if (!snddev_reg_l11)
			return;

		rc = regulator_disable(snddev_reg_l11);
		if (rc < 0)
			pr_aud_err("%s: regulator_disable(8058_l11) failed (%d)\n",
					__func__, rc);
		regulator_put(snddev_reg_l11);

		snddev_reg_l11 = NULL;
	}
#endif
}

void villec2_snddev_poweramp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		
		gpio_direction_output(PM8058_GPIO_PM_TO_SYS(VILLEC2_AUD_SPK_ENO), 1);
	} else {
		
		gpio_direction_output(PM8058_GPIO_PM_TO_SYS(VILLEC2_AUD_SPK_ENO), 0);
	}
}

void villec2_snddev_usb_headset_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
	} else {
	}
}


void villec2_snddev_hsed_pamp_on(int en)
{
#if 0
	int rc;
	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		snddev_reg_ncp = regulator_get(NULL, "8058_ncp");
		if (IS_ERR(snddev_reg_ncp)) {
			pr_aud_err("%s: regulator_get(%s) failed (%ld)\n", __func__,
				"ncp", PTR_ERR(snddev_reg_ncp));
			return;
		}

		rc = regulator_set_voltage(snddev_reg_ncp, 1800000, 1800000);
		if (rc < 0)
			pr_aud_err("%s: regulator_set_voltage(ncp) failed (%d)\n",
				__func__, rc);

		rc = regulator_enable(snddev_reg_ncp);
		if (rc < 0)
			pr_aud_err("%s: regulator_enable(ncp) failed (%d)\n",
				__func__, rc);
	} else {
		if (!snddev_reg_ncp)
			return;

		rc = regulator_disable(snddev_reg_ncp);
		if (rc < 0)
			pr_aud_err("%s: regulator_disable(ncp) failed (%d)\n",
					__func__, rc);
		regulator_put(snddev_reg_ncp);

		snddev_reg_ncp = NULL;
	}
#endif
}

void villec2_snddev_hs_spk_pamp_on(int en)
{
	villec2_snddev_poweramp_on(en);
	villec2_snddev_hsed_pamp_on(en);
}

void villec2_snddev_receiver_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);
	if (en) {
		
		gpio_direction_output(PM8058_GPIO_PM_TO_SYS(VILLEC2_AUD_HANDSET_ENO), 1);
	} else {
		
		gpio_direction_output(PM8058_GPIO_PM_TO_SYS(VILLEC2_AUD_HANDSET_ENO), 0);
	}
}

void villec2_mic_enable(int en, int shift)
{
	pr_aud_info("%s: %d, shift %d\n", __func__, en, shift);

	mutex_lock(&mic_lock);

	if (en)
		villec2_mic_bias_on(en);
	else
		villec2_mic_bias_on(en);

	mutex_unlock(&mic_lock);
}

void villec2_snddev_imic_pamp_on(int en)
{
	int ret;

	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_0, OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", __func__);

	} else {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_0, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Enabling int mic power failed\n", __func__);

	}
}

void villec2_snddev_bmic_pamp_on(int en)
{
	int ret;

	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_ALWAYS_ON);
		if (ret)
			pr_aud_err("%s: Enabling back mic power failed\n", __func__);
	} else {
		ret = pm8058_micbias_enable(OTHC_MICBIAS_1, OTHC_SIGNAL_OFF);
		if (ret)
			pr_aud_err("%s: Enabling back mic power failed\n", __func__);
	}
}

void villec2_snddev_stereo_mic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);

	if (en) {
		villec2_snddev_imic_pamp_on(en);
		villec2_snddev_bmic_pamp_on(en);
	} else {
		villec2_snddev_imic_pamp_on(en);
		villec2_snddev_bmic_pamp_on(en);
	}
}

void villec2_snddev_emic_pamp_on(int en)
{
	pr_aud_info("%s %d\n", __func__, en);

#if 0 
	if (en) {
		
		gpio_request(PM8058_GPIO_PM_TO_SYS(VILLEC2_AUD_2V85_EN), "AUD_2V85_EN");
		gpio_direction_output(PM8058_GPIO_PM_TO_SYS(VILLEC2_AUD_2V85_EN), 1);
	} else {
		gpio_request(PM8058_GPIO_PM_TO_SYS(VILLEC2_AUD_2V85_EN), "AUD_2V85_EN");
		gpio_direction_output(PM8058_GPIO_PM_TO_SYS(VILLEC2_AUD_2V85_EN), 0);
	}
#endif
}

void villec2_snddev_fmspk_pamp_on(int en)
{
	villec2_snddev_poweramp_on(en);
}

void villec2_snddev_fmhs_pamp_on(int en)
{
	villec2_snddev_hsed_pamp_on(en);
}

void villec2_voltage_on (int en)
{
}

int villec2_get_rx_vol(uint8_t hw, int network, int level)
{
	int vol = 0;

	

	pr_aud_info("%s(%d, %d, %d) => %d\n", __func__, hw, network, level, vol);

	return vol;
}

int villec2_get_speaker_channels(void)
{
	
	return 1;
}

int villec2_support_beats(void)
{
	
	return 1;
}

#if 0
void villec2_enable_beats(int en)
{
	pr_aud_info("%s: %d\n", __func__, en);
	if (en)
		adie_codec_set_device_analog_volume(NULL, 2, 0x04);
	else
		adie_codec_set_device_analog_volume(NULL, 2, 0x14);
}
#endif

int villec2_is_msm_i2s_slave(void)
{
	
	return 0;
}

int villec2_support_adie(void)
{
	return 1;
}

int villec2_support_back_mic(void)
{
	return 1;
}

int villec2_is_msm_i2s_master(void)
{
	
	return 1;
}

void villec2_enable_beats(int en)
{
	pr_aud_info("%s: %d\n", __func__, en);
	if (en)
		adie_codec_set_device_analog_volume(NULL, 2, 0x04);
	else
		adie_codec_set_device_analog_volume(NULL, 2, 0x14);
}

void villec2_set_q6_effect_mode(int mode)
{
	pr_aud_info("%s: mode %d\n", __func__, mode);
	atomic_set(&q6_effect_mode, mode);
}

int villec2_get_q6_effect_mode(void)
{
	int mode = atomic_read(&q6_effect_mode);
	pr_aud_info("%s: mode %d\n", __func__, mode);
	return mode;
}

static struct q6v2audio_analog_ops ops = {
	.speaker_enable	        = villec2_snddev_poweramp_on,
	.headset_enable	        = villec2_snddev_hsed_pamp_on,
	.handset_enable	        = villec2_snddev_receiver_pamp_on,
	.headset_speaker_enable	= villec2_snddev_hs_spk_pamp_on,
	.int_mic_enable         = villec2_snddev_imic_pamp_on,
	.back_mic_enable        = villec2_snddev_bmic_pamp_on,
	.ext_mic_enable         = villec2_snddev_emic_pamp_on,
	.fm_headset_enable      = villec2_snddev_fmhs_pamp_on,
	.fm_speaker_enable      = villec2_snddev_fmspk_pamp_on,
	.stereo_mic_enable      = villec2_snddev_stereo_mic_pamp_on,
	.usb_headset_enable     = villec2_snddev_usb_headset_pamp_on,
	.voltage_on             = villec2_voltage_on,
};

static struct q6v2audio_icodec_ops iops = {
	.is_msm_i2s_slave = villec2_is_msm_i2s_slave,
	.support_adie = villec2_support_adie,
};

static struct acoustic_ops acoustic = {
	.enable_mic_bias = villec2_mic_enable,
	.support_adie = villec2_support_adie,
	.support_back_mic = villec2_support_back_mic,
	.get_speaker_channels = villec2_get_speaker_channels,
	.support_beats = villec2_support_beats,
	.enable_beats = villec2_enable_beats,
	.set_q6_effect = villec2_set_q6_effect_mode,
};

static struct q6asm_ops qops = {
	.get_q6_effect = villec2_get_q6_effect_mode,
};

void villec2_audio_gpios_init(void)
{
	pr_aud_info("%s\n", __func__);
	gpio_request(PM8058_GPIO_PM_TO_SYS(VILLEC2_AUD_SPK_ENO), "AUD_SPK_ENO");
	gpio_request(PM8058_GPIO_PM_TO_SYS(VILLEC2_AUD_HANDSET_ENO), "AUD_HANDSET_ENO");
}

void __init villec2_audio_init(void)
{
	int i = 0;
	mutex_init(&mic_lock);

	pr_aud_info("%s\n", __func__);
	htc_8x60_register_analog_ops(&ops);
	htc_8x60_register_icodec_ops(&iops);
	htc_register_q6asm_ops(&qops);
	acoustic_register_ops(&acoustic);

	
	for (i = 0 ; i < ARRAY_SIZE(msm_snddev_gpio); i++)
		gpio_tlmm_config(msm_snddev_gpio[i], GPIO_CFG_DISABLE);

	villec2_audio_gpios_init();
}
