/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2013 Sebastian Sobczyk <sebastiansobczyk@wp.pl>
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

#include "msm_actuator.h"
#include "msm_camera_i2c.h"
#include <mach/gpio.h>

#define	S5K3H1GX_TOTAL_STEPS_NEAR_TO_FAR 42
#define REG_VCM_I2C_ADDR 0x18

DEFINE_MUTEX(s5k3h1gx_act_mutex);
static struct msm_actuator_ctrl_t s5k3h1gx_act_t;

static struct msm_actuator_info *s5k3h1gx_msm_actuator_info;

static int32_t s5k3h1gx_poweron_af(void)
{
	int32_t rc = 0;
	pr_info("%s enable AF actuator, gpio = %d\n", __func__,
			s5k3h1gx_msm_actuator_info->vcm_pwd);
	mdelay(1);
	rc = gpio_request(s5k3h1gx_msm_actuator_info->vcm_pwd, "s5k3h1gx");
	if (!rc)
		gpio_direction_output(s5k3h1gx_msm_actuator_info->vcm_pwd, 1);
	else
		pr_err("%s: AF PowerON gpio_request failed %d\n", __func__, rc);
	gpio_free(s5k3h1gx_msm_actuator_info->vcm_pwd);
	mdelay(1);
	return rc;
}

static void s5k3h1gx_poweroff_af(void)
{
	int32_t rc = 0;

	pr_info("%s disable AF actuator, gpio = %d\n", __func__,
			s5k3h1gx_msm_actuator_info->vcm_pwd);

	msleep(1);
	rc = gpio_request(s5k3h1gx_msm_actuator_info->vcm_pwd, "s5k3h1gx");
	if (!rc)
		gpio_direction_output(s5k3h1gx_msm_actuator_info->vcm_pwd, 0);
	else
		pr_err("%s: AF PowerOFF gpio_request failed %d\n", __func__, rc);
	gpio_free(s5k3h1gx_msm_actuator_info->vcm_pwd);
	msleep(1);
}

int32_t s5k3h1gx_msm_actuator_init_table(
	struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;

	LINFO("%s called\n", __func__);

	if (a_ctrl->func_tbl.actuator_set_params)
		a_ctrl->func_tbl.actuator_set_params(a_ctrl);

	a_ctrl->set_info.total_steps = S5K3H1GX_TOTAL_STEPS_NEAR_TO_FAR;

	if (a_ctrl->step_position_table != NULL) {
		kfree(a_ctrl->step_position_table);
		a_ctrl->step_position_table = NULL;
	}
	a_ctrl->step_position_table =
		kmalloc(sizeof(uint16_t) * (a_ctrl->set_info.total_steps + 1),
			GFP_KERNEL);

	if (a_ctrl->step_position_table != NULL) {
		uint16_t i = 0;
		uint16_t s5k3h1gx_nl_region_boundary1 = 3;
		uint16_t s5k3h1gx_nl_region_boundary2 = 5;
		uint16_t s5k3h1gx_nl_region_code_per_step1 = 40;
		uint16_t s5k3h1gx_nl_region_code_per_step2 = 20;
		uint16_t s5k3h1gx_l_region_code_per_step = 16;
		uint16_t s5k3h1gx_max_value = 1023;

		a_ctrl->step_position_table[0] = a_ctrl->initial_code;
		for (i = 1; i <= a_ctrl->set_info.total_steps; i++) {
			if (i <= s5k3h1gx_nl_region_boundary1) {
				a_ctrl->step_position_table[i] =
					a_ctrl->step_position_table[i-1]
					+ s5k3h1gx_nl_region_code_per_step1;
			} else if (i <= s5k3h1gx_nl_region_boundary2) {
				a_ctrl->step_position_table[i] =
					a_ctrl->step_position_table[i-1]
					+ s5k3h1gx_nl_region_code_per_step2;
			} else {
				a_ctrl->step_position_table[i] =
					a_ctrl->step_position_table[i-1]
					+ s5k3h1gx_l_region_code_per_step;
			}

			if (a_ctrl->step_position_table[i] > s5k3h1gx_max_value)
				a_ctrl->step_position_table[i] = s5k3h1gx_max_value;
		}
		a_ctrl->curr_step_pos = 0;
		a_ctrl->curr_region_index = 0;
	} else {
		pr_err("%s table init failed\n", __func__);
		rc = -EFAULT;
	}

	return rc;
}

int32_t s5k3h1gx_msm_actuator_move_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	int dir,
	int32_t num_steps)
{
	int32_t rc = 0;
	int8_t sign_dir = 0;
	int16_t dest_step_pos = 0;

	int16_t curr_lens_pos;
	int16_t next_lens_pos;
	int16_t dest_lens_pos;
	int16_t target_dist;
	uint16_t sw_damping_time_wait;
	int32_t sw_damping_step_dynamic;
	int16_t time_wait_per_step;
	uint32_t time_wait;
	uint16_t small_step;
	uint8_t mode_mask;

	LINFO("%s called, dir %d, num_steps %d\n",
		__func__,
		dir,
		num_steps);

	if (dir == MOVE_NEAR)
		sign_dir = 1;
	else if (dir == MOVE_FAR)
		sign_dir = -1;
	else {
		pr_err("Illegal focus direction\n");
		rc = -EINVAL;
		return rc;
	}
	
	dest_step_pos = a_ctrl->curr_step_pos +
		(sign_dir * num_steps);

	if (dest_step_pos < 0)
		dest_step_pos = 0;
	else if (dest_step_pos > a_ctrl->set_info.total_steps)
		dest_step_pos = a_ctrl->set_info.total_steps;

	if (dest_step_pos == a_ctrl->curr_step_pos)
		return rc;

	curr_lens_pos = a_ctrl->step_position_table[a_ctrl->curr_step_pos];
	dest_lens_pos = a_ctrl->step_position_table[dest_step_pos];

	target_dist = sign_dir * (dest_lens_pos - curr_lens_pos);

	if (sign_dir < 0 && target_dist >= a_ctrl->step_position_table[5]) {
		sw_damping_step_dynamic = 10;
		sw_damping_time_wait = 1;
		time_wait = 1000000 / 30 - 10000;
	}
	else {
		if (num_steps > 2) {
			sw_damping_step_dynamic = 4;
			sw_damping_time_wait = 4;
		} else {
			sw_damping_step_dynamic = 2;
			sw_damping_time_wait = 2;
		}
		time_wait = 1000000 / 30;
	}
	time_wait_per_step = (int16_t)(time_wait / target_dist);

	if (time_wait_per_step >= 800)
		mode_mask = 0x5;
	else if (time_wait_per_step >= 400)
		mode_mask = 0x4;
	else if (time_wait_per_step >= 200)
		mode_mask = 0x3;
	else if (time_wait_per_step >= 100)
		mode_mask = 0x2;
	else if (time_wait_per_step >= 50)
		mode_mask = 0x1;
	else {
		if (time_wait >= 17600)
			mode_mask = 0x0D;
		else if (time_wait >= 8800)
			mode_mask = 0x0C;
		else if (time_wait >= 4400)
			mode_mask = 0x0B;
		else if (time_wait >= 2200)
			mode_mask = 0x0A;
		else
			mode_mask = 0x09;
	}

	small_step = (uint16_t)(target_dist / sw_damping_step_dynamic);
	if ((target_dist % sw_damping_step_dynamic) != 0) {
		small_step++;
	}

	for (next_lens_pos = curr_lens_pos + (sign_dir * small_step);
		(sign_dir * next_lens_pos) <= (sign_dir * dest_lens_pos);
		next_lens_pos += sign_dir * small_step) {

		rc = a_ctrl->func_tbl.actuator_i2c_write(a_ctrl, next_lens_pos, &mode_mask);
		if (rc < 0) {
			pr_err("%s: focus move failed\n", __func__);
			return rc;
		}
		mdelay(sw_damping_time_wait);
		curr_lens_pos = next_lens_pos;
	}

	if(curr_lens_pos != dest_lens_pos) {
		rc = a_ctrl->func_tbl.actuator_i2c_write(a_ctrl, dest_lens_pos, &mode_mask);
		if (rc < 0) {
			pr_err("%s: focus move failed\n", __func__);
			return rc;
		}
		mdelay(sw_damping_time_wait);
	}

	a_ctrl->curr_step_pos = dest_step_pos;
	return rc;
}

int s5k3h1gx_actuator_af_power_down(void *params)
{
	int rc = 0;
	LINFO("%s called\n", __func__);

	rc = (int) msm_actuator_af_power_down(&s5k3h1gx_act_t);
	s5k3h1gx_poweroff_af();
	return rc;
}

static int32_t s5k3h1gx_wrapper_i2c_write(struct msm_actuator_ctrl_t *a_ctrl,
	int16_t next_lens_position, void *params)
{
	int32_t rc = 0;
	unsigned char buf[2];
	uint8_t mode_mask = 0;
	if(params) {
		mode_mask = *((uint8_t*)params);
	}

	buf[0] = next_lens_position >> 4;
	buf[1] = ((next_lens_position & 0x000F) << 4) | mode_mask;
	rc = msm_camera_i2c_txdata(&a_ctrl->i2c_client, buf, 2);
	if (rc < 0) {
		pr_err("%s: write failed (%d)\n", __func__, rc);
	}

	return rc;
}

int32_t s5k3h1gx_act_write_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	uint16_t curr_lens_pos,
	struct damping_params_t *damping_params,
	int8_t sign_direction,
	int16_t code_boundary)
{
	int32_t rc = 0;
	uint16_t dac_value = 0;

	LINFO("%s called, curr lens pos = %d, code_boundary = %d\n",
		  __func__,
		  curr_lens_pos,
		  code_boundary);

	if (sign_direction == 1)
		dac_value = (code_boundary - curr_lens_pos);
	else
		dac_value = (curr_lens_pos - code_boundary);

	LINFO("%s dac_value = %d\n",
		  __func__,
		  dac_value);

	rc = a_ctrl->func_tbl.actuator_i2c_write(a_ctrl, dac_value, NULL);

	return rc;
}

static int32_t s5k3h1gx_act_init_focus(struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;

	rc = a_ctrl->func_tbl.actuator_i2c_write(a_ctrl, a_ctrl->initial_code,
		NULL);
	if (rc < 0)
		pr_err("%s i2c write failed\n", __func__);
	else
		a_ctrl->curr_step_pos = 0;

	return rc;
}

static const struct i2c_device_id s5k3h1gx_act_i2c_id[] = {
	{"s5k3h1gx_act", (kernel_ulong_t)&s5k3h1gx_act_t},
	{ }
};

static int s5k3h1gx_act_config(
	void __user *argp)
{
	LINFO("%s called\n", __func__);
	return (int) msm_actuator_config(&s5k3h1gx_act_t,
		s5k3h1gx_msm_actuator_info, argp); 
}

static int s5k3h1gx_i2c_add_driver_table(
	void)
{
	int32_t rc = 0;

	LINFO("%s called\n", __func__);

	rc = s5k3h1gx_poweron_af();
	if (rc < 0) {
		pr_err("%s power on failed\n", __func__);
		return (int) rc;
	}

	s5k3h1gx_act_t.step_position_table = NULL;
	rc = s5k3h1gx_act_t.func_tbl.actuator_init_table(&s5k3h1gx_act_t);
	if (rc < 0) {
		pr_err("%s init table failed\n", __func__);
		return (int) rc;
	}

	rc = msm_camera_i2c_write(&(s5k3h1gx_act_t.i2c_client),
		0x0001, 0x01,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s i2c write failed\n", __func__);
		return (int) rc;
	}

	return (int) rc;
}

static struct i2c_driver s5k3h1gx_act_i2c_driver = {
	.id_table = s5k3h1gx_act_i2c_id,
	.probe  = msm_actuator_i2c_probe,
	.remove = __exit_p(s5k3h1gx_act_i2c_remove),
	.driver = {
		.name = "s5k3h1gx_act",
	},
};

static int __init s5k3h1gx_i2c_add_driver(void)
{
	LINFO("%s called\n", __func__);
	return i2c_add_driver(s5k3h1gx_act_t.i2c_driver);
}

static struct v4l2_subdev_core_ops s5k3h1gx_act_subdev_core_ops;

static struct v4l2_subdev_ops s5k3h1gx_act_subdev_ops = {
	.core = &s5k3h1gx_act_subdev_core_ops,
};

static int32_t s5k3h1gx_act_create_subdevice(void *act_info, void *sdev)
{
	LINFO("%s called\n", __func__);

	s5k3h1gx_msm_actuator_info = (struct msm_actuator_info *)act_info;

	return (int) msm_actuator_create_subdevice(&s5k3h1gx_act_t,
		s5k3h1gx_msm_actuator_info->board_info,
		(struct v4l2_subdev *)sdev);
}

static struct msm_actuator_ctrl_t s5k3h1gx_act_t = {
	.i2c_driver = &s5k3h1gx_act_i2c_driver,
	.i2c_addr = REG_VCM_I2C_ADDR,
	.act_v4l2_subdev_ops = &s5k3h1gx_act_subdev_ops,
	.actuator_ext_ctrl = {
		.a_init_table = s5k3h1gx_i2c_add_driver_table,
		.a_power_down = s5k3h1gx_actuator_af_power_down,
		.a_create_subdevice = s5k3h1gx_act_create_subdevice,
		.a_config = s5k3h1gx_act_config,
	},
	.i2c_client = {
		.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
	},
	.set_info = {
		.total_steps = S5K3H1GX_TOTAL_STEPS_NEAR_TO_FAR,
		.gross_steps = 3,	
		.fine_steps = 1,	
	},
	.curr_step_pos = 0,
	.curr_region_index = 0,
	.initial_code = 0,	
	.actuator_mutex = &s5k3h1gx_act_mutex,
	.func_tbl = {
		.actuator_init_table = s5k3h1gx_msm_actuator_init_table,
		.actuator_move_focus = s5k3h1gx_msm_actuator_move_focus,
		.actuator_write_focus = s5k3h1gx_act_write_focus,
		.actuator_set_default_focus = msm_actuator_set_default_focus,
		.actuator_init_focus = s5k3h1gx_act_init_focus,
		.actuator_i2c_write = s5k3h1gx_wrapper_i2c_write,
	},
	.get_info = {	
		.focal_length_num = 46,
		.focal_length_den = 10,
		.f_number_num = 265,
		.f_number_den = 100,
		.f_pix_num = 14,
		.f_pix_den = 10,
		.total_f_dist_num = 197681,
		.total_f_dist_den = 1000,
	},
};

subsys_initcall(s5k3h1gx_i2c_add_driver);
MODULE_DESCRIPTION("S5K3H1GX actuator");
MODULE_LICENSE("GPL v2");
