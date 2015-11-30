/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "msm_actuator.h"

int32_t msm_actuator_write_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	uint16_t curr_lens_pos,
	struct damping_params_t *damping_params,
	int8_t sign_direction,
	int16_t code_boundary)
{
	int32_t rc = 0;
	int16_t next_lens_pos = 0;
	uint16_t damping_code_step = 0;
	uint16_t wait_time = 0;

	damping_code_step = damping_params->damping_step;
	wait_time = damping_params->damping_delay;

	
	for (next_lens_pos =
		curr_lens_pos + (sign_direction * damping_code_step);
		(sign_direction * next_lens_pos) <=
			(sign_direction * code_boundary);
		next_lens_pos =
			(next_lens_pos +
				(sign_direction * damping_code_step))) {
		rc = a_ctrl->func_tbl.
			actuator_i2c_write(a_ctrl, next_lens_pos,
				(void *) damping_params->hw_params);
		curr_lens_pos = next_lens_pos;
		usleep(wait_time);
	}

	if (curr_lens_pos != code_boundary) {
		rc = a_ctrl->func_tbl.
			actuator_i2c_write(a_ctrl, code_boundary,
				(void *) damping_params->hw_params);
		usleep(wait_time);
	}
	return rc;
}


int32_t msm_actuator_move_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	int dir,
	int32_t num_steps)
{
	int32_t rc = 0;
	int8_t sign_dir = 0;
	uint16_t curr_scene = 0;
	uint16_t scenario_size = 0;
	uint16_t index = 0;
	uint16_t step_boundary = 0;
	uint16_t target_step_pos = 0;
	uint16_t target_lens_pos = 0;
	int16_t dest_step_pos = 0;
	uint16_t curr_lens_pos = 0;
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

	
	scenario_size = a_ctrl->scenario_size[dir];
	for (index = 0; index < scenario_size; index++) {
		if (num_steps <= a_ctrl->ringing_scenario[dir][index]) {
			curr_scene = index;
			break;
		}
	}

	curr_lens_pos = a_ctrl->step_position_table[a_ctrl->curr_step_pos];
	CDBG("curr_step_pos =%d dest_step_pos =%d curr_lens_pos=%d\n",
		a_ctrl->curr_step_pos, dest_step_pos, curr_lens_pos);

	while (a_ctrl->curr_step_pos != dest_step_pos) {
		step_boundary =
			a_ctrl->region_params[a_ctrl->curr_region_index].
			step_bound[dir];
		if ((dest_step_pos * sign_dir) <=
			(step_boundary * sign_dir)) {

			target_step_pos = dest_step_pos;
			target_lens_pos =
				a_ctrl->step_position_table[target_step_pos];
			curr_lens_pos = a_ctrl->func_tbl.
				actuator_write_focus(
					a_ctrl,
					curr_lens_pos,
					&(a_ctrl->damping[dir]\
						[a_ctrl->curr_region_index].
						ringing_params[curr_scene]),
					sign_dir,
					target_lens_pos);

		} else {
			target_step_pos = step_boundary;
			target_lens_pos =
				a_ctrl->step_position_table[target_step_pos];
			curr_lens_pos = a_ctrl->func_tbl.
				actuator_write_focus(
					a_ctrl,
					curr_lens_pos,
					&(a_ctrl->damping[dir]\
						[a_ctrl->curr_region_index].
						ringing_params[curr_scene]),
					sign_dir,
					target_lens_pos);

			a_ctrl->curr_region_index += sign_dir;
		}
		a_ctrl->curr_step_pos = target_step_pos;
	}

	return rc;
}

int32_t msm_actuator_init_table(
	struct msm_actuator_ctrl_t *a_ctrl)
{
	int16_t code_per_step = 0;
	int32_t rc = 0;
	int16_t cur_code = 0;
	int16_t step_index = 0, region_index = 0;
	uint16_t step_boundary = 0;
	LINFO("%s called\n", __func__);

	if (a_ctrl->func_tbl.actuator_set_params)
		a_ctrl->func_tbl.actuator_set_params(a_ctrl);

	
	a_ctrl->step_position_table =
		kmalloc(sizeof(uint16_t) * (a_ctrl->set_info.total_steps + 1),
			GFP_KERNEL);
	cur_code = a_ctrl->initial_code;
	a_ctrl->step_position_table[step_index++] = cur_code;
	for (region_index = 0;
		region_index < a_ctrl->region_size;
		region_index++) {
		code_per_step =
			a_ctrl->region_params[region_index].code_per_step;
		step_boundary =
			a_ctrl->region_params[region_index].
			step_bound[MOVE_NEAR];
		for (; step_index <= step_boundary;
			step_index++) {
			cur_code += code_per_step;
			a_ctrl->step_position_table[step_index] = cur_code;
		}
	}
	for (step_index = 0;
		step_index < a_ctrl->set_info.total_steps;
		step_index++) {
		CDBG("step_position_table[%d]= %d\n",
			step_index,
			a_ctrl->step_position_table[step_index]);
	}

	a_ctrl->curr_step_pos = 0;
	a_ctrl->curr_region_index = 0;
	a_ctrl->af_OTP_info.VCM_OTP_Read = false;
	return rc;
}

int32_t msm_actuator_set_default_focus(
	struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	LINFO("%s called\n", __func__);

	if (!a_ctrl->step_position_table)
		a_ctrl->func_tbl.actuator_init_table(a_ctrl);

	if (a_ctrl->curr_step_pos != 0)
		rc = a_ctrl->func_tbl.actuator_move_focus(a_ctrl, MOVE_FAR,
			a_ctrl->curr_step_pos);
	else if (a_ctrl->func_tbl.actuator_init_focus)
		rc = a_ctrl->func_tbl.actuator_init_focus(a_ctrl);
	return rc;
}

int32_t msm_actuator_af_power_down(struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	LINFO("%s called\n", __func__);
	if (!a_ctrl->step_position_table)
	    return rc;

	if (a_ctrl->step_position_table[a_ctrl->curr_step_pos] !=
		a_ctrl->initial_code) {
		rc = a_ctrl->func_tbl.actuator_set_default_focus(a_ctrl);
		LINFO("%s after msm_actuator_set_default_focus\n", __func__);
	}
	kfree(a_ctrl->step_position_table);
	a_ctrl->step_position_table=NULL; 
	return rc;
}

int32_t msm_actuator_config(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_info *board_info,
	void __user *argp) 
{
	struct msm_actuator_cfg_data cdata;
	int32_t rc = 0;
	if (copy_from_user(&cdata,
		(void *)argp,
		sizeof(struct msm_actuator_cfg_data)))
		return -EFAULT;
	mutex_lock(a_ctrl->actuator_mutex);
	LINFO("%s called, type %d\n", __func__, cdata.cfgtype);
	switch (cdata.cfgtype) {
	case CFG_GET_ACTUATOR_INFO:
		cdata.is_af_supported = 1;
		cdata.is_ois_supported = a_ctrl->actuator_ext_ctrl.is_ois_supported;
#if (CONFIG_HTC_CAMERA_HAL_VERSION >= 3)
		cdata.is_af_infinity_supported = a_ctrl->actuator_ext_ctrl.is_af_infinity_supported;
#endif
		cdata.cfg.get_info = a_ctrl->get_info;
		if (copy_to_user((void *)argp,
				 &cdata,
				 sizeof(struct msm_actuator_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_SET_ACTUATOR_INFO:
		a_ctrl->set_info = cdata.cfg.set_info;
#if (CONFIG_HTC_CAMERA_HAL_VERSION >= 3)
		a_ctrl->enable_focus_step_log = cdata.enable_focus_step_log;
#endif
		rc = a_ctrl->func_tbl.actuator_init_table(a_ctrl);
		if (rc < 0)
			LERROR("%s init table failed %d\n", __func__, rc);
		break;

	case CFG_SET_DEFAULT_FOCUS:
		rc = a_ctrl->func_tbl.actuator_set_default_focus(a_ctrl);
		if (rc < 0)
			LERROR("%s move focus failed %d\n", __func__, rc);
		break;

	case CFG_MOVE_FOCUS:
		rc = a_ctrl->func_tbl.actuator_move_focus(a_ctrl,
			cdata.cfg.move.dir,
			cdata.cfg.move.num_steps);
		if (rc < 0)
			LERROR("%s move focus failed %d\n", __func__, rc);
		break;

	case CFG_GET_ACTUATOR_CURR_STEP_POS:
		LINFO("%s current step: %d\n", __func__, a_ctrl->curr_step_pos);
		cdata.cfg.curr_step_pos = a_ctrl->curr_step_pos;
		if (copy_to_user((void *)argp,
				 &cdata,
				 sizeof(struct msm_actuator_cfg_data)))
			rc = -EFAULT;
		break;
	case CFG_SET_ACTUATOR_AF_ALGO:
		a_ctrl->af_algo = cdata.cfg.af_algo;
		rc = a_ctrl->func_tbl.actuator_init_table(a_ctrl);
		if (rc < 0)
			LERROR("%s init table failed %d\n", __func__, rc);
		break;

	case CFG_SET_OIS_MODE:
		if (a_ctrl->actuator_ext_ctrl.is_ois_supported) {
			if (a_ctrl->func_tbl.actuator_set_ois_mode != NULL) {
				rc = a_ctrl->func_tbl.actuator_set_ois_mode(a_ctrl, cdata.cfg.ois_mode);
				if (rc < 0)
					LERROR("%s set ois mode failed %d\n", __func__, rc);
			} else {
				LERROR("%s a_ctrl->func_tbl.actuator_set_ois_mode is NULL\n", __func__);
			}
		} else {
			LINFO("%s ois is not supported\n", __func__);
		}
		break;
	case CFG_UPDATE_OIS_TBL:
		if (a_ctrl->actuator_ext_ctrl.is_ois_supported) {
			if (a_ctrl->func_tbl.actuator_update_ois_tbl != NULL) {
				rc = a_ctrl->func_tbl.actuator_update_ois_tbl(a_ctrl, &(cdata.cfg.sensor_actuator_info));
				if (rc < 0)
					LERROR("%s update ois table failed %d\n", __func__, rc);
			} else {
				LERROR("%s a_ctrl->func_tbl.actuator_update_ois_tbl is NULL\n", __func__);
			}
		} else {
			LINFO("%s ois is not supported\n", __func__);
		}
		break;
	case CFG_GET_OIS_DEBUG_INFO:
		if (a_ctrl->actuator_ext_ctrl.is_ois_supported) {
			cdata.cfg.get_ois_info = a_ctrl->get_ois_info;
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct msm_actuator_cfg_data)))
			rc = -EFAULT;
		} else {
			LINFO("%s ois is not supported\n", __func__);
		}
		break;
	case CFG_GET_OIS_DEBUG_TBL:
		if (a_ctrl->actuator_ext_ctrl.is_ois_supported) {
			cdata.cfg.get_ois_tbl = a_ctrl->get_ois_tbl;
			if (copy_to_user((void *)argp,
				&cdata,
				sizeof(struct msm_actuator_cfg_data)))
			rc = -EFAULT;
		} else {
			LINFO("%s ois is not supported\n", __func__);
		}
		break;
	case CFG_SET_ACTUATOR_AF_VALUE:
		if (a_ctrl->func_tbl.actuator_set_af_value != NULL) {
			rc = a_ctrl->func_tbl.actuator_set_af_value(a_ctrl, cdata.cfg.af_value);
			if (rc < 0)
				LERROR("%s set af value failed %d\n", __func__, rc);
		} else {
			LERROR("%s a_ctrl->func_tbl.actuator_set_af_value is NULL\n", __func__);
		}
		break;
	case CFG_SET_OIS_CALIBRATION:
		if (a_ctrl->actuator_ext_ctrl.is_ois_supported) {
			if (a_ctrl->func_tbl.actuator_set_ois_calibration != NULL) {
#if (CONFIG_HTC_CAMERA_HAL_VERSION >= 2)
				cdata.cfg.get_osi_cal_info.bypass_ois_cal = false;
#endif
				rc = a_ctrl->func_tbl.actuator_set_ois_calibration(a_ctrl, &cdata.cfg.get_osi_cal_info);
				if (rc < 0) {
					LERROR("%s set ois calibration failed %d\n", __func__, rc);
				} else {
					if (copy_to_user((void *)argp,
						&cdata,
						sizeof(struct msm_actuator_cfg_data)))
						rc = -EFAULT;
				}
			} else {
#if (CONFIG_HTC_CAMERA_HAL_VERSION >= 2)
				pr_info("%s a_ctrl->func_tbl.actuator_set_ois_calibration is NULL  ,  bypass ois calibration\n", __func__);
				cdata.cfg.get_osi_cal_info.bypass_ois_cal = true;
				if (copy_to_user((void *)argp,
					&cdata,
					sizeof(struct msm_actuator_cfg_data)))
					rc = -EFAULT;
#else
				LERROR("%s a_ctrl->func_tbl.actuator_set_ois_calibration is NULL\n", __func__);
				rc = -EFAULT;
#endif
			}
		} else {
			LINFO("%s ois is not supported\n", __func__);
		}
		break;
	
    case CFG_SET_VCM_CALIBRATION:
        if (a_ctrl->actuator_ext_ctrl.is_cal_supported) {
            if (a_ctrl->func_tbl.actuator_do_cal) {
                rc = a_ctrl->func_tbl.actuator_do_cal (a_ctrl, &cdata.cfg.get_vcm_cal_info);
                if (rc < 0) {
                    LERROR("%s calibration failed %d\n", __func__, rc);
                } else {
                    if (copy_to_user((void *)argp,
                        &cdata,
                        sizeof(struct msm_actuator_cfg_data)))
                        rc = -EFAULT;
                }
            }
            else {
                LERROR("%s a_ctrl->func_tbl.acturator_do_cal is NULL\n", __func__);
                rc = -EFAULT;
            }
        }
        else {
            LINFO("%s cal is not supported\n", __func__);
        }
        break;
	
	default:
		break;
	}
	mutex_unlock(a_ctrl->actuator_mutex);
	return rc;
}

int32_t msm_actuator_i2c_probe(
	struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_actuator_ctrl_t *act_ctrl_t = NULL;
	LINFO("%s called\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	act_ctrl_t = (struct msm_actuator_ctrl_t *)(id->driver_data);
	i2c_set_clientdata(client, (void *)&act_ctrl_t->actuator_ext_ctrl);
	LINFO("%s client = %x act ctrl t = %x\n",
		__func__,
		(unsigned int) client,
		(unsigned int)&act_ctrl_t->actuator_ext_ctrl);
	act_ctrl_t->i2c_client.client = client;
	if (act_ctrl_t->i2c_addr != 0)
		act_ctrl_t->i2c_client.client->addr =
			act_ctrl_t->i2c_addr;

	
	LINFO("%s succeeded\n", __func__);
	return rc;

probe_failure:
	pr_err("%s failed! rc = %d\n", __func__, rc);
	return rc;
}

int32_t msm_actuator_create_subdevice(struct msm_actuator_ctrl_t *a_ctrl,
	struct i2c_board_info const *board_info,
	struct v4l2_subdev *sdev)
{
	int32_t rc = 0;

	LINFO("%s called\n", __func__);

	
	a_ctrl->sdev = sdev;

	
	snprintf(sdev->name, sizeof(sdev->name), "%s", board_info->type);

	
	v4l2_i2c_subdev_init(sdev,
		a_ctrl->i2c_client.client,
		a_ctrl->act_v4l2_subdev_ops);

	return rc;
}
