/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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

#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/ioctl.h>
#include <linux/spinlock.h>
#include <linux/proc_fs.h>
#include "msm.h"
#include "msm_csid.h"
#include "msm_csic.h"
#include "msm_csiphy.h"
#include "msm_ispif.h"

#include "msm_sensor.h"
#include "msm_vfe32.h"

#include <linux/switch.h>

#ifdef CONFIG_RAWCHIP
#include "rawchip/rawchip.h"
#endif

#define MSM_MAX_CAMERA_SENSORS 5

#ifdef CONFIG_MSM_CAMERA_DEBUG
#define D(fmt, args...) pr_debug("msm: " fmt, ##args)
#else
#define D(fmt, args...) do {} while (0)
#endif

static unsigned msm_camera_v4l2_nr = -1;
static struct msm_cam_server_dev g_server_dev;
static struct class *msm_class;
static dev_t msm_devno;
static int vnode_count;

static struct cam_vcm_wa_ctrl g_vcm_wa_ctl;
DEFINE_MUTEX(cam_vcm_on_mut);
static enum cam_vcm_onoff_type cam_vcm_on = STATUS_OFF;
static enum cam_vcm_onoff_type camera_on = STATUS_OFF;
static struct workqueue_struct *cam_vcm_off_wq;
static struct work_struct cam_vcm_off_work;
static int is_actuator_probe_success = 0;
static void cam_on_check_vcm(void);
static atomic_t serv_running[MAX_NUM_ACTIVE_CAMERA];

module_param(msm_camera_v4l2_nr, uint, 0644);
MODULE_PARM_DESC(msm_camera_v4l2_nr, "videoX start number, -1 is autodetect");

static long msm_server_send_v4l2_evt(void *evt);
static void msm_cam_server_subdev_notify(struct v4l2_subdev *sd,
	unsigned int notification, void *arg);

static void msm_queue_init(struct msm_device_queue *queue, const char *name)
{
	D("%s\n", __func__);
	spin_lock_init(&queue->lock);
	queue->len = 0;
	queue->max = 0;
	queue->name = name;
	INIT_LIST_HEAD(&queue->list);
	init_waitqueue_head(&queue->wait);
}

static void msm_enqueue(struct msm_device_queue *queue,
			struct list_head *entry)
{
	unsigned long flags;
	spin_lock_irqsave(&queue->lock, flags);
	queue->len++;
	if (queue->len > queue->max) {
		queue->max = queue->len;
		pr_info("%s: queue %s new max is %d\n", __func__,
			queue->name, queue->max);
	}
	list_add_tail(entry, &queue->list);
	wake_up(&queue->wait);
	D("%s: woke up %s\n", __func__, queue->name);
	spin_unlock_irqrestore(&queue->lock, flags);
}
static void msm_drain_eventq(struct msm_device_queue *queue)
{
	unsigned long flags;
	struct msm_queue_cmd *qcmd;
	struct msm_isp_event_ctrl *isp_event;

	spin_lock_irqsave(&queue->lock, flags);
	while (!list_empty(&queue->list)) {
		qcmd = list_first_entry(&queue->list,
			struct msm_queue_cmd, list_eventdata);
		list_del_init(&qcmd->list_eventdata);
		isp_event =
			(struct msm_isp_event_ctrl *)
			qcmd->command;
		if (isp_event->isp_data.ctrl.value != NULL)
			kfree(isp_event->isp_data.ctrl.value);

		kfree(qcmd->command);
		free_qcmd(qcmd);
	}
	spin_unlock_irqrestore(&queue->lock, flags);
}

static int32_t msm_find_free_queue(void)
{
	int i;
	for (i = 0; i < MAX_NUM_ACTIVE_CAMERA; i++) {
		struct msm_cam_server_queue *queue;
		queue = &g_server_dev.server_queue[i];
		if (!queue->queue_active)
			return i;
	}
	return -EINVAL;
}

uint32_t msm_camera_get_mctl_handle(void)
{
	uint32_t i;
	if ((g_server_dev.mctl_handle_cnt << 8) == 0)
		g_server_dev.mctl_handle_cnt++;
	for (i = 0; i < MAX_NUM_ACTIVE_CAMERA; i++) {
		if (g_server_dev.mctl[i].handle == 0) {
			g_server_dev.mctl[i].handle =
				(++g_server_dev.mctl_handle_cnt) << 8 | i;
			memset(&g_server_dev.mctl[i].mctl,
				   0, sizeof(g_server_dev.mctl[i].mctl));
			return g_server_dev.mctl[i].handle;
		}
	}
	return 0;
}

struct msm_cam_media_controller *msm_camera_get_mctl(uint32_t handle)
{
	uint32_t mctl_index;
	mctl_index = handle & 0xff;
	if ((mctl_index < MAX_NUM_ACTIVE_CAMERA) &&
		(g_server_dev.mctl[mctl_index].handle == handle))
		return &g_server_dev.mctl[mctl_index].mctl;
	return NULL;
}

void msm_camera_free_mctl(uint32_t handle)
{
	uint32_t mctl_index;
	mctl_index = handle & 0xff;
	if ((mctl_index < MAX_NUM_ACTIVE_CAMERA) &&
		(g_server_dev.mctl[mctl_index].handle == handle))
		g_server_dev.mctl[mctl_index].handle = 0;
	else
		pr_err("%s: invalid free handle\n", __func__);
}

struct msm_cam_media_controller *msm_camera_get_rdi0_mctl(void)
{
	return g_server_dev.rdi0_mctl;
}

void msm_camera_set_rdi0_mctl(struct msm_cam_media_controller *mctl)
{
	g_server_dev.rdi0_mctl = mctl;
}

struct msm_cam_media_controller *msm_camera_get_pix0_mctl(void)
{
	return g_server_dev.pix0_mctl;
}

void msm_camera_set_pix0_mctl(struct msm_cam_media_controller *mctl)
{
	g_server_dev.pix0_mctl = mctl;
}

static void msm_cam_v4l2_subdev_notify(struct v4l2_subdev *sd,
				unsigned int notification, void *arg)
{
	struct msm_cam_v4l2_device *pcam;
	struct msm_cam_media_controller *pmctl;

	if (sd == NULL)
		return;

	pcam = to_pcam(sd->v4l2_dev);

	if (pcam == NULL)
		return;
	pmctl = msm_camera_get_mctl(pcam->mctl_handle);
	if (pmctl == NULL)
		return;

}

static int msm_ctrl_cmd_done(void *arg)
{
	void __user *uptr;
	struct msm_queue_cmd *qcmd;
	struct msm_camera_v4l2_ioctl_t *ioctl_ptr = arg;
	struct msm_ctrl_cmd *command;
	D("%s\n", __func__);

	command = kzalloc(sizeof(struct msm_ctrl_cmd), GFP_KERNEL);

	if (!command) {
		pr_err("%s Insufficient memory. return", __func__);
		goto command_alloc_fail;
	}
 
 	 qcmd = kzalloc(sizeof(struct msm_queue_cmd), GFP_KERNEL);
 	 if (!qcmd) {
 		 pr_err("%s Insufficient memory. return", __func__);
 		 goto qcmd_alloc_fail;
 	 }
 
 	 mutex_lock(&g_server_dev.server_queue_lock);
	if (copy_from_user(command, (void __user *)ioctl_ptr->ioctl_ptr,
					   sizeof(struct msm_ctrl_cmd))) {
		pr_err("%s: copy_from_user failed, size=%d\n",
			   __func__, sizeof(struct msm_ctrl_cmd));
		goto ctrl_cmd_done_error;
	}

	if(command->queue_idx < 0 ||
		command->queue_idx >= MAX_NUM_ACTIVE_CAMERA) {
		pr_err("%s: Invalid value OR index %d\n", __func__,
			command->queue_idx);
		goto ctrl_cmd_done_error;
	}

	if (!g_server_dev.server_queue[command->queue_idx].queue_active) {
		pr_err("%s: Invalid queue\n", __func__);
		goto ctrl_cmd_done_error;
	}
	D("%s qid %d evtid %d %d\n", __func__, command->queue_idx,
		command->evt_id,
		g_server_dev.server_queue[command->queue_idx].evt_id);

	if (command->evt_id !=
		g_server_dev.server_queue[command->queue_idx].evt_id) {
		pr_err("%s Invalid event id from userspace cmd id %d %d qid %d\n",
			__func__, command->evt_id,
			g_server_dev.server_queue[command->queue_idx].evt_id,
			command->queue_idx);
		goto ctrl_cmd_done_error;
	}
	atomic_set(&qcmd->on_heap, 1);
	uptr = command->value;
	qcmd->command = command;

	if (command->length > 0) {
		command->value =
			g_server_dev.server_queue[command->queue_idx].ctrl_data;
		if (command->length > max_control_command_size) {
			pr_err("%s: user data %d is too big (max %d)\n",
				__func__, command->length,
				max_control_command_size);
			goto ctrl_cmd_done_error;
		}
		if (copy_from_user(command->value, (void __user *)uptr,
				command->length)) {
			pr_err("%s: copy_from_user failed, size=%d\n",
			__func__, sizeof(struct msm_ctrl_cmd));
			goto ctrl_cmd_done_error;
		}
	}

	msm_enqueue(&g_server_dev.server_queue
		[command->queue_idx].ctrl_q, &qcmd->list_control);
	mutex_unlock(&g_server_dev.server_queue_lock);
	return 0;

ctrl_cmd_done_error:
	mutex_unlock(&g_server_dev.server_queue_lock);
	free_qcmd(qcmd);
qcmd_alloc_fail:
	kfree(command);
command_alloc_fail:
	return -EINVAL;	
}


static void msm_cam_stop_hardware(struct msm_cam_v4l2_device *pcam)
{
	struct msm_cam_media_controller *pmctl;
	int rc = 0;
	int i=100;
	if (pcam == NULL)
		return;
	pmctl = msm_camera_get_mctl(pcam->mctl_handle);
	if (pmctl && pmctl->mctl_release) {
		pr_err("%s: stopping hardware upon error\n", __func__);
		
		pmctl->mctl_cmd = NULL;
		while (i--) {
			if (!atomic_read(&pmctl->dispatch_command)) {
				pr_info("%s: no executing mctl command at %d\n", __func__,i);
				break;
			}
			msleep(5);
		}

		rc = pmctl->mctl_release(pmctl);
		if (rc < 0)
			pr_err("mctl_release fails %d\n", rc);
		pmctl->mctl_release = NULL;
	}
}


static int msm_server_control(struct msm_cam_server_dev *server_dev,
				struct msm_ctrl_cmd *out)
{
	int rc = 0;
	void *value;
	struct msm_queue_cmd *rcmd;
	struct msm_queue_cmd *event_qcmd;	
	struct msm_ctrl_cmd *ctrlcmd;
	struct msm_device_queue *queue =
		&server_dev->server_queue[out->queue_idx].ctrl_q;
	struct msm_cam_v4l2_device *pcam = server_dev->pcam_active[out->queue_idx];
	struct v4l2_event v4l2_evt;
	struct msm_isp_event_ctrl *isp_event;
	void *ctrlcmd_data;
	int loop = 0; 

	if (!atomic_read(&serv_running[pcam->server_queue_idx]) && out->type != MSM_V4L2_OPEN) {
		pr_info("%s: daemon hasn't subscribed yet!\n", __func__);
		return -EIO;
	}

	event_qcmd = kzalloc(sizeof(struct msm_queue_cmd), GFP_KERNEL);
	if (!event_qcmd) {
		pr_err("%s Insufficient memory. return", __func__);
		rc = -ENOMEM;
		goto event_qcmd_alloc_fail;
	}

	isp_event = kzalloc(sizeof(struct msm_isp_event_ctrl), GFP_KERNEL);
	if (!isp_event) {
		pr_err("%s Insufficient memory. return", __func__);
		rc = -ENOMEM;
		goto isp_event_alloc_fail;
	}

	D("%s\n", __func__);
	mutex_lock(&server_dev->server_queue_lock);
	if (++server_dev->server_evt_id == 0)
		server_dev->server_evt_id++;
 
	 D("%s qid %d evtid %d\n", __func__, out->queue_idx,
		 server_dev->server_evt_id);
	server_dev->server_queue[out->queue_idx].evt_id =
		 server_dev->server_evt_id;
	v4l2_evt.type = V4L2_EVENT_PRIVATE_START + MSM_CAM_RESP_V4L2;
	v4l2_evt.u.data[0] = out->queue_idx;
	
	isp_event->resptype = MSM_CAM_RESP_V4L2;
	isp_event->isp_data.ctrl = *out;
	isp_event->isp_data.ctrl.evt_id = server_dev->server_evt_id;
	if (out->value != NULL && out->length != 0) {
		ctrlcmd_data = kzalloc(out->length, GFP_KERNEL);
		if (!ctrlcmd_data) {
			rc = -ENOMEM;
			goto ctrlcmd_alloc_fail;
		}
		memcpy(ctrlcmd_data, out->value, out->length);
		isp_event->isp_data.ctrl.value = ctrlcmd_data;
	}

	atomic_set(&event_qcmd->on_heap, 1);
	event_qcmd->command = isp_event;

	msm_enqueue(&server_dev->server_queue[out->queue_idx].eventData_q,
				&event_qcmd->list_eventdata);

	v4l2_event_queue(server_dev->server_command_queue.pvdev,
					  &v4l2_evt);

	D("%s v4l2_event_queue: type = 0x%x\n", __func__, v4l2_evt.type);
	mutex_unlock(&server_dev->server_queue_lock);
	
	D("Waiting for config status\n");

wait_event:
	rc = wait_event_timeout(queue->wait,
		!list_empty_careful(&queue->list),
		msecs_to_jiffies(out->timeout_ms));
	D("Waiting is over for config status\n");
	if (list_empty_careful(&queue->list)) {
		if (!rc)
			rc = -ETIMEDOUT;
		if (rc == -ERESTARTSYS && loop < 20) {
			loop++;
			msleep(5);
			pr_info("%s: goto wait_event loop %d\n", __func__, loop);
			goto wait_event;
		}
		else if (rc < 0) {
			
			struct msm_cam_media_controller *pmctl = NULL;
			struct msm_sensor_ctrl_t *s_ctrl = NULL;
			pmctl = msm_camera_get_mctl(pcam->mctl_handle);
			if (pmctl)
				s_ctrl = get_sctrl(pmctl->sensor_sdev);
			
			if (++server_dev->server_evt_id == 0)
				server_dev->server_evt_id++;
			pr_err("%s: wait_event error %d\n", __func__, rc);
			
			pr_info("%s: ctrlcmd.type = %d\n", __func__, out->type);
			if (out->type == MSM_V4L2_SET_CTRL_CMD) {
				if (out->value)
					pr_info("%s: ctrl_cmd.type = %d\n", __func__, 
						((struct msm_ctrl_cmd *)out->value)->type);
			}
			
			if (out->type == MSM_V4L2_CLOSE) {
				msm_cam_stop_hardware(pcam);
			} else {
				if(s_ctrl && s_ctrl->sensor_first_mutex)  {
					mutex_lock(s_ctrl->sensor_first_mutex);
				}

				msm_cam_stop_hardware(pcam);

				if(s_ctrl && s_ctrl->sensor_first_mutex)  {
					mutex_unlock(s_ctrl->sensor_first_mutex);
				}
			}

			return rc;
		}
	}

	rcmd = msm_dequeue(queue, list_control);
	BUG_ON(!rcmd);
	if(!rcmd) return -EINVAL;
	D("%s Finished servicing ioctl\n", __func__);

	ctrlcmd = (struct msm_ctrl_cmd *)(rcmd->command);
	value = out->value;
	if (ctrlcmd->length > 0 && value != NULL &&
		ctrlcmd->length <= out->length)
		memcpy(value, ctrlcmd->value, ctrlcmd->length);

	memcpy(out, ctrlcmd, sizeof(struct msm_ctrl_cmd));
	out->value = value;
	kfree(ctrlcmd);

	free_qcmd(rcmd);
	D("%s: rc %d\n", __func__, rc);
	
	if (rc >= 0) {
		
		if (out->status == 0)
			rc = -1;
		else if (out->status == 1 || out->status == 4)
			rc = 0;
		else
			rc = -EINVAL;
	}
	return rc;

ctrlcmd_alloc_fail:
	kfree(isp_event);
isp_event_alloc_fail:
	kfree(event_qcmd);
event_qcmd_alloc_fail:
	return rc;	
}

static int msm_send_open_server(struct msm_cam_v4l2_device *pcam)
{
	int rc = 0;
	struct msm_ctrl_cmd ctrlcmd;
	pr_info("%s qid %d\n", __func__, pcam->server_queue_idx);

	memset(&ctrlcmd, 0, sizeof(ctrlcmd)); 
	ctrlcmd.type	   = MSM_V4L2_OPEN;
	ctrlcmd.timeout_ms = 20000;
	ctrlcmd.length	 = strnlen(g_server_dev.config_info.config_dev_name[pcam->server_queue_idx],
				MAX_DEV_NAME_LEN)+1;
	ctrlcmd.value    = (char *)g_server_dev.config_info.config_dev_name[pcam->server_queue_idx];
	ctrlcmd.vnode_id = pcam->vnode_id;
	ctrlcmd.queue_idx = pcam->server_queue_idx;
	ctrlcmd.config_ident = g_server_dev.config_info.config_dev_id[pcam->server_queue_idx];

	
	rc = msm_server_control(&g_server_dev, &ctrlcmd);

	return rc;
}

static int msm_send_close_server(struct msm_cam_v4l2_device *pcam)
{
	int rc = 0;
	struct msm_ctrl_cmd ctrlcmd;
	pr_info("%s qid %d\n", __func__, pcam->server_queue_idx);

	memset(&ctrlcmd, 0, sizeof(ctrlcmd)); 
	ctrlcmd.type	   = MSM_V4L2_CLOSE;
	ctrlcmd.timeout_ms = 20000;
	ctrlcmd.length	 = strnlen(g_server_dev.config_info.config_dev_name[pcam->server_queue_idx],
				MAX_DEV_NAME_LEN)+1;
	ctrlcmd.value    = (char *)g_server_dev.config_info.config_dev_name[pcam->server_queue_idx];
	ctrlcmd.vnode_id = pcam->vnode_id;
	ctrlcmd.queue_idx = pcam->server_queue_idx;
	ctrlcmd.config_ident = g_server_dev.config_info.config_dev_id[pcam->server_queue_idx];

	
	rc = msm_server_control(&g_server_dev, &ctrlcmd);
	if (rc == 0)   {
		pr_info("%s: serv_running[%d] = %d\n", __func__,pcam->server_queue_idx, 0);
		atomic_set(&serv_running[pcam->server_queue_idx],0);
	}

	return rc;
}

static int msm_server_set_fmt(struct msm_cam_v4l2_device *pcam, int idx,
				 struct v4l2_format *pfmt)
{
	int rc = 0;
	int i = 0;
	struct v4l2_pix_format *pix = &pfmt->fmt.pix;
	struct msm_ctrl_cmd ctrlcmd;
	struct img_plane_info plane_info;

	plane_info.width = pix->width;
	plane_info.height = pix->height;
	plane_info.pixelformat = pix->pixelformat;
	plane_info.buffer_type = pfmt->type;
	plane_info.ext_mode = pcam->dev_inst[idx]->image_mode;
	plane_info.num_planes = 1;
	D("%s: %d, %d, 0x%x\n", __func__,
		pfmt->fmt.pix.width, pfmt->fmt.pix.height,
		pfmt->fmt.pix.pixelformat);

	if (pfmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		D("%s, Attention! Wrong buf-type %d\n", __func__, pfmt->type);

	for (i = 0; i < pcam->num_fmts; i++)
		if (pcam->usr_fmts[i].fourcc == pix->pixelformat)
			break;
	if (i == pcam->num_fmts) {
		pr_err("%s: User requested pixelformat %x not supported\n",
						__func__, pix->pixelformat);
		return -EINVAL;
	}

	memset(&ctrlcmd, 0, sizeof(ctrlcmd)); 
	ctrlcmd.type       = MSM_V4L2_VID_CAP_TYPE;
	ctrlcmd.length     = sizeof(struct img_plane_info);
	ctrlcmd.value      = (void *)&plane_info;
	ctrlcmd.timeout_ms = 10000;
	ctrlcmd.vnode_id   = pcam->vnode_id;
	ctrlcmd.queue_idx = pcam->server_queue_idx;	
	ctrlcmd.config_ident = g_server_dev.config_info.config_dev_id[pcam->server_queue_idx];

	
	rc = msm_server_control(&g_server_dev, &ctrlcmd);

	if (rc >= 0) {
		pcam->dev_inst[idx]->vid_fmt = *pfmt;
		pcam->dev_inst[idx]->sensor_pxlcode
					= pcam->usr_fmts[i].pxlcode;
		D("%s:inst=0x%x,idx=%d,width=%d,heigth=%d\n",
			 __func__, (u32)pcam->dev_inst[idx], idx,
			 pcam->dev_inst[idx]->vid_fmt.fmt.pix.width,
			 pcam->dev_inst[idx]->vid_fmt.fmt.pix.height);
		pcam->dev_inst[idx]->plane_info = plane_info;
	}

	return rc;
}

static int msm_server_set_fmt_mplane(struct msm_cam_v4l2_device *pcam, int idx,
				 struct v4l2_format *pfmt)
{
	int rc = 0;
	int i = 0;
	struct v4l2_pix_format_mplane *pix_mp = &pfmt->fmt.pix_mp;
	struct msm_ctrl_cmd ctrlcmd;
	struct img_plane_info plane_info;

	plane_info.width = pix_mp->width;
	plane_info.height = pix_mp->height;
	plane_info.pixelformat = pix_mp->pixelformat;
	plane_info.buffer_type = pfmt->type;
	plane_info.ext_mode = pcam->dev_inst[idx]->image_mode;
	plane_info.num_planes = pix_mp->num_planes;
	if (plane_info.num_planes <= 0 ||
		plane_info.num_planes > VIDEO_MAX_PLANES) {
		pr_err("%s Invalid number of planes set %d", __func__,
				plane_info.num_planes);
		return -EINVAL;
	}
	D("%s: %d, %d, 0x%x\n", __func__,
		pfmt->fmt.pix_mp.width, pfmt->fmt.pix_mp.height,
		pfmt->fmt.pix_mp.pixelformat);

	if (pfmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pr_err("%s, Attention! Wrong buf-type %d\n",
			__func__, pfmt->type);
		return -EINVAL;
	}

	for (i = 0; i < pcam->num_fmts; i++)
		if (pcam->usr_fmts[i].fourcc == pix_mp->pixelformat)
			break;
	if (i == pcam->num_fmts) {
		pr_err("%s: User requested pixelformat %x not supported\n",
						__func__, pix_mp->pixelformat);
		return -EINVAL;
	}

	memset(&ctrlcmd, 0, sizeof(ctrlcmd)); 
	ctrlcmd.type       = MSM_V4L2_VID_CAP_TYPE;
	ctrlcmd.length     = sizeof(struct img_plane_info);
	ctrlcmd.value      = (void *)&plane_info;
	ctrlcmd.timeout_ms = 20000;
	ctrlcmd.vnode_id   = pcam->vnode_id;
	ctrlcmd.queue_idx = pcam->server_queue_idx;

	
	rc = msm_server_control(&g_server_dev, &ctrlcmd);
	if (rc >= 0) {
		pcam->dev_inst[idx]->vid_fmt = *pfmt;
		pcam->dev_inst[idx]->sensor_pxlcode
					= pcam->usr_fmts[i].pxlcode;
		D("%s:inst=0x%x,idx=%d,width=%d,heigth=%d\n",
			 __func__, (u32)pcam->dev_inst[idx], idx,
			 pcam->dev_inst[idx]->vid_fmt.fmt.pix_mp.width,
			 pcam->dev_inst[idx]->vid_fmt.fmt.pix_mp.height);
		pcam->dev_inst[idx]->plane_info = plane_info;
	}

	return rc;
}

static int msm_server_streamon(struct msm_cam_v4l2_device *pcam, int idx)
{
	int rc = 0;
	struct msm_ctrl_cmd ctrlcmd;
	D("%s\n", __func__);

	memset(&ctrlcmd, 0, sizeof(ctrlcmd)); 
	ctrlcmd.type	   = MSM_V4L2_STREAM_ON;
	ctrlcmd.timeout_ms = 6000;
	ctrlcmd.length	 = 0;
	ctrlcmd.value    = NULL;
	ctrlcmd.stream_type = pcam->dev_inst[idx]->image_mode;
	ctrlcmd.vnode_id = pcam->vnode_id;
	ctrlcmd.queue_idx = pcam->server_queue_idx;
	ctrlcmd.config_ident = g_server_dev.config_info.config_dev_id[pcam->server_queue_idx];


	
	rc = msm_server_control(&g_server_dev, &ctrlcmd);

	return rc;
}

static int msm_server_streamoff(struct msm_cam_v4l2_device *pcam, int idx)
{
	int rc = 0;
	struct msm_ctrl_cmd ctrlcmd;

	D("%s, pcam = 0x%x\n", __func__, (u32)pcam);
	memset(&ctrlcmd, 0, sizeof(ctrlcmd));  
	ctrlcmd.type        = MSM_V4L2_STREAM_OFF;
	ctrlcmd.timeout_ms  = 6000;
	ctrlcmd.length      = 0;
	ctrlcmd.value       = NULL;
	ctrlcmd.stream_type = pcam->dev_inst[idx]->image_mode;
	ctrlcmd.vnode_id = pcam->vnode_id;
	ctrlcmd.queue_idx = pcam->server_queue_idx;
	ctrlcmd.config_ident = g_server_dev.config_info.config_dev_id[pcam->server_queue_idx];

	
	rc = msm_server_control(&g_server_dev, &ctrlcmd);

	return rc;
}

static int msm_server_proc_ctrl_cmd(struct msm_cam_v4l2_device *pcam,
				 struct v4l2_control *ctrl, int is_set_cmd)
{
	int rc = 0;
	struct msm_ctrl_cmd ctrlcmd, *tmp_cmd;
	uint8_t *ctrl_data = NULL;
	void __user *uptr_cmd;
	void __user *uptr_value;
	uint32_t cmd_len = sizeof(struct msm_ctrl_cmd);
	uint32_t value_len;

	tmp_cmd = (struct msm_ctrl_cmd *)ctrl->value;
	uptr_cmd = (void __user *)ctrl->value;
	uptr_value = (void __user *)tmp_cmd->value;
	value_len = tmp_cmd->length;

	D("%s: cmd type = %d, up1=0x%x, ulen1=%d, up2=0x%x, ulen2=%d\n",
		__func__, tmp_cmd->type, (uint32_t)uptr_cmd, cmd_len,
		(uint32_t)uptr_value, tmp_cmd->length);

	ctrl_data = kzalloc(value_len+cmd_len, GFP_KERNEL);
	if (ctrl_data == 0) {
		pr_err("%s could not allocate memory\n", __func__);
		rc = -ENOMEM;
		goto end;
	}
	tmp_cmd = (struct msm_ctrl_cmd *)ctrl_data;
	if (copy_from_user((void *)ctrl_data, uptr_cmd,
					cmd_len)) {
		pr_err("%s: copy_from_user failed.\n", __func__);
		rc = -EINVAL;
		goto end;
	}
	tmp_cmd->value = (void *)(ctrl_data+cmd_len);
	if (uptr_value && tmp_cmd->length > 0) {
		if (copy_from_user((void *)tmp_cmd->value, uptr_value,
						value_len)) {
			pr_err("%s: copy_from_user failed, size=%d\n",
				__func__, value_len);
			rc = -EINVAL;
			goto end;
		}
	} else
	tmp_cmd->value = NULL;

	memset(&ctrlcmd, 0, sizeof(ctrlcmd));  
	ctrlcmd.type = MSM_V4L2_SET_CTRL_CMD;
	ctrlcmd.length = cmd_len + value_len;
	ctrlcmd.value = (void *)ctrl_data;
	if (tmp_cmd->timeout_ms > 0)
		ctrlcmd.timeout_ms = tmp_cmd->timeout_ms;
	else
		ctrlcmd.timeout_ms = 2000;
	ctrlcmd.vnode_id = pcam->vnode_id;
	ctrlcmd.queue_idx = pcam->server_queue_idx;
	ctrlcmd.config_ident = g_server_dev.config_info.config_dev_id[pcam->server_queue_idx];
	
	rc = msm_server_control(&g_server_dev, &ctrlcmd);
	D("%s: msm_server_control rc=%d\n", __func__, rc);
	if (rc == 0) {
		if (uptr_value && tmp_cmd->length > 0 &&
			copy_to_user((void __user *)uptr_value,
				(void *)(ctrl_data+cmd_len), tmp_cmd->length)) {
			pr_err("%s: copy_to_user failed, size=%d\n",
				__func__, tmp_cmd->length);
			rc = -EINVAL;
			goto end;
		}
		tmp_cmd->value = uptr_value;
		if (copy_to_user((void __user *)uptr_cmd,
			(void *)tmp_cmd, cmd_len)) {
			pr_err("%s: copy_to_user failed in cpy, size=%d\n",
				__func__, cmd_len);
			rc = -EINVAL;
			goto end;
		}
	}
end:
	D("%s: END, type = %d, vaddr = 0x%x, vlen = %d, status = %d, rc = %d\n",
		__func__, tmp_cmd->type, (uint32_t)tmp_cmd->value,
		tmp_cmd->length, tmp_cmd->status, rc);
	kfree(ctrl_data);
	ctrl_data = NULL;
	return rc;
}

static int msm_server_s_ctrl(struct msm_cam_v4l2_device *pcam,
				 struct v4l2_control *ctrl)
{
	int rc = 0;
	struct msm_ctrl_cmd ctrlcmd;
	uint8_t ctrl_data[max_control_command_size];

	WARN_ON(ctrl == NULL);
	if (ctrl == NULL) {
		pr_err("%s Invalid control\n", __func__);
		return -EINVAL;
	}
	if (ctrl->id == MSM_V4L2_PID_CTRL_CMD)
		return msm_server_proc_ctrl_cmd(pcam, ctrl, 1);

	memset(ctrl_data, 0, sizeof(ctrl_data));

	memset(&ctrlcmd, 0, sizeof(ctrlcmd));  
	ctrlcmd.type = MSM_V4L2_SET_CTRL;
	ctrlcmd.length = sizeof(struct v4l2_control);
	ctrlcmd.value = (void *)ctrl_data;
	memcpy(ctrlcmd.value, ctrl, ctrlcmd.length);
	ctrlcmd.timeout_ms = 2000;
	ctrlcmd.vnode_id = pcam->vnode_id;
	ctrlcmd.queue_idx = pcam->server_queue_idx;
	ctrlcmd.config_ident = g_server_dev.config_info.config_dev_id[pcam->server_queue_idx];

	
	rc = msm_server_control(&g_server_dev, &ctrlcmd);

	return rc;
}

static int msm_server_g_ctrl(struct msm_cam_v4l2_device *pcam,
				 struct v4l2_control *ctrl)
{
	int rc = 0;
	struct msm_ctrl_cmd ctrlcmd;
	uint8_t ctrl_data[max_control_command_size];

	WARN_ON(ctrl == NULL);
	if (ctrl == NULL) {
		pr_err("%s Invalid control\n", __func__);
		return -EINVAL;
	}
	if (ctrl->id == MSM_V4L2_PID_CTRL_CMD)
		return msm_server_proc_ctrl_cmd(pcam, ctrl, 0);

	memset(ctrl_data, 0, sizeof(ctrl_data));

	memset(&ctrlcmd, 0, sizeof(ctrlcmd));  
	ctrlcmd.type = MSM_V4L2_GET_CTRL;
	ctrlcmd.length = sizeof(struct v4l2_control);
	ctrlcmd.value = (void *)ctrl_data;
	memcpy(ctrlcmd.value, ctrl, ctrlcmd.length);
	ctrlcmd.timeout_ms = 2000;
	ctrlcmd.vnode_id = pcam->vnode_id;
	ctrlcmd.queue_idx = pcam->server_queue_idx;
	ctrlcmd.config_ident = g_server_dev.config_info.config_dev_id[pcam->server_queue_idx];

	
	rc = msm_server_control(&g_server_dev, &ctrlcmd);

	ctrl->value = ((struct v4l2_control *)ctrlcmd.value)->value;

	return rc;
}

static int msm_server_q_ctrl(struct msm_cam_v4l2_device *pcam,
			struct v4l2_queryctrl *queryctrl)
{
	int rc = 0;
	struct msm_ctrl_cmd ctrlcmd;
	uint8_t ctrl_data[max_control_command_size];

	WARN_ON(queryctrl == NULL);
	memset(ctrl_data, 0, sizeof(ctrl_data));

	memset(&ctrlcmd, 0, sizeof(ctrlcmd));  
	ctrlcmd.type = MSM_V4L2_QUERY_CTRL;
	ctrlcmd.length = sizeof(struct v4l2_queryctrl);
	ctrlcmd.value = (void *)ctrl_data;
	memcpy(ctrlcmd.value, queryctrl, ctrlcmd.length);
	ctrlcmd.timeout_ms = 2000;
	ctrlcmd.vnode_id = pcam->vnode_id;
	ctrlcmd.config_ident = g_server_dev.config_info.config_dev_id[pcam->server_queue_idx];

	
	rc = msm_server_control(&g_server_dev, &ctrlcmd);
	D("%s: rc = %d\n", __func__, rc);

	if (rc >= 0)
		memcpy(queryctrl, ctrlcmd.value, sizeof(struct v4l2_queryctrl));

	return rc;
}

static int msm_server_get_fmt(struct msm_cam_v4l2_device *pcam,
		 int idx, struct v4l2_format *pfmt)
{
	struct v4l2_pix_format *pix = &pfmt->fmt.pix;

	pix->width        = pcam->dev_inst[idx]->vid_fmt.fmt.pix.width;
	pix->height       = pcam->dev_inst[idx]->vid_fmt.fmt.pix.height;
	pix->field        = pcam->dev_inst[idx]->vid_fmt.fmt.pix.field;
	pix->pixelformat  = pcam->dev_inst[idx]->vid_fmt.fmt.pix.pixelformat;
	pix->bytesperline = pcam->dev_inst[idx]->vid_fmt.fmt.pix.bytesperline;
	pix->colorspace   = pcam->dev_inst[idx]->vid_fmt.fmt.pix.colorspace;
	pix->sizeimage    = pix->height * pix->bytesperline;

	return 0;
}

static int msm_server_get_fmt_mplane(struct msm_cam_v4l2_device *pcam,
		 int idx, struct v4l2_format *pfmt)
{
	*pfmt = pcam->dev_inst[idx]->vid_fmt;
	return 0;
}

static int msm_server_try_fmt(struct msm_cam_v4l2_device *pcam,
				 struct v4l2_format *pfmt)
{
	int rc = 0;
	int i = 0;
	struct v4l2_pix_format *pix = &pfmt->fmt.pix;

	D("%s: 0x%x\n", __func__, pix->pixelformat);
	if (pfmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		pr_err("%s: pfmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE!\n",
							__func__);
		return -EINVAL;
	}

	
	for (i = 0; i < pcam->num_fmts; i++) {
		D("%s: usr_fmts.fourcc: 0x%x\n", __func__,
			pcam->usr_fmts[i].fourcc);
		if (pcam->usr_fmts[i].fourcc == pix->pixelformat)
			break;
	}

	if (i == pcam->num_fmts) {
		pr_err("%s: Format %x not found\n", __func__, pix->pixelformat);
		return -EINVAL;
	}
	return rc;
}

static int msm_server_try_fmt_mplane(struct msm_cam_v4l2_device *pcam,
				 struct v4l2_format *pfmt)
{
	int rc = 0;
	int i = 0;
	struct v4l2_pix_format_mplane *pix_mp = &pfmt->fmt.pix_mp;

	D("%s: 0x%x\n", __func__, pix_mp->pixelformat);
	if (pfmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		pr_err("%s: Incorrect format type %d ",
			__func__, pfmt->type);
		return -EINVAL;
	}

	
	for (i = 0; i < pcam->num_fmts; i++) {
		D("%s: usr_fmts.fourcc: 0x%x\n", __func__,
			pcam->usr_fmts[i].fourcc);
		if (pcam->usr_fmts[i].fourcc == pix_mp->pixelformat)
			break;
	}

	if (i == pcam->num_fmts) {
		pr_err("%s: Format %x not found\n",
			__func__, pix_mp->pixelformat);
		return -EINVAL;
	}
	return rc;
}

static int msm_camera_get_crop(struct msm_cam_v4l2_device *pcam,
				int idx, struct v4l2_crop *crop)
{
	int rc = 0;
	struct msm_ctrl_cmd ctrlcmd;

	BUG_ON(crop == NULL);

	memset(&ctrlcmd, 0, sizeof(ctrlcmd));  
	ctrlcmd.type = MSM_V4L2_GET_CROP;
	ctrlcmd.length = sizeof(struct v4l2_crop);
	ctrlcmd.value = (void *)crop;
	ctrlcmd.timeout_ms = 2000;
	ctrlcmd.vnode_id = pcam->vnode_id;
	ctrlcmd.queue_idx = pcam->server_queue_idx;
	ctrlcmd.stream_type = pcam->dev_inst[idx]->image_mode;
	ctrlcmd.config_ident = g_server_dev.config_info.config_dev_id[pcam->server_queue_idx];

	
	rc = msm_server_control(&g_server_dev, &ctrlcmd);
	D("%s: rc = %d\n", __func__, rc);

	return rc;
}

static int msm_camera_v4l2_querycap(struct file *f, void *pctx,
				struct v4l2_capability *pcaps)
{

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	
	
	pcaps->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	return 0;
}

static int msm_camera_v4l2_queryctrl(struct file *f, void *pctx,
				struct v4l2_queryctrl *pqctrl)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam->vid_lock);
	rc = msm_server_q_ctrl(pcam, pqctrl);
	mutex_unlock(&pcam->vid_lock);
	return rc;
}

static int msm_camera_v4l2_g_ctrl(struct file *f, void *pctx,
					struct v4l2_control *c)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam->vid_lock);
	rc = msm_server_g_ctrl(pcam, c);
	mutex_unlock(&pcam->vid_lock);

	return rc;
}

static int msm_camera_v4l2_s_ctrl(struct file *f, void *pctx,
					struct v4l2_control *ctrl)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s\n", __func__);

	WARN_ON(pctx != f->private_data);
	mutex_lock(&pcam->vid_lock);
	switch (ctrl->id) {
	case MSM_V4L2_PID_MMAP_INST:
		D("%s: mmap_inst=(0x%p, %d)\n",
			 __func__, pcam_inst, pcam_inst->my_index);
		pcam_inst->is_mem_map_inst = 1;
		break;
	case MSM_V4L2_PID_MMAP_ENTRY:
		if (copy_from_user(&pcam_inst->mem_map,
			(void *)ctrl->value,
			sizeof(struct msm_mem_map_info))) {
			rc = -EFAULT;
		} else
			D("%s:mmap entry:cookie=0x%x,mem_type=%d,len=%d\n",
				__func__, pcam_inst->mem_map.cookie,
				pcam_inst->mem_map.mem_type,
				pcam_inst->mem_map.length);
		break;
	default:
		if (ctrl->id == MSM_V4L2_PID_CAM_MODE)
			pcam->op_mode = ctrl->value;
		rc = msm_server_s_ctrl(pcam, ctrl);
		break;
	}
	mutex_unlock(&pcam->vid_lock);

	return rc;
}

static int msm_camera_v4l2_reqbufs(struct file *f, void *pctx,
				struct v4l2_requestbuffers *pb)
{
	int rc = 0, i, j;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);
	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam_inst->inst_lock);

	rc = vb2_reqbufs(&pcam_inst->vid_bufq, pb);
	if (rc < 0) {
		pr_err("%s reqbufs failed %d ", __func__, rc);
		mutex_unlock(&pcam_inst->inst_lock);
		return rc;
	}
	if (!pb->count) {
		
		pr_info("%s Inst %p freeing buffer offsets array",
			__func__, pcam_inst);
		for (j = 0 ; j < pcam_inst->buf_count ; j++) {
			kfree(pcam_inst->buf_offset[j]);
			pcam_inst->buf_offset[j] = NULL;
		}
		kfree(pcam_inst->buf_offset);
		pcam_inst->buf_offset = NULL;
		if (pcam_inst->vbqueue_initialized) {
			vb2_queue_release(&pcam_inst->vid_bufq);
			pcam_inst->vbqueue_initialized = 0;
		}
	} else {
		pr_info("%s Inst %p Allocating buf_offset array",
			__func__, pcam_inst);
		
		pcam_inst->buf_offset = (struct msm_cam_buf_offset **)
			kzalloc(pb->count * sizeof(struct msm_cam_buf_offset *),
							GFP_KERNEL);
		if (!pcam_inst->buf_offset) {
			pr_err("%s out of memory ", __func__);
			mutex_unlock(&pcam_inst->inst_lock);
			return -ENOMEM;
		}
		for (i = 0; i < pb->count; i++) {
			pcam_inst->buf_offset[i] =
				kzalloc(sizeof(struct msm_cam_buf_offset) *
				pcam_inst->plane_info.num_planes, GFP_KERNEL);
			if (!pcam_inst->buf_offset[i]) {
				pr_err("%s out of memory ", __func__);
				for (j = i-1 ; j >= 0; j--) {
					kfree(pcam_inst->buf_offset[j]);
					pcam_inst->buf_offset[j] = NULL;
				}
				kfree(pcam_inst->buf_offset);
				pcam_inst->buf_offset = NULL;
				mutex_unlock(&pcam_inst->inst_lock);
				return -ENOMEM;
			}
		}
	}
	pcam_inst->buf_count = pb->count;
	mutex_unlock(&pcam_inst->inst_lock);
	return rc;
}

static int msm_camera_v4l2_querybuf(struct file *f, void *pctx,
					struct v4l2_buffer *pb)
{
	
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	int rc = 0;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);
	mutex_lock(&pcam_inst->inst_lock);
	rc = vb2_querybuf(&pcam_inst->vid_bufq, pb);
	mutex_unlock(&pcam_inst->inst_lock);
	return rc;
}

static int msm_camera_v4l2_qbuf(struct file *f, void *pctx,
					struct v4l2_buffer *pb)
{
	int rc = 0, i = 0;
	
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s Inst=%p, mode=%d, idx=%d\n", __func__, pcam_inst,
		pcam_inst->image_mode, pb->index);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam_inst->inst_lock);

	if (!pcam_inst->buf_offset) {
		pr_err("%s Buffer is already released. Returning. ", __func__);
		mutex_unlock(&pcam_inst->inst_lock);
		return -EINVAL;
	}

	if (pb->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		
		if (pb->m.planes == NULL) {
			pr_err("%s Planes array is null ", __func__);
			mutex_unlock(&pcam_inst->inst_lock);
			return -EINVAL;
		}
		for (i = 0; i < pcam_inst->plane_info.num_planes; i++) {
			D("%s stored offsets for plane %d as"
				"addr offset %d, data offset %d",
				__func__, i, pb->m.planes[i].reserved[0],
				pb->m.planes[i].data_offset);
			pcam_inst->buf_offset[pb->index][i].data_offset =
				pb->m.planes[i].data_offset;
			pcam_inst->buf_offset[pb->index][i].addr_offset =
				pb->m.planes[i].reserved[0];
		}
	} else {
		D("%s stored reserved info %d", __func__, pb->reserved);
		pcam_inst->buf_offset[pb->index][0].addr_offset = pb->reserved;
	}

	rc = vb2_qbuf(&pcam_inst->vid_bufq, pb);
	D("%s, videobuf_qbuf mode %d and idx %d returns %d\n", __func__,
		pcam_inst->image_mode, pb->index, rc);
	mutex_unlock(&pcam_inst->inst_lock);
	return rc;
}

static int msm_camera_v4l2_dqbuf(struct file *f, void *pctx,
					struct v4l2_buffer *pb)
{
	int rc = 0;
	
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);
	mutex_lock(&pcam_inst->inst_lock);
	if (pcam_inst->streamon == 0) {
		mutex_unlock(&pcam_inst->inst_lock);
		return -EACCES;
	}

	rc = vb2_dqbuf(&pcam_inst->vid_bufq, pb,  f->f_flags & O_NONBLOCK);
	D("%s, videobuf_dqbuf returns %d\n", __func__, rc);
	mutex_unlock(&pcam_inst->inst_lock);

	return rc;
}

static int msm_camera_v4l2_streamon(struct file *f, void *pctx,
					enum v4l2_buf_type buf_type)
{
	int rc = 0;
	
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s Inst %p\n", __func__, pcam_inst);
	WARN_ON(pctx != f->private_data);
	mutex_lock(&pcam->vid_lock);
	mutex_lock(&pcam_inst->inst_lock);

	if ((buf_type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) &&
		(buf_type != V4L2_BUF_TYPE_VIDEO_CAPTURE)) {
		pr_err("%s Invalid buffer type ", __func__);
		mutex_unlock(&pcam_inst->inst_lock);
		mutex_unlock(&pcam->vid_lock);
		return -EINVAL;
	}

	D("%s Calling videobuf_streamon", __func__);
	
	rc = vb2_streamon(&pcam_inst->vid_bufq, buf_type);
	D("%s, videobuf_streamon returns %d\n", __func__, rc);

	
	pcam_inst->streamon = 1;
	rc = msm_server_streamon(pcam, pcam_inst->my_index);
	mutex_unlock(&pcam_inst->inst_lock);
	mutex_unlock(&pcam->vid_lock);
	D("%s rc = %d\n", __func__, rc);
	return rc;
}

static int msm_camera_v4l2_streamoff(struct file *f, void *pctx,
					enum v4l2_buf_type buf_type)
{
	int rc = 0;
	
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s Inst %p\n", __func__, pcam_inst);
	WARN_ON(pctx != f->private_data);

	if ((buf_type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) &&
		(buf_type != V4L2_BUF_TYPE_VIDEO_CAPTURE)) {
		pr_err("%s Invalid buffer type ", __func__);
		return -EINVAL;
	}

	mutex_lock(&pcam->vid_lock);
	mutex_lock(&pcam_inst->inst_lock);
	pcam_inst->streamon = 0;
	if (g_server_dev.use_count > 0)
		rc = msm_server_streamoff(pcam, pcam_inst->my_index);
	if (rc < 0)
		pr_err("%s: hw failed to stop streaming\n", __func__);

	
	rc = vb2_streamoff(&pcam_inst->vid_bufq, buf_type);
	D("%s, videobuf_streamoff returns %d\n", __func__, rc);
	mutex_unlock(&pcam_inst->inst_lock);
	mutex_unlock(&pcam->vid_lock);

	return rc;
}

static int msm_camera_v4l2_enum_fmt_cap(struct file *f, void *pctx,
					struct v4l2_fmtdesc *pfmtdesc)
{
	
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	const struct msm_isp_color_fmt *isp_fmt;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);
	if ((pfmtdesc->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) &&
		(pfmtdesc->type != V4L2_BUF_TYPE_VIDEO_CAPTURE))
		return -EINVAL;

	if (pfmtdesc->index >= pcam->num_fmts)
		return -EINVAL;

	isp_fmt = &pcam->usr_fmts[pfmtdesc->index];

	if (isp_fmt->name)
		strlcpy(pfmtdesc->description, isp_fmt->name,
						sizeof(pfmtdesc->description));

	pfmtdesc->pixelformat = isp_fmt->fourcc;

	D("%s: [%d] 0x%x, %s\n", __func__, pfmtdesc->index,
		isp_fmt->fourcc, isp_fmt->name);
	return 0;
}

static int msm_camera_v4l2_g_fmt_cap(struct file *f,
		void *pctx, struct v4l2_format *pfmt)
{
	int rc = 0;
	
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	if (pfmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	rc = msm_server_get_fmt(pcam, pcam_inst->my_index, pfmt);
	D("%s: current_fmt->fourcc: 0x%08x, rc = %d\n", __func__,
				pfmt->fmt.pix.pixelformat, rc);
	return rc;
}

static int msm_camera_v4l2_g_fmt_cap_mplane(struct file *f,
		void *pctx, struct v4l2_format *pfmt)
{
	int rc = 0;
	
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	if (pfmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	rc = msm_server_get_fmt_mplane(pcam, pcam_inst->my_index, pfmt);
	D("%s: current_fmt->fourcc: 0x%08x, rc = %d\n", __func__,
					pfmt->fmt.pix_mp.pixelformat, rc);
	return rc;
}

static int msm_camera_v4l2_try_fmt_cap(struct file *f, void *pctx,
					struct v4l2_format *pfmt)
{
	int rc = 0;
	
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);
	mutex_lock(&pcam->vid_lock);

	rc = msm_server_try_fmt(pcam, pfmt);
	if (rc)
		pr_err("Format %x not found, rc = %d\n",
				pfmt->fmt.pix.pixelformat, rc);
	mutex_unlock(&pcam->vid_lock);

	return rc;
}

static int msm_camera_v4l2_try_fmt_cap_mplane(struct file *f, void *pctx,
					struct v4l2_format *pfmt)
{
	int rc = 0;
	
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);
	mutex_lock(&pcam->vid_lock);

	rc = msm_server_try_fmt_mplane(pcam, pfmt);
	if (rc)
		pr_err("Format %x not found, rc = %d\n",
				pfmt->fmt.pix_mp.pixelformat, rc);
	mutex_unlock(&pcam->vid_lock);
	return rc;
}

static int msm_camera_v4l2_s_fmt_cap(struct file *f, void *pctx,
					struct v4l2_format *pfmt)
{
	int rc;
	
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_media_controller *pmctl;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s\n", __func__);
	D("%s, inst=0x%x,idx=%d,priv = 0x%p\n",
		__func__, (u32)pcam_inst, pcam_inst->my_index,
		(void *)pfmt->fmt.pix.priv);
	WARN_ON(pctx != f->private_data);

	pmctl = msm_camera_get_mctl(pcam->mctl_handle);
	if (pmctl == NULL)
		return -EINVAL;

	if (!pcam_inst->vbqueue_initialized) {
		pmctl->mctl_vbqueue_init(pcam_inst, &pcam_inst->vid_bufq,
					V4L2_BUF_TYPE_VIDEO_CAPTURE);
		pcam_inst->vbqueue_initialized = 1;
	}

	mutex_lock(&pcam->vid_lock);

	rc = msm_server_set_fmt(pcam, pcam_inst->my_index, pfmt);
	if (rc < 0) {
		pr_err("%s: msm_server_set_fmt Error: %d\n",
				__func__, rc);
	}
	mutex_unlock(&pcam->vid_lock);

	return rc;
}

static int msm_camera_v4l2_s_fmt_cap_mplane(struct file *f, void *pctx,
				struct v4l2_format *pfmt)
{
	int rc;
	struct msm_cam_v4l2_device *pcam = video_drvdata(f);
	struct msm_cam_media_controller *pmctl;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
			struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s Inst %p\n", __func__, pcam_inst);
	WARN_ON(pctx != f->private_data);

	pmctl = msm_camera_get_mctl(pcam->mctl_handle);
	if (pmctl == NULL)
		return -EINVAL;

	if (!pcam_inst->vbqueue_initialized) {
		pmctl->mctl_vbqueue_init(pcam_inst, &pcam_inst->vid_bufq,
					V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
		pcam_inst->vbqueue_initialized = 1;
	}

	mutex_lock(&pcam->vid_lock);
	rc = msm_server_set_fmt_mplane(pcam, pcam_inst->my_index, pfmt);
	mutex_unlock(&pcam->vid_lock);

	return rc;
}
static int msm_camera_v4l2_g_jpegcomp(struct file *f, void *pctx,
				struct v4l2_jpegcompression *pcomp)
{
	int rc = -EINVAL;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	return rc;
}

static int msm_camera_v4l2_s_jpegcomp(struct file *f, void *pctx,
				struct v4l2_jpegcompression *pcomp)
{
	int rc = -EINVAL;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	return rc;
}


static int msm_camera_v4l2_g_crop(struct file *f, void *pctx,
					struct v4l2_crop *crop)
{
	int rc = -EINVAL;
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;

	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	mutex_lock(&pcam->vid_lock);
	rc = msm_camera_get_crop(pcam, pcam_inst->my_index, crop);
	mutex_unlock(&pcam->vid_lock);
	return rc;
}

static int msm_camera_v4l2_s_crop(struct file *f, void *pctx,
					struct v4l2_crop *a)
{
	int rc = -EINVAL;

	D("%s\n", __func__);
	WARN_ON(pctx != f->private_data);

	return rc;
}

static int msm_camera_v4l2_g_parm(struct file *f, void *pctx,
				struct v4l2_streamparm *a)
{
	int rc = -EINVAL;
	return rc;
}
static int msm_vidbuf_get_path(u32 extendedmode)
{
	switch (extendedmode) {
	case MSM_V4L2_EXT_CAPTURE_MODE_THUMBNAIL:
		return OUTPUT_TYPE_T;
	case MSM_V4L2_EXT_CAPTURE_MODE_MAIN:
		return OUTPUT_TYPE_S;
	case MSM_V4L2_EXT_CAPTURE_MODE_VIDEO:
		return OUTPUT_TYPE_V;
	case MSM_V4L2_EXT_CAPTURE_MODE_RDI:
		return OUTPUT_TYPE_R;
	case MSM_V4L2_EXT_CAPTURE_MODE_RDI1:
		return OUTPUT_TYPE_R1;
	case MSM_V4L2_EXT_CAPTURE_MODE_DEFAULT:
	case MSM_V4L2_EXT_CAPTURE_MODE_PREVIEW:
	default:
		return OUTPUT_TYPE_P;
	}
}

static int msm_camera_v4l2_s_parm(struct file *f, void *pctx,
				struct v4l2_streamparm *a)
{
	int rc = 0;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);
	pcam_inst->image_mode = a->parm.capture.extendedmode;
	pcam_inst->pcam->dev_inst_map[pcam_inst->image_mode] = pcam_inst;
	pcam_inst->path = msm_vidbuf_get_path(pcam_inst->image_mode);
	pr_info("%spath=%d,rc=%d\n", __func__,
		pcam_inst->path, rc);
	return rc;
}

static int msm_camera_v4l2_subscribe_event(struct v4l2_fh *fh,
			struct v4l2_event_subscription *sub)
{
	int rc = 0;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst =
		(struct msm_cam_v4l2_dev_inst *)container_of(fh,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s:fh = 0x%x, type = 0x%x\n", __func__, (u32)fh, sub->type);
	if (pcam_inst->my_index != 0)
		return -EINVAL;
	if (sub->type == V4L2_EVENT_ALL)
		sub->type = V4L2_EVENT_PRIVATE_START+MSM_CAM_APP_NOTIFY_EVENT;
	rc = v4l2_event_subscribe(fh, sub);
	if (rc < 0)
		D("%s: failed for evtType = 0x%x, rc = %d\n",
						__func__, sub->type, rc);
	return rc;
}

static int msm_camera_v4l2_unsubscribe_event(struct v4l2_fh *fh,
			struct v4l2_event_subscription *sub)
{
	int rc = 0;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst =
		(struct msm_cam_v4l2_dev_inst *)container_of(fh,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("%s: fh = 0x%x\n", __func__, (u32)fh);
	if (pcam_inst->my_index != 0)
		return -EINVAL;

	rc = v4l2_event_unsubscribe(fh, sub);
	D("%s: rc = %d\n", __func__, rc);
	return rc;
}

static int msm_server_v4l2_subscribe_event(struct v4l2_fh *fh,
			struct v4l2_event_subscription *sub)
{
	int rc = 0;

	D("%s: fh = 0x%x, type = 0x%x", __func__, (u32)fh, sub->type);
	if (sub->type == V4L2_EVENT_ALL) {
		
		sub->type = V4L2_EVENT_PRIVATE_START + MSM_CAM_RESP_CTRL;
		D("sub->type start = 0x%x\n", sub->type);
		do {
			rc = v4l2_event_subscribe(fh, sub);
			if (rc < 0) {
				D("%s: failed for evtType = 0x%x, rc = %d\n",
						__func__, sub->type, rc);
			
			sub->type = V4L2_EVENT_ALL;
			v4l2_event_unsubscribe(fh, sub);
			return rc;
			} else
				D("%s: subscribed evtType = 0x%x, rc = %d\n",
						__func__, sub->type, rc);
			sub->type++;
			D("sub->type while = 0x%x\n", sub->type);
		} while (sub->type !=
			V4L2_EVENT_PRIVATE_START + MSM_SVR_RESP_MAX);
	} else {
		D("sub->type not V4L2_EVENT_ALL = 0x%x\n", sub->type);
		rc = v4l2_event_subscribe(fh, sub);
		if (rc < 0)
			D("%s: failed for evtType = 0x%x, rc = %d\n",
						__func__, sub->type, rc);
	}

	D("%s: rc = %d\n", __func__, rc);
	return rc;
}

static int msm_server_v4l2_unsubscribe_event(struct v4l2_fh *fh,
			struct v4l2_event_subscription *sub)
{
	int rc = 0;

	D("%s: fh = 0x%x\n", __func__, (u32)fh);
	rc = v4l2_event_unsubscribe(fh, sub);
	D("%s: rc = %d\n", __func__, rc);
	return rc;
}

static const struct v4l2_ioctl_ops g_msm_ioctl_ops = {
	.vidioc_querycap = msm_camera_v4l2_querycap,

	.vidioc_s_crop = msm_camera_v4l2_s_crop,
	.vidioc_g_crop = msm_camera_v4l2_g_crop,

	.vidioc_queryctrl = msm_camera_v4l2_queryctrl,
	.vidioc_g_ctrl = msm_camera_v4l2_g_ctrl,
	.vidioc_s_ctrl = msm_camera_v4l2_s_ctrl,

	.vidioc_reqbufs = msm_camera_v4l2_reqbufs,
	.vidioc_querybuf = msm_camera_v4l2_querybuf,
	.vidioc_qbuf = msm_camera_v4l2_qbuf,
	.vidioc_dqbuf = msm_camera_v4l2_dqbuf,

	.vidioc_streamon = msm_camera_v4l2_streamon,
	.vidioc_streamoff = msm_camera_v4l2_streamoff,

	
	.vidioc_enum_fmt_vid_cap = msm_camera_v4l2_enum_fmt_cap,
	.vidioc_enum_fmt_vid_cap_mplane = msm_camera_v4l2_enum_fmt_cap,
	.vidioc_try_fmt_vid_cap = msm_camera_v4l2_try_fmt_cap,
	.vidioc_try_fmt_vid_cap_mplane = msm_camera_v4l2_try_fmt_cap_mplane,
	.vidioc_g_fmt_vid_cap = msm_camera_v4l2_g_fmt_cap,
	.vidioc_g_fmt_vid_cap_mplane = msm_camera_v4l2_g_fmt_cap_mplane,
	.vidioc_s_fmt_vid_cap = msm_camera_v4l2_s_fmt_cap,
	.vidioc_s_fmt_vid_cap_mplane = msm_camera_v4l2_s_fmt_cap_mplane,

	.vidioc_g_jpegcomp = msm_camera_v4l2_g_jpegcomp,
	.vidioc_s_jpegcomp = msm_camera_v4l2_s_jpegcomp,

	
	.vidioc_g_parm =  msm_camera_v4l2_g_parm,
	.vidioc_s_parm =  msm_camera_v4l2_s_parm,

	
	.vidioc_subscribe_event = msm_camera_v4l2_subscribe_event,
	.vidioc_unsubscribe_event = msm_camera_v4l2_unsubscribe_event,
};

static int msm_cam_server_open_session(struct msm_cam_server_dev *ps,
	struct msm_cam_v4l2_device *pcam)
{
	int rc = 0;
	struct msm_cam_media_controller *pmctl;
	D("%s\n", __func__);

	if (!ps || !pcam) {
		pr_err("%s NULL pointer passed in!\n", __func__);
		return rc;
	}

	if (atomic_read(&ps->number_pcam_active) > 1) {
		pr_err("%s Cannot have more than two active camera %d\n",
			__func__, atomic_read(&ps->number_pcam_active));
		return -EINVAL;
	}
	
	ps->pcam_active[pcam->server_queue_idx] = pcam;
	atomic_inc(&ps->number_pcam_active);

	D("config pcam = 0x%p\n", ps->pcam_active[pcam->server_queue_idx]);

	
	msm_mctl_init(pcam);

	pmctl = msm_camera_get_mctl(pcam->mctl_handle);
	if(!pmctl) return -EINVAL;
	pmctl->axi_sdev = ps->axi_device[0];
	pmctl->isp_sdev = ps->isp_subdev[0];

#ifdef CONFIG_PERFLOCK
	pmctl->cam_perf_lock = &ps->cam_perf_lock;
#endif

	return rc;

}

static int msm_cam_server_close_session(struct msm_cam_server_dev *ps,
	struct msm_cam_v4l2_device *pcam)
{
	int rc = 0;
	D("%s\n", __func__);

	if (!ps || !pcam) {
		D("%s NULL pointer passed in!\n", __func__);
		return rc;
	}


	atomic_dec(&ps->number_pcam_active);
	ps->pcam_active[pcam->server_queue_idx] = NULL;
	msm_mctl_free(pcam);

	return rc;
}


int msm_server_open_client(int *p_qidx)
{
	int rc = 0;
	int server_q_idx = 0;
	struct msm_cam_server_queue *queue = NULL;

	mutex_lock(&g_server_dev.server_lock);
	server_q_idx = msm_find_free_queue();
	if (server_q_idx < 0) {
		mutex_unlock(&g_server_dev.server_lock);
		return server_q_idx;
	}

	*p_qidx = server_q_idx;
	queue = &g_server_dev.server_queue[server_q_idx];
	queue->ctrl_data = kzalloc(sizeof(uint8_t) *
		max_control_command_size, GFP_KERNEL);
	msm_queue_init(&queue->ctrl_q, "control");
	msm_queue_init(&queue->eventData_q, "eventdata");
	queue->queue_active = 1;
	mutex_unlock(&g_server_dev.server_lock);
	return rc;
}

int msm_server_send_ctrl(struct msm_ctrl_cmd *out,
	int ctrl_id)
{
	int rc = 0;
	void *value;
	struct msm_queue_cmd *rcmd;
	struct msm_queue_cmd *event_qcmd;
	struct msm_ctrl_cmd *ctrlcmd;
	struct msm_cam_server_dev *server_dev = &g_server_dev;
	struct msm_device_queue *queue;

	struct v4l2_event v4l2_evt;
	struct msm_isp_event_ctrl *isp_event;

	if(out->queue_idx < 0 || out->queue_idx >= MAX_NUM_ACTIVE_CAMERA) {
		pr_err("%s: Invalid index %d\n", __func__, out->queue_idx);
		return -EINVAL;
	}
	queue = &server_dev->server_queue[out->queue_idx].ctrl_q;

	isp_event = kzalloc(sizeof(struct msm_isp_event_ctrl), GFP_KERNEL);
	if (!isp_event) {
		pr_err("%s Insufficient memory. return", __func__);
		return -ENOMEM;
	}
	event_qcmd = kzalloc(sizeof(struct msm_queue_cmd), GFP_KERNEL);
	if (!event_qcmd) {
		pr_err("%s Insufficient memory. return", __func__);
		kfree(isp_event);
		return -ENOMEM;
	}

	D("%s\n", __func__);
	mutex_lock(&server_dev->server_queue_lock);
	if (++server_dev->server_evt_id == 0)
		server_dev->server_evt_id++;

	D("%s qid %d evtid %d\n", __func__, out->queue_idx,
		server_dev->server_evt_id);
	server_dev->server_queue[out->queue_idx].evt_id =
		server_dev->server_evt_id;
	v4l2_evt.type = V4L2_EVENT_PRIVATE_START + ctrl_id;
	v4l2_evt.u.data[0] = out->queue_idx;
	
	isp_event->resptype = MSM_CAM_RESP_V4L2;
	isp_event->isp_data.ctrl = *out;
	isp_event->isp_data.ctrl.evt_id = server_dev->server_evt_id;

	atomic_set(&event_qcmd->on_heap, 1);
	event_qcmd->command = isp_event;

	msm_enqueue(&server_dev->server_queue[out->queue_idx].eventData_q,
				&event_qcmd->list_eventdata);

	v4l2_event_queue(server_dev->server_command_queue.pvdev,
					  &v4l2_evt);
	D("%s v4l2_event_queue: type = 0x%x\n", __func__, v4l2_evt.type);
	mutex_unlock(&server_dev->server_queue_lock);

	
	D("Waiting for config status\n");
	rc = wait_event_timeout(queue->wait,
		!list_empty_careful(&queue->list),
		msecs_to_jiffies(out->timeout_ms));
	D("Waiting is over for config status\n");
	if (list_empty_careful(&queue->list)) {
		if (!rc)
			rc = -ETIMEDOUT;
		if (rc < 0) {
			kfree(isp_event);
			pr_err("%s: wait_event error %d\n", __func__, rc);
			return rc;
		}
	}

	rcmd = msm_dequeue(queue, list_control);
	BUG_ON(!rcmd);
	if(!rcmd) return -EINVAL;
	D("%s Finished servicing ioctl\n", __func__);

	ctrlcmd = (struct msm_ctrl_cmd *)(rcmd->command);
	value = out->value;
	if (ctrlcmd->length > 0)
		memcpy(value, ctrlcmd->value, ctrlcmd->length);

	memcpy(out, ctrlcmd, sizeof(struct msm_ctrl_cmd));
	out->value = value;

	kfree(ctrlcmd);

	free_qcmd(rcmd);
	kfree(isp_event);
	D("%s: rc %d\n", __func__, rc);
	
	if (rc >= 0) {
		
		if (out->status == 0)
			rc = -1;
		else if (out->status == 1 || out->status == 4)
			rc = 0;
		else
			rc = -EINVAL;
	}
	return rc;
}

int msm_server_close_client(int idx)
{
	int rc = 0;
	struct msm_cam_server_queue *queue = NULL;
	mutex_lock(&g_server_dev.server_lock);
	queue = &g_server_dev.server_queue[idx];
	queue->queue_active = 0;
	kfree(queue->ctrl_data);
	queue->ctrl_data = NULL;
	msm_queue_drain(&queue->ctrl_q, list_control);
	msm_drain_eventq(&queue->eventData_q);
	mutex_unlock(&g_server_dev.server_lock);
	return rc;
}

static int msm_open(struct file *f)
{
	int i;
	int rc = -EINVAL;
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	int ion_client_created = 0;
#endif
	int server_q_idx = 0;

	
	
	struct msm_cam_v4l2_device *pcam  = video_drvdata(f);
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	struct msm_cam_media_controller *pmctl = NULL;
	struct msm_cam_server_queue *queue = NULL;
	D("%s\n", __func__);

	if (!pcam) {
		pr_err("%s NULL pointer passed in!\n", __func__);
		return rc;
	}
	if (!g_server_dev.use_count) {
		pr_err("%s: error, daemon not yet started.", __func__);
		return -EINVAL;
	}
	mutex_lock(&pcam->vid_lock);
	for (i = 0; i < MSM_DEV_INST_MAX; i++) {
		if (pcam->dev_inst[i] == NULL)
			break;
	}

	server_q_idx = msm_find_free_queue();
	if (server_q_idx < 0)
		return server_q_idx;

	
	if (i == MSM_DEV_INST_MAX) {
		mutex_unlock(&pcam->vid_lock);
		return rc;
	}
	pcam_inst = kzalloc(sizeof(struct msm_cam_v4l2_dev_inst), GFP_KERNEL);
	if (!pcam_inst) {
		mutex_unlock(&pcam->vid_lock);
		return rc;
	}
	mutex_init(&pcam_inst->inst_lock);
	pcam_inst->sensor_pxlcode = pcam->usr_fmts[0].pxlcode;
	pcam_inst->my_index = i;
	pcam_inst->pcam = pcam;
	pcam->dev_inst[i] = pcam_inst;

	D("%s index %d nodeid %d count %d\n", __func__,
			pcam_inst->my_index,
			pcam->vnode_id, pcam->use_count);
	pcam->use_count++;
	D("%s use_count %d\n", __func__, pcam->use_count);
	if (pcam->use_count == 1) {
		int ges_evt = MSM_V4L2_GES_CAM_OPEN;

		pr_info("%s use_count %d\n", __func__, pcam->use_count); 

		
		if (atomic_read(&g_server_dev.number_pcam_active) > 1) {
			pr_err("%s: Cannot have more than two active camera\n", __func__);
			rc = -EINVAL;
			goto more_than_one_active_cam_error;
		}
		
		pcam->server_queue_idx = server_q_idx;
		queue = &g_server_dev.server_queue[server_q_idx];
		queue->ctrl_data = kzalloc(sizeof(uint8_t) *
			max_control_command_size, GFP_KERNEL);
		msm_queue_init(&queue->ctrl_q, "control");
		msm_queue_init(&queue->eventData_q, "eventdata");
		queue->queue_active = 1;

		msm_cam_server_subdev_notify(g_server_dev.gesture_device,
			NOTIFY_GESTURE_CAM_EVT, &ges_evt);


		rc = msm_cam_server_open_session(&g_server_dev, pcam);
		if (rc < 0) {
			pr_err("%s: cam_server_open_session failed %d\n",
			__func__, rc);
			goto msm_cam_server_open_session_failed;
		}

		pmctl = msm_camera_get_mctl(pcam->mctl_handle);
		if (!pmctl)  {
			rc = -EINVAL;
			goto msm_cam_server_open_session_failed;
		}

#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
		if (pmctl->client) {
			pr_info("%s: pmctl->client(%p) not null\n", __func__, (void*)(pmctl->client));
			ion_client_destroy(pmctl->client);
			pmctl->client = NULL;
		}
		{
			char ion_debug_name[64];
			snprintf(ion_debug_name, 64, "%u", task_pid_nr(current->group_leader));
			pmctl->client = msm_ion_client_create(-1, ion_debug_name);
		}
		kref_init(&pmctl->refcount);
		ion_client_created = 1;
#endif
		
		if (!pmctl->mctl_open) {
			D("%s: media contoller is not inited\n", __func__);
			rc = -ENODEV;
			goto mctl_open_failed;
		}

		
		D("%s: call mctl_open\n", __func__);

		cam_on_check_vcm(); 

		rc = pmctl->mctl_open(pmctl, MSM_APPS_ID_V4L2);

		if (rc < 0) {
			pr_err("%s: HW open failed rc = 0x%x\n",  __func__, rc);
			goto mctl_open_failed;
		}
		pmctl->pcam_ptr = pcam;

		rc = msm_setup_v4l2_event_queue(&pcam_inst->eventHandle,
			pcam->pvdev);
		if (rc < 0) {
			pr_err("%s: msm_setup_v4l2_event_queue failed %d",
				__func__, rc);
			goto mctl_event_q_setup_failed;
		}
	}
	pcam_inst->vbqueue_initialized = 0;
	rc = 0;

	f->private_data = &pcam_inst->eventHandle;

	D("f->private_data = 0x%x, pcam = 0x%x\n",
		(u32)f->private_data, (u32)pcam_inst);


	if (pcam->use_count == 1) {
		rc = msm_send_open_server(pcam);
		if (rc < 0) {	
			pr_err("%s: msm_send_open_server failed %d\n",
				__func__, rc);
			goto msm_send_open_server_failed;
		}
		if (rc == 0)   {
			pr_info("%s: serv_running[%d] = %d \n", __func__,pcam->server_queue_idx, 1);
			atomic_set(&serv_running[pcam->server_queue_idx],1);
		}

	}
	mutex_unlock(&pcam->vid_lock);
	D("%s: end\n", __func__);
	return rc;
	
msm_send_open_server_failed:
	
	pr_info("%s: rc = %d", __func__, rc);
	if (rc == -ERESTARTSYS) {
		msm_send_close_server(pcam);
	}
	
	v4l2_fh_del(&pcam_inst->eventHandle);
	v4l2_fh_exit(&pcam_inst->eventHandle);
mctl_event_q_setup_failed:
	if (pmctl->mctl_release)
		if (pmctl->mctl_release(pmctl) < 0)
			pr_err("%s: mctl_release failed\n", __func__);
mctl_open_failed:

	if (pcam->use_count == 1) {
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
		if (ion_client_created) {
			D("%s: destroy ion client", __func__);
			kref_put(&pmctl->refcount, msm_release_ion_client);
		}
#endif
		if (msm_cam_server_close_session(&g_server_dev, pcam) < 0)
			pr_err("%s: msm_cam_server_close_session failed\n",
				__func__);
	}
msm_cam_server_open_session_failed:
	if (pcam->use_count == 1) {
		if (queue) {
			queue->queue_active = 0;
			msm_drain_eventq(&queue->eventData_q);
			kfree(queue->ctrl_data);
			queue->ctrl_data = NULL;
			msm_queue_drain(&queue->ctrl_q, list_control);
			msm_drain_eventq(&queue->eventData_q);
			queue = NULL;
		}

		pcam->dev_inst[i] = NULL;
		pcam->use_count = 0;
	}
more_than_one_active_cam_error:
	pcam->dev_inst[i] = NULL;
	mutex_unlock(&pcam->vid_lock);
	kfree(pcam_inst);
	pr_err("%s: error end", __func__);
	return rc;
}

int msm_cam_server_close_mctl_session(struct msm_cam_v4l2_device *pcam)
{
	int rc = 0;
	struct msm_cam_media_controller *pmctl = NULL;

	pmctl = msm_camera_get_mctl(pcam->mctl_handle);
	if (!pmctl) {
		D("%s: invalid handle\n", __func__);
		return -ENODEV;
	}

	if (pmctl->mctl_release) {
		rc = pmctl->mctl_release(pmctl);
		if (rc < 0)
			pr_err("mctl_release fails %d\n", rc);
	}

#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	kref_put(&pmctl->refcount, msm_release_ion_client);
#endif

	rc = msm_cam_server_close_session(&g_server_dev, pcam);
	if (rc < 0)
		pr_err("msm_cam_server_close_session fails %d\n", rc);

	return rc;
}

int msm_cam_server_open_mctl_session(struct msm_cam_v4l2_device *pcam,
	int *p_active)
{
	int rc = 0, i;
	struct msm_cam_media_controller *pmctl = NULL;
	pr_info("%s: E", __func__);
	*p_active = 0;
	for (i = 0; i < MAX_NUM_ACTIVE_CAMERA; i++) {
		if (NULL != g_server_dev.pcam_active[i]) {
			pr_info("%s: Active camera present return", __func__);
			return 0;
		}
	}
	rc = msm_cam_server_open_session(&g_server_dev, pcam);
	if (rc < 0) {
		pr_err("%s: cam_server_open_session failed %d\n",
		__func__, rc);
		return rc;
	}

	pmctl = msm_camera_get_mctl(pcam->mctl_handle);
	
	if (!pmctl) {
		pr_err("%s: pmctl is NULL\n",
			 __func__);
		rc = -ENODEV;
		return rc;
	}
	
	
	if (!pmctl->mctl_open) {
		D("%s: media contoller is not inited\n",
			 __func__);
		rc = -ENODEV;
		return rc;
	}

	D("%s: call mctl_open\n", __func__);

	cam_on_check_vcm(); 

	rc = pmctl->mctl_open(pmctl, MSM_APPS_ID_V4L2);

	if (rc < 0) {
		pr_err("%s: HW open failed rc = 0x%x\n",  __func__, rc);
		return rc;
	}
	pmctl->pcam_ptr = pcam;
	*p_active = 1;	
	return rc;
}

static int msm_addr_remap(struct msm_cam_v4l2_dev_inst *pcam_inst,
				struct vm_area_struct *vma)
{
	int phyaddr;
	int retval;
	unsigned long size;
	int rc = 0;
	struct msm_cam_media_controller *mctl;

	mctl = msm_camera_get_mctl(pcam_inst->pcam->mctl_handle);
	if (!mctl) {
		pr_err("%s: invalid mctl pointer", __func__);
		return -EFAULT;
	}

	rc = msm_pmem_region_get_phy_addr(&mctl->stats_info.pmem_stats_list,
			&pcam_inst->mem_map,
			&phyaddr);
	if (rc) {
		pr_err("%s: cannot map vaddr", __func__);
		return -EFAULT;
	}
	size = vma->vm_end - vma->vm_start;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	retval = remap_pfn_range(vma, vma->vm_start,
			phyaddr >> PAGE_SHIFT,
			size, vma->vm_page_prot);
	if (retval) {
		pr_err("%s:mmap: remap failed with error %d. ",
			   __func__, retval);
		memset(&pcam_inst->mem_map, 0, sizeof(pcam_inst->mem_map));
		return -ENOMEM;
	}
	D("%s:mmap: phy_addr=0x%x: %08lx-%08lx, pgoff %08lx\n",
		   __func__, (uint32_t)phyaddr,
		   vma->vm_start, vma->vm_end, vma->vm_pgoff);
	memset(&pcam_inst->mem_map, 0, sizeof(pcam_inst->mem_map));
	return 0;
}

static int msm_mmap(struct file *f, struct vm_area_struct *vma)
{
	int rc = 0;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);

	D("mmap called, vma=0x%08lx\n", (unsigned long)vma);

	if (pcam_inst->is_mem_map_inst &&
		pcam_inst->mem_map.cookie) {
		rc = msm_addr_remap(pcam_inst, vma);
		D("%s: msm_addr_remap ret=%d\n", __func__, rc);
		return rc;
	} else
		rc = vb2_mmap(&pcam_inst->vid_bufq, vma);
	D("vma start=0x%08lx, size=%ld, ret=%d\n",
		(unsigned long)vma->vm_start,
		(unsigned long)vma->vm_end - (unsigned long)vma->vm_start,
		rc);

	return rc;
}

void msm_release_ion_client(struct kref *ref)
{
	struct msm_cam_media_controller *mctl = container_of(ref,
		struct msm_cam_media_controller, refcount);
	pr_info("%s Calling ion_client_destroy\n", __func__);

	if (mctl && (mctl->client)) {
		ion_client_destroy(mctl->client);
		mctl->client = NULL;
	}
}

static int msm_close(struct file *f)
{
	int rc = 0,i=0;
	struct msm_cam_v4l2_device *pcam;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	struct msm_cam_server_queue *queue;
	struct msm_cam_media_controller *pmctl;
	
	struct msm_sensor_ctrl_t *s_ctrl ;
	
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);
	pcam = pcam_inst->pcam;
	if (!pcam) {
		pr_err("%s NULL pointer of camera device!\n", __func__);
		return -EINVAL;
	}

	pmctl = msm_camera_get_mctl(pcam->mctl_handle);
	if (!pmctl) {
		pr_err("%s NULL mctl pointer\n", __func__);
		return -EINVAL;
	}
	
    s_ctrl  = get_sctrl(pmctl->sensor_sdev);
    if(s_ctrl && s_ctrl->sensor_first_mutex)  {
	    mutex_lock(s_ctrl->sensor_first_mutex);
	}
	

	mutex_lock(&pcam->vid_lock);
	mutex_lock(&pcam_inst->inst_lock);

	if (pcam_inst->streamon) {
	
	#if 0
		if (pmctl->mctl_release) {
			rc = pmctl->mctl_release(pmctl);
			if (rc < 0)
				pr_err("mctl_release fails %d\n", rc);
		}
		pmctl->mctl_release = NULL;
	#endif
	
		msm_cam_stop_hardware(pcam);
	}

	pcam_inst->streamon = 0;
	pcam->use_count--;
	pcam->dev_inst_map[pcam_inst->image_mode] = NULL;

	
	pr_info("%s Inst %p freeing buffer offsets array",__func__, pcam_inst);
	if (pcam_inst->buf_offset) {
		for (i = 0 ; i < pcam_inst->buf_count ; i++)
			kfree(pcam_inst->buf_offset[i]);
		kfree(pcam_inst->buf_offset);
		pcam_inst->buf_offset = NULL;
	}
	
	if (pcam_inst->vbqueue_initialized)
		vb2_queue_release(&pcam_inst->vid_bufq);
	pr_info("%s Closing down instance %p, [%d, %d]", __func__, pcam_inst, pcam->use_count , g_server_dev.use_count);
	D("%s index %d nodeid %d count %d\n", __func__, pcam_inst->my_index,
	pcam->vnode_id, pcam->use_count);

	pcam->dev_inst[pcam_inst->my_index] = NULL;
	if (pcam_inst->my_index == 0) {
		v4l2_fh_del(&pcam_inst->eventHandle);
		v4l2_fh_exit(&pcam_inst->eventHandle);
	}
	mutex_unlock(&pcam_inst->inst_lock);
	mutex_destroy(&pcam_inst->inst_lock);
	kfree(pcam_inst);
	f->private_data = NULL;

	if (pcam->use_count == 0) {
		int ges_evt = MSM_V4L2_GES_CAM_CLOSE;
		

		if (g_server_dev.use_count > 0) {
			rc = msm_send_close_server(pcam);
			if (rc < 0)
				pr_err("msm_send_close_server failed %d\n", rc);
		}
		if (pmctl->mctl_release) {
			rc = pmctl->mctl_release(pmctl);
			if (rc < 0)
				pr_err("mctl_release fails %d\n", rc);
		}
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
		kref_put(&pmctl->refcount, msm_release_ion_client);
#endif
		mutex_lock(&g_server_dev.server_queue_lock);
		queue = &g_server_dev.server_queue[pcam->server_queue_idx];
		if (queue) {
			queue->queue_active = 0;
			if(queue->ctrl_data)  kfree(queue->ctrl_data);
			queue->ctrl_data = NULL;
			msm_queue_drain(&queue->ctrl_q, list_control);
			msm_drain_eventq(&queue->eventData_q);
			
		}
		mutex_unlock(&g_server_dev.server_queue_lock);

		rc = msm_cam_server_close_session(&g_server_dev, pcam);
		if (rc < 0)
			pr_err("msm_cam_server_close_session fails %d\n", rc);

		msm_cam_server_subdev_notify(g_server_dev.gesture_device,
			NOTIFY_GESTURE_CAM_EVT, &ges_evt);
	}
	mutex_unlock(&pcam->vid_lock);
	
    if(s_ctrl && s_ctrl->sensor_first_mutex)  {
	   mutex_unlock(s_ctrl->sensor_first_mutex);
    }
	
	return rc;
}

static unsigned int msm_poll(struct file *f, struct poll_table_struct *wait)
{
	int rc = 0;
	struct msm_cam_v4l2_device *pcam;
	struct msm_cam_v4l2_dev_inst *pcam_inst;
	pcam_inst = container_of(f->private_data,
		struct msm_cam_v4l2_dev_inst, eventHandle);
	pcam = pcam_inst->pcam;
	D("%s\n", __func__);
	if (!pcam) {
		pr_err("%s NULL pointer of camera device!\n", __func__);
		return -EINVAL;
	}
	if (pcam_inst->my_index == 0) {
		poll_wait(f, &(pcam_inst->eventHandle.events->wait), wait);
		if (v4l2_event_pending(&pcam_inst->eventHandle))
			rc |= POLLPRI;
	} else {
		if (!pcam_inst->vid_bufq.streaming) {
			D("%s vid_bufq.streaming is off, inst=0x%x\n",
			__func__, (u32)pcam_inst);
			return -EINVAL;
		}
		rc |= vb2_poll(&pcam_inst->vid_bufq, f, wait);
	}
	D("%s returns, rc  = 0x%x\n", __func__, rc);
	return rc;
}

static unsigned int msm_poll_server(struct file *fp,
					struct poll_table_struct *wait)
{
	int rc = 0;

	D("%s\n", __func__);
	poll_wait(fp,
		 &g_server_dev.server_command_queue.eventHandle.events->wait,
		 wait);
	if (v4l2_event_pending(&g_server_dev.server_command_queue.eventHandle))
		rc |= POLLPRI;

	return rc;
}
static long msm_ioctl_server(struct file *file, void *fh,
		bool valid_prio, int cmd, void *arg)
{
	int rc = -EINVAL;
	struct msm_camera_v4l2_ioctl_t *ioctl_ptr = arg;
	struct msm_camera_info temp_cam_info;
	struct msm_cam_config_dev_info temp_config_info;
	struct msm_mctl_node_info temp_mctl_info;
	int i;

	D("%s: cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case MSM_CAM_V4L2_IOCTL_GET_CAMERA_INFO:
		if (copy_from_user(&temp_cam_info,
			(void __user *)ioctl_ptr->ioctl_ptr,
			sizeof(struct msm_camera_info))) {
			rc = -EINVAL;
			return rc;
		}
		for (i = 0; i < g_server_dev.camera_info.num_cameras; i++) {
			if (copy_to_user((void __user *)
				temp_cam_info.video_dev_name[i],
				g_server_dev.camera_info.video_dev_name[i],
				strnlen(
				g_server_dev.camera_info.video_dev_name[i],
				MAX_DEV_NAME_LEN))) {
				rc = -EINVAL;
				return rc;
			}
			temp_cam_info.has_3d_support[i] =
				g_server_dev.camera_info.has_3d_support[i];
			temp_cam_info.is_internal_cam[i] =
				g_server_dev.camera_info.is_internal_cam[i];
			temp_cam_info.s_mount_angle[i] =
				g_server_dev.camera_info.s_mount_angle[i];
			temp_cam_info.sensor_type[i] =
				g_server_dev.camera_info.sensor_type[i];

		}
		temp_cam_info.num_cameras =
			g_server_dev.camera_info.num_cameras;
		if (copy_to_user((void __user *)ioctl_ptr->ioctl_ptr,
				&temp_cam_info,
				sizeof(struct msm_camera_info))) {
					rc = -EINVAL;
					return rc;
		}
		rc = 0;
		break;

	case MSM_CAM_V4L2_IOCTL_GET_CONFIG_INFO:
		if (copy_from_user(&temp_config_info,
				(void __user *)ioctl_ptr->ioctl_ptr,
				sizeof(struct msm_cam_config_dev_info))) {
			rc = -EINVAL;
			return rc;
		}
		for (i = 0;
		 i < g_server_dev.config_info.num_config_nodes; i++) {
			if (copy_to_user(
			(void __user *)temp_config_info.config_dev_name[i],
			g_server_dev.config_info.config_dev_name[i],
			strlen(g_server_dev.config_info.config_dev_name[i]))) {
				rc = -EINVAL;
				return rc;
			}
		}
		temp_config_info.num_config_nodes =
			g_server_dev.config_info.num_config_nodes;
		if (copy_to_user((void __user *)ioctl_ptr->ioctl_ptr,
							  &temp_config_info,
				sizeof(struct msm_cam_config_dev_info))) {
			rc = -EINVAL;
			return rc;
		}
		rc = 0;
		break;
	case MSM_CAM_V4L2_IOCTL_GET_MCTL_INFO:
		if (copy_from_user(&temp_mctl_info,
				(void __user *)ioctl_ptr->ioctl_ptr,
				sizeof(struct msm_mctl_node_info))) {
			rc = -EINVAL;
			return rc;
		}

		for (i = 0; i < g_server_dev.mctl_node_info.num_mctl_nodes;
				i++) {
			if (copy_to_user((void __user *)
			temp_mctl_info.mctl_node_name[i],
			g_server_dev.mctl_node_info.mctl_node_name[i], strnlen(
			g_server_dev.mctl_node_info.mctl_node_name[i],
			MAX_DEV_NAME_LEN))) {
				rc = -EINVAL;
				return rc;
			}
		}
		temp_mctl_info.num_mctl_nodes =
			g_server_dev.mctl_node_info.num_mctl_nodes;
		if (copy_to_user((void __user *)ioctl_ptr->ioctl_ptr,
							  &temp_mctl_info,
				sizeof(struct msm_mctl_node_info))) {
			rc = -EINVAL;
			return rc;
		}
		rc = 0;
	break;

	case MSM_CAM_V4L2_IOCTL_CTRL_CMD_DONE:
		D("%s: MSM_CAM_IOCTL_CTRL_CMD_DONE\n", __func__);
		rc = msm_ctrl_cmd_done(arg);
		break;

	case MSM_CAM_V4L2_IOCTL_GET_EVENT_PAYLOAD: {
		struct msm_queue_cmd *event_cmd;
		struct msm_isp_event_ctrl u_isp_event;
		struct msm_isp_event_ctrl *k_isp_event;

		struct msm_device_queue *queue;
		void __user *u_ctrl_value = NULL;
		if (copy_from_user(&u_isp_event,
			(void __user *)ioctl_ptr->ioctl_ptr,
			sizeof(struct msm_isp_event_ctrl))) {
			rc = -EINVAL;
			return rc;

		}
		mutex_lock(&g_server_dev.server_queue_lock);
		if(u_isp_event.isp_data.ctrl.queue_idx < 0 ||
			u_isp_event.isp_data.ctrl.queue_idx >= MAX_NUM_ACTIVE_CAMERA) {
			pr_err("%s: Invalid index %d\n", __func__,
				u_isp_event.isp_data.ctrl.queue_idx);
			rc = -EINVAL;
			return rc;
		}

		if (!g_server_dev.server_queue
			[u_isp_event.isp_data.ctrl.queue_idx].queue_active) {
			pr_err("%s: Invalid queue\n", __func__);
			mutex_unlock(&g_server_dev.server_queue_lock);
			rc = -EINVAL;
			return rc;
		}		
		queue = &g_server_dev.server_queue
			[u_isp_event.isp_data.ctrl.queue_idx].eventData_q;
		event_cmd = msm_dequeue(queue, list_eventdata);
		if (!event_cmd) {
			pr_err("%s: No event payload\n", __func__);
			rc = -EINVAL;
			mutex_unlock(&g_server_dev.server_queue_lock);
			return rc;

		}
		k_isp_event = (struct msm_isp_event_ctrl *)
				event_cmd->command;
		free_qcmd(event_cmd);

		
		u_ctrl_value = u_isp_event.isp_data.ctrl.value;

		
		u_isp_event = *k_isp_event;

		u_isp_event.isp_data.ctrl.value = u_ctrl_value;

		
		if (k_isp_event->isp_data.ctrl.length > 0 &&
			k_isp_event->isp_data.ctrl.value != NULL) {
			void *k_ctrl_value =
				k_isp_event->isp_data.ctrl.value;
			if (copy_to_user(u_ctrl_value, k_ctrl_value,
				 k_isp_event->isp_data.ctrl.length)) {
				kfree(k_isp_event->isp_data.ctrl.value);
				kfree(k_isp_event);
				rc = -EINVAL;
				mutex_unlock(&g_server_dev.server_queue_lock);
				break;
			}
			kfree(k_isp_event->isp_data.ctrl.value);
		}

		if (copy_to_user((void __user *)ioctl_ptr->ioctl_ptr,
							  &u_isp_event,
				sizeof(struct msm_isp_event_ctrl))) {
			kfree(k_isp_event);
			rc = -EINVAL;
			mutex_unlock(&g_server_dev.server_queue_lock);
			return rc;
		}
		kfree(k_isp_event);
		mutex_unlock(&g_server_dev.server_queue_lock);
	
		rc = 0;
		break;
	}
	
	case MSM_CAM_IOCTL_SEND_EVENT:
		rc = msm_server_send_v4l2_evt(arg);
		break;

	default:
	
		pr_err("%s: Invalid IOCTL = %d", __func__, cmd);
		break;
	}
	return rc;
}
static long msm_server_send_v4l2_evt(void *evt)
{
	struct v4l2_event *v4l2_ev = (struct v4l2_event *)evt;
	int rc = 0;

	if (NULL == evt) {
		pr_err("%s: evt is NULL\n", __func__);
		return -EINVAL;
	}

	D("%s: evt type 0x%x\n", __func__, v4l2_ev->type);
	if ((v4l2_ev->type >= MSM_GES_APP_EVT_MIN) &&
		(v4l2_ev->type < MSM_GES_APP_EVT_MAX)) {
		msm_cam_server_subdev_notify(g_server_dev.gesture_device,
			NOTIFY_GESTURE_EVT, v4l2_ev);
	} else {
		pr_err("%s: Invalid evt %d\n", __func__, v4l2_ev->type);
		rc = -EINVAL;
	}
	D("%s: end\n", __func__);

	return rc;
}

static int msm_open_server(struct file *fp)
{
	int rc = 0;
	D("%s: open %s\n", __func__, fp->f_path.dentry->d_name.name);
	mutex_lock(&g_server_dev.server_lock);
	
	g_server_dev.use_count++;
	if (g_server_dev.use_count == 1)
		fp->private_data =
			&g_server_dev.server_command_queue.eventHandle;
	mutex_unlock(&g_server_dev.server_lock);
	return rc;
}

static unsigned int msm_poll_config(struct file *fp,
					struct poll_table_struct *wait)
{
	int rc = 0;
	struct msm_cam_config_dev *config = fp->private_data;
	if (config == NULL)
		return -EINVAL;

	D("%s\n", __func__);

	poll_wait(fp,
	&config->config_stat_event_queue.eventHandle.events->wait, wait);
	if (v4l2_event_pending(&config->config_stat_event_queue.eventHandle))
		rc |= POLLPRI;
	return rc;
}

static int msm_close_server(struct file *fp)
{
	struct v4l2_event_subscription sub;

	D("%s\n", __func__);

	mutex_lock(&g_server_dev.server_lock);
	if (g_server_dev.use_count > 0)
		g_server_dev.use_count--;
	mutex_unlock(&g_server_dev.server_lock);
	if (g_server_dev.use_count == 0) {
		int i;
		mutex_lock(&g_server_dev.server_lock);
		for (i = 0; i < MAX_NUM_ACTIVE_CAMERA; i++) {
			if (g_server_dev.pcam_active[i]) {
				struct v4l2_event v4l2_ev;

				msm_cam_stop_hardware(g_server_dev.pcam_active[i]);

				v4l2_ev.type = V4L2_EVENT_PRIVATE_START
					+ MSM_CAM_APP_NOTIFY_ERROR_EVENT;
				ktime_get_ts(&v4l2_ev.timestamp);
				v4l2_event_queue(
					g_server_dev.pcam_active[i]->pvdev, &v4l2_ev);
			}
		}
		
		sub.type = V4L2_EVENT_ALL;
		msm_server_v4l2_unsubscribe_event(
			&g_server_dev.server_command_queue.eventHandle, &sub);
		mutex_unlock(&g_server_dev.server_lock);
	}
	return 0;
}


static long msm_v4l2_evt_notify(struct msm_cam_media_controller *mctl,
		unsigned int cmd, unsigned long evt)
{
	struct v4l2_event v4l2_ev;
	struct msm_cam_v4l2_device *pcam = NULL;

	if (!mctl) {
		pr_err("%s: mctl is NULL\n", __func__);
		return -EINVAL;
	}

	if (copy_from_user(&v4l2_ev, (void __user *)evt,
		sizeof(struct v4l2_event))) {
		ERR_COPY_FROM_USER();
		return -EFAULT;
	}

	pcam = mctl->pcam_ptr;
	ktime_get_ts(&v4l2_ev.timestamp);
	v4l2_event_queue(pcam->pvdev, &v4l2_ev);
	return 0;
}

static long msm_ioctl_config(struct file *fp, unsigned int cmd,
	unsigned long arg)
{

	int rc = 0;
	struct v4l2_event ev;
	struct msm_cam_config_dev *config_cam = fp->private_data;
	struct v4l2_event_subscription temp_sub;

	D("%s: cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	
	case MSM_CAM_IOCTL_REGISTER_PMEM:
		return msm_register_pmem(
			&config_cam->p_mctl->stats_info.pmem_stats_list,
			(void __user *)arg, config_cam->p_mctl->client);
		break;

	case MSM_CAM_IOCTL_UNREGISTER_PMEM:
		return msm_pmem_table_del(
			&config_cam->p_mctl->stats_info.pmem_stats_list,
			(void __user *)arg, config_cam->p_mctl->client);
		break;
	case VIDIOC_SUBSCRIBE_EVENT:
		if (copy_from_user(&temp_sub,
			(void __user *)arg,
			sizeof(struct v4l2_event_subscription))) {
			rc = -EINVAL;
			return rc;
		}
		rc = msm_server_v4l2_subscribe_event
			(&config_cam->config_stat_event_queue.eventHandle,
			&temp_sub);
		if (rc < 0) {
			pr_err("%s: cam_v4l2_subscribe_event failed rc=%d\n",
				__func__, rc);
			return rc;
		}
		break;

	case VIDIOC_UNSUBSCRIBE_EVENT:
		if (copy_from_user(&temp_sub, (void __user *)arg,
			  sizeof(struct v4l2_event_subscription))) {
			rc = -EINVAL;
			return rc;
		}
		rc = msm_server_v4l2_unsubscribe_event
			(&config_cam->config_stat_event_queue.eventHandle,
			&temp_sub);
		if (rc < 0) {
			pr_err("%s: server_unsubscribe_event failed rc=%d\n",
				__func__, rc);
		}
		break;

	case VIDIOC_DQEVENT: {
		void __user *u_msg_value = NULL, *user_ptr = NULL;
		struct msm_isp_event_ctrl u_isp_event;
		struct msm_isp_event_ctrl *k_isp_event;

		
		D("%s: VIDIOC_DQEVENT\n", __func__);
		if (copy_from_user(&ev, (void __user *)arg,
				sizeof(struct v4l2_event)))
			break;
		user_ptr = (void __user *)(*((uint32_t *)ev.u.data));

		
		if (copy_from_user((void *)&u_isp_event, user_ptr,
				   sizeof(struct msm_isp_event_ctrl))) {
			rc = -EFAULT;
			break;
		}
		
		u_msg_value = u_isp_event.isp_data.isp_msg.data;

		
		rc = v4l2_event_dequeue(
			&config_cam->config_stat_event_queue.eventHandle,
			&ev, fp->f_flags & O_NONBLOCK);
		if (rc < 0) {
			pr_err("no pending events?");
			rc = -EFAULT;
			break;
		}
		k_isp_event = (struct msm_isp_event_ctrl *)
				(*((uint32_t *)ev.u.data));
		
		u_isp_event = *k_isp_event;
		if (ev.type != (V4L2_EVENT_PRIVATE_START +
				MSM_CAM_RESP_DIV_FRAME_EVT_MSG) &&
				ev.type != (V4L2_EVENT_PRIVATE_START +
				MSM_CAM_RESP_MCTL_PP_EVENT)) {

			u_isp_event.isp_data.isp_msg.data = u_msg_value;

			if (ev.type == (V4L2_EVENT_PRIVATE_START +
					MSM_CAM_RESP_STAT_EVT_MSG)) {
				if (k_isp_event->isp_data.isp_msg.len > 0) {
					void *k_msg_value =
					k_isp_event->isp_data.isp_msg.data;
					if (copy_to_user(u_msg_value,
							k_msg_value,
					 k_isp_event->isp_data.isp_msg.len)) {
						rc = -EINVAL;
						
						pr_err("%s: %d copy_to_user failed. msg_id: %d, frame id: %d\n",
							__func__, __LINE__, k_isp_event->isp_data.isp_msg.msg_id,
							k_isp_event->isp_data.isp_msg.frame_id);
						kfree(k_msg_value);
						kfree(k_isp_event);
						
						break;
					}
					kfree(k_msg_value);
				}
			}
		}
		if (copy_to_user(user_ptr,
				(void *)&u_isp_event, sizeof(
				struct msm_isp_event_ctrl))) {
			rc = -EINVAL;
			
			pr_err("%s: %d copy_to_user failed. msg_id: %d, frame id: %d\n",
				__func__, __LINE__, k_isp_event->isp_data.isp_msg.msg_id,
				k_isp_event->isp_data.isp_msg.frame_id);
			kfree(k_isp_event);
			
			break;
		}
		kfree(k_isp_event);

		
		if (copy_to_user((void __user *)arg, &ev,
				sizeof(struct v4l2_event))) {
			rc = -EINVAL;
			break;
		}
		}

		break;

	case MSM_CAM_IOCTL_V4L2_EVT_NOTIFY:
		rc = msm_v4l2_evt_notify(config_cam->p_mctl, cmd, arg);
		break;
#if 0
	case MSM_CAM_IOCTL_SET_MEM_MAP_INFO:
		if (copy_from_user(&config_cam->mem_map, (void __user *)arg,
				sizeof(struct msm_mem_map_info)))
			rc = -EINVAL;
		break;
#endif

	default:{
		
		struct msm_cam_media_controller *p_mctl = config_cam->p_mctl;
		if (p_mctl && p_mctl->mctl_cmd) {
			rc = config_cam->p_mctl->mctl_cmd(p_mctl, cmd, arg);
		} else {
			rc = -EINVAL;
			pr_err("%s: media controller is null\n", __func__);
		}

		break;
	} 
	} 
	return rc;
}

static int msm_mmap_config(struct file *fp, struct vm_area_struct *vma)
{
	struct msm_cam_config_dev *config_cam = fp->private_data;
	int rc = 0;
	int phyaddr;
	int retval;
	unsigned long size;

	D("%s: phy_addr=0x%x", __func__, config_cam->mem_map.cookie);
	phyaddr = (int)config_cam->mem_map.cookie;
	if (!phyaddr) {
		pr_err("%s: no physical memory to map", __func__);
		return -EFAULT;
	}
	memset(&config_cam->mem_map, 0,
		sizeof(struct msm_mem_map_info));
	size = vma->vm_end - vma->vm_start;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	retval = remap_pfn_range(vma, vma->vm_start,
					phyaddr >> PAGE_SHIFT,
					size, vma->vm_page_prot);
	if (retval) {
		pr_err("%s: remap failed, rc = %d",
					__func__, retval);
		rc = -ENOMEM;
		goto end;
	}
	D("%s: phy_addr=0x%x: %08lx-%08lx, pgoff %08lx\n",
			__func__, (uint32_t)phyaddr,
			vma->vm_start, vma->vm_end, vma->vm_pgoff);
end:
	return rc;
}

static int msm_open_config(struct inode *inode, struct file *fp)
{
	int rc;

	struct msm_cam_config_dev *config_cam = container_of(inode->i_cdev,
		struct msm_cam_config_dev, config_cdev);

	D("%s: open %s\n", __func__, fp->f_path.dentry->d_name.name);

	rc = nonseekable_open(inode, fp);
	if (rc < 0) {
		pr_err("%s: nonseekable_open error %d\n", __func__, rc);
		return rc;
	}
	config_cam->use_count++;

	
	
	config_cam->p_mctl =
		msm_camera_get_mctl(g_server_dev.pcam_active[config_cam->dev_num]->mctl_handle);

	if(!config_cam->p_mctl)
		return -EFAULT;

		
	INIT_HLIST_HEAD(&config_cam->p_mctl->stats_info.pmem_stats_list);
	spin_lock_init(&config_cam->p_mctl->stats_info.pmem_stats_spinlock);

	config_cam->p_mctl->config_device = config_cam;
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	kref_get(&config_cam->p_mctl->refcount);
#endif
	fp->private_data = config_cam;
	return rc;
}

static int msm_close_config(struct inode *node, struct file *f)
{
	struct v4l2_event ev;
	struct v4l2_event_subscription sub;
	struct msm_isp_event_ctrl *isp_event;
	struct msm_cam_config_dev *config_cam = f->private_data;
	
#ifdef CONFIG_MSM_MULTIMEDIA_USE_ION
	D("%s Decrementing ref count of config node ", __func__);
	kref_put(&config_cam->p_mctl->refcount, msm_release_ion_client);
#endif
	sub.type = V4L2_EVENT_ALL;
	msm_server_v4l2_unsubscribe_event(
		&config_cam->config_stat_event_queue.eventHandle,
		&sub);
	while (v4l2_event_pending(
		&config_cam->config_stat_event_queue.eventHandle)) {
		v4l2_event_dequeue(
			&config_cam->config_stat_event_queue.eventHandle,
			&ev, O_NONBLOCK);
		isp_event = (struct msm_isp_event_ctrl *)
			(*((uint32_t *)ev.u.data));
		if (isp_event) {
			if (isp_event->isp_data.isp_msg.len != 0 &&
				isp_event->isp_data.isp_msg.data != NULL)
				kfree(isp_event->isp_data.isp_msg.data);
			kfree(isp_event);
		}
	}

	return 0;
}

static struct v4l2_file_operations g_msm_fops = {
	.owner   = THIS_MODULE,
	.open	= msm_open,
	.poll	= msm_poll,
	.mmap	= msm_mmap,
	.release = msm_close,
	.ioctl   = video_ioctl2,
};

static const struct v4l2_file_operations msm_fops_server = {
	.owner = THIS_MODULE,
	.open  = msm_open_server,
	.poll  = msm_poll_server,
	.unlocked_ioctl = video_ioctl2,
	.release = msm_close_server,
};

static const struct v4l2_ioctl_ops msm_ioctl_ops_server = {
	.vidioc_subscribe_event = msm_server_v4l2_subscribe_event,
	.vidioc_default = msm_ioctl_server,
};

static const struct file_operations msm_fops_config = {
	.owner = THIS_MODULE,
	.open  = msm_open_config,
	.poll  = msm_poll_config,
	.unlocked_ioctl = msm_ioctl_config,
	.mmap	= msm_mmap_config,
	.release = msm_close_config,
};

static struct camera_flash_info *p_flash_led_info;
static struct kobject *led_status_obj; 

static ssize_t flash_led_info_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length = 0;

	if (p_flash_led_info != NULL)
		length = sprintf(buf, "%d %d %d %d\n",
			p_flash_led_info->led_info->enable,
			p_flash_led_info->led_info->low_limit_led_state,
			p_flash_led_info->led_info->max_led_current_ma,
			p_flash_led_info->led_info->num_led_est_table);
	else
		length = sprintf(buf, "%d\n", 0);
	
	return length;
}

static ssize_t flash_led_tbl_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length = 0;
	uint16_t i = 0;
	char sub[64] = {0};
	struct camera_led_est *sub_tbl = NULL;

	if (p_flash_led_info != NULL)
		for (i = 0; i < p_flash_led_info->led_info->num_led_est_table; i++) {
			sub_tbl = (struct camera_led_est *)
				(((char *)p_flash_led_info->led_est_table) +
				(i * sizeof(struct camera_led_est)));
			if (sub_tbl != NULL) {
			length += sprintf(sub, "%d %d %d %d %d %d ",
				sub_tbl->enable,
				sub_tbl->led_state,
				sub_tbl->current_ma,
				sub_tbl->lumen_value,
				sub_tbl->min_step,
				sub_tbl->max_step);
			strcat(buf, sub);
			}
		}
	else
		length = sprintf(buf, "%d\n", 0);
	return length;
}

static DEVICE_ATTR(flash_led_info, 0444,
	flash_led_info_get,
	NULL);

static DEVICE_ATTR(flash_led_tbl, 0444,
	flash_led_tbl_get,
	NULL);

static uint32_t led_ril_status_value;
static uint32_t led_wimax_status_value;
static uint32_t led_hotspot_status_value;
static uint16_t led_low_temp_limit;
static uint16_t led_low_cap_limit;
static uint16_t led_low_cap_limit_dual;

static ssize_t led_ril_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_ril_status_value);
	return length;
}

static ssize_t led_ril_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	if (buf[1] == '\n')
		tmp = buf[0] - 0x30;

	led_ril_status_value = tmp;
	pr_info("led_ril_status_value = %d\n", led_ril_status_value);
	return count;
}

static ssize_t led_wimax_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_wimax_status_value);
	return length;
}

static ssize_t led_wimax_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	if (buf[1] == '\n')
		tmp = buf[0] - 0x30;

	led_wimax_status_value = tmp;
	pr_info("led_wimax_status_value = %d\n", led_wimax_status_value);
	return count;
}

static ssize_t led_hotspot_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_hotspot_status_value);
	return length;
}

static ssize_t led_hotspot_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	tmp = buf[0] - 0x30; 

	led_hotspot_status_value = tmp;
	pr_info("led_hotspot_status_value = %d\n", led_hotspot_status_value);
	return count;
}

static ssize_t low_temp_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_temp_limit);
	return length;
}

static ssize_t low_cap_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_cap_limit);
	return length;
}

static ssize_t low_cap_limit_dual_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_cap_limit_dual);
	return length;
}

static DEVICE_ATTR(led_ril_status, 0644,
	led_ril_status_get,
	led_ril_status_set);

static DEVICE_ATTR(led_wimax_status, 0644,
	led_wimax_status_get,
	led_wimax_status_set);

static DEVICE_ATTR(led_hotspot_status, 0644,
	led_hotspot_status_get,
	led_hotspot_status_set);

static DEVICE_ATTR(low_temp_limit, 0444,
	low_temp_limit_get,
	NULL);

static DEVICE_ATTR(low_cap_limit, 0444,
	low_cap_limit_get,
	NULL);

static DEVICE_ATTR(low_cap_limit_dual, 0444,
	low_cap_limit_dual_get,
	NULL);

static int msm_sensor_attr_node(struct msm_camera_sensor_info *sdata)
{
	int ret = 0;

	led_status_obj = kobject_create_and_add("camera_led_status", NULL);
	if (led_status_obj == NULL) {
		pr_info("msm_camera: subsystem_register failed\n");
		ret = -ENOMEM;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_flash_led_info.attr);
	if (ret) {
		pr_info("msm_camera: sysfs_create_file flash_led_info failed\n");
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_flash_led_tbl.attr);
	if (ret) {
		pr_info("msm_camera: sysfs_create_file flash_led_tbl failed\n");
		ret = -EFAULT;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_ril_status.attr);
	if (ret) {
		pr_info("msm_camera: sysfs_create_file dev_attr_led_ril_status failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_wimax_status.attr);
	if (ret) {
		pr_info("msm_camera: sysfs_create_file dev_attr_led_wimax_status failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_hotspot_status.attr);
	if (ret) {
		pr_info("msm_camera: sysfs_create_file dev_attr_led_hotspot_status failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_temp_limit.attr);
	if (ret) {
		pr_info("msm_camera: sysfs_create_file dev_attr_low_temp_limit failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_cap_limit.attr);
	if (ret) {
		pr_info("msm_camera: sysfs_create_file dev_attr_low_cap_limit failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_cap_limit_dual.attr);
	if (ret) {
		pr_info("msm_camera: sysfs_create_file dev_attr_low_cap_limit_dual failed\n");
		ret = -EFAULT;
		goto error;
	}

	if ((sdata->flash_data->flash_type != MSM_CAMERA_FLASH_NONE) &&
		sdata->flash_cfg && sdata->flash_cfg->flash_info) {
		p_flash_led_info = sdata->flash_cfg->flash_info;
	} else {
		p_flash_led_info = NULL;
	}

	led_low_temp_limit = sdata->flash_cfg->low_temp_limit;
	led_low_cap_limit = sdata->flash_cfg->low_cap_limit;
	led_low_cap_limit_dual = sdata->flash_cfg->low_cap_limit_dual;

	return ret;

error:
	kobject_del(led_status_obj);
	return ret;
}

#ifdef CONFIG_RAWCHIP
static struct kobject *rawchip_status_obj;

uint32_t rawchip_id;

static ssize_t probed_rawchip_id_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "0x%x\n", rawchip_id);
	return length;
}

static DEVICE_ATTR(probed_rawchip_id, 0444,
	probed_rawchip_id_get,
	NULL);

int msm_rawchip_attr_node(void)
{
	int ret = 0;

	rawchip_status_obj = kobject_create_and_add("camera_rawchip_status", NULL);
	if (rawchip_status_obj == NULL) {
		pr_err("msm_camera: create camera_rawchip_status failed\n");
		ret = -ENOMEM;
		goto error;
	}

	ret = sysfs_create_file(rawchip_status_obj,
		&dev_attr_probed_rawchip_id.attr);
	if (ret) {
		pr_info("msm_camera: sysfs_create_file dev_attr_probed_rawchip_id failed\n");
		ret = -EFAULT;
		goto error;
	}

	return ret;

error:
	kobject_del(rawchip_status_obj);
	return ret;
}
#endif


int msm_setup_v4l2_event_queue(struct v4l2_fh *eventHandle,
	struct video_device *pvdev)
{
	int rc = 0;
	
	spin_lock_init(&pvdev->fh_lock);
	INIT_LIST_HEAD(&pvdev->fh_list);

	rc = v4l2_fh_init(eventHandle, pvdev);
	if (rc < 0)
		return rc;
	if (eventHandle->events == NULL) {
		rc = v4l2_event_init(eventHandle);
		if (rc < 0)
			return rc;
	}

	
	
	rc = v4l2_event_alloc(eventHandle, 200);
	
	if (rc < 0) {
		pr_err("%s, v4l2_event_alloc failed, rc = %d", __func__, rc);
		return rc;
	}
	v4l2_fh_add(eventHandle);
	return rc;

}

static int msm_setup_config_dev(int node, char *device_name)
{
	int rc = -ENODEV;
	struct device *device_config;
	int dev_num = node;
	dev_t devno;
	struct msm_cam_config_dev *config_cam;

	config_cam = kzalloc(sizeof(*config_cam), GFP_KERNEL);
	if (!config_cam) {
		pr_err("%s: could not allocate memory for msm_cam_config_device\n",
			__func__);
		return -ENOMEM;
	}

	D("%s\n", __func__);

	devno = MKDEV(MAJOR(msm_devno), dev_num+1);
	device_config = device_create(msm_class, NULL, devno, NULL, "%s%d",
		device_name, dev_num);

	if (IS_ERR(device_config)) {
		rc = PTR_ERR(device_config);
		pr_err("%s: error creating device: %d\n", __func__, rc);
		goto config_setup_fail;
	}

	cdev_init(&config_cam->config_cdev, &msm_fops_config);
	config_cam->config_cdev.owner = THIS_MODULE;

	rc = cdev_add(&config_cam->config_cdev, devno, 1);
	if (rc < 0) {
		pr_err("%s: error adding cdev: %d\n", __func__, rc);
		device_destroy(msm_class, devno);
		goto config_setup_fail;
	}

	g_server_dev.config_info.config_dev_name[dev_num] =
		dev_name(device_config);
	D("%s Connected config device %s\n", __func__,
		g_server_dev.config_info.config_dev_name[dev_num]);
	g_server_dev.config_info.config_dev_id[dev_num] = dev_num;

	config_cam->config_stat_event_queue.pvdev = video_device_alloc();
	if (config_cam->config_stat_event_queue.pvdev == NULL) {
		pr_err("%s: video_device_alloc failed\n", __func__);
		goto config_setup_fail;
	}

	rc = msm_setup_v4l2_event_queue(
		&config_cam->config_stat_event_queue.eventHandle,
		config_cam->config_stat_event_queue.pvdev);
	if (rc < 0) {
		pr_err("%s failed to initialize event queue\n", __func__);
		video_device_release(config_cam->config_stat_event_queue.pvdev);
		goto config_setup_fail;
	}

	config_cam->dev_num = dev_num;

	return rc;

config_setup_fail:
	kfree(config_cam);
	return rc;

}

static void msm_cam_server_send_error_evt(
		struct msm_cam_media_controller *pmctl, int evt_type)
{
	struct v4l2_event v4l2_ev;
	v4l2_ev.type = evt_type;
	ktime_get_ts(&v4l2_ev.timestamp);
	v4l2_event_queue(pmctl->pcam_ptr->pvdev, &v4l2_ev);
}

static void msm_cam_server_subdev_notify(struct v4l2_subdev *sd,
				unsigned int notification, void *arg)
{
	int rc = -EINVAL;
	struct msm_sensor_ctrl_t *s_ctrl;
	struct msm_camera_sensor_info *sinfo;
	struct msm_camera_device_platform_data *camdev;
	struct msm_cam_media_controller *p_mctl =
		(struct msm_cam_media_controller *)v4l2_get_subdev_hostdata(sd);
	uint8_t csid_core = 0;

	if (notification == NOTIFY_CID_CHANGE ||
		notification == NOTIFY_ISPIF_STREAM ||
		notification == NOTIFY_PCLK_CHANGE ||
		notification == NOTIFY_CSIPHY_CFG ||
		notification == NOTIFY_CSID_CFG ||
		notification == NOTIFY_CSIC_CFG) {
		s_ctrl = get_sctrl(sd);
		sinfo = (struct msm_camera_sensor_info *) s_ctrl->sensordata;
		camdev = sinfo->pdata;
		csid_core = camdev->csid_core;
	}

	switch (notification) {
	case NOTIFY_CID_CHANGE:
		
		if (g_server_dev.ispif_device) {
			enum msm_ispif_intftype *intf = (enum msm_ispif_intftype *) arg;
			struct msm_ispif_params_list ispif_params;
			ispif_params.len = 1;
			ispif_params.params[0].intftype = *intf;
			ispif_params.params[0].cid_mask = 0x0001;
			ispif_params.params[0].csid = csid_core;

			pr_info("%s: NOTIFY_CID_CHANGE intftype %d\n", __func__, *intf);

			rc = v4l2_subdev_call(
				g_server_dev.ispif_device, core, ioctl,
				VIDIOC_MSM_ISPIF_CFG, &ispif_params);
			if (rc < 0) {
				pr_err("msm_cam_server_subdev_notify: VIDIOC_MSM_ISPIF_CFG failed");
				return;
			}
		}
		break;
	case NOTIFY_ISPIF_STREAM:
		
		rc = v4l2_subdev_call(g_server_dev.ispif_device, video,
				s_stream, (int)arg);
		if (rc < 0)
			return;

		break;
	case NOTIFY_ISP_MSG_EVT:
	case NOTIFY_VFE_MSG_OUT:
	case NOTIFY_VFE_MSG_STATS:
	case NOTIFY_VFE_MSG_COMP_STATS:
	case NOTIFY_VFE_BUF_EVT:
	case NOTIFY_VFE_BUF_FREE_EVT:
		if (g_server_dev.isp_subdev[0] &&
			g_server_dev.isp_subdev[0]->isp_notify) {
			rc = g_server_dev.isp_subdev[0]->isp_notify(
				g_server_dev.vfe_device[0], notification, arg);
		}
		break;
	case NOTIFY_VPE_MSG_EVT: {
		struct msm_cam_media_controller *pmctl =
		(struct msm_cam_media_controller *)
		v4l2_get_subdev_hostdata(sd);
		struct msm_vpe_resp *vdata = (struct msm_vpe_resp *)arg;
		msm_mctl_pp_notify(pmctl,
		(struct msm_mctl_pp_frame_info *)
		vdata->extdata);
		break;
	}
	case NOTIFY_VFE_IRQ:{
		struct msm_vfe_cfg_cmd cfg_cmd;
		struct msm_camvfe_params vfe_params;
		cfg_cmd.cmd_type = CMD_VFE_PROCESS_IRQ;
		vfe_params.vfe_cfg = &cfg_cmd;
		vfe_params.data = arg;
		rc = v4l2_subdev_call(g_server_dev.vfe_device[0],
			core, ioctl, 0, &vfe_params);
	}
		break;
	case NOTIFY_AXI_IRQ:
		rc = v4l2_subdev_call(g_server_dev.axi_device[0],
			core, ioctl, VIDIOC_MSM_AXI_IRQ, arg);
		break;
	case NOTIFY_AXI_RDI_SOF_COUNT:
		rc = v4l2_subdev_call(g_server_dev.axi_device[0], core, ioctl,
			VIDIOC_MSM_AXI_RDI_COUNT_UPDATE, arg);
		break;
	case NOTIFY_PCLK_CHANGE:
		if (g_server_dev.axi_device[0])
			rc = v4l2_subdev_call(g_server_dev.axi_device[0], video,
				s_crystal_freq, *(uint32_t *)arg, 0);
		else
			rc = v4l2_subdev_call(g_server_dev.vfe_device[0], video,
				s_crystal_freq, *(uint32_t *)arg, 0);
		break;
	case NOTIFY_CSIPHY_CFG:
		rc = v4l2_subdev_call(g_server_dev.csiphy_device[csid_core],
			core, ioctl, VIDIOC_MSM_CSIPHY_CFG, arg);
		break;
	case NOTIFY_CSID_CFG:
		rc = v4l2_subdev_call(g_server_dev.csid_device[csid_core],
			core, ioctl, VIDIOC_MSM_CSID_CFG, arg);
		break;
	case NOTIFY_CSIC_CFG:
		rc = v4l2_subdev_call(g_server_dev.csic_device[csid_core],
			core, ioctl, VIDIOC_MSM_CSIC_CFG, arg);
		break;
	case NOTIFY_GESTURE_EVT:
		rc = v4l2_subdev_call(g_server_dev.gesture_device,
			core, ioctl, VIDIOC_MSM_GESTURE_EVT, arg);
		break;
	case NOTIFY_GESTURE_CAM_EVT:
		rc = v4l2_subdev_call(g_server_dev.gesture_device,
			core, ioctl, VIDIOC_MSM_GESTURE_CAM_EVT, arg);
		break;
	case NOTIFY_VFE_CAMIF_ERROR: {
		if (p_mctl)
			msm_cam_server_send_error_evt(p_mctl,
				V4L2_EVENT_PRIVATE_START +
				MSM_CAM_APP_NOTIFY_ERROR_EVENT);
		break;
	}
	case NOTIFY_VFE_VIOLATION: {
		if (p_mctl)
			msm_cam_server_send_error_evt(p_mctl,
				V4L2_EVENT_PRIVATE_START +
				MSM_CAM_APP_NOTIFY_ERROR_EVENT);
		break;
	}
	default:
		break;
	}

	return;
}

int msm_cam_register_subdev_node(struct v4l2_subdev *sd,
	enum msm_cam_subdev_type sdev_type, uint8_t index)
{
	struct video_device *vdev;
	int err = 0;

	if (sdev_type == CSIPHY_DEV) {
		if (index >= MAX_NUM_CSIPHY_DEV)
			return -EINVAL;
		g_server_dev.csiphy_device[index] = sd;
	} else if (sdev_type == CSID_DEV) {
		if (index >= MAX_NUM_CSID_DEV)
			return -EINVAL;
		g_server_dev.csid_device[index] = sd;
	} else if (sdev_type == CSIC_DEV) {
		if (index >= MAX_NUM_CSIC_DEV)
			return -EINVAL;
		g_server_dev.csic_device[index] = sd;
	} else if (sdev_type == ISPIF_DEV) {
		g_server_dev.ispif_device = sd;
	} else if (sdev_type == VFE_DEV) {
		if (index >= MAX_NUM_VFE_DEV)
			return -EINVAL;
		g_server_dev.vfe_device[index] = sd;
	} else if (sdev_type == VPE_DEV) {
		if (index >= MAX_NUM_VPE_DEV)
			return -EINVAL;
		g_server_dev.vpe_device[index] = sd;
	} else if (sdev_type == AXI_DEV) {
		if (index >= MAX_NUM_AXI_DEV)
			return -EINVAL;
		g_server_dev.axi_device[index] = sd;		
	}
	else if (sdev_type == GESTURE_DEV) {
		g_server_dev.gesture_device = sd;
	}

	err = v4l2_device_register_subdev(&g_server_dev.v4l2_dev, sd);
	if (err < 0)
		return err;

	if (!(sd->flags & V4L2_SUBDEV_FL_HAS_DEVNODE))
		return err;

	vdev = &sd->devnode;
	strlcpy(vdev->name, sd->name, sizeof(vdev->name));
	vdev->v4l2_dev = &g_server_dev.v4l2_dev;
	vdev->fops = &v4l2_subdev_fops;
	vdev->release = video_device_release_empty;
	err = __video_register_device(vdev, VFL_TYPE_SUBDEV, -1, 1,
						  sd->owner);
	if (err < 0)
		return err;
#if defined(CONFIG_MEDIA_CONTROLLER)
	sd->entity.v4l.major = VIDEO_MAJOR;
	sd->entity.v4l.minor = vdev->minor;
#endif
	return 0;
}


static int msm_setup_server_dev(struct platform_device *pdev)
{
	int rc = -ENODEV, i;

	D("%s\n", __func__);
	g_server_dev.server_pdev = pdev;
	g_server_dev.v4l2_dev.dev = &pdev->dev;
	g_server_dev.v4l2_dev.notify = msm_cam_server_subdev_notify;

	rc = v4l2_device_register(g_server_dev.v4l2_dev.dev,
			&g_server_dev.v4l2_dev);
	if (rc < 0)
		return -EINVAL;


	g_server_dev.video_dev = video_device_alloc();
	if (g_server_dev.video_dev == NULL) {
		pr_err("%s: video_device_alloc failed\n", __func__);
		return rc;
	}

	strlcpy(g_server_dev.video_dev->name, pdev->name,
			sizeof(g_server_dev.video_dev->name));

	g_server_dev.video_dev->v4l2_dev = &g_server_dev.v4l2_dev;
	g_server_dev.video_dev->fops = &msm_fops_server;
	g_server_dev.video_dev->ioctl_ops = &msm_ioctl_ops_server;
	g_server_dev.video_dev->release   = video_device_release;
	g_server_dev.video_dev->minor = 100;
	g_server_dev.video_dev->vfl_type = 1;

	video_set_drvdata(g_server_dev.video_dev, &g_server_dev);

	strlcpy(g_server_dev.media_dev.model, "qcamera",
		sizeof(g_server_dev.media_dev.model));
	g_server_dev.media_dev.dev = &pdev->dev;
#if defined(CONFIG_MEDIA_CONTROLLER)
	rc = media_device_register(&g_server_dev.media_dev);

	g_server_dev.v4l2_dev.mdev = &g_server_dev.media_dev;
#endif
	rc = video_register_device(g_server_dev.video_dev,
		VFL_TYPE_GRABBER, 100);


	mutex_init(&g_server_dev.server_lock);
	mutex_init(&g_server_dev.server_queue_lock);
	g_server_dev.camera_info.num_cameras = 0;
	atomic_set(&g_server_dev.number_pcam_active, 0);
	g_server_dev.server_evt_id = 0;

	

	g_server_dev.server_command_queue.pvdev = g_server_dev.video_dev;

	rc = msm_setup_v4l2_event_queue(
		&g_server_dev.server_command_queue.eventHandle,
		g_server_dev.server_command_queue.pvdev);
	if (rc < 0) {
		pr_err("%s failed to initialize event queue\n", __func__);
		video_device_release(g_server_dev.server_command_queue.pvdev);
		return rc;
	}
	for (i = 0; i < MAX_NUM_ACTIVE_CAMERA; i++) {
		struct msm_cam_server_queue *queue;
		queue = &g_server_dev.server_queue[i];
		queue->queue_active = 0;
		msm_queue_init(&queue->ctrl_q, "control");
		msm_queue_init(&queue->eventData_q, "eventdata");
		g_server_dev.pcam_active[i] = NULL;
	}
	return rc;
}

static int msm_cam_dev_init(struct msm_cam_v4l2_device *pcam)
{
	int rc = -ENOMEM;
	struct video_device *pvdev = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(pcam->sensor_sdev);
	D("%s\n", __func__);

	
	pcam->v4l2_dev.dev = &client->dev;
	rc = v4l2_device_register(pcam->v4l2_dev.dev, &pcam->v4l2_dev);
	if (rc < 0)
		return -EINVAL;
	else
		pcam->v4l2_dev.notify = msm_cam_v4l2_subdev_notify;


	
	pvdev = video_device_alloc();
	if (pvdev == NULL) {
		pr_err("%s: video_device_alloc failed\n", __func__);
		return rc;
	}

#if defined(CONFIG_MEDIA_CONTROLLER)
	strlcpy(pcam->media_dev.model, QCAMERA_NAME,
			sizeof(pcam->media_dev.model));
	pcam->media_dev.dev = &client->dev;
	rc = media_device_register(&pcam->media_dev);
	pvdev->v4l2_dev = &pcam->v4l2_dev;
	pcam->v4l2_dev.mdev = &pcam->media_dev;
#endif

	
	D("sensor name = %s, sizeof(pvdev->name)=%d\n",
		pcam->sensor_sdev->name, sizeof(pvdev->name));

	strlcpy(pvdev->name, pcam->sensor_sdev->name, sizeof(pvdev->name));

	pvdev->release   = video_device_release;
	pvdev->fops	  = &g_msm_fops;
	pvdev->ioctl_ops = &g_msm_ioctl_ops;
	pvdev->minor	 = -1;
	pvdev->vfl_type  = 1;

#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_init(&pvdev->entity, 0, NULL, 0);
	pvdev->entity.type = MEDIA_ENT_T_DEVNODE_V4L;
	pvdev->entity.group_id = QCAMERA_VNODE_GROUP_ID;
#endif

	
	D("video_register_device\n");
	rc = video_register_device(pvdev,
					VFL_TYPE_GRABBER,
					msm_camera_v4l2_nr);
	if (rc) {
		pr_err("%s: video_register_device failed\n", __func__);
		goto reg_fail;
	}

#if defined(CONFIG_MEDIA_CONTROLLER)
	pvdev->entity.name = video_device_node_name(pvdev);
#endif

	D("%s: video device registered as /dev/video%d\n",
		__func__, pvdev->num);

	
	pcam->pvdev	= pvdev;
	video_set_drvdata(pcam->pvdev, pcam);

	
	

	return rc ;

reg_fail:
	video_device_release(pvdev);
	v4l2_device_unregister(&pcam->v4l2_dev);
	pcam->v4l2_dev.dev = NULL;
	return rc;
}

static void cam_on_check_vcm(void)
{
	pr_info("[CAM_VCM_CB]  cam_on_check_vcm()  START\n");
	mutex_lock(&cam_vcm_on_mut);

	if (cam_vcm_on == STATUS_ON) {
		pr_info("[CAM_VCM_CB]  cam_on_check_vcm()   CAM_VCM_ON !!!   need to power off vcm and sensor\n");
		if (g_vcm_wa_ctl.actuator_power_off_af)
			g_vcm_wa_ctl.actuator_power_off_af();
		if (g_vcm_wa_ctl.vcm_vreg_off)
			g_vcm_wa_ctl.vcm_vreg_off();
		mdelay(1);
		cam_vcm_on = STATUS_OFF;
	}
	camera_on = STATUS_ON;

	mutex_unlock(&cam_vcm_on_mut);
	pr_info("[CAM_VCM_CB]  cam_on_check_vcm()  END\n");
}

void cam_vcm_on_cb(void)
{
	if (!is_actuator_probe_success) {
		pr_info("[CAM_VCM_CB] %s: no actuator, skip vibration workaround", __func__);
		return;
	}

	pr_info("[CAM_VCM_CB]  cam_vcm_on_cb()  START\n");
	mutex_lock(&cam_vcm_on_mut);

	if(camera_on == STATUS_ON) {
		pr_info("[CAM_VCM_CB]  cam_vcm_on_cb() END   already CAMERA ON\n");
		mutex_unlock(&cam_vcm_on_mut);
		return;
	}

	if (cam_vcm_on == STATUS_ON) {
		pr_info("[CAM_VCM_CB]  cam_vcm_on_cb()  END   already VCM ON\n");
		mutex_unlock(&cam_vcm_on_mut);
		return;
	}

	if(g_vcm_wa_ctl.cam_do_vcm_on_cb)
		g_vcm_wa_ctl.cam_do_vcm_on_cb();

	cam_vcm_on = STATUS_ON;
	mutex_unlock(&cam_vcm_on_mut);
	pr_info("[CAM_VCM_CB]  cam_vcm_on_cb()  END\n");
}

void cam_vcm_off_cb(void)
{
	if (!is_actuator_probe_success) {
		pr_info("[CAM_VCM_CB] %s: no actuator, skip vibration workaround", __func__);
		return;
	}

	if (cam_vcm_off_wq) {
		pr_info("[CAM_VCM_CB]  cam_vcm_off_cb()   queue work cam_vcm_off_work\n");
		queue_work_on(0, cam_vcm_off_wq, &cam_vcm_off_work);
	} else {
		pr_info("[CAM_VCM_CB]  cam_vcm_off_cb()   cam_vcm_off_wq is NULL\n");
	}
}

static void do_cam_vcm_off_work(struct work_struct *work)
{
	pr_info("[CAM_VCM_CB]  do_cam_vcm_off_work()  START\n");

	mutex_lock(&cam_vcm_on_mut);

	if(camera_on == STATUS_ON) {
		pr_info("[CAM_VCM_CB]  do_cam_vcm_off_work() already CAMERA ON\n");
		mutex_unlock(&cam_vcm_on_mut);
		return;
	}

	if (cam_vcm_on == STATUS_OFF) {
		pr_info("[CAM_VCM_CB]  do_cam_vcm_off_work() already VCM OFF\n");
		mutex_unlock(&cam_vcm_on_mut);
		return;
	}
	cam_vcm_on = STATUS_OFF;

	if(g_vcm_wa_ctl.cam_do_vcm_off_cb)
		g_vcm_wa_ctl.cam_do_vcm_off_cb();

	mutex_unlock(&cam_vcm_on_mut);
	pr_info("[CAM_VCM_CB]  do_cam_vcm_off_work()  END\n");

}

static int msm_actuator_probe(struct msm_actuator_info *actuator_info,
			      struct v4l2_subdev *act_sdev,
			      struct msm_actuator_ctrl *actctrl)
{
	int rc = 0;
	struct i2c_adapter *adapter = NULL;
	void *act_client = NULL;
	struct msm_actuator_ctrl *a_ext_ctrl = NULL;

	D("%s called\n", __func__);

	if (!actuator_info)
		goto probe_fail;

	adapter = i2c_get_adapter(actuator_info->bus_id);
	if (!adapter)
		goto probe_fail;

	act_client = i2c_new_device(adapter, actuator_info->board_info);
	if (!act_client)
		goto device_fail;

	a_ext_ctrl = (struct msm_actuator_ctrl *)i2c_get_clientdata(act_client);
	if (!a_ext_ctrl)
		goto client_fail;

	*actctrl = *a_ext_ctrl;
	a_ext_ctrl->a_create_subdevice((void *)actuator_info,
				       (void *)act_sdev);

	is_actuator_probe_success = 1; 
	pr_info("%s: actuator_info->board_info->type=%s", __func__, actuator_info->board_info->type);
	pr_info("%s: actuator_info->board_info->addr=0x%x", __func__, actuator_info->board_info->addr);

	return rc;

client_fail:
	i2c_unregister_device(act_client);
device_fail:
	i2c_put_adapter(adapter);
	adapter = NULL;
probe_fail:
	pr_info("%s: probe failed", __func__);
	actctrl->a_init_table = NULL;
	actctrl->a_power_up = NULL;
	actctrl->a_power_down = NULL;
	actctrl->a_config = NULL;
	actctrl->a_create_subdevice = NULL;
	return rc;
}

int msm_sensor_register(struct v4l2_subdev *sensor_sd)
{
	int rc = -EINVAL;
	struct msm_camera_sensor_info *sdata;
	struct msm_cam_v4l2_device *pcam;
	struct msm_sensor_ctrl_t *s_ctrl;
	struct v4l2_subdev *act_sdev = NULL;
	struct msm_actuator_ctrl *actctrl = NULL;

	D("%s for %s\n", __func__, sensor_sd->name);

	
	pcam = kzalloc(sizeof(*pcam), GFP_KERNEL);
	if (!pcam) {
		pr_err("%s: could not allocate mem for msm_cam_v4l2_device\n",
			__func__);
		return -ENOMEM;
	}

	pcam->sensor_sdev = sensor_sd;
	s_ctrl = get_sctrl(sensor_sd);
	if(!s_ctrl) goto failure;
	sdata = (struct msm_camera_sensor_info *) s_ctrl->sensordata;
	if(!sdata) goto failure;

	pcam->act_sdev = kzalloc(sizeof(struct v4l2_subdev),
								  GFP_KERNEL);
	if (!pcam->act_sdev) {
		pr_err("%s: could not allocate mem for actuator v4l2_subdev\n",
			   __func__);
		kfree(pcam);
		return -ENOMEM;
	}

	act_sdev = pcam->act_sdev;
	actctrl = &pcam->actctrl;

	
	if (sdata->actuator_info) {
		if (sdata->use_rawchip)
			sdata->actuator_info->use_rawchip_af = 1;
		else
			sdata->actuator_info->use_rawchip_af = 0;
	}
	

	msm_actuator_probe(sdata->actuator_info,
					   act_sdev, actctrl);

	pcam->sdata = sdata;

	
	if (pcam->sdata && pcam->sdata->flash_cfg )
		msm_sensor_attr_node(pcam->sdata);
	

	
	pcam->use_count = 0;
	mutex_init(&pcam->vid_lock);
	mutex_init(&pcam->mctl_node.dev_lock);

	
	rc  = msm_mctl_init_user_formats(pcam);
	if (rc < 0)
		goto failure;

	rc  = msm_cam_dev_init(pcam);
	if (rc < 0)
		goto failure;

	rc = msm_setup_mctl_node(pcam);
	if (rc < 0) {
		pr_err("%s:failed to create mctl device: %d\n",
			 __func__, rc);
		goto failure;
	}

	g_server_dev.camera_info.video_dev_name
	[g_server_dev.camera_info.num_cameras]
	= video_device_node_name(pcam->pvdev);
	D("%s Connected video device %s\n", __func__,
		g_server_dev.camera_info.video_dev_name
		[g_server_dev.camera_info.num_cameras]);

	g_server_dev.camera_info.s_mount_angle
	[g_server_dev.camera_info.num_cameras]
	= sdata->sensor_platform_info->mount_angle;

	g_server_dev.camera_info.is_internal_cam
	[g_server_dev.camera_info.num_cameras]
	= sdata->camera_type;

	g_server_dev.mctl_node_info.mctl_node_name
	[g_server_dev.mctl_node_info.num_mctl_nodes]
	= video_device_node_name(pcam->mctl_node.pvdev);

	pr_info("%s mctl_node_name[%d] = %s\n", __func__,
		g_server_dev.mctl_node_info.num_mctl_nodes,
		g_server_dev.mctl_node_info.mctl_node_name
		[g_server_dev.mctl_node_info.num_mctl_nodes]);

	snprintf(pcam->media_dev.serial,
			sizeof(pcam->media_dev.serial),
			"%s-%d-%d", QCAMERA_NAME,
			sdata->sensor_platform_info->mount_angle,
			sdata->camera_type);

	g_server_dev.camera_info.num_cameras++;
	g_server_dev.mctl_node_info.num_mctl_nodes++;

	D("%s done, rc = %d\n", __func__, rc);
	D("%s number of sensors connected is %d\n", __func__,
		g_server_dev.camera_info.num_cameras);

	
	if(actctrl) {
		
		actctrl->actrl_vcm_on_mut = &cam_vcm_on_mut;
		
		actctrl->actrl_vcm_wa_camera_on = &camera_on;

		
		if(actctrl->actuator_poweroff_af) {
			g_vcm_wa_ctl.actuator_power_off_af = actctrl->actuator_poweroff_af;
		}

		
		if(actctrl->do_vcm_on_cb) {
			g_vcm_wa_ctl.cam_do_vcm_on_cb = actctrl->do_vcm_on_cb;
		}
		if(actctrl->do_vcm_off_cb) {
			g_vcm_wa_ctl.cam_do_vcm_off_cb = actctrl->do_vcm_off_cb;
		}
	}

	
	if(sdata->actuator_info) {
		if(sdata->actuator_info->vcm_wa_vreg_off)
			g_vcm_wa_ctl.vcm_vreg_off = sdata->actuator_info->vcm_wa_vreg_off;
	}
	

	
	rc = msm_cam_register_subdev_node(sensor_sd, SENSOR_DEV, vnode_count);
	if (rc < 0) {
		D("%s sensor sub device register failed\n",
			__func__);
		goto failure;
	}

	if (sdata->actuator_info) {
		rc = v4l2_device_register_subdev(&pcam->v4l2_dev, act_sdev);
		if (rc < 0) {
			D("%s actuator sub device register failed\n",
			  __func__);
			goto failure;
		}
	}

	pcam->vnode_id = vnode_count++;
	return rc;

failure:
	kfree(act_sdev);
	kzfree(pcam);
	return rc;
}
EXPORT_SYMBOL(msm_sensor_register);

static struct switch_dev htccallback_switch = {
	.name = "htccallback",
};

static struct kobject *htccallback_obj;
static struct kobject *camera_attrs_obj;

static uint32_t htccallback_value;
static uint32_t videochat_value;

static ssize_t htccallback_set(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
	char *tmp;
	htccallback_value = simple_strtoul(buf, &tmp, 0);

	switch_set_state(&htccallback_switch, htccallback_value);

       pr_info("htccallback_value = %d\n", htccallback_value);

       return count;
}

static ssize_t videochat_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", videochat_value);
	return length;
}

static ssize_t videochat_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	if (buf[1] == '\n')
		tmp = buf[0] - 0x30;

	videochat_value = tmp;
	pr_info("videochat_value = %d\n", videochat_value);
	return count;
}

static DEVICE_ATTR(htccallback, 0660,
    NULL,
    htccallback_set);

static DEVICE_ATTR(videochat, 0660,
    videochat_get,
    videochat_set);

static int msm_camera_sysfs_init(void)
{
	int ret = 0;
	pr_info("htccallback:kobject creat and add\n");

       htccallback_obj = kobject_create_and_add("camera_htccallback", NULL);
       if (htccallback_obj == NULL) {
              pr_info("htccallback: subsystem_register_htccallback failed\n");
              ret = -ENOMEM;
              goto error;
       }

       ret = sysfs_create_file(htccallback_obj,
                  &dev_attr_htccallback.attr);
	if (ret) {
		pr_info("htccallback: sysfs_create_htccallback_file failed\n");
		ret = -EFAULT;
		goto error;
	}
	if (switch_dev_register(&htccallback_switch) < 0) {
		pr_info("htccallback : switch_dev_register error\n");
	}


	pr_info("camera_attrs:kobject creat and add\n");

       camera_attrs_obj = kobject_create_and_add("camera_attrs", NULL);
       if (camera_attrs_obj == NULL) {
              pr_info("camera_attrs: subsystem_register_camera_attrs failed\n");
              ret = -ENOMEM;
              goto error;
       }

       ret = sysfs_create_file(camera_attrs_obj,
                  &dev_attr_videochat.attr);
	if (ret) {
		pr_info("dev_attr_videochat: sysfs_create_dev attr_videochat file failed\n");
		ret = -EFAULT;
		goto error;
	}

	
	INIT_WORK(&cam_vcm_off_work, do_cam_vcm_off_work);
	cam_vcm_off_wq = create_singlethread_workqueue("cam_vcm_off_wq");
	if (!cam_vcm_off_wq) {
		pr_err("%s, create_singlethread_workqueue cam_vcm_off_wq fail\n", __func__);
	}
	memset(&g_vcm_wa_ctl, 0, sizeof(g_vcm_wa_ctl));
	

	return ret;
error:
	kobject_del(htccallback_obj);
	kobject_del(camera_attrs_obj);
	return ret;
}
extern unsigned system_rev;

static int __devinit msm_camera_probe(struct platform_device *pdev)
{
	int rc = 0, i;

	pr_info("system rev = %d",system_rev);
	g_server_dev.config_info.num_config_nodes = 2;

	rc = msm_isp_init_module(g_server_dev.config_info.num_config_nodes);
	if (rc < 0) {
		pr_err("Failed to initialize isp\n");
		return rc;
	}

	if (!msm_class) {
		rc = alloc_chrdev_region(&msm_devno, 0,
		g_server_dev.config_info.num_config_nodes+1, "msm_camera");
		if (rc < 0) {
			pr_err("%s: failed to allocate chrdev: %d\n", __func__,
			rc);
			return rc;
		}

		msm_class = class_create(THIS_MODULE, "msm_camera");
		if (IS_ERR(msm_class)) {
			rc = PTR_ERR(msm_class);
			pr_err("%s: create device class failed: %d\n",
			__func__, rc);
			return rc;
		}
	}

	D("creating server and config nodes\n");
	rc = msm_setup_server_dev(pdev);
	if (rc < 0) {
		pr_err("%s: failed to create server dev: %d\n", __func__,
		rc);
		return rc;
	}

	for (i = 0; i < g_server_dev.config_info.num_config_nodes; i++) {
		rc = msm_setup_config_dev(i, "config");
		if (rc < 0) {
			pr_err("%s:failed to create config dev: %d\n",
			 __func__, rc);
			return rc;
		}
	}

	msm_camera_sysfs_init();

#ifdef CONFIG_PERFLOCK
	perf_lock_init(&g_server_dev.cam_perf_lock, TYPE_PERF_LOCK, PERF_LOCK_HIGHEST, "camera_v4l2");
#endif

	msm_isp_register(&g_server_dev);

	msm_camera_set_rdi0_mctl(NULL);
	msm_camera_set_pix0_mctl(NULL);

	return rc;
}

static int __exit msm_camera_exit(struct platform_device *pdev)
{
	msm_isp_unregister(&g_server_dev);
	return 0;
}


static struct platform_driver msm_cam_server_driver = {
	.probe = msm_camera_probe,
	.remove = msm_camera_exit,
	.driver = {
		.name = "msm_cam_server",
		.owner = THIS_MODULE,
	},
};

static int __init msm_camera_init(void)
{
	return platform_driver_register(&msm_cam_server_driver);
}

static void __exit msm_cam_server_exit(void)
{
	platform_driver_unregister(&msm_cam_server_driver);	
}

module_init(msm_camera_init);
module_exit(msm_cam_server_exit);

