
/* Copyright (c) 2008-2012, Code Aurora Forum. All rights reserved.
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
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <mach/debug_display.h>

#include <asm/system.h>
#include <asm/mach-types.h>

#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/clk.h>
#include <mach/dma.h>

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mdp.h"
#include "mdp4.h"

static struct completion dsi_dma_comp;
static struct completion dsi_mdp_comp;
static struct dsi_buf dsi_tx_buf;
static struct dsi_buf dsi_rx_buf;
static spinlock_t dsi_irq_lock;
static spinlock_t dsi_mdp_lock;
spinlock_t dsi_clk_lock;
static int dsi_ctrl_lock;
static int dsi_mdp_busy;
static struct mutex cmd_mutex;
static struct mutex clk_mutex;

static struct list_head pre_kickoff_list;
static struct list_head post_kickoff_list;

enum {
	STAT_DSI_START,
	STAT_DSI_ERROR,
	STAT_DSI_CMD,
	STAT_DSI_MDP
};

struct dcs_cmd_list	cmdlist;

#ifdef CONFIG_FB_MSM_MDP40
void mipi_dsi_mdp_stat_inc(int which)
{
	switch (which) {
	case STAT_DSI_START:
		mdp4_stat.dsi_mdp_start++;
		break;
	case STAT_DSI_ERROR:
		mdp4_stat.intr_dsi_err++;
		break;
	case STAT_DSI_CMD:
		mdp4_stat.intr_dsi_cmd++;
		break;
	case STAT_DSI_MDP:
		mdp4_stat.intr_dsi_mdp++;
		break;
	default:
		break;
	}
}
#else
void mipi_dsi_mdp_stat_inc(int which)
{
}
#endif

void mipi_dsi_init(void)
{
	init_completion(&dsi_dma_comp);
	init_completion(&dsi_mdp_comp);
	mipi_dsi_buf_alloc(&dsi_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&dsi_rx_buf, DSI_BUF_SIZE);
	spin_lock_init(&dsi_irq_lock);
	spin_lock_init(&dsi_mdp_lock);
	spin_lock_init(&dsi_clk_lock);
	mutex_init(&cmd_mutex);
	mutex_init(&clk_mutex);

	INIT_LIST_HEAD(&pre_kickoff_list);
	INIT_LIST_HEAD(&post_kickoff_list);
}


static u32 dsi_irq_mask;

void mipi_dsi_enable_irq(u32 term)
{
	unsigned long flags;

	spin_lock_irqsave(&dsi_irq_lock, flags);
	if (dsi_irq_mask & term) {
		spin_unlock_irqrestore(&dsi_irq_lock, flags);
		return;
	}
	if (dsi_irq_mask == 0) {
		enable_irq(dsi_irq);
		pr_debug("%s: IRQ Enable, mask=%x term=%x\n",
				__func__, (int)dsi_irq_mask, (int)term);
	}
	dsi_irq_mask |= term;
	spin_unlock_irqrestore(&dsi_irq_lock, flags);
}

void mipi_dsi_disable_irq(u32 term)
{
	unsigned long flags;

	spin_lock_irqsave(&dsi_irq_lock, flags);
	if (!(dsi_irq_mask & term)) {
		spin_unlock_irqrestore(&dsi_irq_lock, flags);
		return;
	}
	dsi_irq_mask &= ~term;
	if (dsi_irq_mask == 0) {
		disable_irq(dsi_irq);
		pr_debug("%s: IRQ Disable, mask=%x term=%x\n",
				__func__, (int)dsi_irq_mask, (int)term);
	}
	spin_unlock_irqrestore(&dsi_irq_lock, flags);
}

void mipi_dsi_disable_irq_nosync(u32 term)
{
	spin_lock(&dsi_irq_lock);
	if (!(dsi_irq_mask & term)) {
		spin_unlock(&dsi_irq_lock);
		return;
	}
	dsi_irq_mask &= ~term;
	if (dsi_irq_mask == 0) {
		disable_irq_nosync(dsi_irq);
		pr_debug("%s: IRQ Disable, mask=%x term=%x\n",
				__func__, (int)dsi_irq_mask, (int)term);
	}
	spin_unlock(&dsi_irq_lock);
}

static int dsi_clk_cnt;
static int dsi_clk_on_aux;

void mipi_dsi_clk_turn_on(struct msm_panel_info const *pinfo, int target_type)
{
	mutex_lock(&clk_mutex);

	if (!dsi_clk_on_aux && !dsi_clk_cnt) {
		mipi_dsi_prepare_clocks();
		mipi_dsi_ahb_ctrl(1);

		mipi_dsi_phy_ctrl(1);
		mipi_dsi_phy_init(0, pinfo, target_type);

		mipi_dsi_clk_enable();

		dsi_clk_on_aux = 1;
		dsi_clk_cnt = 0;
	}

	mutex_unlock(&clk_mutex);
}

void mipi_dsi_clk_turn_off()
{
	mutex_lock(&clk_mutex);

	pr_debug("%s: turn off dsi clk and dsi engine, cnt = %d\n",
	    __func__, dsi_clk_cnt);

	mipi_dsi_clk_disable();

	
	MIPI_OUTP(MIPI_DSI_BASE + 0x0000, 0);

	mipi_dsi_phy_ctrl(0);
	mipi_dsi_ahb_ctrl(0);
	mipi_dsi_unprepare_clocks();

	dsi_clk_on_aux = 0;
	dsi_clk_cnt = 0;

	mutex_unlock(&clk_mutex);
}

void mipi_dsi_clk_cfg(int on)
{
	mutex_lock(&clk_mutex);
	if (on) {
		if (dsi_clk_on_aux) {
			dsi_clk_on_aux = 0;
		} else if (dsi_clk_cnt == 0) {
			mipi_dsi_prepare_clocks();
			mipi_dsi_ahb_ctrl(1);
			mipi_dsi_clk_enable();
		}
		dsi_clk_cnt++;
	} else {
		if (dsi_clk_cnt || dsi_clk_on_aux) {
			if (dsi_clk_cnt)
				dsi_clk_cnt--;
			if (dsi_clk_cnt == 0) {
				mipi_dsi_clk_disable();
				mipi_dsi_ahb_ctrl(0);
				mipi_dsi_unprepare_clocks();
			}
			dsi_clk_on_aux = 0;
		}
	}
	pr_debug("%s: on=%d clk_cnt=%d pid=%d\n", __func__,
				on, dsi_clk_cnt, current->pid);
	mutex_unlock(&clk_mutex);
}

void mipi_dsi_turn_on_clks(void)
{
	mipi_dsi_ahb_ctrl(1);
	mipi_dsi_clk_enable();
}

void mipi_dsi_turn_off_clks(void)
{
	mipi_dsi_clk_disable();
	mipi_dsi_ahb_ctrl(0);
}

static void mipi_dsi_action(struct list_head *act_list)
{
	struct list_head *lp;
	struct dsi_kickoff_action *act;

	list_for_each(lp, act_list) {
		act = list_entry(lp, struct dsi_kickoff_action, act_entry);
		if (act && act->action)
			act->action(act->data);
	}
}

void mipi_dsi_pre_kickoff_action(void)
{
	mipi_dsi_action(&pre_kickoff_list);
}

void mipi_dsi_post_kickoff_action(void)
{
	mipi_dsi_action(&post_kickoff_list);
}

void mipi_dsi_pre_kickoff_add(struct dsi_kickoff_action *act)
{
	if (act)
		list_add_tail(&act->act_entry, &pre_kickoff_list);
}

void mipi_dsi_post_kickoff_add(struct dsi_kickoff_action *act)
{
	if (act)
		list_add_tail(&act->act_entry, &post_kickoff_list);
}

void mipi_dsi_pre_kickoff_del(struct dsi_kickoff_action *act)
{
	if (!list_empty(&pre_kickoff_list) && act)
		list_del(&act->act_entry);
}

void mipi_dsi_post_kickoff_del(struct dsi_kickoff_action *act)
{
	if (!list_empty(&post_kickoff_list) && act)
		list_del(&act->act_entry);
}

char *mipi_dsi_buf_reserve(struct dsi_buf *dp, int len)
{
	dp->data += len;
	return dp->data;
}

char *mipi_dsi_buf_unreserve(struct dsi_buf *dp, int len)
{
	dp->data -= len;
	return dp->data;
}

char *mipi_dsi_buf_push(struct dsi_buf *dp, int len)
{
	dp->data -= len;
	dp->len += len;
	return dp->data;
}

char *mipi_dsi_buf_reserve_hdr(struct dsi_buf *dp, int hlen)
{
	dp->hdr = (uint32 *)dp->data;
	return mipi_dsi_buf_reserve(dp, hlen);
}

char *mipi_dsi_buf_init(struct dsi_buf *dp)
{
	int off;

	dp->data = dp->start;
	off = (int)dp->data;
	
	off &= 0x07;
	if (off)
		off = 8 - off;
	dp->data += off;
	dp->len = 0;
	return dp->data;
}

int mipi_dsi_buf_alloc(struct dsi_buf *dp, int size)
{

	dp->start = kmalloc(size, GFP_KERNEL);
	if (dp->start == NULL) {
		pr_err("%s:%u\n", __func__, __LINE__);
		return -ENOMEM;
	}

	dp->end = dp->start + size;
	dp->size = size;

	if ((int)dp->start & 0x07)
		pr_err("%s: buf NOT 8 bytes aligned\n", __func__);

	dp->data = dp->start;
	dp->len = 0;
	return size;
}

static int mipi_dsi_generic_lwrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	char *bp;
	uint32 *hp;
	int i, len;

	bp = mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);

	
	if (cm->payload) {
		len = cm->dlen;
		len += 3;
		len &= ~0x03;	
		for (i = 0; i < cm->dlen; i++)
			*bp++ = cm->payload[i];

		
		for (; i < len; i++)
			*bp++ = 0xff;

		dp->len += len;
	}

	
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(cm->dlen);
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_DTYPE(DTYPE_GEN_LWRITE);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;
}

static int mipi_dsi_generic_swrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;
	int len;

	if (cm->dlen && cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return 0;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	if (cm->last)
		*hp |= DSI_HDR_LAST;


	len = (cm->dlen > 2) ? 2 : cm->dlen;

	if (len == 1) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_WRITE1);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(0);
	} else if (len == 2) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_WRITE2);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(cm->payload[1]);
	} else {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_WRITE);
		*hp |= DSI_HDR_DATA1(0);
		*hp |= DSI_HDR_DATA2(0);
	}

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	
}

static int mipi_dsi_generic_read(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;
	int len;

	if (cm->dlen && cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return 0;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_BTA;
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	len = (cm->dlen > 2) ? 2 : cm->dlen;

	if (len == 1) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_READ1);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(0);
	} else if (len == 2) {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_READ2);
		*hp |= DSI_HDR_DATA1(cm->payload[0]);
		*hp |= DSI_HDR_DATA2(cm->payload[1]);
	} else {
		*hp |= DSI_HDR_DTYPE(DTYPE_GEN_READ);
		*hp |= DSI_HDR_DATA1(0);
		*hp |= DSI_HDR_DATA2(0);
	}

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return dp->len;	
}

static int mipi_dsi_dcs_lwrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	char *bp;
	uint32 *hp;
	int i, len;

	bp = mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);

	if (cm->payload) {
		len = cm->dlen;
		len += 3;
		len &= ~0x03;	
		for (i = 0; i < cm->dlen; i++)
			*bp++ = cm->payload[i];

		
		for (; i < len; i++)
			*bp++ = 0xff;

		dp->len += len;
	}

	
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(cm->dlen);
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_LWRITE);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;
}

static int mipi_dsi_dcs_swrite(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;
	int len;

	if (cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return -EINVAL;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	if (cm->ack)		
		*hp |= DSI_HDR_BTA;
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	len = (cm->dlen > 1) ? 1 : cm->dlen;

	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_WRITE);
	*hp |= DSI_HDR_DATA1(cm->payload[0]);	
	*hp |= DSI_HDR_DATA2(0);

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);
	return dp->len;
}

static int mipi_dsi_dcs_swrite1(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	if (cm->dlen < 2 || cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return -EINVAL;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	if (cm->ack)		
		*hp |= DSI_HDR_BTA;
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_WRITE1);
	*hp |= DSI_HDR_DATA1(cm->payload[0]);	
	*hp |= DSI_HDR_DATA2(cm->payload[1]);	

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;
}

static int mipi_dsi_dcs_read(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	if (cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return -EINVAL;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_BTA;
	*hp |= DSI_HDR_DTYPE(DTYPE_DCS_READ);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	*hp |= DSI_HDR_DATA1(cm->payload[0]);	
	*hp |= DSI_HDR_DATA2(0);

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	
}

static int mipi_dsi_cm_on(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_CM_ON);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	
}

static int mipi_dsi_cm_off(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_CM_OFF);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	
}

static int mipi_dsi_peripheral_on(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_PERIPHERAL_ON);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	
}

static int mipi_dsi_peripheral_off(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_PERIPHERAL_OFF);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	
}

static int mipi_dsi_set_max_pktsize(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	if (cm->payload == 0) {
		pr_err("%s: NO payload error\n", __func__);
		return 0;
	}

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_MAX_PKTSIZE);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	*hp |= DSI_HDR_DATA1(cm->payload[0]);
	*hp |= DSI_HDR_DATA2(cm->payload[1]);

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	
}

static int mipi_dsi_null_pkt(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(cm->dlen);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_NULL_PKT);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	
}

static int mipi_dsi_blank_pkt(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	uint32 *hp;

	mipi_dsi_buf_reserve_hdr(dp, DSI_HOST_HDR_SIZE);
	hp = dp->hdr;
	*hp = 0;
	*hp = DSI_HDR_WC(cm->dlen);
	*hp |= DSI_HDR_LONG_PKT;
	*hp |= DSI_HDR_VC(cm->vc);
	*hp |= DSI_HDR_DTYPE(DTYPE_BLANK_PKT);
	if (cm->last)
		*hp |= DSI_HDR_LAST;

	mipi_dsi_buf_push(dp, DSI_HOST_HDR_SIZE);

	return dp->len;	
}

int mipi_dsi_cmd_dma_add(struct dsi_buf *dp, struct dsi_cmd_desc *cm)
{
	int len = 0;

	switch (cm->dtype) {
	case DTYPE_GEN_WRITE:
	case DTYPE_GEN_WRITE1:
	case DTYPE_GEN_WRITE2:
		len = mipi_dsi_generic_swrite(dp, cm);
		break;
	case DTYPE_GEN_LWRITE:
		len = mipi_dsi_generic_lwrite(dp, cm);
		break;
	case DTYPE_GEN_READ:
	case DTYPE_GEN_READ1:
	case DTYPE_GEN_READ2:
		len = mipi_dsi_generic_read(dp, cm);
		break;
	case DTYPE_DCS_LWRITE:
		len = mipi_dsi_dcs_lwrite(dp, cm);
		break;
	case DTYPE_DCS_WRITE:
		len = mipi_dsi_dcs_swrite(dp, cm);
		break;
	case DTYPE_DCS_WRITE1:
		len = mipi_dsi_dcs_swrite1(dp, cm);
		break;
	case DTYPE_DCS_READ:
		len = mipi_dsi_dcs_read(dp, cm);
		break;
	case DTYPE_MAX_PKTSIZE:
		len = mipi_dsi_set_max_pktsize(dp, cm);
		break;
	case DTYPE_NULL_PKT:
		len = mipi_dsi_null_pkt(dp, cm);
		break;
	case DTYPE_BLANK_PKT:
		len = mipi_dsi_blank_pkt(dp, cm);
		break;
	case DTYPE_CM_ON:
		len = mipi_dsi_cm_on(dp, cm);
		break;
	case DTYPE_CM_OFF:
		len = mipi_dsi_cm_off(dp, cm);
		break;
	case DTYPE_PERIPHERAL_ON:
		len = mipi_dsi_peripheral_on(dp, cm);
		break;
	case DTYPE_PERIPHERAL_OFF:
		len = mipi_dsi_peripheral_off(dp, cm);
		break;
	default:
		pr_debug("%s: dtype=%x NOT supported\n",
					__func__, cm->dtype);
		break;

	}

	return len;
}

static int mipi_dsi_short_read1_resp(struct dsi_buf *rp)
{
	
	rp->data++;
	rp->len = 1;
	return rp->len;
}

static int mipi_dsi_short_read2_resp(struct dsi_buf *rp)
{
	
	rp->data++;
	rp->len = 2;
	return rp->len;
}

static int mipi_dsi_long_read_resp(struct dsi_buf *rp)
{
	short len;

	len = rp->data[2];
	len <<= 8;
	len |= rp->data[1];
	
	rp->data += 4;
	rp->len -= 4;
	
	rp->len -= 2;
	return len;
}

void mipi_dsi_host_init(struct mipi_panel_info *pinfo)
{
	uint32 dsi_ctrl, intr_ctrl;
	uint32 data;

	if (mdp_rev > MDP_REV_41 || mdp_rev == MDP_REV_303)
		pinfo->rgb_swap = DSI_RGB_SWAP_RGB;
	else
		pinfo->rgb_swap = DSI_RGB_SWAP_BGR;

	if (pinfo->mode == DSI_VIDEO_MODE) {
		data = 0;
		if (pinfo->pulse_mode_hsa_he)
			data |= BIT(28);
		if (pinfo->hfp_power_stop)
			data |= BIT(24);
		if (pinfo->hbp_power_stop)
			data |= BIT(20);
		if (pinfo->hsa_power_stop)
			data |= BIT(16);
		if (pinfo->eof_bllp_power_stop)
			data |= BIT(15);
		if (pinfo->bllp_power_stop)
			data |= BIT(12);
		data |= ((pinfo->traffic_mode & 0x03) << 8);
		data |= ((pinfo->dst_format & 0x03) << 4); 
		data |= (pinfo->vc & 0x03);
		MIPI_OUTP(MIPI_DSI_BASE + 0x000c, data);

		data = 0;
		data |= ((pinfo->rgb_swap & 0x07) << 12);
		if (pinfo->b_sel)
			data |= BIT(8);
		if (pinfo->g_sel)
			data |= BIT(4);
		if (pinfo->r_sel)
			data |= BIT(0);
		MIPI_OUTP(MIPI_DSI_BASE + 0x001c, data);
	} else if (pinfo->mode == DSI_CMD_MODE) {
		data = 0;
		data |= ((pinfo->interleave_max & 0x0f) << 20);
		data |= ((pinfo->rgb_swap & 0x07) << 16);
		if (pinfo->b_sel)
			data |= BIT(12);
		if (pinfo->g_sel)
			data |= BIT(8);
		if (pinfo->r_sel)
			data |= BIT(4);
		data |= (pinfo->dst_format & 0x0f);	
		MIPI_OUTP(MIPI_DSI_BASE + 0x003c, data);

		
		data = pinfo->wr_mem_continue & 0x0ff;
		data <<= 8;
		data |= (pinfo->wr_mem_start & 0x0ff);
		if (pinfo->insert_dcs_cmd)
			data |= BIT(16);
		MIPI_OUTP(MIPI_DSI_BASE + 0x0040, data);
	} else
		pr_err("%s: Unknown DSI mode=%d\n", __func__, pinfo->mode);

	dsi_ctrl = BIT(8) | BIT(2);	
	intr_ctrl = 0;
	intr_ctrl = (DSI_INTR_CMD_DMA_DONE_MASK | DSI_INTR_CMD_MDP_DONE_MASK);

	if (pinfo->crc_check)
		dsi_ctrl |= BIT(24);
	if (pinfo->ecc_check)
		dsi_ctrl |= BIT(20);
	if (pinfo->data_lane3)
		dsi_ctrl |= BIT(7);
	if (pinfo->data_lane2)
		dsi_ctrl |= BIT(6);
	if (pinfo->data_lane1)
		dsi_ctrl |= BIT(5);
	if (pinfo->data_lane0)
		dsi_ctrl |= BIT(4);

	
	
	MIPI_OUTP(MIPI_DSI_BASE + 0x38, 0x14000000);

	data = 0;
	if (pinfo->te_sel)
		data |= BIT(31);
	data |= pinfo->mdp_trigger << 4;
	data |= pinfo->dma_trigger;	
	data |= (pinfo->stream & 0x01) << 8;
	MIPI_OUTP(MIPI_DSI_BASE + 0x0080, data); 

	
	MIPI_OUTP(MIPI_DSI_BASE + 0x00ac, pinfo->dlane_swap);

	
	data = pinfo->t_clk_post & 0x3f;	
	data <<= 8;
	data |= pinfo->t_clk_pre & 0x3f;	
	MIPI_OUTP(MIPI_DSI_BASE + 0xc0, data);	

	data = 0;
	if (pinfo->rx_eot_ignore)
		data |= BIT(4);
	if (pinfo->tx_eot_append)
		data |= BIT(0);
	MIPI_OUTP(MIPI_DSI_BASE + 0x00c8, data); 


	
	MIPI_OUTP(MIPI_DSI_BASE + 0x0108, 0x13ff3fe0); 

	intr_ctrl |= DSI_INTR_ERROR_MASK;
	MIPI_OUTP(MIPI_DSI_BASE + 0x010c, intr_ctrl); 

	
	if (mdp_rev >= MDP_REV_41)
		MIPI_OUTP(MIPI_DSI_BASE + 0x118, 0x23f); 
	else
		MIPI_OUTP(MIPI_DSI_BASE + 0x118, 0x33f); 

	dsi_ctrl |= BIT(0);	
	MIPI_OUTP(MIPI_DSI_BASE + 0x0000, dsi_ctrl);

	wmb();
}

void mipi_set_tx_power_mode(int mode)
{
	uint32 data = MIPI_INP(MIPI_DSI_BASE + 0x38);

	if (mode == 0)
		data &= ~BIT(26);
	else
		data |= BIT(26);

	MIPI_OUTP(MIPI_DSI_BASE + 0x38, data);
}

void mipi_dsi_sw_reset(void)
{
	MIPI_OUTP(MIPI_DSI_BASE + 0x114, 0x01);
	wmb();
	MIPI_OUTP(MIPI_DSI_BASE + 0x114, 0x00);
	wmb();
}

void mipi_dsi_controller_cfg(int enable)
{

	uint32 dsi_ctrl;
	uint32 status;
	int cnt;

	cnt = 16;
	while (cnt--) {
		status = MIPI_INP(MIPI_DSI_BASE + 0x0004);
		status &= 0x02;		
		if (status == 0)
			break;
		usleep(1000);
	}
	if (cnt == 0)
		pr_info("%s: DSI status=%x failed\n", __func__, status);

	cnt = 16;
	while (cnt--) {
		status = MIPI_INP(MIPI_DSI_BASE + 0x0008);
		status &= 0x11111000;	
		if (status == 0x11111000)	
			break;
		usleep(1000);
	}

	if (cnt == 0)
		pr_info("%s: FIFO status=%x failed\n", __func__, status);

	dsi_ctrl = MIPI_INP(MIPI_DSI_BASE + 0x0000);
	if (enable)
		dsi_ctrl |= 0x01;
	else
		dsi_ctrl &= ~0x01;

	MIPI_OUTP(MIPI_DSI_BASE + 0x0000, dsi_ctrl);
	wmb();
}

void mipi_dsi_op_mode_config(int mode)
{

	uint32 dsi_ctrl, intr_ctrl;

	dsi_ctrl = MIPI_INP(MIPI_DSI_BASE + 0x0000);
	dsi_ctrl &= ~0x07;
	if (mode == DSI_VIDEO_MODE) {
		dsi_ctrl |= 0x03;
		intr_ctrl = DSI_INTR_CMD_DMA_DONE_MASK;
	} else {		
		dsi_ctrl |= 0x05;
		intr_ctrl = DSI_INTR_CMD_DMA_DONE_MASK | DSI_INTR_ERROR_MASK |
				DSI_INTR_CMD_MDP_DONE_MASK;
	}

	pr_debug("%s: dsi_ctrl=%x intr=%x\n", __func__, dsi_ctrl, intr_ctrl);

	MIPI_OUTP(MIPI_DSI_BASE + 0x010c, intr_ctrl); 
	MIPI_OUTP(MIPI_DSI_BASE + 0x0000, dsi_ctrl);
	wmb();
}

void mipi_dsi_mdp_busy_wait(void)
{
	mutex_lock(&cmd_mutex);
	mipi_dsi_cmd_mdp_busy();
	mutex_unlock(&cmd_mutex);
}

void mipi_dsi_cmd_mdp_start(void)
{
	unsigned long flag;

	mipi_dsi_mdp_stat_inc(STAT_DSI_START);

	spin_lock_irqsave(&dsi_mdp_lock, flag);
	mipi_dsi_enable_irq(DSI_MDP_TERM);
	dsi_mdp_busy = TRUE;
	INIT_COMPLETION(dsi_mdp_comp);
	spin_unlock_irqrestore(&dsi_mdp_lock, flag);
}

void mipi_dsi_cmd_bta_sw_trigger(void)
{
	uint32 data;
	int cnt = 0;

	MIPI_OUTP(MIPI_DSI_BASE + 0x094, 0x01);	
	wmb();

	while (cnt < 10000) {
		data = MIPI_INP(MIPI_DSI_BASE + 0x0004);
		if ((data & 0x0010) == 0)
			break;
		cnt++;
	}

	mipi_dsi_ack_err_status();

	pr_debug("%s: BTA done, cnt=%d\n", __func__, cnt);
}

static char set_tear_on[2] = {0x35, 0x00};
static struct dsi_cmd_desc dsi_tear_on_cmd = {
	DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(set_tear_on), set_tear_on};

static char set_tear_off[2] = {0x34, 0x00};
static struct dsi_cmd_desc dsi_tear_off_cmd = {
	DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(set_tear_off), set_tear_off};

static struct dcs_cmd_req cmdreq;

void mipi_dsi_set_tear_on(struct msm_fb_data_type *mfd)
{

	cmdreq.cmds = &dsi_tear_on_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);
}

void mipi_dsi_set_tear_off(struct msm_fb_data_type *mfd)
{
	cmdreq.cmds = &dsi_tear_off_cmd;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mipi_dsi_cmdlist_put(&cmdreq);
}

int mipi_dsi_cmd_reg_tx(uint32 data)
{
#ifdef DSI_HOST_DEBUG
	int i;
	char *bp;

	bp = (char *)&data;
	pr_debug("%s: ", __func__);
	for (i = 0; i < 4; i++)
		pr_debug("%x ", *bp++);

	pr_debug("\n");
#endif

	MIPI_OUTP(MIPI_DSI_BASE + 0x0080, 0x04);
	MIPI_OUTP(MIPI_DSI_BASE + 0x0, 0x135);

	wmb();

	MIPI_OUTP(MIPI_DSI_BASE + 0x038, data);
	wmb();
	MIPI_OUTP(MIPI_DSI_BASE + 0x08c, 0x01);	
	wmb();

	udelay(300);

	return 4;
}

int mipi_dsi_cmds_tx(struct dsi_buf *tp, struct dsi_cmd_desc *cmds, int cnt)
{
	struct dsi_cmd_desc *cm;
	uint32 dsi_ctrl, ctrl;
	int i, video_mode;

	dsi_ctrl = MIPI_INP(MIPI_DSI_BASE + 0x0000);
	video_mode = dsi_ctrl & 0x02; 
	if (video_mode) {
		ctrl = dsi_ctrl | 0x04; 
		MIPI_OUTP(MIPI_DSI_BASE + 0x0000, ctrl);
	}

	cm = cmds;
	mipi_dsi_buf_init(tp);
	for (i = 0; i < cnt; i++) {
		mipi_dsi_enable_irq(DSI_CMD_TERM);
		mipi_dsi_buf_init(tp);
		mipi_dsi_cmd_dma_add(tp, cm);
		mipi_dsi_cmd_dma_tx(tp);
		if (cm->wait)
			msleep(cm->wait);
		cm++;
	}

	if (video_mode)
		MIPI_OUTP(MIPI_DSI_BASE + 0x0000, dsi_ctrl); 

	return cnt;
}

static char max_pktsize[2] = {0x00, 0x00}; 

static struct dsi_cmd_desc pkt_size_cmd[] = {
	{DTYPE_MAX_PKTSIZE, 1, 0, 0, 0,
		sizeof(max_pktsize), max_pktsize}
};

int mipi_dsi_cmds_rx(struct msm_fb_data_type *mfd,
			struct dsi_buf *tp, struct dsi_buf *rp,
			struct dsi_cmd_desc *cmds, int rlen)
{
	int cnt, len, diff, pkt_size;
	char cmd;

	if (mfd->panel_info.mipi.no_max_pkt_size) {
		
		rlen += 3;
		rlen &= ~0x03;
	}

	len = rlen;
	diff = 0;

	if (len <= 2)
		cnt = 4;	
	else {
		if (len > MIPI_DSI_LEN)
			len = MIPI_DSI_LEN;	

		len = (len + 3) & ~0x03; 
		diff = len - rlen;
		len += 2;
		cnt = len + 6; 
	}

	if (mfd->panel_info.type == MIPI_CMD_PANEL) {
		
#ifdef CONFIG_FB_MSM_MDP303
			mdp3_dsi_cmd_dma_busy_wait(mfd);
#endif
	}

	if (!mfd->panel_info.mipi.no_max_pkt_size) {
		
		pkt_size = len;
		max_pktsize[0] = pkt_size;
		mipi_dsi_enable_irq(DSI_CMD_TERM);
		mipi_dsi_buf_init(tp);
		mipi_dsi_cmd_dma_add(tp, pkt_size_cmd);
		mipi_dsi_cmd_dma_tx(tp);
	}

	mipi_dsi_enable_irq(DSI_CMD_TERM);
	mipi_dsi_buf_init(tp);
	mipi_dsi_cmd_dma_add(tp, cmds);

	
	mipi_dsi_cmd_dma_tx(tp);

	mipi_dsi_disable_irq(DSI_CMD_TERM);
	mipi_dsi_buf_init(rp);
	if (mfd->panel_info.mipi.no_max_pkt_size) {
		rp->data += 2;
	}

	mipi_dsi_cmd_dma_rx(rp, cnt);

	if (mfd->panel_info.mipi.no_max_pkt_size) {
		rp->data += 2;
	}

	cmd = rp->data[0];
	switch (cmd) {
	case DTYPE_ACK_ERR_RESP:
		pr_debug("%s: rx ACK_ERR_PACLAGE\n", __func__);
		break;
	case DTYPE_GEN_READ1_RESP:
	case DTYPE_DCS_READ1_RESP:
		mipi_dsi_short_read1_resp(rp);
		break;
	case DTYPE_GEN_READ2_RESP:
	case DTYPE_DCS_READ2_RESP:
		mipi_dsi_short_read2_resp(rp);
		break;
	case DTYPE_GEN_LREAD_RESP:
	case DTYPE_DCS_LREAD_RESP:
		mipi_dsi_long_read_resp(rp);
		rp->len -= 2; 
		rp->len -= diff; 
		break;
	default:
		break;
	}

	return rp->len;
}

int mipi_dsi_cmds_rx_new(struct dsi_buf *tp, struct dsi_buf *rp,
			struct dcs_cmd_req *req, int rlen)
{
	struct dsi_cmd_desc *cmds;
	int cnt, len, diff, pkt_size;
	char cmd;

	if (req->flags & CMD_REQ_NO_MAX_PKT_SIZE) {
		
		rlen += 3;
		rlen &= ~0x03;
	}

	cmds = req->cmds;

	len = rlen;
	diff = 0;

	if (len <= 2)
		cnt = 4;	
	else {
		if (len > MIPI_DSI_LEN)
			len = MIPI_DSI_LEN;	

		len = (len + 3) & ~0x03; 
		diff = len - rlen;
		len += 2;
		cnt = len + 6; 
	}

	if (!(req->flags & CMD_REQ_NO_MAX_PKT_SIZE)) {


		
		pkt_size = len;
		max_pktsize[0] = pkt_size;
		mipi_dsi_enable_irq(DSI_CMD_TERM);
		mipi_dsi_buf_init(tp);
		mipi_dsi_cmd_dma_add(tp, pkt_size_cmd);
		mipi_dsi_cmd_dma_tx(tp);
	}

	mipi_dsi_enable_irq(DSI_CMD_TERM);
	mipi_dsi_buf_init(tp);
	mipi_dsi_cmd_dma_add(tp, cmds);

	
	mipi_dsi_cmd_dma_tx(tp);

	mipi_dsi_disable_irq(DSI_CMD_TERM);
	mipi_dsi_buf_init(rp);
	if (req->flags & CMD_REQ_NO_MAX_PKT_SIZE) {
		rp->data += 2;
	}

	mipi_dsi_cmd_dma_rx(rp, cnt);

	if (req->flags & CMD_REQ_NO_MAX_PKT_SIZE) {
		rp->data += 2;
	}

	cmd = rp->data[0];
	switch (cmd) {
	case DTYPE_ACK_ERR_RESP:
		pr_debug("%s: rx ACK_ERR_PACLAGE\n", __func__);
		break;
	case DTYPE_GEN_READ1_RESP:
	case DTYPE_DCS_READ1_RESP:
		mipi_dsi_short_read1_resp(rp);
		break;
	case DTYPE_GEN_READ2_RESP:
	case DTYPE_DCS_READ2_RESP:
		mipi_dsi_short_read2_resp(rp);
		break;
	case DTYPE_GEN_LREAD_RESP:
	case DTYPE_DCS_LREAD_RESP:
		mipi_dsi_long_read_resp(rp);
		rp->len -= 2; 
		rp->len -= diff; 
		break;
	default:
		break;
	}

	return rp->len;
}

int dsi_cmd_dma_need_wait;
int mipi_dsi_cmd_dma_tx(struct dsi_buf *tp)
{

	unsigned long flags;
	int ret = 0;

#ifdef DSI_HOST_DEBUG
	int i;
	char *bp;

	bp = tp->data;

	pr_debug("%s: ", __func__);
	for (i = 0; i < tp->len; i++)
		pr_debug("%x ", *bp++);

	pr_debug("\n");
#endif

	if (tp->len == 0) {
		pr_err("%s: Error, len=0\n", __func__);
		return 0;
	}

	spin_lock_irqsave(&dsi_mdp_lock, flags);
	tp->len += 3;
	tp->len &= ~0x03;	

	tp->dmap = dma_map_single(&dsi_dev, tp->data, tp->len, DMA_TO_DEVICE);
	if (dma_mapping_error(&dsi_dev, tp->dmap))
		pr_err("%s: dmap mapp failed\n", __func__);

	INIT_COMPLETION(dsi_dma_comp);

	MIPI_OUTP(MIPI_DSI_BASE + 0x044, tp->dmap);
	MIPI_OUTP(MIPI_DSI_BASE + 0x048, tp->len);
	wmb();
	MIPI_OUTP(MIPI_DSI_BASE + 0x08c, 0x01);	
	wmb();
	dsi_cmd_dma_need_wait++;
	spin_unlock_irqrestore(&dsi_mdp_lock, flags);

	if (dsi_cmd_dma_need_wait) {
		ret = wait_for_completion_timeout(&dsi_dma_comp, HZ/20);
		if (ret <= 0) {
			pr_info("%s: wait for dsi_dma complete timeout (ret=%d, busy=%d, stat=0x%x)\n",
			    __func__, ret, dsi_cmd_dma_need_wait, MIPI_INP(MIPI_DSI_BASE + 0x0004));
		}
	}

	dma_unmap_single(&dsi_dev, tp->dmap, tp->len, DMA_TO_DEVICE);
	tp->dmap = 0;
	return tp->len;
}

int mipi_dsi_cmd_dma_rx(struct dsi_buf *rp, int rlen)
{
	uint32 *lp, data;
	int i, off, cnt;

	lp = (uint32 *)rp->data;
	cnt = rlen;
	cnt += 3;
	cnt >>= 2;

	if (cnt > 4)
		cnt = 4; 

	off = 0x068;	
	off += ((cnt - 1) * 4);


	for (i = 0; i < cnt; i++) {
		data = (uint32)MIPI_INP(MIPI_DSI_BASE + off);
		*lp++ = ntohl(data);	
		off -= 4;
		rp->len += sizeof(*lp);
	}

	return rlen;
}

void mipi_dsi_video_wait_to_mdp_busy(void)
{
	u32 status;

	status = MIPI_INP(MIPI_DSI_BASE + 0x0004);
	if (!(status & 0x08))	
		usleep(4000);
}

void mipi_dsi_cmd_mdp_busy(void)
{
	unsigned long flags;
	int need_wait = 0;
	int timeoutResult = 0;
	static int cnt = 0;

	pr_debug("%s: start pid=%d\n", __func__, current->pid);
	spin_lock_irqsave(&dsi_mdp_lock, flags);
	if (dsi_mdp_busy == TRUE)
		need_wait++;
	spin_unlock_irqrestore(&dsi_mdp_lock, flags);

	if (need_wait) {
		
		pr_debug("%s: pending pid=%d\n",
				__func__, current->pid);
		timeoutResult = wait_for_completion_timeout(&dsi_mdp_comp, HZ/10);
		if (!timeoutResult) {
			PR_DISP_WARN("%s:wait_for_completion\n",__func__);
			if (cnt > 2)
				PR_DISP_WARN("%s:still timeout\n",__func__);
			cnt++;
		} else {
			cnt = 0;
		}
	}
	pr_debug("%s: done pid=%d\n",
				__func__, current->pid);
}

struct dcs_cmd_req *mipi_dsi_cmdlist_get(void)
{
	struct dcs_cmd_req *req = NULL;

	if (cmdlist.get != cmdlist.put) {
		req = &cmdlist.list[cmdlist.get];
		cmdlist.get++;
		cmdlist.get %= CMD_REQ_MAX;
		cmdlist.tot--;
		pr_debug("%s: tot=%d put=%d get=%d\n", __func__,
		cmdlist.tot, cmdlist.put, cmdlist.get);
	}
	return req;
}
void mipi_dsi_cmdlist_tx(struct dcs_cmd_req *req)
{
	struct dsi_buf *tp;
	int ret;

	mipi_dsi_buf_init(&dsi_tx_buf);
	tp = &dsi_tx_buf;
	ret = mipi_dsi_cmds_tx(tp, req->cmds, req->cmds_cnt);

	if (req->cb)
		req->cb(ret);

}

void mipi_dsi_cmdlist_rx(struct dcs_cmd_req *req)
{
	int len;
	u32 *dp;
	struct dsi_buf *tp;
	struct dsi_buf *rp;

	mipi_dsi_buf_init(&dsi_tx_buf);
	mipi_dsi_buf_init(&dsi_rx_buf);

	tp = &dsi_tx_buf;
	rp = &dsi_rx_buf;

	len = mipi_dsi_cmds_rx_new(tp, rp, req, req->rlen);
	dp = (u32 *)rp->data;

	if (req->cb)
		req->cb(*dp);
}

void mipi_dsi_cmdlist_commit(int from_mdp)
{
	struct dcs_cmd_req *req;
	int video;
	u32 dsi_ctrl;

	mutex_lock(&cmd_mutex);
	req = mipi_dsi_cmdlist_get();

	
	mipi_dsi_cmd_mdp_busy();

	if (req == NULL)
		goto need_lock;

	video = MIPI_INP(MIPI_DSI_BASE + 0x0000);
	video &= 0x02; 

	if (!video)
		mipi_dsi_clk_cfg(1);

	pr_debug("%s:  from_mdp=%d pid=%d\n", __func__, from_mdp, current->pid);

	dsi_ctrl = MIPI_INP(MIPI_DSI_BASE + 0x0000);
	if (dsi_ctrl & 0x02) {
		mipi_dsi_video_wait_to_mdp_busy();
	} else {
		
		if (!from_mdp) { 
			
			mipi_dsi_cmd_mdp_busy();
		}
	}

	if (req->flags & CMD_REQ_RX)
		mipi_dsi_cmdlist_rx(req);
	else
		mipi_dsi_cmdlist_tx(req);

	if (!video)
		mipi_dsi_clk_cfg(0);

need_lock:
	if (from_mdp) 
		mipi_dsi_cmd_mdp_start();

	mutex_unlock(&cmd_mutex);
}

int mipi_dsi_cmdlist_put(struct dcs_cmd_req *cmdreq)
{
	struct dcs_cmd_req *req;
	int ret = 0;

	mutex_lock(&cmd_mutex);
	req = &cmdlist.list[cmdlist.put];
	*req = *cmdreq;
	cmdlist.put++;
	cmdlist.put %= CMD_REQ_MAX;
	cmdlist.tot++;
	if (cmdlist.put == cmdlist.get) {
		
		pr_debug("%s: DROP, tot=%d put=%d get=%d\n", __func__,
			cmdlist.tot, cmdlist.put, cmdlist.get);
		cmdlist.get++;
		cmdlist.get %= CMD_REQ_MAX;
		cmdlist.tot--;
	}
	mutex_unlock(&cmd_mutex);

	ret++;
	pr_debug("%s: tot=%d put=%d get=%d\n", __func__,
		cmdlist.tot, cmdlist.put, cmdlist.get);

	if (req->flags & CMD_REQ_COMMIT)
		mipi_dsi_cmdlist_commit(0);

	return ret;
}

void mipi_dsi_irq_set(uint32 mask, uint32 irq)
{
	uint32 data;

	data = MIPI_INP(MIPI_DSI_BASE + 0x010c);
	data &= ~mask;
	data |= irq;
	MIPI_OUTP(MIPI_DSI_BASE + 0x010c, data);
}

void mipi_dsi_ack_err_status(void)
{
	uint32 status;

	status = MIPI_INP(MIPI_DSI_BASE + 0x0064);

	if (status) {
		MIPI_OUTP(MIPI_DSI_BASE + 0x0064, status);
		pr_debug("%s: status=%x\n", __func__, status);
	}
}

void mipi_dsi_timeout_status(void)
{
	uint32 status;

	status = MIPI_INP(MIPI_DSI_BASE + 0x00bc);
	if (status & 0x0111) {
		MIPI_OUTP(MIPI_DSI_BASE + 0x00bc, status);
		pr_debug("%s: status=%x\n", __func__, status);
	}
}

void mipi_dsi_dln0_phy_err(void)
{
	uint32 status;

	status = MIPI_INP(MIPI_DSI_BASE + 0x00b0);

	if (status & 0x011111) {
		MIPI_OUTP(MIPI_DSI_BASE + 0x00b0, status);
		pr_debug("%s: status=%x\n", __func__, status);
	}
}

void mipi_dsi_fifo_status(void)
{
	uint32 status;

	status = MIPI_INP(MIPI_DSI_BASE + 0x0008);

	if (status & 0x44444489) {
		MIPI_OUTP(MIPI_DSI_BASE + 0x0008, status);
		pr_debug("%s: status=%x\n", __func__, status);
	}
}

void mipi_dsi_status(void)
{
	uint32 status;

	status = MIPI_INP(MIPI_DSI_BASE + 0x0004);

	if (status & 0x80000000) {
		MIPI_OUTP(MIPI_DSI_BASE + 0x0004, status);
		pr_debug("%s: status=%x\n", __func__, status);
	}
}

void mipi_dsi_error(void)
{
	
	mipi_dsi_ack_err_status();	
	mipi_dsi_timeout_status();	
	mipi_dsi_fifo_status();		
	mipi_dsi_status();		
	mipi_dsi_dln0_phy_err();	
}


irqreturn_t mipi_dsi_isr(int irq, void *ptr)
{
	uint32 isr, status;

	isr = MIPI_INP(MIPI_DSI_BASE + 0x010c);
	MIPI_OUTP(MIPI_DSI_BASE + 0x010c, isr);

	
	status = MIPI_INP(MIPI_DSI_BASE + 0x0004);

	pr_debug("%s: isr=%x\n", __func__, (int)isr);

#ifdef CONFIG_FB_MSM_MDP40
	mdp4_stat.intr_dsi++;
#endif
	if (isr & DSI_INTR_ERROR) {
		mipi_dsi_mdp_stat_inc(STAT_DSI_ERROR);
		mipi_dsi_error();
	}

	if (isr & DSI_INTR_VIDEO_DONE) {
	}

	if (isr & DSI_INTR_CMD_DMA_DONE) {
		mipi_dsi_mdp_stat_inc(STAT_DSI_CMD);
		spin_lock(&dsi_mdp_lock);
		complete(&dsi_dma_comp);
		dsi_ctrl_lock = FALSE;
		mipi_dsi_disable_irq_nosync(DSI_CMD_TERM);
		dsi_cmd_dma_need_wait = 0;
		spin_unlock(&dsi_mdp_lock);
	}

	if (isr & DSI_INTR_CMD_MDP_DONE) {
		mipi_dsi_mdp_stat_inc(STAT_DSI_MDP);
		spin_lock(&dsi_mdp_lock);
		dsi_ctrl_lock = FALSE;
		dsi_mdp_busy = FALSE;
		mipi_dsi_disable_irq_nosync(DSI_MDP_TERM);
		complete_all(&dsi_mdp_comp);
		spin_unlock(&dsi_mdp_lock);
	}


	return IRQ_HANDLED;
}
