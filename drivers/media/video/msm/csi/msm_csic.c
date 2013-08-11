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
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <mach/clk.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <media/msm_isp.h>
#include "msm_csic.h"
#include "msm.h"

#define DBG_CSIC 0

#define V4L2_IDENT_CSIC			50004
#define	MIPI_PHY_CONTROL		0x00000000
#define	MIPI_PROTOCOL_CONTROL		0x00000004
#define	MIPI_INTERRUPT_STATUS		0x00000008
#define	MIPI_INTERRUPT_MASK		0x0000000C
#define	MIPI_CAMERA_CNTL		0x00000024
#define	MIPI_CALIBRATION_CONTROL	0x00000018
#define	MIPI_PHY_D0_CONTROL2		0x00000038
#define	MIPI_PHY_D1_CONTROL2		0x0000003C
#define	MIPI_PHY_D2_CONTROL2		0x00000040
#define	MIPI_PHY_D3_CONTROL2		0x00000044
#define	MIPI_PHY_CL_CONTROL		0x00000048
#define	MIPI_PHY_D0_CONTROL		0x00000034
#define	MIPI_PHY_D1_CONTROL		0x00000020
#define	MIPI_PHY_D2_CONTROL		0x0000002C
#define	MIPI_PHY_D3_CONTROL		0x00000030
#define	MIPI_PWR_CNTL			0x00000054


#define	MIPI_PROTOCOL_CONTROL_DPCM_SCHEME_SHFT			0x1e
#define	MIPI_PROTOCOL_CONTROL_SW_RST_BMSK			0x8000000
#define	MIPI_PROTOCOL_CONTROL_LONG_PACKET_HEADER_CAPTURE_BMSK	0x200000
#define	MIPI_PROTOCOL_CONTROL_DATA_FORMAT_SHFT			0x13
#define	MIPI_PROTOCOL_CONTROL_DECODE_ID_BMSK			0x40000
#define	MIPI_PROTOCOL_CONTROL_ECC_EN_BMSK			0x20000


#define	MIPI_CALIBRATION_CONTROL_SWCAL_CAL_EN_SHFT		0x16
#define	MIPI_CALIBRATION_CONTROL_SWCAL_STRENGTH_OVERRIDE_EN_SHFT	0x15
#define	MIPI_CALIBRATION_CONTROL_CAL_SW_HW_MODE_SHFT		0x14
#define	MIPI_CALIBRATION_CONTROL_MANUAL_OVERRIDE_EN_SHFT	0x7

#define	MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT			0x4
#define	MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3

#define	MIPI_PHY_D1_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D1_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D1_CONTROL2_LP_REC_EN_SHFT			0x4
#define	MIPI_PHY_D1_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3

#define	MIPI_PHY_D2_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D2_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D2_CONTROL2_LP_REC_EN_SHFT			0x4
#define	MIPI_PHY_D2_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3

#define	MIPI_PHY_D3_CONTROL2_SETTLE_COUNT_SHFT			0x18
#define	MIPI_PHY_D3_CONTROL2_HS_TERM_IMP_SHFT			0x10
#define	MIPI_PHY_D3_CONTROL2_LP_REC_EN_SHFT			0x4
#define	MIPI_PHY_D3_CONTROL2_ERR_SOT_HS_EN_SHFT			0x3

#define	MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT			0x18
#define	MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT			0x2

#define	MIPI_PHY_D0_CONTROL_HS_REC_EQ_SHFT			0x1c
#define	MIPI_PHY_D1_CONTROL_MIPI_CLK_PHY_SHUTDOWNB_SHFT		0x9
#define	MIPI_PHY_D1_CONTROL_MIPI_DATA_PHY_SHUTDOWNB_SHFT	0x8

#define MSM_AXI_QOS_PREVIEW 200000
#define MSM_AXI_QOS_SNAPSHOT 200000
#define MSM_AXI_QOS_RECORDING 200000

#define MIPI_PWR_CNTL_ENA	0x07
#define MIPI_PWR_CNTL_DIS	0x0

static int msm_csic_config(struct csic_cfg_params *cfg_params)
{
	int rc = 0, count, i;
	uint32_t val = 0;
	struct csic_device *csic_dev;
	struct msm_camera_csi_params *csic_params;
	void __iomem *csicbase;

	csic_dev = v4l2_get_subdevdata(cfg_params->subdev);
	csicbase = csic_dev->base;
	csic_params = cfg_params->parms;

	
	msm_io_w(0x4, csicbase + MIPI_PHY_CONTROL);

	msm_io_w(MIPI_PROTOCOL_CONTROL_SW_RST_BMSK,
		csicbase + MIPI_PROTOCOL_CONTROL);

	val = MIPI_PROTOCOL_CONTROL_LONG_PACKET_HEADER_CAPTURE_BMSK |
		MIPI_PROTOCOL_CONTROL_DECODE_ID_BMSK |
		MIPI_PROTOCOL_CONTROL_ECC_EN_BMSK;
	val |= (uint32_t)(csic_params->data_format) <<
		MIPI_PROTOCOL_CONTROL_DATA_FORMAT_SHFT;
	val |= csic_params->dpcm_scheme <<
		MIPI_PROTOCOL_CONTROL_DPCM_SCHEME_SHFT;
	CDBG("%s MIPI_PROTOCOL_CONTROL val=0x%x\n", __func__, val);
	
	for (count = 1; count <= 10; count++) {
		rc = msm_io_r(csicbase + MIPI_PROTOCOL_CONTROL) & MIPI_PROTOCOL_CONTROL_SW_RST_BMSK;
		if (rc == 0)
			break;
		if (count == 10) {
			pr_err("[CAM] csic SW RESET failed\n");
			return -EINVAL;
		}
		pr_info("[CAM] polling for csic SW RESET status, cnt %d\n", count);
		mdelay(1);
	}
	msm_io_w(val, csicbase + MIPI_PROTOCOL_CONTROL);

	

	val = (csic_params->settle_cnt <<
		MIPI_PHY_D0_CONTROL2_SETTLE_COUNT_SHFT) |
		(0x0F << MIPI_PHY_D0_CONTROL2_HS_TERM_IMP_SHFT) |
		(0x1 << MIPI_PHY_D0_CONTROL2_LP_REC_EN_SHFT) |
		(0x1 << MIPI_PHY_D0_CONTROL2_ERR_SOT_HS_EN_SHFT);
	CDBG("%s MIPI_PHY_D0_CONTROL2 val=0x%x\n", __func__, val);

	
	for(i=0;i < csic_params->lane_cnt;i++)
		msm_io_w(val, csicbase + MIPI_PHY_D0_CONTROL2 + i*4);


	val = (0x0F << MIPI_PHY_CL_CONTROL_HS_TERM_IMP_SHFT) |
		(0x1 << MIPI_PHY_CL_CONTROL_LP_REC_EN_SHFT);
	CDBG("%s MIPI_PHY_CL_CONTROL val=0x%x\n", __func__, val);
	msm_io_w(val, csicbase + MIPI_PHY_CL_CONTROL);

	val = 0 << MIPI_PHY_D0_CONTROL_HS_REC_EQ_SHFT;
	msm_io_w(val, csicbase + MIPI_PHY_D0_CONTROL);

	val = (0x1 << MIPI_PHY_D1_CONTROL_MIPI_CLK_PHY_SHUTDOWNB_SHFT) |
		(0x1 << MIPI_PHY_D1_CONTROL_MIPI_DATA_PHY_SHUTDOWNB_SHFT);
	CDBG("%s MIPI_PHY_D1_CONTROL val=0x%x\n", __func__, val);
	msm_io_w(val, csicbase + MIPI_PHY_D1_CONTROL);

	msm_io_w(0x00000000, csicbase + MIPI_PHY_D2_CONTROL);
	msm_io_w(0x00000000, csicbase + MIPI_PHY_D3_CONTROL);

	
	switch (csic_params->lane_cnt) {
	case 1:
		msm_io_w(csic_params->lane_assign << 8 | 0x4,
			csicbase + MIPI_CAMERA_CNTL);
		break;
	case 2:
		msm_io_w(csic_params->lane_assign << 8 | 0x5,
			csicbase + MIPI_CAMERA_CNTL);
		break;
	case 3:
		msm_io_w(csic_params->lane_assign << 8 | 0x6,
			csicbase + MIPI_CAMERA_CNTL);
		break;
	case 4:
		msm_io_w(csic_params->lane_assign << 8 | 0x7,
			csicbase + MIPI_CAMERA_CNTL);
		break;
	}

	msm_io_w(0xF077F3C0, csicbase + MIPI_INTERRUPT_MASK);
	
	msm_io_w(0xF077F3C0, csicbase + MIPI_INTERRUPT_STATUS);

	return rc;
}

static irqreturn_t msm_csic_irq(int irq_num, void *data)
{
	uint32_t irq;
	struct csic_device *csic_dev = data;

	pr_info("msm_csic_irq: %x\n", (unsigned int)csic_dev->base);
	irq = msm_io_r(csic_dev->base + MIPI_INTERRUPT_STATUS);
	pr_info("%s MIPI_INTERRUPT_STATUS = 0x%x\n", __func__, irq);
	msm_io_w(irq, csic_dev->base + MIPI_INTERRUPT_STATUS);

	
	if ((irq >> 19) & 0x1)
		pr_info("Unsupported packet format is received\n");
	return IRQ_HANDLED;
}

static int msm_csic_subdev_g_chip_ident(struct v4l2_subdev *sd,
			struct v4l2_dbg_chip_ident *chip)
{
	BUG_ON(!chip);
	chip->ident = V4L2_IDENT_CSIC;
	chip->revision = 0;
	return 0;
}

static struct msm_cam_clk_info csic_clk_info[] = {
	{"csi_src_clk", 384000000},
	{"csi_clk", -1},
	{"csi_pclk", -1},
};

static int msm_csic_init(struct v4l2_subdev *sd, uint32_t *csic_version)
{
	int rc = 0;
	struct csic_device *csic_dev;
	struct clk *clk1;
	csic_dev = v4l2_get_subdevdata(sd);
	if (csic_dev == NULL) {
		rc = -ENOMEM;
		return rc;
	}

	csic_dev->base = ioremap(csic_dev->mem->start,
		resource_size(csic_dev->mem));
	if (!csic_dev->base) {
		rc = -ENOMEM;
		return rc;
	}

	clk1 = clk_get(&csic_dev->pdev->dev, "csi_clk");
	if (IS_ERR(clk1)) {
		pr_err("%s: did not get csi_clk\n", __func__);
		return PTR_ERR(clk1);
	}
	rc = clk_reset(clk1, CLK_RESET_ASSERT);
	if (rc) {
		pr_err("%s:csi_clk assert failed\n", __func__);
		clk_put(clk1);
		return rc;
	}
	usleep_range(1000, 1200);
	rc = clk_reset(clk1, CLK_RESET_DEASSERT);
	if (rc) {
		pr_err("%s:csi_clk deassert failed\n", __func__);
		clk_put(clk1);
		return rc;
	}
	clk_put(clk1);

	clk1 = clk_get(&csic_dev->pdev->dev, "csi_pclk");
	if (IS_ERR(clk1)) {
		pr_err("%s: did not get csi_pclk\n", __func__);
		return PTR_ERR(clk1);
	}
	rc = clk_reset(clk1, CLK_RESET_ASSERT);
	if (rc) {
		pr_err("%s:csi_pclk assert failed\n", __func__);
		clk_put(clk1);
		return rc;
	}
	usleep_range(1000, 1200);
	rc = clk_reset(clk1, CLK_RESET_DEASSERT);
	if (rc) {
		pr_err("%s:csi_pclk deassert failed\n", __func__);
		clk_put(clk1);
		return rc;
	}
	clk_put(clk1);

	rc = msm_cam_clk_enable(&csic_dev->pdev->dev, csic_clk_info,
		csic_dev->csic_clk, ARRAY_SIZE(csic_clk_info), 1);
	if (rc < 0) {
		csic_dev->hw_version = 0;
		iounmap(csic_dev->base);
		csic_dev->base = NULL;
		return rc;
	}

#if DBG_CSIC
	enable_irq(csic_dev->irq->start);
#endif

	return 0;
}

static void msm_csic_disable(struct csic_device *csic_dev)
{
	uint32_t val;

	val = 0x0;
	if (csic_dev->base != NULL) {
		CDBG("%s MIPI_PHY_D0_CONTROL2 val=0x%x\n", __func__, val);
		msm_io_w(val, csic_dev->base + MIPI_PHY_D0_CONTROL2);
		msm_io_w(val, csic_dev->base + MIPI_PHY_D1_CONTROL2);
		msm_io_w(val, csic_dev->base + MIPI_PHY_D2_CONTROL2);
		msm_io_w(val, csic_dev->base + MIPI_PHY_D3_CONTROL2);
		CDBG("%s MIPI_PHY_CL_CONTROL val=0x%x\n", __func__, val);
		msm_io_w(val, csic_dev->base + MIPI_PHY_CL_CONTROL);
		msleep(20);
		val = msm_io_r(csic_dev->base + MIPI_PHY_D1_CONTROL);
		val &=
		~((0x1 << MIPI_PHY_D1_CONTROL_MIPI_CLK_PHY_SHUTDOWNB_SHFT)
		|(0x1 << MIPI_PHY_D1_CONTROL_MIPI_DATA_PHY_SHUTDOWNB_SHFT));
		CDBG("%s MIPI_PHY_D1_CONTROL val=0x%x\n", __func__, val);
		msm_io_w(val, csic_dev->base + MIPI_PHY_D1_CONTROL);
		usleep_range(5000, 6000);
		msm_io_w(0x0, csic_dev->base + MIPI_INTERRUPT_MASK);
		msm_io_w(0x0, csic_dev->base + MIPI_INTERRUPT_STATUS);
		msm_io_w(MIPI_PROTOCOL_CONTROL_SW_RST_BMSK,
			csic_dev->base + MIPI_PROTOCOL_CONTROL);

		msm_io_w(0xE400, csic_dev->base + MIPI_CAMERA_CNTL);
	}
}

static int msm_csic_release(struct v4l2_subdev *sd)
{
	struct csic_device *csic_dev;
	csic_dev = v4l2_get_subdevdata(sd);

	msm_csic_disable(csic_dev);
#if DBG_CSIC
	disable_irq(csic_dev->irq->start);
#endif

	msm_cam_clk_enable(&csic_dev->pdev->dev, csic_clk_info,
		csic_dev->csic_clk, ARRAY_SIZE(csic_clk_info), 0);

	iounmap(csic_dev->base);
	csic_dev->base = NULL;
	return 0;
}

static long msm_csic_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	struct csic_cfg_params cfg_params;
	switch (cmd) {
	case VIDIOC_MSM_CSIC_CFG:
		cfg_params.subdev = sd;
		cfg_params.parms = arg;
		return msm_csic_config((struct csic_cfg_params *)&cfg_params);
	case VIDIOC_MSM_CSIC_INIT:
		return msm_csic_init(sd, (uint32_t *)arg);
	case VIDIOC_MSM_CSIC_RELEASE:
		return msm_csic_release(sd);
	default:
		return -ENOIOCTLCMD;
	}
}

static struct v4l2_subdev_core_ops msm_csic_subdev_core_ops = {
	.g_chip_ident = &msm_csic_subdev_g_chip_ident,
	.ioctl = &msm_csic_subdev_ioctl,
};

static const struct v4l2_subdev_ops msm_csic_subdev_ops = {
	.core = &msm_csic_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops msm_csic_internal_ops;

static int __devinit csic_probe(struct platform_device *pdev)
{
	struct csic_device *new_csic_dev;
	int rc = 0;
	CDBG("%s: device id = %d\n", __func__, pdev->id);
	new_csic_dev = kzalloc(sizeof(struct csic_device), GFP_KERNEL);
	if (!new_csic_dev) {
		pr_err("%s: no enough memory\n", __func__);
		return -ENOMEM;
	}

	v4l2_subdev_init(&new_csic_dev->subdev, &msm_csic_subdev_ops);
	new_csic_dev->subdev.internal_ops = &msm_csic_internal_ops;
	new_csic_dev->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(new_csic_dev->subdev.name,
			ARRAY_SIZE(new_csic_dev->subdev.name), "msm_csic");

	v4l2_set_subdevdata(&new_csic_dev->subdev, new_csic_dev);
	platform_set_drvdata(pdev, &new_csic_dev->subdev);
	mutex_init(&new_csic_dev->mutex);

	new_csic_dev->mem = platform_get_resource_byname(pdev,
					IORESOURCE_MEM, "csic");
	if (!new_csic_dev->mem) {
		pr_err("%s: no mem resource?\n", __func__);
		rc = -ENODEV;
		goto csic_no_resource;
	}
	new_csic_dev->irq = platform_get_resource_byname(pdev,
					IORESOURCE_IRQ, "csic");
	if (!new_csic_dev->irq) {
		pr_err("%s: no irq resource?\n", __func__);
		rc = -ENODEV;
		goto csic_no_resource;
	}
	new_csic_dev->io = request_mem_region(new_csic_dev->mem->start,
		resource_size(new_csic_dev->mem), pdev->name);
	if (!new_csic_dev->io) {
		pr_err("%s: no valid mem region\n", __func__);
		rc = -EBUSY;
		goto csic_no_resource;
	}

	rc = request_irq(new_csic_dev->irq->start, msm_csic_irq,
		IRQF_TRIGGER_HIGH, "csic", new_csic_dev);
	if (rc < 0) {
		release_mem_region(new_csic_dev->mem->start,
			resource_size(new_csic_dev->mem));
		pr_err("%s: irq request fail\n", __func__);
		rc = -EBUSY;
		goto csic_no_resource;
	}
	disable_irq(new_csic_dev->irq->start);

	new_csic_dev->pdev = pdev;

	msm_cam_clk_enable(&pdev->dev, csic_clk_info,
		new_csic_dev->csic_clk, ARRAY_SIZE(csic_clk_info), 1);
	new_csic_dev->base = ioremap(new_csic_dev->mem->start,
		resource_size(new_csic_dev->mem));
	if (!new_csic_dev->base) {
		rc = -ENOMEM;
		goto csic_no_resource;
	}

	msm_io_w(MIPI_PWR_CNTL_DIS, new_csic_dev->base + MIPI_PWR_CNTL);
	msm_cam_clk_enable(&pdev->dev, csic_clk_info,
		new_csic_dev->csic_clk, ARRAY_SIZE(csic_clk_info), 0);
	iounmap(new_csic_dev->base);

	new_csic_dev->base = NULL;
	msm_cam_register_subdev_node(
		&new_csic_dev->subdev, CSIC_DEV, pdev->id);

	return 0;

csic_no_resource:
	mutex_destroy(&new_csic_dev->mutex);
	kfree(new_csic_dev);
	return 0;
}

static struct platform_driver csic_driver = {
	.probe = csic_probe,
	.driver = {
		.name = MSM_CSIC_DRV_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init msm_csic_init_module(void)
{
	return platform_driver_register(&csic_driver);
}

static void __exit msm_csic_exit_module(void)
{
	platform_driver_unregister(&csic_driver);
}

module_init(msm_csic_init_module);
module_exit(msm_csic_exit_module);
MODULE_DESCRIPTION("MSM csic driver");
MODULE_LICENSE("GPL v2");