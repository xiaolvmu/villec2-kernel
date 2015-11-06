/*
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2010 HTC Corporation.
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


#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/rfkill.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/mach-types.h>

#include <linux/mfd/pmic8058.h>
#include "board-villec2.h"

static struct rfkill *bt_rfk;
static const char bt_name[] = "bcm4329";

static uint32_t villec2_bt_on_table[] = {

	
	GPIO_CFG(VILLEC2_GPIO_BT_UART1_RTS,
				1,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA),
	
	GPIO_CFG(VILLEC2_GPIO_BT_UART1_CTS,
				1,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA),
	
	GPIO_CFG(VILLEC2_GPIO_BT_UART1_RX,
				1,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA),
	
	GPIO_CFG(VILLEC2_GPIO_BT_UART1_TX,
				1,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA),

	
	GPIO_CFG(VILLEC2_GPIO_BT_HOST_WAKE,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
	
	GPIO_CFG(VILLEC2_GPIO_BT_CHIP_WAKE,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),

	
	GPIO_CFG(VILLEC2_GPIO_BT_RESET_N,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
	
	GPIO_CFG(VILLEC2_GPIO_BT_SHUTDOWN_N,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
};

static uint32_t villec2_bt_off_table[] = {

	
	GPIO_CFG(VILLEC2_GPIO_BT_UART1_RTS,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA),
	
	GPIO_CFG(VILLEC2_GPIO_BT_UART1_CTS,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA),
	
	GPIO_CFG(VILLEC2_GPIO_BT_UART1_RX,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP,
				GPIO_CFG_8MA),
	
	GPIO_CFG(VILLEC2_GPIO_BT_UART1_TX,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_8MA),

	
	GPIO_CFG(VILLEC2_GPIO_BT_RESET_N,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
	
	GPIO_CFG(VILLEC2_GPIO_BT_SHUTDOWN_N,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),

	
	GPIO_CFG(VILLEC2_GPIO_BT_HOST_WAKE,
				0,
				GPIO_CFG_INPUT,
				GPIO_CFG_PULL_DOWN,
				GPIO_CFG_2MA),
	
	GPIO_CFG(VILLEC2_GPIO_BT_CHIP_WAKE,
				0,
				GPIO_CFG_OUTPUT,
				GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA),
};

static void config_bt_table(uint32_t *table, int len)
{
	int n, rc;

	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[BT]%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static void villec2_config_bt_on(void)
{
	printk(KERN_INFO "[BT]-- R ON --\n");

	
	config_bt_table(villec2_bt_on_table,
				ARRAY_SIZE(villec2_bt_on_table));
	mdelay(2);

	
	gpio_set_value(VILLEC2_GPIO_BT_RESET_N, 0);
	mdelay(1);

	
	gpio_set_value(VILLEC2_GPIO_BT_SHUTDOWN_N, 0);
	mdelay(5);

	
	gpio_set_value(VILLEC2_GPIO_BT_SHUTDOWN_N, 1);
	mdelay(1);

	
	gpio_set_value(VILLEC2_GPIO_BT_RESET_N, 1);
	mdelay(2);

}

static void villec2_config_bt_off(void)
{
	printk(KERN_INFO "[BT]-- R OFF --\n");

	
	gpio_set_value(VILLEC2_GPIO_BT_RESET_N, 0);
	mdelay(1);

	
	gpio_set_value(VILLEC2_GPIO_BT_SHUTDOWN_N, 0);
	mdelay(1);

	
	config_bt_table(villec2_bt_off_table,
				ARRAY_SIZE(villec2_bt_off_table));
	mdelay(2);

	
	gpio_set_value(VILLEC2_GPIO_BT_UART1_RTS, 1);

	

	
	gpio_set_value(VILLEC2_GPIO_BT_UART1_TX, 0);

	


	

	
	gpio_set_value(VILLEC2_GPIO_BT_CHIP_WAKE, 0);
}

static int bluetooth_set_power(void *data, bool blocked)
{
	if (!blocked)
		villec2_config_bt_on();
	else
		villec2_config_bt_off();

	return 0;
}

static struct rfkill_ops villec2_rfkill_ops = {
	.set_block = bluetooth_set_power,
};

static int villec2_rfkill_probe(struct platform_device *pdev)
{
	int rc = 0;
	bool default_state = true; 

#if 0 
	rc = gpio_request(VILLEC2_GPIO_BT_RESET_N, "bt_reset");
	if (rc)
		goto err_gpio_reset;
	rc = gpio_request(VILLEC2_GPIO_BT_SHUTDOWN_N, "bt_shutdown");
	if (rc)
		goto err_gpio_shutdown;
#endif

	
	mdelay(2);

	bluetooth_set_power(NULL, default_state);

	bt_rfk = rfkill_alloc(bt_name, &pdev->dev, RFKILL_TYPE_BLUETOOTH,
				&villec2_rfkill_ops, NULL);
	if (!bt_rfk) {
		rc = -ENOMEM;
		goto err_rfkill_alloc;
	}

	rfkill_set_states(bt_rfk, default_state, false);

	
	rc = rfkill_register(bt_rfk);
	if (rc)
		goto err_rfkill_reg;

	return 0;

err_rfkill_reg:
	rfkill_destroy(bt_rfk);
err_rfkill_alloc:
#if 0
	gpio_free(VILLEC2_GPIO_BT_SHUTDOWN_N);
err_gpio_shutdown:
	gpio_free(VILLEC2_GPIO_BT_RESET_N);
err_gpio_reset:
#endif
	return rc;
}

static int villec2_rfkill_remove(struct platform_device *dev)
{
	rfkill_unregister(bt_rfk);
	rfkill_destroy(bt_rfk);
#if 0
	gpio_free(VILLEC2_GPIO_BT_SHUTDOWN_N);
	gpio_free(VILLEC2_GPIO_BT_RESET_N);
#endif

	return 0;
}

static struct platform_driver villec2_rfkill_driver = {
	.probe = villec2_rfkill_probe,
	.remove = villec2_rfkill_remove,
	.driver = {
		.name = "villec2_rfkill",
		.owner = THIS_MODULE,
	},
};

static int __init villec2_rfkill_init(void)
{
	return platform_driver_register(&villec2_rfkill_driver);
}

static void __exit villec2_rfkill_exit(void)
{
	platform_driver_unregister(&villec2_rfkill_driver);
}

module_init(villec2_rfkill_init);
module_exit(villec2_rfkill_exit);
MODULE_DESCRIPTION("villec2 rfkill");
MODULE_AUTHOR("Nick Pelly <npelly@google.com>");
MODULE_LICENSE("GPL");
