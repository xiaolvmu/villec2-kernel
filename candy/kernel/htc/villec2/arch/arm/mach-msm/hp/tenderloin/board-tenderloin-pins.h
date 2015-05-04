#ifndef __BOARD_TENDERLOIN_PINS_H__
#define __BOARD_TENDERLOIN_PINS_H__

extern int *pin_table;

enum tenderloin_pins {
	MXT1386_TS_PEN_IRQ = 0,
	GPIO_CTP_WAKE_PIN,
	BT_RST_N_PIN,
	BT_POWER_PIN,
	BT_WAKE_PIN,
	BT_HOST_WAKE_PIN,
	GYRO_INT_PIN,
	LM8502_LIGHTING_INT_IRQ_PIN,
	MAX8903B_GPIO_USB_CHG_MODE_PIN,
	MAX8903B_GPIO_DC_OK_PIN,
	TENDERLOIN_GPIO_WLAN_RST_N_PIN,
	TENDERLOIN_GPIO_HOST_WAKE_WL_PIN,
	TENDERLOIN_A6_0_TCK_PIN,
	TENDERLOIN_A6_0_WAKEUP_PIN,
	TENDERLOIN_A6_0_TDIO_PIN,
	TENDERLOIN_A6_0_MSM_IRQ_PIN,
	TENDERLOIN_A6_1_TCK_PIN,
	TENDERLOIN_A6_1_WAKEUP_PIN,
	TENDERLOIN_A6_1_TDIO_PIN,
	TENDERLOIN_A6_1_MSM_IRQ_PIN,
	VOL_UP_GPIO_PIN,
	VOL_DN_GPIO_PIN,
	TENDERLOIN_GPIO_3G_3V3_EN,
	NUM_TOPAZ_PINS
};

void __init tenderloin_a6_fixup_pins(void);
void __init tenderloin_init_pins(void);
void __init tenderloin_fixup_pins(void);

#endif /* !__BOARD_TENDERLOIN_PINS_H__ */