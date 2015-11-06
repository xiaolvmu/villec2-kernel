/*
 * SDIO device core hardware definitions.
 * sdio is a portion of the pcmcia core in core rev 3 - rev 8
 *
 * SDIO core support 1bit, 4 bit SDIO mode as well as SPI mode.
 *
 * Copyright (C) 1999-2012, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: sbsdio.h 308945 2012-01-18 02:15:27Z $
 */

#ifndef	_SBSDIO_H
#define	_SBSDIO_H

#define SBSDIO_NUM_FUNCTION		3	

#define SBSDIO_SPROM_CS			0x10000		
#define SBSDIO_SPROM_INFO		0x10001		
#define SBSDIO_SPROM_DATA_LOW		0x10002		
#define SBSDIO_SPROM_DATA_HIGH		0x10003 	
#define SBSDIO_SPROM_ADDR_LOW		0x10004		
#define SBSDIO_SPROM_ADDR_HIGH		0x10005		
#define SBSDIO_CHIP_CTRL_DATA		0x10006		
#define SBSDIO_CHIP_CTRL_EN		0x10007		
#define SBSDIO_WATERMARK		0x10008		
#define SBSDIO_DEVICE_CTL		0x10009		

#define SBSDIO_FUNC1_SBADDRLOW		0x1000A		
#define SBSDIO_FUNC1_SBADDRMID		0x1000B		
#define SBSDIO_FUNC1_SBADDRHIGH		0x1000C		
#define SBSDIO_FUNC1_FRAMECTRL		0x1000D		
#define SBSDIO_FUNC1_CHIPCLKCSR		0x1000E		
#define SBSDIO_FUNC1_SDIOPULLUP 	0x1000F		
#define SBSDIO_FUNC1_WFRAMEBCLO		0x10019		
#define SBSDIO_FUNC1_WFRAMEBCHI		0x1001A		
#define SBSDIO_FUNC1_RFRAMEBCLO		0x1001B		
#define SBSDIO_FUNC1_RFRAMEBCHI		0x1001C		
#define SBSDIO_FUNC1_MESBUSYCTRL	0x1001D		

#define SBSDIO_FUNC1_MISC_REG_START	0x10000 	
#define SBSDIO_FUNC1_MISC_REG_LIMIT	0x1001C 	

#define SBSDIO_FUNC1_WAKEUPCTRL			0x1001E
#define SBSDIO_FUNC1_WCTRL_ALPWAIT_MASK		0x1
#define SBSDIO_FUNC1_WCTRL_ALPWAIT_SHIFT	0
#define SBSDIO_FUNC1_WCTRL_HTWAIT_MASK		0x2
#define SBSDIO_FUNC1_WCTRL_HTWAIT_SHIFT		1
#define SBSDIO_FUNC1_SLEEPCSR			0x1001F
#define SBSDIO_FUNC1_SLEEPCSR_KSO_MASK		0x1
#define SBSDIO_FUNC1_SLEEPCSR_KSO_SHIFT		0
#define SBSDIO_FUNC1_SLEEPCSR_KSO_EN		1
#define SBSDIO_FUNC1_SLEEPCSR_DEVON_MASK	0x2
#define SBSDIO_FUNC1_SLEEPCSR_DEVON_SHIFT	1

#define SBSDIO_SPROM_IDLE		0
#define SBSDIO_SPROM_WRITE		1
#define SBSDIO_SPROM_READ		2
#define SBSDIO_SPROM_WEN		4
#define SBSDIO_SPROM_WDS		7
#define SBSDIO_SPROM_DONE		8

#define SROM_SZ_MASK			0x03		
#define SROM_BLANK			0x04		
#define	SROM_OTP			0x80		

#define SBSDIO_CHIP_CTRL_XTAL		0x01		
#define SBSDIO_WATERMARK_MASK		0x7f		

#define SBSDIO_MESBUSYCTRL_MASK		0x7f

#define SBSDIO_DEVCTL_SETBUSY		0x01		
#define SBSDIO_DEVCTL_SPI_INTR_SYNC	0x02		
#define SBSDIO_DEVCTL_CA_INT_ONLY	0x04		
#define SBSDIO_DEVCTL_PADS_ISO		0x08		
#define SBSDIO_DEVCTL_SB_RST_CTL	0x30		
#define SBSDIO_DEVCTL_RST_CORECTL	0x00		
#define SBSDIO_DEVCTL_RST_BPRESET	0x10		
#define SBSDIO_DEVCTL_RST_NOBPRESET	0x20		


#define SBSDIO_FORCE_ALP		0x01		
#define SBSDIO_FORCE_HT			0x02		
#define SBSDIO_FORCE_ILP		0x04		
#define SBSDIO_ALP_AVAIL_REQ		0x08		
#define SBSDIO_HT_AVAIL_REQ		0x10		
#define SBSDIO_FORCE_HW_CLKREQ_OFF	0x20		
#define SBSDIO_ALP_AVAIL		0x40		
#define SBSDIO_HT_AVAIL			0x80		
#define SBSDIO_Rev8_HT_AVAIL		0x40
#define SBSDIO_Rev8_ALP_AVAIL		0x80
#define SBSDIO_CSR_MASK			0x1F

#define SBSDIO_AVBITS			(SBSDIO_HT_AVAIL | SBSDIO_ALP_AVAIL)
#define SBSDIO_ALPAV(regval)		((regval) & SBSDIO_AVBITS)
#define SBSDIO_HTAV(regval)		(((regval) & SBSDIO_AVBITS) == SBSDIO_AVBITS)
#define SBSDIO_ALPONLY(regval)		(SBSDIO_ALPAV(regval) && !SBSDIO_HTAV(regval))
#define SBSDIO_CLKAV(regval, alponly)	(SBSDIO_ALPAV(regval) && \
					(alponly ? 1 : SBSDIO_HTAV(regval)))

#define SBSDIO_PULLUP_D0		0x01		
#define SBSDIO_PULLUP_D1		0x02		
#define SBSDIO_PULLUP_D2		0x04		
#define SBSDIO_PULLUP_CMD		0x08		
#define SBSDIO_PULLUP_ALL		0x0f		

#define SBSDIO_SB_OFT_ADDR_MASK		0x07FFF		
#define SBSDIO_SB_OFT_ADDR_LIMIT	0x08000
#define SBSDIO_SB_ACCESS_2_4B_FLAG	0x08000		

#define SBSDIO_SBADDRLOW_MASK		0x80		
#define SBSDIO_SBADDRMID_MASK		0xff		
#define SBSDIO_SBADDRHIGH_MASK		0xffU		
#define SBSDIO_SBWINDOW_MASK		0xffff8000	

#define SBSDIO_CIS_BASE_COMMON		0x1000		
#define SBSDIO_CIS_SIZE_LIMIT		0x200		
#define SBSDIO_OTP_CIS_SIZE_LIMIT       0x078           

#define SBSDIO_CIS_OFT_ADDR_MASK	0x1FFFF		

#define SBSDIO_CIS_MANFID_TUPLE_LEN	6		

#define SBSDIO_SPROM_CIS_OFFSET		0x8		

#define SBSDIO_BYTEMODE_DATALEN_MAX	64		

#define SBSDIO_CORE_ADDR_MASK		0x1FFFF		

#endif	
