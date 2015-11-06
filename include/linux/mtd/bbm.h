/*
 *  linux/include/linux/mtd/bbm.h
 *
 *  NAND family Bad Block Management (BBM) header file
 *    - Bad Block Table (BBT) implementation
 *
 *  Copyright © 2005 Samsung Electronics
 *  Kyungmin Park <kyungmin.park@samsung.com>
 *
 *  Copyright © 2000-2005
 *  Thomas Gleixner <tglx@linuxtronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#ifndef __LINUX_MTD_BBM_H
#define __LINUX_MTD_BBM_H

#define NAND_MAX_CHIPS		8

struct nand_bbt_descr {
	int options;
	int pages[NAND_MAX_CHIPS];
	int offs;
	int veroffs;
	uint8_t version[NAND_MAX_CHIPS];
	int len;
	int maxblocks;
	int reserved_block_code;
	uint8_t *pattern;
};


#define NAND_BBT_NRBITS_MSK	0x0000000F
#define NAND_BBT_1BIT		0x00000001
#define NAND_BBT_2BIT		0x00000002
#define NAND_BBT_4BIT		0x00000004
#define NAND_BBT_8BIT		0x00000008
#define NAND_BBT_LASTBLOCK	0x00000010
#define NAND_BBT_ABSPAGE	0x00000020
#define NAND_BBT_SEARCH		0x00000040
#define NAND_BBT_PERCHIP	0x00000080
#define NAND_BBT_VERSION	0x00000100
#define NAND_BBT_CREATE		0x00000200
#define NAND_BBT_CREATE_EMPTY	0x00000400
#define NAND_BBT_SCANALLPAGES	0x00000800
#define NAND_BBT_SCANEMPTY	0x00001000
#define NAND_BBT_WRITE		0x00002000
#define NAND_BBT_SAVECONTENT	0x00004000
#define NAND_BBT_SCAN2NDPAGE	0x00008000
#define NAND_BBT_SCANLASTPAGE	0x00010000
#define NAND_BBT_USE_FLASH	0x00020000
#define NAND_BBT_NO_OOB		0x00040000
#define NAND_BBT_NO_OOB_BBM	0x00080000

#define NAND_BBT_DYNAMICSTRUCT	0x80000000

#define NAND_BBT_SCAN_MAXBLOCKS	4

#define NAND_SMALL_BADBLOCK_POS		5
#define NAND_LARGE_BADBLOCK_POS		0
#define ONENAND_BADBLOCK_POS		0

#define ONENAND_BBT_READ_ERROR		1
#define ONENAND_BBT_READ_ECC_ERROR	2
#define ONENAND_BBT_READ_FATAL_ERROR	4

struct bbm_info {
	int bbt_erase_shift;
	int badblockpos;
	int options;

	uint8_t *bbt;

	int (*isbad_bbt)(struct mtd_info *mtd, loff_t ofs, int allowbbt);

	
	struct nand_bbt_descr *badblock_pattern;

	void *priv;
};

extern int onenand_scan_bbt(struct mtd_info *mtd, struct nand_bbt_descr *bd);
extern int onenand_default_bbt(struct mtd_info *mtd);

#endif	
