/*
 *  Key chord input driver
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
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

#ifndef __LINUX_KEYCHORD_H_
#define __LINUX_KEYCHORD_H_

#include <linux/input.h>

#define KEYCHORD_VERSION		1

struct input_keychord {
	
	__u16 version;
	__u16 id;

	
	__u16 count;

	
	__u16 keycodes[];
};

#endif	
