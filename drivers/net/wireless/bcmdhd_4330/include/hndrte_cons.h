/*
 * Console support for hndrte.
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
 * $Id: hndrte_cons.h 300516 2011-12-04 17:39:44Z $
 */
#ifndef	_HNDRTE_CONS_H
#define	_HNDRTE_CONS_H

#include <typedefs.h>

#define CBUF_LEN	(128)

#define LOG_BUF_LEN	1024

typedef struct {
	uint32		buf;		
	uint		buf_size;
	uint		idx;
	char		*_buf_compat;	
} hndrte_log_t;

typedef struct {
	volatile uint	vcons_in;
	volatile uint	vcons_out;

	hndrte_log_t	log;

	uint		cbuf_idx;
	char		cbuf[CBUF_LEN];
} hndrte_cons_t;

#endif 
