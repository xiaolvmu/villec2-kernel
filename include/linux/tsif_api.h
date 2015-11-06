/**
 * TSIF driver
 *
 * Kernel API
 *
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights
 * reserved.
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
#ifndef _TSIF_API_H_
#define _TSIF_API_H_

#define TSIF_PKT_SIZE             (192)

static inline u32 tsif_pkt_status(void *pkt)
{
	u32 *x = pkt;
	return x[TSIF_PKT_SIZE / sizeof(u32) - 1];
}

#define TSIF_STATUS_TTS(x)   ((x) & 0xffffff)
#define TSIF_STATUS_VALID(x) ((x) & (1<<24))
#define TSIF_STATUS_FIRST(x) ((x) & (1<<25))
#define TSIF_STATUS_OVFLW(x) ((x) & (1<<26))
#define TSIF_STATUS_ERROR(x) ((x) & (1<<27))
#define TSIF_STATUS_NULL(x)  ((x) & (1<<28))
#define TSIF_STATUS_TIMEO(x) ((x) & (1<<30))

enum tsif_state {
	tsif_state_stopped  = 0,
	tsif_state_running  = 1,
	tsif_state_flushing = 2,
	tsif_state_error    = 3,
};

int tsif_get_active(void);

void *tsif_attach(int id, void (*notify)(void *client_data), void *client_data);
void tsif_detach(void *cookie);
void tsif_get_info(void *cookie, void **pdata, int *psize);
int tsif_set_mode(void *cookie, int mode);
int tsif_set_time_limit(void *cookie, u32 value);
int tsif_set_buf_config(void *cookie, u32 pkts_in_chunk, u32 chunks_in_buf);
void tsif_get_state(void *cookie, int *ri, int *wi, enum tsif_state *state);
int tsif_start(void *cookie);
void tsif_stop(void *cookie);
void tsif_reclaim_packets(void *cookie, int ri);

#endif 

