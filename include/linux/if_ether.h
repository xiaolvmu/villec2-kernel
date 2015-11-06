/*
 * INET		An implementation of the TCP/IP protocol suite for the LINUX
 *		operating system.  INET is implemented using the  BSD Socket
 *		interface as the means of communication with the user level.
 *
 *		Global definitions for the Ethernet IEEE 802.3 interface.
 *
 * Version:	@(#)if_ether.h	1.0.1a	02/08/94
 *
 * Author:	Fred N. van Kempen, <waltje@uWalt.NL.Mugnet.ORG>
 *		Donald Becker, <becker@super.org>
 *		Alan Cox, <alan@lxorguk.ukuu.org.uk>
 *		Steve Whitehouse, <gw7rrm@eeshack3.swan.ac.uk>
 *
 *		This program is free software; you can redistribute it and/or
 *		modify it under the terms of the GNU General Public License
 *		as published by the Free Software Foundation; either version
 *		2 of the License, or (at your option) any later version.
 */

#ifndef _LINUX_IF_ETHER_H
#define _LINUX_IF_ETHER_H

#include <linux/types.h>


#define ETH_ALEN	6		
#define ETH_HLEN	14		
#define ETH_ZLEN	60		
#define ETH_DATA_LEN	1500		
#define ETH_FRAME_LEN	1514		
#define ETH_FCS_LEN	4		


#define ETH_P_LOOP	0x0060		
#define ETH_P_PUP	0x0200		
#define ETH_P_PUPAT	0x0201		
#define ETH_P_IP	0x0800		
#define ETH_P_X25	0x0805		
#define ETH_P_ARP	0x0806		
#define	ETH_P_BPQ	0x08FF		
#define ETH_P_IEEEPUP	0x0a00		
#define ETH_P_IEEEPUPAT	0x0a01		
#define ETH_P_DEC       0x6000          
#define ETH_P_DNA_DL    0x6001          
#define ETH_P_DNA_RC    0x6002          
#define ETH_P_DNA_RT    0x6003          
#define ETH_P_LAT       0x6004          
#define ETH_P_DIAG      0x6005          
#define ETH_P_CUST      0x6006          
#define ETH_P_SCA       0x6007          
#define ETH_P_TEB	0x6558		
#define ETH_P_RARP      0x8035		
#define ETH_P_ATALK	0x809B		
#define ETH_P_AARP	0x80F3		
#define ETH_P_8021Q	0x8100          
#define ETH_P_IPX	0x8137		
#define ETH_P_IPV6	0x86DD		
#define ETH_P_PAUSE	0x8808		
#define ETH_P_SLOW	0x8809		
#define ETH_P_WCCP	0x883E		
#define ETH_P_PPP_DISC	0x8863		
#define ETH_P_PPP_SES	0x8864		
#define ETH_P_MPLS_UC	0x8847		
#define ETH_P_MPLS_MC	0x8848		
#define ETH_P_ATMMPOA	0x884c		
#define ETH_P_LINK_CTL	0x886c		
#define ETH_P_ATMFATE	0x8884		
#define ETH_P_PAE	0x888E		
#define ETH_P_AOE	0x88A2		
#define ETH_P_8021AD	0x88A8          
#define ETH_P_802_EX1	0x88B5		
#define ETH_P_TIPC	0x88CA		
#define ETH_P_8021AH	0x88E7          
#define ETH_P_1588	0x88F7		
#define ETH_P_FCOE	0x8906		
#define ETH_P_TDLS	0x890D          
#define ETH_P_FIP	0x8914		
#define ETH_P_QINQ1	0x9100		
#define ETH_P_QINQ2	0x9200		
#define ETH_P_QINQ3	0x9300		
#define ETH_P_EDSA	0xDADA		
#define ETH_P_AF_IUCV   0xFBFB		


#define ETH_P_802_3	0x0001		
#define ETH_P_AX25	0x0002		
#define ETH_P_ALL	0x0003		
#define ETH_P_802_2	0x0004		
#define ETH_P_SNAP	0x0005		
#define ETH_P_DDCMP     0x0006          
#define ETH_P_WAN_PPP   0x0007          
#define ETH_P_PPP_MP    0x0008          
#define ETH_P_LOCALTALK 0x0009		
#define ETH_P_CAN	0x000C		
#define ETH_P_PPPTALK	0x0010		
#define ETH_P_TR_802_2	0x0011		
#define ETH_P_MOBITEX	0x0015		
#define ETH_P_CONTROL	0x0016		
#define ETH_P_IRDA	0x0017		
#define ETH_P_ECONET	0x0018		
#define ETH_P_HDLC	0x0019		
#define ETH_P_ARCNET	0x001A		
#define ETH_P_DSA	0x001B		
#define ETH_P_TRAILER	0x001C		
#define ETH_P_PHONET	0x00F5		
#define ETH_P_IEEE802154 0x00F6		
#define ETH_P_CAIF	0x00F7		


struct ethhdr {
	unsigned char	h_dest[ETH_ALEN];	
	unsigned char	h_source[ETH_ALEN];	
	__be16		h_proto;		
} __attribute__((packed));

#ifdef __KERNEL__
#include <linux/skbuff.h>

static inline struct ethhdr *eth_hdr(const struct sk_buff *skb)
{
	return (struct ethhdr *)skb_mac_header(skb);
}

int eth_header_parse(const struct sk_buff *skb, unsigned char *haddr);

int mac_pton(const char *s, u8 *mac);
extern ssize_t sysfs_format_mac(char *buf, const unsigned char *addr, int len);

#endif

#endif	
