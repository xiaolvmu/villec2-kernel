/*
 * Linux cfg80211 driver
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
 * $Id: wl_cfg80211.c 323022 2012-03-22 17:48:58Z $
 */

#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>
#include <linux/kernel.h>

#include <bcmutils.h>
#include <bcmwifi_channels.h>
#include <bcmendian.h>
#include <proto/ethernet.h>
#include <proto/802.11.h>
#include <linux/if_arp.h>
#include <asm/uaccess.h>

#include <dngl_stats.h>
#include <dhd.h>
#include <dhdioctl.h>
#include <wlioctl.h>
#include <dhd_cfg80211.h>

#include <proto/ethernet.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/netdevice.h>
#include <linux/sched.h>
#include <linux/etherdevice.h>
#include <linux/wireless.h>
#include <linux/ieee80211.h>
#include <linux/wait.h>
#include <net/cfg80211.h>
#include <net/rtnetlink.h>

#include <wlioctl.h>
#include <wldev_common.h>
#include <wl_cfg80211.h>
#include <wl_cfgp2p.h>

#include <wl_iw.h>

#ifdef BCMWAPI_WPI
#ifndef IW_ENCODE_ALG_SM4
#define IW_ENCODE_ALG_SM4 0x20
#endif

#ifndef IW_AUTH_WAPI_ENABLED
#define IW_AUTH_WAPI_ENABLED 0x20
#endif

#ifndef IW_AUTH_WAPI_VERSION_1
#define IW_AUTH_WAPI_VERSION_1  0x00000008
#endif

#ifndef IW_AUTH_CIPHER_SMS4
#define IW_AUTH_CIPHER_SMS4     0x00000020
#endif

#ifndef IW_AUTH_KEY_MGMT_WAPI_PSK
#define IW_AUTH_KEY_MGMT_WAPI_PSK 4
#endif

#ifndef IW_AUTH_KEY_MGMT_WAPI_CERT
#define IW_AUTH_KEY_MGMT_WAPI_CERT 8
#endif
#endif 

#ifdef BCMWAPI_WPI
#define IW_WSEC_ENABLED(wsec)   ((wsec) & (WEP_ENABLED | TKIP_ENABLED | AES_ENABLED | SMS4_ENABLED))
#else 
#define IW_WSEC_ENABLED(wsec)   ((wsec) & (WEP_ENABLED | TKIP_ENABLED | AES_ENABLED))
#endif 

#define FORCE_MPC
#if defined(APSTA_CONCURRENT) && defined(SOFTAP)
extern struct net_device *ap_net_dev;
extern void wldev_san_check_channel(struct net_device *ndev,int *errcode);
extern s32 wldev_set_ssid(struct net_device *dev,int *channel);
#endif

static struct device *cfg80211_parent_dev = NULL;
static int vsdb_supported = 0;
struct wl_priv *wlcfg_drv_priv = NULL;

u32 wl_dbg_level = WL_DBG_ERR;

#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAX_WAIT_TIME 1500
#define WL_SCAN_ACTIVE_TIME	 40 
#define WL_SCAN_PASSIVE_TIME	130 

#ifdef VSDB
#define DEFAULT_SLEEP_TIME_VSDB 200

#define WL_AF_TX_KEEP_PRI_CONNECTION_VSDB(wl)	\
	do {	\
		if (wl_get_drv_status(wl, CONNECTED, wl_to_prmry_ndev(wl)) ||	\
			wl_get_drv_status(wl, CONNECTING, wl_to_prmry_ndev(wl))) {	\
			msleep(DEFAULT_SLEEP_TIME_VSDB);	\
		}	\
	} while (0)
#else 
#define WL_AF_TX_KEEP_PRI_CONNECTION_VSDB(wl)
#endif 

#ifdef WL_CFG80211_SYNC_GON
#define WL_DRV_STATUS_SENDING_AF_FRM_EXT(wl) \
	(wl_get_drv_status_all(wl, SENDING_ACT_FRM) || \
		wl_get_drv_status_all(wl, WAITING_NEXT_ACT_FRM_LISTEN))
#else
#define WL_DRV_STATUS_SENDING_AF_FRM_EXT(wl) wl_get_drv_status_all(wl, SENDING_ACT_FRM)
#endif 

#define WL_CHANSPEC_CTL_SB_NONE WL_CHANSPEC_CTL_SB_LLL


#define DNGL_FUNC(func, parameters) func parameters;
#define COEX_DHCP

static const struct ieee80211_regdomain brcm_regdom = {
	.n_reg_rules = 4,
	.alpha2 =  "99",
	.reg_rules = {
		
		REG_RULE(2412-10, 2472+10, 40, 6, 20, 0),
		
		REG_RULE(2484-10, 2484+10, 20, 6, 20, 0),
		
		REG_RULE(5150-10, 5350+10, 40, 6, 20, 0),
		
		REG_RULE(5470-10, 5850+10, 40, 6, 20, 0), }
};


#define WPS_ID_CONFIG_METHODS     0x1008
#define WPS_ID_REQ_TYPE           0x103A
#define WPS_ID_DEVICE_NAME        0x1011
#define WPS_ID_VERSION            0x104A
#define WPS_ID_DEVICE_PWD_ID      0x1012
#define WPS_ID_REQ_DEV_TYPE       0x106A
#define WPS_ID_SELECTED_REGISTRAR_CONFIG_METHODS 0x1053
#define WPS_ID_PRIM_DEV_TYPE      0x1054

#define DEV_PW_DEFAULT 0x0000
#define DEV_PW_USER_SPECIFIED 0x0001,
#define DEV_PW_MACHINE_SPECIFIED 0x0002
#define DEV_PW_REKEY 0x0003
#define DEV_PW_PUSHBUTTON 0x0004
#define DEV_PW_REGISTRAR_SPECIFIED 0x0005

#define WPS_CONFIG_USBA 0x0001
#define WPS_CONFIG_ETHERNET 0x0002
#define WPS_CONFIG_LABEL 0x0004
#define WPS_CONFIG_DISPLAY 0x0008
#define WPS_CONFIG_EXT_NFC_TOKEN 0x0010
#define WPS_CONFIG_INT_NFC_TOKEN 0x0020
#define WPS_CONFIG_NFC_INTERFACE 0x0040
#define WPS_CONFIG_PUSHBUTTON 0x0080
#define WPS_CONFIG_KEYPAD 0x0100
#define WPS_CONFIG_VIRT_PUSHBUTTON 0x0280
#define WPS_CONFIG_PHY_PUSHBUTTON 0x0480
#define WPS_CONFIG_VIRT_DISPLAY 0x2008
#define WPS_CONFIG_PHY_DISPLAY 0x4008

#define PM_BLOCK 1
#define PM_ENABLE 0

#ifndef RSSI_OFFSET
#define RSSI_OFFSET	0
#endif

#ifdef BCMCCX
#ifndef WLAN_AKM_SUITE_CCKM
#define WLAN_AKM_SUITE_CCKM 0x000FAC04
#endif
#define DOT11_LEAP_AUTH	0x80 
#endif 

static s32 wl_frame_get_mgmt(u16 fc, const struct ether_addr *da,
	const struct ether_addr *sa, const struct ether_addr *bssid,
	u8 **pheader, u32 *body_len, u8 *pbody);
static s32 __wl_cfg80211_scan(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_scan_request *request,
	struct cfg80211_ssid *this_ssid);
static s32 wl_cfg80211_scan(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_scan_request *request);
static s32 wl_cfg80211_set_wiphy_params(struct wiphy *wiphy, u32 changed);
static s32 wl_cfg80211_join_ibss(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_ibss_params *params);
static s32 wl_cfg80211_leave_ibss(struct wiphy *wiphy,
	struct net_device *dev);
static s32 wl_cfg80211_get_station(struct wiphy *wiphy,
	struct net_device *dev, u8 *mac,
	struct station_info *sinfo);
static s32 wl_cfg80211_set_power_mgmt(struct wiphy *wiphy,
	struct net_device *dev, bool enabled,
	s32 timeout);
static int wl_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_connect_params *sme);
static s32 wl_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev,
	u16 reason_code);
static s32 wl_cfg80211_set_tx_power(struct wiphy *wiphy,
	enum nl80211_tx_power_setting type,
	s32 dbm);
static s32 wl_cfg80211_get_tx_power(struct wiphy *wiphy, s32 *dbm);
static s32 wl_cfg80211_config_default_key(struct wiphy *wiphy,
	struct net_device *dev,
	u8 key_idx, bool unicast, bool multicast);
static s32 wl_cfg80211_add_key(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, bool pairwise, const u8 *mac_addr,
	struct key_params *params);
static s32 wl_cfg80211_del_key(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, bool pairwise, const u8 *mac_addr);
static s32 wl_cfg80211_get_key(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, bool pairwise, const u8 *mac_addr,
	void *cookie, void (*callback) (void *cookie,
	struct key_params *params));
static s32 wl_cfg80211_config_default_mgmt_key(struct wiphy *wiphy,
	struct net_device *dev,	u8 key_idx);
static s32 wl_cfg80211_resume(struct wiphy *wiphy);
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 39)
static s32 wl_cfg80211_suspend(struct wiphy *wiphy, struct cfg80211_wowlan *wow);
#else
static s32 wl_cfg80211_suspend(struct wiphy *wiphy);
#endif
static s32 wl_cfg80211_set_pmksa(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_pmksa *pmksa);
static s32 wl_cfg80211_del_pmksa(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_pmksa *pmksa);
static s32 wl_cfg80211_flush_pmksa(struct wiphy *wiphy,
	struct net_device *dev);
static s32 wl_notify_escan_complete(struct wl_priv *wl,
	struct net_device *ndev, bool aborted, bool fw_abort);
static s32 wl_create_event_handler(struct wl_priv *wl);
static void wl_destroy_event_handler(struct wl_priv *wl);
static s32 wl_event_handler(void *data);
static void wl_init_eq(struct wl_priv *wl);
static void wl_flush_eq(struct wl_priv *wl);
static unsigned long wl_lock_eq(struct wl_priv *wl);
static void wl_unlock_eq(struct wl_priv *wl, unsigned long flags);
static void wl_init_eq_lock(struct wl_priv *wl);
static void wl_init_event_handler(struct wl_priv *wl);
static struct wl_event_q *wl_deq_event(struct wl_priv *wl);
static s32 wl_enq_event(struct wl_priv *wl, struct net_device *ndev, u32 type,
	const wl_event_msg_t *msg, void *data);
static void wl_put_event(struct wl_event_q *e);
static void wl_wakeup_event(struct wl_priv *wl);
#if defined(HTC_TX_TRACKING)
static s32 wl_notify_txfail(struct wl_priv *wl, struct net_device *ndev, const wl_event_msg_t *e, void *data);
#endif
static s32 wl_notify_connect_status_ap(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data);
static s32 wl_notify_connect_status(struct wl_priv *wl,
	struct net_device *ndev,
	const wl_event_msg_t *e, void *data);
static s32 wl_notify_roaming_status(struct wl_priv *wl,
	struct net_device *ndev,
	const wl_event_msg_t *e, void *data);
static s32 wl_notify_scan_status(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data);
static s32 wl_bss_connect_done(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data, bool completed);
static s32 wl_bss_roaming_done(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data);
static s32 wl_notify_mic_status(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data);
static s32 wl_notifier_change_state(struct wl_priv *wl, struct net_info *_net_info,
	enum wl_status state, bool set);
static void wl_cfg80211_clear_parent_dev(void);


static s32 wl_set_frag(struct net_device *dev, u32 frag_threshold);
static s32 wl_set_rts(struct net_device *dev, u32 frag_threshold);
static s32 wl_set_retry(struct net_device *dev, u32 retry, bool l);

static s32 wl_update_prof(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data, s32 item);
static void *wl_read_prof(struct wl_priv *wl, struct net_device *ndev, s32 item);
static void wl_init_prof(struct wl_priv *wl, struct net_device *ndev);

static s32 wl_set_wpa_version(struct net_device *dev,
	struct cfg80211_connect_params *sme);
static s32 wl_set_auth_type(struct net_device *dev,
	struct cfg80211_connect_params *sme);
static s32 wl_set_set_cipher(struct net_device *dev,
	struct cfg80211_connect_params *sme);
static s32 wl_set_key_mgmt(struct net_device *dev,
	struct cfg80211_connect_params *sme);
static s32 wl_set_set_sharedkey(struct net_device *dev,
	struct cfg80211_connect_params *sme);
#ifdef BCMWAPI_WPI
static s32 wl_set_set_wapi_ie(struct net_device *dev,
        struct cfg80211_connect_params *sme);
#endif
static s32 wl_get_assoc_ies(struct wl_priv *wl, struct net_device *ndev);
static void wl_ch_to_chanspec(int ch,
	struct wl_join_params *join_params, size_t *join_params_size);

static void wl_rst_ie(struct wl_priv *wl);
static __used s32 wl_add_ie(struct wl_priv *wl, u8 t, u8 l, u8 *v);
static s32 wl_mrg_ie(struct wl_priv *wl, u8 *ie_stream, u16 ie_size);
static s32 wl_cp_ie(struct wl_priv *wl, u8 *dst, u16 dst_size);
static u32 wl_get_ielen(struct wl_priv *wl);


static s32 wl_setup_wiphy(struct wireless_dev *wdev, struct device *dev);
static void wl_free_wdev(struct wl_priv *wl);

static s32 wl_inform_bss(struct wl_priv *wl);
static s32 wl_inform_single_bss(struct wl_priv *wl, struct wl_bss_info *bi);
static s32 wl_update_bss_info(struct wl_priv *wl, struct net_device *ndev);
static chanspec_t wl_cfg80211_get_shared_freq(struct wiphy *wiphy);

static s32 wl_add_keyext(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, const u8 *mac_addr,
	struct key_params *params);
static void swap_key_from_BE(struct wl_wsec_key *key);
static void swap_key_to_BE(struct wl_wsec_key *key);

static s32 wl_init_priv_mem(struct wl_priv *wl);
static void wl_deinit_priv_mem(struct wl_priv *wl);

static void wl_delay(u32 ms);

static bool wl_is_ibssmode(struct wl_priv *wl, struct net_device *ndev);
static __used bool wl_is_ibssstarter(struct wl_priv *wl);

static s32 __wl_cfg80211_up(struct wl_priv *wl);
static s32 __wl_cfg80211_down(struct wl_priv *wl);
static bool wl_is_linkdown(struct wl_priv *wl, const wl_event_msg_t *e);
static bool wl_is_linkup(struct wl_priv *wl, const wl_event_msg_t *e, struct net_device *ndev);
static bool wl_is_nonetwork(struct wl_priv *wl, const wl_event_msg_t *e);
static void wl_link_up(struct wl_priv *wl);
static void wl_link_down(struct wl_priv *wl);
static s32 wl_config_ifmode(struct wl_priv *wl, struct net_device *ndev, s32 iftype);
static void wl_init_conf(struct wl_conf *conf);

static void wl_iscan_timer(unsigned long data);
static void wl_term_iscan(struct wl_priv *wl);
static s32 wl_init_scan(struct wl_priv *wl);
static s32 wl_iscan_thread(void *data);
static s32 wl_run_iscan(struct wl_iscan_ctrl *iscan, struct cfg80211_scan_request *request,
	u16 action);
static s32 wl_do_iscan(struct wl_priv *wl,  struct cfg80211_scan_request *request);
static s32 wl_wakeup_iscan(struct wl_iscan_ctrl *iscan);
static s32 wl_invoke_iscan(struct wl_priv *wl);
static s32 wl_get_iscan_results(struct wl_iscan_ctrl *iscan, u32 *status,
	struct wl_scan_results **bss_list);
static void wl_notify_iscan_complete(struct wl_iscan_ctrl *iscan, bool aborted);
static void wl_init_iscan_handler(struct wl_iscan_ctrl *iscan);
static s32 wl_iscan_done(struct wl_priv *wl);
static s32 wl_iscan_pending(struct wl_priv *wl);
static s32 wl_iscan_inprogress(struct wl_priv *wl);
static s32 wl_iscan_aborted(struct wl_priv *wl);

s32 wl_cfg80211_scan_abort(struct net_device *ndev);
#define SCAN_FAILE_CNT	5
static int scan_timeout_cnt = 0;
static __used u32 wl_find_msb(u16 bit16);

static int wl_setup_rfkill(struct wl_priv *wl, bool setup);
static int wl_rfkill_set(void *data, bool blocked);

static wl_scan_params_t *wl_cfg80211_scan_alloc_params(int channel,
	int nprobes, int *out_params_size);
static void get_primary_mac(struct wl_priv *wl, struct ether_addr *mac);
int dhd_add_monitor(char *name, struct net_device **new_ndev);
int dhd_del_monitor(struct net_device *ndev);
int dhd_monitor_init(void *dhd_pub);
int dhd_monitor_uninit(void);
int dhd_start_xmit(struct sk_buff *skb, struct net_device *net);

#ifdef ROAM_CHANNEL_CACHE
void reset_roam_cache(void);
void add_roam_cache(wl_bss_info_t *bi);
int  get_roam_channel_list(int target_chan,
	chanspec_t *channels, const wlc_ssid_t *ssid, int ioctl_ver);
void print_roam_cache(void);
void set_roam_band(int band);
#endif

bool dhd_APUP = false;
extern void wlan_unlock_perf(void);
extern void wlan_lock_perf(void);

#define CHECK_SYS_UP(wlpriv)						\
do {									\
	struct net_device *ndev = wl_to_prmry_ndev(wlpriv);       	\
	if (unlikely(!wl_get_drv_status(wlpriv, READY, ndev))) {	\
		WL_INFO(("device is not ready\n"));			\
		return -EIO;						\
	}								\
} while (0)


#define IS_WPA_AKM(akm) ((akm) == RSN_AKM_NONE || 			\
				 (akm) == RSN_AKM_UNSPECIFIED || 	\
				 (akm) == RSN_AKM_PSK)


extern int dhd_wait_pend8021x(struct net_device *dev);
#ifdef PROP_TXSTATUS
extern int disable_proptx;
extern int dhd_wlfc_init(dhd_pub_t *dhd);
extern void dhd_wlfc_deinit(dhd_pub_t *dhd);
#endif 

#if (WL_DBG_LEVEL > 0)
#define WL_DBG_ESTR_MAX	50
static s8 wl_dbg_estr[][WL_DBG_ESTR_MAX] = {
	"SET_SSID", "JOIN", "START", "AUTH", "AUTH_IND",
	"DEAUTH", "DEAUTH_IND", "ASSOC", "ASSOC_IND", "REASSOC",
	"REASSOC_IND", "DISASSOC", "DISASSOC_IND", "QUIET_START", "QUIET_END",
	"BEACON_RX", "LINK", "MIC_ERROR", "NDIS_LINK", "ROAM",
	"TXFAIL", "PMKID_CACHE", "RETROGRADE_TSF", "PRUNE", "AUTOAUTH",
	"EAPOL_MSG", "SCAN_COMPLETE", "ADDTS_IND", "DELTS_IND", "BCNSENT_IND",
	"BCNRX_MSG", "BCNLOST_MSG", "ROAM_PREP", "PFN_NET_FOUND",
	"PFN_NET_LOST",
	"RESET_COMPLETE", "JOIN_START", "ROAM_START", "ASSOC_START",
	"IBSS_ASSOC",
	"RADIO", "PSM_WATCHDOG", "WLC_E_CCX_ASSOC_START", "WLC_E_CCX_ASSOC_ABORT",
	"PROBREQ_MSG",
	"SCAN_CONFIRM_IND", "PSK_SUP", "COUNTRY_CODE_CHANGED",
	"EXCEEDED_MEDIUM_TIME", "ICV_ERROR",
	"UNICAST_DECODE_ERROR", "MULTICAST_DECODE_ERROR", "TRACE",
	"WLC_E_BTA_HCI_EVENT", "IF", "WLC_E_P2P_DISC_LISTEN_COMPLETE",
	"RSSI", "PFN_SCAN_COMPLETE", "WLC_E_EXTLOG_MSG",
	"ACTION_FRAME", "ACTION_FRAME_COMPLETE", "WLC_E_PRE_ASSOC_IND",
	"WLC_E_PRE_REASSOC_IND", "WLC_E_CHANNEL_ADOPTED", "WLC_E_AP_STARTED",
	"WLC_E_DFS_AP_STOP", "WLC_E_DFS_AP_RESUME", "WLC_E_WAI_STA_EVENT",
	"WLC_E_WAI_MSG", "WLC_E_ESCAN_RESULT", "WLC_E_ACTION_FRAME_OFF_CHAN_COMPLETE",
	"WLC_E_PROBRESP_MSG", "WLC_E_P2P_PROBREQ_MSG", "WLC_E_DCS_REQUEST", "WLC_E_FIFO_CREDIT_MAP",
	"WLC_E_ACTION_FRAME_RX", "WLC_E_WAKE_EVENT", "WLC_E_RM_COMPLETE"
};
#endif				

#define CHAN2G(_channel, _freq, _flags) {			\
	.band			= IEEE80211_BAND_2GHZ,		\
	.center_freq		= (_freq),			\
	.hw_value		= (_channel),			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}

#define CHAN5G(_channel, _flags) {				\
	.band			= IEEE80211_BAND_5GHZ,		\
	.center_freq		= 5000 + (5 * (_channel)),	\
	.hw_value		= (_channel),			\
	.flags			= (_flags),			\
	.max_antenna_gain	= 0,				\
	.max_power		= 30,				\
}

#define RATE_TO_BASE100KBPS(rate)   (((rate) * 10) / 2)
#define RATETAB_ENT(_rateid, _flags) \
	{								\
		.bitrate	= RATE_TO_BASE100KBPS(_rateid),     \
		.hw_value	= (_rateid),			    \
		.flags	  = (_flags),			     \
	}

static struct ieee80211_rate __wl_rates[] = {
	RATETAB_ENT(WLC_RATE_1M, 0),
	RATETAB_ENT(WLC_RATE_2M, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(WLC_RATE_5M5, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(WLC_RATE_11M, IEEE80211_RATE_SHORT_PREAMBLE),
	RATETAB_ENT(WLC_RATE_6M, 0),
	RATETAB_ENT(WLC_RATE_9M, 0),
	RATETAB_ENT(WLC_RATE_12M, 0),
	RATETAB_ENT(WLC_RATE_18M, 0),
	RATETAB_ENT(WLC_RATE_24M, 0),
	RATETAB_ENT(WLC_RATE_36M, 0),
	RATETAB_ENT(WLC_RATE_48M, 0),
	RATETAB_ENT(WLC_RATE_54M, 0)
};

#define wl_a_rates		(__wl_rates + 4)
#define wl_a_rates_size	8
#define wl_g_rates		(__wl_rates + 0)
#define wl_g_rates_size	12

static struct ieee80211_channel __wl_2ghz_channels[] = {
	CHAN2G(1, 2412, 0),
	CHAN2G(2, 2417, 0),
	CHAN2G(3, 2422, 0),
	CHAN2G(4, 2427, 0),
	CHAN2G(5, 2432, 0),
	CHAN2G(6, 2437, 0),
	CHAN2G(7, 2442, 0),
	CHAN2G(8, 2447, 0),
	CHAN2G(9, 2452, 0),
	CHAN2G(10, 2457, 0),
	CHAN2G(11, 2462, 0),
	CHAN2G(12, 2467, 0),
	CHAN2G(13, 2472, 0),
	CHAN2G(14, 2484, 0)
};

static struct ieee80211_channel __wl_5ghz_a_channels[] = {
	CHAN5G(34, 0), CHAN5G(36, 0),
	CHAN5G(38, 0), CHAN5G(40, 0),
	CHAN5G(42, 0), CHAN5G(44, 0),
	CHAN5G(46, 0), CHAN5G(48, 0),
	CHAN5G(52, 0), CHAN5G(56, 0),
	CHAN5G(60, 0), CHAN5G(64, 0),
	CHAN5G(100, 0), CHAN5G(104, 0),
	CHAN5G(108, 0), CHAN5G(112, 0),
	CHAN5G(116, 0), CHAN5G(120, 0),
	CHAN5G(124, 0), CHAN5G(128, 0),
	CHAN5G(132, 0), CHAN5G(136, 0),
	CHAN5G(140, 0), CHAN5G(149, 0),
	CHAN5G(153, 0), CHAN5G(157, 0),
	CHAN5G(161, 0), CHAN5G(165, 0)
};

static struct ieee80211_supported_band __wl_band_2ghz = {
	.band = IEEE80211_BAND_2GHZ,
	.channels = __wl_2ghz_channels,
	.n_channels = ARRAY_SIZE(__wl_2ghz_channels),
	.bitrates = wl_g_rates,
	.n_bitrates = wl_g_rates_size
};

static struct ieee80211_supported_band __wl_band_5ghz_a = {
	.band = IEEE80211_BAND_5GHZ,
	.channels = __wl_5ghz_a_channels,
	.n_channels = ARRAY_SIZE(__wl_5ghz_a_channels),
	.bitrates = wl_a_rates,
	.n_bitrates = wl_a_rates_size
};

static const u32 __wl_cipher_suites[] = {
	WLAN_CIPHER_SUITE_WEP40,
	WLAN_CIPHER_SUITE_WEP104,
	WLAN_CIPHER_SUITE_TKIP,
	WLAN_CIPHER_SUITE_CCMP,
	WLAN_CIPHER_SUITE_AES_CMAC
#ifdef BCMWAPI_WPI
	,WLAN_CIPHER_SUITE_SMS4
#endif
};

#ifdef WL_CFG80211_GON_COLLISION
#define BLOCK_GON_REQ_MAX_NUM 5
#endif 

static int ioctl_version;

static chanspec_t
wl_chspec_from_legacy(chanspec_t legacy_chspec)
{
	chanspec_t chspec;

	
	chspec = LCHSPEC_CHANNEL(legacy_chspec);

	
	if (LCHSPEC_IS2G(legacy_chspec)) {
		chspec |= WL_CHANSPEC_BAND_2G;
	} else {
		chspec |= WL_CHANSPEC_BAND_5G;
	}

	
	if (LCHSPEC_IS20(legacy_chspec)) {
		chspec |= WL_CHANSPEC_BW_20;
	} else {
		chspec |= WL_CHANSPEC_BW_40;
		if (LCHSPEC_CTL_SB(legacy_chspec) == WL_LCHANSPEC_CTL_SB_LOWER) {
			chspec |= WL_CHANSPEC_CTL_SB_L;
		} else {
			chspec |= WL_CHANSPEC_CTL_SB_U;
		}
	}

	if (wf_chspec_malformed(chspec)) {
		WL_ERR(("wl_chspec_from_legacy: output chanspec (0x%04X) malformed\n",
		        chspec));
		return INVCHANSPEC;
	}

	return chspec;
}

static chanspec_t
wl_chspec_to_legacy(chanspec_t chspec)
{
	chanspec_t lchspec;

	if (wf_chspec_malformed(chspec)) {
		WL_ERR(("wl_chspec_to_legacy: input chanspec (0x%04X) malformed\n",
		        chspec));
		return INVCHANSPEC;
	}

	
	lchspec = CHSPEC_CHANNEL(chspec);

	
	if (CHSPEC_IS2G(chspec)) {
		lchspec |= WL_LCHANSPEC_BAND_2G;
	} else {
		lchspec |= WL_LCHANSPEC_BAND_5G;
	}

	
	if (CHSPEC_IS20(chspec)) {
		lchspec |= WL_LCHANSPEC_BW_20;
		lchspec |= WL_LCHANSPEC_CTL_SB_NONE;
	} else if (CHSPEC_IS40(chspec)) {
		lchspec |= WL_LCHANSPEC_BW_40;
		if (CHSPEC_CTL_SB(chspec) == WL_CHANSPEC_CTL_SB_L) {
			lchspec |= WL_LCHANSPEC_CTL_SB_LOWER;
		} else {
			lchspec |= WL_LCHANSPEC_CTL_SB_UPPER;
		}
	} else {
		
		char chanbuf[CHANSPEC_STR_LEN];
		WL_ERR((
		        "wl_chspec_to_legacy: unable to convert chanspec %s (0x%04X) "
		        "to pre-11ac format\n",
		        wf_chspec_ntoa(chspec, chanbuf), chspec));
		return INVCHANSPEC;
	}

	return lchspec;
}

static chanspec_t
wl_chspec_host_to_driver(chanspec_t chanspec)
{
	if (ioctl_version == 1) {
		chanspec = wl_chspec_to_legacy(chanspec);
		if (chanspec == INVCHANSPEC) {
			return chanspec;
		}
	}
	chanspec = htodchanspec(chanspec);

	return chanspec;
}

chanspec_t
wl_ch_host_to_driver(u16 channel)
{

	chanspec_t chanspec;

	chanspec = channel & WL_CHANSPEC_CHAN_MASK;

	if (channel <= CH_MAX_2G_CHANNEL)
		chanspec |= WL_CHANSPEC_BAND_2G;
	else
		chanspec |= WL_CHANSPEC_BAND_5G;

	chanspec |= WL_CHANSPEC_BW_20;
	chanspec |= WL_CHANSPEC_CTL_SB_NONE;

	return wl_chspec_host_to_driver(chanspec);
}

static chanspec_t
wl_chspec_driver_to_host(chanspec_t chanspec)
{
	chanspec = dtohchanspec(chanspec);
	if (ioctl_version == 1) {
		chanspec = wl_chspec_from_legacy(chanspec);
	}

	return chanspec;
}

static const struct ieee80211_txrx_stypes
wl_cfg80211_default_mgmt_stypes[NUM_NL80211_IFTYPES] = {
	[NL80211_IFTYPE_ADHOC] = {
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4)
	},
	[NL80211_IFTYPE_STATION] = {
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
		BIT(IEEE80211_STYPE_PROBE_REQ >> 4)
	},
	[NL80211_IFTYPE_AP] = {
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
		BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
		BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
		BIT(IEEE80211_STYPE_DISASSOC >> 4) |
		BIT(IEEE80211_STYPE_AUTH >> 4) |
		BIT(IEEE80211_STYPE_DEAUTH >> 4) |
		BIT(IEEE80211_STYPE_ACTION >> 4)
	},
	[NL80211_IFTYPE_AP_VLAN] = {
		
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
		BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
		BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
		BIT(IEEE80211_STYPE_DISASSOC >> 4) |
		BIT(IEEE80211_STYPE_AUTH >> 4) |
		BIT(IEEE80211_STYPE_DEAUTH >> 4) |
		BIT(IEEE80211_STYPE_ACTION >> 4)
	},
	[NL80211_IFTYPE_P2P_CLIENT] = {
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ACTION >> 4) |
		BIT(IEEE80211_STYPE_PROBE_REQ >> 4)
	},
	[NL80211_IFTYPE_P2P_GO] = {
		.tx = 0xffff,
		.rx = BIT(IEEE80211_STYPE_ASSOC_REQ >> 4) |
		BIT(IEEE80211_STYPE_REASSOC_REQ >> 4) |
		BIT(IEEE80211_STYPE_PROBE_REQ >> 4) |
		BIT(IEEE80211_STYPE_DISASSOC >> 4) |
		BIT(IEEE80211_STYPE_AUTH >> 4) |
		BIT(IEEE80211_STYPE_DEAUTH >> 4) |
		BIT(IEEE80211_STYPE_ACTION >> 4)
	}
};

static void swap_key_from_BE(struct wl_wsec_key *key)
{
	key->index = htod32(key->index);
	key->len = htod32(key->len);
	key->algo = htod32(key->algo);
	key->flags = htod32(key->flags);
	key->rxiv.hi = htod32(key->rxiv.hi);
	key->rxiv.lo = htod16(key->rxiv.lo);
	key->iv_initialized = htod32(key->iv_initialized);
}

static void swap_key_to_BE(struct wl_wsec_key *key)
{
	key->index = dtoh32(key->index);
	key->len = dtoh32(key->len);
	key->algo = dtoh32(key->algo);
	key->flags = dtoh32(key->flags);
	key->rxiv.hi = dtoh32(key->rxiv.hi);
	key->rxiv.lo = dtoh16(key->rxiv.lo);
	key->iv_initialized = dtoh32(key->iv_initialized);
}

#if 1 
static void
wl_validate_wps_ie(char *wps_ie, bool *pbc)
{
	#define WPS_IE_FIXED_LEN 6
	u16 len = (u16) wps_ie[TLV_LEN_OFF];
	u8 *subel = wps_ie+  WPS_IE_FIXED_LEN;
	u16 subelt_id;
	u16 subelt_len;
	u16 val;
	u8 *valptr = (uint8*) &val;

	WL_DBG(("wps_ie len=%d\n", len));

	len -= 4;	

	while (len >= 4) {		
		valptr[0] = *subel++;
		valptr[1] = *subel++;
		subelt_id = HTON16(val);

		valptr[0] = *subel++;
		valptr[1] = *subel++;
		subelt_len = HTON16(val);

		len -= 4;			
		len -= subelt_len;	
		WL_DBG((" subel=%p, subelt_id=0x%x subelt_len=%u\n",
			subel, subelt_id, subelt_len));

		if (subelt_id == WPS_ID_VERSION) {
			WL_DBG(("  attr WPS_ID_VERSION: %u\n", *subel));
		} else if (subelt_id == WPS_ID_REQ_TYPE) {
			WL_DBG(("  attr WPS_ID_REQ_TYPE: %u\n", *subel));
		} else if (subelt_id == WPS_ID_CONFIG_METHODS) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_CONFIG_METHODS: %x\n", HTON16(val)));
		} else if (subelt_id == WPS_ID_DEVICE_NAME) {
			char devname[100];
			memcpy(devname, subel, subelt_len);
			devname[subelt_len] = '\0';
			WL_DBG(("  attr WPS_ID_DEVICE_NAME: %s (len %u)\n",
				devname, subelt_len));
		} else if (subelt_id == WPS_ID_DEVICE_PWD_ID) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_DEVICE_PWD_ID: %u\n", HTON16(val)));
			*pbc = (HTON16(val) == DEV_PW_PUSHBUTTON) ? true : false;
		} else if (subelt_id == WPS_ID_PRIM_DEV_TYPE) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_PRIM_DEV_TYPE: cat=%u \n", HTON16(val)));
			valptr[0] = *(subel + 6);
			valptr[1] = *(subel + 7);
			WL_DBG(("  attr WPS_ID_PRIM_DEV_TYPE: subcat=%u\n", HTON16(val)));
		} else if (subelt_id == WPS_ID_REQ_DEV_TYPE) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_REQ_DEV_TYPE: cat=%u\n", HTON16(val)));
			valptr[0] = *(subel + 6);
			valptr[1] = *(subel + 7);
			WL_DBG(("  attr WPS_ID_REQ_DEV_TYPE: subcat=%u\n", HTON16(val)));
		} else if (subelt_id == WPS_ID_SELECTED_REGISTRAR_CONFIG_METHODS) {
			valptr[0] = *subel;
			valptr[1] = *(subel + 1);
			WL_DBG(("  attr WPS_ID_SELECTED_REGISTRAR_CONFIG_METHODS"
				": cat=%u\n", HTON16(val)));
		} else {
			WL_DBG(("  unknown attr 0x%x\n", subelt_id));
		}

		subel += subelt_len;
	}
}
#endif 

static chanspec_t wl_cfg80211_get_shared_freq(struct wiphy *wiphy)
{
	if (vsdb_supported) {
		return wl_ch_host_to_driver(WL_P2P_TEMP_CHAN);
	}
	else {
		chanspec_t chspec;
		int err = 0;
		struct wl_priv *wl = wiphy_priv(wiphy);
		struct net_device *dev = wl_to_prmry_ndev(wl);
		struct ether_addr bssid;
		struct wl_bss_info *bss = NULL;

		if ((err = wldev_ioctl(dev, WLC_GET_BSSID, &bssid, sizeof(bssid), false))) {
			WL_DBG(("Not associated. Return a temp channel. \n"));
			return wl_ch_host_to_driver(WL_P2P_TEMP_CHAN);
		}


		*(u32 *) wl->extra_buf = htod32(WL_EXTRA_BUF_MAX);
		if ((err = wldev_ioctl(dev, WLC_GET_BSS_INFO, wl->extra_buf,
			WL_EXTRA_BUF_MAX, false))) {
				WL_ERR(("Failed to get associated bss info, use temp channel \n"));
				chspec = wl_ch_host_to_driver(WL_P2P_TEMP_CHAN);
		}
		else {
				bss = (struct wl_bss_info *) (wl->extra_buf + 4);
				chspec =  bss->chanspec;
				WL_DBG(("Valid BSS Found. chanspec:%d \n", bss->chanspec));
		}
		return chspec;
	}
}

static struct net_device* wl_cfg80211_add_monitor_if(char *name)
{
#if defined(WLP2P) && defined(WL_ENABLE_P2P_IF)
	WL_INFO(("wl_cfg80211_add_monitor_if: No more support monitor interface\n"));
	return ERR_PTR(-EOPNOTSUPP);
#else
	struct net_device* ndev = NULL;

	dhd_add_monitor(name, &ndev);
	WL_INFO(("wl_cfg80211_add_monitor_if net device returned: 0x%p\n", ndev));
	return ndev;
#endif 
}

static struct net_device *
wl_cfg80211_add_virtual_iface(struct wiphy *wiphy, char *name,
	enum nl80211_iftype type, u32 *flags,
	struct vif_params *params)
{
	s32 err;
	s32 timeout = -1;
	s32 wlif_type = -1;
	s32 mode = 0;
	s32 val = 0;
#if defined(WL_ENABLE_P2P_IF)
	s32 dhd_mode = 0;
#endif 
	chanspec_t chspec;
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct net_device *_ndev;
	struct ether_addr primary_mac;
	int (*net_attach)(void *dhdp, int ifidx);
	bool rollback_lock = false;
	int bcn_timeout = 40;
	int dtim = 1;
#ifdef PROP_TXSTATUS
	s32 up = 1;
	dhd_pub_t *dhd;
#endif 

	if (!wl)
		return ERR_PTR(-EINVAL);

#ifdef PROP_TXSTATUS
	dhd = (dhd_pub_t *)(wl->pub);
#endif 

	
	_ndev = wl_to_prmry_ndev(wl);

	WL_DBG(("if name: %s, type: %d\n", name, type));
	switch (type) {
	case NL80211_IFTYPE_ADHOC:
	case NL80211_IFTYPE_AP_VLAN:
	case NL80211_IFTYPE_WDS:
	case NL80211_IFTYPE_MESH_POINT:
		WL_ERR(("Unsupported interface type\n"));
		mode = WL_MODE_IBSS;
		return NULL;
	case NL80211_IFTYPE_MONITOR:
		return wl_cfg80211_add_monitor_if(name);
	case NL80211_IFTYPE_P2P_CLIENT:
	case NL80211_IFTYPE_STATION:
		wlif_type = WL_P2P_IF_CLIENT;
		mode = WL_MODE_BSS;
		break;
	case NL80211_IFTYPE_P2P_GO:
	case NL80211_IFTYPE_AP:
		wlif_type = WL_P2P_IF_GO;
		mode = WL_MODE_AP;
		break;
	default:
		WL_ERR(("Unsupported interface type\n"));
		return NULL;
		break;
	}

	if (!name) {
		WL_ERR(("name is NULL\n"));
		return NULL;
	}

	if (wl->p2p_supported && (wlif_type != -1)) {
		if (wl_get_p2p_status(wl, IF_DELETING)) {
			if (rtnl_is_locked()) {
				rtnl_unlock();
				rollback_lock = true;
			}
			WL_INFO(("%s: Released the lock and wait till IF_DEL is complete\n",
				__func__));
			timeout = wait_event_interruptible_timeout(wl->netif_change_event,
				(wl_get_p2p_status(wl, IF_DELETING) == false),
				msecs_to_jiffies(MAX_WAIT_TIME));

			
			if (rollback_lock) {
				rtnl_lock();
				rollback_lock = false;
			}
			if (timeout > 0) {
				WL_ERR(("IF DEL is Success\n"));

			} else {
				WL_ERR(("timeount < 0, return -EAGAIN\n"));
				return ERR_PTR(-EAGAIN);
			}
			if (wl->iface_cnt == IFACE_MAX_CNT)
				return ERR_PTR(-ENOMEM);
		}

#ifdef PROP_TXSTATUS
		if (!dhd)
			return ERR_PTR(-ENODEV);
#endif 
		if (!wl->p2p || !wl->p2p->vir_ifname)
			return ERR_PTR(-ENODEV);

		if (wl->p2p && !wl->p2p->on && strstr(name, WL_P2P_INTERFACE_PREFIX)) {
			p2p_on(wl) = true;
			wl_cfgp2p_set_firm_p2p(wl);
			wl_cfgp2p_init_discovery(wl);
			get_primary_mac(wl, &primary_mac);
			wl_cfgp2p_generate_bss_mac(&primary_mac,
				&wl->p2p->dev_addr, &wl->p2p->int_addr);
		}

		memset(wl->p2p->vir_ifname, 0, IFNAMSIZ);
		strncpy(wl->p2p->vir_ifname, name, IFNAMSIZ - 1);

		wl_notify_escan_complete(wl, _ndev, true, true);
#ifdef PROP_TXSTATUS
		if (!wl->wlfc_on && !disable_proptx) {
			dhd->wlfc_enabled = true;
			dhd_wlfc_init(dhd);
			err = wldev_ioctl(_ndev, WLC_UP, &up, sizeof(s32), true);
			if (err < 0)
				WL_ERR(("WLC_UP return err:%d\n", err));
			wl->wlfc_on = true;
		}
#endif 

		 chspec = wl_cfg80211_get_shared_freq(wiphy);

		wl_set_p2p_status(wl, IF_ADD);
		if (wlif_type == WL_P2P_IF_GO)
			wldev_iovar_setint(_ndev, "mpc", 0);
		err = wl_cfgp2p_ifadd(wl, &wl->p2p->int_addr, htod32(wlif_type), chspec);

		if (unlikely(err)) {
			WL_ERR((" virtual iface add failed (%d) \n", err));
			return ERR_PTR(-ENOMEM);
		}

		timeout = wait_event_interruptible_timeout(wl->netif_change_event,
			(wl_get_p2p_status(wl, IF_ADD) == false),
			msecs_to_jiffies(MAX_WAIT_TIME));
		if (timeout > 0 && (!wl_get_p2p_status(wl, IF_ADD))) {

			struct wireless_dev *vwdev;
			vwdev = kzalloc(sizeof(*vwdev), GFP_KERNEL);
			if (unlikely(!vwdev)) {
				WL_ERR(("Could not allocate wireless device\n"));
				return ERR_PTR(-ENOMEM);
			}
			
			if(vwdev != NULL)
				vwdev->wiphy = wl->wdev->wiphy;
			
			WL_INFO((" virtual interface(%s) is created memalloc done \n",
				wl->p2p->vir_ifname));
			vwdev->iftype = type;
			_ndev =  wl_to_p2p_bss_ndev(wl, P2PAPI_BSSCFG_CONNECTION);
			_ndev->ieee80211_ptr = vwdev;
			SET_NETDEV_DEV(_ndev, wiphy_dev(vwdev->wiphy));
			vwdev->netdev = _ndev;
			wl_set_drv_status(wl, READY, _ndev);
			wl->p2p->vif_created = true;
			wl_set_mode_by_netdev(wl, _ndev, mode);
			net_attach =  wl_to_p2p_bss_private(wl, P2PAPI_BSSCFG_CONNECTION);

			
			if ((err = wldev_iovar_setint(_ndev, "bcn_timeout", bcn_timeout))) {
				printf("%s: bcn_timeout setting error\n", __func__);
			}
			if ((err = wldev_ioctl(_ndev, WLC_SET_DTIMPRD, &dtim, sizeof(dtim), 1))) {
				printf("%s: dtim setting error\n", __func__);
			}

			if (rtnl_is_locked()) {
				rtnl_unlock();
				rollback_lock = true;
			}
			if (net_attach && !net_attach(wl->pub, _ndev->ifindex)) {
				wl_alloc_netinfo(wl, _ndev, vwdev, mode, PM_ENABLE);
				val = 1;
				
				wldev_iovar_setint(_ndev, "roam_off", val);
				WL_ERR((" virtual interface(%s) is "
					"created net attach done\n", wl->p2p->vir_ifname));
				if (mode == WL_MODE_AP)
					wl_set_drv_status(wl, CONNECTED, _ndev);
#if defined(WL_ENABLE_P2P_IF)
				if (type == NL80211_IFTYPE_P2P_CLIENT)
					dhd_mode = P2P_GC_ENABLED;
				else if (type == NL80211_IFTYPE_P2P_GO)
					dhd_mode = P2P_GO_ENABLED;
				DNGL_FUNC(dhd_cfg80211_set_p2p_info, (wl, dhd_mode));
#endif 
			} else {
				
				if (rollback_lock)
					rtnl_lock();
				goto fail;
			}
			
			if (rollback_lock)
				rtnl_lock();
			return _ndev;

		} else {
			wl_clr_p2p_status(wl, IF_ADD);
			WL_ERR((" virtual interface(%s) is not created \n", wl->p2p->vir_ifname));
			memset(wl->p2p->vir_ifname, '\0', IFNAMSIZ);
			wl->p2p->vif_created = false;
#ifdef PROP_TXSTATUS
		if (dhd->wlfc_enabled && wl->wlfc_on) {
			dhd->wlfc_enabled = false;
			dhd_wlfc_deinit(dhd);
			wl->wlfc_on = false;
		}
#endif 
		}
	}
fail:
	if (wlif_type == WL_P2P_IF_GO)
		wldev_iovar_setint(_ndev, "mpc", 1);
	return ERR_PTR(-ENODEV);
}

static s32
wl_cfg80211_del_virtual_iface(struct wiphy *wiphy, struct net_device *dev)
{
	struct ether_addr p2p_mac;
	struct wl_priv *wl = wiphy_priv(wiphy);
	s32 timeout = -1;
	s32 ret = 0;
	WL_DBG(("Enter\n"));

	if (wl->p2p_net == dev) {
		dev = wl_to_prmry_ndev(wl);
	}

	if (wl->p2p_supported) {
		memcpy(p2p_mac.octet, wl->p2p->int_addr.octet, ETHER_ADDR_LEN);
		WL_DBG(("P2P: GO_NEG_PHASE status cleared "));
		wl_clr_p2p_status(wl, GO_NEG_PHASE);
		if (wl->p2p->vif_created) {
			if (wl_get_drv_status(wl, SCANNING, dev)) {
				wl_notify_escan_complete(wl, dev, true, true);
			}
			wldev_iovar_setint(dev, "mpc", 1);
			
			if (wl_get_drv_status(wl, DISCONNECTING, dev) &&
				(wl_get_mode_by_netdev(wl, dev) != WL_MODE_AP)) {
				WL_ERR(("Wait for Link Down event for GC !\n"));
				wait_for_completion_timeout
					(&wl->iface_disable, msecs_to_jiffies(500));
			}
			wl_set_p2p_status(wl, IF_DELETING);
			
			if (wl_get_mode_by_netdev(wl, dev) == WL_MODE_AP) {
				
				ret = wl_cfgp2p_ifdisable(wl, &p2p_mac);
				if (ret == 0) {
					WL_ERR(("Wait for Link Down event for GO !!!\n"));
					wait_for_completion_timeout(&wl->iface_disable,
						msecs_to_jiffies(500));
				} else {
					msleep(300);
				}
			}
			
			ret = wl_cfgp2p_ifdel(wl, &p2p_mac);
			
			if (ret) {
				struct net_device *ndev = wl_to_prmry_ndev(wl);
				WL_ERR(("Firmware returned an error (%d) from p2p_ifdel"
					"HANG Notification sent to %s\n", ret, ndev->name));
				wl_cfg80211_hang(ndev, WLAN_REASON_UNSPECIFIED);
			}

			
			timeout = wait_event_interruptible_timeout(wl->netif_change_event,
				(wl->p2p->vif_created == false),
				msecs_to_jiffies(MAX_WAIT_TIME));
			if (timeout > 0 && (wl->p2p->vif_created == false)) {
				WL_DBG(("IFDEL operation done\n"));
#if  defined(WL_ENABLE_P2P_IF)
				DNGL_FUNC(dhd_cfg80211_clean_p2p_info, (wl));
#endif 
			} else {
				WL_ERR(("IFDEL didn't complete properly\n"));
			}
			ret = dhd_del_monitor(dev);
		}
	}
	return ret;
}

static s32
wl_cfg80211_change_virtual_iface(struct wiphy *wiphy, struct net_device *ndev,
	enum nl80211_iftype type, u32 *flags,
	struct vif_params *params)
{
	s32 ap = 0;
	s32 infra = 0;
	s32 wlif_type;
	s32 mode = 0;
	chanspec_t chspec;
	struct wl_priv *wl = wiphy_priv(wiphy);

	WL_DBG(("Enter \n"));
	switch (type) {
	case NL80211_IFTYPE_MONITOR:
	case NL80211_IFTYPE_WDS:
	case NL80211_IFTYPE_MESH_POINT:
		ap = 1;
		WL_ERR(("type (%d) : currently we do not support this type\n",
			type));
		break;
	case NL80211_IFTYPE_ADHOC:
		mode = WL_MODE_IBSS;
		break;
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_P2P_CLIENT:
		mode = WL_MODE_BSS;
		infra = 1;
		break;
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_AP_VLAN:
	case NL80211_IFTYPE_P2P_GO:
		mode = WL_MODE_AP;
		ap = 1;
		break;
	default:
		return -EINVAL;
	}
	WL_DBG(("%s : ap (%d), infra (%d), iftype: (%d)\n", ndev->name, ap, infra, type));

	if (ap) {
		wl_set_mode_by_netdev(wl, ndev, mode);
		if (wl->p2p_supported && wl->p2p->vif_created) {
			WL_DBG(("p2p_vif_created (%d) p2p_on (%d)\n", wl->p2p->vif_created,
			p2p_on(wl)));
			wldev_iovar_setint(ndev, "mpc", 0);
			wl_notify_escan_complete(wl, ndev, true, true);

			chspec = wl_cfg80211_get_shared_freq(wiphy);

			wlif_type = WL_P2P_IF_GO;
			WL_ERR(("%s : ap (%d), infra (%d), iftype: (%d)\n",
				ndev->name, ap, infra, type));
			wl_set_p2p_status(wl, IF_CHANGING);
			wl_clr_p2p_status(wl, IF_CHANGED);
			wl_cfgp2p_ifchange(wl, &wl->p2p->int_addr, htod32(wlif_type), chspec);
			wait_event_interruptible_timeout(wl->netif_change_event,
				(wl_get_p2p_status(wl, IF_CHANGED) == true),
				msecs_to_jiffies(MAX_WAIT_TIME));
			wl_set_mode_by_netdev(wl, ndev, mode);
			wl_clr_p2p_status(wl, IF_CHANGING);
			wl_clr_p2p_status(wl, IF_CHANGED);
			if (mode == WL_MODE_AP)
				wl_set_drv_status(wl, CONNECTED, ndev);
		} else if (ndev == wl_to_prmry_ndev(wl) &&
			!wl_get_drv_status(wl, AP_CREATED, ndev)) {
			wl_set_drv_status(wl, AP_CREATING, ndev);
			if (!wl->ap_info &&
				!(wl->ap_info = kzalloc(sizeof(struct ap_info), GFP_KERNEL))) {
				WL_ERR(("struct ap_saved_ie allocation failed\n"));
				return -ENOMEM;
			}
		} else {
			WL_ERR(("Cannot change the interface for GO or SOFTAP\n"));
			return -EINVAL;
		}
	} else {
		WL_ERR(("INVALID:change_virtual_iface for transition from GO/AP to client/STA"));
	}

	ndev->ieee80211_ptr->iftype = type;
	return 0;
}

s32
wl_cfg80211_notify_ifadd(struct net_device *ndev, s32 idx, s32 bssidx,
	void* _net_attach)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	s32 ret = BCME_OK;
	WL_DBG(("Enter"));
	if (!ndev) {
		WL_ERR(("net is NULL\n"));
		return 0;
	}

#ifdef APSTA_CONCURRENT
	if (wl->apsta_concurrent){
		return BCME_ERROR;
	}
#endif

	if (wl->p2p_supported && wl_get_p2p_status(wl, IF_ADD)) {
		WL_DBG(("IF_ADD event called from dongle, old interface name: %s,"
			"new name: %s\n", ndev->name, wl->p2p->vir_ifname));
		
		strncpy(ndev->name, wl->p2p->vir_ifname, IFNAMSIZ - 1);
		wl_to_p2p_bss_ndev(wl, P2PAPI_BSSCFG_CONNECTION) = ndev;
		wl_to_p2p_bss_bssidx(wl, P2PAPI_BSSCFG_CONNECTION) = bssidx;
		wl_to_p2p_bss_private(wl, P2PAPI_BSSCFG_CONNECTION) = _net_attach;
		ndev->ifindex = idx;
		wl_clr_p2p_status(wl, IF_ADD);

		wake_up_interruptible(&wl->netif_change_event);
	} else {
		ret = BCME_NOTREADY;
	}
	return ret;
}

s32
wl_cfg80211_notify_ifdel(void)
{
	struct wl_priv *wl = wlcfg_drv_priv;

	WL_DBG(("Enter \n"));
	wl_clr_p2p_status(wl, IF_DELETING);
	wake_up_interruptible(&wl->netif_change_event);
	return 0;
}

s32
wl_cfg80211_ifdel_ops(struct net_device *ndev)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	bool rollback_lock = false;
	s32 index = 0;
#ifdef PROP_TXSTATUS
	dhd_pub_t *dhd =  (dhd_pub_t *)(wl->pub);
#endif 
	if (!ndev || (strlen(ndev->name) == 0)) {
		WL_ERR(("net is NULL\n"));
		return 0;
	}
#ifdef APSTA_CONCURRENT
	if (wl->apsta_concurrent){
		wl->apsta_concurrent = FALSE;
		return BCME_ERROR;
	}
#endif

	if (p2p_is_on(wl) && wl->p2p->vif_created &&
		wl_get_p2p_status(wl, IF_DELETING)) {
		if (wl->scan_request &&
			(wl->escan_info.ndev == ndev)) {
			
			wl->escan_info.escan_state = WL_ESCAN_STATE_IDLE;
			if (!rtnl_is_locked()) {
				rtnl_lock();
				rollback_lock = true;
			}
			WL_DBG(("ESCAN COMPLETED\n"));
			wl_notify_escan_complete(wl, ndev, true, false);
			if (rollback_lock)
				rtnl_unlock();
		}
		WL_ERR(("IF_DEL event called from dongle, net %x, vif name: %s\n",
			(unsigned int)ndev, wl->p2p->vir_ifname));

		memset(wl->p2p->vir_ifname, '\0', IFNAMSIZ);
		index = wl_cfgp2p_find_idx(wl, ndev);
		wl_to_p2p_bss_ndev(wl, index) = NULL;
		wl_to_p2p_bss_bssidx(wl, index) = 0;
		wl->p2p->vif_created = false;
		wl_cfgp2p_clear_management_ie(wl,
			index);
		WL_DBG(("index : %d\n", index));

#ifdef PROP_TXSTATUS
		if (dhd->wlfc_enabled && wl->wlfc_on) {
			dhd->wlfc_enabled = false;
			dhd_wlfc_deinit(dhd);
			wl->wlfc_on = false;
		}
#endif 
		wl_clr_drv_status(wl, CONNECTED, ndev);
	}
	
	wake_up_interruptible(&wl->netif_change_event);

	return 0;
}

s32
wl_cfg80211_is_progress_ifadd(void)
{
	s32 is_progress = 0;
	struct wl_priv *wl = wlcfg_drv_priv;
	if (wl_get_p2p_status(wl, IF_ADD))
		is_progress = 1;
	return is_progress;
}

s32
wl_cfg80211_is_progress_ifchange(void)
{
	s32 is_progress = 0;
	struct wl_priv *wl = wlcfg_drv_priv;
	if (wl_get_p2p_status(wl, IF_CHANGING))
		is_progress = 1;
	return is_progress;
}


s32
wl_cfg80211_notify_ifchange(void)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	if (wl_get_p2p_status(wl, IF_CHANGING)) {
		wl_set_p2p_status(wl, IF_CHANGED);
		wake_up_interruptible(&wl->netif_change_event);
	}
	return 0;
}

static s32 wl_find_listen_channel(struct wl_priv *wl,
	u8 *ie, u32 ie_len)
{
	wifi_p2p_ie_t *p2p_ie;
	u8 *end, *pos;
	s32 listen_channel;

	p2p_ie = wl_cfgp2p_find_p2pie(ie, ie_len);

	if (p2p_ie == NULL)
		return 0;

	pos = p2p_ie->subelts;
	end = p2p_ie->subelts + (p2p_ie->len - 4);

	CFGP2P_DBG((" found p2p ie ! lenth %d \n",
		p2p_ie->len));

	while (pos < end) {
		uint16 attr_len;
		if (pos + 2 >= end) {
			CFGP2P_DBG((" -- Invalid P2P attribute"));
			return 0;
		}
		attr_len = ((uint16) (((pos + 1)[1] << 8) | (pos + 1)[0]));

		if (pos + 3 + attr_len > end) {
			CFGP2P_DBG(("P2P: Attribute underflow "
				   "(len=%u left=%d)",
				   attr_len, (int) (end - pos - 3)));
			return 0;
		}

		if (pos[0] == 6) {
			listen_channel = pos[1 + 2 + 3 + 1];

			if (listen_channel == SOCIAL_CHAN_1 ||
				listen_channel == SOCIAL_CHAN_2 ||
				listen_channel == SOCIAL_CHAN_3) {
				CFGP2P_DBG((" Found my Listen Channel %d \n", listen_channel));
				return listen_channel;
			}
		}
		pos += 3 + attr_len;
	}
	return 0;
}

#if defined(CUSTOMER_HW4)
static void wl_scan_prep(struct wl_scan_params *params,
	struct cfg80211_scan_request *request, int32 active_time)
#else
static void wl_scan_prep(struct wl_scan_params *params, struct cfg80211_scan_request *request)
#endif 
{
	u32 n_ssids;
	u32 n_channels;
	u16 channel;
	chanspec_t chanspec;
	s32 i = 0, offset;
	char *ptr;
	wlc_ssid_t ssid;
	struct wl_priv *wl = wlcfg_drv_priv;

	memcpy(&params->bssid, &ether_bcast, ETHER_ADDR_LEN);
	params->bss_type = DOT11_BSSTYPE_ANY;
	params->scan_type = 0;
	params->nprobes = -1;
#if defined(CUSTOMER_HW4)
	params->active_time = active_time;
#else
	params->active_time = -1;
#endif
	params->passive_time = -1;
	params->home_time = -1;
	params->channel_num = 0;
	memset(&params->ssid, 0, sizeof(wlc_ssid_t));

	WL_SCAN(("Preparing Scan request\n"));
	WL_SCAN(("nprobes=%d\n", params->nprobes));
	WL_SCAN(("active_time=%d\n", params->active_time));
	WL_SCAN(("passive_time=%d\n", params->passive_time));
	WL_SCAN(("home_time=%d\n", params->home_time));
	WL_SCAN(("scan_type=%d\n", params->scan_type));

	params->nprobes = htod32(params->nprobes);
	params->active_time = htod32(params->active_time);
	params->passive_time = htod32(params->passive_time);
	params->home_time = htod32(params->home_time);

	
	if (!request)
		return;

#ifdef APSTA_CONCURRENT
	if( request->ie && request->ie[0] == 0xdd &&
		request->ie[2] == 0x00 && request->ie[3] == 0x09 &&
		request->ie[4] == 0x2d ) {
		params->nprobes = 5;
		params->active_time = 20;
		printf("%s HTC specific IE found size, set nprobes = 5\n",__FUNCTION__);
	}
#endif

	n_ssids = request->n_ssids;
	n_channels = request->n_channels;

	
	WL_SCAN(("### List of channelspecs to scan ###\n"));
	if (n_channels > 0) {
		for (i = 0; i < n_channels; i++) {
			chanspec = 0;
			channel = ieee80211_frequency_to_channel(request->channels[i]->center_freq);
			
			if ((wl->escan_info.ndev != wl_to_prmry_ndev(wl)) &&
				(request->channels[i]->flags &
				(IEEE80211_CHAN_RADAR | IEEE80211_CHAN_PASSIVE_SCAN)))
				continue;
			if (request->channels[i]->band == IEEE80211_BAND_2GHZ)
				chanspec |= WL_CHANSPEC_BAND_2G;
			else
				chanspec |= WL_CHANSPEC_BAND_5G;

			chanspec |= WL_CHANSPEC_BW_20;
			chanspec |= WL_CHANSPEC_CTL_SB_NONE;

			params->channel_list[i] = channel;
			params->channel_list[i] &= WL_CHANSPEC_CHAN_MASK;
			params->channel_list[i] |= chanspec;
			WL_SCAN(("Chan : %d, Channel spec: %x \n",
				channel, params->channel_list[i]));
			params->channel_list[i] = wl_chspec_host_to_driver(params->channel_list[i]);
		}
	} else {
		WL_SCAN(("Scanning all channels\n"));
	}

	n_channels = i;
	
	WL_SCAN(("### List of SSIDs to scan ###\n"));
	if (n_ssids > 0) {
		offset = offsetof(wl_scan_params_t, channel_list) + n_channels * sizeof(u16);
		offset = roundup(offset, sizeof(u32));
		ptr = (char*)params + offset;
		for (i = 0; i < n_ssids; i++) {
			memset(&ssid, 0, sizeof(wlc_ssid_t));
			ssid.SSID_len = request->ssids[i].ssid_len;
			memcpy(ssid.SSID, request->ssids[i].ssid, ssid.SSID_len);
			if (!ssid.SSID_len)
				WL_SCAN(("%d: Broadcast scan\n", i));
			else
				WL_SCAN(("%d: scan  for  %s size =%d\n", i,
				ssid.SSID, ssid.SSID_len));
			memcpy(ptr, &ssid, sizeof(wlc_ssid_t));
			ptr += sizeof(wlc_ssid_t);
		}
	} else {
		WL_SCAN(("Broadcast scan\n"));
	}
	
	params->channel_num =
	        htod32((n_ssids << WL_SCAN_PARAMS_NSSID_SHIFT) |
	               (n_channels & WL_SCAN_PARAMS_COUNT_MASK));
	if (n_channels == 1 && wl_get_drv_status_all(wl, CONNECTED)) {
		params->active_time = WL_SCAN_CONNECT_DWELL_TIME_MS;
	}
}

static s32
wl_run_iscan(struct wl_iscan_ctrl *iscan, struct cfg80211_scan_request *request, u16 action)
{
	u32 n_channels;
	u32 n_ssids;
	s32 params_size =
	    (WL_SCAN_PARAMS_FIXED_SIZE + offsetof(wl_iscan_params_t, params));
	struct wl_iscan_params *params = NULL;
	s32 err = 0;

	if (!request) {
			err = -EINVAL;
			goto done;
	} else {
		n_channels = request->n_channels;
		n_ssids = request->n_ssids;
		
		if (n_channels % 2)
			
			params_size += sizeof(u16) * (n_channels + 1);
		else
			params_size += sizeof(u16) * n_channels;

		
		params_size += sizeof(struct wlc_ssid) * n_ssids;
	}
	params = (struct wl_iscan_params *)kzalloc(params_size, GFP_KERNEL);
	if (!params) {
		err = -ENOMEM;
		goto done;
	}

	if (request != NULL)
#if defined(CUSTOMER_HW4)
		wl_scan_prep(&params->params, request, -1);
#else
		wl_scan_prep(&params->params, request);
#endif

	params->version = htod32(ISCAN_REQ_VERSION);
	params->action = htod16(action);
	params->scan_duration = htod16(0);

	if (params_size + sizeof("iscan") >= WLC_IOCTL_MEDLEN) {
		WL_ERR(("ioctl buffer length is not sufficient\n"));
		err = -ENOMEM;
		goto done;
	}
	err = wldev_iovar_setbuf(iscan->dev, "iscan", params, params_size,
		iscan->ioctl_buf, WLC_IOCTL_MEDLEN, NULL);
	if (unlikely(err)) {
		if (err == -EBUSY) {
			WL_ERR(("system busy : iscan canceled\n"));
		} else {
			WL_ERR(("error (%d)\n", err));
		}
	}

done:
	if (params)
		kfree(params);
	return err;
}

static s32 wl_do_iscan(struct wl_priv *wl, struct cfg80211_scan_request *request)
{
	struct wl_iscan_ctrl *iscan = wl_to_iscan(wl);
	struct net_device *ndev = wl_to_prmry_ndev(wl);
	s32 passive_scan;
	s32 err = 0;

	iscan->state = WL_ISCAN_STATE_SCANING;

	passive_scan = wl->active_scan ? 0 : 1;
	err = wldev_ioctl(ndev, WLC_SET_PASSIVE_SCAN,
		&passive_scan, sizeof(passive_scan), false);
	if (unlikely(err)) {
		WL_DBG(("error (%d)\n", err));
		return err;
	}
	wl->iscan_kickstart = true;
	wl_run_iscan(iscan, request, WL_SCAN_ACTION_START);
	mod_timer(&iscan->timer, jiffies + iscan->timer_ms * HZ / 1000);
	iscan->timer_on = 1;

	return err;
}

static s32
wl_get_valid_channels(struct net_device *ndev, u8 *valid_chan_list, s32 size)
{
	wl_uint32_list_t *list;
	s32 err = BCME_OK;
	if (valid_chan_list == NULL || size <= 0)
		return -ENOMEM;

	memset(valid_chan_list, 0, size);
	list = (wl_uint32_list_t *)(void *) valid_chan_list;
	list->count = htod32(WL_NUMCHANNELS);
	err = wldev_ioctl(ndev, WLC_GET_VALID_CHANNELS, valid_chan_list, size, false);
	if (err != 0) {
		WL_ERR(("get channels failed with %d\n", err));
	}

	return err;
}

#ifdef USE_INITIAL_2G_SCAN
#define FIRST_SCAN_ACTIVE_DWELL_TIME_MS 40
static bool g_first_broadcast_scan = TRUE;
#endif 

void wl_abort_scan(struct work_struct *work);
DECLARE_DELAYED_WORK(abort_scan, wl_abort_scan);
void wl_abort_scan(struct work_struct *work)
{
        struct wl_priv *wl= NULL;
        struct net_device *ndev;
        wl_scan_params_t *params = NULL;
        int params_size;
        int err;

        wl = wlcfg_drv_priv;

        if (!wl) {
                printf("%s: wl is null\n", __FUNCTION__);
        }

        ndev = wl_to_prmry_ndev(wl);

        if (!ndev) {
                printf("%s: ndev is null\n", __FUNCTION__);
        }

        
	params = wl_cfg80211_scan_alloc_params(-1, 0, &params_size);
        if (params) {
		
                err = wldev_ioctl(ndev, WLC_SCAN, params, params_size, true);
                if (err < 0)
			WL_ERR(("scan abort failed(%d)\n", err));
                else
			WL_ERR(("scan abort issued\n"));
        }
}

#ifdef APSTA_CONCURRENT
extern int scan_suppress_flag;
#define CHK_HTC_IE(ie) ((ie) && (ie)[0] == 0xdd && (ie)[2] == 0x00 && (ie)[3] == 0x09 && (ie)[4] == 0x2d)

void wl_cfg_stop_scansuppress(struct work_struct *work);

DECLARE_DELAYED_WORK(stop_scansuppress, wl_cfg_stop_scansuppress);
extern struct wl_priv *wlcfg_drv_priv;
extern int wldev_start_stop_scansuppress(struct net_device * dev);

void wl_cfg_stop_scansuppress(struct work_struct *work)
{
	struct wl_priv *wl= NULL;
	struct net_device *netdev;

	wl = wlcfg_drv_priv;
	
	if (!wl) {
		printf("%s: wl is null\n", __FUNCTION__);
	}
	
	netdev = wl->wdev->netdev;
	
	if (!netdev) {
		printf("%s: netdev is null\n", __FUNCTION__);
	}

		wldev_start_stop_scansuppress(netdev);
}
#endif

static s32
wl_run_escan(struct wl_priv *wl, struct net_device *ndev,
	struct cfg80211_scan_request *request, uint16 action)
{
	s32 err = BCME_OK;
	u32 n_channels;
	u32 n_ssids;
	s32 params_size = (WL_SCAN_PARAMS_FIXED_SIZE + OFFSETOF(wl_escan_params_t, params));
	wl_escan_params_t *params = NULL;
	u8 chan_buf[sizeof(u32)*(WL_NUMCHANNELS + 1)];
	u32 num_chans = 0;
	s32 channel;
	s32 n_valid_chan;
	s32 search_state = WL_P2P_DISC_ST_SCAN;
	u32 i, j, n_nodfs = 0;
	u16 *default_chan_list = NULL;
	wl_uint32_list_t *list;
	struct net_device *dev = NULL;
	WL_DBG(("Enter \n"));


	if (!request || !wl) {
		err = -EINVAL;
		goto exit;
	}
	if (!wl->p2p_supported || !p2p_scan(wl)) {
#if defined(CUSTOMER_HW4)
		s32 active_time = -1;
#endif
		
		WL_SCAN((" LEGACY E-SCAN START\n"));

		if (request != NULL) {
#ifdef USE_INITIAL_2G_SCAN
			if (ndev == wl_to_prmry_ndev(wl) && g_first_broadcast_scan == true) {
				j = 0;
				if (!wl_get_valid_channels(ndev, chan_buf, sizeof(chan_buf))) {
					list = (wl_uint32_list_t *) chan_buf;
					n_valid_chan = dtoh32(list->count);
					for (i = 0; i < n_valid_chan; i++) {
#if defined(BCM4334_CHIP)
						request->channels[i]->flags |=
							IEEE80211_CHAN_NO_HT40;
#endif

						WL_SCAN(("list->element[%d]=%d\n",
							i, list->element[i]));
						if (list->element[i] > CH_MAX_2G_CHANNEL)
							break;
						j++;
					}
					request->n_channels = j;

#if defined(CUSTOMER_HW4)
					active_time = FIRST_SCAN_ACTIVE_DWELL_TIME_MS;
#endif 
					WL_SCAN(("request->n_channels=%d\n", request->n_channels));
					g_first_broadcast_scan = false;
				}
			}

#endif 
			n_channels = request->n_channels;
			n_ssids = request->n_ssids;
			
			if (n_channels % 2)
				
				params_size += sizeof(u16) * (n_channels + 1);
			else
				params_size += sizeof(u16) * n_channels;

			
			params_size += sizeof(struct wlc_ssid) * n_ssids;
		}
		params = (wl_escan_params_t *) kzalloc(params_size, GFP_KERNEL);
		if (params == NULL) {
			err = -ENOMEM;
			goto exit;
		}

#if defined(CUSTOMER_HW4)
			wl_scan_prep(&params->params, request, active_time);
#else
			wl_scan_prep(&params->params, request);
#endif
		params->version = htod32(ESCAN_REQ_VERSION);
		params->action =  htod16(action);
#if defined(DUAL_ESCAN_RESULT_BUFFER)
		params->sync_id = wl->escan_info.cur_sync_id;
#else
		params->sync_id = htod16(0x1234);
#endif
		if (params_size + sizeof("escan") >= WLC_IOCTL_MEDLEN) {
			WL_ERR(("ioctl buffer length not sufficient\n"));
			kfree(params);
			err = -ENOMEM;
			goto exit;
		}
#ifndef APSTA_CONCURRENT
		err = wldev_iovar_setbuf(ndev, "escan", params, params_size,
			wl->escan_ioctl_buf, WLC_IOCTL_MEDLEN, NULL);
#else
		if(CHK_HTC_IE(request->ie))
			printf("%s CHK_HTC_IE return TRUE scan_suppress_flag[%d]\n",__FUNCTION__,scan_suppress_flag);
		else
			printf("%s CHK_HTC_IE return FALSE scan_suppress_flag[%d]\n",__FUNCTION__,scan_suppress_flag);

		if( (CHK_HTC_IE(request->ie) || !scan_suppress_flag || wl->apsta_concurrent)){
			int enable;
			if(scan_suppress_flag){
				int res;
				enable= 0;
            	res = wldev_ioctl(ndev, WLC_SET_SCANSUPPRESS, &enable, sizeof(enable), 1);
            	printf("%s res[%d]",__FUNCTION__,res);
			}
			
			if (wl_get_drv_status(wl, CONNECTING, ndev)) {
				printf("%s: connecting, return busy instead.\n", __FUNCTION__);
				err = BCME_BUSY;
			} else {
				err = wldev_iovar_setbuf(ndev, "escan", params, params_size,
					wl->escan_ioctl_buf, WLC_IOCTL_MEDLEN, NULL);
				
				if (err == BCME_NOTUP)
					err = 0;
			}
			schedule_delayed_work(&stop_scansuppress, (20*HZ/1000));
		}
#endif
		if (unlikely(err))
			WL_ERR((" Escan set error (%d)\n", err));
		kfree(params);
	}
	else if (p2p_is_on(wl) && p2p_scan(wl)) {
		
		s32 _freq = 0;
		n_nodfs = 0;
		if (request && request->n_channels) {
			num_chans = request->n_channels;
			WL_SCAN((" chann number : %d\n", num_chans));
			default_chan_list = kzalloc(num_chans * sizeof(*default_chan_list),
				GFP_KERNEL);
			if (default_chan_list == NULL) {
				WL_ERR(("channel list allocation failed \n"));
				err = -ENOMEM;
				goto exit;
			}
			if (!wl_get_valid_channels(ndev, chan_buf, sizeof(chan_buf))) {
				list = (wl_uint32_list_t *) chan_buf;
				n_valid_chan = dtoh32(list->count);
				for (i = 0; i < num_chans; i++)
				{
					_freq = request->channels[i]->center_freq;
					channel = ieee80211_frequency_to_channel(_freq);
					
					if (!(request->channels[i]->flags &
						(IEEE80211_CHAN_RADAR
						| IEEE80211_CHAN_PASSIVE_SCAN))) {
						for (j = 0; j < n_valid_chan; j++) {
							if (channel == (dtoh32(list->element[j])))
								default_chan_list[n_nodfs++] =
									channel;
						}
					}

				}
			}
			if (num_chans == 3 && (
						(default_chan_list[0] == SOCIAL_CHAN_1) &&
						(default_chan_list[1] == SOCIAL_CHAN_2) &&
						(default_chan_list[2] == SOCIAL_CHAN_3))) {
				
				search_state = WL_P2P_DISC_ST_SEARCH;
				WL_INFO(("P2P SEARCH PHASE START \n"));
			} else if ((dev = wl_to_p2p_bss_ndev(wl, P2PAPI_BSSCFG_CONNECTION)) &&
				(wl_get_mode_by_netdev(wl, dev) == WL_MODE_AP)) {
				
				WL_INFO(("Already a GO. Do SEARCH Only"));
				search_state = WL_P2P_DISC_ST_SEARCH;
				num_chans = n_nodfs;

			} else {
				WL_INFO(("P2P SCAN STATE START \n"));
				num_chans = n_nodfs;
			}

		}
		err = wl_cfgp2p_escan(wl, ndev, wl->active_scan, num_chans, default_chan_list,
			search_state, action,
			wl_to_p2p_bss_bssidx(wl, P2PAPI_BSSCFG_DEVICE));
		kfree(default_chan_list);
	}
exit:
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
	}
	return err;
}


static s32
wl_do_escan(struct wl_priv *wl, struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_scan_request *request)
{
	s32 err = BCME_OK;
	s32 passive_scan;
	wl_scan_results_t *results;
	WL_SCAN(("Enter \n"));
	mutex_lock(&wl->usr_sync);
#if defined(DUAL_ESCAN_RESULT_BUFFER)
	results = (wl_scan_results_t *) wl->escan_info.escan_buf[wl->escan_info.cur_sync_id%2];
#else
	results = (wl_scan_results_t *) wl->escan_info.escan_buf;
#endif
	results->version = 0;
	results->count = 0;
	results->buflen = WL_SCAN_RESULTS_FIXED_SIZE;
	wl->escan_info.ndev = ndev;
	wl->escan_info.wiphy = wiphy;
	wl->escan_info.escan_state = WL_ESCAN_STATE_SCANING;
	passive_scan = wl->active_scan ? 0 : 1;
	err = wldev_ioctl(ndev, WLC_SET_PASSIVE_SCAN,
		&passive_scan, sizeof(passive_scan), false);
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
		goto exit;
	}

	err = wl_run_escan(wl, ndev, request, WL_SCAN_ACTION_START);
exit:
	mutex_unlock(&wl->usr_sync);
	return err;
}

static s32
__wl_cfg80211_scan(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_scan_request *request,
	struct cfg80211_ssid *this_ssid)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct cfg80211_ssid *ssids;
	struct wl_scan_req *sr = wl_to_sr(wl);
	struct ether_addr primary_mac;
	s32 passive_scan;
	bool iscan_req;
	bool escan_req = false;
	bool p2p_ssid;
	s32 err = 0;
	s32 bssidx = -1;
	s32 i;
	unsigned long flags;
	static s32 busy_count = 0;
	static s32 disconnect_flag = 0;

	if (ndev == wl->p2p_net) {
		ndev = wl_to_prmry_ndev(wl);
	}

	if (WL_DRV_STATUS_SENDING_AF_FRM_EXT(wl)) {
		WL_ERR(("Sending Action Frames. Try it again.\n"));
		return -EAGAIN;
	}
	WL_DBG(("Enter wiphy (%p)\n", wiphy));
	if (wl_get_drv_status_all(wl, SCANNING)) {
		if (wl->scan_request == NULL) {
			wl_clr_drv_status_all(wl, SCANNING);
			WL_DBG(("<<<<<<<<<<<Force Clear Scanning Status>>>>>>>>>>>\n"));
		} else {
			WL_ERR(("Scanning already\n"));
			return -EAGAIN;
		}
	}
	if (wl_get_drv_status(wl, SCAN_ABORTING, ndev)) {
		WL_ERR(("Scanning being aborted\n"));
		return -EAGAIN;
	}
	if (is_scan_request_valid(request) && request->n_ssids > WL_SCAN_PARAMS_SSID_MAX) {
		WL_ERR(("request null or n_ssids > WL_SCAN_PARAMS_SSID_MAX\n"));
		return -EOPNOTSUPP;
	}
#ifdef WL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
	if (wl_get_drv_status_all(wl, REMAINING_ON_CHANNEL)) {
		
		wl_notify_escan_complete(wl, ndev, true, true);
	}
#endif 

	
	mod_timer(&wl->scan_timeout, jiffies + WL_SCAN_TIMER_INTERVAL_MS * HZ / 1000);
	iscan_req = false;
	
	if (is_scan_request_valid(request)) {		
		ssids = request->ssids;
		if (wl->iscan_on && (!ssids || !ssids->ssid_len || request->n_ssids != 1)) {
			iscan_req = true;
		} else if (wl->escan_on) {
			escan_req = true;
			p2p_ssid = false;
			for (i = 0; i < request->n_ssids; i++) {
				if (ssids[i].ssid_len &&
					IS_P2P_SSID(ssids[i].ssid, ssids[i].ssid_len)) {
					p2p_ssid = true;
					break;
				}
			}
			if (p2p_ssid) {
				if (wl->p2p_supported) {
					
					wldev_iovar_setint(ndev, "mpc", 0);
					if (p2p_on(wl) == false) {
						
						p2p_on(wl) = true;
						wl_cfgp2p_set_firm_p2p(wl);
						get_primary_mac(wl, &primary_mac);
						wl_cfgp2p_generate_bss_mac(&primary_mac,
							&wl->p2p->dev_addr, &wl->p2p->int_addr);
					}
					wl_clr_p2p_status(wl, GO_NEG_PHASE);
					WL_DBG(("P2P: GO_NEG_PHASE status cleared \n"));
					p2p_scan(wl) = true;
				}
			} else {
				if (wl->p2p_supported) {
					p2p_scan(wl) = false;

					if (p2p_scan(wl) == false) {
						if (wl_get_p2p_status(wl, DISCOVERY_ON)) {
							err = wl_cfgp2p_discover_enable_search(wl,
							false);
							if (unlikely(err)) {
								goto scan_out;
							}

						}
					}
				}
				if (!wl->p2p_supported || !p2p_scan(wl)) {
					if ((wl_cfgp2p_find_p2pie((u8 *)request->ie,
						request->ie_len)) &&
						(ndev != wl_to_prmry_ndev(wl))) {
						bssidx = wl_cfgp2p_find_idx(wl, ndev);
					
					}else if( request->ie && request->ie[0] == 0xdd &&
					  request->ie[2] == 0x00 && request->ie[3] == 0x09
					  && request->ie[4] == 0x2d ) {
					  printf("Our specific IE found size:%d\n", request->ie_len);
					}
					
					err = wl_cfgp2p_set_management_ie(wl, ndev, bssidx,
						VNDR_IE_PRBREQ_FLAG, (u8 *)request->ie,
						request->ie_len);

					if (unlikely(err)) {
						goto scan_out;
					}
				}
			}
		}
	} else {		
		
		ssids = this_ssid;
	}
	wl->scan_request = request;
	wl_set_drv_status(wl, SCANNING, ndev);
	if (iscan_req) {
		err = wl_do_iscan(wl, request);
		if (likely(!err))
			goto scan_success;
		else
			goto scan_out;
	} else if (escan_req) {
		if (wl->p2p_supported) {
			if (p2p_on(wl) && p2p_scan(wl)) {
#ifdef HTC_KlocWork
				if (is_scan_request_valid(request)) {
#endif
					
					if(request->ie != NULL){
						
						wl->afx_hdl->my_listen_chan =
							wl_find_listen_channel(wl, (u8 *)request->ie,
							request->ie_len);
						err = wl_cfgp2p_enable_discovery(wl, ndev,
							request->ie, request->ie_len);
					}
					
					if (unlikely(err)) {
						goto scan_out;
					}
#ifdef HTC_KlocWork
				}
#endif
			}
		}

		err = wl_do_escan(wl, wiphy, ndev, request);
		if (likely(!err))
			goto scan_success;
		else
			goto scan_out;


	} else {
		memset(&sr->ssid, 0, sizeof(sr->ssid));
		sr->ssid.SSID_len =
			min_t(u8, sizeof(sr->ssid.SSID), ssids->ssid_len);
		if (sr->ssid.SSID_len) {
			memcpy(sr->ssid.SSID, ssids->ssid, sr->ssid.SSID_len);
			sr->ssid.SSID_len = htod32(sr->ssid.SSID_len);
			WL_SCAN(("Specific scan ssid=\"%s\" len=%d\n",
				sr->ssid.SSID, sr->ssid.SSID_len));
		} else {
			WL_SCAN(("Broadcast scan\n"));
		}
		WL_SCAN(("sr->ssid.SSID_len (%d)\n", sr->ssid.SSID_len));
		passive_scan = wl->active_scan ? 0 : 1;
		err = wldev_ioctl(ndev, WLC_SET_PASSIVE_SCAN,
			&passive_scan, sizeof(passive_scan), false);
		if (unlikely(err)) {
			WL_SCAN(("WLC_SET_PASSIVE_SCAN error (%d)\n", err));
			goto scan_out;
		}
		err = wldev_ioctl(ndev, WLC_SCAN, &sr->ssid,
			sizeof(sr->ssid), false);
		if (err) {
			if (err == -EBUSY) {
				WL_ERR(("system busy : scan for \"%s\" "
					"canceled\n", sr->ssid.SSID));
			} else {
				WL_ERR(("WLC_SCAN error (%d)\n", err));
			}
			goto scan_out;
		}
	}

scan_success:

	busy_count = 0;
	disconnect_flag = 0;
	return 0;

scan_out:
	if (err == BCME_BUSY || err == BCME_NOTREADY) {
		WL_ERR(("Scan err = (%d), busy?%d", err, -EBUSY));
		err = -EBUSY;
	}

#define SCAN_EBUSY_RETRY_LIMIT 10
	if (err == -EBUSY) {
		if (busy_count++ > SCAN_EBUSY_RETRY_LIMIT) {
			struct ether_addr bssid;
			s32 ret = 0;
			busy_count = 0;
			WL_ERR(("Unusual continuous EBUSY error, %d %d %d %d %d %d %d %d %d\n",
				wl_get_drv_status(wl, SCANNING, ndev),
				wl_get_drv_status(wl, SCAN_ABORTING, ndev),
				wl_get_drv_status(wl, CONNECTING, ndev),
				wl_get_drv_status(wl, CONNECTED, ndev),
				wl_get_drv_status(wl, DISCONNECTING, ndev),
				wl_get_drv_status(wl, AP_CREATING, ndev),
				wl_get_drv_status(wl, AP_CREATED, ndev),
				wl_get_drv_status(wl, SENDING_ACT_FRM, ndev),
				wl_get_drv_status(wl, SENDING_ACT_FRM, ndev)));

			bzero(&bssid, sizeof(bssid));
			if ((ret = wldev_ioctl(ndev, WLC_GET_BSSID,
				&bssid, ETHER_ADDR_LEN, false)) == 0)
				WL_ERR(("FW is connected with " MACSTR "/n",
				MAC2STR(bssid.octet)));
			else
				WL_ERR(("GET BSSID failed with %d\n", ret));

			if (!disconnect_flag) {
				if (wl_get_drv_status(wl, CONNECTING, ndev)) {
					int val = 0;
					int err;

					
					printf("%s: abort the connecting procedure.\n", __FUNCTION__);
                        		err = wldev_ioctl(ndev, WLC_DISASSOC, &val, sizeof(val), true);
                        		if (unlikely(err)) {
                                		WL_ERR(("call WLC_DISASSOC failed. (%d)\n", err));
                        		}

					
					wl_bss_connect_done(wl, ndev, NULL, NULL, false);
                } else { 
					wl_cfg80211_disconnect(wiphy, ndev, DOT11_RC_DISASSOC_LEAVING);
				}
				disconnect_flag = 1;
			} else {
				net_os_send_hang_message(ndev);
				disconnect_flag = 0;
			}
		}
	} else {
		busy_count = 0;
		disconnect_flag = 0;
	}
	wl_clr_drv_status(wl, SCANNING, ndev);
	if (timer_pending(&wl->scan_timeout))
		del_timer_sync(&wl->scan_timeout);
	spin_lock_irqsave(&wl->cfgdrv_lock, flags);
	wl->scan_request = NULL;
	spin_unlock_irqrestore(&wl->cfgdrv_lock, flags);
	return err;
}

static s32
wl_cfg80211_scan(struct wiphy *wiphy, struct net_device *ndev,
	struct cfg80211_scan_request *request)
{
	s32 err = 0;
	struct wl_priv *wl = wiphy_priv(wiphy);

	WL_DBG(("Enter \n"));
	CHECK_SYS_UP(wl);

	err = __wl_cfg80211_scan(wiphy, ndev, request, NULL);
	if (unlikely(err)) {
		WL_ERR(("scan error (%d)\n", err));
		return err;
	}

	return err;
}

static s32 wl_set_rts(struct net_device *dev, u32 rts_threshold)
{
	s32 err = 0;

	err = wldev_iovar_setint(dev, "rtsthresh", rts_threshold);
	if (unlikely(err)) {
		WL_ERR(("Error (%d)\n", err));
		return err;
	}
	return err;
}

static s32 wl_set_frag(struct net_device *dev, u32 frag_threshold)
{
	s32 err = 0;

	err = wldev_iovar_setint_bsscfg(dev, "fragthresh", frag_threshold, 0);
	if (unlikely(err)) {
		WL_ERR(("Error (%d)\n", err));
		return err;
	}
	return err;
}

static s32 wl_set_retry(struct net_device *dev, u32 retry, bool l)
{
	s32 err = 0;
	u32 cmd = (l ? WLC_SET_LRL : WLC_SET_SRL);

	retry = htod32(retry);
	err = wldev_ioctl(dev, cmd, &retry, sizeof(retry), false);
	if (unlikely(err)) {
		WL_ERR(("cmd (%d) , error (%d)\n", cmd, err));
		return err;
	}
	return err;
}

static s32 wl_cfg80211_set_wiphy_params(struct wiphy *wiphy, u32 changed)
{
	struct wl_priv *wl = (struct wl_priv *)wiphy_priv(wiphy);
	struct net_device *ndev = wl_to_prmry_ndev(wl);
	s32 err = 0;

	CHECK_SYS_UP(wl);
	WL_DBG(("Enter\n"));
	if (changed & WIPHY_PARAM_RTS_THRESHOLD &&
		(wl->conf->rts_threshold != wiphy->rts_threshold)) {
		wl->conf->rts_threshold = wiphy->rts_threshold;
		err = wl_set_rts(ndev, wl->conf->rts_threshold);
		if (!err)
			return err;
	}
	if (changed & WIPHY_PARAM_FRAG_THRESHOLD &&
		(wl->conf->frag_threshold != wiphy->frag_threshold)) {
		wl->conf->frag_threshold = wiphy->frag_threshold;
		err = wl_set_frag(ndev, wl->conf->frag_threshold);
		if (!err)
			return err;
	}
	if (changed & WIPHY_PARAM_RETRY_LONG &&
		(wl->conf->retry_long != wiphy->retry_long)) {
		wl->conf->retry_long = wiphy->retry_long;
		err = wl_set_retry(ndev, wl->conf->retry_long, true);
		if (!err)
			return err;
	}
	if (changed & WIPHY_PARAM_RETRY_SHORT &&
		(wl->conf->retry_short != wiphy->retry_short)) {
		wl->conf->retry_short = wiphy->retry_short;
		err = wl_set_retry(ndev, wl->conf->retry_short, false);
		if (!err) {
			return err;
		}
	}

	return err;
}

static s32
wl_cfg80211_join_ibss(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_ibss_params *params)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct cfg80211_bss *bss;
	struct ieee80211_channel *chan;
	struct wl_join_params join_params;
	struct cfg80211_ssid ssid;
	s32 scan_retry = 0;
	s32 err = 0;
	bool rollback_lock = false;

	WL_TRACE(("In\n"));
	CHECK_SYS_UP(wl);
	if (params->bssid) {
		WL_ERR(("Invalid bssid\n"));
		return -EOPNOTSUPP;
	}
	bss = cfg80211_get_ibss(wiphy, NULL, params->ssid, params->ssid_len);
	if (!bss) {
		memcpy(ssid.ssid, params->ssid, params->ssid_len);
		ssid.ssid_len = params->ssid_len;
		do {
			if (unlikely
				(__wl_cfg80211_scan(wiphy, dev, NULL, &ssid) ==
				 -EBUSY)) {
				wl_delay(150);
			} else {
				break;
			}
		} while (++scan_retry < WL_SCAN_RETRY_MAX);
		
		if (rtnl_is_locked()) {
			rtnl_unlock();
			rollback_lock = true;
		}

		
		schedule_timeout_interruptible(4 * HZ);
		if (rollback_lock)
			rtnl_lock();
		bss = cfg80211_get_ibss(wiphy, NULL,
			params->ssid, params->ssid_len);
	}
	if (bss) {
		wl->ibss_starter = false;
		WL_DBG(("Found IBSS\n"));
	} else {
		wl->ibss_starter = true;
	}
	chan = params->channel;
	if (chan)
		wl->channel = ieee80211_frequency_to_channel(chan->center_freq);
	memset(&join_params, 0, sizeof(join_params));
	memcpy((void *)join_params.ssid.SSID, (void *)params->ssid,
		params->ssid_len);
	join_params.ssid.SSID_len = htod32(params->ssid_len);
	if (params->bssid)
		memcpy(&join_params.params.bssid, params->bssid,
			ETHER_ADDR_LEN);
	else
		memset(&join_params.params.bssid, 0, ETHER_ADDR_LEN);

	err = wldev_ioctl(dev, WLC_SET_SSID, &join_params,
		sizeof(join_params), false);
	if (unlikely(err)) {
		WL_ERR(("Error (%d)\n", err));
		return err;
	}
	return err;
}

static s32 wl_cfg80211_leave_ibss(struct wiphy *wiphy, struct net_device *dev)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	s32 err = 0;

	CHECK_SYS_UP(wl);
	wl_link_down(wl);

	return err;
}

static s32
wl_set_wpa_version(struct net_device *dev, struct cfg80211_connect_params *sme)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct wl_security *sec;
	s32 val = 0;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	if (sme->crypto.wpa_versions & NL80211_WPA_VERSION_1)
		val = WPA_AUTH_PSK |
#ifdef BCMCCX
		WPA_AUTH_CCKM |
#endif
		 WPA_AUTH_UNSPECIFIED;
	else if (sme->crypto.wpa_versions & NL80211_WPA_VERSION_2)
		val = WPA2_AUTH_PSK |
#ifdef BCMCCX
		WPA2_AUTH_CCKM |
#endif
		WPA2_AUTH_UNSPECIFIED;
	else
		val = WPA_AUTH_DISABLED;

	if (is_wps_conn(sme))
		val = WPA_AUTH_DISABLED;

#ifdef BCMWAPI_WPI
	if (sme->crypto.wpa_versions & NL80211_WAPI_VERSION_1) {
		WL_DBG((" * wl_set_wpa_version, set wpa_auth"
			" to WPA_AUTH_WAPI 0x400"));
		val = WAPI_AUTH_PSK | WAPI_AUTH_UNSPECIFIED;
	}
#endif
	WL_DBG(("setting wpa_auth to 0x%0x\n", val));
	err = wldev_iovar_setint_bsscfg(dev, "wpa_auth", val, bssidx);
	if (unlikely(err)) {
		WL_ERR(("set wpa_auth failed (%d)\n", err));
		return err;
	}
	sec = wl_read_prof(wl, dev, WL_PROF_SEC);
	sec->wpa_versions = sme->crypto.wpa_versions;
	return err;
}

#ifdef BCMWAPI_WPI
static s32
wl_set_set_wapi_ie(struct net_device *dev, struct cfg80211_connect_params *sme)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	WL_DBG((" %s \n", __FUNCTION__));

	if (sme->crypto.wpa_versions & NL80211_WAPI_VERSION_1) {
		err = wldev_iovar_setbuf_bsscfg(dev, "wapiie", sme->ie,
			sme->ie_len, wl->ioctl_buf, WLC_IOCTL_MAXLEN, bssidx, &wl->ioctl_buf_sync);

		if (unlikely(err)) {
			WL_ERR(("===> set_wapi_ie Error (%d)\n", err));
			return err;
		}
	} else
		WL_DBG((" * skip \n"));
	return err;
}
#endif 

static s32
wl_set_auth_type(struct net_device *dev, struct cfg80211_connect_params *sme)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct wl_security *sec;
	s32 val = 0;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);
	switch (sme->auth_type) {
	case NL80211_AUTHTYPE_OPEN_SYSTEM:
		val = WL_AUTH_OPEN_SYSTEM;
		WL_DBG(("open system\n"));
		break;
	case NL80211_AUTHTYPE_SHARED_KEY:
		val = WL_AUTH_SHARED_KEY;
		WL_DBG(("shared key\n"));
		break;
	case NL80211_AUTHTYPE_AUTOMATIC:
		val = WL_AUTH_OPEN_SHARED;
		WL_DBG(("automatic\n"));
		break;
	case NL80211_AUTHTYPE_NETWORK_EAP:
		WL_DBG(("network eap\n"));
#ifdef BCMCCX
		val = DOT11_LEAP_AUTH;
		break;
#endif
	default:
		val = WL_AUTH_OPEN_SHARED;
		WL_ERR(("invalid auth type (%d)\n", sme->auth_type));
		break;
	}

	err = wldev_iovar_setint_bsscfg(dev, "auth", val, bssidx);
	if (unlikely(err)) {
		WL_ERR(("set auth failed (%d)\n", err));
		return err;
	}
	sec = wl_read_prof(wl, dev, WL_PROF_SEC);
	sec->auth_type = sme->auth_type;
	return err;
}

static s32
wl_set_set_cipher(struct net_device *dev, struct cfg80211_connect_params *sme)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct wl_security *sec;
	s32 pval = 0;
	s32 gval = 0;
	s32 err = 0;
#ifdef BCMWAPI_WPI
	s32 val = 0;
#endif
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	if (sme->crypto.n_ciphers_pairwise) {
		switch (sme->crypto.ciphers_pairwise[0]) {
		case WLAN_CIPHER_SUITE_WEP40:
		case WLAN_CIPHER_SUITE_WEP104:
			pval = WEP_ENABLED;
			break;
		case WLAN_CIPHER_SUITE_TKIP:
			pval = TKIP_ENABLED;
			break;
		case WLAN_CIPHER_SUITE_CCMP:
			pval = AES_ENABLED;
			break;
		case WLAN_CIPHER_SUITE_AES_CMAC:
			pval = AES_ENABLED;
			break;
#ifdef BCMWAPI_WPI
		case WLAN_CIPHER_SUITE_SMS4:
			val = SMS4_ENABLED;
			pval = SMS4_ENABLED;
			break;
#endif
		default:
			WL_ERR(("invalid cipher pairwise (%d)\n",
				sme->crypto.ciphers_pairwise[0]));
			return -EINVAL;
		}
	}
	if (sme->crypto.cipher_group) {
		switch (sme->crypto.cipher_group) {
		case WLAN_CIPHER_SUITE_WEP40:
		case WLAN_CIPHER_SUITE_WEP104:
			gval = WEP_ENABLED;
			break;
		case WLAN_CIPHER_SUITE_TKIP:
			gval = TKIP_ENABLED;
			break;
		case WLAN_CIPHER_SUITE_CCMP:
			gval = AES_ENABLED;
			break;
		case WLAN_CIPHER_SUITE_AES_CMAC:
			gval = AES_ENABLED;
			break;
#ifdef BCMWAPI_WPI
		case WLAN_CIPHER_SUITE_SMS4:
			val = SMS4_ENABLED;
			gval = SMS4_ENABLED;
			break;
#endif
		default:
			WL_ERR(("invalid cipher group (%d)\n",
				sme->crypto.cipher_group));
			return -EINVAL;
		}
	}

	WL_DBG(("pval (%d) gval (%d)\n", pval, gval));

	if (is_wps_conn(sme)) {
		if (sme->privacy)
			err = wldev_iovar_setint_bsscfg(dev, "wsec", 4, bssidx);
		else
			
			err = wldev_iovar_setint_bsscfg(dev, "wsec", 0, bssidx);
	} else {
#ifdef BCMWAPI_WPI
		if (sme->crypto.cipher_group == WLAN_CIPHER_SUITE_SMS4) {
			WL_DBG((" NO, is_wps_conn, WAPI set to SMS4_ENABLED"));
			err = wldev_iovar_setint_bsscfg(dev, "wsec", val, bssidx);
		} else {
#endif
			WL_DBG((" NO, is_wps_conn, Set pval | gval to WSEC"));
			err = wldev_iovar_setint_bsscfg(dev, "wsec",
				pval | gval, bssidx);
#ifdef BCMWAPI_WPI
		}
#endif
	}
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
		return err;
	}

	sec = wl_read_prof(wl, dev, WL_PROF_SEC);
	sec->cipher_pairwise = sme->crypto.ciphers_pairwise[0];
	sec->cipher_group = sme->crypto.cipher_group;

	return err;
}

static s32
wl_set_key_mgmt(struct net_device *dev, struct cfg80211_connect_params *sme)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct wl_security *sec;
	s32 val = 0;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	if (sme->crypto.n_akm_suites) {
		err = wldev_iovar_getint(dev, "wpa_auth", &val);
		if (unlikely(err)) {
			WL_ERR(("could not get wpa_auth (%d)\n", err));
			return err;
		}
		if (val & (WPA_AUTH_PSK |
#ifdef BCMCCX
			WPA_AUTH_CCKM |
#endif
		 WPA_AUTH_UNSPECIFIED)) {
			switch (sme->crypto.akm_suites[0]) {
			case WLAN_AKM_SUITE_8021X:
				val = WPA_AUTH_UNSPECIFIED;
				break;
			case WLAN_AKM_SUITE_PSK:
				val = WPA_AUTH_PSK;
				break;
#ifdef BCMCCX
			case WLAN_AKM_SUITE_CCKM:
				val = WPA_AUTH_CCKM;
				break;
#endif
			default:
				WL_ERR(("invalid cipher group (%d)\n",
					sme->crypto.cipher_group));
				return -EINVAL;
			}
		} else if (val & (WPA2_AUTH_PSK | 
#ifdef BCMCCX
				WPA2_AUTH_CCKM |
#endif
			   WPA2_AUTH_UNSPECIFIED)) {
			switch (sme->crypto.akm_suites[0]) {
			case WLAN_AKM_SUITE_8021X:
				val = WPA2_AUTH_UNSPECIFIED;
				break;
			case WLAN_AKM_SUITE_PSK:
				val = WPA2_AUTH_PSK;
				break;
#ifdef BCMCCX
			case WLAN_AKM_SUITE_CCKM:
				val = WPA2_AUTH_CCKM;
				break;
#endif
			default:
				WL_ERR(("invalid cipher group (%d)\n",
					sme->crypto.cipher_group));
				return -EINVAL;
			}
		}
#ifdef BCMWAPI_WPI
		else if (val & (WAPI_AUTH_PSK | WAPI_AUTH_UNSPECIFIED)) {
			switch (sme->crypto.akm_suites[0]) {
			case WLAN_AKM_SUITE_WAPI_CERT:
				val = WAPI_AUTH_UNSPECIFIED;
				break;
			case WLAN_AKM_SUITE_WAPI_PSK:
				val = WAPI_AUTH_PSK;
				break;
			default:
				WL_ERR(("invalid cipher group (%d)\n",
					sme->crypto.cipher_group));
				return -EINVAL;
			}
		}
#endif
		WL_DBG(("setting wpa_auth to %d\n", val));

		err = wldev_iovar_setint_bsscfg(dev, "wpa_auth", val, bssidx);
		if (unlikely(err)) {
			WL_ERR(("could not set wpa_auth (%d)\n", err));
			return err;
		}
	}
	sec = wl_read_prof(wl, dev, WL_PROF_SEC);
	sec->wpa_auth = sme->crypto.akm_suites[0];

	return err;
}

static s32
wl_set_set_sharedkey(struct net_device *dev,
	struct cfg80211_connect_params *sme)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct wl_security *sec;
	struct wl_wsec_key key;
	s32 val;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	WL_DBG(("key len (%d)\n", sme->key_len));
	if (sme->key_len) {
		sec = wl_read_prof(wl, dev, WL_PROF_SEC);
		WL_DBG(("wpa_versions 0x%x cipher_pairwise 0x%x\n",
			sec->wpa_versions, sec->cipher_pairwise));
		if (!(sec->wpa_versions & (NL80211_WPA_VERSION_1 |
#ifdef BCMWAPI_WPI
			NL80211_WPA_VERSION_2 | NL80211_WAPI_VERSION_1)) &&
#else
			NL80211_WPA_VERSION_2)) &&
#endif
			(sec->cipher_pairwise & (WLAN_CIPHER_SUITE_WEP40 |
#ifdef BCMWAPI_WPI
		WLAN_CIPHER_SUITE_WEP104 | WLAN_CIPHER_SUITE_SMS4)))
#else
		WLAN_CIPHER_SUITE_WEP104)))
#endif
		{
			memset(&key, 0, sizeof(key));
			key.len = (u32) sme->key_len;
			key.index = (u32) sme->key_idx;
			if (unlikely(key.len > sizeof(key.data))) {
				WL_ERR(("Too long key length (%u)\n", key.len));
				return -EINVAL;
			}
			memcpy(key.data, sme->key, key.len);
			key.flags = WL_PRIMARY_KEY;
			switch (sec->cipher_pairwise) {
			case WLAN_CIPHER_SUITE_WEP40:
				key.algo = CRYPTO_ALGO_WEP1;
				break;
			case WLAN_CIPHER_SUITE_WEP104:
				key.algo = CRYPTO_ALGO_WEP128;
				break;
#ifdef BCMWAPI_WPI
			case WLAN_CIPHER_SUITE_SMS4:
				WL_DBG((" * wl_set_set_sharedkey, set shared key to CRYPTO_ALGO_SMS4"));
				key.algo = CRYPTO_ALGO_SMS4;
				break;
#endif
			default:
				WL_ERR(("Invalid algorithm (%d)\n",
					sme->crypto.ciphers_pairwise[0]));
				return -EINVAL;
			}
			
			WL_DBG(("key length (%d) key index (%d) algo (%d)\n",
				key.len, key.index, key.algo));
			WL_DBG(("key \"%s\"\n", key.data));
			swap_key_from_BE(&key);
			err = wldev_iovar_setbuf_bsscfg(dev, "wsec_key", &key, sizeof(key),
				wl->ioctl_buf, WLC_IOCTL_MAXLEN, bssidx, &wl->ioctl_buf_sync);
			if (unlikely(err)) {
				WL_ERR(("WLC_SET_KEY error (%d)\n", err));
				return err;
			}
			if (sec->auth_type == NL80211_AUTHTYPE_SHARED_KEY) {
				WL_DBG(("set auth_type to shared key\n"));
				val = WL_AUTH_SHARED_KEY;	
				err = wldev_iovar_setint_bsscfg(dev, "auth", val, bssidx);
				if (unlikely(err)) {
					WL_ERR(("set auth failed (%d)\n", err));
					return err;
				}
			}
		}
	}
	return err;
}
#ifdef ESCAN_RESULT_PATCH
static u8 connect_req_bssid[6];
static u8 broad_bssid[6];
#endif

#ifdef ROAM_CHANNEL_CACHE
#define MAX_ROAM_CACHE_NUM 100
#endif

static s32
wl_cfg80211_connect(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_connect_params *sme)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct ieee80211_channel *chan = sme->channel;
	wl_extjoin_params_t *ext_join_params;
	struct wl_join_params join_params;
	size_t join_params_size;
	s32 err = 0;
	wpa_ie_fixed_t *wpa_ie;
	bcm_tlv_t *wpa2_ie;
	u8* wpaie  = 0;
	u32 wpaie_len = 0;
	u32 chan_cnt = 0;
	struct ether_addr bssid;
#ifdef ROAM_CHANNEL_CACHE
	chanspec_t chanspec_list[MAX_ROAM_CACHE_NUM];
#endif
	int ret;
	WL_DBG(("In\n"));
	if (unlikely(!sme->ssid)) {
		WL_ERR(("Invalid ssid\n"));
		return -EOPNOTSUPP;
	}
	CHECK_SYS_UP(wl);

#ifdef PNO_SUPPORT
	if(dev == wl_to_prmry_ndev(wl)){
		printf("%s sme->ssid[%s],sme->ssid_len[%d]\n", __FUNCTION__, sme->ssid,sme->ssid_len);
		dhd_set_pfn_ssid(sme->ssid, sme->ssid_len);
	}
#endif

#if (defined(BCM4334_CHIP) || !defined(ESCAN_RESULT_PATCH))
	
	if (is_scan_request_valid(wl->scan_request)) {
		wl_notify_escan_complete(wl, dev, true, true);
	}
	else WL_ERR(("is_scan_request_valid: 0 (1)\n"));
#endif
#ifdef WL_CFG80211_GON_COLLISION
	
	wl->block_gon_req_tx_count = 0;
	wl->block_gon_req_rx_count = 0;
#endif 
#ifdef ESCAN_RESULT_PATCH
	if (sme->bssid) {
		memcpy(connect_req_bssid, sme->bssid, ETHER_ADDR_LEN);
	}
	else {
		bzero(connect_req_bssid, ETHER_ADDR_LEN);
	}
	bzero(broad_bssid, ETHER_ADDR_LEN);
#endif

	bzero(&bssid, sizeof(bssid));
	if (!wl_get_drv_status(wl, CONNECTED, dev)&&
		(ret = wldev_ioctl(dev, WLC_GET_BSSID, &bssid, ETHER_ADDR_LEN, false)) == 0) {
		if (!ETHER_ISNULLADDR(&bssid)) {
			scb_val_t scbval;
			wl_set_drv_status(wl, DISCONNECTING, dev);
			scbval.val = DOT11_RC_DISASSOC_LEAVING;
			memcpy(&scbval.ea, &bssid, ETHER_ADDR_LEN);
			scbval.val = htod32(scbval.val);

			WL_DBG(("drv status CONNECTED is not set, but connected in FW!" MACSTR "/n",
				MAC2STR(bssid.octet)));
			err = wldev_ioctl(dev, WLC_DISASSOC, &scbval,
				sizeof(scb_val_t), true);
			if (unlikely(err)) {
				wl_clr_drv_status(wl, DISCONNECTING, dev);
				WL_ERR(("error (%d)\n", err));
				return err;
			}
			while (wl_get_drv_status(wl, DISCONNECTING, dev)) {
				WL_ERR(("Waiting for disconnection terminated.\n"));
				msleep(20);
			}
		} else
			WL_DBG(("Currently not associated!\n"));
	}
	
	bzero(&bssid, sizeof(bssid));
	if (!wl_get_drv_status(wl, DISCONNECTING, dev))
		wl_update_prof(wl, dev, NULL, (void *)&bssid, WL_PROF_BSSID);

	if (!memcmp(sme->ssid, WL_P2P_WILDCARD_SSID, WL_P2P_WILDCARD_SSID_LEN) &&
		(dev != wl_to_prmry_ndev(wl))) {
		
		if (p2p_is_on(wl) && is_wps_conn(sme)) {
			WL_DBG(("ASSOC1 p2p index : %d sme->ie_len %d\n",
				wl_cfgp2p_find_idx(wl, dev), sme->ie_len));
			
			wl_cfgp2p_set_management_ie(wl, dev,
				wl_cfgp2p_find_idx(wl, dev), VNDR_IE_PRBREQ_FLAG,
				wl_to_p2p_bss_saved_ie(wl, P2PAPI_BSSCFG_DEVICE).p2p_probe_req_ie,
				wl_to_p2p_bss_saved_ie(wl,
				P2PAPI_BSSCFG_DEVICE).p2p_probe_req_ie_len);
			wl_cfgp2p_set_management_ie(wl, dev, wl_cfgp2p_find_idx(wl, dev),
				VNDR_IE_ASSOCREQ_FLAG, sme->ie, sme->ie_len);
		} else if (p2p_is_on(wl) && (sme->crypto.wpa_versions & NL80211_WPA_VERSION_2)) {
			WL_DBG(("ASSOC2 p2p index : %d sme->ie_len %d\n",
				wl_cfgp2p_find_idx(wl, dev), sme->ie_len));
			wl_cfgp2p_set_management_ie(wl, dev,
				wl_cfgp2p_find_idx(wl, dev), VNDR_IE_PRBREQ_FLAG,
				sme->ie, sme->ie_len);
			wl_cfgp2p_set_management_ie(wl, dev, wl_cfgp2p_find_idx(wl, dev),
				VNDR_IE_ASSOCREQ_FLAG, sme->ie, sme->ie_len);
		}

	} else if (dev == wl_to_prmry_ndev(wl)) {
		
		if ((wpa2_ie = bcm_parse_tlvs((u8 *)sme->ie, sme->ie_len,
			DOT11_MNG_RSN_ID)) != NULL) {
			WL_DBG((" WPA2 IE is found\n"));
		}
		
		if ((wpa_ie = wl_cfgp2p_find_wpaie((u8 *)sme->ie,
			sme->ie_len)) != NULL) {
			WL_DBG((" WPA IE is found\n"));
		}
		if (wpa_ie != NULL || wpa2_ie != NULL) {
			wpaie = (wpa_ie != NULL) ? (u8 *)wpa_ie : (u8 *)wpa2_ie;
			wpaie_len = (wpa_ie != NULL) ? wpa_ie->length : wpa2_ie->len;
			wpaie_len += WPA_RSN_IE_TAG_FIXED_LEN;
			wldev_iovar_setbuf(dev, "wpaie", wpaie, wpaie_len,
				wl->ioctl_buf, WLC_IOCTL_MAXLEN, &wl->ioctl_buf_sync);
		} else {
			wldev_iovar_setbuf(dev, "wpaie", NULL, 0,
				wl->ioctl_buf, WLC_IOCTL_MAXLEN, &wl->ioctl_buf_sync);
		}

		err = wl_cfgp2p_set_management_ie(wl, dev, -1,
			VNDR_IE_ASSOCREQ_FLAG, (u8 *)sme->ie, sme->ie_len);
		if (unlikely(err)) {
			return err;
		}
	}
	if (chan) {
#ifdef ROAM_CHANNEL_CACHE
		wlc_ssid_t ssid;
		int band;

		err = wldev_get_band(dev, &band);
		if (!err) {
			set_roam_band(band);
		}

		wl->channel = ieee80211_frequency_to_channel(chan->center_freq);
		memcpy(ssid.SSID, sme->ssid, sme->ssid_len);
		ssid.SSID_len = sme->ssid_len;
		chan_cnt = get_roam_channel_list(wl->channel, chanspec_list, &ssid, ioctl_version);
#else
		wl->channel = ieee80211_frequency_to_channel(chan->center_freq);
		chan_cnt = 1;
#endif 
		WL_DBG(("channel (%d), center_req (%d), %d channels\n", wl->channel,
			chan->center_freq, chan_cnt));
	} else
		wl->channel = 0;
#ifdef BCMWAPI_WPI
	WL_DBG(("1. enable wapi auth\n"));
	if (sme->crypto.wpa_versions & NL80211_WAPI_VERSION_1) {
		WL_DBG(("2. set wapi ie  \n"));
		err = wl_set_set_wapi_ie(dev, sme);
		if (unlikely(err))
			return err;
	} else
		WL_DBG(("2. Not wapi ie  \n"));
#endif
	WL_DBG(("ie (%p), ie_len (%zd)\n", sme->ie, sme->ie_len));
	WL_DBG(("3. set wapi version \n"));
	err = wl_set_wpa_version(dev, sme);
	if (unlikely(err)) {
		WL_ERR(("Invalid wpa_version\n"));
		return err;
	}
#ifdef BCMWAPI_WPI
	if (sme->crypto.wpa_versions & NL80211_WAPI_VERSION_1)
		WL_DBG(("4. WAPI Dont Set wl_set_auth_type\n"));
	else {
		WL_DBG(("4. wl_set_auth_type\n"));
#endif
		err = wl_set_auth_type(dev, sme);
		if (unlikely(err)) {
			WL_ERR(("Invalid auth type\n"));
			return err;
		}
#ifdef BCMWAPI_WPI
	}
#endif

	err = wl_set_set_cipher(dev, sme);
	if (unlikely(err)) {
		WL_ERR(("Invalid ciper\n"));
		return err;
	}

	err = wl_set_key_mgmt(dev, sme);
	if (unlikely(err)) {
		WL_ERR(("Invalid key mgmt\n"));
		return err;
	}

	err = wl_set_set_sharedkey(dev, sme);
	if (unlikely(err)) {
		WL_ERR(("Invalid shared key\n"));
		return err;
	}

	join_params_size = WL_EXTJOIN_PARAMS_FIXED_SIZE +
		chan_cnt * sizeof(chanspec_t);
	ext_join_params =  (wl_extjoin_params_t*)kzalloc(join_params_size, GFP_KERNEL);
	if (ext_join_params == NULL) {
		err = -ENOMEM;
		wl_clr_drv_status(wl, CONNECTING, dev);
		goto exit;
	}
	ext_join_params->ssid.SSID_len = min(sizeof(ext_join_params->ssid.SSID), sme->ssid_len);
	memcpy(&ext_join_params->ssid.SSID, sme->ssid, ext_join_params->ssid.SSID_len);
	ext_join_params->ssid.SSID_len = htod32(ext_join_params->ssid.SSID_len);
	ext_join_params->scan.active_time = WL_SCAN_ACTIVE_TIME*8;
	ext_join_params->scan.passive_time = WL_SCAN_PASSIVE_TIME*3;
	
	ext_join_params->scan.scan_type = -1;
	ext_join_params->scan.nprobes
		= (ext_join_params->scan.active_time/(WL_SCAN_ACTIVE_TIME *2));
	ext_join_params->scan.home_time = -1;

	if (sme->bssid)
		memcpy(&ext_join_params->assoc.bssid, sme->bssid, ETH_ALEN);
	else
		memcpy(&ext_join_params->assoc.bssid, &ether_bcast, ETH_ALEN);
	ext_join_params->assoc.chanspec_num = chan_cnt;
	if (chan_cnt) {
#ifdef ROAM_CHANNEL_CACHE
		memcpy(ext_join_params->assoc.chanspec_list, chanspec_list,
			sizeof(chanspec_t) * chan_cnt);
#else
		u16 channel, band, bw, ctl_sb;
		chanspec_t chspec;
		channel = wl->channel;
		band = (channel <= CH_MAX_2G_CHANNEL) ? WL_CHANSPEC_BAND_2G
			: WL_CHANSPEC_BAND_5G;
		bw = WL_CHANSPEC_BW_20;
		ctl_sb = WL_CHANSPEC_CTL_SB_NONE;
		chspec = (channel | band | bw | ctl_sb);
		ext_join_params->assoc.chanspec_list[0]  &= WL_CHANSPEC_CHAN_MASK;
		ext_join_params->assoc.chanspec_list[0] |= chspec;
		ext_join_params->assoc.chanspec_list[0] =
			wl_chspec_host_to_driver(ext_join_params->assoc.chanspec_list[0]);
#endif 
	}
	ext_join_params->assoc.chanspec_num = htod32(ext_join_params->assoc.chanspec_num);
	if (ext_join_params->ssid.SSID_len < IEEE80211_MAX_SSID_LEN) {
		WL_INFO(("ssid \"%s\", len (%d)\n", ext_join_params->ssid.SSID,
			ext_join_params->ssid.SSID_len));
	}
#if 0 
    if (wl->apsta_concurrent && ap_net_dev) {
        bss_setbuf_t bss_setbuf;
		int res;

        bss_setbuf.cfg = 1;
        bss_setbuf.val = 0;  

        if ((res = wldev_iovar_setbuf_bsscfg(dev, "bss", &bss_setbuf, sizeof(bss_setbuf), 
            wl->ioctl_buf, WLC_IOCTL_MAXLEN, 1, &wl->ioctl_buf_sync)) < 0){
            WL_ERR(("%s: ERROR:%d, set bss up failed\n", __FUNCTION__, res));
        }
		printf("%s: stop the ap part of apsta concurrent\n", __FUNCTION__);
	}
#endif

	wl_set_drv_status(wl, CONNECTING, dev);
	err = wldev_iovar_setbuf_bsscfg(dev, "join", ext_join_params, join_params_size,
		wl->ioctl_buf, WLC_IOCTL_MAXLEN, wl_cfgp2p_find_idx(wl, dev), &wl->ioctl_buf_sync);
	kfree(ext_join_params);
	if (err) {
		wl_clr_drv_status(wl, CONNECTING, dev);
		if (err == BCME_UNSUPPORTED) {
			WL_DBG(("join iovar is not supported\n"));
			goto set_ssid;
		} else
			WL_ERR(("error (%d)\n", err));
	} else
		goto exit;

set_ssid:
	memset(&join_params, 0, sizeof(join_params));
	join_params_size = sizeof(join_params.ssid);

	join_params.ssid.SSID_len = min(sizeof(join_params.ssid.SSID), sme->ssid_len);
	memcpy(&join_params.ssid.SSID, sme->ssid, join_params.ssid.SSID_len);
	join_params.ssid.SSID_len = htod32(join_params.ssid.SSID_len);
	wl_update_prof(wl, dev, NULL, &join_params.ssid, WL_PROF_SSID);
	if (sme->bssid)
		memcpy(&join_params.params.bssid, sme->bssid, ETH_ALEN);
	else
		memcpy(&join_params.params.bssid, &ether_bcast, ETH_ALEN);

	wl_ch_to_chanspec(wl->channel, &join_params, &join_params_size);
	WL_DBG(("join_param_size %d\n", join_params_size));

	if (join_params.ssid.SSID_len < IEEE80211_MAX_SSID_LEN) {
		WL_INFO(("ssid \"%s\", len (%d)\n", join_params.ssid.SSID,
			join_params.ssid.SSID_len));
	}
	wl_set_drv_status(wl, CONNECTING, dev);
	err = wldev_ioctl(dev, WLC_SET_SSID, &join_params, join_params_size, true);
	if (err) {
		WL_ERR(("error (%d)\n", err));
		wl_clr_drv_status(wl, CONNECTING, dev);
	}
exit:
	return err;
}

static s32
wl_cfg80211_disconnect(struct wiphy *wiphy, struct net_device *dev,
	u16 reason_code)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	scb_val_t scbval;
	bool act = false;
	s32 err = 0;
	u8 *curbssid;
	WL_ERR(("Reason %d\n", reason_code));
	CHECK_SYS_UP(wl);
	act = *(bool *) wl_read_prof(wl, dev, WL_PROF_ACT);
	curbssid = wl_read_prof(wl, dev, WL_PROF_BSSID);
#if (defined(BCM4334_CHIP) || !defined(ESCAN_RESULT_PATCH))
		
		
		if (is_scan_request_valid(wl->scan_request)) {
			wl_notify_escan_complete(wl, dev, true, true);
		}
		else WL_ERR(("is_scan_request_valid: 0 (2)\n"));
#endif 
	if (act) {
		wl_set_drv_status(wl, DISCONNECTING, dev);
		scbval.val = reason_code;
		memcpy(&scbval.ea, curbssid, ETHER_ADDR_LEN);
		scbval.val = htod32(scbval.val);
		err = wldev_ioctl(dev, WLC_DISASSOC, &scbval,
			sizeof(scb_val_t), true);
		if (unlikely(err)) {
			wl_clr_drv_status(wl, DISCONNECTING, dev);
			WL_ERR(("error (%d)\n", err));
			return err;
		}
	}

	return err;
}

static s32
wl_cfg80211_set_tx_power(struct wiphy *wiphy,
	enum nl80211_tx_power_setting type, s32 dbm)
{

	struct wl_priv *wl = wiphy_priv(wiphy);
	struct net_device *ndev = wl_to_prmry_ndev(wl);
	u16 txpwrmw;
	s32 err = 0;
	s32 disable = 0;

	CHECK_SYS_UP(wl);
	switch (type) {
	case NL80211_TX_POWER_AUTOMATIC:
		break;
	case NL80211_TX_POWER_LIMITED:
		if (dbm < 0) {
			WL_ERR(("TX_POWER_LIMITTED - dbm is negative\n"));
			return -EINVAL;
		}
		break;
	case NL80211_TX_POWER_FIXED:
		if (dbm < 0) {
			WL_ERR(("TX_POWER_FIXED - dbm is negative..\n"));
			return -EINVAL;
		}
		break;
	}
	
	disable = WL_RADIO_SW_DISABLE << 16;
	disable = htod32(disable);
	err = wldev_ioctl(ndev, WLC_SET_RADIO, &disable, sizeof(disable), true);
	if (unlikely(err)) {
		WL_ERR(("WLC_SET_RADIO error (%d)\n", err));
		return err;
	}

	if (dbm > 0xffff)
		txpwrmw = 0xffff;
	else
		txpwrmw = (u16) dbm;
	err = wldev_iovar_setint(ndev, "qtxpower",
		(s32) (bcm_mw_to_qdbm(txpwrmw)));
	if (unlikely(err)) {
		WL_ERR(("qtxpower error (%d)\n", err));
		return err;
	}
	wl->conf->tx_power = dbm;

	return err;
}

static s32 wl_cfg80211_get_tx_power(struct wiphy *wiphy, s32 *dbm)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct net_device *ndev = wl_to_prmry_ndev(wl);
	s32 txpwrdbm;
	u8 result;
	s32 err = 0;

	CHECK_SYS_UP(wl);
	err = wldev_iovar_getint(ndev, "qtxpower", &txpwrdbm);
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
		return err;
	}
	result = (u8) (txpwrdbm & ~WL_TXPWR_OVERRIDE);
	*dbm = (s32) bcm_qdbm_to_mw(result);

	return err;
}

static s32
wl_cfg80211_config_default_key(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, bool unicast, bool multicast)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	u32 index;
	s32 wsec;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	WL_DBG(("key index (%d)\n", key_idx));
	CHECK_SYS_UP(wl);
	err = wldev_iovar_getint_bsscfg(dev, "wsec", &wsec, bssidx);
	if (unlikely(err)) {
		WL_ERR(("WLC_GET_WSEC error (%d)\n", err));
		return err;
	}
	if (wsec == WEP_ENABLED) {
		
		index = (u32) key_idx;
		index = htod32(index);
		err = wldev_ioctl(dev, WLC_SET_KEY_PRIMARY, &index,
			sizeof(index), true);
		if (unlikely(err)) {
			WL_ERR(("error (%d)\n", err));
		}
	}
	return err;
}

static s32
wl_add_keyext(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, const u8 *mac_addr, struct key_params *params)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct wl_wsec_key key;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);
	s32 mode = wl_get_mode_by_netdev(wl, dev);
	memset(&key, 0, sizeof(key));
	key.index = (u32) key_idx;

	if (!ETHER_ISMULTI(mac_addr))
		memcpy((char *)&key.ea, (void *)mac_addr, ETHER_ADDR_LEN);
	key.len = (u32) params->key_len;

	
	if (key.len == 0) {
		
		swap_key_from_BE(&key);
		err = wldev_iovar_setbuf_bsscfg(dev, "wsec_key", &key, sizeof(key),
			wl->ioctl_buf, WLC_IOCTL_MAXLEN, bssidx, &wl->ioctl_buf_sync);
		if (unlikely(err)) {
			WL_ERR(("key delete error (%d)\n", err));
			return err;
		}
	} else {
		if (key.len > sizeof(key.data)) {
			WL_ERR(("Invalid key length (%d)\n", key.len));
			return -EINVAL;
		}
		WL_DBG(("Setting the key index %d\n", key.index));
		memcpy(key.data, params->key, key.len);

		if ((mode == WL_MODE_BSS) &&
			(params->cipher == WLAN_CIPHER_SUITE_TKIP)) {
			u8 keybuf[8];
			memcpy(keybuf, &key.data[24], sizeof(keybuf));
			memcpy(&key.data[24], &key.data[16], sizeof(keybuf));
			memcpy(&key.data[16], keybuf, sizeof(keybuf));
		}

		
		if (params->seq && params->seq_len == 6) {
			
			u8 *ivptr;
			ivptr = (u8 *) params->seq;
			key.rxiv.hi = (ivptr[5] << 24) | (ivptr[4] << 16) |
				(ivptr[3] << 8) | ivptr[2];
			key.rxiv.lo = (ivptr[1] << 8) | ivptr[0];
			key.iv_initialized = true;
		}

		switch (params->cipher) {
		case WLAN_CIPHER_SUITE_WEP40:
			key.algo = CRYPTO_ALGO_WEP1;
			WL_DBG(("WLAN_CIPHER_SUITE_WEP40\n"));
			break;
		case WLAN_CIPHER_SUITE_WEP104:
			key.algo = CRYPTO_ALGO_WEP128;
			WL_DBG(("WLAN_CIPHER_SUITE_WEP104\n"));
			break;
		case WLAN_CIPHER_SUITE_TKIP:
			key.algo = CRYPTO_ALGO_TKIP;
			WL_DBG(("WLAN_CIPHER_SUITE_TKIP\n"));
			break;
		case WLAN_CIPHER_SUITE_AES_CMAC:
			key.algo = CRYPTO_ALGO_AES_CCM;
			WL_DBG(("WLAN_CIPHER_SUITE_AES_CMAC\n"));
			break;
		case WLAN_CIPHER_SUITE_CCMP:
			key.algo = CRYPTO_ALGO_AES_CCM;
			WL_DBG(("WLAN_CIPHER_SUITE_CCMP\n"));
			break;
#ifdef BCMWAPI_WPI
		case WLAN_CIPHER_SUITE_SMS4:
			key.algo = CRYPTO_ALGO_SMS4;
			WL_DBG(("WLAN_CIPHER_SUITE_SMS4\n"));
			break;
#endif
		default:
			WL_ERR(("Invalid cipher (0x%x)\n", params->cipher));
			return -EINVAL;
		}
		swap_key_from_BE(&key);
		
		dhd_wait_pend8021x(dev);
		err = wldev_iovar_setbuf_bsscfg(dev, "wsec_key", &key, sizeof(key),
			wl->ioctl_buf, WLC_IOCTL_MAXLEN, bssidx, &wl->ioctl_buf_sync);
		if (unlikely(err)) {
			WL_ERR(("WLC_SET_KEY error (%d)\n", err));
			return err;
		}
	}
	return err;
}

static s32
wl_cfg80211_add_key(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, bool pairwise, const u8 *mac_addr,
	struct key_params *params)
{
	struct wl_wsec_key key;
	s32 val = 0;
	s32 wsec = 0;
	s32 err = 0;
	u8 keybuf[8];
	s32 bssidx = 0;
	struct wl_priv *wl = wiphy_priv(wiphy);
	s32 mode = wl_get_mode_by_netdev(wl, dev);
	WL_DBG(("key index (%d)\n", key_idx));
	CHECK_SYS_UP(wl);

	bssidx = wl_cfgp2p_find_idx(wl, dev);

	if (mac_addr) {
		wl_add_keyext(wiphy, dev, key_idx, mac_addr, params);
		goto exit;
	}
	memset(&key, 0, sizeof(key));

	key.len = (u32) params->key_len;
	key.index = (u32) key_idx;

	if (unlikely(key.len > sizeof(key.data))) {
		WL_ERR(("Too long key length (%u)\n", key.len));
		return -EINVAL;
	}
	memcpy(key.data, params->key, key.len);

	key.flags = WL_PRIMARY_KEY;
	switch (params->cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
		key.algo = CRYPTO_ALGO_WEP1;
		val = WEP_ENABLED;
		WL_DBG(("WLAN_CIPHER_SUITE_WEP40\n"));
		break;
	case WLAN_CIPHER_SUITE_WEP104:
		key.algo = CRYPTO_ALGO_WEP128;
		val = WEP_ENABLED;
		WL_DBG(("WLAN_CIPHER_SUITE_WEP104\n"));
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		key.algo = CRYPTO_ALGO_TKIP;
		val = TKIP_ENABLED;
		
		if (mode == WL_MODE_BSS) {
			bcopy(&key.data[24], keybuf, sizeof(keybuf));
			bcopy(&key.data[16], &key.data[24], sizeof(keybuf));
			bcopy(keybuf, &key.data[16], sizeof(keybuf));
		}
		WL_DBG(("WLAN_CIPHER_SUITE_TKIP\n"));
		break;
	case WLAN_CIPHER_SUITE_AES_CMAC:
		key.algo = CRYPTO_ALGO_AES_CCM;
		val = AES_ENABLED;
		WL_DBG(("WLAN_CIPHER_SUITE_AES_CMAC\n"));
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		key.algo = CRYPTO_ALGO_AES_CCM;
		val = AES_ENABLED;
		WL_DBG(("WLAN_CIPHER_SUITE_CCMP\n"));
		break;
#ifdef BCMWAPI_WPI
	case WLAN_CIPHER_SUITE_SMS4:
		key.algo = CRYPTO_ALGO_SMS4;
		WL_DBG(("WLAN_CIPHER_SUITE_SMS4\n"));
		val = SMS4_ENABLED;
		break;
#endif 
	default:
		WL_ERR(("Invalid cipher (0x%x)\n", params->cipher));
		return -EINVAL;
	}

	
	swap_key_from_BE(&key);
	err = wldev_iovar_setbuf_bsscfg(dev, "wsec_key", &key, sizeof(key), wl->ioctl_buf,
		WLC_IOCTL_MAXLEN, bssidx, &wl->ioctl_buf_sync);
	if (unlikely(err)) {
		WL_ERR(("WLC_SET_KEY error (%d)\n", err));
		return err;
	}

exit:
	err = wldev_iovar_getint_bsscfg(dev, "wsec", &wsec, bssidx);
	if (unlikely(err)) {
		WL_ERR(("get wsec error (%d)\n", err));
		return err;
	}

	wsec |= val;
	err = wldev_iovar_setint_bsscfg(dev, "wsec", wsec, bssidx);
	if (unlikely(err)) {
		WL_ERR(("set wsec error (%d)\n", err));
		return err;
	}

	return err;
}

static s32
wl_cfg80211_del_key(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, bool pairwise, const u8 *mac_addr)
{
	struct wl_wsec_key key;
	struct wl_priv *wl = wiphy_priv(wiphy);
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	WL_DBG(("Enter\n"));

#ifndef IEEE80211W
	if ((key_idx >= DOT11_MAX_DEFAULT_KEYS) && (key_idx < DOT11_MAX_DEFAULT_KEYS+2))
		return -EINVAL;
#endif

	CHECK_SYS_UP(wl);
	memset(&key, 0, sizeof(key));

	key.flags = WL_PRIMARY_KEY;
	key.algo = CRYPTO_ALGO_OFF;
	key.index = (u32) key_idx;

	WL_DBG(("key index (%d)\n", key_idx));
	
	swap_key_from_BE(&key);
	err = wldev_iovar_setbuf_bsscfg(dev, "wsec_key", &key, sizeof(key), wl->ioctl_buf,
		WLC_IOCTL_MAXLEN, bssidx, &wl->ioctl_buf_sync);
	if (unlikely(err)) {
		if (err == -EINVAL) {
			if (key.index >= DOT11_MAX_DEFAULT_KEYS) {
				
				WL_DBG(("invalid key index (%d)\n", key_idx));
			}
		} else {
			WL_ERR(("WLC_SET_KEY error (%d)\n", err));
		}
		return err;
	}
	return err;
}

static s32
wl_cfg80211_get_key(struct wiphy *wiphy, struct net_device *dev,
	u8 key_idx, bool pairwise, const u8 *mac_addr, void *cookie,
	void (*callback) (void *cookie, struct key_params * params))
{
	struct key_params params;
	struct wl_wsec_key key;
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct wl_security *sec;
	s32 wsec;
	s32 err = 0;
	s32 bssidx = wl_cfgp2p_find_idx(wl, dev);

	WL_DBG(("key index (%d)\n", key_idx));
	CHECK_SYS_UP(wl);
	memset(&key, 0, sizeof(key));
	key.index = key_idx;
	swap_key_to_BE(&key);
	memset(&params, 0, sizeof(params));
	params.key_len = (u8) min_t(u8, DOT11_MAX_KEY_SIZE, key.len);
	memcpy(params.key, key.data, params.key_len);

	wldev_iovar_getint_bsscfg(dev, "wsec", &wsec, bssidx);
	if (unlikely(err)) {
		WL_ERR(("WLC_GET_WSEC error (%d)\n", err));
		return err;
	}
	switch (wsec & ~SES_OW_ENABLED) {
		case WEP_ENABLED:
			sec = wl_read_prof(wl, dev, WL_PROF_SEC);
			if (sec->cipher_pairwise & WLAN_CIPHER_SUITE_WEP40) {
				params.cipher = WLAN_CIPHER_SUITE_WEP40;
				WL_DBG(("WLAN_CIPHER_SUITE_WEP40\n"));
			} else if (sec->cipher_pairwise & WLAN_CIPHER_SUITE_WEP104) {
				params.cipher = WLAN_CIPHER_SUITE_WEP104;
				WL_DBG(("WLAN_CIPHER_SUITE_WEP104\n"));
			}
			break;
		case TKIP_ENABLED:
			params.cipher = WLAN_CIPHER_SUITE_TKIP;
			WL_DBG(("WLAN_CIPHER_SUITE_TKIP\n"));
			break;
		case AES_ENABLED:
			params.cipher = WLAN_CIPHER_SUITE_AES_CMAC;
			WL_DBG(("WLAN_CIPHER_SUITE_AES_CMAC\n"));
			break;
#ifdef BCMWAPI_WPI
		case WLAN_CIPHER_SUITE_SMS4:
			key.algo = CRYPTO_ALGO_SMS4;
			WL_DBG(("WLAN_CIPHER_SUITE_SMS4\n"));
			break;
#endif
		default:
			WL_ERR(("Invalid algo (0x%x)\n", wsec));
			return -EINVAL;
	}

	callback(cookie, &params);
	return err;
}

static s32
wl_cfg80211_config_default_mgmt_key(struct wiphy *wiphy,
	struct net_device *dev, u8 key_idx)
{
	WL_INFO(("Not supported\n"));
	return -EOPNOTSUPP;
}

extern void wl_android_traffic_monitor(struct net_device *dev);

static int rssi_errcnt = 0;
static int old_rssi = 0;

static s32
wl_cfg80211_get_station(struct wiphy *wiphy, struct net_device *dev,
	u8 *mac, struct station_info *sinfo)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	scb_val_t scb_val;
	s32 rssi;
	s32 rate;
	s32 err = 0;
	sta_info_t *sta;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
	s8 eabuf[ETHER_ADDR_STR_LEN];
#endif
	dhd_pub_t *dhd =  (dhd_pub_t *)(wl->pub);
	CHECK_SYS_UP(wl);
	if (wl_get_mode_by_netdev(wl, dev) == WL_MODE_AP) {
		err = wldev_iovar_getbuf(dev, "sta_info", (struct ether_addr *)mac,
			ETHER_ADDR_LEN, wl->ioctl_buf, WLC_IOCTL_SMLEN, &wl->ioctl_buf_sync);
		if (err < 0) {
			WL_ERR(("GET STA INFO failed, %d\n", err));
			return err;
		}
		sinfo->filled = STATION_INFO_INACTIVE_TIME;
		sta = (sta_info_t *)wl->ioctl_buf;
		sta->len = dtoh16(sta->len);
		sta->cap = dtoh16(sta->cap);
		sta->flags = dtoh32(sta->flags);
		sta->idle = dtoh32(sta->idle);
		sta->in = dtoh32(sta->in);
		sinfo->inactive_time = sta->idle * 1000;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 0, 0)
		if (sta->flags & WL_STA_ASSOC) {
			sinfo->filled |= STATION_INFO_CONNECTED_TIME;
			sinfo->connected_time = sta->in;
		}
		WL_INFO(("STA %s : idle time : %d sec, connected time :%d ms\n",
			bcm_ether_ntoa((const struct ether_addr *)mac, eabuf), sinfo->inactive_time,
			sta->idle * 1000));
#endif
	} else if (wl_get_mode_by_netdev(wl, dev) == WL_MODE_BSS) {
		u8 *curmacp = wl_read_prof(wl, dev, WL_PROF_BSSID);
		if (!wl_get_drv_status(wl, CONNECTED, dev) ||
			(dhd_is_associated(dhd, NULL, &err) == FALSE)) {
			WL_ERR(("NOT assoc\n"));
			if (err == -ERESTARTSYS)
				return err;
			err = -ENODEV;
			return err;
		}
		if (memcmp(mac, curmacp, ETHER_ADDR_LEN)) {
			WL_ERR(("Wrong Mac address: "MACSTR" != "MACSTR"\n",
				MAC2STR(mac), MAC2STR(curmacp)));
		}

		
		err = wldev_ioctl(dev, WLC_GET_RATE, &rate, sizeof(rate), false);
		if (err) {
			WL_ERR(("Could not get rate (%d)\n", err));
		} else {
			rate = dtoh32(rate);
			sinfo->filled |= STATION_INFO_TX_BITRATE;
			sinfo->txrate.legacy = rate * 5;
			WL_DBG(("Rate %d Mbps\n", (rate / 2)));
		}

		memset(&scb_val, 0, sizeof(scb_val));
		scb_val.val = 0;
		err = wldev_ioctl(dev, WLC_GET_RSSI, &scb_val,
			sizeof(scb_val_t), false);
		if (err) {
			WL_ERR(("Could not get rssi (%d)\n", err));
			
			if(rssi_errcnt > 2)
				goto get_station_err;
			else{
				WL_ERR(("counter Get rssi fail cnt[%d] errcode[%d]\n",rssi_errcnt,err));
				sinfo->signal = old_rssi;
				err = 0;
				rssi_errcnt++;
				return err;
			}	
		}else{
			rssi_errcnt = 0;
		}

		rssi = dtoh32(scb_val.val) + RSSI_OFFSET;
		sinfo->filled |= STATION_INFO_SIGNAL;
		sinfo->signal = rssi;
		old_rssi = rssi; 
		WL_DBG(("RSSI %d dBm\n", rssi));

get_station_err:
		if (err && (err != -ERESTARTSYS)) {
			
			WL_ERR(("force cfg80211_disconnected\n"));
			
			rssi_errcnt = 0;
			old_rssi = 0;
			wl_clr_drv_status(wl, CONNECTED, dev);
			cfg80211_disconnected(dev, 0, NULL, 0, GFP_KERNEL);
			wl_link_down(wl);
		}
		wl_android_traffic_monitor(dev);
	}

	return err;
}

static s32
wl_cfg80211_set_power_mgmt(struct wiphy *wiphy, struct net_device *dev,
	bool enabled, s32 timeout)
{
	s32 pm;
	s32 err = 0;
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct net_info *_net_info = wl_get_netinfo_by_netdev(wl, dev);

	CHECK_SYS_UP(wl);

	if (wl->p2p_net == dev || _net_info == NULL) {
		return err;
	}

	pm = enabled ? PM_FAST : PM_OFF;
	
	if (_net_info->pm_block || wl->vsdb_mode) {
		WL_DBG(("Do not enable the power save\n"));
		pm = PM_OFF;
	}
	pm = htod32(pm);
	WL_DBG(("power save %s\n", (pm ? "enabled" : "disabled")));
	err = wldev_ioctl(dev, WLC_SET_PM, &pm, sizeof(pm), true);
	if (unlikely(err)) {
		if (err == -ENODEV)
			WL_DBG(("net_device is not ready yet\n"));
		else
			WL_ERR(("error (%d)\n", err));
		return err;
	}
	return err;
}

static __used u32 wl_find_msb(u16 bit16)
{
	u32 ret = 0;

	if (bit16 & 0xff00) {
		ret += 8;
		bit16 >>= 8;
	}

	if (bit16 & 0xf0) {
		ret += 4;
		bit16 >>= 4;
	}

	if (bit16 & 0xc) {
		ret += 2;
		bit16 >>= 2;
	}

	if (bit16 & 2)
		ret += bit16 & 2;
	else if (bit16)
		ret += bit16;

	return ret;
}

static s32 wl_cfg80211_resume(struct wiphy *wiphy)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct net_device *ndev = wl_to_prmry_ndev(wl);
	s32 err = 0;

	if (unlikely(!wl_get_drv_status(wl, READY, ndev))) {
		WL_INFO(("device is not ready\n"));
		return 0;
	}

	wl_invoke_iscan(wl);

	return err;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 39)
static s32 wl_cfg80211_suspend(struct wiphy *wiphy, struct cfg80211_wowlan *wow)
#else
static s32 wl_cfg80211_suspend(struct wiphy *wiphy)
#endif
{
#ifdef DHD_CLEAR_ON_SUSPEND
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct net_info *iter, *next;
	struct net_device *ndev = wl_to_prmry_ndev(wl);
	unsigned long flags;
	if (unlikely(!wl_get_drv_status(wl, READY, ndev))) {
		WL_INFO(("device is not ready : status (%d)\n",
			(int)wl->status));
		return 0;
	}
	for_each_ndev(wl, iter, next)
		wl_set_drv_status(wl, SCAN_ABORTING, iter->ndev);
	wl_term_iscan(wl);
	spin_lock_irqsave(&wl->cfgdrv_lock, flags);
	if (wl->scan_request) {
		cfg80211_scan_done(wl->scan_request, true);
		wl->scan_request = NULL;
	}
	for_each_ndev(wl, iter, next) {
		wl_clr_drv_status(wl, SCANNING, iter->ndev);
		wl_clr_drv_status(wl, SCAN_ABORTING, iter->ndev);
	}
	spin_unlock_irqrestore(&wl->cfgdrv_lock, flags);
	for_each_ndev(wl, iter, next) {
		if (wl_get_drv_status(wl, CONNECTING, iter->ndev)) {
			wl_bss_connect_done(wl, iter->ndev, NULL, NULL, false);
		}
	}
#endif 
	return 0;
}

static s32
wl_update_pmklist(struct net_device *dev, struct wl_pmk_list *pmk_list,
	s32 err)
{
	int i, j;
	struct wl_priv *wl = wlcfg_drv_priv;
	struct net_device *primary_dev = wl_to_prmry_ndev(wl);

	if (!pmk_list) {
		printk("pmk_list is NULL\n");
		return -EINVAL;
	}
	if (primary_dev != dev) {
		WL_INFO(("Not supporting Flushing pmklist on virtual"
			" interfaces than primary interface\n"));
		return err;
	}

	WL_DBG(("No of elements %d\n", pmk_list->pmkids.npmkid));
	for (i = 0; i < pmk_list->pmkids.npmkid; i++) {
		WL_DBG(("PMKID[%d]: %pM =\n", i,
			&pmk_list->pmkids.pmkid[i].BSSID));
		for (j = 0; j < WPA2_PMKID_LEN; j++) {
			WL_DBG(("%02x\n", pmk_list->pmkids.pmkid[i].PMKID[j]));
		}
	}
	if (likely(!err)) {
		err = wldev_iovar_setbuf(dev, "pmkid_info", (char *)pmk_list,
			sizeof(*pmk_list), wl->ioctl_buf, WLC_IOCTL_MAXLEN, NULL);
	}

	return err;
}

static s32
wl_cfg80211_set_pmksa(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_pmksa *pmksa)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	s32 err = 0;
	int i;

	CHECK_SYS_UP(wl);
	for (i = 0; i < wl->pmk_list->pmkids.npmkid; i++)
		if (!memcmp(pmksa->bssid, &wl->pmk_list->pmkids.pmkid[i].BSSID,
			ETHER_ADDR_LEN))
			break;
	if (i < WL_NUM_PMKIDS_MAX) {
		memcpy(&wl->pmk_list->pmkids.pmkid[i].BSSID, pmksa->bssid,
			ETHER_ADDR_LEN);
		memcpy(&wl->pmk_list->pmkids.pmkid[i].PMKID, pmksa->pmkid,
			WPA2_PMKID_LEN);
		if (i == wl->pmk_list->pmkids.npmkid)
			wl->pmk_list->pmkids.npmkid++;
	} else {
		err = -EINVAL;
	}
	WL_DBG(("set_pmksa,IW_PMKSA_ADD - PMKID: %pM =\n",
		&wl->pmk_list->pmkids.pmkid[wl->pmk_list->pmkids.npmkid - 1].BSSID));
	for (i = 0; i < WPA2_PMKID_LEN; i++) {
		WL_DBG(("%02x\n",
			wl->pmk_list->pmkids.pmkid[wl->pmk_list->pmkids.npmkid - 1].
			PMKID[i]));
	}

	err = wl_update_pmklist(dev, wl->pmk_list, err);

	return err;
}

static s32
wl_cfg80211_del_pmksa(struct wiphy *wiphy, struct net_device *dev,
	struct cfg80211_pmksa *pmksa)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct _pmkid_list pmkid;
	s32 err = 0;
	int i;

	CHECK_SYS_UP(wl);
	memcpy(&pmkid.pmkid[0].BSSID, pmksa->bssid, ETHER_ADDR_LEN);
	memcpy(&pmkid.pmkid[0].PMKID, pmksa->pmkid, WPA2_PMKID_LEN);

	WL_DBG(("del_pmksa,IW_PMKSA_REMOVE - PMKID: %pM =\n",
		&pmkid.pmkid[0].BSSID));
	for (i = 0; i < WPA2_PMKID_LEN; i++) {
		WL_DBG(("%02x\n", pmkid.pmkid[0].PMKID[i]));
	}

	for (i = 0; i < wl->pmk_list->pmkids.npmkid; i++)
		if (!memcmp
		    (pmksa->bssid, &wl->pmk_list->pmkids.pmkid[i].BSSID,
		     ETHER_ADDR_LEN))
			break;

	if ((wl->pmk_list->pmkids.npmkid > 0) &&
		(i < wl->pmk_list->pmkids.npmkid)) {
		memset(&wl->pmk_list->pmkids.pmkid[i], 0, sizeof(pmkid_t));
		for (; i < (wl->pmk_list->pmkids.npmkid - 1); i++) {
			memcpy(&wl->pmk_list->pmkids.pmkid[i].BSSID,
				&wl->pmk_list->pmkids.pmkid[i + 1].BSSID,
				ETHER_ADDR_LEN);
			memcpy(&wl->pmk_list->pmkids.pmkid[i].PMKID,
				&wl->pmk_list->pmkids.pmkid[i + 1].PMKID,
				WPA2_PMKID_LEN);
		}
		wl->pmk_list->pmkids.npmkid--;
	} else {
		err = -EINVAL;
	}

	err = wl_update_pmklist(dev, wl->pmk_list, err);

	return err;

}

static s32
wl_cfg80211_flush_pmksa(struct wiphy *wiphy, struct net_device *dev)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	s32 err = 0;
	CHECK_SYS_UP(wl);
	memset(wl->pmk_list, 0, sizeof(*wl->pmk_list));
	err = wl_update_pmklist(dev, wl->pmk_list, err);
	return err;

}

static wl_scan_params_t *
wl_cfg80211_scan_alloc_params(int channel, int nprobes, int *out_params_size)
{
	wl_scan_params_t *params;
	int params_size;
	int num_chans;
	
	gfp_t aflags;
	

	*out_params_size = 0;

	
	params_size = WL_SCAN_PARAMS_FIXED_SIZE + 1 * sizeof(uint16);
	
	aflags = (in_atomic()) ? GFP_ATOMIC : GFP_KERNEL;
	params = (wl_scan_params_t*) kzalloc(params_size, aflags);
	
	if (params == NULL) {
		WL_ERR(("%s: mem alloc failed (%d bytes)\n", __func__, params_size));
		return params;
	}
	memset(params, 0, params_size);
	params->nprobes = nprobes;

	num_chans = (channel == 0) ? 0 : 1;

	memcpy(&params->bssid, &ether_bcast, ETHER_ADDR_LEN);
	params->bss_type = DOT11_BSSTYPE_ANY;
	params->scan_type = DOT11_SCANTYPE_ACTIVE;
	params->nprobes = htod32(1);
	params->active_time = htod32(-1);
	params->passive_time = htod32(-1);
	params->home_time = htod32(10);
	if (channel == -1)
		params->channel_list[0] = htodchanspec(channel);
	else
		params->channel_list[0] = wl_ch_host_to_driver(channel);

	
	params->channel_num = htod32((0 << WL_SCAN_PARAMS_NSSID_SHIFT) |
		(num_chans & WL_SCAN_PARAMS_COUNT_MASK));

	*out_params_size = params_size;	
	return params;
}
s32
wl_cfg80211_scan_abort(struct net_device *ndev)
{
 	struct wl_priv *wl = wlcfg_drv_priv;

    WL_DBG(("Enter\n"));

	if (wl->scan_request) {
		return wl_notify_escan_complete(wl, ndev, true, true);
	}

    return 0;
}

static s32
wl_cfg80211_remain_on_channel(struct wiphy *wiphy, struct net_device *dev,
	struct ieee80211_channel * channel,
	enum nl80211_channel_type channel_type,
	unsigned int duration, u64 *cookie)
{
	s32 target_channel;
	u32 id;
	struct ether_addr primary_mac;
	struct net_device *ndev = NULL;

	s32 err = BCME_OK;
	struct wl_priv *wl = wiphy_priv(wiphy);
	WL_DBG(("Enter, ifindex: %d, channel: %d, duration ms (%d) SCANNING ?? %s \n",
		dev->ifindex, ieee80211_frequency_to_channel(channel->center_freq),
		duration, (wl_get_drv_status(wl, SCANNING, ndev)) ? "YES":"NO"));

	if (wl->p2p_net == dev) {
		ndev = wl_to_prmry_ndev(wl);
	} else {
		ndev = dev;
	}

#ifndef WL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
	if (wl_get_drv_status(wl, SCANNING, ndev)) {
		wl_notify_escan_complete(wl, ndev, true, true);
	}
#endif 

	target_channel = ieee80211_frequency_to_channel(channel->center_freq);
	memcpy(&wl->remain_on_chan, channel, sizeof(struct ieee80211_channel));
	wl->remain_on_chan_type = channel_type;
	id = ++wl->last_roc_id;
	if (id == 0)
		id = ++wl->last_roc_id;
	*cookie = id;
	cfg80211_ready_on_channel(dev, *cookie, channel,
		channel_type, duration, GFP_KERNEL);
#ifdef WL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
	if (wl_get_drv_status(wl, SCANNING, ndev)) {
		struct timer_list *_timer;
		WL_DBG(("scan is running. go to fake listen state\n"));

		wl_set_drv_status(wl, FAKE_REMAINING_ON_CHANNEL, ndev);

		if (timer_pending(&wl->p2p->listen_timer)) {
			WL_DBG(("cancel current listen timer \n"));
			del_timer_sync(&wl->p2p->listen_timer);
		}

		_timer = &wl->p2p->listen_timer;
		wl_clr_p2p_status(wl, LISTEN_EXPIRED);

		INIT_TIMER(_timer, wl_cfgp2p_listen_expired, duration, 0);

		return BCME_OK;
	}
#endif 

#ifdef WL_CFG80211_SYNC_GON
	if (wl_get_drv_status_all(wl, WAITING_NEXT_ACT_FRM_LISTEN)) {
#ifdef WL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
		wl_set_drv_status(wl, FAKE_REMAINING_ON_CHANNEL, ndev);
#else
		wl_set_drv_status(wl, REMAINING_ON_CHANNEL, ndev);
#endif 
		goto exit;
	}
#endif 
	if (wl->p2p && !wl->p2p->on) {

		get_primary_mac(wl, &primary_mac);
		wl_cfgp2p_generate_bss_mac(&primary_mac, &wl->p2p->dev_addr, &wl->p2p->int_addr);
		p2p_on(wl) = true;
	}

	if (p2p_is_on(wl)) {
		err = wl_cfgp2p_enable_discovery(wl, ndev, NULL, 0);

		if (unlikely(err)) {
			goto exit;
		}
#ifndef WL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
		wl_set_drv_status(wl, REMAINING_ON_CHANNEL, ndev);
#endif 
		err = wl_cfgp2p_discover_listen(wl, target_channel, duration);

#ifdef WL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
		if (err == BCME_OK) {
			wl_set_drv_status(wl, REMAINING_ON_CHANNEL, ndev);
		} else {
			wl_set_drv_status(wl, FAKE_REMAINING_ON_CHANNEL, ndev);
		}
#endif 
		err = BCME_OK;
	}


exit:
	return err;
}

static s32
wl_cfg80211_cancel_remain_on_channel(struct wiphy *wiphy, struct net_device *dev,
	u64 cookie)
{
	s32 err = 0;
	WL_DBG((" enter ) netdev_ifidx: %d \n", dev->ifindex));
	return err;
}
static s32
wl_cfg80211_send_pending_tx_act_frm(struct wl_priv *wl)
{
	wl_af_params_t *tx_act_frm;
	struct net_device *dev = wl->afx_hdl->dev;
	if (dev == NULL)
		return -1;
	if (!p2p_is_on(wl))
		return -1;

	if (dev == wl->p2p_net) {
		dev = wl_to_prmry_ndev(wl);
	}

	tx_act_frm = wl->afx_hdl->pending_tx_act_frm;
	WL_DBG(("Sending the action frame\n"));
	wl->afx_hdl->pending_tx_act_frm = NULL;
	if (tx_act_frm != NULL) {

		wl_clr_drv_status(wl, SCANNING, wl->afx_hdl->dev);
#ifndef WL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
		wl_notify_escan_complete(wl, dev, true, true);
#endif 
		wl_cfgp2p_discover_enable_search(wl, false);
		tx_act_frm->channel = wl->afx_hdl->peer_chan;
		wl->afx_hdl->ack_recv = (wl_cfgp2p_tx_action_frame(wl, dev,
			tx_act_frm, wl->afx_hdl->bssidx)) ? false : true;
	}
	return 0;
}
static void
wl_cfg80211_afx_handler(struct work_struct *work)
{
	struct afx_hdl *afx_instance;
	struct wl_priv *wl = wlcfg_drv_priv;
	s32 ret = BCME_OK;
	afx_instance = container_of(work, struct afx_hdl, work);
	if (afx_instance != NULL && wl->afx_hdl->is_active) {
		if (wl->afx_hdl->is_listen && wl->afx_hdl->my_listen_chan) {
			ret = wl_cfgp2p_discover_listen(wl, wl->afx_hdl->my_listen_chan, 200);
		} else {
			ret = wl_cfgp2p_act_frm_search(wl, wl->afx_hdl->dev,
				wl->afx_hdl->bssidx, wl->afx_hdl->peer_listen_chan);
		}
		if (unlikely(ret != BCME_OK)) {
			WL_ERR(("ERROR occurred! returned value is (%d)\n", ret));
			if (wl_get_drv_status_all(wl, FINDING_COMMON_CHANNEL))
				complete(&wl->act_frm_scan);
		}
	}
}

static bool
wl_cfg80211_send_at_common_channel(struct wl_priv *wl,
	struct net_device *dev,
	wl_af_params_t *af_params)
{
	u32 max_retry = WL_CHANNEL_SYNC_RETRY;
	WL_DBG((" enter ) \n"));
	
	wl->afx_hdl->pending_tx_act_frm = af_params;
	wl->afx_hdl->bssidx = wl_cfgp2p_find_idx(wl, dev);
	wl->afx_hdl->dev = dev;
	wl->afx_hdl->retry = 0;
	wl->afx_hdl->peer_chan = WL_INVALID;
	wl->afx_hdl->ack_recv = false;
	wl_set_drv_status(wl, FINDING_COMMON_CHANNEL, dev);
	wl->afx_hdl->is_active = TRUE;
	while ((wl->afx_hdl->retry < max_retry) &&
		(wl->afx_hdl->peer_chan == WL_INVALID)) {
		wl->afx_hdl->is_listen = FALSE;
		wl_set_drv_status(wl, SCANNING, dev);
		WL_DBG(("Scheduling the action frame for sending.. retry %d\n",
			wl->afx_hdl->retry));
		
		schedule_work(&wl->afx_hdl->work);
		wait_for_completion_timeout(&wl->act_frm_scan,
			msecs_to_jiffies(MAX_WAIT_TIME));

		if ((wl->afx_hdl->peer_chan != WL_INVALID) ||
			!(wl_get_drv_status(wl, FINDING_COMMON_CHANNEL, dev)))
			break;

		if (wl->afx_hdl->my_listen_chan) {
			WL_DBG(("Scheduling Listen peer in my listen channel = %d\n",
				wl->afx_hdl->my_listen_chan));
			
			wl->afx_hdl->is_listen = TRUE;
			schedule_work(&wl->afx_hdl->work);
			wait_for_completion_timeout(&wl->act_frm_scan,
				msecs_to_jiffies(MAX_WAIT_TIME));
		}
		if (!wl_get_drv_status(wl, FINDING_COMMON_CHANNEL, dev))
			break;
		wl->afx_hdl->retry++;
		WL_AF_TX_KEEP_PRI_CONNECTION_VSDB(wl);
	}
	wl->afx_hdl->is_active = FALSE;

	wl_clr_drv_status(wl, SCANNING, dev);
	wl_clr_drv_status(wl, FINDING_COMMON_CHANNEL, dev);
	if (wl->afx_hdl->peer_chan != WL_INVALID)
		wl_cfg80211_send_pending_tx_act_frm(wl);
	else {
		WL_ERR(("Couldn't find the peer after %d retries\n",
			wl->afx_hdl->retry));
	}
	wl->afx_hdl->is_listen = FALSE;
	wl->afx_hdl->pending_tx_act_frm = NULL;
	wl->afx_hdl->dev = NULL;
	wl->afx_hdl->bssidx = WL_INVALID;

	if (wl->afx_hdl->ack_recv)
		return true; 
	else
		return false; 
}

static s32
wl_cfg80211_mgmt_tx(struct wiphy *wiphy, struct net_device *ndev,
	struct ieee80211_channel *channel, bool offchan,
	enum nl80211_channel_type channel_type,
	bool channel_type_valid, unsigned int wait,
	const u8* buf, size_t len,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	bool no_cck,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	bool dont_wait_for_ack,
#endif
	u64 *cookie)
{
	wl_action_frame_t *action_frame;
	wl_af_params_t *af_params;
	scb_val_t scb_val;
	const struct ieee80211_mgmt *mgmt;
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct net_device *dev = NULL;
	s32 err = BCME_OK;
	s32 bssidx = 0;
	u32 id;
	u32 retry = 0;
	bool ack = false;
	wifi_p2p_pub_act_frame_t *act_frm = NULL;
	wifi_p2p_action_frame_t *p2p_act_frm = NULL;
	wifi_p2psd_gas_pub_act_frame_t *sd_act_frm = NULL;
	s8 eabuf[ETHER_ADDR_STR_LEN];
#ifdef WL_CFG80211_SYNC_GON
	bool is_waiting_more_time = false;
#endif 

	WL_DBG(("Enter \n"));

	if (ndev == wl->p2p_net) {
		dev = wl_to_prmry_ndev(wl);
	} else {
		
		dev = ndev;
	}

	
	bssidx = wl_cfgp2p_find_idx(wl, dev);
	if (bssidx == -1) {

		WL_ERR(("Can not find the bssidx for dev( %p )\n", dev));
		return -ENODEV;
	}
	if (p2p_is_on(wl)) {
		if ((err = wl_cfgp2p_discover_enable_search(wl, false)) < 0) {
			WL_ERR(("Can not disable discovery mode\n"));
			return -EFAULT;
		}
	}
	*cookie = 0;
	id = wl->send_action_id++;
	if (id == 0)
		id = wl->send_action_id++;
	*cookie = id;
	mgmt = (const struct ieee80211_mgmt *)buf;
	if (ieee80211_is_mgmt(mgmt->frame_control)) {
		if (ieee80211_is_probe_resp(mgmt->frame_control)) {
			s32 ie_offset =  DOT11_MGMT_HDR_LEN + DOT11_BCN_PRB_FIXED_LEN;
			s32 ie_len = len - ie_offset;
			wl_cfgp2p_set_management_ie(wl, dev, bssidx,
				VNDR_IE_PRBRSP_FLAG, (u8 *)(buf + ie_offset), ie_len);
			cfg80211_mgmt_tx_status(ndev, *cookie, buf, len, true, GFP_KERNEL);
			goto exit;
		} else if (ieee80211_is_disassoc(mgmt->frame_control) ||
			ieee80211_is_deauth(mgmt->frame_control)) {
			memcpy(scb_val.ea.octet, mgmt->da, ETH_ALEN);
			scb_val.val = mgmt->u.disassoc.reason_code;
			wldev_ioctl(dev, WLC_SCB_DEAUTHENTICATE_FOR_REASON, &scb_val,
				sizeof(scb_val_t), true);
			WL_DBG(("Disconnect STA : %s scb_val.val %d\n",
				bcm_ether_ntoa((const struct ether_addr *)mgmt->da, eabuf),
				scb_val.val));
			wl_delay(400);
			cfg80211_mgmt_tx_status(ndev, *cookie, buf, len, true, GFP_KERNEL);
			goto exit;

		} else if (ieee80211_is_action(mgmt->frame_control)) {
#ifndef WL_CFG80211_VSDB_PRIORITIZE_SCAN_REQUEST
			wl_notify_escan_complete(wl, dev, true, true);
#endif 
		}

	} else {
		WL_ERR(("Driver only allows MGMT packet type\n"));
		goto exit;
	}

	af_params = (wl_af_params_t *) kzalloc(WL_WIFI_AF_PARAMS_SIZE, GFP_KERNEL);

	if (af_params == NULL)
	{
		WL_ERR(("unable to allocate frame\n"));
		return -ENOMEM;
	}

	action_frame = &af_params->action_frame;

	
	action_frame->packetId = *cookie;
	WL_DBG(("action frame %d\n", action_frame->packetId));
	
	memcpy(&action_frame->da, &mgmt->da[0], ETHER_ADDR_LEN);
	memcpy(&af_params->BSSID, &mgmt->bssid[0], ETHER_ADDR_LEN);

	
	action_frame->len = len - DOT11_MGMT_HDR_LEN;
	WL_DBG(("action_frame->len: %d\n", action_frame->len));

	
	af_params->channel =
		ieee80211_frequency_to_channel(channel->center_freq);

	
	wl->afx_hdl->peer_listen_chan = af_params->channel;
	WL_DBG(("channel from upper layer %d\n", wl->afx_hdl->peer_listen_chan));

	af_params->dwell_time = WL_DWELL_TIME;

	memcpy(action_frame->data, &buf[DOT11_MGMT_HDR_LEN], action_frame->len);
	if (wl_cfgp2p_is_pub_action(action_frame->data, action_frame->len)) {
		act_frm = (wifi_p2p_pub_act_frame_t *) (action_frame->data);
		WL_ERR(("P2P PUB action_frame->len: %d chan %d category %d subtype %d\n",
			action_frame->len, af_params->channel,
			act_frm->category, act_frm->subtype));

		if (act_frm->subtype == P2P_PAF_GON_REQ) {
			WL_DBG(("P2P: GO_NEG_PHASE status set \n"));
			wl_set_p2p_status(wl, GO_NEG_PHASE);

		} else if (act_frm->subtype == P2P_PAF_GON_CONF) {
			WL_DBG(("P2P: GO_NEG_PHASE status cleared \n"));
			wl_clr_p2p_status(wl, GO_NEG_PHASE);
		}

	} else if (wl_cfgp2p_is_p2p_action(action_frame->data, action_frame->len)) {
		p2p_act_frm = (wifi_p2p_action_frame_t *) (action_frame->data);
		WL_ERR(("P2P action_frame->len: %d chan %d category %d subtype %d\n",
			action_frame->len, af_params->channel,
			p2p_act_frm->category, p2p_act_frm->subtype));
	} else if (wl_cfgp2p_is_gas_action(action_frame->data, action_frame->len)) {
		sd_act_frm = (wifi_p2psd_gas_pub_act_frame_t *) (action_frame->data);
		WL_ERR(("Service Discovery action_frame->len: %d chan %d category %d action %d\n",
			action_frame->len, af_params->channel,
			sd_act_frm->category, sd_act_frm->action));
		if (sd_act_frm) {
			if (sd_act_frm->action == P2PSD_ACTION_ID_GAS_IREQ ||
				sd_act_frm->action == P2PSD_ACTION_ID_GAS_CREQ) {
				af_params->dwell_time = WL_MED_DWELL_TIME;
				wl->next_af_subtype = sd_act_frm->action + 1;
				wl_set_drv_status(wl, WAITING_NEXT_ACT_FRM, dev);
			} else {
				af_params->dwell_time = WL_MIN_DWELL_TIME;
				wl->next_af_subtype = -1;
			}
		}
	}

	wl_cfgp2p_print_actframe(true, action_frame->data, action_frame->len);
	if (act_frm) {
		if ((act_frm->subtype == P2P_PAF_GON_REQ) ||
		  (act_frm->subtype == P2P_PAF_GON_RSP) ||
		  (act_frm->subtype == P2P_PAF_GON_CONF) ||
		  (act_frm->subtype == P2P_PAF_PROVDIS_REQ)) {
			wldev_iovar_setint(dev, "mpc", 0);
		}

		if (act_frm->subtype == P2P_PAF_DEVDIS_REQ) {
			af_params->dwell_time = WL_LONG_DWELL_TIME;
		} else if (act_frm->subtype == P2P_PAF_PROVDIS_REQ ||
			act_frm->subtype == P2P_PAF_PROVDIS_RSP ||
			act_frm->subtype == P2P_PAF_GON_REQ ||
			act_frm->subtype == P2P_PAF_GON_RSP ||
			act_frm->subtype == P2P_PAF_INVITE_REQ) {
			af_params->dwell_time = WL_MED_DWELL_TIME;
		} else {
			af_params->dwell_time = WL_MIN_DWELL_TIME;
		}

#ifdef WL_CFG80211_GON_COLLISION
		if (wl->block_gon_req_tx_count && act_frm->subtype == P2P_PAF_GON_REQ) {
			
			wl->block_gon_req_tx_count--;
			WL_ERR(("Drop gon req tx action frame: count %d\n",
				wl->block_gon_req_tx_count));
			cfg80211_mgmt_tx_status(ndev, *cookie, buf, len, true, GFP_KERNEL);
			kfree(af_params);
			goto exit;
		} else if (act_frm->subtype == P2P_PAF_GON_CONF) {
			
			wl->block_gon_req_tx_count = 0;
			wl->block_gon_req_rx_count = 0;
		}
#endif 

		if (act_frm->subtype == P2P_PAF_GON_REQ) {
			wl->afx_hdl->my_listen_chan =
				wl_find_listen_channel(wl, act_frm->elts, action_frame->len);
		}

		if (act_frm->subtype == P2P_PAF_PROVDIS_REQ ||
			act_frm->subtype == P2P_PAF_DEVDIS_REQ ||
			act_frm->subtype == P2P_PAF_INVITE_REQ ||
			act_frm->subtype == P2P_PAF_GON_REQ ||
			act_frm->subtype == P2P_PAF_GON_RSP) {
#ifdef WL_CFG80211_SYNC_GON
			is_waiting_more_time = true;
#endif 

			
			if (act_frm->subtype == P2P_PAF_PROVDIS_RSP)
				wl->next_af_subtype = P2P_PAF_GON_REQ;
			else
				wl->next_af_subtype = act_frm->subtype + 1;

			wl_set_drv_status(wl, WAITING_NEXT_ACT_FRM, dev);
			wl->afx_hdl->peer_chan = WL_INVALID;
		} else
			wl->next_af_subtype = -1;
	}

#ifdef VSDB
	
	if (wl_get_drv_status(wl, CONNECTING, wl_to_prmry_ndev(wl))) {
		msleep(50);
	}
#endif

	
	if (wl_get_drv_status_all(wl, SCANNING)) {
		
		wl_notify_escan_complete(wl, ndev, true, true);
	}

	
	wl_set_drv_status(wl, SENDING_ACT_FRM, dev);
	memcpy(wl->afx_hdl->tx_dst_addr.octet,
		af_params->action_frame.da.octet,
		sizeof(wl->afx_hdl->tx_dst_addr.octet));
	if (IS_P2P_SOCIAL(af_params->channel) &&
		(IS_P2P_PUB_ACT_REQ(act_frm, &act_frm->elts[0], action_frame->len) ||
		IS_GAS_REQ(sd_act_frm, action_frame->len)) &&
		wl_to_p2p_bss_saved_ie(wl, P2PAPI_BSSCFG_DEVICE).p2p_probe_req_ie_len) {

		
		ack = wl_cfg80211_send_at_common_channel(wl, dev, af_params);
	} else {
		ack = (wl_cfgp2p_tx_action_frame(wl, dev, af_params, bssidx)) ? false : true;
		if (!ack) {
			u32 max_retry = WL_CHANNEL_SYNC_RETRY;

			if (wl_to_p2p_bss_saved_ie(wl, P2PAPI_BSSCFG_DEVICE).p2p_probe_req_ie_len) {
				if (act_frm && IS_P2P_SOCIAL(af_params->channel) &&
					!IS_P2P_PUB_ACT_RSP_SUBTYPE(act_frm->subtype)) {
					ack = wl_cfg80211_send_at_common_channel(wl, dev,
						af_params);
				} else {
					for (retry = 0; retry < max_retry; retry++) {
						WL_AF_TX_KEEP_PRI_CONNECTION_VSDB(wl);
						ack = (wl_cfgp2p_tx_action_frame(wl, dev,
							af_params, bssidx)) ? false : true;
						if (ack)
							break;
					}
				}
			} else {
				for (retry = 0; retry < max_retry; retry++) {
					WL_AF_TX_KEEP_PRI_CONNECTION_VSDB(wl);
					ack = (wl_cfgp2p_tx_action_frame(wl, dev,
						af_params, bssidx)) ? false : true;
					if (ack)
						break;
				}
			}
		}

	}
	
	wl_clr_drv_status(wl, SENDING_ACT_FRM, dev);

#ifdef WL_CFG80211_SYNC_GON
	if (ack && is_waiting_more_time &&
#ifdef WL_CFG80211_GON_COLLISION
		!wl->block_gon_req_tx_count &&
#endif 
		wl_get_drv_status_all(wl, WAITING_NEXT_ACT_FRM) &&
		wl->af_sent_channel == wl->afx_hdl->my_listen_chan) {
		s32 extar_listen_time;

		extar_listen_time =	af_params->dwell_time -
			jiffies_to_msecs(jiffies - wl->af_tx_sent_jiffies);

		if (extar_listen_time > 50) {
			wl_set_drv_status(wl, WAITING_NEXT_ACT_FRM_LISTEN, dev);
			WL_DBG(("Wait more time! actual af time:%d,"
				"calculated extar listen:%d\n",
				af_params->dwell_time, extar_listen_time));
			if (wl_cfgp2p_discover_listen(wl, wl->af_sent_channel,
				extar_listen_time + 100) == BCME_OK) {
				wait_for_completion_timeout(&wl->wait_next_af,
					msecs_to_jiffies(extar_listen_time + 100 + 300));
			}
			wl_clr_drv_status(wl, WAITING_NEXT_ACT_FRM_LISTEN, dev);
		}
	}
#endif 
	wl_clr_drv_status(wl, WAITING_NEXT_ACT_FRM, dev);

	WL_INFO(("-- sending Action Frame is %s, my listen chan: %d\n",
		(ack) ? "Succeeded!!":"Failed!!", wl->afx_hdl->my_listen_chan));

#ifdef WL_CFG80211_GON_COLLISION
	if (wl->block_gon_req_tx_count)
		ack = true;
#endif 
	cfg80211_mgmt_tx_status(ndev, *cookie, buf, len, ack, GFP_KERNEL);
	if (ack && act_frm && act_frm->subtype == P2P_PAF_GON_CONF) {
#ifdef FORCE_MPC
		if (!strstr(firmware_path, "_p2p"))
#endif
			wldev_iovar_setint(dev, "mpc", 1);
	}
	kfree(af_params);
exit:
	return err;
}


static void
wl_cfg80211_mgmt_frame_register(struct wiphy *wiphy, struct net_device *dev,
	u16 frame_type, bool reg)
{

	WL_DBG(("%s: frame_type: %x, reg: %d\n", __func__, frame_type, reg));

	if (frame_type != (IEEE80211_FTYPE_MGMT | IEEE80211_STYPE_PROBE_REQ))
		return;

	return;
}


static s32
wl_cfg80211_change_bss(struct wiphy *wiphy,
	struct net_device *dev,
	struct bss_parameters *params)
{
	if (params->use_cts_prot >= 0) {
	}

	if (params->use_short_preamble >= 0) {
	}

	if (params->use_short_slot_time >= 0) {
	}

	if (params->basic_rates) {
	}

	if (params->ap_isolate >= 0) {
	}

	if (params->ht_opmode >= 0) {
	}

	return 0;
}

static s32
wl_cfg80211_set_channel(struct wiphy *wiphy, struct net_device *dev,
	struct ieee80211_channel *chan,
	enum nl80211_channel_type channel_type)
{
	s32 _chan;
#ifdef HT40_GO
	s32 center_chan;
	chanspec_t chspec = 0;
#endif
	s32 err = BCME_OK;
	struct wl_priv *wl = wiphy_priv(wiphy);

	if (wl->p2p_net == dev) {
		dev = wl_to_prmry_ndev(wl);
	}

	if (!dev) {
		printf("%s: no dev!\n", __func__);
		dev = wl_to_prmry_ndev(wl);
	}

	_chan = ieee80211_frequency_to_channel(chan->center_freq);
	WL_ERR(("netdev_ifidx(%d), chan_type(%d) target channel(%d) \n",
		dev->ifindex, channel_type, _chan));

#ifdef NOT_YET
	switch (channel_type) {
		case NL80211_CHAN_HT40MINUS:
			
			chspec = CH40MHZ_CHSPEC(channel, WL_CHANSPEC_CTL_SB_UPPER);
			break;
		case NL80211_CHAN_HT40PLUS:
			
			chspec = CH40MHZ_CHSPEC(channel, WL_CHANSPEC_CTL_SB_LOWER);
			break;
		default:
			chspec = CH20MHZ_CHSPEC(channel);

	}
#endif 
#ifdef HT40_GO
	switch (_chan) {
		
		case 40:
		case 48:
		case 153:
		case 161:
			if (_chan <= (MAXCHANNEL - CH_20MHZ_APART))
				center_chan = _chan - CH_10MHZ_APART;
				chspec = CH40MHZ_CHSPEC(center_chan, WL_CHANSPEC_CTL_SB_UPPER);
			break;
		case 36:
		case 44:
		case 149:
		case 157:
			if (_chan <= (MAXCHANNEL - CH_20MHZ_APART))
				center_chan = _chan + CH_10MHZ_APART;
				chspec = CH40MHZ_CHSPEC(center_chan, WL_CHANSPEC_CTL_SB_LOWER);
			break;
		default:
			chspec = CH20MHZ_CHSPEC(_chan);
			break;
	}

	chspec = wl_chspec_host_to_driver(chspec);
	if ((err = wldev_iovar_setint(dev, "chanspec", chspec)) == BCME_BADCHAN) {
		err = wldev_ioctl(dev, WLC_SET_CHANNEL, &_chan, sizeof(_chan), true);
		if (err < 0) {
			WL_ERR(("WLC_SET_CHANNEL error %d"
				"chip may not be supporting this channel\n", err));
		}
	}
#else
	err = wldev_ioctl(dev, WLC_SET_CHANNEL, &_chan, sizeof(_chan), true);
	if (err < 0) {
		WL_ERR(("WLC_SET_CHANNEL error %d chip may not be supporting this channel\n", err));
	}
#endif 
#ifdef HT40_GO
	switch (_chan) {
		
		case 40:
		case 48:
		case 153:
		case 161:
			if (_chan <= (MAXCHANNEL - CH_20MHZ_APART))
				center_chan = _chan - CH_10MHZ_APART;
				chspec = CH40MHZ_CHSPEC(center_chan, WL_CHANSPEC_CTL_SB_UPPER);
			break;
		case 36:
		case 44:
		case 149:
		case 157:
			if (_chan <= (MAXCHANNEL - CH_20MHZ_APART))
				center_chan = _chan + CH_10MHZ_APART;
				chspec = CH40MHZ_CHSPEC(center_chan, WL_CHANSPEC_CTL_SB_LOWER);
			break;
		default:
			chspec = CH20MHZ_CHSPEC(_chan);
			break;
	}

	chspec = wl_chspec_host_to_driver(chspec);
	if ((err = wldev_iovar_setint(dev, "chanspec", chspec)) == BCME_BADCHAN) {
		err = wldev_ioctl(dev, WLC_SET_CHANNEL, &_chan, sizeof(_chan), true);
		if (err < 0) {
			WL_ERR(("WLC_SET_CHANNEL error %d"
				"chip may not be supporting this channel\n", err));
		}
	}
#else
	err = wldev_ioctl(dev, WLC_SET_CHANNEL, &_chan, sizeof(_chan), true);
	if (err < 0) {
		WL_ERR(("WLC_SET_CHANNEL error %d"
			"chip may not be supporting this channel\n", err));
	}
#endif 
	return err;
}

#if 0
static s32
wl_validate_opensecurity(struct net_device *dev, s32 bssidx)
{
	s32 err = BCME_OK;

	
	err = wldev_iovar_setint_bsscfg(dev, "auth", 0, bssidx);
	if (err < 0) {
		WL_ERR(("auth error %d\n", err));
		return BCME_ERROR;
	}
	
	err = wldev_iovar_setint_bsscfg(dev, "wsec", 0, bssidx);
	if (err < 0) {
		WL_ERR(("wsec error %d\n", err));
		return BCME_ERROR;
	}
	
	err = wldev_iovar_setint_bsscfg(dev, "wpa_auth", WPA_AUTH_NONE, bssidx);
	if (err < 0) {
		WL_ERR(("wpa_auth error %d\n", err));
		return BCME_ERROR;
	}

	return 0;
}
#endif

static s32
wl_validate_wpa2ie(struct net_device *dev, bcm_tlv_t *wpa2ie, s32 bssidx)
{
	s32 len = 0;
	s32 err = BCME_OK;
	u16 auth = 0; 
	u32 wsec;
	u32 pval = 0;
	u32 gval = 0;
	u32 wpa_auth = 0;
	wpa_suite_mcast_t *mcast;
	wpa_suite_ucast_t *ucast;
	wpa_suite_auth_key_mgmt_t *mgmt;
	u16 suite_count;
	u8 rsn_cap[2];
	u32 wme_bss_disable;
	if (wpa2ie == NULL)
		goto exit;

	WL_DBG(("Enter \n"));
	len =  wpa2ie->len;
	
	mcast = (wpa_suite_mcast_t *)&wpa2ie->data[WPA2_VERSION_LEN];
	switch (mcast->type) {
		case WPA_CIPHER_NONE:
			gval = 0;
			break;
		case WPA_CIPHER_WEP_40:
		case WPA_CIPHER_WEP_104:
			gval = WEP_ENABLED;
			break;
		case WPA_CIPHER_TKIP:
			gval = TKIP_ENABLED;
			break;
		case WPA_CIPHER_AES_CCM:
			gval = AES_ENABLED;
			break;
#ifdef BCMWAPI_WPI
		case WAPI_CIPHER_SMS4:
			gval = SMS4_ENABLED;
			break;
#endif
		default:
			WL_ERR(("No Security Info\n"));
			break;
	}
	if ((len -= WPA_SUITE_LEN) <= 0)
		return BCME_BADLEN;
	
	ucast = (wpa_suite_ucast_t *)&mcast[1];
	suite_count = ltoh16_ua(&ucast->count);
	switch (ucast->list[0].type) {
		case WPA_CIPHER_NONE:
			pval = 0;
			break;
		case WPA_CIPHER_WEP_40:
		case WPA_CIPHER_WEP_104:
			pval = WEP_ENABLED;
			break;
		case WPA_CIPHER_TKIP:
			pval = TKIP_ENABLED;
			break;
		case WPA_CIPHER_AES_CCM:
			pval = AES_ENABLED;
			break;
#ifdef BCMWAPI_WPI
		case WAPI_CIPHER_SMS4:
			pval = SMS4_ENABLED;
			break;
#endif
		default:
			WL_ERR(("No Security Info\n"));
	}
	if ((len -= (WPA_IE_SUITE_COUNT_LEN + (WPA_SUITE_LEN * suite_count))) <= 0)
		return BCME_BADLEN;

	
	wsec = (pval | gval | SES_OW_ENABLED);
	
	mgmt = (wpa_suite_auth_key_mgmt_t *)&ucast->list[suite_count];
	suite_count = ltoh16_ua(&mgmt->count);
	switch (mgmt->list[0].type) {
		case RSN_AKM_NONE:
			wpa_auth = WPA_AUTH_NONE;
			break;
		case RSN_AKM_UNSPECIFIED:
			wpa_auth = WPA2_AUTH_UNSPECIFIED;
			break;
		case RSN_AKM_PSK:
			wpa_auth = WPA2_AUTH_PSK;
			break;
		default:
			WL_ERR(("No Key Mgmt Info\n"));
	}

	if ((len -= (WPA_IE_SUITE_COUNT_LEN + (WPA_SUITE_LEN * suite_count))) >= RSN_CAP_LEN) {
		rsn_cap[0] = *(u8 *)&mgmt->list[suite_count];
		rsn_cap[1] = *((u8 *)&mgmt->list[suite_count] + 1);

		if (rsn_cap[0] & (RSN_CAP_16_REPLAY_CNTRS << RSN_CAP_PTK_REPLAY_CNTR_SHIFT)) {
			wme_bss_disable = 0;
		} else {
			wme_bss_disable = 1;
		}

		
		err = wldev_iovar_setint_bsscfg(dev, "wme_bss_disable", wme_bss_disable, bssidx);
		if (err < 0) {
			WL_ERR(("wme_bss_disable error %d\n", err));
			return BCME_ERROR;
		}
	} else {
		WL_DBG(("There is no RSN Capabilities. remained len %d\n", len));
	}

	
	err = wldev_iovar_setint_bsscfg(dev, "auth", auth, bssidx);
	if (err < 0) {
		WL_ERR(("auth error %d\n", err));
		return BCME_ERROR;
	}
	
	err = wldev_iovar_setint_bsscfg(dev, "wsec", wsec, bssidx);
	if (err < 0) {
		WL_ERR(("wsec error %d\n", err));
		return BCME_ERROR;
	}
	
	err = wldev_iovar_setint_bsscfg(dev, "wpa_auth", wpa_auth, bssidx);
	if (err < 0) {
		WL_ERR(("wpa_auth error %d\n", err));
		return BCME_ERROR;
	}
exit:
	return 0;
}

static s32
wl_validate_wpaie(struct net_device *dev, wpa_ie_fixed_t *wpaie, s32 bssidx)
{
	wpa_suite_mcast_t *mcast;
	wpa_suite_ucast_t *ucast;
	wpa_suite_auth_key_mgmt_t *mgmt;
	u16 auth = 0; 
	u16 count;
	s32 err = BCME_OK;
	s32 len = 0;
	u32 i;
	u32 wsec;
	u32 pval = 0;
	u32 gval = 0;
	u32 wpa_auth = 0;
	u32 tmp = 0;

	if (wpaie == NULL)
		goto exit;
	WL_DBG(("Enter \n"));
	len = wpaie->length;    
	len -= WPA_IE_TAG_FIXED_LEN;
	
	if (len < WPA_SUITE_LEN) {
		WL_INFO(("no multicast cipher suite\n"));
		goto exit;
	}

	
	mcast = (wpa_suite_mcast_t *)&wpaie[1];
	len -= WPA_SUITE_LEN;
	if (!bcmp(mcast->oui, WPA_OUI, WPA_OUI_LEN)) {
		if (IS_WPA_CIPHER(mcast->type)) {
			tmp = 0;
			switch (mcast->type) {
				case WPA_CIPHER_NONE:
					tmp = 0;
					break;
				case WPA_CIPHER_WEP_40:
				case WPA_CIPHER_WEP_104:
					tmp = WEP_ENABLED;
					break;
				case WPA_CIPHER_TKIP:
					tmp = TKIP_ENABLED;
					break;
				case WPA_CIPHER_AES_CCM:
					tmp = AES_ENABLED;
					break;
				default:
					WL_ERR(("No Security Info\n"));
			}
			gval |= tmp;
		}
	}
	
	if (len < WPA_IE_SUITE_COUNT_LEN) {
		WL_INFO(("no unicast suite\n"));
		goto exit;
	}
	
	ucast = (wpa_suite_ucast_t *)&mcast[1];
	count = ltoh16_ua(&ucast->count);
	len -= WPA_IE_SUITE_COUNT_LEN;
	for (i = 0; i < count && len >= WPA_SUITE_LEN;
		i++, len -= WPA_SUITE_LEN) {
		if (!bcmp(ucast->list[i].oui, WPA_OUI, WPA_OUI_LEN)) {
			if (IS_WPA_CIPHER(ucast->list[i].type)) {
				tmp = 0;
				switch (ucast->list[i].type) {
					case WPA_CIPHER_NONE:
						tmp = 0;
						break;
					case WPA_CIPHER_WEP_40:
					case WPA_CIPHER_WEP_104:
						tmp = WEP_ENABLED;
						break;
					case WPA_CIPHER_TKIP:
						tmp = TKIP_ENABLED;
						break;
					case WPA_CIPHER_AES_CCM:
						tmp = AES_ENABLED;
						break;
					default:
						WL_ERR(("No Security Info\n"));
				}
				pval |= tmp;
			}
		}
	}
	len -= (count - i) * WPA_SUITE_LEN;
	
	if (len < WPA_IE_SUITE_COUNT_LEN) {
		WL_INFO((" no auth key mgmt suite\n"));
		goto exit;
	}
	
	mgmt = (wpa_suite_auth_key_mgmt_t *)&ucast->list[count];
	count = ltoh16_ua(&mgmt->count);
	len -= WPA_IE_SUITE_COUNT_LEN;
	for (i = 0; i < count && len >= WPA_SUITE_LEN;
		i++, len -= WPA_SUITE_LEN) {
		if (!bcmp(mgmt->list[i].oui, WPA_OUI, WPA_OUI_LEN)) {
			if (IS_WPA_AKM(mgmt->list[i].type)) {
				tmp = 0;
				switch (mgmt->list[i].type) {
					case RSN_AKM_NONE:
						tmp = WPA_AUTH_NONE;
						break;
					case RSN_AKM_UNSPECIFIED:
						tmp = WPA_AUTH_UNSPECIFIED;
						break;
					case RSN_AKM_PSK:
						tmp = WPA_AUTH_PSK;
						break;
					default:
						WL_ERR(("No Key Mgmt Info\n"));
				}
				wpa_auth |= tmp;
			}
		}

	}
	
	wsec = (pval | gval | SES_OW_ENABLED);
	
	err = wldev_iovar_setint_bsscfg(dev, "auth", auth, bssidx);
	if (err < 0) {
		WL_ERR(("auth error %d\n", err));
		return BCME_ERROR;
	}
	
	err = wldev_iovar_setint_bsscfg(dev, "wsec", wsec, bssidx);
	if (err < 0) {
		WL_ERR(("wsec error %d\n", err));
		return BCME_ERROR;
	}
	
	err = wldev_iovar_setint_bsscfg(dev, "wpa_auth", wpa_auth, bssidx);
	if (err < 0) {
		WL_ERR(("wpa_auth error %d\n", err));
		return BCME_ERROR;
	}
exit:
	return 0;
}

#define default_ssid "HTC Portable Hotspot"

#if 0
static s32
wl_cfg80211_bcn_validate_sec(
	struct net_device *dev,
	struct parsed_ies *ies,
	u32 dev_role,
	s32 bssidx)
{
	struct wl_priv *wl = wlcfg_drv_priv;

	if (dev_role == NL80211_IFTYPE_P2P_GO && (ies->wpa2_ie)) {
		
		WL_DBG(("P2P GO: validating wpa2_ie"));
		if (wl_validate_wpa2ie(dev, ies->wpa2_ie, bssidx)  < 0)
			return BCME_ERROR;

	} else if (dev_role == NL80211_IFTYPE_AP) {

		WL_DBG(("SoftAP: validating security"));
		
		if ((ies->wpa2_ie || ies->wpa_ie) &&
			((wl_validate_wpa2ie(dev, ies->wpa2_ie, bssidx)  < 0 ||
			wl_validate_wpaie(dev, ies->wpa_ie, bssidx) < 0))) {
			wl->ap_info->security_mode = false;
			return BCME_ERROR;
		}

		wl->ap_info->security_mode = true;
		if (wl->ap_info->rsn_ie) {
			kfree(wl->ap_info->rsn_ie);
			wl->ap_info->rsn_ie = NULL;
		}
		if (wl->ap_info->wpa_ie) {
			kfree(wl->ap_info->wpa_ie);
			wl->ap_info->wpa_ie = NULL;
		}
		if (wl->ap_info->wps_ie) {
			kfree(wl->ap_info->wps_ie);
			wl->ap_info->wps_ie = NULL;
		}
		if (ies->wpa_ie != NULL) {
			
			wl->ap_info->rsn_ie = NULL;
			wl->ap_info->wpa_ie = kmemdup(ies->wpa_ie,
				ies->wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN,
				GFP_KERNEL);
		} else if (ies->wpa2_ie != NULL) {
			
			wl->ap_info->wpa_ie = NULL;
			wl->ap_info->rsn_ie = kmemdup(ies->wpa2_ie,
				ies->wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN,
				GFP_KERNEL);
		}

		if (!ies->wpa2_ie && !ies->wpa_ie) {
#if 0
			wl_validate_opensecurity(dev, bssidx);
#endif
			wl->ap_info->security_mode = false;
		}

		if (ies->wps_ie) {
			wl->ap_info->wps_ie = kmemdup(ies->wps_ie, ies->wps_ie_len, GFP_KERNEL);
		}
	}

	return 0;

}

static s32 wl_cfg80211_bcn_set_params(
	struct cfg80211_ap_settings *info,
	struct net_device *dev,
	u32 dev_role, s32 bssidx)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	s32 err = BCME_OK;

	WL_DBG(("interval (%d) \ndtim_period (%d) \n",
		info->beacon_interval, info->dtim_period));

	if (info->beacon_interval) {
		if ((err = wldev_ioctl(dev, WLC_SET_BCNPRD,
			&info->beacon_interval, sizeof(s32), true)) < 0) {
			WL_ERR(("Beacon Interval Set Error, %d\n", err));
			return err;
		}
	}

	if (info->dtim_period) {
		if ((err = wldev_ioctl(dev, WLC_SET_DTIMPRD,
			&info->dtim_period, sizeof(s32), true)) < 0) {
			WL_ERR(("DTIM Interval Set Error, %d\n", err));
			return err;
		}
	}

	if ((info->ssid) && (info->ssid_len > 0) &&
		(info->ssid_len <= 32)) {
		WL_DBG(("SSID (%s) len:%d \n", info->ssid, info->ssid_len));
		if (dev_role == NL80211_IFTYPE_AP) {
			
			memset(wl->hostapd_ssid.SSID, 0x00, 32);
			memcpy(wl->hostapd_ssid.SSID, info->ssid, info->ssid_len);
			wl->hostapd_ssid.SSID_len = info->ssid_len;
		} else {
				
			memset(wl->p2p->ssid.SSID, 0x00, 32);
			memcpy(wl->p2p->ssid.SSID, info->ssid, info->ssid_len);
			wl->p2p->ssid.SSID_len = info->ssid_len;
		}
	}

	if (info->hidden_ssid) {
		WL_DBG(("hidden_ssid_enum_val: %d \n", info->hidden_ssid));
	}

	return err;
}

static s32
wl_cfg80211_parse_ies(u8 *ptr, u32 len, struct parsed_ies *ies)
{
	s32 err = BCME_OK;

	memset(ies, 0, sizeof(struct parsed_ies));

	
	if ((ies->wps_ie = wl_cfgp2p_find_wpsie(ptr, len)) != NULL) {
		WL_DBG(("WPSIE in beacon \n"));
		ies->wps_ie_len = ies->wps_ie->length + WPA_RSN_IE_TAG_FIXED_LEN;
	} else {
		WL_ERR(("No WPSIE in beacon \n"));
	}

	
	if ((ies->wpa2_ie = bcm_parse_tlvs(ptr, len,
		DOT11_MNG_RSN_ID)) != NULL) {
		WL_DBG((" WPA2 IE found\n"));
		ies->wpa2_ie_len = ies->wpa2_ie->len;
	}

	
	if ((ies->wpa_ie = wl_cfgp2p_find_wpaie(ptr, len)) != NULL) {
		WL_DBG((" WPA found\n"));
		ies->wpa_ie_len = ies->wpa_ie->length;
	}

	return err;

}

static s32
wl_cfg80211_bcn_bringup_ap(
	struct net_device *dev,
	struct parsed_ies *ies,
	u32 dev_role, s32 bssidx)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct wl_join_params join_params;
	bool is_bssup = false;
	s32 infra = 1;
	s32 join_params_size = 0;
	s32 ap = 1;
	s32 err = BCME_OK;

	WL_DBG(("Enter dev_role: %d\n", dev_role));

	
	wldev_iovar_setint(dev, "mpc", 0);

	if (dev_role == NL80211_IFTYPE_P2P_GO) {
		is_bssup = wl_cfgp2p_bss_isup(dev, bssidx);
		if (!is_bssup && (ies->wpa2_ie != NULL)) {

			err = wldev_ioctl(dev, WLC_SET_INFRA, &infra, sizeof(s32), true);
			if (err < 0) {
				WL_ERR(("SET INFRA error %d\n", err));
				goto exit;
			}

			err = wldev_iovar_setbuf_bsscfg(dev, "ssid", &wl->p2p->ssid,
				sizeof(wl->p2p->ssid), wl->ioctl_buf, WLC_IOCTL_MAXLEN,
				bssidx, &wl->ioctl_buf_sync);
			if (err < 0) {
				WL_ERR(("GO SSID setting error %d\n", err));
				goto exit;
			}

			if ((err = wl_cfgp2p_bss(wl, dev, bssidx, 1)) < 0) {
				WL_ERR(("GO Bring up error %d\n", err));
				goto exit;
			}
		} else
			WL_DBG(("Bss is already up\n"));
	} else if ((dev_role == NL80211_IFTYPE_AP) &&
		(wl_get_drv_status(wl, AP_CREATING, dev))) {
		
		err = wldev_ioctl(dev, WLC_DOWN, &ap, sizeof(s32), true);
		if (err < 0) {
			WL_ERR(("WLC_DOWN error %d\n", err));
			goto exit;
		}
		err = wldev_ioctl(dev, WLC_SET_INFRA, &infra, sizeof(s32), true);
		if (err < 0) {
			WL_ERR(("SET INFRA error %d\n", err));
			goto exit;
		}
		if ((err = wldev_ioctl(dev, WLC_SET_AP, &ap, sizeof(s32), true)) < 0) {
			WL_ERR(("setting AP mode failed %d \n", err));
			goto exit;
		}

		err = wldev_ioctl(dev, WLC_UP, &ap, sizeof(s32), true);
		if (unlikely(err)) {
			WL_ERR(("WLC_UP error (%d)\n", err));
			goto exit;
		}

		memset(&join_params, 0, sizeof(join_params));
		
		join_params_size = sizeof(join_params.ssid);
		memcpy(join_params.ssid.SSID, wl->hostapd_ssid.SSID,
			wl->hostapd_ssid.SSID_len);
		join_params.ssid.SSID_len = htod32(wl->hostapd_ssid.SSID_len);

		
		if ((err = wldev_ioctl(dev, WLC_SET_SSID, &join_params,
			join_params_size, true)) == 0) {
			WL_DBG(("SoftAP set SSID (%s) success\n", join_params.ssid.SSID));
			wl_clr_drv_status(wl, AP_CREATING, dev);
			wl_set_drv_status(wl, AP_CREATED, dev);
		}
	}


exit:
	return err;
}

s32
wl_cfg80211_parse_set_ies(
	struct net_device *dev,
	struct cfg80211_beacon_data *info,
	struct parsed_ies *ies,
	u32 dev_role,
	s32 bssidx)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct parsed_ies prb_ies;
	s32 err = BCME_OK;

	memset(ies, 0, sizeof(struct parsed_ies));
	memset(&prb_ies, 0, sizeof(struct parsed_ies));

	
	if (wl_cfg80211_parse_ies((u8 *)info->tail,
		info->tail_len, ies) < 0) {
		WL_ERR(("Beacon get IEs failed \n"));
		err = -EINVAL;
		goto fail;
	}

	
	if ((err = wl_cfgp2p_set_management_ie(wl, dev, bssidx,
		VNDR_IE_BEACON_FLAG, (u8 *)info->tail,
		info->tail_len)) < 0) {
		WL_ERR(("Set Beacon IE Failed \n"));
	} else {
		WL_DBG(("Applied Vndr IEs for Beacon \n"));
	}

	
	if (wl_cfg80211_parse_ies((u8 *)info->proberesp_ies,
		info->proberesp_ies_len, &prb_ies) < 0) {
		WL_ERR(("PRB RESP get IEs failed \n"));
		err = -EINVAL;
		goto fail;
	}

	
	if ((err = wl_cfgp2p_set_management_ie(wl, dev, bssidx,
		VNDR_IE_PRBRSP_FLAG, (u8 *)info->proberesp_ies,
		info->proberesp_ies_len)) < 0) {
		WL_ERR(("Set Probe Resp IE Failed \n"));
	} else {
		WL_DBG(("Applied Vndr IEs for Probe Resp \n"));
	}

fail:

	return err;
}

static s32 wl_cfg80211_hostapd_sec(
	struct net_device *dev,
	struct parsed_ies *ies,
	s32 bssidx)
{
	bool update_bss = 0;
	struct wl_priv *wl = wlcfg_drv_priv;


	if (ies->wps_ie) {
		if (wl->ap_info->wps_ie &&
			memcmp(wl->ap_info->wps_ie, ies->wps_ie, ies->wps_ie_len)) {
			WL_DBG((" WPS IE is changed\n"));
			kfree(wl->ap_info->wps_ie);
			wl->ap_info->wps_ie = kmemdup(ies->wps_ie, ies->wps_ie_len, GFP_KERNEL);
		} else if (wl->ap_info->wps_ie == NULL) {
			WL_DBG((" WPS IE is added\n"));
			wl->ap_info->wps_ie = kmemdup(ies->wps_ie, ies->wps_ie_len, GFP_KERNEL);
		}
		if ((ies->wpa_ie != NULL || ies->wpa2_ie != NULL)) {
			if (!wl->ap_info->security_mode) {
				
				update_bss = true;
				if (ies->wpa_ie != NULL) {
					wl->ap_info->wpa_ie = kmemdup(ies->wpa_ie,
					ies->wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN,
					GFP_KERNEL);
				} else {
					wl->ap_info->rsn_ie = kmemdup(ies->wpa2_ie,
					ies->wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN,
					GFP_KERNEL);
				}
			} else if (wl->ap_info->wpa_ie) {
				
				if (ies->wpa_ie != NULL) {
					update_bss = true;
					kfree(wl->ap_info->rsn_ie);
					wl->ap_info->rsn_ie = NULL;
					wl->ap_info->wpa_ie = kmemdup(ies->wpa_ie,
					ies->wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN,
					GFP_KERNEL);
				} else if (memcmp(wl->ap_info->rsn_ie,
					ies->wpa2_ie, ies->wpa2_ie->len
					+ WPA_RSN_IE_TAG_FIXED_LEN)) {
					update_bss = true;
					kfree(wl->ap_info->rsn_ie);
					wl->ap_info->rsn_ie = kmemdup(ies->wpa2_ie,
					ies->wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN,
					GFP_KERNEL);
					wl->ap_info->wpa_ie = NULL;
				}
			}
			if (update_bss) {
				wl->ap_info->security_mode = true;
				wl_cfgp2p_bss(wl, dev, bssidx, 0);
				if (wl_validate_wpa2ie(dev, ies->wpa2_ie, bssidx)  < 0 ||
					wl_validate_wpaie(dev, ies->wpa_ie, bssidx) < 0) {
					return BCME_ERROR;
				}
				wl_cfgp2p_bss(wl, dev, bssidx, 1);
			}
		}
	} else {
		WL_ERR(("No WPSIE in beacon \n"));
	}
	return 0;
}

static s32
wl_cfg80211_start_ap(
	struct wiphy *wiphy,
	struct net_device *dev,
	struct cfg80211_ap_settings *info)
{
	struct wl_priv *wl = wiphy_priv(wiphy);
	s32 err = BCME_OK;
	struct parsed_ies ies;
	s32 bssidx = 0;
	u32 dev_role = 0;

	WL_DBG(("Enter \n"));
	if (dev == wl_to_prmry_ndev(wl)) {
		WL_DBG(("Start AP req on primary iface: Softap\n"));
		dev_role = NL80211_IFTYPE_AP;
	} else if (dev == wl->p2p_net) {
		
		WL_DBG(("Start AP req on P2P iface: GO\n"));
		dev = wl_to_prmry_ndev(wl);
		dev_role = NL80211_IFTYPE_P2P_GO;
	}

	bssidx = wl_cfgp2p_find_idx(wl, dev);
	if (p2p_is_on(wl) &&
		(bssidx == wl_to_p2p_bss_bssidx(wl,
		P2PAPI_BSSCFG_CONNECTION))) {
		dev_role = NL80211_IFTYPE_P2P_GO;
		WL_DBG(("Start AP req on P2P connection iface\n"));
	}

	if ((err = wl_cfg80211_bcn_set_params(info, dev,
		dev_role, bssidx)) < 0) {
		WL_ERR(("Beacon params set failed \n"));
		goto fail;
	}

	
	if ((err = wl_cfg80211_parse_set_ies(dev, &info->beacon,
		&ies, dev_role, bssidx) < 0)) {
		WL_ERR(("Set IEs failed \n"));
		goto fail;
	}

	if ((wl_cfg80211_bcn_validate_sec(dev, &ies,
		dev_role, bssidx)) < 0)
	{
		WL_ERR(("Beacon set security failed \n"));
		goto fail;
	}

	if ((err = wl_cfg80211_bcn_bringup_ap(dev, &ies,
		dev_role, bssidx)) < 0) {
		WL_ERR(("Beacon bring up AP/GO failed \n"));
		goto fail;
	}

	WL_DBG(("** AP/GO Created **\n"));

fail:
	if (err) {
		WL_ERR(("ADD/SET beacon failed\n"));
		wldev_iovar_setint(dev, "mpc", 1);
	}

	return err;
}

static s32
wl_cfg80211_stop_ap(
	struct wiphy *wiphy,
	struct net_device *dev)
{
	int err = 0;
	u32 dev_role = 0;
	int infra = 0;
	int ap = 0;
	s32 bssidx = 0;
	struct wl_priv *wl = wiphy_priv(wiphy);

	WL_DBG(("Enter \n"));
	if (dev == wl_to_prmry_ndev(wl)) {
		dev_role = NL80211_IFTYPE_AP;
	} else if (dev == wl->p2p_net) {
		
		dev = wl_to_prmry_ndev(wl);
		dev_role = NL80211_IFTYPE_P2P_GO;
	}
	bssidx = wl_cfgp2p_find_idx(wl, dev);
	if (p2p_is_on(wl) &&
		(bssidx == wl_to_p2p_bss_bssidx(wl,
		P2PAPI_BSSCFG_CONNECTION))) {
		dev_role = NL80211_IFTYPE_P2P_GO;
	}

	if (dev_role == NL80211_IFTYPE_AP) {
		err = wldev_ioctl(dev, WLC_SET_INFRA, &infra, sizeof(s32), true);
		if (err < 0) {
			WL_ERR(("SET INFRA error %d\n", err));
			err = -ENOTSUPP;
			goto exit;
		}
		if ((err = wldev_ioctl(dev, WLC_SET_AP, &ap, sizeof(s32), true)) < 0) {
			WL_ERR(("setting AP mode failed %d \n", err));
			err = -ENOTSUPP;
			goto exit;
		}

		err = wldev_ioctl(dev, WLC_UP, &ap, sizeof(s32), true);
		if (unlikely(err)) {
			WL_ERR(("WLC_UP error (%d)\n", err));
			err = -EINVAL;
			goto exit;
		}

		wl_clr_drv_status(wl, AP_CREATED, dev);
		
		wldev_iovar_setint(dev, "mpc", 1);
	} else {
		WL_DBG(("Stopping P2P GO \n"));
	}

exit:
	return err;
}

static s32
wl_cfg80211_change_beacon(
	struct wiphy *wiphy,
	struct net_device *dev,
	struct cfg80211_beacon_data *info)
{
	s32 err = BCME_OK;
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct parsed_ies ies;
	u32 dev_role = 0;
	s32 bssidx = 0;

	WL_DBG(("Enter \n"));

	if (dev == wl_to_prmry_ndev(wl)) {
		dev_role = NL80211_IFTYPE_AP;
	} else if (dev == wl->p2p_net) {
		
		dev = wl_to_prmry_ndev(wl);
		dev_role = NL80211_IFTYPE_P2P_GO;
	}

	bssidx = wl_cfgp2p_find_idx(wl, dev);
	if (p2p_is_on(wl) &&
		(bssidx == wl_to_p2p_bss_bssidx(wl,
		P2PAPI_BSSCFG_CONNECTION))) {
		dev_role = NL80211_IFTYPE_P2P_GO;
	}

	
	if ((err = wl_cfg80211_parse_set_ies(dev, info,
		&ies, dev_role, bssidx) < 0)) {
		WL_ERR(("Set IEs failed \n"));
		goto fail;
	}

	if (dev_role == NL80211_IFTYPE_AP) {
		if (wl_cfg80211_hostapd_sec(dev, &ies, bssidx) < 0) {
			WL_ERR(("Hostapd update sec failed \n"));
			err = -EINVAL;
			goto fail;
		}
	}

fail:
	return err;
}
#else 
static int used_default_ssid = 0;

static s32
wl_cfg80211_add_set_beacon(struct wiphy *wiphy, struct net_device *dev,
	struct beacon_parameters *info)
{
	s32 err = BCME_OK;
	bcm_tlv_t *ssid_ie;
	wlc_ssid_t ssid;
	struct wl_priv *wl = wiphy_priv(wiphy);
	struct wl_join_params join_params;
	wpa_ie_fixed_t *wps_ie;
	wpa_ie_fixed_t *wpa_ie;
	bcm_tlv_t *wpa2_ie;
#ifdef BCMWAPI_WPI
	bcm_tlv_t *wapi_ie;
#endif
	wifi_p2p_ie_t *p2p_ie;
	wifi_wfd_ie_t *wfd_ie;
	bool is_bssup = false;
	bool update_bss = false;
	bool pbc = false;
	u16 wpsie_len = 0;
	u16 p2pie_len = 0;
	u32 wfdie_len = 0;
	u8 beacon_ie[IE_MAX_LEN];
	s32 ie_offset = 0;
	s32 bssidx = 0;
	s32 infra = 1;
	s32 join_params_size = 0;
	s32 ap = 0;
	WL_DBG(("interval (%d) dtim_period (%d) head_len (%d) tail_len (%d)\n",
		info->interval, info->dtim_period, info->head_len, info->tail_len));

	if (wl->p2p_net == dev) {
		dev = wl_to_prmry_ndev(wl);
	}

	bssidx = wl_cfgp2p_find_idx(wl, dev);
	if (p2p_is_on(wl) &&
		(bssidx == wl_to_p2p_bss_bssidx(wl,
		P2PAPI_BSSCFG_CONNECTION))) {
		memset(beacon_ie, 0, sizeof(beacon_ie));
		ie_offset = DOT11_MGMT_HDR_LEN + DOT11_BCN_PRB_FIXED_LEN;
		
		if ((ssid_ie = bcm_parse_tlvs((u8 *)&info->head[ie_offset],
			info->head_len - ie_offset,
			DOT11_MNG_SSID_ID)) != NULL) {
			memcpy(wl->p2p->ssid.SSID, ssid_ie->data, ssid_ie->len);
			wl->p2p->ssid.SSID_len = ssid_ie->len;
			WL_DBG(("SSID (%s) in Head \n", ssid_ie->data));

		} else {
			WL_ERR(("No SSID in beacon \n"));
		}

		
		if ((wps_ie = wl_cfgp2p_find_wpsie((u8 *)info->tail, info->tail_len)) != NULL) {
			wpsie_len = wps_ie->length + WPA_RSN_IE_TAG_FIXED_LEN;
			wl_validate_wps_ie((char *) wps_ie, &pbc);
			memcpy(beacon_ie, wps_ie, wpsie_len);
		} else {
			WL_ERR(("No WPSIE in beacon \n"));
		}


		
		if ((p2p_ie = wl_cfgp2p_find_p2pie((u8 *)info->tail, info->tail_len)) != NULL) {
			
			p2pie_len = p2p_ie->len + sizeof(p2p_ie->len) + sizeof(p2p_ie->id);
			memcpy(&beacon_ie[wpsie_len], p2p_ie, p2pie_len);

		} else {
			WL_ERR(("No P2PIE in beacon \n"));
		}

		
		if ((wfd_ie = wl_cfgp2p_find_wfdie((u8 *)info->tail, info->tail_len)) != NULL) {
			
			wfdie_len = wfd_ie->len + sizeof(wfd_ie->len) + sizeof(wfd_ie->id);
			if ((wpsie_len + p2pie_len + wfdie_len) < IE_MAX_LEN) {
				memcpy(&beacon_ie[wpsie_len + p2pie_len], wfd_ie, wfdie_len);
			} else {
				WL_ERR(("Found WFD IE but there is no space, (%d)(%d)(%d)\n",
					wpsie_len, p2pie_len, wfdie_len));
				wfdie_len = 0;
			}
		} else {
			WL_ERR(("No WFDIE in beacon \n"));
		}
		
		wl_add_remove_eventmsg(dev, WLC_E_PROBREQ_MSG, pbc);
		wl_cfgp2p_set_management_ie(wl, dev, bssidx, VNDR_IE_BEACON_FLAG,
			beacon_ie, wpsie_len + p2pie_len);
		
		if ((wpa_ie = wl_cfgp2p_find_wpaie((u8 *)info->tail, info->tail_len)) != NULL) {
			WL_DBG((" WPA IE is found\n"));
		}

		
		if ((wpa2_ie = bcm_parse_tlvs((u8 *)info->tail, info->tail_len,
			DOT11_MNG_RSN_ID)) != NULL) {
			WL_DBG((" WPA2 IE is found\n"));
		}
#ifdef BCMWAPI_WPI
		if ((wapi_ie = bcm_parse_tlvs((u8 *)info->tail, info->tail_len,
			DOT11_MNG_WAPI_ID)) != NULL) {
			WL_DBG(("%s: found a WAPI IE...\n", __FUNCTION__));
		}
#endif
		is_bssup = wl_cfgp2p_bss_isup(dev, bssidx);

		if (!is_bssup && (wpa_ie != NULL || wpa2_ie != NULL
#ifdef BCMWAPI_WPI
			|| wapi_ie != NULL
#endif
		)) {
			wldev_iovar_setint(dev, "mpc", 0);
			if ((err = wl_validate_wpa2ie(dev, wpa2_ie, bssidx)) < 0) {
				WL_ERR(("WPA2 IE parsing error"));
				goto exit;
			}
			err = wldev_ioctl(dev, WLC_SET_INFRA, &infra, sizeof(s32), true);
			if (err < 0) {
				WL_ERR(("SET INFRA error %d\n", err));
				goto exit;
			}
			err = wldev_iovar_setbuf_bsscfg(dev, "ssid", &wl->p2p->ssid,
				sizeof(wl->p2p->ssid), wl->ioctl_buf, WLC_IOCTL_MAXLEN,
				bssidx, &wl->ioctl_buf_sync);
			if (err < 0) {
				WL_ERR(("GO SSID setting error %d\n", err));
				goto exit;
			}
			if ((err = wl_cfgp2p_bss(wl, dev, bssidx, 1)) < 0) {
				WL_ERR(("GO Bring up error %d\n", err));
				goto exit;
			}
		}
	} else if (wl_get_drv_status(wl, AP_CREATING, dev)) {
		ie_offset = DOT11_MGMT_HDR_LEN + DOT11_BCN_PRB_FIXED_LEN;
		ap = 1;
		
		if ((ssid_ie = bcm_parse_tlvs((u8 *)&info->head[ie_offset],
			info->head_len - ie_offset,
			DOT11_MNG_SSID_ID)) != NULL) {
			
			int closed = 0;
			int max_assoc = 8;
			int prslmt = 15;
			bcm_tlv_t *max_assoc_ie;

			if (ssid_ie->len == 0  && ssid_ie->data[0] == DOT11_MNG_SSID_ID) {
				printf("%s: search SSID again to get real SSID.\n", __func__);
				ssid_ie = bcm_parse_tlvs((u8 *)ssid_ie->data,
							info->head_len - ie_offset - 1,
							DOT11_MNG_SSID_ID);
				if (ssid_ie != NULL)
					printf("%s: got real ssid, use it instead. ssid_ie->len=%d\n", __func__, ssid_ie->len);
				else
					printf("%s: so sad. no real ssid found.\n", __func__);
				wl_iw_send_priv_event(dev,"NULL_SSID_IE");
				used_default_ssid = 1;
				closed = 1;
			}else if(ssid_ie->len == 0){
				printf("%s: so sad!!!!. no real ssid found.\n", __func__);
				wl_iw_send_priv_event(dev,"NULL_SSID_IE");
				used_default_ssid = 1;
			}
			
			memset(&ssid, 0, sizeof(wlc_ssid_t)); 
			
			if(ssid_ie != NULL && ssid_ie->len){
				memcpy(ssid.SSID, ssid_ie->data, ssid_ie->len);
				ssid.SSID_len = ssid_ie->len;
			}
			else if(used_default_ssid){
				printf("Use default SSID to if SSID is NULL\n");
				memcpy(ssid.SSID, default_ssid, strlen(default_ssid));
				ssid.SSID_len = strlen(default_ssid);
				used_default_ssid = 0;
				
			}		
			

			printf("SSID is (%s) in Head  , ssid.SSID_len[%d]\n", ssid.SSID,ssid.SSID_len);
			
			wldev_iovar_setint(dev, "mpc", 0);
			err = wldev_ioctl(dev, WLC_DOWN, &ap, sizeof(s32), true);
			if (err < 0) {
				WL_ERR(("WLC_DOWN error %d\n", err));
				goto exit;
			}
			err = wldev_ioctl(dev, WLC_SET_INFRA, &infra, sizeof(s32), true);
			if (err < 0) {
				WL_ERR(("SET INFRA error %d\n", err));
				goto exit;
			}
			if ((err = wldev_ioctl(dev, WLC_SET_AP, &ap, sizeof(s32), true)) < 0) {
				WL_ERR(("setting AP mode failed %d \n", err));
				return err;
			}
			
			if ((wpa2_ie = bcm_parse_tlvs((u8 *)info->tail, info->tail_len,
				DOT11_MNG_RSN_ID)) != NULL) {
				WL_DBG((" WPA2 IE is found\n"));
			}
			
			if ((wpa_ie = wl_cfgp2p_find_wpaie((u8 *)info->tail,
			info->tail_len)) != NULL) {
				WL_DBG((" WPA IE is found\n"));
			}
			if ((wpa_ie != NULL || wpa2_ie != NULL)) {
				if (wl_validate_wpa2ie(dev, wpa2_ie, bssidx)  < 0 ||
					wl_validate_wpaie(dev, wpa_ie, bssidx) < 0) {
					wl->ap_info->security_mode = false;
					return BCME_ERROR;
				}
				wl->ap_info->security_mode = true;
				if (wl->ap_info->rsn_ie) {
					kfree(wl->ap_info->rsn_ie);
					wl->ap_info->rsn_ie = NULL;
				}
				if (wl->ap_info->wpa_ie) {
					kfree(wl->ap_info->wpa_ie);
					wl->ap_info->wpa_ie = NULL;
				}
				if (wl->ap_info->wps_ie) {
					kfree(wl->ap_info->wps_ie);
					wl->ap_info->wps_ie = NULL;
				}
				if (wpa_ie != NULL) {
					
					wl->ap_info->rsn_ie = NULL;
					wl->ap_info->wpa_ie = kmemdup(wpa_ie,
						wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
				} else {
					
					wl->ap_info->wpa_ie = NULL;
					wl->ap_info->rsn_ie = kmemdup(wpa2_ie,
						wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
				}
			} else {
#if 0
				wl_validate_opensecurity(dev, bssidx);
#endif
				wl->ap_info->security_mode = false;
			}
			
			if ((wps_ie = wl_cfgp2p_find_wpsie((u8 *)info->tail,
				info->tail_len)) != NULL) {
				wpsie_len = wps_ie->length +WPA_RSN_IE_TAG_FIXED_LEN;
				wl_validate_wps_ie((char *) wps_ie, &pbc);
				memcpy(beacon_ie, wps_ie, wpsie_len);
				wl_cfgp2p_set_management_ie(wl, dev, bssidx, VNDR_IE_BEACON_FLAG,
				beacon_ie, wpsie_len);
				wl->ap_info->wps_ie = kmemdup(wps_ie, wpsie_len, GFP_KERNEL);
				
				wl_add_remove_eventmsg(dev, WLC_E_PROBREQ_MSG, pbc);
			} else {
				WL_DBG(("No WPSIE in beacon \n"));
			}
			if (info->interval) {
				if ((err = wldev_ioctl(dev, WLC_SET_BCNPRD,
					&info->interval, sizeof(s32), true)) < 0) {
					WL_ERR(("Beacon Interval Set Error, %d\n", err));
					return err;
				}
			}
			if (info->dtim_period) {
				if ((err = wldev_ioctl(dev, WLC_SET_DTIMPRD,
					&info->dtim_period, sizeof(s32), true)) < 0) {
					WL_ERR(("DTIM Interval Set Error, %d\n", err));
					return err;
				}
			}
			
			if (closed) {
				err = wldev_ioctl(dev, WLC_SET_CLOSED, &closed, sizeof(closed), true);
				if (err)
					WL_ERR(("Set hidden ap error, %d\n", err));
			}
			
			err = wldev_ioctl(dev, WLC_UP, &ap, sizeof(s32), true);
			if (unlikely(err)) {
				WL_ERR(("WLC_UP error (%d)\n", err));
				return err;
			}

			
			
			max_assoc_ie = bcm_parse_tlvs((u8 *)&info->head[ie_offset],
							info->head_len - ie_offset,
							DOT11_MNG_VS_ID + 1);
			if (max_assoc_ie != NULL) {
				max_assoc = max_assoc_ie->data[0];
				printf("%s: got max_num_sta, max_assoc=%d\n", __func__, max_assoc);
			}
			err = wldev_iovar_setint(dev, "maxassoc", max_assoc);
			if (err)
				WL_ERR(("%s fail to set maxassoc, err=(%d)\n", __func__, err));
			
			memset(&join_params, 0, sizeof(join_params));
			
			join_params_size = sizeof(join_params.ssid);
			memcpy(join_params.ssid.SSID, ssid.SSID, ssid.SSID_len);
			join_params.ssid.SSID_len = htod32(ssid.SSID_len);
			
			if ((err = wldev_ioctl(dev, WLC_SET_SSID, &join_params,
				join_params_size, true)) == 0) {
				wl_clr_drv_status(wl, AP_CREATING, dev);
				wl_set_drv_status(wl, AP_CREATED, dev);
			}

			
			bcm_mkiovar("prslmt", (char *)&prslmt, 4, wl->ioctl_buf, WLC_IOCTL_MAXLEN);
			if ((err = wldev_ioctl(dev, WLC_SET_VAR, wl->ioctl_buf, WLC_IOCTL_MAXLEN, TRUE)) < 0)
				WL_ERR(("%s set probe response limit for HostAPD failed %d\n", __FUNCTION__, err));
		}
	} else if (wl_get_drv_status(wl, AP_CREATED, dev)) {
		ap = 1;
		
		if ((wps_ie = wl_cfgp2p_find_wpsie((u8 *)info->tail, info->tail_len)) != NULL) {
			wpsie_len = wps_ie->length + WPA_RSN_IE_TAG_FIXED_LEN;
			wl_validate_wps_ie((char *) wps_ie, &pbc);
			memcpy(beacon_ie, wps_ie, wpsie_len);
			wl_cfgp2p_set_management_ie(wl, dev, bssidx, VNDR_IE_BEACON_FLAG,
			beacon_ie, wpsie_len);
			if (wl->ap_info->wps_ie &&
				memcmp(wl->ap_info->wps_ie, wps_ie, wpsie_len)) {
				WL_DBG((" WPS IE is changed\n"));
				kfree(wl->ap_info->wps_ie);
				wl->ap_info->wps_ie = kmemdup(wps_ie, wpsie_len, GFP_KERNEL);
				
				wl_add_remove_eventmsg(dev, WLC_E_PROBREQ_MSG, pbc);
			} else if (wl->ap_info->wps_ie == NULL) {
				WL_DBG((" WPS IE is added\n"));
				wl->ap_info->wps_ie = kmemdup(wps_ie, wpsie_len, GFP_KERNEL);
				
				wl_add_remove_eventmsg(dev, WLC_E_PROBREQ_MSG, pbc);
			}
			
			if ((wpa2_ie = bcm_parse_tlvs((u8 *)info->tail, info->tail_len,
				DOT11_MNG_RSN_ID)) != NULL) {
				WL_DBG((" WPA2 IE is found\n"));
			}
			
			if ((wpa_ie = wl_cfgp2p_find_wpaie((u8 *)info->tail,
				info->tail_len)) != NULL) {
				WL_DBG((" WPA IE is found\n"));
			}
			if ((wpa_ie != NULL || wpa2_ie != NULL)) {
				if (!wl->ap_info->security_mode) {
					
					update_bss = true;
					if (wpa_ie != NULL) {
						wl->ap_info->wpa_ie = kmemdup(wpa_ie,
						wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
					} else {
						wl->ap_info->rsn_ie = kmemdup(wpa2_ie,
						wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
					}
				} else if (wl->ap_info->wpa_ie) {
					
					if (wpa2_ie != NULL) {
						update_bss = true;
						kfree(wl->ap_info->wpa_ie);
						wl->ap_info->rsn_ie = kmemdup(wpa2_ie,
						wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
						wl->ap_info->wpa_ie = NULL;
					}
					else if (memcmp(wl->ap_info->wpa_ie,
						wpa_ie, wpa_ie->length +
						WPA_RSN_IE_TAG_FIXED_LEN)) {
						kfree(wl->ap_info->wpa_ie);
						update_bss = true;
						wl->ap_info->wpa_ie = kmemdup(wpa_ie,
						wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
						wl->ap_info->rsn_ie = NULL;
					}
				} else {
					
					if (wpa_ie != NULL) {
						update_bss = true;
						kfree(wl->ap_info->rsn_ie);
						wl->ap_info->rsn_ie = NULL;
						wl->ap_info->wpa_ie = kmemdup(wpa_ie,
						wpa_ie->length + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
					} else if (memcmp(wl->ap_info->rsn_ie,
						wpa2_ie, wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN)) {
						update_bss = true;
						kfree(wl->ap_info->rsn_ie);
						wl->ap_info->rsn_ie = kmemdup(wpa2_ie,
						wpa2_ie->len + WPA_RSN_IE_TAG_FIXED_LEN,
						GFP_KERNEL);
						wl->ap_info->wpa_ie = NULL;
					}
				}
				if (update_bss) {
					wl->ap_info->security_mode = true;
					wl_cfgp2p_bss(wl, dev, bssidx, 0);
					if (wl_validate_wpa2ie(dev, wpa2_ie, bssidx)  < 0 ||
						wl_validate_wpaie(dev, wpa_ie, bssidx) < 0) {
						return BCME_ERROR;
					}
					wl_cfgp2p_bss(wl, dev, bssidx, 1);
				}
			}
		} else {
			WL_ERR(("No WPSIE in beacon \n"));
		}
	}
exit:
	if (err)
#ifdef FORCE_MPC
		if (!strstr(firmware_path, "_p2p"))
#endif
			wldev_iovar_setint(dev, "mpc", 1);
	return err;
}
#endif 

static struct cfg80211_ops wl_cfg80211_ops = {
	.add_virtual_intf = wl_cfg80211_add_virtual_iface,
	.del_virtual_intf = wl_cfg80211_del_virtual_iface,
	.change_virtual_intf = wl_cfg80211_change_virtual_iface,
	.scan = wl_cfg80211_scan,
	.set_wiphy_params = wl_cfg80211_set_wiphy_params,
	.join_ibss = wl_cfg80211_join_ibss,
	.leave_ibss = wl_cfg80211_leave_ibss,
	.get_station = wl_cfg80211_get_station,
	.set_tx_power = wl_cfg80211_set_tx_power,
	.get_tx_power = wl_cfg80211_get_tx_power,
	.add_key = wl_cfg80211_add_key,
	.del_key = wl_cfg80211_del_key,
	.get_key = wl_cfg80211_get_key,
	.set_default_key = wl_cfg80211_config_default_key,
	.set_default_mgmt_key = wl_cfg80211_config_default_mgmt_key,
	.set_power_mgmt = wl_cfg80211_set_power_mgmt,
	.connect = wl_cfg80211_connect,
	.disconnect = wl_cfg80211_disconnect,
	.suspend = wl_cfg80211_suspend,
	.resume = wl_cfg80211_resume,
	.set_pmksa = wl_cfg80211_set_pmksa,
	.del_pmksa = wl_cfg80211_del_pmksa,
	.flush_pmksa = wl_cfg80211_flush_pmksa,
	.remain_on_channel = wl_cfg80211_remain_on_channel,
	.cancel_remain_on_channel = wl_cfg80211_cancel_remain_on_channel,
	.mgmt_tx = wl_cfg80211_mgmt_tx,
	.mgmt_frame_register = wl_cfg80211_mgmt_frame_register,
	.change_bss = wl_cfg80211_change_bss,
	.set_channel = wl_cfg80211_set_channel,
#if 1 
	.set_beacon = wl_cfg80211_add_set_beacon,
	.add_beacon = wl_cfg80211_add_set_beacon,
#else
	.change_beacon = wl_cfg80211_change_beacon,
	.start_ap = wl_cfg80211_start_ap,
	.stop_ap = wl_cfg80211_stop_ap,
#endif 
};

s32 wl_mode_to_nl80211_iftype(s32 mode)
{
	s32 err = 0;

	switch (mode) {
	case WL_MODE_BSS:
		return NL80211_IFTYPE_STATION;
	case WL_MODE_IBSS:
		return NL80211_IFTYPE_ADHOC;
	case WL_MODE_AP:
		return NL80211_IFTYPE_AP;
	default:
		return NL80211_IFTYPE_UNSPECIFIED;
	}

	return err;
}

static s32 wl_setup_wiphy(struct wireless_dev *wdev, struct device *sdiofunc_dev)
{
	s32 err = 0;
	wdev->wiphy =
	    wiphy_new(&wl_cfg80211_ops, sizeof(struct wl_priv));
	if (unlikely(!wdev->wiphy)) {
		WL_ERR(("Couldn not allocate wiphy device\n"));
		err = -ENOMEM;
		return err;
	}
	set_wiphy_dev(wdev->wiphy, sdiofunc_dev);
	wdev->wiphy->max_scan_ie_len = WL_SCAN_IE_LEN_MAX;
	
	wdev->wiphy->max_scan_ssids = WL_SCAN_PARAMS_SSID_MAX;
	wdev->wiphy->max_num_pmkids = WL_NUM_PMKIDS_MAX;
	wdev->wiphy->interface_modes =
		BIT(NL80211_IFTYPE_STATION)
#if !(defined(WLP2P) && defined(WL_ENABLE_P2P_IF))
		| BIT(NL80211_IFTYPE_MONITOR)
#endif
		| BIT(NL80211_IFTYPE_AP);

	wdev->wiphy->bands[IEEE80211_BAND_2GHZ] = &__wl_band_2ghz;

	wdev->wiphy->signal_type = CFG80211_SIGNAL_TYPE_MBM;
	wdev->wiphy->cipher_suites = __wl_cipher_suites;
	wdev->wiphy->n_cipher_suites = ARRAY_SIZE(__wl_cipher_suites);
	wdev->wiphy->max_remain_on_channel_duration = 5000;
	wdev->wiphy->mgmt_stypes = wl_cfg80211_default_mgmt_stypes;
#ifndef WL_POWERSAVE_DISABLED
	wdev->wiphy->flags |= WIPHY_FLAG_PS_ON_BY_DEFAULT;
#else
	wdev->wiphy->flags &= ~WIPHY_FLAG_PS_ON_BY_DEFAULT;
#endif				
	wdev->wiphy->flags |= WIPHY_FLAG_NETNS_OK |
		WIPHY_FLAG_4ADDR_AP |
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 39)
		WIPHY_FLAG_SUPPORTS_SEPARATE_DEFAULT_KEYS |
#endif
		WIPHY_FLAG_4ADDR_STATION;
#ifdef NOT_YET
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 2, 0)
	wdev->wiphy->flags |= WIPHY_FLAG_SUPPORTS_FW_ROAM;
#endif
#endif 
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
	wdev->wiphy->flags |= WIPHY_FLAG_HAS_REMAIN_ON_CHANNEL |
		WIPHY_FLAG_OFFCHAN_TX;
#endif
	WL_DBG(("Registering custom regulatory)\n"));
	wdev->wiphy->flags |= WIPHY_FLAG_CUSTOM_REGULATORY;
	wiphy_apply_custom_regulatory(wdev->wiphy, &brcm_regdom);
	
	err = wiphy_register(wdev->wiphy);
	if (unlikely(err < 0)) {
		WL_ERR(("Couldn not register wiphy device (%d)\n", err));
		wiphy_free(wdev->wiphy);
	}
	return err;
}

static void wl_free_wdev(struct wl_priv *wl)
{
	struct wireless_dev *wdev = wl->wdev;
	struct wiphy *wiphy;
	if (!wdev) {
		WL_ERR(("wdev is invalid\n"));
		return;
	}
	wiphy = wdev->wiphy;
	wiphy_unregister(wdev->wiphy);
	wdev->wiphy->dev.parent = NULL;

	wl_delete_all_netinfo(wl);
	wiphy_free(wiphy);
}

static s32 wl_inform_bss(struct wl_priv *wl)
{
	struct wl_scan_results *bss_list;
	struct wl_bss_info *bi = NULL;	
	s32 err = 0;
	s32 i;

	bss_list = wl->bss_list;
	WL_DBG(("scanned AP count (%d)\n", bss_list->count));
#ifdef ROAM_CHANNEL_CACHE
	reset_roam_cache();
#endif
	bi = next_bss(bss_list, bi);
	for_each_bss(bss_list, bi, i) {
#ifdef ROAM_CHANNEL_CACHE
		add_roam_cache(bi);
#endif
		err = wl_inform_single_bss(wl, bi);
		if (unlikely(err))
			break;
	}
#ifdef ROAM_CHANNEL_CACHE
	
#endif
	return err;
}

static s32 wl_inform_single_bss(struct wl_priv *wl, struct wl_bss_info *bi)
{
	struct wiphy *wiphy = wiphy_from_scan(wl);
	struct ieee80211_mgmt *mgmt;
	struct ieee80211_channel *channel;
	struct ieee80211_supported_band *band;
	struct wl_cfg80211_bss_info *notif_bss_info;
	struct wl_scan_req *sr = wl_to_sr(wl);
	struct beacon_proberesp *beacon_proberesp;
	struct cfg80211_bss *cbss = NULL;
	s32 mgmt_type;
	s32 signal;
	u32 freq;
	s32 err = 0;

	gfp_t aflags;

	if (unlikely(dtoh32(bi->length) > WL_BSS_INFO_MAX)) {
		WL_DBG(("Beacon is larger than buffer. Discarding\n"));
		return err;
	}
	aflags = (in_atomic()) ? GFP_ATOMIC : GFP_KERNEL;
	notif_bss_info = kzalloc(sizeof(*notif_bss_info) + sizeof(*mgmt)
		- sizeof(u8) + WL_BSS_INFO_MAX, aflags);
	if (unlikely(!notif_bss_info)) {
		WL_ERR(("notif_bss_info alloc failed\n"));
		return -ENOMEM;
	}
	mgmt = (struct ieee80211_mgmt *)notif_bss_info->frame_buf;
	notif_bss_info->channel =
		bi->ctl_ch ? bi->ctl_ch : CHSPEC_CHANNEL(wl_chspec_driver_to_host(bi->chanspec));

	if (notif_bss_info->channel <= CH_MAX_2G_CHANNEL)
		band = wiphy->bands[IEEE80211_BAND_2GHZ];
	else
		band = wiphy->bands[IEEE80211_BAND_5GHZ];
	if (!band) {
		WL_ERR(("No valid band"));
		kfree(notif_bss_info);
		return -EINVAL;
	}
	notif_bss_info->rssi = dtoh16(bi->RSSI) + RSSI_OFFSET;
	memcpy(mgmt->bssid, &bi->BSSID, ETHER_ADDR_LEN);
	mgmt_type = wl->active_scan ?
		IEEE80211_STYPE_PROBE_RESP : IEEE80211_STYPE_BEACON;
	if (!memcmp(bi->SSID, sr->ssid.SSID, bi->SSID_len)) {
	    mgmt->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT | mgmt_type);
	}
	beacon_proberesp = wl->active_scan ?
		(struct beacon_proberesp *)&mgmt->u.probe_resp :
		(struct beacon_proberesp *)&mgmt->u.beacon;
	beacon_proberesp->timestamp = 0;
	beacon_proberesp->beacon_int = cpu_to_le16(bi->beacon_period);
	beacon_proberesp->capab_info = cpu_to_le16(bi->capability);
	wl_rst_ie(wl);

	wl_mrg_ie(wl, ((u8 *) bi) + bi->ie_offset, bi->ie_length);
	wl_cp_ie(wl, beacon_proberesp->variable, WL_BSS_INFO_MAX -
		offsetof(struct wl_cfg80211_bss_info, frame_buf));
	notif_bss_info->frame_len = offsetof(struct ieee80211_mgmt,
		u.beacon.variable) + wl_get_ielen(wl);
#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 38) && !defined(WL_COMPAT_WIRELESS)
	freq = ieee80211_channel_to_frequency(notif_bss_info->channel);
	(void)band->band;
#else
	freq = ieee80211_channel_to_frequency(notif_bss_info->channel, band->band);
#endif
	if (freq == 0) {
		WL_ERR(("Invalid channel, fail to chcnage channel to freq\n"));
		kfree(notif_bss_info);
		return EINVAL;
	}
	channel = ieee80211_get_channel(wiphy, freq);

	if (unlikely(!channel)) {
		WL_ERR(("ieee80211_get_channel error\n"));
		kfree(notif_bss_info);
		return EINVAL;
	}
	WL_DBG(("SSID : \"%s\", rssi %d, channel %d, capability : 0x04%x, bssid %pM"
			"mgmt_type %d frame_len %d\n", bi->SSID,
			notif_bss_info->rssi, notif_bss_info->channel,
			mgmt->u.beacon.capab_info, &bi->BSSID, mgmt_type,
			notif_bss_info->frame_len));

	signal = notif_bss_info->rssi * 100;

	cbss = cfg80211_inform_bss_frame(wiphy, channel, mgmt,
		le16_to_cpu(notif_bss_info->frame_len), signal, aflags);
	if (unlikely(!cbss)) {
		WL_ERR(("cfg80211_inform_bss_frame error\n"));
		kfree(notif_bss_info);
		return -EINVAL;
	}

	cfg80211_put_bss(cbss);
	kfree(notif_bss_info);

	return err;
}

static bool wl_is_linkup(struct wl_priv *wl, const wl_event_msg_t *e, struct net_device *ndev)
{
	u32 event = ntoh32(e->event_type);
	u32 status =  ntoh32(e->status);
	u16 flags = ntoh16(e->flags);

	WL_DBG(("event %d, status %d flags %x\n", event, status, flags));
	if (event == WLC_E_SET_SSID) {
		if (status == WLC_E_STATUS_SUCCESS) {
			if (!wl_is_ibssmode(wl, ndev))
				return true;
		}
	} else if (event == WLC_E_LINK) {
		if (flags & WLC_EVENT_MSG_LINK)
			return true;
	}

	WL_DBG(("wl_is_linkup false\n"));
	return false;
}

static bool wl_is_linkdown(struct wl_priv *wl, const wl_event_msg_t *e)
{
	u32 event = ntoh32(e->event_type);
	u16 flags = ntoh16(e->flags);

	if (event == WLC_E_DEAUTH_IND ||
	event == WLC_E_DISASSOC_IND ||
	event == WLC_E_DISASSOC ||
	event == WLC_E_DEAUTH) {
		return true;
	} else if (event == WLC_E_LINK) {
		if (!(flags & WLC_EVENT_MSG_LINK))
			return true;
	}

	return false;
}

static bool wl_is_nonetwork(struct wl_priv *wl, const wl_event_msg_t *e)
{
	u32 event = ntoh32(e->event_type);
	u32 status = ntoh32(e->status);

	if (event == WLC_E_LINK && status == WLC_E_STATUS_NO_NETWORKS)
		return true;
	if (event == WLC_E_SET_SSID && status != WLC_E_STATUS_SUCCESS)
		return true;

	return false;
}

static s32
wl_notify_connect_status_ap(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	s32 err = 0;
	u32 event = ntoh32(e->event_type);
	u32 reason = ntoh32(e->reason);
	u32 len = ntoh32(e->datalen);

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)) && !defined(WL_CFG80211_STA_EVENT)
	bool isfree = false;
	u8 *mgmt_frame;
	u8 bsscfgidx = e->bsscfgidx;
	s32 freq;
	s32 channel;
	u8 *body = NULL;
	u16 fc = 0;

	struct ieee80211_supported_band *band;
	struct ether_addr da;
	struct ether_addr bssid;
	struct wiphy *wiphy = wl_to_wiphy(wl);
	channel_info_t ci;
#else
	struct station_info sinfo;
#endif 

	
	if (event == WLC_E_LINK && reason == WLC_E_LINK_BSSCFG_DIS &&
		wl_get_p2p_status(wl, IF_DELETING) && (ndev != wl_to_prmry_ndev(wl))) {
		WL_INFO(("AP mode link down !! \n"));
		complete(&wl->iface_disable);
		return 0;
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 2, 0)) && !defined(WL_CFG80211_STA_EVENT)
	body = kzalloc(len, GFP_KERNEL);
	WL_DBG(("Enter \n"));
	if (!len && (event == WLC_E_DEAUTH)) {
		len = 2; 
		data = &reason;
	}
	if (len) {
		body = kzalloc(len, GFP_KERNEL);

		if (body == NULL) {
			WL_ERR(("wl_notify_connect_status: Failed to allocate body\n"));
			return WL_INVALID;
		}
	}
	memset(&bssid, 0, ETHER_ADDR_LEN);
	WL_DBG(("Enter event %d ndev %p\n", event, ndev));
	if (wl_get_mode_by_netdev(wl, ndev) == WL_INVALID) {
		kfree(body);
		return WL_INVALID;
	}
	if (len)
		memcpy(body, data, len);

	wldev_iovar_getbuf_bsscfg(ndev, "cur_etheraddr",
		NULL, 0, wl->ioctl_buf, WLC_IOCTL_SMLEN, bsscfgidx, &wl->ioctl_buf_sync);
	memcpy(da.octet, wl->ioctl_buf, ETHER_ADDR_LEN);
	err = wldev_ioctl(ndev, WLC_GET_BSSID, &bssid, ETHER_ADDR_LEN, false);
	switch (event) {
		case WLC_E_ASSOC_IND:
			fc = FC_ASSOC_REQ;
			break;
		case WLC_E_REASSOC_IND:
			fc = FC_REASSOC_REQ;
			break;
		case WLC_E_DISASSOC_IND:
			fc = FC_DISASSOC;
			break;
		case WLC_E_DEAUTH_IND:
			fc = FC_DISASSOC;
			break;
		case WLC_E_DEAUTH:
			fc = FC_DISASSOC;
			break;
		default:
			fc = 0;
			goto exit;
	}
	if ((err = wldev_ioctl(ndev, WLC_GET_CHANNEL, &ci, sizeof(ci), false))) {
		kfree(body);
		return err;
	}

	channel = dtoh32(ci.hw_channel);
	if (channel <= CH_MAX_2G_CHANNEL)
		band = wiphy->bands[IEEE80211_BAND_2GHZ];
	else
		band = wiphy->bands[IEEE80211_BAND_5GHZ];

	if (!band) {
		WL_ERR(("No valid band"));
		if (body)
			kfree(body);
		return -EINVAL;
	}
#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 38) && !defined(WL_COMPAT_WIRELESS)
	freq = ieee80211_channel_to_frequency(channel);
	(void)band->band;
#else
	freq = ieee80211_channel_to_frequency(channel, band->band);
#endif

	err = wl_frame_get_mgmt(fc, &da, &e->addr, &bssid,
		&mgmt_frame, &len, body);
	if (err < 0)
		goto exit;
	isfree = true;

	if (((event == WLC_E_ASSOC_IND) && (reason == DOT11_SC_SUCCESS))
		|| ((event == WLC_E_DEAUTH_IND) || (event == WLC_E_DEAUTH))
		|| (event == WLC_E_DISASSOC_IND)) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
		cfg80211_rx_mgmt(ndev, freq, 0, mgmt_frame, len, GFP_ATOMIC);
#else
		cfg80211_rx_mgmt(ndev, freq, mgmt_frame, len, GFP_ATOMIC);
#endif
	}

exit:
	if (isfree)
		kfree(mgmt_frame);
	if (body)
		kfree(body);
	return err;
#else 
	sinfo.filled = 0;
	if (((event == WLC_E_ASSOC_IND) || (event == WLC_E_REASSOC_IND)) &&
		reason == DOT11_SC_SUCCESS) {
		sinfo.filled = STATION_INFO_ASSOC_REQ_IES;
		if (!data) {
			WL_ERR(("No IEs present in ASSOC/REASSOC_IND"));
			return -EINVAL;
		}
		sinfo.assoc_req_ies = data;
		sinfo.assoc_req_ies_len = len;
		cfg80211_new_sta(ndev, e->addr.octet, &sinfo, GFP_ATOMIC);
	} else if (event == WLC_E_DISASSOC_IND) {
		cfg80211_del_sta(ndev, e->addr.octet, GFP_ATOMIC);
	} else if ((event == WLC_E_DEAUTH_IND) || (event == WLC_E_DEAUTH)) {
		cfg80211_del_sta(ndev, e->addr.octet, GFP_ATOMIC);
	}
#endif 
	return err;
}

static s32
wl_notify_connect_status(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	bool act;
	s32 err = 0;
	u32 event = ntoh32(e->event_type);

	if (wl_get_mode_by_netdev(wl, ndev) == WL_MODE_AP) {
		wl_notify_connect_status_ap(wl, ndev, e, data);
	} else {
		WL_DBG(("wl_notify_connect_status : event %d status : %d ndev %p\n",
			ntoh32(e->event_type), ntoh32(e->status), ndev));
		if (wl_is_linkup(wl, e, ndev)) {
			wl_link_up(wl);
			act = true;
			if (wl_is_ibssmode(wl, ndev)) {
				printk("cfg80211_ibss_joined\n");
				cfg80211_ibss_joined(ndev, (s8 *)&e->addr,
					GFP_KERNEL);
				WL_DBG(("joined in IBSS network\n"));
			} else {
				if (!wl_get_drv_status(wl, DISCONNECTING, ndev)) {
					printf("wl_bss_connect_done succeeded with " MACSTR "\n", MAC2STR(e->addr.octet));
					wl_bss_connect_done(wl, ndev, e, data, true);
					WL_DBG(("joined in BSS network \"%s\"\n",
					((struct wlc_ssid *)
					 wl_read_prof(wl, ndev, WL_PROF_SSID))->SSID));
				
				#ifdef APSTA_CONCURRENT
				if(wl->apsta_concurrent && ap_net_dev){
					int errcode;
					printf("%s Sanity check  ConAP and STA channel",__FUNCTION__);
					wldev_san_check_channel(ndev,&errcode);
					if(errcode == 0)
						printf("%s Pass STA_AP_Chan Sanity Check\n",__FUNCTION__);
					else{
						if(ntoh32(e->event_type) == WLC_E_SET_SSID)
							errcode = wldev_set_ssid(ndev,&errcode); 
					}
				}
				#endif
				
				}
			}
			wl_update_prof(wl, ndev, e, &act, WL_PROF_ACT);
			wl_update_prof(wl, ndev, NULL, (void *)&e->addr, WL_PROF_BSSID);

		} else if (wl_is_linkdown(wl, e)) {
			if (wl->scan_request) {
				if (wl->escan_on) {
					wl_notify_escan_complete(wl, ndev, true, true);
				} else {
					del_timer_sync(&wl->scan_timeout);
					wl_iscan_aborted(wl);
				}
			}
			if (wl_get_drv_status(wl, CONNECTED, ndev)) {
				scb_val_t scbval;
				u8 *curbssid = wl_read_prof(wl, ndev, WL_PROF_BSSID);
				printf("link down, call cfg80211_disconnected from " MACSTR ". (reason=%d)\n",
					MAC2STR(e->addr.octet), ntoh32(e->reason));
#ifdef ESCAN_RESULT_PATCH
				if (memcmp(curbssid, &e->addr, ETHER_ADDR_LEN) != 0) {
					WL_ERR(("BSSID of event is not the connected BSSID\n"));
				}
#endif 
				wl_clr_drv_status(wl, CONNECTED, ndev);
				if (! wl_get_drv_status(wl, DISCONNECTING, ndev)) {
					scbval.val = WLAN_REASON_DEAUTH_LEAVING;

					memcpy(&scbval.ea, curbssid, ETHER_ADDR_LEN);
					scbval.val = htod32(scbval.val);
					wldev_ioctl(ndev, WLC_DISASSOC, &scbval,
						sizeof(scb_val_t), true);
					cfg80211_disconnected(ndev, 0, NULL, 0, GFP_KERNEL);
					wl_link_down(wl);
					wl_init_prof(wl, ndev);
				}
			}
			else if (wl_get_drv_status(wl, CONNECTING, ndev)) {
				printk("link down, during connecting\n");
#ifdef ESCAN_RESULT_PATCH
				if ((memcmp(connect_req_bssid, broad_bssid, ETHER_ADDR_LEN) == 0) ||
					(memcmp(&e->addr, broad_bssid, ETHER_ADDR_LEN) == 0) ||
					(memcmp(&e->addr, connect_req_bssid, ETHER_ADDR_LEN) == 0))
					
#endif 
					wl_bss_connect_done(wl, ndev, e, data, false);
			}
			wl_clr_drv_status(wl, DISCONNECTING, ndev);

			
			if (ndev != wl_to_prmry_ndev(wl))
				complete(&wl->iface_disable);

		} else if (wl_is_nonetwork(wl, e)) {
			printk("connect failed event=%d e->status 0x%x\n",
				event, (int)ntoh32(e->status));
			
			if (wl->scan_request) {
				if (wl->escan_on) {
					wl_notify_escan_complete(wl, ndev, true, true);
				} else {
					del_timer_sync(&wl->scan_timeout);
					wl_iscan_aborted(wl);
				}
			}
			if (wl_get_drv_status(wl, CONNECTING, ndev))
				wl_bss_connect_done(wl, ndev, e, data, false);
		} else {
			printk("%s nothing\n", __FUNCTION__);
		}
	}
	return err;
}

static s32
wl_notify_roaming_status(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	bool act;
	s32 err = 0;
	u32 event = be32_to_cpu(e->event_type);
	u32 status = be32_to_cpu(e->status);
	WL_DBG(("Enter \n"));
	if (event == WLC_E_ROAM && status == WLC_E_STATUS_SUCCESS) {
		if (wl_get_drv_status(wl, CONNECTED, ndev))
			wl_bss_roaming_done(wl, ndev, e, data);
		else
			wl_bss_connect_done(wl, ndev, e, data, true);
		act = true;
		wl_update_prof(wl, ndev, e, &act, WL_PROF_ACT);
		wl_update_prof(wl, ndev, NULL, (void *)&e->addr, WL_PROF_BSSID);
	}
	return err;
}

static s32 wl_get_assoc_ies(struct wl_priv *wl, struct net_device *ndev)
{
	wl_assoc_info_t assoc_info;
	struct wl_connect_info *conn_info = wl_to_conn(wl);
	s32 err = 0;

	WL_DBG(("Enter \n"));
	err = wldev_iovar_getbuf(ndev, "assoc_info", NULL, 0, wl->extra_buf,
		WL_ASSOC_INFO_MAX, NULL);
	if (unlikely(err)) {
		WL_ERR(("could not get assoc info (%d)\n", err));
		return err;
	}
	memcpy(&assoc_info, wl->extra_buf, sizeof(wl_assoc_info_t));
	assoc_info.req_len = htod32(assoc_info.req_len);
	assoc_info.resp_len = htod32(assoc_info.resp_len);
	assoc_info.flags = htod32(assoc_info.flags);
	if (conn_info->req_ie_len) {
		conn_info->req_ie_len = 0;
		bzero(conn_info->req_ie, sizeof(conn_info->req_ie));
	}
	if (conn_info->resp_ie_len) {
		conn_info->resp_ie_len = 0;
		bzero(conn_info->resp_ie, sizeof(conn_info->resp_ie));
	}
	if (assoc_info.req_len) {
		err = wldev_iovar_getbuf(ndev, "assoc_req_ies", NULL, 0, wl->extra_buf,
			WL_ASSOC_INFO_MAX, NULL);
		if (unlikely(err)) {
			WL_ERR(("could not get assoc req (%d)\n", err));
			return err;
		}
		conn_info->req_ie_len = assoc_info.req_len - sizeof(struct dot11_assoc_req);
		if (assoc_info.flags & WLC_ASSOC_REQ_IS_REASSOC) {
			conn_info->req_ie_len -= ETHER_ADDR_LEN;
		}
		if (conn_info->req_ie_len <= MAX_REQ_LINE)
			memcpy(conn_info->req_ie, wl->extra_buf, conn_info->req_ie_len);
		else {
			WL_ERR(("%s IE size %d above max %d size \n",
				__FUNCTION__, conn_info->req_ie_len, MAX_REQ_LINE));
			return err;
		}
	} else {
		conn_info->req_ie_len = 0;
	}
	if (assoc_info.resp_len) {
		err = wldev_iovar_getbuf(ndev, "assoc_resp_ies", NULL, 0, wl->extra_buf,
			WL_ASSOC_INFO_MAX, NULL);
		if (unlikely(err)) {
			WL_ERR(("could not get assoc resp (%d)\n", err));
			return err;
		}
		conn_info->resp_ie_len = assoc_info.resp_len -sizeof(struct dot11_assoc_resp);
		if (conn_info->resp_ie_len <= MAX_REQ_LINE)
			memcpy(conn_info->resp_ie, wl->extra_buf, conn_info->resp_ie_len);
		else {
			WL_ERR(("%s IE size %d above max %d size \n",
				__FUNCTION__, conn_info->resp_ie_len, MAX_REQ_LINE));
			return err;
		}
	} else {
		conn_info->resp_ie_len = 0;
	}
	WL_DBG(("req len (%d) resp len (%d)\n", conn_info->req_ie_len,
		conn_info->resp_ie_len));

	return err;
}

static void wl_ch_to_chanspec(int ch, struct wl_join_params *join_params,
        size_t *join_params_size)
{
#ifndef ROAM_CHANNEL_CACHE
	chanspec_t chanspec = 0;

#endif
	if (ch != 0) {
#ifdef ROAM_CHANNEL_CACHE
		int n_channels;

		n_channels = get_roam_channel_list(ch, join_params->params.chanspec_list,
			&join_params->ssid, ioctl_version);
		join_params->params.chanspec_num = htod32(n_channels);
		*join_params_size += WL_ASSOC_PARAMS_FIXED_SIZE +
			join_params->params.chanspec_num * sizeof(chanspec_t);
#else
		join_params->params.chanspec_num = 1;
		join_params->params.chanspec_list[0] = ch;

		if (join_params->params.chanspec_list[0] <= CH_MAX_2G_CHANNEL)
			chanspec |= WL_CHANSPEC_BAND_2G;
		else
			chanspec |= WL_CHANSPEC_BAND_5G;

		chanspec |= WL_CHANSPEC_BW_20;
		chanspec |= WL_CHANSPEC_CTL_SB_NONE;

		*join_params_size += WL_ASSOC_PARAMS_FIXED_SIZE +
			join_params->params.chanspec_num * sizeof(chanspec_t);

		join_params->params.chanspec_list[0]  &= WL_CHANSPEC_CHAN_MASK;
		join_params->params.chanspec_list[0] |= chanspec;
		join_params->params.chanspec_list[0] =
			wl_chspec_host_to_driver(join_params->params.chanspec_list[0]);

		join_params->params.chanspec_num =
			htod32(join_params->params.chanspec_num);

#endif 
		WL_DBG(("join_params->params.chanspec_list[0]= %X, %d channels\n",
			join_params->params.chanspec_list[0],
			join_params->params.chanspec_num));

	}
}

static s32 wl_update_bss_info(struct wl_priv *wl, struct net_device *ndev)
{
	struct cfg80211_bss *bss;
	struct wl_bss_info *bi;
	struct wlc_ssid *ssid;
	struct bcm_tlv *tim;
	s32 beacon_interval;
	s32 dtim_period;
	size_t ie_len;
	u8 *ie;
	u8 *curbssid;
	s32 err = 0;
	struct wiphy *wiphy;

	wiphy = wl_to_wiphy(wl);

	if (wl_is_ibssmode(wl, ndev))
		return err;

	ssid = (struct wlc_ssid *)wl_read_prof(wl, ndev, WL_PROF_SSID);
	curbssid = wl_read_prof(wl, ndev, WL_PROF_BSSID);
	bss = cfg80211_get_bss(wiphy, NULL, curbssid,
		ssid->SSID, ssid->SSID_len, WLAN_CAPABILITY_ESS,
		WLAN_CAPABILITY_ESS);

	mutex_lock(&wl->usr_sync);
	if (!bss) {
		WL_DBG(("Could not find the AP\n"));
		*(u32 *) wl->extra_buf = htod32(WL_EXTRA_BUF_MAX);
		err = wldev_ioctl(ndev, WLC_GET_BSS_INFO,
			wl->extra_buf, WL_EXTRA_BUF_MAX, false);
		if (unlikely(err)) {
			WL_ERR(("Could not get bss info %d\n", err));
			goto update_bss_info_out;
		}
		bi = (struct wl_bss_info *)(wl->extra_buf + 4);
		if (memcmp(bi->BSSID.octet, curbssid, ETHER_ADDR_LEN)) {
			err = -EIO;
			goto update_bss_info_out;
		}
		err = wl_inform_single_bss(wl, bi);
		if (unlikely(err))
			goto update_bss_info_out;

		ie = ((u8 *)bi) + bi->ie_offset;
		ie_len = bi->ie_length;
		beacon_interval = cpu_to_le16(bi->beacon_period);
	} else {
#ifdef HTC_KlocWork
		if (bss) {
#endif
			WL_DBG(("Found the AP in the list - BSSID %pM\n", bss->bssid));
			ie = bss->information_elements;
			ie_len = bss->len_information_elements;
			beacon_interval = bss->beacon_interval;
			cfg80211_put_bss(bss);
		} else {
			WL_DBG(("Could not find the AP\n"));
			goto update_bss_info_out;
		}
	}

	tim = bcm_parse_tlvs(ie, ie_len, WLAN_EID_TIM);
	if (tim) {
		dtim_period = tim->data[1];
	} else {
		err = wldev_ioctl(ndev, WLC_GET_DTIMPRD,
			&dtim_period, sizeof(dtim_period), false);
		if (unlikely(err)) {
			WL_ERR(("WLC_GET_DTIMPRD error (%d)\n", err));
			goto update_bss_info_out;
		}
	}

	wl_update_prof(wl, ndev, NULL, &beacon_interval, WL_PROF_BEACONINT);
	wl_update_prof(wl, ndev, NULL, &dtim_period, WL_PROF_DTIMPERIOD);

update_bss_info_out:
	mutex_unlock(&wl->usr_sync);
	return err;
}

static s32
wl_bss_roaming_done(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	struct wl_connect_info *conn_info = wl_to_conn(wl);
	s32 err = 0;
	u8 *curbssid;

	WL_DBG(("Enter %s\n", __func__));
	wl_get_assoc_ies(wl, ndev);
	wl_update_prof(wl, ndev, NULL, (void *)(e->addr.octet), WL_PROF_BSSID);
	curbssid = wl_read_prof(wl, ndev, WL_PROF_BSSID);
	wl_update_bss_info(wl, ndev);
	wl_update_pmklist(ndev, wl->pmk_list, err);


	cfg80211_roamed(ndev,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 3, 0)
		NULL,	
#elif LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 39)
		NULL,
#endif
		curbssid,
		conn_info->req_ie, conn_info->req_ie_len,
		conn_info->resp_ie, conn_info->resp_ie_len, GFP_KERNEL);
	WL_DBG(("Report roaming result\n"));

	wl_set_drv_status(wl, CONNECTED, ndev);

	return err;
}

static s32
wl_bss_connect_done(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data, bool completed)
{
	struct wl_connect_info *conn_info = wl_to_conn(wl);
	s32 err = 0;
	u8 *curbssid = wl_read_prof(wl, ndev, WL_PROF_BSSID);

	WL_DBG((" enter\n"));
#ifdef ESCAN_RESULT_PATCH
	if (wl_get_drv_status(wl, CONNECTED, ndev)) {
		if (memcmp(curbssid, connect_req_bssid, ETHER_ADDR_LEN) == 0) {
			WL_ERR((" Connected event of connected device, ignore it\n"));
			return err;
		}
	}
	if (memcmp(curbssid, broad_bssid, ETHER_ADDR_LEN) == 0 &&
		memcmp(broad_bssid, connect_req_bssid, ETHER_ADDR_LEN) != 0) {
		WL_DBG(("copy bssid\n"));
		memcpy(curbssid, connect_req_bssid, ETHER_ADDR_LEN);
	}

#if defined(BCM4334_CHIP)
	if (wl->scan_request) {
		wl_notify_escan_complete(wl, ndev, true, true);
	}
#endif
#else
	if (wl->scan_request) {
		wl_notify_escan_complete(wl, ndev, true, true);
	}
#endif 
	if (wl_get_drv_status(wl, CONNECTING, ndev)) {
		wl_clr_drv_status(wl, CONNECTING, ndev);
		if (completed) {
			wl_get_assoc_ies(wl, ndev);
			wl_update_prof(wl, ndev, NULL, (void *)(e->addr.octet), WL_PROF_BSSID);
			curbssid = wl_read_prof(wl, ndev, WL_PROF_BSSID);
			wl_update_bss_info(wl, ndev);
			wl_update_pmklist(ndev, wl->pmk_list, err);
			wl_set_drv_status(wl, CONNECTED, ndev);
		}
		cfg80211_connect_result(ndev,
			curbssid,
			conn_info->req_ie,
			conn_info->req_ie_len,
			conn_info->resp_ie,
			conn_info->resp_ie_len,
			completed ? WLAN_STATUS_SUCCESS : WLAN_STATUS_AUTH_TIMEOUT,
			GFP_KERNEL);
		if (completed)
			WL_INFO(("Report connect result - connection succeeded\n"));
		else
			WL_ERR(("Report connect result - connection failed\n"));
	}
	return err;
}

static s32
wl_notify_mic_status(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	u16 flags = ntoh16(e->flags);
	enum nl80211_key_type key_type;

	mutex_lock(&wl->usr_sync);
	if (flags & WLC_EVENT_MSG_GROUP)
		key_type = NL80211_KEYTYPE_GROUP;
	else
		key_type = NL80211_KEYTYPE_PAIRWISE;

	cfg80211_michael_mic_failure(ndev, (u8 *)&e->addr, key_type, -1,
		NULL, GFP_KERNEL);
	mutex_unlock(&wl->usr_sync);

	return 0;
}

static s32
wl_notify_scan_status(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	struct channel_info channel_inform;
	struct wl_scan_results *bss_list;
	u32 len = WL_SCAN_BUF_MAX;
	s32 err = 0;
	unsigned long flags;

	WL_DBG(("Enter \n"));
	if (!wl_get_drv_status(wl, SCANNING, ndev)) {
		WL_ERR(("scan is not ready \n"));
		return err;
	}
	if (wl->iscan_on && wl->iscan_kickstart)
		return wl_wakeup_iscan(wl_to_iscan(wl));

	mutex_lock(&wl->usr_sync);
	wl_clr_drv_status(wl, SCANNING, ndev);
	err = wldev_ioctl(ndev, WLC_GET_CHANNEL, &channel_inform,
		sizeof(channel_inform), false);
	if (unlikely(err)) {
		WL_ERR(("scan busy (%d)\n", err));
		goto scan_done_out;
	}
	channel_inform.scan_channel = dtoh32(channel_inform.scan_channel);
	if (unlikely(channel_inform.scan_channel)) {

		WL_DBG(("channel_inform.scan_channel (%d)\n",
			channel_inform.scan_channel));
	}
	wl->bss_list = wl->scan_results;
	bss_list = wl->bss_list;
	memset(bss_list, 0, len);
	bss_list->buflen = htod32(len);
	err = wldev_ioctl(ndev, WLC_SCAN_RESULTS, bss_list, len, false);
	if (unlikely(err)) {
		WL_ERR(("%s Scan_results error (%d)\n", ndev->name, err));
		err = -EINVAL;
		goto scan_done_out;
	}
	bss_list->buflen = dtoh32(bss_list->buflen);
	bss_list->version = dtoh32(bss_list->version);
	bss_list->count = dtoh32(bss_list->count);

	printf("scan count = %d\n", bss_list->count);
	err = wl_inform_bss(wl);

scan_done_out:
	del_timer_sync(&wl->scan_timeout);
	spin_lock_irqsave(&wl->cfgdrv_lock, flags);
	if (wl->scan_request) {
		cfg80211_scan_done(wl->scan_request, false);
		wl->scan_request = NULL;
	}
	spin_unlock_irqrestore(&wl->cfgdrv_lock, flags);
	WL_DBG(("cfg80211_scan_done\n"));
	mutex_unlock(&wl->usr_sync);
	return err;
}
static s32
wl_frame_get_mgmt(u16 fc, const struct ether_addr *da,
	const struct ether_addr *sa, const struct ether_addr *bssid,
	u8 **pheader, u32 *body_len, u8 *pbody)
{
	struct dot11_management_header *hdr;
	u32 totlen = 0;
	s32 err = 0;
	u8 *offset;
	u32 prebody_len = *body_len;
	switch (fc) {
		case FC_ASSOC_REQ:
			
			totlen = DOT11_ASSOC_REQ_FIXED_LEN;
			*body_len += DOT11_ASSOC_REQ_FIXED_LEN;
			break;

		case FC_REASSOC_REQ:
			
			totlen = DOT11_REASSOC_REQ_FIXED_LEN;
			*body_len += DOT11_REASSOC_REQ_FIXED_LEN;
			break;
	}
	totlen += DOT11_MGMT_HDR_LEN + prebody_len;
	*pheader = kzalloc(totlen, GFP_KERNEL);
	if (*pheader == NULL) {
		WL_ERR(("memory alloc failed \n"));
		return -ENOMEM;
	}
	hdr = (struct dot11_management_header *) (*pheader);
	hdr->fc = htol16(fc);
	hdr->durid = 0;
	hdr->seq = 0;
	offset = (u8*)(hdr + 1) + (totlen - DOT11_MGMT_HDR_LEN - prebody_len);
	bcopy((const char*)da, (u8*)&hdr->da, ETHER_ADDR_LEN);
	bcopy((const char*)sa, (u8*)&hdr->sa, ETHER_ADDR_LEN);
	bcopy((const char*)bssid, (u8*)&hdr->bssid, ETHER_ADDR_LEN);
	if ((pbody != NULL) && prebody_len)
		bcopy((const char*)pbody, offset, prebody_len);
	*body_len = totlen;
	return err;
}

#ifdef WL_CFG80211_GON_COLLISION
static void
wl_gon_req_collision(struct wl_priv *wl, wl_action_frame_t *tx_act_frm,
	wifi_p2p_pub_act_frame_t *rx_act_frm, struct net_device *ndev,
	struct ether_addr sa, struct ether_addr da)
{
	if (wl->afx_hdl->pending_tx_act_frm == NULL)
		return;

	if (tx_act_frm &&
		wl_cfgp2p_is_pub_action(tx_act_frm->data, tx_act_frm->len)) {
		wifi_p2p_pub_act_frame_t *pact_frm;

		pact_frm = (wifi_p2p_pub_act_frame_t *)tx_act_frm->data;

		if (!(pact_frm->subtype == P2P_PAF_GON_REQ &&
			rx_act_frm->subtype == P2P_PAF_GON_REQ)) {
			return;
		}
	}

	WL_ERR((" GO NEGO Request COLLISION !!! \n"));

	if (memcmp(sa.octet, da.octet, ETHER_ADDR_LEN) < 0) {
		
		wl->block_gon_req_tx_count = BLOCK_GON_REQ_MAX_NUM;
		WL_ERR((" block to send gon req tx !!!\n"));

		if (wl_get_drv_status_all(wl, FINDING_COMMON_CHANNEL)) {
			wl_clr_drv_status(wl, FINDING_COMMON_CHANNEL, ndev);
			complete(&wl->act_frm_scan);
		}
	} else {
		
		WL_ERR((" drop to receive gon req rx !!! \n"));
		wl->block_gon_req_rx_count = BLOCK_GON_REQ_MAX_NUM;
	}

	return;
}
#endif 

static void
wl_stop_wait_next_action_frame(struct wl_priv *wl, struct net_device *ndev)
{
	if (wl_get_drv_status_all(wl, SENDING_ACT_FRM) &&
		(wl_get_p2p_status(wl, ACTION_TX_COMPLETED) ||
		wl_get_p2p_status(wl, ACTION_TX_NOACK))) {
		WL_DBG(("*** Wake UP ** abort actframe iovar\n"));
		if (wl->af_sent_channel)
			
			wl_notify_escan_complete(wl,
				(ndev == wl->p2p_net) ? wl_to_prmry_ndev(wl) : ndev, true, true);
	}
#ifdef WL_CFG80211_SYNC_GON
	else if (wl_get_drv_status_all(wl, WAITING_NEXT_ACT_FRM_LISTEN)) {
		WL_DBG(("*** Wake UP ** abort listen for next af frame\n"));
		
		wl_notify_escan_complete(wl,
			(ndev == wl->p2p_net) ? wl_to_prmry_ndev(wl) : ndev, true, true);
	}
#endif 
}
static s32
wl_notify_rx_mgmt_frame(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	struct ieee80211_supported_band *band;
	struct wiphy *wiphy = wl_to_wiphy(wl);
	struct ether_addr da;
	struct ether_addr bssid;
	bool isfree = false;
	s32 err = 0;
	s32 freq;
	struct net_device *dev = NULL;
	wifi_p2p_pub_act_frame_t *act_frm = NULL;
	wifi_p2p_action_frame_t *p2p_act_frm = NULL;
	wifi_p2psd_gas_pub_act_frame_t *sd_act_frm = NULL;
	wl_event_rx_frame_data_t *rxframe =
		(wl_event_rx_frame_data_t*)data;
	u32 event = ntoh32(e->event_type);
	u8 *mgmt_frame;
	u8 bsscfgidx = e->bsscfgidx;
	u32 mgmt_frame_len = ntoh32(e->datalen) - sizeof(wl_event_rx_frame_data_t);
	u16 channel = ((ntoh16(rxframe->channel) & WL_CHANSPEC_CHAN_MASK));

	memset(&bssid, 0, ETHER_ADDR_LEN);

	if (wl->p2p_net == ndev) {
		dev = wl_to_prmry_ndev(wl);
	} else {
		dev = ndev;
	}

	if (channel <= CH_MAX_2G_CHANNEL)
		band = wiphy->bands[IEEE80211_BAND_2GHZ];
	else
		band = wiphy->bands[IEEE80211_BAND_5GHZ];

	if (!band) {
		WL_ERR(("No valid band"));
		return -EINVAL;
	}
#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 38) && !defined(WL_COMPAT_WIRELESS)
	freq = ieee80211_channel_to_frequency(channel);
	(void)band->band;
#else
	freq = ieee80211_channel_to_frequency(channel, band->band);
#endif
	if (event == WLC_E_ACTION_FRAME_RX) {
		wldev_iovar_getbuf_bsscfg(dev, "cur_etheraddr",
			NULL, 0, wl->ioctl_buf, WLC_IOCTL_SMLEN, bsscfgidx, &wl->ioctl_buf_sync);

		wldev_ioctl(dev, WLC_GET_BSSID, &bssid, ETHER_ADDR_LEN, false);
		memcpy(da.octet, wl->ioctl_buf, ETHER_ADDR_LEN);
		err = wl_frame_get_mgmt(FC_ACTION, &da, &e->addr, &bssid,
			&mgmt_frame, &mgmt_frame_len,
			(u8 *)((wl_event_rx_frame_data_t *)rxframe + 1));
		if (err < 0) {
			WL_ERR(("%s: Error in receiving action frame len %d channel %d freq %d\n",
				__func__, mgmt_frame_len, channel, freq));
			goto exit;
		}
		isfree = true;
		if (wl_cfgp2p_is_pub_action(&mgmt_frame[DOT11_MGMT_HDR_LEN],
			mgmt_frame_len - DOT11_MGMT_HDR_LEN)) {
			act_frm = (wifi_p2p_pub_act_frame_t *)
					(&mgmt_frame[DOT11_MGMT_HDR_LEN]);
		} else if (wl_cfgp2p_is_p2p_action(&mgmt_frame[DOT11_MGMT_HDR_LEN],
			mgmt_frame_len - DOT11_MGMT_HDR_LEN)) {
			p2p_act_frm = (wifi_p2p_action_frame_t *)
					(&mgmt_frame[DOT11_MGMT_HDR_LEN]);
			(void) p2p_act_frm;
		} else if (wl_cfgp2p_is_gas_action(&mgmt_frame[DOT11_MGMT_HDR_LEN],
			mgmt_frame_len - DOT11_MGMT_HDR_LEN)) {
			sd_act_frm = (wifi_p2psd_gas_pub_act_frame_t *)
					(&mgmt_frame[DOT11_MGMT_HDR_LEN]);
			if (sd_act_frm && wl_get_drv_status_all(wl, WAITING_NEXT_ACT_FRM)) {
				if (wl->next_af_subtype == sd_act_frm->action) {
					WL_DBG(("We got a right next frame of SD!(%d)\n",
						sd_act_frm->action));
					wl_clr_drv_status(wl, WAITING_NEXT_ACT_FRM,
						(ndev == wl->p2p_net) ?
						wl_to_prmry_ndev(wl) : ndev);

					
					wl_stop_wait_next_action_frame(wl, ndev);
				}
			}
			(void) sd_act_frm;
		}
		if (act_frm) {
#ifdef WL_CFG80211_GON_COLLISION
			if (act_frm->subtype == P2P_PAF_GON_REQ) {
				wl_gon_req_collision(wl,
					&wl->afx_hdl->pending_tx_act_frm->action_frame,
					act_frm, ndev, e->addr, da);

				if (wl->block_gon_req_rx_count) {
					WL_ERR(("drop frame GON Req Rx : count (%d)\n",
						wl->block_gon_req_rx_count));
					wl->block_gon_req_rx_count--;
					goto exit;
				}
			} else if (act_frm->subtype == P2P_PAF_GON_CONF) {
				
				wl->block_gon_req_tx_count = 0;
				wl->block_gon_req_rx_count = 0;
			}
#endif 

			if (wl_get_drv_status_all(wl, WAITING_NEXT_ACT_FRM)) {
				if (wl->next_af_subtype == act_frm->subtype) {
					WL_DBG(("We got a right next frame!(%d)\n",
						act_frm->subtype));
					wl_clr_drv_status(wl, WAITING_NEXT_ACT_FRM,
						(ndev == wl->p2p_net) ?
						wl_to_prmry_ndev(wl) : ndev);

					
					wl_stop_wait_next_action_frame(wl, ndev);
				}
			}
		}
		wl_cfgp2p_print_actframe(false, &mgmt_frame[DOT11_MGMT_HDR_LEN],
			mgmt_frame_len - DOT11_MGMT_HDR_LEN);
		if (act_frm && ((act_frm->subtype == P2P_PAF_GON_CONF) ||
			(act_frm->subtype == P2P_PAF_PROVDIS_RSP))) {
#ifdef FORCE_MPC
			if (!strstr(firmware_path, "_p2p"))
#endif
				wldev_iovar_setint(dev, "mpc", 1);
		}
		if (act_frm && (act_frm->subtype == P2P_PAF_GON_CONF)) {
			WL_DBG(("P2P: GO_NEG_PHASE status cleared \n"));
			wl_clr_p2p_status(wl, GO_NEG_PHASE);
		}
	} else {
		mgmt_frame = (u8 *)((wl_event_rx_frame_data_t *)rxframe + 1);
		if (event == WLC_E_P2P_PROBREQ_MSG) {
			WL_DBG((" Event %s\n", (event == WLC_E_P2P_PROBREQ_MSG) ?
				"WLC_E_P2P_PROBREQ_MSG":"WLC_E_PROBREQ_MSG"));

#ifdef WL_CFG80211_USE_PRB_REQ_FOR_AF_TX
			if (WL_DRV_STATUS_SENDING_AF_FRM_EXT(wl) &&
				!memcmp(wl->afx_hdl->tx_dst_addr.octet, e->addr.octet,
				ETHER_ADDR_LEN)) {
				if (wl->afx_hdl->pending_tx_act_frm &&
					wl_get_drv_status_all(wl, FINDING_COMMON_CHANNEL)) {
					s32 channel = CHSPEC_CHANNEL(hton16(rxframe->channel));
					WL_DBG(("PROBE REQUEST : Peer found, channel : %d\n",
						channel));
					wl->afx_hdl->peer_chan = channel;
					complete(&wl->act_frm_scan);
				}
			}
#endif 

			if (wl->p2p &&
				wl_get_p2p_status(wl, GO_NEG_PHASE)) {
				WL_DBG(("Filtering P2P probe_req while "
					"being in GO-Neg state\n"));
				return 0;
			}
		}
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 4, 0)
	cfg80211_rx_mgmt(ndev, freq, 0, mgmt_frame, mgmt_frame_len, GFP_ATOMIC);
#else
	cfg80211_rx_mgmt(ndev, freq, mgmt_frame, mgmt_frame_len, GFP_ATOMIC);
#endif 

	WL_DBG(("%s: mgmt_frame_len (%d) , e->datalen (%d), channel (%d), freq (%d)\n", __func__,
		mgmt_frame_len, ntoh32(e->datalen), channel, freq));

	if (isfree)
		kfree(mgmt_frame);
exit:
	return 0;
}

static void wl_init_conf(struct wl_conf *conf)
{
	WL_DBG(("Enter \n"));
	conf->frag_threshold = (u32)-1;
	conf->rts_threshold = (u32)-1;
	conf->retry_short = (u32)-1;
	conf->retry_long = (u32)-1;
	conf->tx_power = -1;
}

static void wl_init_prof(struct wl_priv *wl, struct net_device *ndev)
{
	unsigned long flags;
	struct wl_profile *profile = wl_get_profile_by_netdev(wl, ndev);

	spin_lock_irqsave(&wl->cfgdrv_lock, flags);
	
	if(profile != NULL){
	memset(profile, 0, sizeof(struct wl_profile));
    }
	
	spin_unlock_irqrestore(&wl->cfgdrv_lock, flags);
}

static void wl_init_event_handler(struct wl_priv *wl)
{
	memset(wl->evt_handler, 0, sizeof(wl->evt_handler));

	wl->evt_handler[WLC_E_SCAN_COMPLETE] = wl_notify_scan_status;
	wl->evt_handler[WLC_E_LINK] = wl_notify_connect_status;
	wl->evt_handler[WLC_E_DEAUTH_IND] = wl_notify_connect_status;
	wl->evt_handler[WLC_E_DEAUTH] = wl_notify_connect_status;
	wl->evt_handler[WLC_E_DISASSOC_IND] = wl_notify_connect_status;
	wl->evt_handler[WLC_E_ASSOC_IND] = wl_notify_connect_status;
	wl->evt_handler[WLC_E_REASSOC_IND] = wl_notify_connect_status;
	wl->evt_handler[WLC_E_ROAM] = wl_notify_roaming_status;
	wl->evt_handler[WLC_E_MIC_ERROR] = wl_notify_mic_status;
	wl->evt_handler[WLC_E_SET_SSID] = wl_notify_connect_status;
	wl->evt_handler[WLC_E_ACTION_FRAME_RX] = wl_notify_rx_mgmt_frame;
	wl->evt_handler[WLC_E_PROBREQ_MSG] = wl_notify_rx_mgmt_frame;
	wl->evt_handler[WLC_E_P2P_PROBREQ_MSG] = wl_notify_rx_mgmt_frame;
	wl->evt_handler[WLC_E_P2P_DISC_LISTEN_COMPLETE] = wl_cfgp2p_listen_complete;
	wl->evt_handler[WLC_E_ACTION_FRAME_COMPLETE] = wl_cfgp2p_action_tx_complete;
	wl->evt_handler[WLC_E_ACTION_FRAME_OFF_CHAN_COMPLETE] = wl_cfgp2p_action_tx_complete;
#if defined(HTC_TX_TRACKING)
	wl->evt_handler[WLC_E_TX_STAT_ERROR] = wl_notify_txfail;
#endif

}

static s32 wl_init_priv_mem(struct wl_priv *wl)
{
	WL_DBG(("Enter \n"));
	wl->scan_results = NULL;
	wl->conf = NULL;
	wl->scan_req_int = NULL;
	wl->ioctl_buf = NULL;
	wl->escan_ioctl_buf = NULL;
	wl->extra_buf = NULL;
	wl->iscan = NULL;
	wl->pmk_list = NULL;
	wl->sta_info = NULL;
	wl->afx_hdl = NULL;
#if defined(STATIC_WL_PRIV_STRUCT)
	wl->conn_info = NULL;
	wl->ie = NULL;
#endif
	wl->scan_results = (void *)kzalloc(WL_SCAN_BUF_MAX, GFP_KERNEL);
	if (unlikely(!wl->scan_results)) {
		WL_ERR(("Scan results alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->conf = (void *)kzalloc(sizeof(*wl->conf), GFP_KERNEL);
	if (unlikely(!wl->conf)) {
		WL_ERR(("wl_conf alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->scan_req_int =
	    (void *)kzalloc(sizeof(*wl->scan_req_int), GFP_KERNEL);
	if (unlikely(!wl->scan_req_int)) {
		WL_ERR(("Scan req alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->ioctl_buf = (void *)kzalloc(WLC_IOCTL_MAXLEN, GFP_KERNEL);
	if (unlikely(!wl->ioctl_buf)) {
		WL_ERR(("Ioctl buf alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->escan_ioctl_buf = (void *)kzalloc(WLC_IOCTL_MAXLEN, GFP_KERNEL);
	if (unlikely(!wl->escan_ioctl_buf)) {
		WL_ERR(("Ioctl buf alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->extra_buf = (void *)kzalloc(WL_EXTRA_BUF_MAX, GFP_KERNEL);
	if (unlikely(!wl->extra_buf)) {
		WL_ERR(("Extra buf alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->iscan = (void *)kzalloc(sizeof(*wl->iscan), GFP_KERNEL);
	if (unlikely(!wl->iscan)) {
		WL_ERR(("Iscan buf alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->pmk_list = (void *)kzalloc(sizeof(*wl->pmk_list), GFP_KERNEL);
	if (unlikely(!wl->pmk_list)) {
		WL_ERR(("pmk list alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->sta_info = (void *)kzalloc(sizeof(*wl->sta_info), GFP_KERNEL);
	if (unlikely(!wl->sta_info)) {
		WL_ERR(("sta info  alloc failed\n"));
		goto init_priv_mem_out;
	}
#if defined(STATIC_WL_PRIV_STRUCT)
	wl->conn_info = (void *)kzalloc(sizeof(*wl->conn_info), GFP_KERNEL);
	if (unlikely(!wl->conn_info)) {
		WL_ERR(("wl->conn_info  alloc failed\n"));
		goto init_priv_mem_out;
	}
	wl->ie = (void *)kzalloc(sizeof(*wl->ie), GFP_KERNEL);
	if (unlikely(!wl->ie)) {
		WL_ERR(("wl->ie  alloc failed\n"));
		goto init_priv_mem_out;
	}
#if defined(DUAL_ESCAN_RESULT_BUFFER)
	wl->escan_info.escan_buf[0] = dhd_os_prealloc(NULL, DHD_PREALLOC_WIPHY_ESCAN0, 0);
	bzero(wl->escan_info.escan_buf[0], ESCAN_BUF_SIZE);
	wl->escan_info.escan_buf[1] = dhd_os_prealloc(NULL, DHD_PREALLOC_WIPHY_ESCAN1, 0);
	bzero(wl->escan_info.escan_buf[1], ESCAN_BUF_SIZE);
#else
	wl->escan_info.escan_buf = dhd_os_prealloc(NULL, DHD_PREALLOC_WIPHY_ESCAN0, 0);
	bzero(wl->escan_info.escan_buf, ESCAN_BUF_SIZE);
#endif 
#endif 
	wl->afx_hdl = (void *)kzalloc(sizeof(*wl->afx_hdl), GFP_KERNEL);
	if (unlikely(!wl->afx_hdl)) {
		WL_ERR(("afx hdl  alloc failed\n"));
		goto init_priv_mem_out;
	} else {
		init_completion(&wl->act_frm_scan);
		init_completion(&wl->wait_next_af);
		
		if(wl->afx_hdl != NULL)
		
			INIT_WORK(&wl->afx_hdl->work, wl_cfg80211_afx_handler);
	}
	return 0;

init_priv_mem_out:
	wl_deinit_priv_mem(wl);

	return -ENOMEM;
}

static void wl_deinit_priv_mem(struct wl_priv *wl)
{
	if (wl->scan_results) {
		kfree(wl->scan_results);
		wl->scan_results = NULL;
	}
	if (wl->conf) {
		kfree(wl->conf);
		wl->conf = NULL;
	}
	if (wl->scan_req_int) {
		kfree(wl->scan_req_int);
		wl->scan_req_int = NULL;
	}
	if (wl->ioctl_buf) {
		kfree(wl->ioctl_buf);
		wl->ioctl_buf = NULL;
	}
	if (wl->escan_ioctl_buf) {
		kfree(wl->escan_ioctl_buf);
		wl->escan_ioctl_buf = NULL;
	}
	if (wl->extra_buf) {
		kfree(wl->extra_buf);
		wl->extra_buf = NULL;
	}
	if (wl->iscan) {
		kfree(wl->iscan);
		wl->iscan = NULL;
	}
	if (wl->pmk_list) {
		kfree(wl->pmk_list);
		wl->pmk_list = NULL;
	}
	if (wl->sta_info) {
		kfree(wl->sta_info);
		wl->sta_info = NULL;
	}
#if defined(STATIC_WL_PRIV_STRUCT)
	if (wl->conn_info) {
		kfree(wl->conn_info);
		wl->conn_info = NULL;
	}
	if (wl->ie) {
		kfree(wl->ie);
		wl->ie = NULL;
	}
#if defined(DUAL_ESCAN_RESULT_BUFFER)
	wl->escan_info.escan_buf[0] = NULL;
	wl->escan_info.escan_buf[1] = NULL;
#else
	wl->escan_info.escan_buf = NULL;
#endif
#endif 
	if (wl->afx_hdl) {
		cancel_work_sync(&wl->afx_hdl->work);
		kfree(wl->afx_hdl);
		wl->afx_hdl = NULL;
	}

	if (wl->ap_info) {
		kfree(wl->ap_info->wpa_ie);
		kfree(wl->ap_info->rsn_ie);
		kfree(wl->ap_info->wps_ie);
		kfree(wl->ap_info);
		wl->ap_info = NULL;
	}
}

static s32 wl_create_event_handler(struct wl_priv *wl)
{
	int ret = 0, count = 0;
	WL_ERR(("Enter \n"));
	printf("%s : Enter\n",__func__);

	
retry:
	if (count < 100) {
		ret = 0;
		wl->event_tsk.thr_pid = -1;
#ifdef USE_KTHREAD_API
		PROC_START2(wl_event_handler, wl, &wl->event_tsk, 0, "wl_event_handler");
#else
		PROC_START(wl_event_handler, wl, &wl->event_tsk, 0);
#endif
		if ((wl->event_tsk.thr_pid < 0) || (wl->event_tsk.thr_pid > 0xf0000000)) {
			printf("%s: %dnd Create thread failed\n", __func__, count);
			count++;
			ret = -ENOMEM;
			msleep(100);
			goto retry;
		}
	}
	return ret;
}

static void wl_destroy_event_handler(struct wl_priv *wl)
{
	if (wl->event_tsk.thr_pid >= 0)
		PROC_STOP(&wl->event_tsk);
}

static void wl_term_iscan(struct wl_priv *wl)
{
	struct wl_iscan_ctrl *iscan = wl_to_iscan(wl);
	WL_TRACE(("In\n"));
	if (wl->iscan_on && iscan->tsk) {
		iscan->state = WL_ISCAN_STATE_IDLE;
		WL_INFO(("SIGTERM\n"));
		send_sig(SIGTERM, iscan->tsk, 1);
		WL_DBG(("kthread_stop\n"));
		kthread_stop(iscan->tsk);
		iscan->tsk = NULL;
	}
}

static void wl_notify_iscan_complete(struct wl_iscan_ctrl *iscan, bool aborted)
{
	struct wl_priv *wl = iscan_to_wl(iscan);
	struct net_device *ndev = wl_to_prmry_ndev(wl);
	unsigned long flags;

	WL_DBG(("Enter \n"));
	if (!wl_get_drv_status(wl, SCANNING, ndev)) {
		wl_clr_drv_status(wl, SCANNING, ndev);
		WL_ERR(("Scan complete while device not scanning\n"));
		return;
	}
	spin_lock_irqsave(&wl->cfgdrv_lock, flags);
	wl_clr_drv_status(wl, SCANNING, ndev);
	if (likely(wl->scan_request)) {
		cfg80211_scan_done(wl->scan_request, aborted);
		wl->scan_request = NULL;
	}
	spin_unlock_irqrestore(&wl->cfgdrv_lock, flags);
	wl->iscan_kickstart = false;
}

static s32 wl_wakeup_iscan(struct wl_iscan_ctrl *iscan)
{
	if (likely(iscan->state != WL_ISCAN_STATE_IDLE)) {
		WL_DBG(("wake up iscan\n"));
		up(&iscan->sync);
		return 0;
	}

	return -EIO;
}

static s32
wl_get_iscan_results(struct wl_iscan_ctrl *iscan, u32 *status,
	struct wl_scan_results **bss_list)
{
	struct wl_iscan_results list;
	struct wl_scan_results *results;
	struct wl_iscan_results *list_buf;
	s32 err = 0;

	WL_DBG(("Enter \n"));
	memset(iscan->scan_buf, 0, WL_ISCAN_BUF_MAX);
	list_buf = (struct wl_iscan_results *)iscan->scan_buf;
	results = &list_buf->results;
	results->buflen = WL_ISCAN_RESULTS_FIXED_SIZE;
	results->version = 0;
	results->count = 0;

	memset(&list, 0, sizeof(list));
	list.results.buflen = htod32(WL_ISCAN_BUF_MAX);
	err = wldev_iovar_getbuf(iscan->dev, "iscanresults", &list,
		WL_ISCAN_RESULTS_FIXED_SIZE, iscan->scan_buf,
		WL_ISCAN_BUF_MAX, NULL);
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
		return err;
	}
	results->buflen = dtoh32(results->buflen);
	results->version = dtoh32(results->version);
	results->count = dtoh32(results->count);
	WL_DBG(("results->count = %d\n", results->count));
	WL_DBG(("results->buflen = %d\n", results->buflen));
	*status = dtoh32(list_buf->status);
	*bss_list = results;

	return err;
}

static s32 wl_iscan_done(struct wl_priv *wl)
{
	struct wl_iscan_ctrl *iscan = wl->iscan;
	s32 err = 0;

	iscan->state = WL_ISCAN_STATE_IDLE;
	mutex_lock(&wl->usr_sync);
	wl_inform_bss(wl);
	wl_notify_iscan_complete(iscan, false);
	mutex_unlock(&wl->usr_sync);

	return err;
}

static s32 wl_iscan_pending(struct wl_priv *wl)
{
	struct wl_iscan_ctrl *iscan = wl->iscan;
	s32 err = 0;

	
	mod_timer(&iscan->timer, jiffies + iscan->timer_ms * HZ / 1000);
	iscan->timer_on = 1;

	return err;
}

static s32 wl_iscan_inprogress(struct wl_priv *wl)
{
	struct wl_iscan_ctrl *iscan = wl->iscan;
	s32 err = 0;

	mutex_lock(&wl->usr_sync);
	wl_inform_bss(wl);
	wl_run_iscan(iscan, NULL, WL_SCAN_ACTION_CONTINUE);
	mutex_unlock(&wl->usr_sync);
	
	mod_timer(&iscan->timer, jiffies + iscan->timer_ms * HZ / 1000);
	iscan->timer_on = 1;

	return err;
}

static s32 wl_iscan_aborted(struct wl_priv *wl)
{
	struct wl_iscan_ctrl *iscan = wl->iscan;
	s32 err = 0;

	iscan->state = WL_ISCAN_STATE_IDLE;
	mutex_lock(&wl->usr_sync);
	wl_notify_iscan_complete(iscan, true);
	mutex_unlock(&wl->usr_sync);

	return err;
}

static s32 wl_iscan_thread(void *data)
{
	struct wl_iscan_ctrl *iscan = (struct wl_iscan_ctrl *)data;
	struct wl_priv *wl = iscan_to_wl(iscan);
	u32 status;
	int err = 0;

	allow_signal(SIGTERM);
	status = WL_SCAN_RESULTS_PARTIAL;
	while (likely(!down_interruptible(&iscan->sync))) {
		if (kthread_should_stop())
			break;
		if (iscan->timer_on) {
			del_timer_sync(&iscan->timer);
			iscan->timer_on = 0;
		}
		mutex_lock(&wl->usr_sync);
		err = wl_get_iscan_results(iscan, &status, &wl->bss_list);
		if (unlikely(err)) {
			status = WL_SCAN_RESULTS_ABORTED;
			WL_ERR(("Abort iscan\n"));
		}
		mutex_unlock(&wl->usr_sync);
		iscan->iscan_handler[status] (wl);
	}
	if (iscan->timer_on) {
		del_timer_sync(&iscan->timer);
		iscan->timer_on = 0;
	}
	WL_DBG(("%s was terminated\n", __func__));

	return 0;
}
static int aborting = 1;
static void wl_scan_timeout(unsigned long data)
{
	struct wl_priv *wl = (struct wl_priv *)data;

	if (wl->scan_request) {
		WL_ERR(("timer expired(%d)\n", aborting));
		if (wl->escan_on) {
			if (aborting) {
				printf("%s: doing abort\n", __FUNCTION__);
				schedule_delayed_work(&abort_scan, 1);
				aborting = 0;
				mod_timer(&wl->scan_timeout, jiffies + 3 * HZ);
			} else {
				printf("%s: report scan failed\n", __FUNCTION__);
				wl_notify_escan_complete(wl, wl->escan_info.ndev, true, true);
				aborting = 1;
				scan_timeout_cnt++;
			}

			if (scan_timeout_cnt > SCAN_FAILE_CNT) {
				printf("Continues SCAN timeout happened, restart!\n");
				scan_timeout_cnt = 0;
				wl_cfg80211_hang(wl->escan_info.ndev, WLAN_REASON_UNSPECIFIED);
			}
		}
		else
			wl_notify_iscan_complete(wl_to_iscan(wl), true);
	}
}
static void wl_iscan_timer(unsigned long data)
{
	struct wl_iscan_ctrl *iscan = (struct wl_iscan_ctrl *)data;

	if (iscan) {
		iscan->timer_on = 0;
		WL_DBG(("timer expired\n"));
		wl_wakeup_iscan(iscan);
	}
}

static s32 wl_invoke_iscan(struct wl_priv *wl)
{
	struct wl_iscan_ctrl *iscan = wl_to_iscan(wl);
	int err = 0;

	if (wl->iscan_on && !iscan->tsk) {
		iscan->state = WL_ISCAN_STATE_IDLE;
		sema_init(&iscan->sync, 0);
		iscan->tsk = kthread_run(wl_iscan_thread, iscan, "wl_iscan");
		if (IS_ERR(iscan->tsk)) {
			WL_ERR(("Could not create iscan thread\n"));
			iscan->tsk = NULL;
			return -ENOMEM;
		}
	}

	return err;
}

static void wl_init_iscan_handler(struct wl_iscan_ctrl *iscan)
{
	memset(iscan->iscan_handler, 0, sizeof(iscan->iscan_handler));
	iscan->iscan_handler[WL_SCAN_RESULTS_SUCCESS] = wl_iscan_done;
	iscan->iscan_handler[WL_SCAN_RESULTS_PARTIAL] = wl_iscan_inprogress;
	iscan->iscan_handler[WL_SCAN_RESULTS_PENDING] = wl_iscan_pending;
	iscan->iscan_handler[WL_SCAN_RESULTS_ABORTED] = wl_iscan_aborted;
	iscan->iscan_handler[WL_SCAN_RESULTS_NO_MEM] = wl_iscan_aborted;
}

static s32
wl_cfg80211_netdev_notifier_call(struct notifier_block * nb,
	unsigned long state,
	void *ndev)
{
	struct net_device *dev = ndev;
	struct wireless_dev *wdev = dev->ieee80211_ptr;
	struct wl_priv *wl = wlcfg_drv_priv;
	int refcnt = 0;

	WL_DBG(("Enter \n"));
	if (!wdev || !wl || dev == wl_to_prmry_ndev(wl))
		return NOTIFY_DONE;
	switch (state) {
		case NETDEV_DOWN:
			while (work_pending(&wdev->cleanup_work) && refcnt < 100) {
				if (refcnt%5 == 0)
					WL_ERR(("%s : [NETDEV_DOWN] work_pending (%d th)\n",
						__FUNCTION__, refcnt));
				set_current_state(TASK_INTERRUPTIBLE);
				schedule_timeout(100);
				set_current_state(TASK_RUNNING);
				refcnt++;
			}
			break;
		case NETDEV_UNREGISTER:
			
			wl_dealloc_netinfo(wl, ndev);
			break;
		case NETDEV_GOING_DOWN:
			if (wl_get_drv_status(wl, SCANNING, dev)) {
				if (wl->escan_on) {
					wl_notify_escan_complete(wl, dev, true, true);
				}
			}
			break;
	}
	return NOTIFY_DONE;
}
static struct notifier_block wl_cfg80211_netdev_notifier = {
	.notifier_call = wl_cfg80211_netdev_notifier_call,
};

static s32 wl_notify_escan_complete(struct wl_priv *wl,
	struct net_device *ndev,
	bool aborted, bool fw_abort)
{

	wl_scan_params_t *params = NULL;
	s32 params_size = 0;
	s32 err = BCME_OK;
	unsigned long flags;
	struct net_device *dev;

	WL_DBG(("Enter \n"));

	if (wl->escan_info.ndev != ndev)
	{
		WL_ERR(("ndev is different %p %p\n", wl->escan_info.ndev, ndev));
		return err;
	}

	if (wl->scan_request) {
		if (wl->scan_request->dev == wl->p2p_net)
			dev = wl_to_prmry_ndev(wl);
		else
			dev = wl->scan_request->dev;
	}
	else {
		WL_ERR(("wl->scan_request is NULL may be internal scan."
			"doing scan_abort for ndev %p primary %p p2p_net %p",
				ndev, wl_to_prmry_ndev(wl), wl->p2p_net));
		dev = ndev;
	}
	if (fw_abort) {
		
		params = wl_cfg80211_scan_alloc_params(-1, 0, &params_size);
		if (params == NULL) {
			WL_ERR(("scan params allocation failed \n"));
			err = -ENOMEM;
		} else if (!in_atomic()) {
			
			err = wldev_ioctl(dev, WLC_SCAN, params, params_size, true);
			if (err < 0) {
				WL_ERR(("scan abort  failed \n"));
			}
		}
	}
	if (timer_pending(&wl->scan_timeout))
		del_timer_sync(&wl->scan_timeout);

#if defined(ESCAN_RESULT_PATCH)
	if (likely(wl->scan_request)) {
#if defined(DUAL_ESCAN_RESULT_BUFFER)
		u8 temp_id = wl->escan_info.cur_sync_id;
		if (aborted)
			wl->bss_list =
				(wl_scan_results_t *)wl->escan_info.escan_buf[(temp_id+1)%2];
		else
			wl->bss_list =
				(wl_scan_results_t *)wl->escan_info.escan_buf[(temp_id)%2];
#else
		wl->bss_list = (wl_scan_results_t *)wl->escan_info.escan_buf;
#endif
		wl_inform_bss(wl);
	}
#endif 
	spin_lock_irqsave(&wl->cfgdrv_lock, flags);
	if (likely(wl->scan_request)) {
		cfg80211_scan_done(wl->scan_request, aborted);
		wl->scan_request = NULL;
	}
	if (p2p_is_on(wl))
		wl_clr_p2p_status(wl, SCANNING);
	wl_clr_drv_status(wl, SCANNING, dev);
	spin_unlock_irqrestore(&wl->cfgdrv_lock, flags);
	if (params)
		kfree(params);

	return err;
}

static s32 wl_escan_handler(struct wl_priv *wl,
	struct net_device *ndev,
	const wl_event_msg_t *e, void *data)
{
	s32 err = BCME_OK;
	s32 status = ntoh32(e->status);
	wl_bss_info_t *bi;
	wl_escan_result_t *escan_result;
	wl_bss_info_t *bss = NULL;
	wl_scan_results_t *list;
	wifi_p2p_ie_t * p2p_ie;
	u32 bi_length;
	u32 i;
	u8 *p2p_dev_addr = NULL;

	WL_DBG((" enter event type : %d, status : %d \n",
		ntoh32(e->event_type), ntoh32(e->status)));

	scan_timeout_cnt = 0;
	mutex_lock(&wl->usr_sync);
	
	if (wl_get_p2p_status(wl, SCANNING)) {
		if (wl_get_drv_status_all(wl, SENDING_ACT_FRM))
			ndev = wl->afx_hdl->dev;
		else
			ndev = wl->escan_info.ndev;

	}

	if (!ndev || !wl->escan_on ||
		!wl_get_drv_status(wl, SCANNING, ndev)) {
#if 1 
		if(e != NULL)
		WL_ERR(("escan is not ready ndev %p wl->escan_on %d"
			" drv_status 0x%x e_type %d e_states %d\n",
			ndev, wl->escan_on, wl_get_drv_status(wl, SCANNING, ndev),
			ntoh32(e->event_type), ntoh32(e->status)));
		else
		WL_ERR(("escan is not ready ndev %p wl->escan_on %d drv_status 0x%x\n",
			ndev, wl->escan_on, wl_get_drv_status(wl, SCANNING, ndev)));
		
#else
		WL_ERR(("escan is not ready ndev %p wl->escan_on %d drv_status 0x%x\n",
			ndev, wl->escan_on, wl_get_drv_status(wl, SCANNING, ndev)));
#endif
		goto exit;
	}

#if defined(DUAL_ESCAN_RESULT_BUFFER)
	escan_result = (wl_escan_result_t *) data;
#endif
	if (status == WLC_E_STATUS_PARTIAL) {
		WL_INFO(("WLC_E_STATUS_PARTIAL \n"));
		escan_result = (wl_escan_result_t *) data;
		if (!escan_result) {
			WL_ERR(("Invalid escan result (NULL pointer)\n"));
			goto exit;
		}
		if (dtoh16(escan_result->bss_count) != 1) {
			WL_ERR(("Invalid bss_count %d: ignoring\n", escan_result->bss_count));
			goto exit;
		}
		bi = escan_result->bss_info;
		if (!bi) {
			WL_ERR(("Invalid escan bss info (NULL pointer)\n"));
			goto exit;
		}
		bi_length = dtoh32(bi->length);
		if (bi_length != (dtoh32(escan_result->buflen) - WL_ESCAN_RESULTS_FIXED_SIZE)) {
			WL_ERR(("Invalid bss_info length %d: ignoring\n", bi_length));
			goto exit;
		}
#if defined(DUAL_ESCAN_RESULT_BUFFER)
		if (escan_result->sync_id != wl->escan_info.cur_sync_id) {
			WL_ERR(("Escan sync id mismatch: status %d "
			"cur_sync_id %d coming_sync_id %d\n",
			status, wl->escan_info.cur_sync_id, escan_result->sync_id));
			goto exit;
		}
#endif

		if (!(wl_to_wiphy(wl)->interface_modes & BIT(NL80211_IFTYPE_ADHOC))) {
			if (dtoh16(bi->capability) & DOT11_CAP_IBSS) {
				WL_DBG(("Ignoring IBSS result\n"));
				goto exit;
			}
		}

		if (wl_get_drv_status_all(wl, FINDING_COMMON_CHANNEL)) {
			p2p_dev_addr = wl_cfgp2p_retreive_p2p_dev_addr(bi, bi_length);
			
			if (p2p_dev_addr && wl->afx_hdl&& !memcmp(p2p_dev_addr,
			
				wl->afx_hdl->tx_dst_addr.octet, ETHER_ADDR_LEN)) {
				s32 channel = CHSPEC_CHANNEL(
					wl_chspec_driver_to_host(bi->chanspec));
				WL_DBG(("ACTION FRAME SCAN : Peer " MACSTR " found, channel : %d\n",
					MAC2STR(wl->afx_hdl->tx_dst_addr.octet), channel));
				wl_clr_p2p_status(wl, SCANNING);
				wl->afx_hdl->peer_chan = channel;
				complete(&wl->act_frm_scan);
				goto exit;
			}

		} else {
			int cur_len = 0;
#if defined(DUAL_ESCAN_RESULT_BUFFER)
			list = (wl_scan_results_t *)
				wl->escan_info.escan_buf[wl->escan_info.cur_sync_id%2];
#else
			list = (wl_scan_results_t *)wl->escan_info.escan_buf;
#endif
#if defined(WLP2P) && defined(WL_ENABLE_P2P_IF)
			if (wl->p2p_net && wl->scan_request &&
				wl->scan_request->dev == wl->p2p_net) {
#else
			if (p2p_is_on(wl) && p2p_scan(wl)) {
#endif
				
				if (bi->flags & WL_BSS_FLAGS_FROM_BEACON)
					goto exit;
				if ((p2p_ie = wl_cfgp2p_find_p2pie(((u8 *) bi) + bi->ie_offset,
					bi->ie_length)) == NULL) {
						WL_ERR(("Couldn't find P2PIE in probe"
							" response/beacon\n"));
						goto exit;
				}
			}
			for (i = 0; i < list->count; i++) {
				bss = bss ? (wl_bss_info_t *)((uintptr)bss + dtoh32(bss->length))
					: list->bss_info;

				if (!bcmp(&bi->BSSID, &bss->BSSID, ETHER_ADDR_LEN) &&
					(CHSPEC_BAND(wl_chspec_driver_to_host(bi->chanspec))
					== CHSPEC_BAND(wl_chspec_driver_to_host(bss->chanspec))) &&
					bi->SSID_len == bss->SSID_len &&
					!bcmp(bi->SSID, bss->SSID, bi->SSID_len)) {
					if (!(bss->flags & WL_BSS_FLAGS_FROM_BEACON) &&
						(bi->flags & WL_BSS_FLAGS_FROM_BEACON))
						goto exit;

					WL_DBG(("%s("MACSTR"), i=%d prev: RSSI %d"
						" flags 0x%x, new: RSSI %d flags 0x%x\n",
						bss->SSID, MAC2STR(bi->BSSID.octet), i,
						bss->RSSI, bss->flags, bi->RSSI, bi->flags));

					if ((bss->flags & WL_BSS_FLAGS_RSSI_ONCHANNEL) ==
						(bi->flags & WL_BSS_FLAGS_RSSI_ONCHANNEL)) {
						WL_SCAN(("%s("MACSTR"), same onchan"
						", RSSI: prev %d new %d\n",
						bss->SSID, MAC2STR(bi->BSSID.octet),
						bss->RSSI, bi->RSSI));
						bi->RSSI = MAX(bss->RSSI, bi->RSSI);
					} else if ((bss->flags & WL_BSS_FLAGS_RSSI_ONCHANNEL) &&
						(bi->flags & WL_BSS_FLAGS_RSSI_ONCHANNEL) == 0) {
						WL_SCAN(("%s("MACSTR"), prev onchan"
						", RSSI: prev %d new %d\n",
						bss->SSID, MAC2STR(bi->BSSID.octet),
						bss->RSSI, bi->RSSI));
						bi->RSSI = bss->RSSI;
						bi->flags |= WL_BSS_FLAGS_RSSI_ONCHANNEL;
					}
					if (dtoh32(bss->length) != bi_length) {
						u32 prev_len = dtoh32(bss->length);

						WL_SCAN(("bss info replacement"
							" is occured(bcast:%d->probresp%d)\n",
							bss->ie_length, bi->ie_length));
						WL_DBG(("%s("MACSTR"), replacement!(%d -> %d)\n",
						bss->SSID, MAC2STR(bi->BSSID.octet),
						prev_len, bi_length));

						if (list->buflen - prev_len + bi_length
							> ESCAN_BUF_SIZE) {
							WL_ERR(("Buffer is too small: keep the"
								" previous result of this AP\n"));
							
							bss->RSSI = bi->RSSI;
							bss->flags |= (bi->flags
								& WL_BSS_FLAGS_RSSI_ONCHANNEL);
							goto exit;
						}

						if (i < list->count - 1) {
							
							memmove((u8 *)bss + bi_length,
								(u8 *)bss + prev_len,
								list->buflen - cur_len - prev_len);
						}
						list->buflen -= prev_len;
						list->buflen += bi_length;
					}
					list->version = dtoh32(bi->version);
					memcpy((u8 *)bss, (u8 *)bi, bi_length);
					goto exit;
				}
				cur_len += dtoh32(bss->length);
			}
			if (bi_length > ESCAN_BUF_SIZE - list->buflen) {
				WL_ERR(("Buffer is too small: ignoring\n"));
				goto exit;
			}
#if defined(DUAL_ESCAN_RESULT_BUFFER)
			memcpy(&(wl->escan_info.escan_buf[wl->escan_info.cur_sync_id%2]
				[list->buflen]), bi, bi_length);
#else
			memcpy(&(wl->escan_info.escan_buf[list->buflen]), bi, bi_length);
#endif
			list->version = dtoh32(bi->version);
			list->buflen += bi_length;
			list->count++;

		}

	}
	else if (status == WLC_E_STATUS_SUCCESS) {
		wl->escan_info.escan_state = WL_ESCAN_STATE_IDLE;
#if defined(DUAL_ESCAN_RESULT_BUFFER)
		if (escan_result->sync_id != wl->escan_info.cur_sync_id)
			WL_ERR(("Escan sync id mismatch: status %d "
			"cur_sync_id %d coming_sync_id %d\n",
			status, wl->escan_info.cur_sync_id, escan_result->sync_id));
#endif
		if (wl_get_drv_status_all(wl, FINDING_COMMON_CHANNEL)) {
			WL_INFO(("ACTION FRAME SCAN DONE\n"));
			wl_clr_p2p_status(wl, SCANNING);
			wl_clr_drv_status(wl, SCANNING, wl->afx_hdl->dev);
			if (wl->afx_hdl->peer_chan == WL_INVALID)
				complete(&wl->act_frm_scan);
		} else if (likely(wl->scan_request)) {
			WL_INFO(("ESCAN COMPLETED\n"));
#if defined(DUAL_ESCAN_RESULT_BUFFER)
			wl->bss_list = (wl_scan_results_t *)
				wl->escan_info.escan_buf[wl->escan_info.cur_sync_id%2];
#else
			wl->bss_list = (wl_scan_results_t *)wl->escan_info.escan_buf;
#endif
			wl_inform_bss(wl);
			wl_notify_escan_complete(wl, ndev, false, false);
		}
#if defined(DUAL_ESCAN_RESULT_BUFFER)
		wl->escan_info.cur_sync_id++;
#endif
	}
	else if (status == WLC_E_STATUS_ABORT) {
		wl->escan_info.escan_state = WL_ESCAN_STATE_IDLE;
#if defined(DUAL_ESCAN_RESULT_BUFFER)
		if (escan_result->sync_id != wl->escan_info.cur_sync_id)
			WL_ERR(("Escan sync id mismatch: status %d "
			"cur_sync_id %d coming_sync_id %d\n",
			status, wl->escan_info.cur_sync_id, escan_result->sync_id));
#endif
		if (wl_get_drv_status_all(wl, FINDING_COMMON_CHANNEL)) {
			WL_INFO(("ACTION FRAME SCAN DONE\n"));
			wl_clr_drv_status(wl, SCANNING, wl->afx_hdl->dev);
			wl_clr_p2p_status(wl, SCANNING);
			if (wl->afx_hdl->peer_chan == WL_INVALID)
				complete(&wl->act_frm_scan);
		} else if (likely(wl->scan_request)) {
			WL_INFO(("ESCAN ABORTED\n"));
#if defined(DUAL_ESCAN_RESULT_BUFFER)
			wl->bss_list = (wl_scan_results_t *)
				wl->escan_info.escan_buf[(wl->escan_info.cur_sync_id+1)%2];
#else
			wl->bss_list = (wl_scan_results_t *)wl->escan_info.escan_buf;
#endif
			wl_inform_bss(wl);
			wl_notify_escan_complete(wl, ndev, true, false);
		}
#if defined(DUAL_ESCAN_RESULT_BUFFER)
		wl->escan_info.cur_sync_id += 2;
#endif
	}
	else if (status == WLC_E_STATUS_NEWSCAN)
	{
		escan_result = (wl_escan_result_t *) data;
		WL_ERR(("WLC_E_STATUS_NEWSCAN : scan_request[%p]\n", wl->scan_request));
		WL_ERR(("sync_id[%d], bss_count[%d]\n", escan_result->sync_id,
			escan_result->bss_count));
	} else {
		WL_ERR(("unexpected Escan Event %d : abort\n", status));
		wl->escan_info.escan_state = WL_ESCAN_STATE_IDLE;
#if defined(DUAL_ESCAN_RESULT_BUFFER)
		if (escan_result->sync_id != wl->escan_info.cur_sync_id)
			WL_ERR(("Escan sync id mismatch: status %d "
			"cur_sync_id %d coming_sync_id %d\n",
			status, wl->escan_info.cur_sync_id, escan_result->sync_id));
#endif
		if (wl_get_drv_status_all(wl, FINDING_COMMON_CHANNEL)) {
			WL_INFO(("ACTION FRAME SCAN DONE\n"));
			wl_clr_p2p_status(wl, SCANNING);
			wl_clr_drv_status(wl, SCANNING, wl->afx_hdl->dev);
			if (wl->afx_hdl->peer_chan == WL_INVALID)
				complete(&wl->act_frm_scan);
		} else if (likely(wl->scan_request)) {
#if defined(DUAL_ESCAN_RESULT_BUFFER)
			wl->bss_list = (wl_scan_results_t *)
				wl->escan_info.escan_buf[(wl->escan_info.cur_sync_id+1)%2];
#else
			wl->bss_list = (wl_scan_results_t *)wl->escan_info.escan_buf;
#endif
			wl_inform_bss(wl);
			wl_notify_escan_complete(wl, ndev, true, false);
		}
#if defined(DUAL_ESCAN_RESULT_BUFFER)
		wl->escan_info.cur_sync_id += 2;
#endif
	}
exit:
	mutex_unlock(&wl->usr_sync);
	return err;
}

static s32 wl_notifier_change_state(struct wl_priv *wl, struct net_info *_net_info,
	enum wl_status state, bool set)
{
	s32 pm = PM_FAST;
	s32 err = BCME_OK;
	u32 chan = 0;
	u32 chanspec = 0;
	u32 prev_chan = 0;
	u32 connected_cnt  = 0;
	struct net_info *iter, *next;
	if (set) { 
		switch (state) {
		case WL_STATUS_CONNECTED: {
			if ((connected_cnt = wl_get_drv_status_all(wl, CONNECTED)) > 1) {
				pm = PM_OFF;
				WL_INFO(("Do not enable the power save for VSDB mode\n"));
			} else if (_net_info->pm_block) {
				pm = PM_OFF;
			} else {
				pm = PM_FAST;
			}
			for_each_ndev(wl, iter, next) {
				if ((connected_cnt == 1) && (iter->ndev != _net_info->ndev))
					continue;
				chanspec = 0;
				chan = 0;
				if (wl_get_drv_status(wl, CONNECTED, iter->ndev)) {
					if (wldev_iovar_getint(iter->ndev, "chanspec",
						(s32 *)&chanspec) == BCME_OK) {
						chan = CHSPEC_CHANNEL(chanspec);
						if (CHSPEC_IS40(chanspec)) {
							if (CHSPEC_SB_UPPER(chanspec))
								chan += CH_10MHZ_APART;
							else
								chan -= CH_10MHZ_APART;
						}
						wl_update_prof(wl, iter->ndev, NULL,
							&chan, WL_PROF_CHAN);
						printf("connect to channel: %d\n", chan);
					}
					if ((wl_get_mode_by_netdev(wl, iter->ndev)
						== WL_MODE_BSS)) {
						pm = htod32(pm);
						WL_DBG(("power save %s\n",
							(pm ? "enabled" : "disabled")));
						err = wldev_ioctl(iter->ndev, WLC_SET_PM,
							&pm, sizeof(pm), true);
						if (unlikely(err)) {
							if (err == -ENODEV)
								WL_DBG(("net_device"
									" is not ready\n"));
							else
								WL_ERR(("error"
									" (%d)\n", err));
								break;
						}
					}
					if (connected_cnt  > 1) {
						if (!prev_chan && chan)
							prev_chan = chan;
						else if (prev_chan && (prev_chan != chan)) {
							wl->vsdb_mode = true;
						}
						if (wl->roamoff_on_concurrent) {
							if ((err = wldev_iovar_getint(iter->ndev,
								"roam_off", (s32 *)&iter->roam_off))
								== BCME_OK) {
								if ((err =
								wldev_iovar_setint(iter->ndev,
									"roam_off", 1)) !=
									BCME_OK) {
									WL_ERR((" failed to set "
									"roam_off : %d\n", err));
								}
							} else
								WL_ERR(("failed to get"
									" roam_off : %d\n", err));
						}
					}
				}
			}
			break;
		}
		default:
			break;
		}
	} else { 
		switch (state) {
		case WL_STATUS_CONNECTED: {
			chan = 0;
			
			wl_update_prof(wl, _net_info->ndev, NULL, &chan, WL_PROF_CHAN);
			if (wl_get_drv_status_all(wl, CONNECTED) == 1) {
				wl->vsdb_mode = false;
				for_each_ndev(wl, iter, next) {
					if (wl_get_drv_status(wl, CONNECTED, iter->ndev) &&
						(wl_get_mode_by_netdev(wl, iter->ndev)
							 == WL_MODE_BSS)) {
						if (wl_get_netinfo_by_netdev(wl,
							iter->ndev)->pm_block)
							pm = PM_OFF;
						else
							pm = PM_FAST;
						pm = htod32(pm);
						WL_DBG(("power save %s\n",
							(pm ? "enabled" : "disabled")));
						err = wldev_ioctl(iter->ndev,
							WLC_SET_PM, &pm, sizeof(pm), true);
						if (unlikely(err)) {
							if (err == -ENODEV)
								WL_DBG(("net_device"
									" is not ready\n"));
							else
								WL_ERR(("error"
									" (%d)\n", err));
							break;
						}
					}
				}
			}
			if (wl->roamoff_on_concurrent) {
				for_each_ndev(wl, iter, next) {
					if ((iter->roam_off != WL_INVALID) &&
						((err = wldev_iovar_setint(iter->ndev, "roam_off",
						iter->roam_off)) == BCME_OK)) {
						iter->roam_off = WL_INVALID;
					} else
						WL_ERR((" failed to set roam_off : %d\n", err));
				}
			}
			break;
		}
		default:
			break;
		}
	}
	return err;
}
static s32 wl_init_scan(struct wl_priv *wl)
{
	struct wl_iscan_ctrl *iscan = wl_to_iscan(wl);
	int err = 0;

	if (wl->iscan_on) {
		iscan->dev = wl_to_prmry_ndev(wl);
		iscan->state = WL_ISCAN_STATE_IDLE;
		wl_init_iscan_handler(iscan);
		iscan->timer_ms = WL_ISCAN_TIMER_INTERVAL_MS;
		init_timer(&iscan->timer);
		iscan->timer.data = (unsigned long) iscan;
		iscan->timer.function = wl_iscan_timer;
		sema_init(&iscan->sync, 0);
		iscan->tsk = kthread_run(wl_iscan_thread, iscan, "wl_iscan");
		if (IS_ERR(iscan->tsk)) {
			WL_ERR(("Could not create iscan thread\n"));
			iscan->tsk = NULL;
			return -ENOMEM;
		}
		iscan->data = wl;
	} else if (wl->escan_on) {
		wl->evt_handler[WLC_E_ESCAN_RESULT] = wl_escan_handler;
		wl->escan_info.escan_state = WL_ESCAN_STATE_IDLE;
	}
	
	init_timer(&wl->scan_timeout);
	wl->scan_timeout.data = (unsigned long) wl;
	wl->scan_timeout.function = wl_scan_timeout;

	return err;
}

static s32 wl_init_priv(struct wl_priv *wl)
{
	struct wiphy *wiphy = wl_to_wiphy(wl);
	struct net_device *ndev = wl_to_prmry_ndev(wl);
	s32 err = 0;

	wl->scan_request = NULL;
	wl->pwr_save = !!(wiphy->flags & WIPHY_FLAG_PS_ON_BY_DEFAULT);
	wl->iscan_on = false;
	wl->escan_on = true;
	wl->roam_on = true;
	wl->iscan_kickstart = false;
	wl->active_scan = true;
	wl->rf_blocked = false;
	wl->vsdb_mode = false;
	wl->wlfc_on = false;
	wl->roamoff_on_concurrent = true;
	
	set_bit(WL_STATUS_CONNECTED, &wl->interrested_state);
	spin_lock_init(&wl->cfgdrv_lock);
	mutex_init(&wl->ioctl_buf_sync);
	init_waitqueue_head(&wl->netif_change_event);
	init_completion(&wl->send_af_done);
	init_completion(&wl->iface_disable);
	wl_init_eq(wl);
	err = wl_init_priv_mem(wl);
	if (err) 
		goto wl_init_priv_fail;
	
	if (wl_create_event_handler(wl)) {
		err = -ENOMEM;
		goto wl_init_priv_fail1;
	}
	wl_init_event_handler(wl);
	mutex_init(&wl->usr_sync);
	err = wl_init_scan(wl);
	if (err)
		goto wl_init_priv_fail2;
	wl_init_conf(wl->conf);
	wl_init_prof(wl, ndev);
	wl_link_down(wl);
	DNGL_FUNC(dhd_cfg80211_init, (wl));
	return err;

wl_init_priv_fail2:
	wl_destroy_event_handler(wl);
wl_init_priv_fail1:
	wl_deinit_priv_mem(wl);
wl_init_priv_fail:
	wl_flush_eq(wl);

	return err;
}

static void wl_deinit_priv(struct wl_priv *wl)
{
	DNGL_FUNC(dhd_cfg80211_deinit, (wl));
	wl_destroy_event_handler(wl);
	wl_flush_eq(wl);
	wl_link_down(wl);
	del_timer_sync(&wl->scan_timeout);
	wl_term_iscan(wl);
	wl_deinit_priv_mem(wl);
	unregister_netdevice_notifier(&wl_cfg80211_netdev_notifier);
}

#if defined(WLP2P) && defined(WL_ENABLE_P2P_IF)
static s32 wl_cfg80211_attach_p2p(void)
{
	struct wl_priv *wl = wlcfg_drv_priv;

	WL_TRACE(("Enter \n"));

	if (wl_cfgp2p_register_ndev(wl) < 0) {
		WL_ERR(("%s: P2P attach failed. \n", __func__));
		return -ENODEV;
	}

	return 0;
}

static s32  wl_cfg80211_detach_p2p(void)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct wireless_dev *wdev = wl->p2p_wdev;

	WL_DBG(("Enter \n"));
	if (!wdev || !wl) {
		WL_ERR(("Invalid Ptr\n"));
		return -EINVAL;
	}

	wl_cfgp2p_unregister_ndev(wl);

	wl->p2p_wdev = NULL;
	wl->p2p_net = NULL;
	WL_DBG(("Freeing 0x%08x \n", (unsigned int)wdev));
	kfree(wdev);

	return 0;
}
#endif 

s32 wl_cfg80211_attach_post(struct net_device *ndev)
{
	struct wl_priv * wl = NULL;
	s32 err = 0;
	WL_TRACE(("In\n"));
	if (unlikely(!ndev)) {
		WL_ERR(("ndev is invaild\n"));
		return -ENODEV;
	}
	wl = wlcfg_drv_priv;
	if (wl && !wl_get_drv_status(wl, READY, ndev)) {
			if (wl->wdev &&
				wl_cfgp2p_supported(wl, ndev)) {
#if !defined(WL_ENABLE_P2P_IF)
				wl->wdev->wiphy->interface_modes |=
					(BIT(NL80211_IFTYPE_P2P_CLIENT)|
					BIT(NL80211_IFTYPE_P2P_GO));
#endif
				if ((err = wl_cfgp2p_init_priv(wl)) != 0)
					goto fail;

#if defined(WLP2P) && defined(WL_ENABLE_P2P_IF)
				if (wl->p2p_net) {
					
					memcpy(wl->p2p_net->dev_addr, ndev->dev_addr, ETH_ALEN);
					wl->p2p_net->dev_addr[0] |= 0x02;
					printk("%s: p2p_dev_addr="MACSTR "\n",
						wl->p2p_net->name, MAC2STR(wl->p2p_net->dev_addr));
				} else {
					WL_ERR(("p2p_net not yet populated."
					" Couldn't update the MAC Address for p2p0 \n"));
					return -ENODEV;
				}
#endif 

				wl->p2p_supported = true;
			}
	} else
		return -ENODEV;
	wl_set_drv_status(wl, READY, ndev);
fail:
	return err;
}

s32 wl_cfg80211_attach(struct net_device *ndev, void *data)
{
	struct wireless_dev *wdev;
	struct wl_priv *wl;
	s32 err = 0;
	struct device *dev;

	WL_TRACE(("In\n"));
	if (!ndev) {
		WL_ERR(("ndev is invaild\n"));
		return -ENODEV;
	}
	WL_DBG(("func %p\n", wl_cfg80211_get_parent_dev()));
	dev = wl_cfg80211_get_parent_dev();

	wdev = kzalloc(sizeof(*wdev), GFP_KERNEL);
	if (unlikely(!wdev)) {
		WL_ERR(("Could not allocate wireless device\n"));
		return -ENOMEM;
	}
	err = wl_setup_wiphy(wdev, dev);
	if (unlikely(err)) {
		kfree(wdev);
		return -ENOMEM;
	}
	wdev->iftype = wl_mode_to_nl80211_iftype(WL_MODE_BSS);
	wl = (struct wl_priv *)wiphy_priv(wdev->wiphy);
	wl->wdev = wdev;
	wl->pub = data;
	INIT_LIST_HEAD(&wl->net_list);
	ndev->ieee80211_ptr = wdev;
	SET_NETDEV_DEV(ndev, wiphy_dev(wdev->wiphy));
	wdev->netdev = ndev;
	wl->state_notifier = wl_notifier_change_state;
	err = wl_alloc_netinfo(wl, ndev, wdev, WL_MODE_BSS, PM_ENABLE);
	if (err) {
		WL_ERR(("Failed to alloc net_info (%d)\n", err));
		goto cfg80211_attach_out;
	}
	err = wl_init_priv(wl);
	if (err) {
		WL_ERR(("Failed to init iwm_priv (%d)\n", err));
		goto cfg80211_attach_out;
	}

	err = wl_setup_rfkill(wl, TRUE);
	if (err) {
		WL_ERR(("Failed to setup rfkill %d\n", err));
		goto cfg80211_attach_out;
	}
	err = register_netdevice_notifier(&wl_cfg80211_netdev_notifier);
	if (err) {
		WL_ERR(("Failed to register notifierl %d\n", err));
		goto cfg80211_attach_out1;
	}
#if defined(COEX_DHCP)
	if (wl_cfg80211_btcoex_init(wl))
		goto cfg80211_attach_out2;
#endif 

	wlcfg_drv_priv = wl;

#if defined(WLP2P) && defined(WL_ENABLE_P2P_IF)
	err = wl_cfg80211_attach_p2p();
	if (err)
		goto cfg80211_attach_out3;
#endif

	return err;

#if defined(WLP2P) && defined(WL_ENABLE_P2P_IF)
cfg80211_attach_out3:
#endif
#if defined(COEX_DHCP)
	wl_cfg80211_btcoex_deinit(wl);
cfg80211_attach_out2:
#endif
	unregister_netdevice_notifier(&wl_cfg80211_netdev_notifier);
cfg80211_attach_out1:
	err = wl_setup_rfkill(wl, FALSE);
cfg80211_attach_out:
	wl_free_wdev(wl);
	return err;
}

void wl_cfg80211_detach(void *para)
{
	struct wl_priv *wl;

	(void)para;
	wl = wlcfg_drv_priv;

	WL_TRACE(("In\n"));

#if defined(COEX_DHCP)
	wl_cfg80211_btcoex_deinit(wl);
#endif 

#if defined(WLP2P) && defined(WL_ENABLE_P2P_IF)
	wl_cfg80211_detach_p2p();
#endif
	wl_setup_rfkill(wl, FALSE);
	if (wl->p2p_supported) {
		WL_ERR(("wl_cfgp2p_down() is not called yet\n"));
		wl_cfgp2p_down(wl);
	}
	wl_deinit_priv(wl);
	wlcfg_drv_priv = NULL;
	wl_cfg80211_clear_parent_dev();
	wl_free_wdev(wl);
}

static void wl_wakeup_event(struct wl_priv *wl)
{
	if (wl->event_tsk.thr_pid >= 0) {
		DHD_OS_WAKE_LOCK(wl->pub);
		up(&wl->event_tsk.sema);
	}
}

static int wl_is_p2p_event(struct wl_event_q *e)
{
	switch (e->etype) {
	case WLC_E_P2P_PROBREQ_MSG:
	case WLC_E_P2P_DISC_LISTEN_COMPLETE:
	case WLC_E_ACTION_FRAME_RX:
	case WLC_E_ACTION_FRAME_OFF_CHAN_COMPLETE:
	case WLC_E_ACTION_FRAME_COMPLETE:

		if (e->emsg.ifidx != 0) {
			WL_TRACE(("P2P Event on Virtual I/F (ifidx:%d) \n",
			e->emsg.ifidx));
			return FALSE;
		} else {
			WL_TRACE(("P2P Event on Primary I/F (ifidx:%d)."
				" Sent it to p2p0 \n", e->emsg.ifidx));
			return TRUE;
		}
		break;

	default:
		WL_TRACE(("NON-P2P Event %d on ifidx (ifidx:%d) \n",
			e->etype, e->emsg.ifidx));
		return FALSE;
	}
}

#include "footprint.h"

static s32 wl_event_handler(void *data)
{
	struct net_device *netdev;
	struct wl_priv *wl = NULL;
	struct wl_event_q *e;
	tsk_ctl_t *tsk = (tsk_ctl_t *)data;

	wl = (struct wl_priv *)tsk->parent;
#ifndef USE_KTHREAD_API
	DAEMONIZE("dhd_cfg80211_event");
	complete(&tsk->completed);
#endif
	set_wifi_foot_print(0x1, 0x18);
	while (down_interruptible (&tsk->sema) == 0) {
		SMP_RD_BARRIER_DEPENDS();
		if (tsk->terminated)
			break;
		while ((e = wl_deq_event(wl))) {
			set_wifi_foot_print(0x2, 0x18);
			WL_DBG(("event type (%d), if idx: %d\n", e->etype, e->emsg.ifidx));
			set_wifi_foot_print(e->etype, 0x1C);
			if ((wl_is_p2p_event(e) == TRUE) && (wl->p2p_net)) {
				netdev = wl->p2p_net;
			} else {
				netdev = dhd_idx2net((struct dhd_pub *)(wl->pub), e->emsg.ifidx);
			}
			if (!netdev)
				netdev = wl_to_prmry_ndev(wl);
			if (e->etype < WLC_E_LAST && wl->evt_handler[e->etype]) {
				wl->evt_handler[e->etype] (wl, netdev, &e->emsg, e->edata);
			} else {
				WL_DBG(("Unknown Event (%d): ignoring\n", e->etype));
			}
			wl_put_event(e);
		}
		set_wifi_foot_print(0x3, 0x18);
		DHD_OS_WAKE_UNLOCK(wl->pub);
		set_wifi_foot_print(0x4, 0x18);
	}
	set_wifi_foot_print(0x5, 0x18);
	WL_ERR(("%s was terminated\n", __func__));
	complete_and_exit(&tsk->completed, 0);
	return 0;
}

void wl_cfg80211_send_priv_event
(
	struct net_device *dev,
	char *flag
)
{
#if defined(CONFIG_WIRELESS_EXT)
	wl_iw_send_priv_event(dev, flag);
#endif
	return;
}

#if defined(HTC_TX_TRACKING)
static s32 wl_notify_txfail(struct wl_priv *wl, struct net_device *ndev, const wl_event_msg_t *e, void *data)
{
        printf("Tx fail!!");
        wl_cfg80211_send_priv_event(ndev, "TX_FAIL");
        return 0;
}
#endif

int hotspot_hight_ind = 0;

#if defined(SOFTAP)
static void wl_cfg80211_hotspot_event_process(struct net_device *ndev, const wl_event_msg_t *e, void *data)
{
	u32 event_type = ntoh32(e->event_type);
	uint16 flags =  ntoh16(e->flags);
	uint32 datalen = ntoh32(e->datalen);
	uint32 status =  ntoh32(e->status);

	switch (event_type) {
	case WLC_E_PRUNE:
		{
			char *macaddr = (char *)&e->addr;
			char mac_buf[32] = {0};
			WL_INFO(("PRUNE received, %02X:%02X:%02X:%02X:%02X:%02X!\n",
				macaddr[0], macaddr[1], macaddr[2], macaddr[3],
				macaddr[4], macaddr[5]));

			if (status) {
				printf("reach max!!\n");
				break;
			}

			if (wl_android_black_list_match(macaddr)) {
				printf("STA block\n");
				sprintf(mac_buf, "STA_BLOCK %02X:%02X:%02X:%02X:%02X:%02X",
					macaddr[0], macaddr[1], macaddr[2],
					macaddr[3], macaddr[4], macaddr[5]);
					wl_iw_send_priv_event(ndev, mac_buf);
			}
		}
			break;
		case WLC_E_JOIN:
		case WLC_E_ASSOC_IND:
		case WLC_E_REASSOC_IND:
			printf("STA connect received %d\n", event_type);
			wl_cfg80211_send_priv_event(ndev, "STA_JOIN");
			break;
		case WLC_E_DEAUTH_IND:
		case WLC_E_DISASSOC_IND:
			printf("STA disconnect received %d\n", event_type);
			wl_cfg80211_send_priv_event(ndev, "STA_LEAVE");
			break;
		case WLC_E_LINK:
		case WLC_E_NDIS_LINK:
			if(block_ap_event) {
				printf("Block ap event\n");
				break;
			}
			if (!(flags & WLC_EVENT_MSG_LINK)) {
				printf("AP DOWN %d\n", event_type);
				wl_cfg80211_send_priv_event(ndev, "AP_DOWN");
			} else {
				printf("AP UP %d\n", event_type);
				dhd_APUP = true;
				wl_cfg80211_send_priv_event(ndev, "AP_UP");
			}
			break;

		case WLC_E_LOAD_IND:
			printf("WLC_E_LOAD_IND, %d\n", datalen);
			if (datalen > IW_CUSTOM_MAX)
				break;

            if (status) {
                    printf("HIGH INDICATE!!\n");
                    wlan_lock_perf();
					wl_iw_send_priv_event(ndev, "PERF_LOCK");
					hotspot_hight_ind = 1;
            } else {
                    printf("LOW INDICATE!!\n");
				    wlan_unlock_perf();
					wl_iw_send_priv_event(ndev, "PERF_UNLOCK");
					hotspot_hight_ind = 0;
            }

		break;
		default:
			break;
		}
	return;
}
#endif
void
wl_cfg80211_event(struct net_device *ndev, const wl_event_msg_t * e, void *data)
{
	u32 event_type = ntoh32(e->event_type);
	struct wl_priv *wl = wlcfg_drv_priv;
	wl_iw_t *iw = *(wl_iw_t **)netdev_priv(ndev);

	char *macaddr = (char *)&e->addr;
	u8 *curbssid;
	u32 reason = ntoh32(e->reason);

#if (WL_DBG_LEVEL > 0)
	s8 *estr = (event_type <= sizeof(wl_dbg_estr) / WL_DBG_ESTR_MAX - 1) ?
	    wl_dbg_estr[event_type] : (s8 *) "Unknown";
	WL_DBG(("event_type (%d):" "WLC_E_" "%s\n", event_type, estr));
#endif 

	if (event_type == WLC_E_PFN_NET_FOUND) {
		WL_INFO((" PNOEVENT: PNO_NET_FOUND\n"));
		WAKE_LOCK_TIMEOUT(iw->pub, 30);
	}
	else if (event_type == WLC_E_PFN_NET_LOST) {
		WL_INFO((" PNOEVENT: PNO_NET_LOST\n"));
	}
#if 0
	else if (event_type == WLC_E_RELOAD)
		WL_ERR(("Reload Event\n"));
#endif
#if defined(SOFTAP)
	if (dhd_check_ap_mode_set(wl->pub))
		wl_cfg80211_hotspot_event_process(ndev, e, data);
#endif

	
	switch (event_type) {
	case WLC_E_DEAUTH:
	{
		printf("DEAUTH received, %02X:%02X:%02X:%02X:%02X:%02X!, reason:%d\n",
					macaddr[0], macaddr[1], macaddr[2], macaddr[3],
					macaddr[4], macaddr[5], reason);
		
		curbssid = wl_read_prof(wl, ndev, WL_PROF_BSSID);
		
		if((wl_get_mode_by_netdev(wl, ndev) == WL_MODE_BSS) &&
			   strncmp(curbssid,macaddr,ETHER_ADDR_LEN)){
			printf("Rece Deauth from not connected bssid, skip it ndev_name[%s]!\n",ndev->name);
			return;
		}
		break;
	}
	case WLC_E_DEAUTH_IND:
	{
		printf("DEAUTH_IND received, %02X:%02X:%02X:%02X:%02X:%02X!, reason:%d\n",
				macaddr[0], macaddr[1], macaddr[2], macaddr[3],
				macaddr[4], macaddr[5], reason);
		break;
	}
	case WLC_E_DISASSOC:
	{
		printf("DISASSOC received, %02X:%02X:%02X:%02X:%02X:%02X!, reason:%d\n",
				macaddr[0], macaddr[1], macaddr[2], macaddr[3],
				macaddr[4], macaddr[5], reason);
		break;
	}
	case WLC_E_DISASSOC_IND:
	{
		printf("DISASSOC_IND received, %02X:%02X:%02X:%02X:%02X:%02X!, reason:%d\n",
				macaddr[0], macaddr[1], macaddr[2], macaddr[3],
				macaddr[4], macaddr[5], reason);
		break;
	}
	default:
		break;

	}
	

	if (likely(!wl_enq_event(wl, ndev, event_type, e, data)))
		wl_wakeup_event(wl);
}

static void wl_init_eq(struct wl_priv *wl)
{
	wl_init_eq_lock(wl);
	INIT_LIST_HEAD(&wl->eq_list);
}

static void wl_flush_eq(struct wl_priv *wl)
{
	struct wl_event_q *e;
	unsigned long flags;

	flags = wl_lock_eq(wl);
	while (!list_empty(&wl->eq_list)) {
		e = list_first_entry(&wl->eq_list, struct wl_event_q, eq_list);
		list_del(&e->eq_list);
		kfree(e);
	}
	wl_unlock_eq(wl, flags);
}


static struct wl_event_q *wl_deq_event(struct wl_priv *wl)
{
	struct wl_event_q *e = NULL;
	unsigned long flags;

	flags = wl_lock_eq(wl);
	if (likely(!list_empty(&wl->eq_list))) {
		e = list_first_entry(&wl->eq_list, struct wl_event_q, eq_list);
		list_del(&e->eq_list);
	}
	wl_unlock_eq(wl, flags);

	return e;
}


static s32
wl_enq_event(struct wl_priv *wl, struct net_device *ndev, u32 event, const wl_event_msg_t *msg,
	void *data)
{
	struct wl_event_q *e;
	s32 err = 0;
	uint32 evtq_size;
	uint32 data_len;
	unsigned long flags;
	gfp_t aflags;

	data_len = 0;
	if (data)
		data_len = ntoh32(msg->datalen);
	evtq_size = sizeof(struct wl_event_q) + data_len;
	aflags = (in_atomic()) ? GFP_ATOMIC : GFP_KERNEL;
	e = kzalloc(evtq_size, aflags);
	if (unlikely(!e)) {
		WL_ERR(("event alloc failed\n"));
		return -ENOMEM;
	}
	
	if(e != NULL)
	
		e->etype = event;
	memcpy(&e->emsg, msg, sizeof(wl_event_msg_t));
	if (data)
		memcpy(e->edata, data, data_len);
	flags = wl_lock_eq(wl);
	list_add_tail(&e->eq_list, &wl->eq_list);
	wl_unlock_eq(wl, flags);

	return err;
}

static void wl_put_event(struct wl_event_q *e)
{
	kfree(e);
}

static s32 wl_config_ifmode(struct wl_priv *wl, struct net_device *ndev, s32 iftype)
{
	s32 infra = 0;
	s32 err = 0;
	s32 mode = 0;
	switch (iftype) {
	case NL80211_IFTYPE_MONITOR:
	case NL80211_IFTYPE_WDS:
		WL_ERR(("type (%d) : currently we do not support this mode\n",
			iftype));
		err = -EINVAL;
		return err;
	case NL80211_IFTYPE_ADHOC:
		mode = WL_MODE_IBSS;
		break;
	case NL80211_IFTYPE_STATION:
	case NL80211_IFTYPE_P2P_CLIENT:
		mode = WL_MODE_BSS;
		infra = 1;
		break;
	case NL80211_IFTYPE_AP:
	case NL80211_IFTYPE_P2P_GO:
		mode = WL_MODE_AP;
		infra = 1;
		break;
	default:
		err = -EINVAL;
		WL_ERR(("invalid type (%d)\n", iftype));
		return err;
	}
	infra = htod32(infra);
	err = wldev_ioctl(ndev, WLC_SET_INFRA, &infra, sizeof(infra), true);
	if (unlikely(err)) {
		WL_ERR(("WLC_SET_INFRA error (%d)\n", err));
		return err;
	}

	wl_set_mode_by_netdev(wl, ndev, mode);

	return 0;
}

s32 wl_add_remove_eventmsg(struct net_device *ndev, u16 event, bool add)
{
	s8 iovbuf[WL_EVENTING_MASK_LEN + 12];

	s8 eventmask[WL_EVENTING_MASK_LEN];
	s32 err = 0;

	
	bcm_mkiovar("event_msgs", NULL, 0, iovbuf,
		sizeof(iovbuf));
	err = wldev_ioctl(ndev, WLC_GET_VAR, iovbuf, sizeof(iovbuf), false);
	if (unlikely(err)) {
		WL_ERR(("Get event_msgs error (%d)\n", err));
		goto eventmsg_out;
	}
	memcpy(eventmask, iovbuf, WL_EVENTING_MASK_LEN);
	if (add) {
		setbit(eventmask, event);
	} else {
		clrbit(eventmask, event);
	}
	bcm_mkiovar("event_msgs", eventmask, WL_EVENTING_MASK_LEN, iovbuf,
		sizeof(iovbuf));
	err = wldev_ioctl(ndev, WLC_SET_VAR, iovbuf, sizeof(iovbuf), true);
	if (unlikely(err)) {
		WL_ERR(("Set event_msgs error (%d)\n", err));
		goto eventmsg_out;
	}

eventmsg_out:
	return err;

}

static int wl_construct_reginfo(struct wl_priv *wl, s32 bw_cap)
{
	struct net_device *dev = wl_to_prmry_ndev(wl);
	struct ieee80211_channel *band_chan_arr = NULL;
	wl_uint32_list_t *list;
	u32 i, j, index, n_2g, n_5g, band, channel, array_size;
	u32 *n_cnt = NULL;
	chanspec_t c = 0;
	s32 err = BCME_OK;
	bool update;
	bool ht40_allowed;
	u8 *pbuf = NULL;

#define LOCAL_BUF_LEN 1024
	pbuf = kzalloc(LOCAL_BUF_LEN, GFP_KERNEL);
	if (pbuf == NULL) {
		WL_ERR(("failed to allocate local buf\n"));
		return -ENOMEM;
	}

	list = (wl_uint32_list_t *)(void *)pbuf;
	list->count = htod32(WL_NUMCHANSPECS);

	err = wldev_iovar_getbuf_bsscfg(dev, "chanspecs", NULL,
		0, pbuf, LOCAL_BUF_LEN, 0, &wl->ioctl_buf_sync);
	if (err != 0) {
		WL_ERR(("get chanspecs failed with %d\n", err));
		kfree(pbuf);
		return err;
	}
#undef LOCAL_BUF_LEN

	list = (wl_uint32_list_t *)(void *)pbuf;
	band = array_size = n_2g = n_5g = 0;
	for (i = 0; i < dtoh32(list->count); i++) {
		index = 0;
		update = FALSE;
		ht40_allowed = FALSE;
		c = (chanspec_t)dtoh32(list->element[i]);
		c = wl_chspec_driver_to_host(c);
		channel = CHSPEC_CHANNEL(c);
		if (CHSPEC_IS40(c)) {
			if (CHSPEC_SB_UPPER(c))
				channel += CH_10MHZ_APART;
			else
				channel -= CH_10MHZ_APART;
		}

		if (CHSPEC_IS2G(c) && channel <= CH_MAX_2G_CHANNEL) {
			band_chan_arr = __wl_2ghz_channels;
			array_size = ARRAYSIZE(__wl_2ghz_channels);
			n_cnt = &n_2g;
			band = IEEE80211_BAND_2GHZ;
			ht40_allowed = (bw_cap == WLC_N_BW_40ALL) ? TRUE : FALSE;
		} else if (CHSPEC_IS5G(c) && channel > CH_MAX_2G_CHANNEL) {
			band_chan_arr = __wl_5ghz_a_channels;
			array_size = ARRAYSIZE(__wl_5ghz_a_channels);
			n_cnt = &n_5g;
			band = IEEE80211_BAND_5GHZ;
			ht40_allowed = (bw_cap == WLC_N_BW_20ALL) ? FALSE : TRUE;
		}
		else {
			WL_ERR(("Invalid Channel received %x chspec %x\n", channel, c));
			kfree(pbuf);
			return BCME_ERROR;
		}

		for (j = 0; (j < *n_cnt && (*n_cnt < array_size)); j++) {
			if (band_chan_arr[j].hw_value == channel) {
				update = TRUE;
				break;
			}
		}

		if (update)
			index = j;
		else
			index = *n_cnt;

		if (index <  array_size) {
#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 38) && !defined(WL_COMPAT_WIRELESS)
			band_chan_arr[index].center_freq =
				ieee80211_channel_to_frequency(channel);
			(void)band;
#else
			band_chan_arr[index].center_freq =
				ieee80211_channel_to_frequency(channel, band);
#endif
			band_chan_arr[index].hw_value = channel;

			if (CHSPEC_IS40(c) && ht40_allowed) {
				u32 ht40_flag = band_chan_arr[index].flags & IEEE80211_CHAN_NO_HT40;
				if (CHSPEC_SB_UPPER(c)) {
					if (ht40_flag == IEEE80211_CHAN_NO_HT40)
						band_chan_arr[index].flags &=
							~IEEE80211_CHAN_NO_HT40;
					band_chan_arr[index].flags |= IEEE80211_CHAN_NO_HT40PLUS;
				} else {
					band_chan_arr[index].flags &= ~IEEE80211_CHAN_NO_HT40;
					if (ht40_flag == IEEE80211_CHAN_NO_HT40)
						band_chan_arr[index].flags |=
							IEEE80211_CHAN_NO_HT40MINUS;
				}
			} else {
				band_chan_arr[index].flags = IEEE80211_CHAN_NO_HT40;
				if (band == IEEE80211_BAND_2GHZ)
					channel |= WL_CHANSPEC_BAND_2G;
				else
					channel |= WL_CHANSPEC_BAND_5G;
				channel |= WL_CHANSPEC_BW_20;
				channel = wl_chspec_host_to_driver(channel);
				err = wldev_iovar_getint(dev, "per_chan_info", &channel);
				if (!err) {
					if (channel & WL_CHAN_RADAR)
						band_chan_arr[index].flags |=
							(IEEE80211_CHAN_RADAR |
							IEEE80211_CHAN_NO_IBSS);
					if (channel & WL_CHAN_PASSIVE)
						band_chan_arr[index].flags |=
							IEEE80211_CHAN_PASSIVE_SCAN;
				}
			}
			if (!update)
				(*n_cnt)++;
		}
	}

	__wl_band_2ghz.n_channels = n_2g;
	__wl_band_5ghz_a.n_channels = n_5g;

	kfree(pbuf);
	return err;
}

s32 wl_update_wiphybands(struct wl_priv *wl)
{
	struct wiphy *wiphy;
	struct net_device *dev;
	u32 bandlist[3];
	u32 nband = 0;
	u32 i = 0;
	s32 err = 0;
	s32 index = 0;
	s32 nmode = 0;
	s32 bw_cap = 0;
	s32 cur_band = -1;

	if (wl == NULL)
		wl = wlcfg_drv_priv;
	dev = wl_to_prmry_ndev(wl);

	memset(bandlist, 0, sizeof(bandlist));
	err = wldev_ioctl(dev, WLC_GET_BANDLIST, bandlist,
		sizeof(bandlist), false);
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
		return err;
	}

	err = wldev_ioctl(dev, WLC_GET_BAND, &cur_band,
		sizeof(s32), false);
	if (unlikely(err)) {
		WL_ERR(("error (%d)\n", err));
		return err;
	}

	err = wldev_iovar_getint(dev, "nmode", &nmode);
	if (err) {
		return err;
	}

	err = wldev_iovar_getint(dev, "mimo_bw_cap", &bw_cap);
	if (err) {
		return err;
	}

	err = wl_construct_reginfo(wl, bw_cap);
	if (err) {
		WL_ERR(("wl_construct_reginfo() fails err=%d\n", err));
		return err;
	}
	wiphy = wl_to_wiphy(wl);
	nband = bandlist[0];
	wiphy->bands[IEEE80211_BAND_5GHZ] = NULL;
	wiphy->bands[IEEE80211_BAND_2GHZ] = NULL;
	for (i = 1; i <= nband && i < ARRAYSIZE(bandlist); i++) {
		index = -1;
		if (bandlist[i] == WLC_BAND_5G && __wl_band_5ghz_a.n_channels > 0) {
			wiphy->bands[IEEE80211_BAND_5GHZ] =
				&__wl_band_5ghz_a;
			index = IEEE80211_BAND_5GHZ;
			if (bw_cap == WLC_N_BW_40ALL || bw_cap == WLC_N_BW_20IN2G_40IN5G)
				wiphy->bands[index]->ht_cap.cap |= IEEE80211_HT_CAP_SGI_40;
		} else if (bandlist[i] == WLC_BAND_2G && __wl_band_2ghz.n_channels > 0) {
			wiphy->bands[IEEE80211_BAND_2GHZ] =
				&__wl_band_2ghz;
			index = IEEE80211_BAND_2GHZ;
			if (bw_cap == WLC_N_BW_40ALL)
				wiphy->bands[index]->ht_cap.cap |= IEEE80211_HT_CAP_SGI_40;
		}

		if ((index >= 0) && nmode) {
			wiphy->bands[index]->ht_cap.cap |=
				(IEEE80211_HT_CAP_SGI_20 | IEEE80211_HT_CAP_DSSSCCK40);
			wiphy->bands[index]->ht_cap.ht_supported = TRUE;
			wiphy->bands[index]->ht_cap.ampdu_factor = IEEE80211_HT_MAX_AMPDU_64K;
			wiphy->bands[index]->ht_cap.ampdu_density = IEEE80211_HT_MPDU_DENSITY_16;
		}
	}

	wiphy_apply_custom_regulatory(wiphy, &brcm_regdom);
	return err;
}

static s32 __wl_cfg80211_up(struct wl_priv *wl)
{
	s32 err = 0;
	struct net_device *ndev = wl_to_prmry_ndev(wl);
	struct wireless_dev *wdev = ndev->ieee80211_ptr;

	WL_DBG(("In\n"));

#if 0
	err = dhd_config_dongle(wl, false);
	if (unlikely(err))
		return err;
#endif

	err = wl_config_ifmode(wl, ndev, wdev->iftype);
	if (unlikely(err && err != -EINPROGRESS)) {
		WL_ERR(("wl_config_ifmode failed\n"));
	}
	err = wl_update_wiphybands(wl);
	if (unlikely(err)) {
		WL_ERR(("wl_update_wiphybands failed\n"));
	}

	err = dhd_monitor_init(wl->pub);
	err = wl_invoke_iscan(wl);
	wl_set_drv_status(wl, READY, ndev);
	return err;
}

static s32 __wl_cfg80211_down(struct wl_priv *wl)
{
	s32 err = 0;
	unsigned long flags;
	struct net_info *iter, *next;
	struct net_device *ndev = wl_to_prmry_ndev(wl);
#ifdef WL_ENABLE_P2P_IF
	struct wiphy *wiphy = wl_to_prmry_ndev(wl)->ieee80211_ptr->wiphy;
	struct net_device *p2p_net = wl->p2p_net;
#endif

	WL_DBG(("In\n"));
	
	if (!wl_get_drv_status(wl, READY, ndev))
		return err;	
	for_each_ndev(wl, iter, next)
		wl_set_drv_status(wl, SCAN_ABORTING, iter->ndev);

	wl_term_iscan(wl);
	spin_lock_irqsave(&wl->cfgdrv_lock, flags);
	if (wl->scan_request) {
		cfg80211_scan_done(wl->scan_request, true);
		wl->scan_request = NULL;
	}
	spin_unlock_irqrestore(&wl->cfgdrv_lock, flags);
	for_each_ndev(wl, iter, next) {
		wl_clr_drv_status(wl, READY, iter->ndev);
		wl_clr_drv_status(wl, SCANNING, iter->ndev);
		wl_clr_drv_status(wl, SCAN_ABORTING, iter->ndev);
		wl_clr_drv_status(wl, CONNECTING, iter->ndev);
		wl_clr_drv_status(wl, CONNECTED, iter->ndev);
		wl_clr_drv_status(wl, DISCONNECTING, iter->ndev);
		wl_clr_drv_status(wl, AP_CREATED, iter->ndev);
		wl_clr_drv_status(wl, AP_CREATING, iter->ndev);
	}
	wl_to_prmry_ndev(wl)->ieee80211_ptr->iftype =
		NL80211_IFTYPE_STATION;
#ifdef WL_ENABLE_P2P_IF
	wiphy->interface_modes = (wiphy->interface_modes)
					& (~(BIT(NL80211_IFTYPE_P2P_CLIENT)|
					BIT(NL80211_IFTYPE_P2P_GO)));
	if ((p2p_net) && (p2p_net->flags & IFF_UP)) {
		
		p2p_net->flags &= ~IFF_UP;
	}
#endif 

	DNGL_FUNC(dhd_cfg80211_down, (wl));
	wl_flush_eq(wl);
	wl_link_down(wl);
	if (wl->p2p_supported)
		wl_cfgp2p_down(wl);
	dhd_monitor_uninit();

	return err;
}

s32 wl_cfg80211_up(void *para)
{
	struct wl_priv *wl;
	s32 err = 0;
	int val = 1;
	dhd_pub_t *dhd;

	(void)para;
	WL_DBG(("In\n"));
	wl = wlcfg_drv_priv;

	if ((err = wldev_ioctl(wl_to_prmry_ndev(wl), WLC_GET_VERSION, &val,
		sizeof(int), false) < 0)) {
		WL_ERR(("WLC_GET_VERSION failed, err=%d\n", err));
		return err;
	}
	val = dtoh32(val);
	if (val != WLC_IOCTL_VERSION && val != 1) {
		WL_ERR(("Version mismatch, please upgrade. Got %d, expected %d or 1\n",
			val, WLC_IOCTL_VERSION));
		return BCME_VERSION;
	}
	ioctl_version = val;
	WL_INFO(("WLC_GET_VERSION=%d\n", ioctl_version));

	mutex_lock(&wl->usr_sync);
	dhd = (dhd_pub_t *)(wl->pub);
	if ((dhd->op_mode & HOSTAPD_MASK) != HOSTAPD_MASK) {
		wl_cfg80211_attach_post(wl_to_prmry_ndev(wl));
	}
	err = __wl_cfg80211_up(wl);
	if (err)
		WL_ERR(("__wl_cfg80211_up failed\n"));
	mutex_unlock(&wl->usr_sync);

	return err;
}

int wl_cfg80211_hang(struct net_device *dev, u16 reason)
{
	
	
	

	WL_ERR(("In : chip crash eventing\n"));
	wl_cfg80211_send_priv_event(dev, "HANG");
	return 0;
}

int wl_cfg80211_rssilow(struct net_device *dev)
{
	struct wl_priv *wl;
	wl = wlcfg_drv_priv;

	WL_ERR(("In : send RSSI low ind\n"));
	cfg80211_disconnected(dev, WLAN_REASON_RSSI_LOW_IND, NULL, 0, GFP_KERNEL);

	return 0;
}

s32 wl_cfg80211_down(void *para)
{
	struct wl_priv *wl;
	s32 err = 0;

	(void)para;
	WL_DBG(("In\n"));
	wl = wlcfg_drv_priv;
	wl->apsta_concurrent = 0;
	mutex_lock(&wl->usr_sync);
	err = __wl_cfg80211_down(wl);
	mutex_unlock(&wl->usr_sync);

	return err;
}

static void *wl_read_prof(struct wl_priv *wl, struct net_device *ndev, s32 item)
{
	unsigned long flags;
	void *rptr = NULL;
	struct wl_profile *profile = wl_get_profile_by_netdev(wl, ndev);

	if (!profile)
		return NULL;
	spin_lock_irqsave(&wl->cfgdrv_lock, flags);
	switch (item) {
	case WL_PROF_SEC:
		rptr = &profile->sec;
		break;
	case WL_PROF_ACT:
		rptr = &profile->active;
		break;
	case WL_PROF_BSSID:
		rptr = profile->bssid;
		break;
	case WL_PROF_SSID:
		rptr = &profile->ssid;
	case WL_PROF_CHAN:
		rptr = &profile->channel;
		break;
	}
	spin_unlock_irqrestore(&wl->cfgdrv_lock, flags);
	if (!rptr)
		WL_ERR(("invalid item (%d)\n", item));
	return rptr;
}

static s32
wl_update_prof(struct wl_priv *wl, struct net_device *ndev,
	const wl_event_msg_t *e, void *data, s32 item)
{
	s32 err = 0;
	struct wlc_ssid *ssid;
	unsigned long flags;
	struct wl_profile *profile = wl_get_profile_by_netdev(wl, ndev);

	if (!profile)
		return WL_INVALID;
	spin_lock_irqsave(&wl->cfgdrv_lock, flags);
	switch (item) {
	case WL_PROF_SSID:
		ssid = (wlc_ssid_t *) data;
		memset(profile->ssid.SSID, 0,
			sizeof(profile->ssid.SSID));
		memcpy(profile->ssid.SSID, ssid->SSID, ssid->SSID_len);
		profile->ssid.SSID_len = ssid->SSID_len;
		break;
	case WL_PROF_BSSID:
		if (data)
			memcpy(profile->bssid, data, ETHER_ADDR_LEN);
		else
			memset(profile->bssid, 0, ETHER_ADDR_LEN);
		break;
	case WL_PROF_SEC:
		memcpy(&profile->sec, data, sizeof(profile->sec));
		break;
	case WL_PROF_ACT:
		profile->active = *(bool *)data;
		break;
	case WL_PROF_BEACONINT:
		profile->beacon_interval = *(u16 *)data;
		break;
	case WL_PROF_DTIMPERIOD:
		profile->dtim_period = *(u8 *)data;
		break;
	case WL_PROF_CHAN:
		profile->channel = *(u32*)data;
	default:
		err = -EOPNOTSUPP;
		break;
	}
	spin_unlock_irqrestore(&wl->cfgdrv_lock, flags);

	if (err == EOPNOTSUPP)
		WL_ERR(("unsupported item (%d)\n", item));
	return err;
}

void wl_cfg80211_dbg_level(u32 level)
{
	
}

static bool wl_is_ibssmode(struct wl_priv *wl, struct net_device *ndev)
{
	return wl_get_mode_by_netdev(wl, ndev) == WL_MODE_IBSS;
}

static __used bool wl_is_ibssstarter(struct wl_priv *wl)
{
	return wl->ibss_starter;
}

static void wl_rst_ie(struct wl_priv *wl)
{
	struct wl_ie *ie = wl_to_ie(wl);

	ie->offset = 0;
}

static __used s32 wl_add_ie(struct wl_priv *wl, u8 t, u8 l, u8 *v)
{
	struct wl_ie *ie = wl_to_ie(wl);
	s32 err = 0;

	if (unlikely(ie->offset + l + 2 > WL_TLV_INFO_MAX)) {
		WL_ERR(("ei crosses buffer boundary\n"));
		return -ENOSPC;
	}
	ie->buf[ie->offset] = t;
	ie->buf[ie->offset + 1] = l;
	memcpy(&ie->buf[ie->offset + 2], v, l);
	ie->offset += l + 2;

	return err;
}

static s32 wl_mrg_ie(struct wl_priv *wl, u8 *ie_stream, u16 ie_size)
{
	struct wl_ie *ie = wl_to_ie(wl);
	s32 err = 0;

	if (unlikely(ie->offset + ie_size > WL_TLV_INFO_MAX)) {
		WL_ERR(("ei_stream crosses buffer boundary\n"));
		return -ENOSPC;
	}
	memcpy(&ie->buf[ie->offset], ie_stream, ie_size);
	ie->offset += ie_size;

	return err;
}

static s32 wl_cp_ie(struct wl_priv *wl, u8 *dst, u16 dst_size)
{
	struct wl_ie *ie = wl_to_ie(wl);
	s32 err = 0;

	if (unlikely(ie->offset > dst_size)) {
		WL_ERR(("dst_size is not enough\n"));
		return -ENOSPC;
	}
	memcpy(dst, &ie->buf[0], ie->offset);

	return err;
}

static u32 wl_get_ielen(struct wl_priv *wl)
{
	struct wl_ie *ie = wl_to_ie(wl);

	return ie->offset;
}

static void wl_link_up(struct wl_priv *wl)
{
	wl->link_up = true;
}

static void wl_link_down(struct wl_priv *wl)
{
	struct wl_connect_info *conn_info = wl_to_conn(wl);

	WL_DBG(("In\n"));
	wl->link_up = false;
	conn_info->req_ie_len = 0;
	conn_info->resp_ie_len = 0;
}

static unsigned long wl_lock_eq(struct wl_priv *wl)
{
	unsigned long flags;

	spin_lock_irqsave(&wl->eq_lock, flags);
	return flags;
}

static void wl_unlock_eq(struct wl_priv *wl, unsigned long flags)
{
	spin_unlock_irqrestore(&wl->eq_lock, flags);
}

static void wl_init_eq_lock(struct wl_priv *wl)
{
	spin_lock_init(&wl->eq_lock);
}

static void wl_delay(u32 ms)
{
	if (in_atomic() || ms < 1000 / HZ) {
		mdelay(ms);
	} else {
		msleep(ms);
	}
}

s32 wl_cfg80211_get_mac_addr(struct net_device *net, struct ether_addr *mac_addr)
{
	struct wl_priv *wl;
	dhd_pub_t *dhd_pub;

	wl = wlcfg_drv_priv;
	dhd_pub = (dhd_pub_t *)wl->pub;
	memcpy(mac_addr, &dhd_pub->mac, sizeof(*mac_addr));

	return 0;
}

s32 wl_cfg80211_get_p2p_dev_addr(struct net_device *net, struct ether_addr *p2pdev_addr)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct ether_addr p2pif_addr;
	struct ether_addr primary_mac;
	if (!wl->p2p)
		return -1;
	if (!p2p_is_on(wl)) {
		get_primary_mac(wl, &primary_mac);
		wl_cfgp2p_generate_bss_mac(&primary_mac, p2pdev_addr, &p2pif_addr);
	} else {
		memcpy(p2pdev_addr->octet,
			wl->p2p->dev_addr.octet, ETHER_ADDR_LEN);
	}


	return 0;
}
s32 wl_cfg80211_set_p2p_noa(struct net_device *net, char* buf, int len)
{
	struct wl_priv *wl;

	wl = wlcfg_drv_priv;

	return wl_cfgp2p_set_p2p_noa(wl, net, buf, len);
}

s32 wl_cfg80211_get_p2p_noa(struct net_device *net, char* buf, int len)
{
	struct wl_priv *wl;
	wl = wlcfg_drv_priv;

	return wl_cfgp2p_get_p2p_noa(wl, net, buf, len);
}

s32 wl_cfg80211_set_p2p_ps(struct net_device *net, char* buf, int len)
{
	struct wl_priv *wl;
	wl = wlcfg_drv_priv;

	return wl_cfgp2p_set_p2p_ps(wl, net, buf, len);
}

extern int wl_iw_softap_deassoc_stations(struct net_device *dev, u8 *mac);
s32 wl_cfg80211_deauth_sta(struct net_device *net, char* buf, int len)
{
	printf("deauth: %02X:%02X:%02X:%02X:%02x:%02X\n", buf[0], buf[1], buf[2],
		buf[3],buf[4],buf[5]);
	return wl_iw_softap_deassoc_stations(net, buf);
}

s32 wl_cfg80211_set_mpc(struct net_device *net, char* buf, int len)
{
	s32 mpc = -1;
	s32 arpoe = -1;

	sscanf(buf, "%d", &mpc);
	arpoe = mpc;
	if ((mpc != 0) && (mpc !=1)) {
		printf("%s: Bad arg!\n", __FUNCTION__);
	}

	wldev_iovar_setint(net, "arpoe", arpoe);
	return wldev_iovar_setint(net, "mpc", mpc);
}

s32 wl_cfg80211_set_wps_p2p_ie(struct net_device *net, char *buf, int len,
	enum wl_management_type type)
{
	struct wl_priv *wl;
	struct net_device *ndev = NULL;
	struct ether_addr primary_mac;
	s32 ret = 0;
	s32 bssidx = 0;
	s32 pktflag = 0;
	wl = wlcfg_drv_priv;
	if (wl_get_drv_status(wl, AP_CREATING, net) ||
		wl_get_drv_status(wl, AP_CREATED, net)) {
		ndev = net;
		bssidx = 0;
	} else if (wl->p2p) {
	   if (net == wl->p2p_net) {
			net = wl_to_prmry_ndev(wl);
		}

		if (!wl->p2p->on) {
			get_primary_mac(wl, &primary_mac);
			wl_cfgp2p_generate_bss_mac(&primary_mac, &wl->p2p->dev_addr,
				&wl->p2p->int_addr);
			p2p_on(wl) = true;
			ret = wl_cfgp2p_enable_discovery(wl, net, NULL, 0);

			if (unlikely(ret)) {
				goto exit;
			}
		}
		if (net  != wl_to_prmry_ndev(wl)) {
			if (wl_get_mode_by_netdev(wl, net) == WL_MODE_AP) {
				ndev = wl_to_p2p_bss_ndev(wl, P2PAPI_BSSCFG_CONNECTION);
				bssidx = wl_to_p2p_bss_bssidx(wl, P2PAPI_BSSCFG_CONNECTION);
			}
		} else {
				ndev = wl_to_p2p_bss_ndev(wl, P2PAPI_BSSCFG_PRIMARY);
				bssidx = wl_to_p2p_bss_bssidx(wl, P2PAPI_BSSCFG_DEVICE);
		}
	}
	if (ndev != NULL) {
		switch (type) {
			case WL_BEACON:
				pktflag = VNDR_IE_BEACON_FLAG;
				break;
			case WL_PROBE_RESP:
				pktflag = VNDR_IE_PRBRSP_FLAG;
				break;
			case WL_ASSOC_RESP:
				pktflag = VNDR_IE_ASSOCRSP_FLAG;
				break;
		}
		if (pktflag)
			ret = wl_cfgp2p_set_management_ie(wl, ndev, bssidx, pktflag, buf, len);
	}

exit:
	return ret;
}

static const struct rfkill_ops wl_rfkill_ops = {
	.set_block = wl_rfkill_set
};

static int wl_rfkill_set(void *data, bool blocked)
{
	struct wl_priv *wl = (struct wl_priv *)data;

	WL_DBG(("Enter \n"));
	WL_DBG(("RF %s\n", blocked ? "blocked" : "unblocked"));

	if (!wl)
		return -EINVAL;

	wl->rf_blocked = blocked;

	return 0;
}

static int wl_setup_rfkill(struct wl_priv *wl, bool setup)
{
	s32 err = 0;

	WL_DBG(("Enter \n"));
	if (!wl)
		return -EINVAL;
	if (setup) {
		wl->rfkill = rfkill_alloc("brcmfmac-wifi",
			wl_cfg80211_get_parent_dev(),
			RFKILL_TYPE_WLAN, &wl_rfkill_ops, (void *)wl);

		if (!wl->rfkill) {
			err = -ENOMEM;
			goto err_out;
		}

		err = rfkill_register(wl->rfkill);

		if (err)
			rfkill_destroy(wl->rfkill);
	} else {
		if (!wl->rfkill) {
			err = -ENOMEM;
			goto err_out;
		}

		rfkill_unregister(wl->rfkill);
		rfkill_destroy(wl->rfkill);
	}

err_out:
	return err;
}

struct device *wl_cfg80211_get_parent_dev(void)
{
	return cfg80211_parent_dev;
}

void wl_cfg80211_set_parent_dev(void *dev)
{
	cfg80211_parent_dev = dev;
}

static void wl_cfg80211_clear_parent_dev(void)
{
	cfg80211_parent_dev = NULL;
}

static void get_primary_mac(struct wl_priv *wl, struct ether_addr *mac)
{
	wldev_iovar_getbuf_bsscfg(wl_to_prmry_ndev(wl), "cur_etheraddr", NULL,
		0, wl->ioctl_buf, WLC_IOCTL_SMLEN, 0, &wl->ioctl_buf_sync);
	memcpy(mac->octet, wl->ioctl_buf, ETHER_ADDR_LEN);
}

void wl_cfg80211_abort_connecting(void)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	struct net_device *dev;

	if(!wl) {
		printf("%s : wl is null, return\n",__func__);
		return;
	}

	dev = wl_to_prmry_ndev(wl);
	if(!dev) {
        printf("%s : dev is null, return\n",__func__);
        return;
	}
	if (wl_get_drv_status(wl, CONNECTING, dev)) {
		int val = 0;
		int err;
		printf("%s: abort the connecting procedure.\n", __FUNCTION__);
        err = wldev_ioctl(dev, WLC_DISASSOC, &val, sizeof(val), true);
        if (unlikely(err)) {
	        WL_ERR(("call WLC_DISASSOC failed. (%d)\n", err));
	    }
        
        wl_bss_connect_done(wl, dev, NULL, NULL, false);
    }
}

#ifdef APSTA_CONCURRENT
int wl_cfg80211_set_apsta_concurrent(struct net_device *dev, bool enable)
{
	struct wl_priv *wl = wlcfg_drv_priv;
	if (wl) {
		printf("%s: setwl->apsta_concurrent=%d\n", __FUNCTION__, enable);
		wl->apsta_concurrent = enable;
		
		
	        if (wl_get_drv_status(wl, CONNECTING, dev)) {
                        int val = 0;
			int err;

			
			printf("%s: abort the connecting procedure.\n", __FUNCTION__);
                        err = wldev_ioctl(dev, WLC_DISASSOC, &val, sizeof(val), true);
                        if (unlikely(err)) {
                                WL_ERR(("call WLC_DISASSOC failed. (%d)\n", err));
                        }

			
			wl_bss_connect_done(wl, dev, NULL, NULL, false);
           } 
    }

	return 0;
}
#endif

int wl_cfg80211_do_driver_init(struct net_device *net)
{
	struct wl_priv *wl = *(struct wl_priv **)netdev_priv(net);

	if (!wl || !wl->wdev)
		return -EINVAL;

	if (dhd_do_driver_init(wl->wdev->netdev) < 0)
		return -1;

	return 0;
}

void wl_cfg80211_enable_trace(int level)
{
	wl_dbg_level |= WL_DBG_DBG;
}
