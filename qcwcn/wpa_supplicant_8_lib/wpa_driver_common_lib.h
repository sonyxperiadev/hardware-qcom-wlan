/*
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of The Linux Foundation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef WPA_DRIVER_COMMON_LIB
#define WPA_DRIVER_COMMON_LIB

#include "android_drv.h"	//needed?
#define OUI_LEN		3
#define MAX_CMD_LEN	32
#define MAC_ADDR_LEN	6

#define IEEE80211_HE_OPERATION_VHT_OPER_MASK 0x00004000
#define IEEE80211_HE_OPERATION_CO_LOC_BSS_MASK 0x00008000
#define IEEE80211_HE_OPERATION_6G_OPER_MASK 0x00020000

#define HE_OPER_VHT_CH_WIDTH_OFFSET 0
#define HE_OPER_VHT_MAX_OFFSET 2

#define HE_OPER_CO_LOCATED_MAX_OFFSET 0

#define HE_OPER_6G_PARAMS_OFFSET 1

#define HE_OPER_6G_PARAMS_SUB_CH_BW_MASK 0X03

#define CHANNEL_BW_INVALID 255

struct bss_info {
	uint8_t oui[OUI_LEN];
	char ssid[MAX_SSID_LEN + 1];
	int channel;
	int bw;
	int rssi;
	int data_rate;
	/* 0 : 11b, 1 : 11g, 2 : 11n, 3 : 11a, 4 : 11ac */
	int mode_80211;
	/* 0 : SISO, 1 : MIMO (2X2), 2 : MIMO (3X3), 3 : MIMO (4X4) */
	int snr;
	int noise;
	int akm;
	int roaming_count;
	/* 0: None, 1: 11k, 2: 11v, 3: 11kv */
	int mode_11kv;
	/* Bit mask value of 11kv support */
	int mask_11kv;
	u32 disc_reasn_code;
};

enum get_info_cmd {
	GETSTATSBSSINFO = 1,
	SETCELLSWITCHMODE = 2,
};

struct resp_info {
	u32 subcmd;
	char *reply_buf;
	int reply_buf_len;
	enum get_info_cmd cmd_type;
	uint8_t mac_addr[MAC_ADDR_LEN];
	u32 freq;
	uint8_t country[4];
};

#define QCA_NL80211_VENDOR_SUBCMD_GET_STATION 121

#ifndef CHANWIDTH_USE_HT
#define CHANWIDTH_USE_HT VHT_CHANWIDTH_USE_HT
#endif /* CHANWIDTH_USE_HT */
#ifndef CHANWIDTH_80MHZ
#define CHANWIDTH_80MHZ VHT_CHANWIDTH_80MHZ
#endif /* CHANWIDTH_80MHZ */
#ifndef CHANWIDTH_160MHZ
#define CHANWIDTH_160MHZ VHT_CHANWIDTH_160MHZ
#endif /* CHANWIDTH_160MHZ */
#ifndef CHANWIDTH_80P80MHZ
#define CHANWIDTH_80P80MHZ VHT_CHANWIDTH_80P80MHZ
#endif /* CHANWIDTH_80P80MHZ */

/**
 * enum qca_wlan_vendor_attr_get_station - Sub commands used by
 * QCA_NL80211_VENDOR_SUBCMD_GET_STATION to get the corresponding
 * station information. The information obtained through these
 * commands signify the current info in connected state and
 * latest cached information during the connected state , if queried
 * when in disconnected state.
 */
enum qca_wlan_vendor_attr_get_station {
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INVALID = 0,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_ASSOC_FAIL_REASON,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_REMOTE,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_GET_STATION_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_MAX =
	QCA_WLAN_VENDOR_ATTR_GET_STATION_AFTER_LAST - 1,
};

/*
these enum changes are temporary, shall be removed when
updated wpa_supplicant_8/src/common/qca-vendor.h is available
*/

enum qca_roam_control_scheme_tmp{
	QCA_ATTR_ROAM_CONTROL_SCAN_SCHEME_TRIGGERS_TMP = 13,
};

enum qca_roam_trigger_reasons_tmp {
	QCA_ROAM_TRIGGER_REASON_USER_TRIGGER_TMP	= 1 << 8,
	QCA_ROAM_TRIGGER_REASON_DEAUTH_TMP              = 1 << 9,
	QCA_ROAM_TRIGGER_REASON_IDLE_TMP		= 1 << 10,
	QCA_ROAM_TRIGGER_REASON_TX_FAILURES_TMP	        = 1 << 11,
	QCA_ROAM_TRIGGER_REASON_EXTERNAL_SCAN_TMP	= 1 << 12,
};

/**
 * enum qca_wlan_vendor_attr_get_station_info - Station Info queried
 * through QCA_NL80211_VENDOR_SUBCMD_GET_STATION.
 */
enum qca_wlan_vendor_attr_get_station_info {
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_INVALID = 0,
	/*
	 * Get the standard NL attributes Nested with this attribute.
	 * Ex : Query BW , BITRATE32 , NSS , Signal , Noise of the Link -
	 * NL80211_ATTR_SSID / NL80211_ATTR_SURVEY_INFO (Connected Channel) /
	 * NL80211_ATTR_STA_INFO
	 */
	QCA_WLAN_VENDOR_ATTR_GET_STATION_LINK_INFO_ATTR,
	/*
	 * Get the standard NL attributes Nested with this attribute.
	 * Ex : Query HT/VHT Capability advertized by the AP.
	 * NL80211_ATTR_VHT_CAPABILITY / NL80211_ATTR_HT_CAPABILITY
	 */
	QCA_WLAN_VENDOR_ATTR_GET_STATION_AP_INFO_ATTR,

	/* Number of successful Roam attempts before a disconnect,
	 * Unsigned 32 bit value
	 */
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_ROAM_COUNT,

	/* Authentication Key Management Type used for the connected session.
	 * Signified by enum qca_wlan_auth_type
	 */
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AKM,

	/* 802.11 Mode of the connected Session,
	 * signified by enum qca_wlan_802_11_mode
	 */
	QCA_WLAN_VENDOR_ATTR_802_11_MODE,

	/* HS20 Indication Element */
	QCA_WLAN_VENDOR_ATTR_GET_STATION_AP_INFO_HS20_INDICATION,

	/* HT/VHT operation elements */
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_HT_OPERATION,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_VHT_OPERATION,

	/* Status Code Corresponding to the Association Failure.
	 * Unsigned 32 bit value
	 */
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_ASSSOC_FAIL_REASON,

	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_MAX_PHY_RATE,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_PACKETS,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_BYTES,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_PACKETS,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_BYTES,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_LAST_TX_RATE,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_LAST_RX_RATE,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_WMM,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_SUPPORTED_MODE,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_AMPDU,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_STBC,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_STBC,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_CH_WIDTH,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_SGI_ENABLE,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_PAD,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_RETRY_COUNT,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_BC_MC_COUNT,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_FAILURE,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_AVG_RSSI_PER_CHAIN,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_RETRY_SUCCEED,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_LAST_PKT_RSSI,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_RETRY,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_RETRY_EXHAUST,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_TOTAL_FW,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_RETRY_FW,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_RETRY_EXHAUST_FW,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_BEACON_IES,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_DRIVER_DISCONNECT_REASON,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_ASSOC_REQ_IES,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_HE_OPERATION,

	/* keep last */
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AFTER_LAST,
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_MAX =
	QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AFTER_LAST - 1,
};

#define NOISE_FLOOR_DBM (96)

#define WMI_MAX_CHAINS (3)

struct remote_sta_info {
	u8 num_sta;
	u8 num_request_sta_info;
	u8 num_received_sta_info;
	u32 rx_retry_pkts;
	u32 rx_bcmc_pkts;
	u16 cap;
	u32 freq;
	u8 bandwidth;
	s8 rssi;
	u32 data_rate;
	u32 dot11_mode;
	u32 reason;
	u8 supported_mode;
	u32 tx_pckts;
	u32 tx_failures;
	u32 tx_rate;
	s32 avg_rssi_per_chain[WMI_MAX_CHAINS];
	u32 tx_pkts_retried;
	u32 tx_pkts_retry_exhausted;
	s32 rx_lastpkt_rssi;
	u32 tx_pkts_total;
	u32 tx_pkts_retries;
	u32 tx_pkts_fw_total;
	u32 tx_pkts_fw_retries;
	u32 tx_pkts_fw_retry_exhausted;
	u8 *supp_op_classes; /* Supported Operating Classes element, if
			      * received, starting from the Length field */
	u8 *supp_channels;
	u32 supported_band;
};

#endif
