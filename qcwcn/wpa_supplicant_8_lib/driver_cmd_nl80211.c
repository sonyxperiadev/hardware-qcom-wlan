/*
 * Driver interaction with extended Linux CFG8021
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Alternatively, this software may be distributed under the terms of BSD
 * license.
 *
 */

#include "includes.h"
#include <sys/types.h>
#include <fcntl.h>
#include <net/if.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/ctrl.h>
#include <netlink/object-api.h>
#include <linux/pkt_sched.h>

#include "common.h"
#include "linux_ioctl.h"
#include "driver_nl80211.h"
#include "wpa_supplicant_i.h"
#include "config.h"
#include "wpa_driver_common_lib.h"
#include "ap/hostapd.h"
#include "ap/sta_info.h"
#ifdef LINUX_EMBEDDED
#include <sys/ioctl.h>
#endif
#if defined(ANDROID) || defined(LINUX_EMBEDDED)
#include "android_drv.h"
#endif
#include "driver_cmd_nl80211_extn.h"

#define WPA_PS_ENABLED		0
#define WPA_PS_DISABLED		1
#define UNUSED(x)	(void)(x)
#define NL80211_ATTR_MAX_INTERNAL 256


/* ============ nl80211 driver extensions ===========  */
static int wpa_driver_cmd_set_tx_power(struct i802_bss *bss, char *cmd)
{
	struct wpa_driver_nl80211_data *drv = bss->drv;
	struct nl_msg *msg;
	char *endptr = NULL;
	int ret;
	int dbm, mbm;

	wpa_printf(MSG_INFO, "%s enter: dbm=%s", __FUNCTION__, cmd);

	dbm = strtol(cmd, &endptr, 10);
	if (*endptr || dbm < 0) {
		wpa_printf(MSG_ERROR, "%s: invalid dbm %d", __FUNCTION__, dbm);
		return -EINVAL;
	}
	mbm = dbm * 100;
	if (mbm < 0) { // integer overflow
		wpa_printf(MSG_ERROR, "%s: invalid mbm %d", __FUNCTION__, mbm);
		return -EINVAL;
	}

	if (!(msg = nl80211_drv_msg(drv, 0, NL80211_CMD_SET_WIPHY)) ||
	    nla_put_u32(msg, NL80211_ATTR_WIPHY_TX_POWER_SETTING,
		NL80211_TX_POWER_LIMITED) ||
	    nla_put_u32(msg, NL80211_ATTR_WIPHY_TX_POWER_LEVEL, mbm)) {
		nlmsg_free(msg);
		return -ENOBUFS;
	}

	ret = send_and_recv_msgs(drv, msg, NULL, NULL);
	if (!ret)
		return 0;

	wpa_printf(MSG_ERROR, "%s: Failed set_tx_power dbm=%d, ret=%d",
		   __FUNCTION__, dbm, ret);
	return ret;
}

/* Return type for setBand*/
enum {
	SEND_CHANNEL_CHANGE_EVENT = 0,
	DO_NOT_SEND_CHANNEL_CHANGE_EVENT,
};

typedef struct android_wifi_priv_cmd {
	char *buf;
	int used_len;
	int total_len;
} android_wifi_priv_cmd;

static int drv_errors = 0;

static void wpa_driver_notify_country_change(void *ctx, char *cmd)
{
	if ((os_strncasecmp(cmd, "COUNTRY", 7) == 0) ||
	    (os_strncasecmp(cmd, "SETBAND", 7) == 0)) {
		union wpa_event_data event;

		os_memset(&event, 0, sizeof(event));
		event.channel_list_changed.initiator = REGDOM_SET_BY_USER;
		if (os_strncasecmp(cmd, "COUNTRY", 7) == 0) {
			event.channel_list_changed.type = REGDOM_TYPE_COUNTRY;
			if (os_strlen(cmd) > 9) {
				event.channel_list_changed.alpha2[0] = cmd[8];
				event.channel_list_changed.alpha2[1] = cmd[9];
			}
		} else {
			event.channel_list_changed.type = REGDOM_TYPE_UNKNOWN;
		}
		wpa_supplicant_event(ctx, EVENT_CHANNEL_LIST_CHANGED, &event);
	}
}

static struct remote_sta_info g_sta_info = {0};

static struct nl_msg *prepare_nlmsg(struct wpa_driver_nl80211_data *drv,
				    char *ifname, int cmdid, int subcmd,
				    int flag)
{
	int res;
	struct nl_msg *nlmsg = nlmsg_alloc();
	int ifindex;

	if (nlmsg == NULL) {
		wpa_printf(MSG_ERROR,"Out of memory");
		return NULL;
	}

	genlmsg_put(nlmsg, /* pid = */ 0, /* seq = */ 0,
		    drv->global->nl80211_id, 0, flag, cmdid, 0);

	if (cmdid == NL80211_CMD_VENDOR) {
		res = nla_put_u32(nlmsg, NL80211_ATTR_VENDOR_ID, OUI_QCA);
		if (res < 0) {
			wpa_printf(MSG_ERROR,"Failed to put vendor id");
			goto cleanup;
		}

		res = nla_put_u32(nlmsg, NL80211_ATTR_VENDOR_SUBCMD, subcmd);
		if (res < 0) {
			wpa_printf(MSG_ERROR,"Failed to put vendor sub command");
			goto cleanup;
		}
	}

	if (ifname && (strlen(ifname) > 0))
		ifindex = if_nametoindex(ifname);
	else
		ifindex = if_nametoindex("wlan0");

	if (nla_put_u32(nlmsg, NL80211_ATTR_IFINDEX, ifindex) != 0) {
		wpa_printf(MSG_ERROR,"Failed to get iface index for iface: %s", ifname);
		goto cleanup;
	}

	return nlmsg;

cleanup:
	if (nlmsg)
		nlmsg_free(nlmsg);
	return NULL;
}

static struct nl_msg *prepare_vendor_nlmsg(struct wpa_driver_nl80211_data *drv,
					   char *ifname, int subcmd)
{
	return prepare_nlmsg(drv, ifname, NL80211_CMD_VENDOR, subcmd, 0);
}

static int parse_station_info(struct resp_info *info, struct nlattr *vendata,
			      int datalen)
{
	struct bss_info data;
	struct nlattr *tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_MAX + 1];
	struct nlattr *attr, *attr1, *attr2;
	u8 *beacon_ies = NULL;
	size_t beacon_ies_len = 0;
	u8 seg0, seg1;

	os_memset(&data, 0, sizeof(struct bss_info));

	data.oui[0] = (OUI_QCA) & 0xFF;
	data.oui[1] = ((OUI_QCA)>>8) & 0xFF;
	data.oui[2] = ((OUI_QCA)>>16) & 0xFF;

	nla_parse(tb_vendor, QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_MAX,
		  vendata, datalen, NULL);

	attr = tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_LINK_INFO_ATTR];
	if (attr) {
		struct nlattr *tb1[NL80211_ATTR_MAX + 1];

		nla_parse(tb1, NL80211_ATTR_MAX, nla_data(attr),
			  nla_len(attr), NULL);
		if (tb1[NL80211_ATTR_SSID] &&
		    (nla_len(tb1[NL80211_ATTR_SSID]) <= MAX_SSID_LEN)) {
			os_memcpy(data.ssid, nla_data(tb1[NL80211_ATTR_SSID]),
					  nla_len(tb1[NL80211_ATTR_SSID]));
			data.ssid[nla_len(tb1[NL80211_ATTR_SSID])] = '\0';
		} else {
			wpa_printf(MSG_ERROR,"NL80211_ATTR_SSID not found");
		}
		if (tb1[NL80211_ATTR_MAC]) {
			os_memcpy(data.oui, nla_data(tb1[NL80211_ATTR_MAC]), OUI_LEN);
		} else {
			wpa_printf(MSG_ERROR,"NL80211_ATTR_MAC not found");
		}
		if (tb1[NL80211_ATTR_SURVEY_INFO]) {
			struct nlattr *tb2[NL80211_SURVEY_INFO_MAX + 1];

			nla_parse(tb2, NL80211_SURVEY_INFO_MAX,
				  nla_data(tb1[NL80211_ATTR_SURVEY_INFO]),
				  nla_len(tb1[NL80211_ATTR_SURVEY_INFO]), NULL);
			if (tb2[NL80211_SURVEY_INFO_FREQUENCY]) {
				data.channel =
				nla_get_u32(tb2[NL80211_SURVEY_INFO_FREQUENCY]);
			} else {
				wpa_printf(MSG_ERROR,
				    "NL80211_SURVEY_INFO_FREQUENCY not found");
			}
			if (tb2[NL80211_SURVEY_INFO_NOISE]) {
				data.noise =
				nla_get_u8(tb2[NL80211_SURVEY_INFO_NOISE]);
				data.noise -= 100;
			} else {
				wpa_printf(MSG_ERROR,"NL80211_SURVEY_INFO_NOISE not found");
			}
		} else {
			wpa_printf(MSG_ERROR,"NL80211_ATTR_SURVEY_INFO not found");
		}

		if (tb1[NL80211_ATTR_STA_INFO]) {
			struct nlattr *tb2[NL80211_STA_INFO_MAX + 1];

			nla_parse(tb2, NL80211_STA_INFO_MAX,
				  nla_data(tb1[NL80211_ATTR_STA_INFO]),
				  nla_len(tb1[NL80211_ATTR_STA_INFO]), NULL);
			if (tb2[NL80211_STA_INFO_SIGNAL]) {
				data.rssi =
				nla_get_u8(tb2[NL80211_STA_INFO_SIGNAL]);
				data.rssi -= 100;
			} else {
				wpa_printf(MSG_ERROR,"NL80211_STA_INFO_SIGNAL not found");
			}
			data.snr = data.rssi - data.noise;

			attr1 = tb2[NL80211_STA_INFO_TX_BITRATE];
			if (attr1) {
				struct nlattr *tb3[NL80211_RATE_INFO_MAX + 1];

				nla_parse(tb3, NL80211_RATE_INFO_MAX,
					  nla_data(attr1), nla_len(attr1),
					  NULL);
				if (tb3[NL80211_RATE_INFO_BITRATE32]) {
					data.data_rate = nla_get_u32(
					tb3[NL80211_RATE_INFO_BITRATE32])/10;
				} else if (tb3[NL80211_RATE_INFO_BITRATE]) {
					data.data_rate = nla_get_u16(
					tb3[NL80211_RATE_INFO_BITRATE])/10;
				}

			} else {
				wpa_printf(MSG_ERROR,"NL80211_STA_INFO_TX_BITRATE not found");
			}
		} else {
			wpa_printf(MSG_ERROR,"NL80211_ATTR_STA_INFO not found");
		}
	} else {
		wpa_printf(MSG_ERROR,
		   "QCA_WLAN_VENDOR_ATTR_GET_STATION_LINK_INFO_ATTR not found");
	}

	if (tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AKM]) {
		data.akm = nla_get_u32(
			tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_AKM]);
	}

	if (tb_vendor[QCA_WLAN_VENDOR_ATTR_802_11_MODE]) {
		data.mode_80211 = nla_get_u32(
			tb_vendor[QCA_WLAN_VENDOR_ATTR_802_11_MODE]);
	}

	attr = tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_VHT_OPERATION];
	attr1 = tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_HT_OPERATION];
	attr2 = tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_HE_OPERATION];
	if (attr) {
		struct ieee80211_vht_operation *info = nla_data(attr);

		switch (info->vht_op_info_chwidth) {
		case CHANWIDTH_USE_HT:
			if (attr1) {
				struct ieee80211_ht_operation *info;

				info = nla_data(attr1);
				data.bw = info->ht_param ? 40:20;
			}
			break;
		case CHANWIDTH_80MHZ:
			seg0 = info->vht_op_info_chan_center_freq_seg0_idx;
			seg1 = info->vht_op_info_chan_center_freq_seg1_idx;
			if (seg1 && abs(seg1 - seg0) == 8)
				data.bw = 160;
			else if (seg1)
				/* Notifying 80P80 as bandwidth = 160 */
				data.bw = 160;
			else
				data.bw = 80;
			break;
		case CHANWIDTH_160MHZ:
			data.bw = 160;
			break;
		case CHANWIDTH_80P80MHZ:
			data.bw = 160;
			break;
		default:
			wpa_printf(MSG_ERROR,"Invalid channel width received : %u",
						 info->vht_op_info_chwidth);
		}
	} else if (attr1) {
		struct ieee80211_ht_operation *info = nla_data(attr1);

		data.bw = info->ht_param ? 40:20;
	} else
		data.bw = 20;

	if (attr2) {
		struct ieee80211_he_operation *he_info = nla_data(attr2);
		uint8_t *opr, ch_bw = CHANNEL_BW_INVALID;

		/* Check optional field in he_info is present of not */
		if (!he_info || (nla_len(attr2) <=
		    (sizeof(he_info->he_oper_params) +
		    sizeof(he_info->he_mcs_nss_set)))) {
			he_info ? wpa_printf(MSG_ERROR,"Invalid he operation len: %d", nla_len(attr2)):
			wpa_printf(MSG_ERROR,"Invalid he_info: NULL");
			goto parse_beacon_ies;
		}

		opr = (uint8_t *)he_info;
		/* Point to operational field */
		opr += (sizeof(he_info->he_oper_params) +
			sizeof(he_info->he_mcs_nss_set));
		if (he_info->he_oper_params &
		    IEEE80211_HE_OPERATION_VHT_OPER_MASK) {
			ch_bw = opr[HE_OPER_VHT_CH_WIDTH_OFFSET];
			opr += (HE_OPER_VHT_MAX_OFFSET + 1);
		}

		if (he_info->he_oper_params &
		    IEEE80211_HE_OPERATION_CO_LOC_BSS_MASK) {
			opr += (HE_OPER_CO_LOCATED_MAX_OFFSET + 1);
		}

		if (he_info->he_oper_params &
		    IEEE80211_HE_OPERATION_6G_OPER_MASK) {
			ch_bw = (opr[HE_OPER_6G_PARAMS_OFFSET] &
				 HE_OPER_6G_PARAMS_SUB_CH_BW_MASK);
		}

		switch (ch_bw) {
		case CHANWIDTH_USE_HT:
			/* TO DO */
			break;
		case CHANWIDTH_80MHZ:
			data.bw = 80;
			break;
		case CHANWIDTH_160MHZ:
			data.bw = 160;
			break;
		case CHANWIDTH_80P80MHZ:
			data.bw = 160;
			break;
		default:
			wpa_printf(MSG_ERROR,"Invalid channel width received : %u", ch_bw);
		}
	}

parse_beacon_ies:
	attr = tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_BEACON_IES];
	if (attr) {
		beacon_ies = nla_data(attr);

		beacon_ies_len = nla_len(attr);
		if (beacon_ies && beacon_ies_len > 12) {
			beacon_ies += 12;
			beacon_ies_len -= 12;
		}
	}

	if (tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_DRIVER_DISCONNECT_REASON]) {
		data.disc_reasn_code = nla_get_u32(tb_vendor[
			QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_DRIVER_DISCONNECT_REASON]);
	}
	snprintf(info->reply_buf, info->reply_buf_len,
		 "%02x%02x%02x %s %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %u %s",
		 data.oui[0],
		 data.oui[1],
		 data.oui[2],
		 data.ssid,
		 data.channel,
		 data.bw,
		 data.rssi,
		 data.data_rate,
		 data.mode_80211,
		 -1,
		 -1,
		 -1,
		 data.snr,
		 data.noise,
		 data.akm,
		 data.roaming_count,
		 -1,
		 -1,
		 -1,
		 -1,
		 data.disc_reasn_code,
		 info->country);

	return 0;
}

static int handle_response(struct resp_info *info, struct nlattr *vendata,
			   int datalen)
{
	switch (info->subcmd) {
	case QCA_NL80211_VENDOR_SUBCMD_GET_STATION:
		os_memset(info->reply_buf, 0, info->reply_buf_len);
		if (info->cmd_type == GETSTATSBSSINFO)
			parse_station_info(info, vendata, datalen);

		wpa_printf(MSG_INFO,"STAINFO: %s", info->reply_buf);
		break;
	default:
		wpa_printf(MSG_ERROR,"Unsupported response type: %d", info->subcmd);
		break;
	}
	return 0;
}

static int response_handler(struct nl_msg *msg, void *arg)
{
	struct genlmsghdr *mHeader;
	struct nlattr *mAttributes[NL80211_ATTR_MAX_INTERNAL + 1];
	struct nlattr *vendata;
	int datalen;
	struct resp_info *info = (struct resp_info *) arg;
	int status;

	mHeader = (struct genlmsghdr *)nlmsg_data(nlmsg_hdr(msg));
	nla_parse(mAttributes, NL80211_ATTR_MAX_INTERNAL,
			  genlmsg_attrdata(mHeader, 0),
			  genlmsg_attrlen(mHeader, 0), NULL);

	if (mAttributes[NL80211_ATTR_VENDOR_DATA]) {
		vendata = nla_data(mAttributes[NL80211_ATTR_VENDOR_DATA]);
		datalen = nla_len(mAttributes[NL80211_ATTR_VENDOR_DATA]);
		if (!vendata) {
			wpa_printf(MSG_ERROR,"Vendor data not found");
			return -1;
		}
		status = handle_response(info, vendata, datalen);
	} else {
		wpa_printf(MSG_ERROR,"NL80211_ATTR_VENDOR_DATA not found");
		status = -1;
	}

	return status;
}

static int ack_handler(struct nl_msg *msg, void *arg)
{
	int *err = (int *)arg;

	*err = 0;
	return NL_STOP;
}


static int finish_handler(struct nl_msg *msg, void *arg)
{
	int *ret = (int *)arg;

	*ret = 0;
	return NL_SKIP;
}


static int error_handler(struct sockaddr_nl *nla, struct nlmsgerr *err,
						 void *arg)
{
	int *ret = (int *)arg;

	*ret = err->error;
	wpa_printf(MSG_ERROR,"%s received : %d - %s", __func__,
	      err->error, strerror(err->error));
	return NL_SKIP;
}


static int no_seq_check(struct nl_msg *msg, void *arg)
{
	return NL_OK;
}

static int send_nlmsg(struct nl_sock *cmd_sock, struct nl_msg *nlmsg,
		      nl_recvmsg_msg_cb_t customer_cb, void *arg)
{
	int err = 0;
	struct nl_cb *cb = nl_cb_alloc(NL_CB_DEFAULT);

	if (!cb)
		goto out;

	err = nl_send_auto_complete(cmd_sock, nlmsg);	/* send message */
	if (err < 0)
		goto out;

	err = 1;

	nl_cb_set(cb, NL_CB_SEQ_CHECK, NL_CB_CUSTOM, no_seq_check, NULL);
	nl_cb_err(cb, NL_CB_CUSTOM, error_handler, &err);
	nl_cb_set(cb, NL_CB_FINISH, NL_CB_CUSTOM, finish_handler, &err);
	nl_cb_set(cb, NL_CB_ACK, NL_CB_CUSTOM, ack_handler, &err);
	if (customer_cb)
		nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, customer_cb, arg);

	while (err > 0) {				   /* wait for reply */
		int res = nl_recvmsgs(cmd_sock, cb);

		if (res)
			wpa_printf(MSG_ERROR,"nl80211: %s->nl_recvmsgs failed: %d",
				 __func__, res);
	}
out:
	nl_cb_put(cb);
	if (nlmsg)
		nlmsg_free(nlmsg);
	return err;
}

static int chartohex(char c)
{
	int val = -1;

	if (c >= '0' && c <= '9')
		val = c - '0';
	else if (c >= 'a' && c <= 'f')
		val = c - 'a' + 10;
	else if (c >= 'A' && c <= 'F')
		val = c - 'A' + 10;

	return val;
}

static int convert_string_to_bytes(u8 *addr, const char *text, u16 max_bytes)
{
	u16 i = 0;
	int nibble;
	const char *temp = text;

	while (temp && *temp != '\0' && i < max_bytes) {
		nibble = chartohex(*temp++);
		if (nibble == -1)
			return -1;
		addr[i] = nibble << 4;
		nibble = chartohex(*temp++);
		if (nibble == -1)
			return -1;
		addr[i++] += nibble;
		if (*temp == ':')
			temp++;
	}

	return i;
}

/*
 * Client can send the cell switch mode in below format
 *
 * SETCELLSWITCHMODE <cs mode>
 *
 * examples:
 * For Default Mode   - "SETCELLSWITCHMODE 0"
 * To Disable Roaming - "SETCELLSWITCHMODE 1"
 * For Partial Scan   - "SETCELLSWITCHMODE 2"
 */
static int parse_and_populate_setcellswitchmode(struct nl_msg *nlmsg,
						    char *cmd)
{
	uint32_t all_trigger_bitmap, scan_scheme_bitmap;
	uint32_t cellswm;
	struct nlattr *config;

	cellswm = atoi(cmd);
	if (cellswm < 0 || cellswm > 2) {
		wpa_printf(MSG_ERROR,"Invalid cell switch mode: %d", cellswm);
		return -1;
	}
	wpa_printf(MSG_DEBUG, "cell switch mode: %d", cellswm);

	all_trigger_bitmap = QCA_ROAM_TRIGGER_REASON_PER |
			     QCA_ROAM_TRIGGER_REASON_BEACON_MISS |
			     QCA_ROAM_TRIGGER_REASON_POOR_RSSI |
			     QCA_ROAM_TRIGGER_REASON_BETTER_RSSI |
			     QCA_ROAM_TRIGGER_REASON_PERIODIC |
			     QCA_ROAM_TRIGGER_REASON_DENSE |
			     QCA_ROAM_TRIGGER_REASON_BTM |
			     QCA_ROAM_TRIGGER_REASON_BSS_LOAD |
			     QCA_ROAM_TRIGGER_REASON_USER_TRIGGER_TMP |
			     QCA_ROAM_TRIGGER_REASON_DEAUTH_TMP |
			     QCA_ROAM_TRIGGER_REASON_IDLE_TMP |
			     QCA_ROAM_TRIGGER_REASON_TX_FAILURES_TMP |
			     QCA_ROAM_TRIGGER_REASON_EXTERNAL_SCAN_TMP;

	scan_scheme_bitmap = QCA_ROAM_TRIGGER_REASON_PER |
			     QCA_ROAM_TRIGGER_REASON_BEACON_MISS |
			     QCA_ROAM_TRIGGER_REASON_POOR_RSSI |
			     QCA_ROAM_TRIGGER_REASON_BSS_LOAD |
			     QCA_ROAM_TRIGGER_REASON_BTM;

	if (nla_put_u32(nlmsg, QCA_WLAN_VENDOR_ATTR_ROAMING_SUBCMD,
		QCA_WLAN_VENDOR_ROAMING_SUBCMD_CONTROL_SET) ||
	    nla_put_u32(nlmsg, QCA_WLAN_VENDOR_ATTR_ROAMING_REQ_ID, 1)) {
		wpa_printf(MSG_ERROR,"Failed to put: roam_subcmd/REQ_ID");
	}

	config = nla_nest_start(nlmsg,
			QCA_WLAN_VENDOR_ATTR_ROAMING_PARAM_CONTROL);
	if (config == NULL)
		goto fail;

	switch (cellswm){
	case 0:
		if (nla_put_u32(nlmsg, QCA_ATTR_ROAM_CONTROL_TRIGGERS, all_trigger_bitmap)) {
			wpa_printf(MSG_ERROR,"Failed to set: ROAM_CONTROL_TRIGGERS");
			goto fail;
		}
		break;
	case 1:
		if (nla_put_u32(nlmsg, QCA_ATTR_ROAM_CONTROL_TRIGGERS, 0)) {
			wpa_printf(MSG_ERROR,"Failed to unset: ROAM_CONTROL_TRIGGERS");
			goto fail;
		}
		break;
	case 2:
		if (nla_put_u32(nlmsg, QCA_ATTR_ROAM_CONTROL_TRIGGERS, all_trigger_bitmap) ||
		    nla_put_u32(nlmsg, QCA_ATTR_ROAM_CONTROL_SCAN_SCHEME_TRIGGERS_TMP, scan_scheme_bitmap)) {
			wpa_printf(MSG_ERROR,"Failed to set: ROAM_CONTROL_TRIGGERS_SCAN_SCHEME");
			goto fail;
		}
		break;
	}
	nla_nest_end(nlmsg, config);

	return 0;
fail:
	return -1;

}

static int populate_nlmsg(struct nl_msg *nlmsg, char *cmd,
			  enum get_info_cmd type)
{
	struct nlattr *attr;

	attr = nla_nest_start(nlmsg, NL80211_ATTR_VENDOR_DATA);
	if (attr == NULL)
		return -1;

	switch (type) {
	case GETSTATSBSSINFO:
		if (nla_put_flag(nlmsg,
				 QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO)) {
			wpa_printf(MSG_ERROR,"Failed to put flag QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO");
			return -1;
		}
		break;
	case SETCELLSWITCHMODE:
		if (parse_and_populate_setcellswitchmode(nlmsg, cmd) != 0) {
			wpa_printf(MSG_ERROR, "Failed to populate nlmsg");
			return -1;
		}
		break;
	default:
		wpa_printf(MSG_ERROR,"Unsupported command: %d", type);
		break;
	}

	nla_nest_end(nlmsg, attr);
	return 0;
}

static char *skip_white_space(char *cmd)
{
	char *pos = cmd;

	while (*pos == ' ')
		pos++;

	return pos;
}

void ap_sta_copy_supp_op_classes(const u8 *supp_op_classes,
				 size_t supp_op_classes_len) {
	if (!supp_op_classes)
		return;
	os_free(g_sta_info.supp_op_classes);
	g_sta_info.supp_op_classes = malloc(1 + supp_op_classes_len);
	if(!g_sta_info.supp_op_classes)
		return;
	g_sta_info.supp_op_classes[0] = supp_op_classes_len;
	os_memcpy(g_sta_info.supp_op_classes + 1, supp_op_classes,
		  supp_op_classes_len);
}

void ap_sta_copy_channels(const u8 *supp_channels,
				 size_t supp_channels_len) {
	if (!supp_channels)
		return;
	os_free(g_sta_info.supp_channels);
	g_sta_info.supp_channels = malloc(1 + supp_channels_len);
	if(!g_sta_info.supp_channels)
		return;
	g_sta_info.supp_channels[0] = supp_channels_len;
	os_memcpy(g_sta_info.supp_channels + 1, supp_channels,
		  supp_channels_len);
}

static void parse_assoc_req_ies(const u8 *ies, int ies_len) {
	int left = ies_len;
	const u8 *pos = ies;

	while (left >= 2) {
		u8 id, ie_len;
		id = *pos++;
		ie_len = *pos++;
		left -= 2;

		if (ie_len > left) {
			wpa_printf(MSG_ERROR,"parse error, id = %d, ie_len = %d, left = %d",
			      id, ie_len, left);
			return;
		}

		switch (id) {
		case WLAN_EID_SUPPORTED_OPERATING_CLASSES:
			ap_sta_copy_supp_op_classes(pos, ie_len);
			break;
		case WLAN_EID_SUPPORTED_CHANNELS:
			ap_sta_copy_channels(pos, ie_len);
			break;
		default:
			break;
		}

		left -= ie_len;
		pos += ie_len;
	}

	if (left)
		wpa_printf(MSG_ERROR,"parse error, left = %d", left);
	return;
}

void op_class_band_conversion(u8 *op_classes) {
	int count = (g_sta_info.supp_op_classes[0]);
	int i = 1;
	int temp;

	if (count <= 1)
		g_sta_info.supported_band = 0;
	while((count-1) != 0) {
		temp = g_sta_info.supp_op_classes[i];
		if (temp >= 81 && temp <= 84)
			g_sta_info.supported_band |= BIT(0);
		else if (temp >= 115 && temp <= 130)
			g_sta_info.supported_band |= BIT(1);
		else if (temp >= 131 && temp <= 135)
			g_sta_info.supported_band |= BIT(2);
		i++;
		count--;
	}
}

void supp_channels_band_conversion(u8 *supp_channels) {
	int count = 0;
	int i = 1;
	int temp = 0;

	count = (g_sta_info.supp_channels[0]);
	if (count < 2)
		g_sta_info.supported_band = 0;

	while((count-1) >= 0) {
		temp = g_sta_info.supp_channels[i];
		if (temp >= 1 && temp <= 13)
			g_sta_info.supported_band |= BIT(0);
		else if (temp >= 32 && temp <= 173)
			g_sta_info.supported_band |= BIT(1);
		i += 2;
		count -= 2;
	}
}

static int get_sta_info_handler(struct nl_msg *msg, void *arg)
{
	struct genlmsghdr *msg_hdr;
	struct nlattr *tb[NL80211_ATTR_MAX_INTERNAL + 1];
	struct nlattr *tb_vendor[NL80211_ATTR_MAX_INTERNAL + 1];
	struct nlattr *vendor_data, *attr_link_info;
	int vendor_len;
	struct resp_info *info = (struct resp_info *)arg;
	u8 *assoc_req_ie = NULL;
	size_t assoc_req_ie_len = 0;

	if (!info || !info->reply_buf) {
		wpa_printf(MSG_ERROR,"Invalid reply_buf");
		return -1;
	}

	wpa_printf(MSG_INFO,"Recv STA info %02x:%02x:%02x:%02x:%02x:%02x",
	      info->mac_addr[0], info->mac_addr[1], info->mac_addr[2],
	      info->mac_addr[3], info->mac_addr[4], info->mac_addr[5]);

	msg_hdr = (struct genlmsghdr *)nlmsg_data(nlmsg_hdr(msg));
	nla_parse(tb, NL80211_ATTR_MAX_INTERNAL, genlmsg_attrdata(msg_hdr, 0),
		  genlmsg_attrlen(msg_hdr, 0), NULL);

	if (!tb[NL80211_ATTR_VENDOR_DATA]) {
		wpa_printf(MSG_ERROR,"NL80211_ATTR_VENDOR_DATA not found");
		return -1;
	}

	vendor_data = nla_data(tb[NL80211_ATTR_VENDOR_DATA]);
	vendor_len = nla_len(tb[NL80211_ATTR_VENDOR_DATA]);

	if (nla_parse(tb_vendor, NL80211_ATTR_MAX_INTERNAL,
		      vendor_data, vendor_len, NULL)) {
		wpa_printf(MSG_ERROR,"NL80211_ATTR_VENDOR_DATA not found");
		return -1;
	}

	attr_link_info = tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_LINK_INFO_ATTR];
	if (attr_link_info) {
		struct nlattr *tb_link_info[NL80211_ATTR_MAX + 1];
		struct nlattr *attr_survey_info, *attr_sta_info;

		nla_parse(tb_link_info, NL80211_ATTR_MAX, nla_data(attr_link_info),
			  nla_len(attr_link_info), NULL);
		attr_survey_info = tb_link_info[NL80211_ATTR_SURVEY_INFO];
		if (attr_survey_info) {
			struct nlattr *tb_survey_info[NL80211_SURVEY_INFO_MAX + 1];

			nla_parse(tb_survey_info, NL80211_SURVEY_INFO_MAX,
				  nla_data(attr_survey_info),
				  nla_len(attr_survey_info), NULL);
			if (tb_survey_info[NL80211_SURVEY_INFO_FREQUENCY]) {
				g_sta_info.freq =
					nla_get_u32(tb_survey_info[NL80211_SURVEY_INFO_FREQUENCY]);
				wpa_printf(MSG_INFO,"channel %d", g_sta_info.freq);
			}
		}

		attr_sta_info = tb_link_info[NL80211_ATTR_STA_INFO];
		if (attr_sta_info) {
			struct nlattr *tb_sta_info[NL80211_STA_INFO_MAX + 1];

			nla_parse(tb_sta_info, NL80211_STA_INFO_MAX,
				  nla_data(attr_sta_info),
				  nla_len(attr_sta_info), NULL);
			if (tb_sta_info[NL80211_STA_INFO_SIGNAL]) {
				g_sta_info.rssi = nla_get_u8(tb_sta_info[NL80211_STA_INFO_SIGNAL]);
				g_sta_info.rssi -= NOISE_FLOOR_DBM;
				wpa_printf(MSG_INFO,"rssi %d", g_sta_info.rssi);
			}
			if (tb_sta_info[NL80211_STA_INFO_TX_BITRATE]) {
				struct nlattr *tb_antenna_info[NL80211_RATE_INFO_MAX + 1];
				nla_parse(tb_antenna_info, NL80211_RATE_INFO_MAX,
					  nla_data(tb_sta_info[NL80211_STA_INFO_TX_BITRATE]),
					  nla_len(tb_sta_info[NL80211_STA_INFO_TX_BITRATE]),
					  NULL);
			}
		}

		if (tb_link_info[NL80211_ATTR_REASON_CODE]) {
			g_sta_info.reason =
				nla_get_u32(tb_link_info[NL80211_ATTR_REASON_CODE]);
			wpa_printf(MSG_INFO,"reason %d", g_sta_info.reason);
		}

		if (tb_link_info[NL80211_ATTR_STA_CAPABILITY]) {
			g_sta_info.cap =
				nla_get_u16(tb_link_info[NL80211_ATTR_STA_CAPABILITY]);
			wpa_printf(MSG_INFO,"cap %04x", g_sta_info.cap);
		}
	}

	if (tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_LAST_RX_RATE]) {
		g_sta_info.data_rate =
			nla_get_u32(tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_LAST_RX_RATE]);
		wpa_printf(MSG_INFO,"data_rate %d", g_sta_info.data_rate);
	}

	if (tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_RETRY_COUNT]) {
		g_sta_info.rx_retry_pkts +=
			nla_get_u32(tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_RETRY_COUNT]);
		wpa_printf(MSG_INFO,"rx_retry_pkts %d", g_sta_info.rx_retry_pkts);
	}

	if (tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_BC_MC_COUNT]) {
		g_sta_info.rx_bcmc_pkts +=
			nla_get_u32(tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_RX_BC_MC_COUNT]);
		wpa_printf(MSG_INFO,"rx_bcmc_pkts %d", g_sta_info.rx_bcmc_pkts);
	}

	if (tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_CH_WIDTH]) {
		g_sta_info.bandwidth =
			nla_get_u8(tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_CH_WIDTH]);
		wpa_printf(MSG_INFO,"bandwidth %d", g_sta_info.bandwidth);
	}

	if (tb_vendor[QCA_WLAN_VENDOR_ATTR_802_11_MODE]) {
		g_sta_info.dot11_mode =
			nla_get_u32(tb_vendor[QCA_WLAN_VENDOR_ATTR_802_11_MODE]);
		wpa_printf(MSG_INFO,"dot11_mode %d", g_sta_info.dot11_mode);
	}

	if (tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_SUPPORTED_MODE]) {
		g_sta_info.supported_mode =
			nla_get_u8(tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_SUPPORTED_MODE]);
		wpa_printf(MSG_INFO,"supported_mode %d", g_sta_info.supported_mode);
	}

	if (tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_ASSOC_REQ_IES]) {
		assoc_req_ie =
			nla_data(tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_ASSOC_REQ_IES]);
		assoc_req_ie_len =
			nla_len(tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_ASSOC_REQ_IES]);
	}
	parse_assoc_req_ies(assoc_req_ie, assoc_req_ie_len);

	if (g_sta_info.supp_op_classes) {
		op_class_band_conversion(g_sta_info.supp_op_classes);
	}
	else if (g_sta_info.supp_channels) {
		supp_channels_band_conversion(g_sta_info.supp_channels);
	}
	else
		wpa_printf(MSG_ERROR,"supp_op_classes and supp_channels both are null");

	g_sta_info.num_received_sta_info++;

	if (g_sta_info.num_received_sta_info == g_sta_info.num_sta) {
		if (g_sta_info.num_sta == 1 && tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_PACKETS]) {
			snprintf(info->reply_buf, info->reply_buf_len,
				 "%02x:%02x:%02x:%02x:%02x:%02x "
				 "%d %d %04x "
				 "%02x:%02x:%02x %d %d %d %d %d %d %d %d %d %s",
				 info->mac_addr[0], info->mac_addr[1],
				 info->mac_addr[2], info->mac_addr[3],
				 info->mac_addr[4], info->mac_addr[5],
				 g_sta_info.rx_retry_pkts, g_sta_info.rx_bcmc_pkts,
				 g_sta_info.cap, info->mac_addr[0],
				 info->mac_addr[1], info->mac_addr[2],
				 g_sta_info.freq,
				 g_sta_info.bandwidth,
				 g_sta_info.rssi,
				 g_sta_info.data_rate,
				 g_sta_info.dot11_mode,
				 -1,
				 -1,
				 g_sta_info.reason,
				 g_sta_info.supported_mode,
				 info->country);
		} else if (g_sta_info.num_sta == 1 && !tb_vendor[QCA_WLAN_VENDOR_ATTR_GET_STATION_INFO_REMOTE_TX_PACKETS]) {
			snprintf(info->reply_buf, info->reply_buf_len,
				 "%02x:%02x:%02x:%02x:%02x:%02x "
				 "%d %d %04x "
				 "%02x:%02x:%02x %d %d %d %d %d %d %d %d %u %s",
				 info->mac_addr[0], info->mac_addr[1],
				 info->mac_addr[2], info->mac_addr[3],
				 info->mac_addr[4], info->mac_addr[5],
				 g_sta_info.rx_retry_pkts, g_sta_info.rx_bcmc_pkts,
				 g_sta_info.cap, info->mac_addr[0],
				 info->mac_addr[1], info->mac_addr[2],
				 g_sta_info.freq,
				 g_sta_info.bandwidth,
				 g_sta_info.rssi,
				 g_sta_info.data_rate,
				 g_sta_info.supported_mode,
				 -1,
				 -1,
				 g_sta_info.reason,
				 g_sta_info.supported_band,
				 info->country);
		} else {
			/* Summary of all STAs */
			snprintf(info->reply_buf, info->reply_buf_len,
				 "%d %d %04x "
				 "%d %d %d %d %d %d %d %d %d %s",
				 g_sta_info.rx_retry_pkts, g_sta_info.rx_bcmc_pkts,
				 -1, /* CAP */
				 -1, /* Channel */
				 -1, /* Bandwidth */
				 -1, /* Rssi */
				 -1, /* Data_rate */
				 -1, /* 11_mode */
				 -1,
				 -1,
				 -1, /* Reason */
				 -1, /* Support_mode */
				 info->country);
		}

		wpa_printf(MSG_INFO,"%s", info->reply_buf);
	}

	return 0;
}

static int wpa_driver_ioctl(struct i802_bss *bss, char *cmd,
				 char *buf, size_t buf_len, int *status,
				 struct wpa_driver_nl80211_data *drv) {
	struct ifreq ifr;
	android_wifi_priv_cmd priv_cmd;
	memset(&ifr, 0, sizeof(ifr));
	memset(&priv_cmd, 0, sizeof(priv_cmd));
	os_memcpy(buf, cmd, strlen(cmd) + 1);
	strlcpy(ifr.ifr_name, bss->ifname, IFNAMSIZ);
	priv_cmd.buf = buf;
	priv_cmd.used_len = buf_len;
	priv_cmd.total_len = buf_len;
	ifr.ifr_data = &priv_cmd;

	if ((ioctl(drv->global->ioctl_sock, SIOCDEVPRIVATE + 1, &ifr)) < 0) {
		wpa_printf(MSG_ERROR,"%s: failed to issue private commands\n", __func__);
		*status = 1;
		return WPA_DRIVER_OEM_STATUS_FAILURE;
	} else {
		wpa_printf(MSG_ERROR,"Response: %s", buf);
		return WPA_DRIVER_OEM_STATUS_SUCCESS;
	}
}

static int wpa_driver_send_get_sta_info_cmd(struct i802_bss *bss, u8 *mac,
					    char *buf, size_t buf_len,
					    int *status)
{
	struct wpa_driver_nl80211_data *drv = bss->drv;
	struct nl_msg *nlmsg;
	struct nlattr *attr;
	struct resp_info info;

	memset(&info, 0, sizeof(info));
	os_memcpy(&info.mac_addr[0], mac, MAC_ADDR_LEN);
	info.reply_buf = buf;
	info.reply_buf_len = buf_len;

	char *p;
	if(wpa_driver_ioctl(bss, "GETCOUNTRYREV", buf, buf_len, &status, drv) == 0){
		p = strstr(buf, " ");
		if(p != NULL)
			memcpy(info.country, (p+1), strlen(p+1)+1);//length of p including null
	}

	nlmsg = prepare_vendor_nlmsg(drv, bss->ifname,
				     QCA_NL80211_VENDOR_SUBCMD_GET_STATION);
	if (!nlmsg) {
		wpa_printf(MSG_ERROR,"Failed to allocate nl message");
		return -1;
	}

	attr = nla_nest_start(nlmsg, NL80211_ATTR_VENDOR_DATA);
	if (!attr) {
		nlmsg_free(nlmsg);
		return -1;
	}

	if (nla_put(nlmsg, QCA_WLAN_VENDOR_ATTR_GET_STATION_REMOTE,
		    MAC_ADDR_LEN, mac)) {
		wpa_printf(MSG_ERROR,"Failed to put QCA_WLAN_VENDOR_ATTR_GET_STATION_REMOTE");
		nlmsg_free(nlmsg);
		return -1;
	}

	nla_nest_end(nlmsg, attr);

	*status = send_nlmsg((struct nl_sock *)drv->global->nl, nlmsg,
			     get_sta_info_handler, &info);
	if (*status != 0) {
		wpa_printf(MSG_ERROR,"Failed to send nl message with err %d", *status);
		return -1;
	}

	return strlen(info.reply_buf);
}

static int wpa_driver_get_all_sta_info(struct i802_bss *bss, char *buf,
				       size_t buf_len, int *status)
{
	struct hostapd_data *hapd = bss->ctx;
	struct sta_info *sta;
	int ret, total_ret = 0;

	if (!hapd) {
		wpa_printf(MSG_ERROR,"hapd is NULL");
		return -1;
	}

	g_sta_info.num_sta = hapd->num_sta;

	sta = hapd->sta_list;
	while (sta) {
		ret = wpa_driver_send_get_sta_info_cmd(bss, sta->addr, buf,
						       buf_len, status);
		if (ret < 0) {
			wpa_printf(MSG_ERROR,"Failed to get STA info, num_sta %d, sent %d",
			      g_sta_info.num_sta, g_sta_info.num_request_sta_info);
			g_sta_info.num_sta = 0;
			return ret;
		}
		g_sta_info.num_request_sta_info++;
		sta = sta->next;
		total_ret += ret;
	}

	return total_ret;
}

static int wpa_driver_handle_get_sta_info(struct i802_bss *bss, char *cmd,
					  char *buf, size_t buf_len,
					  int *status)
{
	u8 mac[MAC_ADDR_LEN];

	os_memset(&g_sta_info, 0, sizeof(g_sta_info));

	cmd = skip_white_space(cmd);
	if (strlen(cmd) >= MAC_ADDR_LEN * 2 + MAC_ADDR_LEN - 1
	    && convert_string_to_bytes(mac, cmd, MAC_ADDR_LEN) > 0) {
		g_sta_info.num_sta = 1;
		return wpa_driver_send_get_sta_info_cmd(bss, mac, buf, buf_len,
							status);
	}

	return wpa_driver_get_all_sta_info(bss, buf, buf_len, status);
}

int wpa_driver_nl80211_driver_cmd(void *priv, char *cmd, char *buf,
				  size_t buf_len )
{
	struct i802_bss *bss = priv;
	struct wpa_driver_nl80211_data *drv = NULL;
	struct wpa_driver_nl80211_data *driver;
	struct ifreq ifr;
	android_wifi_priv_cmd priv_cmd;
	int ret = 0, status = 0;
	static wpa_driver_oem_cb_table_t oem_cb_table = {NULL};

	if (bss) {
		drv = bss->drv;
	} else {
		if (os_strncasecmp(cmd, "SET_AP_SUSPEND", 14)) {
			wpa_printf(MSG_ERROR, "%s: bss is NULL for cmd %s\n",
				   __func__, cmd);
			return -EINVAL;
		}
	}

	if (wpa_driver_oem_initialize(&oem_cb_table) !=
		WPA_DRIVER_OEM_STATUS_FAILURE) {
		ret = oem_cb_table.wpa_driver_driver_cmd_oem_cb(
				priv, cmd, buf, buf_len, &status);
		if (ret == WPA_DRIVER_OEM_STATUS_SUCCESS ) {
			return strlen(buf);
		} else if ((ret == WPA_DRIVER_OEM_STATUS_FAILURE) &&
							 (status != 0)) {
			wpa_printf(MSG_DEBUG, "%s: Received error: %d",
					__func__, ret);
			return -1;
		}
		/* else proceed with legacy handling as below */
	}

	if (!drv) {
		wpa_printf(MSG_ERROR, "%s: drv is NULL for cmd %s\n",
			   __func__, cmd);
		return -EINVAL;
	}

	if (os_strcasecmp(cmd, "START") == 0) {
		dl_list_for_each(driver, &drv->global->interfaces, struct wpa_driver_nl80211_data, list) {
			linux_set_iface_flags(drv->global->ioctl_sock, driver->first_bss->ifname, 1);
			wpa_msg(drv->ctx, MSG_INFO, WPA_EVENT_DRIVER_STATE "STARTED");
		}
	} else if (os_strcasecmp(cmd, "MACADDR") == 0) {
		u8 macaddr[ETH_ALEN] = {};

		ret = linux_get_ifhwaddr(drv->global->ioctl_sock, bss->ifname, macaddr);
		if (!ret)
			ret = os_snprintf(buf, buf_len,
					  "Macaddr = " MACSTR "\n", MAC2STR(macaddr));
	} else if (os_strncasecmp(cmd, "SET_TXPOWER ", 12) == 0) {
		return wpa_driver_cmd_set_tx_power(priv, cmd + 12);
	} else if(os_strncasecmp(cmd, "GETSTATSBSSINFO", 15) == 0) {

		struct resp_info info;
		struct nl_msg *nlmsg;

		memset(&info, 0, sizeof(struct resp_info));
		info.subcmd = QCA_NL80211_VENDOR_SUBCMD_GET_STATION;
		info.cmd_type = GETSTATSBSSINFO;
		char *p;
		if(wpa_driver_ioctl(bss, "GETCOUNTRYREV", buf, buf_len, &status, drv) == 0){
			p = strstr(buf, " ");
			if(p != NULL)
				memcpy(info.country, (p+1), strlen(p+1)+1);//length of p including null
		}
		cmd += 16;
		os_memset(buf, 0, buf_len);

		info.reply_buf = buf;
		info.reply_buf_len = buf_len;

		nlmsg = prepare_vendor_nlmsg(drv, bss->ifname,
					     info.subcmd);
		if (!nlmsg) {
			wpa_printf(MSG_ERROR,"Failed to allocate nl message");
			return -1;
		}

		if (populate_nlmsg(nlmsg, cmd, info.cmd_type)) {
			wpa_printf(MSG_ERROR,"Failed to populate nl message");
			nlmsg_free(nlmsg);
			return -1;
		}

		status = send_nlmsg((struct nl_sock *)drv->global->nl, nlmsg,
				     response_handler, &info);
		if (status != 0) {
			wpa_printf(MSG_ERROR,"Failed to send nl message with err %d", status);
			return -1;
		}

		return strlen(info.reply_buf);
	} else if (os_strncasecmp(cmd, "GETSTATSSTAINFO", 15) == 0) {
		cmd += 15;
		return wpa_driver_handle_get_sta_info(bss, cmd, buf, buf_len,
						      &status);
	} else if (os_strncasecmp(cmd, "SETCELLSWITCHMODE", 17) == 0) {
		cmd += 17;
		struct resp_info info;
		struct nl_msg *nlmsg;

		memset(&info, 0, sizeof(struct resp_info));

		info.subcmd = QCA_NL80211_VENDOR_SUBCMD_ROAM;
		info.cmd_type = SETCELLSWITCHMODE;

		nlmsg = prepare_vendor_nlmsg(drv, bss->ifname,
					     info.subcmd);
		if (!nlmsg) {
			wpa_printf(MSG_ERROR,"Failed to allocate nl message");
			return WPA_DRIVER_OEM_STATUS_FAILURE;
		}

		if (populate_nlmsg(nlmsg, cmd, info.cmd_type)) {
			wpa_printf(MSG_ERROR,"Failed to populate nl message");
			nlmsg_free(nlmsg);
			return WPA_DRIVER_OEM_STATUS_FAILURE;
		}

		status = send_nlmsg((struct nl_sock *)drv->global->nl, nlmsg,
				     NULL, NULL);
		if (status != 0) {
			wpa_printf(MSG_ERROR,"Failed to send nl message with err %d", status);
			return WPA_DRIVER_OEM_STATUS_FAILURE;
		}

		return WPA_DRIVER_OEM_STATUS_SUCCESS;
	} else { /* Use private command */
		memset(&ifr, 0, sizeof(ifr));
		memset(&priv_cmd, 0, sizeof(priv_cmd));
		os_memcpy(buf, cmd, strlen(cmd) + 1);
		os_strlcpy(ifr.ifr_name, bss->ifname, IFNAMSIZ);

		priv_cmd.buf = buf;
		priv_cmd.used_len = buf_len;
		priv_cmd.total_len = buf_len;
		ifr.ifr_data = &priv_cmd;

		if ((ret = ioctl(drv->global->ioctl_sock, SIOCDEVPRIVATE + 1, &ifr)) < 0) {
			wpa_printf(MSG_ERROR, "%s: failed to issue private commands\n", __func__);
		} else {
			drv_errors = 0;
			if((os_strncasecmp(cmd, "SETBAND", 7) == 0) &&
				ret == DO_NOT_SEND_CHANNEL_CHANGE_EVENT) {
				return 0;
			}

			ret = 0;
			if ((os_strcasecmp(cmd, "LINKSPEED") == 0) ||
			    (os_strcasecmp(cmd, "RSSI") == 0) ||
			    (os_strstr(cmd, "GET") != NULL))
				ret = strlen(buf);
			else if (os_strcasecmp(cmd, "P2P_DEV_ADDR") == 0)
				wpa_printf(MSG_DEBUG, "%s: P2P: Device address ("MACSTR")",
					__func__, MAC2STR(buf));
			else if (os_strcasecmp(cmd, "P2P_SET_PS") == 0)
				wpa_printf(MSG_DEBUG, "%s: P2P: %s ", __func__, buf);
			else if (os_strcasecmp(cmd, "P2P_SET_NOA") == 0)
				wpa_printf(MSG_DEBUG, "%s: P2P: %s ", __func__, buf);
			else if (os_strcasecmp(cmd, "STOP") == 0) {
				wpa_printf(MSG_DEBUG, "%s: %s ", __func__, buf);
				dl_list_for_each(driver, &drv->global->interfaces, struct wpa_driver_nl80211_data, list) {
					linux_set_iface_flags(drv->global->ioctl_sock, driver->first_bss->ifname, 0);
					wpa_msg(drv->ctx, MSG_INFO, WPA_EVENT_DRIVER_STATE "STOPPED");
				}
			}
			else
				wpa_printf(MSG_DEBUG, "%s %s len = %d, %zu", __func__, buf, ret, buf_len);
			wpa_driver_notify_country_change(drv->ctx, cmd);
		}
	}
	return ret;
}

int wpa_driver_set_p2p_noa(void *priv, u8 count, int start, int duration)
{
	char buf[MAX_DRV_CMD_SIZE];

	memset(buf, 0, sizeof(buf));
	wpa_printf(MSG_DEBUG, "%s: Entry", __func__);
	snprintf(buf, sizeof(buf), "P2P_SET_NOA %d %d %d", count, start, duration);
	return wpa_driver_nl80211_driver_cmd(priv, buf, buf, strlen(buf)+1);
}

int wpa_driver_get_p2p_noa(void *priv, u8 *buf, size_t len)
{
	UNUSED(priv), UNUSED(buf), UNUSED(len);
	/* Return 0 till we handle p2p_presence request completely in the driver */
	return 0;
}

int wpa_driver_set_p2p_ps(void *priv, int legacy_ps, int opp_ps, int ctwindow)
{
	char buf[MAX_DRV_CMD_SIZE];

	memset(buf, 0, sizeof(buf));
	wpa_printf(MSG_DEBUG, "%s: Entry", __func__);
	snprintf(buf, sizeof(buf), "P2P_SET_PS %d %d %d", legacy_ps, opp_ps, ctwindow);
	return wpa_driver_nl80211_driver_cmd(priv, buf, buf, strlen(buf) + 1);
}

int wpa_driver_set_ap_wps_p2p_ie(void *priv, const struct wpabuf *beacon,
				 const struct wpabuf *proberesp,
				 const struct wpabuf *assocresp)
{
	UNUSED(priv), UNUSED(beacon), UNUSED(proberesp), UNUSED(assocresp);
	return 0;
}
