/* Copyright (c) 2015, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __WIFI_HAL_WIFILOGGER_DIAG_H__
#define __WIFI_HAL_WIFILOGGER_DIAG_H__

#include "common.h"
#include "wifi_hal.h"
#include "wifilogger_event_defs.h"

#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#include <linux/rtnetlink.h>

#define DIAG_FWID_OFFSET            24
#define DIAG_FWID_MASK              0xFF000000 /* Bit 24-31 */

#define DIAG_TIMESTAMP_OFFSET       0
#define DIAG_TIMESTAMP_MASK         0x00FFFFFF /* Bit 0-23 */

#define DIAG_ID_OFFSET              16
#define DIAG_ID_MASK                0xFFFF0000 /* Bit 16-31 */

#define DIAG_PAYLEN_OFFSET          0
#define DIAG_PAYLEN_MASK            0x000000FF /* Bit 0-7 */

#define DIAG_PAYLEN_OFFSET16        0
#define DIAG_PAYLEN_MASK16          0x0000FFFF /* Bit 0-16 */



#define DIAG_GET_TYPE(arg) \
     ((arg & DIAG_FWID_MASK) >> DIAG_FWID_OFFSET)

#define DIAG_GET_TIME_STAMP(arg) \
     ((arg & DIAG_TIMESTAMP_MASK) >> DIAG_TIMESTAMP_OFFSET)

#define DIAG_GET_ID(arg) \
     ((arg & DIAG_ID_MASK) >> DIAG_ID_OFFSET)


#define DIAG_GET_PAYLEN(arg) \
     ((arg & DIAG_PAYLEN_MASK) >> DIAG_PAYLEN_OFFSET)

#define DIAG_GET_PAYLEN16(arg) \
     ((arg & DIAG_PAYLEN_MASK16) >> DIAG_PAYLEN_OFFSET16)


#define ANI_NL_MSG_LOG_REG_TYPE  0x0001
#define ANI_NL_MSG_BASE     0x10    /* Some arbitrary base */
#define WIFI_HAL_USER_SOCK_PORT    646
#define WLAN_NL_MSG_CNSS_HOST_EVENT_LOG    17
#define ANI_NL_MSG_LOG_HOST_EVENT_LOG_TYPE 0x5050

#define WLAN_PKT_LOG_STATS            0x18E0

/*
 *  - verbose_level 0 corresponds to no collection
 *  - verbose_level 1 correspond to normal log level, with minimal user impact.
 *    this is the default value
 *  - verbose_level 2 are enabled when user is lazily trying to reproduce a
 problem, wifi performances and power
 *     can be impacted but device should not otherwise be significantly impacted
 *  - verbose_level 3+ are used when trying to actively debug a problem
 */

enum wifilogger_verbose_level {
   VERBOSE_NO_COLLECTION,
   VERBOSE_NORMAL_LOG,
   VERBOSE_REPRO_PROBLEM,
   VERBOSE_DEBUG_PROBLEM
};

enum wifilogger_fw_diag_type {
   DIAG_TYPE_FW_EVENT,     /* send fw event- to diag*/
   DIAG_TYPE_FW_LOG,       /* send log event- to diag*/
   DIAG_TYPE_FW_DEBUG_MSG, /* send dbg message- to diag*/
   DIAG_TYPE_FW_MSG = 4,   /* send fw message- to diag*/
};

enum wifilogger_host_diag_type {
   DIAG_TYPE_HOST_LOG_MSGS=1,
   DIAG_TYPE_HOST_EVENTS=2,
};

enum wlan_diag_frame_type {
     WLAN_DIAG_TYPE_CONFIG,
     WLAN_DIAG_TYPE_EVENT,
     WLAN_DIAG_TYPE_LOG,
     WLAN_DIAG_TYPE_MSG,
     WLAN_DIAG_TYPE_LEGACY_MSG,
};

static uint32_t get_le32(const uint8_t *pos)
{
    return pos[0] | (pos[1] << 8) | (pos[2] << 16) | (pos[3] << 24);
}

typedef struct event_remap {
    int q_event;
    int g_event;
} event_remap_t;

typedef struct {
    u32 diag_type;
    u32 timestamp;
    u32 length;
    u32 dropped;
    /* max ATH6KL_FWLOG_PAYLOAD_SIZE bytes */
    u_int8_t payload[0];
}__attribute__((packed)) dbglog_slot;

typedef enum eAniNlModuleTypes {
    ANI_NL_MSG_NETSIM = ANI_NL_MSG_BASE,// NetSim Messages (to the server)
    ANI_NL_MSG_PUMAC,       // Messages for/from the Upper MAC driver
    ANI_NL_MSG_WNS,         // Messages for the Wireless Networking
                            //  Services module(s)
    ANI_NL_MSG_MACSW,       // Messages from MAC
    ANI_NL_MSG_ES,          // Messages from ES
    ANI_NL_MSG_WSM,         // Message from the WSM in user space
    ANI_NL_MSG_DVT,         // Message from the DVT application
    ANI_NL_MSG_PTT,         // Message from the PTT application
    ANI_NL_MSG_MAC_CLONE,     //Message from the Mac clone App
    ANI_NL_MSG_LOG = ANI_NL_MSG_BASE + 0x0C, // Message for WLAN logging
    ANI_NL_MSG_MAX
} tAniNlModTypes;

//All Netlink messages must contain this header
typedef struct sAniHdr {
   unsigned short type;
   unsigned short length;
} tAniHdr, tAniMsgHdr;

/*
 * This msg hdr will always follow tAniHdr in all the messages exchanged
 * between the Applications in userspace the Pseudo Driver, in either
 * direction.
 */
typedef struct sAniNlMsg {
    struct  nlmsghdr nlh;   // Netlink Header
    int radio;          // unit number of the radio
    tAniHdr wmsg;       // Airgo Message Header
} tAniNlHdr;

typedef struct sAniAppRegReq {
    tAniNlModTypes type;    /* The module id that the application is
                    registering for */
    int pid;            /* Pid returned in the nl_sockaddr structure
                    in the call getsockbyname after the
                    application opens and binds a netlink
                    socket */
} tAniNlAppRegReq;

typedef struct host_event_hdr_s
{
    u16 event_id;
    u16 length;
} host_event_hdr_t;

typedef struct fw_event_hdr_s
{
    u16 diag_type;
    u16 length;
} fw_event_hdr_t;

typedef struct wlan_wake_lock_event {
    u32 status;
    u32 reason;
    u32 timeout;
    u32 name_len;
    char name[];
} wlan_wake_lock_event_t;

wifi_error diag_message_handler(hal_info *info, nl_msg *msg);

typedef struct bt_coex_common_data {
    u8 link_id;
    u8 link_state;
    u8 link_role;
} __attribute__((packed)) bt_coex_common_data_t;

typedef struct bt_coex_vendor_data {
    u8 link_type;
    u16 Tsco;
    u8 Rsco;
} __attribute__((packed)) bt_coex_vendor_data_t;

typedef struct bt_coex_hid_vendor_data {
    u8 Tsniff;
    u8 attempts;
} bt_coex_hid_vendor_data_t;

typedef struct ext_scan_cycle_vendor_data {
    u32 timer_tick;
    u32 scheduled_bucket_mask;
    u32 scan_cycle_count;
} __attribute__((packed)) ext_scan_cycle_vendor_data_t;

typedef struct ext_scan_results_available_vendor_data {
    u32 table_type;
    u32 entries_in_use;
    u32 maximum_entries;
    u32 scan_count_after_getResults;
    u8 threshold_num_scans;
} __attribute__((packed)) ext_scan_results_available_vendor_data;

typedef struct {
    u32 roam_scan_flags;
    u32 cur_rssi;
    u16 scan_params[18];
    u16 scan_channels[40]; // first 40 channels only
} __attribute__((packed)) roam_scan_started_vendor_data_t;

typedef struct {
    u32 completion_flags;
    u32 num_candidate;
    u32 flags;
} __attribute__((packed)) roam_scan_complete_vendor_data_t;

typedef struct {
    u8 ssid[33];
    u8 auth_mode;
    u8 ucast_cipher;
    u8 mcast_cipher;
} __attribute__((packed)) roam_candidate_found_vendor_data_t;

typedef struct {
    u8 scan_type;
    u8 scan_bitmap;
} __attribute__((packed)) bt_coex_bt_scan_start_vendor_data_t;

typedef struct {
    u8 scan_type;
    u8 scan_bitmap;
} __attribute__((packed)) bt_coex_bt_scan_stop_vendor_data_t;

typedef struct {
    u16 sme_state;
    u16 mlm_state;
} __attribute__((packed)) pe_event_vendor_data_t;

typedef struct {
    u8 Tsniff;
    u8 attempts;
} __attribute__((packed)) bt_coex_bt_hid_vendor_data_t;
#endif /* __WIFI_HAL_WIFILOGGER_DIAG_H__ */
