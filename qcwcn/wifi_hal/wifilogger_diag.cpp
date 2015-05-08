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



#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#include <linux/rtnetlink.h>
#include <netinet/in.h>
#include "wifiloggercmd.h"
#include "wifilogger_event_defs.h"
#include "wifilogger_diag.h"
#include "pkt_stats.h"

#define RING_BUF_ENTRY_SIZE 512
#define MAX_CONNECTIVITY_EVENTS 15 // should match the value in wifi_logger.h
static event_remap_t events[MAX_CONNECTIVITY_EVENTS] = {
    {WLAN_PE_DIAG_ASSOC_REQ_EVENT, WIFI_EVENT_ASSOCIATION_REQUESTED},
    {WLAN_PE_DIAG_AUTH_COMP_EVENT, WIFI_EVENT_AUTH_COMPLETE},
    {WLAN_PE_DIAG_ASSOC_COMP_EVENT, WIFI_EVENT_ASSOC_COMPLETE},
    {WLAN_PE_DIAG_AUTH_START_EVENT, WIFI_EVENT_FW_AUTH_STARTED},
    {WLAN_PE_DIAG_ASSOC_START_EVENT, WIFI_EVENT_FW_ASSOC_STARTED},
    {WLAN_PE_DIAG_REASSOC_START_EVENT, WIFI_EVENT_FW_RE_ASSOC_STARTED},
    {WLAN_PE_DIAG_SCAN_REQ_EVENT, WIFI_EVENT_DRIVER_SCAN_REQUESTED},
    {WLAN_PE_DIAG_SCAN_RES_FOUND_EVENT, WIFI_EVENT_DRIVER_SCAN_RESULT_FOUND},
    {WLAN_PE_DIAG_SCAN_COMP_EVENT, WIFI_EVENT_DRIVER_SCAN_COMPLETE},
    {WLAN_PE_DIAG_DISASSOC_REQ_EVENT, WIFI_EVENT_DISASSOCIATION_REQUESTED},
    {WLAN_PE_DIAG_ASSOC_REQ_EVENT, WIFI_EVENT_RE_ASSOCIATION_REQUESTED},
    {WLAN_PE_DIAG_ROAM_AUTH_START_EVENT, WIFI_EVENT_ROAM_AUTH_STARTED},
    {WLAN_PE_DIAG_ROAM_AUTH_COMP_EVENT, WIFI_EVENT_ROAM_AUTH_COMPLETE},
    {WLAN_PE_DIAG_ROAM_ASSOC_START_EVENT, WIFI_EVENT_ROAM_ASSOC_STARTED},
    {WLAN_PE_DIAG_ROAM_ASSOC_COMP_EVENT, WIFI_EVENT_ROAM_ASSOC_COMPLETE},
};

tlv_log* addLoggerTlv(u16 type, u16 length, u8* value, tlv_log *pOutTlv)
{

   pOutTlv->tag = type;
   pOutTlv->length = length;
   memcpy(&pOutTlv->value[0], value, length);

   return((tlv_log *)((u8 *)pOutTlv + sizeof(tlv_log) + length));
}

static wifi_error update_connectivity_ring_buf(hal_info *info,
                                               wifi_ring_buffer_entry *rbe,
                                               u32 size)
{
    struct timeval time;
    u32 total_length = size + sizeof(wifi_ring_buffer_entry);

    rbe->entry_size = size;
    rbe->flags = RING_BUFFER_ENTRY_FLAGS_HAS_BINARY |
                              RING_BUFFER_ENTRY_FLAGS_HAS_TIMESTAMP;
    rbe->type = ENTRY_TYPE_CONNECT_EVENT;
    gettimeofday(&time,NULL);
    rbe->timestamp = time.tv_usec + time.tv_sec * 1000 * 1000;

    /* Write if verbose level and handler are set */
    if (info->rb_infos[CONNECTIVITY_EVENTS_RB_ID].verbose_level >= 1 &&
        info->on_ring_buffer_data)
        return ring_buffer_write(&info->rb_infos[CONNECTIVITY_EVENTS_RB_ID],
                      (u8*)rbe, total_length, 1);

    return WIFI_SUCCESS;
}

static wifi_error process_bt_coex_scan_event(hal_info *info,
                                             u32 id, u8* buf, int length)
{
    wifi_ring_buffer_driver_connectivity_event *pConnectEvent;
    wifi_ring_buffer_entry *pRingBufferEntry;
    tlv_log *pTlv;
    int tot_len;
    u8 out_buf[RING_BUF_ENTRY_SIZE];
    u16 vendor_data;
    wifi_error status;

    pRingBufferEntry = (wifi_ring_buffer_entry *)&out_buf[0];
    memset(pRingBufferEntry, 0, RING_BUF_ENTRY_SIZE);
    pConnectEvent = (wifi_ring_buffer_driver_connectivity_event *)
                     (pRingBufferEntry + 1);
    pTlv = &pConnectEvent->tlvs[0];

    if (id == EVENT_WLAN_BT_COEX_BT_SCAN_START) {
        wlan_bt_coex_bt_scan_start_payload_type *pBtScanStart;
        bt_coex_bt_scan_start_vendor_data_t *pBtScanStartVenData;
        pConnectEvent->event = WIFI_EVENT_BT_COEX_BT_SCAN_START;
        pBtScanStart = (wlan_bt_coex_bt_scan_start_payload_type *)buf;
        pBtScanStartVenData =
              (bt_coex_bt_scan_start_vendor_data_t *)&pBtScanStart->scan_type;
        memcpy(pBtScanStartVenData,
               pBtScanStart, sizeof(bt_coex_bt_scan_start_vendor_data_t));
        pTlv = addLoggerTlv(WIFI_TAG_VENDOR_SPECIFIC,
                            sizeof(bt_coex_bt_scan_start_vendor_data_t),
                            (u8 *)pBtScanStartVenData, pTlv);
        tot_len = sizeof(tlv_log) + sizeof(bt_coex_bt_scan_start_vendor_data_t);
    } else if(id == EVENT_WLAN_BT_COEX_BT_SCAN_STOP) {
        wlan_bt_coex_bt_scan_stop_payload_type *pBtScanStop;
        bt_coex_bt_scan_stop_vendor_data_t *pBtScanStopVenData;
        pConnectEvent->event = WIFI_EVENT_BT_COEX_BT_SCAN_STOP;
        pBtScanStop = (wlan_bt_coex_bt_scan_stop_payload_type *)buf;
        pBtScanStopVenData =
                (bt_coex_bt_scan_stop_vendor_data_t *)&pBtScanStop->scan_type;
        memcpy(pBtScanStopVenData,
               pBtScanStop, sizeof(bt_coex_bt_scan_stop_vendor_data_t));
        pTlv = addLoggerTlv(WIFI_TAG_VENDOR_SPECIFIC,
                            sizeof(bt_coex_bt_scan_stop_vendor_data_t),
                            (u8 *)pBtScanStopVenData, pTlv);
        tot_len = sizeof(tlv_log) + sizeof(bt_coex_bt_scan_stop_vendor_data_t);
    }
    status = update_connectivity_ring_buf(info, pRingBufferEntry, tot_len);
    if (status != WIFI_SUCCESS) {
        ALOGE("Failed to write bt_coex_scan event into ring buffer");
    }

    return status;
}

static wifi_error process_bt_coex_event(hal_info *info, u32 id,
                                        u8* buf, int length)
{
    wifi_ring_buffer_driver_connectivity_event *pConnectEvent;
    wifi_ring_buffer_entry *pRingBufferEntry;
    tlv_log *pTlv;
    int tot_len;
    u8 out_buf[RING_BUF_ENTRY_SIZE];
    bt_coex_common_data *pBtCoexData;
    bt_coex_vendor_data *pBtCoexVenData;
    bt_coex_bt_hid_vendor_data_t *pBtCoexHidVenData;
    u16 vendor_data;
    wifi_error status;

    pRingBufferEntry = (wifi_ring_buffer_entry *)&out_buf[0];
    memset(pRingBufferEntry, 0, RING_BUF_ENTRY_SIZE);
    pConnectEvent = (wifi_ring_buffer_driver_connectivity_event *)
                     (pRingBufferEntry + 1);

    switch (id) {
        case EVENT_WLAN_BT_COEX_BT_SCO_START:
        {
            wlan_bt_coex_bt_sco_start_payload_type *pBtCoexStartPL;
            pBtCoexStartPL = (wlan_bt_coex_bt_sco_start_payload_type *)buf;
            pBtCoexData = (struct bt_coex_common_data *)pBtCoexStartPL;
            pBtCoexVenData =
                      (struct bt_coex_vendor_data *)&pBtCoexStartPL->link_type;
            pConnectEvent->event = WIFI_EVENT_BT_COEX_BT_SCO_START;
        }
        break;
        case EVENT_WLAN_BT_COEX_BT_SCO_STOP:
        {
            wlan_bt_coex_bt_sco_stop_payload_type *pBtCoexStopPL;
            pBtCoexStopPL = (wlan_bt_coex_bt_sco_stop_payload_type *)buf;
            pBtCoexData = (struct bt_coex_common_data *)pBtCoexStopPL;
            pBtCoexVenData =
                    (struct bt_coex_vendor_data *)&pBtCoexStopPL->link_type ;
            pConnectEvent->event = WIFI_EVENT_BT_COEX_BT_SCO_STOP;
        }
        break;
        case EVENT_WIFI_BT_COEX_BT_HID_START:
        {
            wlan_bt_coex_bt_hid_start_payload_type *pBtCoexHidStartPL;
            pBtCoexHidStartPL = (wlan_bt_coex_bt_hid_start_payload_type *)buf;
            pBtCoexData = (struct bt_coex_common_data *)pBtCoexHidStartPL;
            pConnectEvent->event = WIFI_EVENT_BT_COEX_BT_HID_START;
        }
        break;
        case EVENT_WIFI_BT_COEX_BT_HID_STOP:
        {
            wlan_bt_coex_bt_hid_stop_payload_type *pBtCoexHidStopPL;
            pBtCoexHidStopPL = (wlan_bt_coex_bt_hid_stop_payload_type *)buf;
            pBtCoexData = (struct bt_coex_common_data *)pBtCoexHidStopPL;
            pConnectEvent->event = WIFI_EVENT_BT_COEX_BT_HID_STOP;
        }
        break;
        default:
            return WIFI_SUCCESS;
    }

    pTlv = &pConnectEvent->tlvs[0];
    pTlv = addLoggerTlv(WIFI_TAG_LINK_ID, sizeof(pBtCoexData->link_id),
                        (u8 *)&pBtCoexData->link_id, pTlv);
    tot_len = sizeof(tlv_log) + sizeof(pBtCoexData->link_id);
    pTlv = addLoggerTlv(WIFI_TAG_LINK_ROLE, sizeof(pBtCoexData->link_role),
                        (u8 *)&pBtCoexData->link_role, pTlv);
    tot_len += sizeof(tlv_log) + sizeof(pBtCoexData->link_role);
    pTlv = addLoggerTlv(WIFI_TAG_LINK_STATE, sizeof(pBtCoexData->link_state),
                        (u8 *)&pBtCoexData->link_state, pTlv);
    tot_len += sizeof(tlv_log) + sizeof(pBtCoexData->link_state);

    if ((pConnectEvent->event == EVENT_WLAN_BT_COEX_BT_SCO_START) ||
        (pConnectEvent->event == EVENT_WLAN_BT_COEX_BT_SCO_STOP)) {
        pTlv = addLoggerTlv(WIFI_TAG_LINK_TYPE,
                            sizeof(pBtCoexVenData->link_type),
                            (u8 *)&pBtCoexVenData->link_type, pTlv);
        tot_len += sizeof(tlv_log) + sizeof(pBtCoexVenData->link_type);

        pTlv = addLoggerTlv(WIFI_TAG_TSCO, sizeof(pBtCoexVenData->Tsco),
                           (u8 *)&pBtCoexVenData->Tsco, pTlv);
        tot_len += sizeof(tlv_log) + sizeof(pBtCoexVenData->Tsco);
        pTlv = addLoggerTlv(WIFI_TAG_RSCO, sizeof(pBtCoexVenData->Rsco),
                        (u8 *)&pBtCoexVenData->Rsco, pTlv);
        tot_len += sizeof(tlv_log) + sizeof(pBtCoexVenData->Rsco);
    }
    status = update_connectivity_ring_buf(info, pRingBufferEntry, tot_len);
    if (status != WIFI_SUCCESS) {
        ALOGE("Failed to write bt_coex_event into ring buffer");
    }

    return status;
}

static wifi_error process_extscan_event(hal_info *info, u32 id,
                                        u8* buf, int length)
{
    wifi_ring_buffer_driver_connectivity_event *pConnectEvent;
    wifi_ring_buffer_entry *pRingBufferEntry;
    tlv_log *pTlv;
    int tot_len = 0;
    u8 out_buf[RING_BUF_ENTRY_SIZE];
    wifi_error status;

    pRingBufferEntry = (wifi_ring_buffer_entry *)&out_buf[0];
    memset(pRingBufferEntry, 0, RING_BUF_ENTRY_SIZE);
    pConnectEvent = (wifi_ring_buffer_driver_connectivity_event *)
                     (pRingBufferEntry + 1);
    pTlv = &pConnectEvent->tlvs[0];

    switch (id) {
    case EVENT_WLAN_EXTSCAN_CYCLE_STARTED:
    case EVENT_WLAN_EXTSCAN_CYCLE_COMPLETED:
        {
            ext_scan_cycle_vendor_data *pExtScanCycleVenData;
            if (id  == EVENT_WLAN_EXTSCAN_CYCLE_STARTED) {
                wlan_ext_scan_cycle_started_payload_type *pExtScanCycleStarted;
                pConnectEvent->event = WIFI_EVENT_G_SCAN_CYCLE_STARTED;
                pExtScanCycleStarted =
                               (wlan_ext_scan_cycle_started_payload_type *)buf;
                pTlv = addLoggerTlv(WIFI_TAG_SCAN_ID, sizeof(u32),
                                (u8 *)&pExtScanCycleStarted->scan_id, pTlv);
                tot_len += sizeof(tlv_log) + sizeof(u32);
                pExtScanCycleVenData = (ext_scan_cycle_vendor_data *)
                                        (pExtScanCycleStarted + 1);
            } else {
                wlan_ext_scan_cycle_completed_payload_type *pExtScanCycleCompleted;
                pConnectEvent->event = WIFI_EVENT_G_SCAN_CYCLE_COMPLETED;
                pExtScanCycleCompleted =
                (wlan_ext_scan_cycle_completed_payload_type *)buf;
                pTlv = addLoggerTlv(WIFI_TAG_SCAN_ID, sizeof(u32),
                                (u8 *)&pExtScanCycleCompleted->scan_id, pTlv);
                tot_len += sizeof(tlv_log) + sizeof(u32);
                pExtScanCycleVenData = (ext_scan_cycle_vendor_data *)
                                        &pExtScanCycleCompleted->timer_tick ;
            }
            pTlv = addLoggerTlv(WIFI_TAG_VENDOR_SPECIFIC,
                                sizeof(ext_scan_cycle_vendor_data),
                                (u8 *)&pExtScanCycleVenData, pTlv);
            tot_len += sizeof(tlv_log) + sizeof(ext_scan_cycle_vendor_data);
        }
        break;
    case EVENT_WLAN_EXTSCAN_BUCKET_STARTED:
        {
            wlan_ext_scan_bucket_started_payload_type *pExtScanBucketStarted;
            u32 bucket_id;
            pConnectEvent->event = WIFI_EVENT_G_SCAN_BUCKET_STARTED;
            pExtScanBucketStarted =
                            (wlan_ext_scan_bucket_started_payload_type *)buf;
            bucket_id = (u32)pExtScanBucketStarted->bucket_id;
            pTlv = addLoggerTlv(WIFI_TAG_BUCKET_ID, sizeof(u32),
                                (u8 *)&bucket_id, pTlv);
            tot_len += sizeof(tlv_log) + sizeof(u32);
        }
        break;
    case EVENT_WLAN_EXTSCAN_BUCKET_COMPLETED:
        {
            wlan_ext_scan_bucket_completed_payload_type *pExtScanBucketCompleted;
            u32 bucket_id;
            pConnectEvent->event = WIFI_EVENT_G_SCAN_BUCKET_COMPLETED;
            pExtScanBucketCompleted =
                            (wlan_ext_scan_bucket_completed_payload_type *)buf;
            bucket_id = (u32)pExtScanBucketCompleted->bucket_id;
            pTlv = addLoggerTlv(WIFI_TAG_BUCKET_ID, sizeof(u32),
                                (u8 *)&bucket_id, pTlv);
            tot_len += sizeof(tlv_log) + sizeof(u32);
        }
        break;
    case EVENT_WLAN_EXTSCAN_FEATURE_STOP:
        {
            wlan_ext_scan_feature_stop_payload_type *pExtScanStop;
            pConnectEvent->event = WIFI_EVENT_G_SCAN_STOP;
            pExtScanStop = (wlan_ext_scan_feature_stop_payload_type *)buf;
            pTlv = addLoggerTlv(WIFI_TAG_REQUEST_ID,
                                sizeof(wlan_ext_scan_feature_stop_payload_type),
                                (u8 *)&pExtScanStop, pTlv);
            tot_len += sizeof(tlv_log) +
                       sizeof(wlan_ext_scan_feature_stop_payload_type);
        }
        break;
    case EVENT_WLAN_EXTSCAN_RESULTS_AVAILABLE:
        {
            wlan_ext_scan_results_available_payload_type *pExtScanResultsAvail;
            ext_scan_results_available_vendor_data *pExtScanResultsAvailVenData;
            u32 request_id;
            pConnectEvent->event = WIFI_EVENT_G_SCAN_RESULTS_AVAILABLE;
            pExtScanResultsAvail =
                          (wlan_ext_scan_results_available_payload_type *)buf;
            request_id = pExtScanResultsAvail->request_id;
            pTlv = addLoggerTlv(WIFI_TAG_REQUEST_ID, sizeof(u32),
                          (u8 *)&request_id, pTlv);
            tot_len += sizeof(tlv_log) + sizeof(u32);
            pExtScanResultsAvailVenData =
                                    (ext_scan_results_available_vendor_data *)
                                    &pExtScanResultsAvail->table_type;
            pTlv = addLoggerTlv(WIFI_TAG_VENDOR_SPECIFIC,
                                sizeof(ext_scan_results_available_vendor_data),
                                (u8 *)&pExtScanResultsAvailVenData, pTlv);
            tot_len += sizeof(tlv_log) +
                       sizeof(ext_scan_results_available_vendor_data);
        }
        break;
    }

    status = update_connectivity_ring_buf(info, pRingBufferEntry, tot_len);
    if (status != WIFI_SUCCESS) {
        ALOGE("Failed to write ext_scan event into ring buffer");
    }

    return status;
}

static wifi_error process_addba_event(hal_info *info, u32 id,
                                      u8* buf, int length)
{
    wifi_ring_buffer_driver_connectivity_event *pConnectEvent;
    wifi_ring_buffer_entry *pRingBufferEntry;
    tlv_log *pTlv;
    int tot_len;
    u8 out_buf[RING_BUF_ENTRY_SIZE];
    wlan_add_block_ack_success_payload_type *pAddBASuccess;
    wlan_add_block_ack_failed_payload_type *pAddBAFailed;
    u32 reason_code;
    wifi_error status;

    pRingBufferEntry = (wifi_ring_buffer_entry *)&out_buf[0];
    memset(pRingBufferEntry, 0, RING_BUF_ENTRY_SIZE);
    pConnectEvent = (wifi_ring_buffer_driver_connectivity_event *)
                     (pRingBufferEntry + 1);

    pConnectEvent->event = WIFI_EVENT_BLOCK_ACK_NEGOTIATION_COMPLETE;
    pTlv = &pConnectEvent->tlvs[0];
    pTlv = addLoggerTlv(WIFI_TAG_REASON_CODE, sizeof(u32),
                        (u8 *)&reason_code, pTlv);
    tot_len = sizeof(tlv_log) + sizeof(u32);

    if (id == EVENT_WLAN_ADD_BLOCK_ACK_SUCCESS) {
        pTlv = addLoggerTlv(WIFI_TAG_VENDOR_SPECIFIC,
                            sizeof(wlan_add_block_ack_success_payload_type),
                            (u8 *)&pAddBASuccess, pTlv);
        tot_len += sizeof(tlv_log) + sizeof(pAddBASuccess);
    } else {
        pTlv = addLoggerTlv(WIFI_TAG_VENDOR_SPECIFIC,
                            sizeof(wlan_add_block_ack_failed_payload_type),
                            (u8 *)&pAddBAFailed, pTlv);
        tot_len += sizeof(tlv_log) + sizeof(pAddBAFailed);
    }

    status = update_connectivity_ring_buf(info, pRingBufferEntry, tot_len);
    if (status != WIFI_SUCCESS) {
        ALOGE("Failed to write addba event into ring buffer");
    }

    return status;
}

static wifi_error process_roam_event(hal_info *info, u32 id,
                                     u8* buf, int length)
{
    wifi_ring_buffer_driver_connectivity_event *pConnectEvent;
    wifi_ring_buffer_entry *pRingBufferEntry;
    tlv_log *pTlv;
    int tot_len = 0;
    u8 out_buf[RING_BUF_ENTRY_SIZE];
    wifi_error status;

    pRingBufferEntry = (wifi_ring_buffer_entry *)&out_buf[0];
    memset(pRingBufferEntry, 0, RING_BUF_ENTRY_SIZE);
    pConnectEvent = (wifi_ring_buffer_driver_connectivity_event *)
                     (pRingBufferEntry + 1);

    switch (id)
    {
    case EVENT_WLAN_ROAM_SCAN_STARTED:
        {
            wlan_roam_scan_started_payload_type *pRoamScanStarted;
            roam_scan_started_vendor_data_t *pRoamScanStartedVenData;
            pConnectEvent->event = WIFI_EVENT_ROAM_SCAN_STARTED;
            pRoamScanStarted = (wlan_roam_scan_started_payload_type *)buf;
            pTlv = &pConnectEvent->tlvs[0];
            pTlv = addLoggerTlv(WIFI_TAG_SCAN_ID,
                                sizeof(pRoamScanStarted->scan_id),
                                (u8 *)&pRoamScanStarted->scan_id, pTlv);
            tot_len += sizeof(tlv_log) + sizeof(pRoamScanStarted->scan_id);
            pRoamScanStartedVenData = (roam_scan_started_vendor_data_t *)
                                       &pRoamScanStarted->roam_scan_flags;
            pTlv = addLoggerTlv(WIFI_TAG_VENDOR_SPECIFIC,
                                sizeof(roam_scan_started_vendor_data_t),
                                (u8 *)&pRoamScanStartedVenData, pTlv);
            tot_len += sizeof(tlv_log) + sizeof(roam_scan_started_vendor_data_t);
        }
        break;
    case EVENT_WLAN_ROAM_SCAN_COMPLETE:
        {
            wlan_roam_scan_complete_payload_type *pRoamScanComplete;
            roam_scan_complete_vendor_data_t *pRoamScanCompleteVenData;
            pConnectEvent->event = WIFI_EVENT_ROAM_SCAN_COMPLETE;
            pRoamScanComplete = (wlan_roam_scan_complete_payload_type *)buf;
            pTlv = &pConnectEvent->tlvs[0];

            pTlv = addLoggerTlv(WIFI_TAG_SCAN_ID,
                                sizeof(pRoamScanComplete->scan_id),
                                (u8 *)&pRoamScanComplete->scan_id, pTlv);
            tot_len += sizeof(tlv_log) + sizeof(pRoamScanComplete->scan_id);
            pTlv = addLoggerTlv(WIFI_TAG_REASON_CODE,
                                sizeof(pRoamScanComplete->reason),
                                (u8 *)&pRoamScanComplete->reason, pTlv);
            tot_len += sizeof(tlv_log) + sizeof(pRoamScanComplete->reason);

            pRoamScanCompleteVenData = (roam_scan_complete_vendor_data_t *)
                                        &pRoamScanComplete->completion_flags;
            pTlv = addLoggerTlv(WIFI_TAG_VENDOR_SPECIFIC,
                                sizeof(roam_scan_complete_vendor_data_t),
                                (u8 *)&pRoamScanCompleteVenData, pTlv);
            tot_len += sizeof(tlv_log) +
                       sizeof(roam_scan_complete_vendor_data_t);
        }
        break;
    case EVENT_WLAN_ROAM_CANDIDATE_FOUND:
        {
            wlan_roam_candidate_found_payload_type *pRoamCandidateFound;
            roam_candidate_found_vendor_data_t *pRoamCandidateFoundVendata;
            pConnectEvent->event = WIFI_EVENT_ROAM_CANDIDATE_FOUND;
            pRoamCandidateFound = (wlan_roam_candidate_found_payload_type *)buf;
            pTlv = &pConnectEvent->tlvs[0];
            pTlv = addLoggerTlv(WIFI_TAG_CHANNEL,
                                sizeof(pRoamCandidateFound->channel),
                                (u8 *)&pRoamCandidateFound->channel, pTlv);
            tot_len += sizeof(tlv_log) + sizeof(pRoamCandidateFound->channel);

            pTlv = addLoggerTlv(WIFI_TAG_RSSI, sizeof(pRoamCandidateFound->rssi),
                                (u8 *)&pRoamCandidateFound->rssi, pTlv);
            tot_len += sizeof(tlv_log) + sizeof(pRoamCandidateFound->rssi);

            pTlv = addLoggerTlv(WIFI_TAG_BSSID,
                                sizeof(pRoamCandidateFound->bssid),
                                (u8 *)pRoamCandidateFound->bssid, pTlv);
            tot_len += sizeof(tlv_log) + sizeof(pRoamCandidateFound->bssid);

            pRoamCandidateFoundVendata = (roam_candidate_found_vendor_data_t *)
                                          pRoamCandidateFound->ssid;
            pTlv = addLoggerTlv(WIFI_TAG_VENDOR_SPECIFIC,
                                sizeof(roam_candidate_found_vendor_data_t),
                                (u8 *)pRoamCandidateFoundVendata, pTlv);
            tot_len += sizeof(tlv_log) +
                       sizeof(roam_candidate_found_vendor_data_t);
        }
        break;
        case EVENT_WLAN_ROAM_SCAN_CONFIG:
        {
            wlan_roam_scan_config_payload_type *pRoamScanConfig;
            pConnectEvent->event = WIFI_EVENT_ROAM_SCAN_CONFIG;
            pRoamScanConfig = (wlan_roam_scan_config_payload_type *)buf;
            pTlv = &pConnectEvent->tlvs[0];
            pTlv = addLoggerTlv(WIFI_TAG_VENDOR_SPECIFIC,
                                sizeof(wlan_roam_scan_config_payload_type),
                                (u8 *)&pRoamScanConfig, pTlv);
            tot_len += sizeof(tlv_log) +
                       sizeof(wlan_roam_scan_config_payload_type);
        }
        break;
    }

    status = update_connectivity_ring_buf(info, pRingBufferEntry, tot_len);
    if (status != WIFI_SUCCESS) {
        ALOGE("Failed to write roam event into ring buffer");
    }

    return status;
}

static wifi_error process_fw_diag_msg(hal_info *info, u8* buf, int length)
{
    u32 *buffer;
    u32 header1 = 0, header2 = 0, count = 0;
    u32 num_buf = 0, index = 0, diagid = 0, id = 0;
    char *payload;
    u32 payloadlen, timestamp = 0;

    buffer = (u32 *)buf;
    buffer++;

    num_buf = length - 4;

    while (num_buf > count) {
        header1 = *(buffer + index);
        header2 = *(buffer + 1 + index);
        payload = (char *)(buffer + 2 + index);
        diagid  = DIAG_GET_TYPE(header1);
        timestamp = DIAG_GET_TIME_STAMP(header1);
        payloadlen = 0;
#ifdef QC_HAL_DEBUG
        ALOGD("\n diagid = %d  timestamp = %d"
              " header1 = %x heade2 = %x\n",
              diagid,  timestamp, header1, header2);
#endif
        switch (diagid) {
        case WLAN_DIAG_TYPE_EVENT:
        {
            id = DIAG_GET_ID(header2);
            payloadlen = DIAG_GET_PAYLEN16(header2);
#ifdef QC_HAL_DEBUG
            ALOGD("DIAG_TYPE_FW_EVENT: id = %d"
                  " payloadlen = %d \n", id, payloadlen);
#endif
            switch (id) {
                case EVENT_WLAN_BT_COEX_BT_SCO_START:
                case EVENT_WLAN_BT_COEX_BT_SCO_STOP:
                case EVENT_WIFI_BT_COEX_BT_HID_START:
                case EVENT_WIFI_BT_COEX_BT_HID_STOP:
                    process_bt_coex_event(info, id, (u8 *)payload, payloadlen);
                    break;
                case EVENT_WLAN_BT_COEX_BT_SCAN_START:
                case EVENT_WLAN_BT_COEX_BT_SCAN_STOP:
                    process_bt_coex_scan_event(info, id,
                                              (u8 *)payload,
                                              payloadlen);
                    break;
               case EVENT_WLAN_EXTSCAN_CYCLE_STARTED:
               case EVENT_WLAN_EXTSCAN_CYCLE_COMPLETED:
               case EVENT_WLAN_EXTSCAN_BUCKET_STARTED:
               case EVENT_WLAN_EXTSCAN_BUCKET_COMPLETED:
               case EVENT_WLAN_EXTSCAN_FEATURE_STOP:
               case EVENT_WLAN_EXTSCAN_RESULTS_AVAILABLE:
                    process_extscan_event(info, id, (u8 *)payload, payloadlen);
                    break;
               case EVENT_WLAN_ROAM_SCAN_STARTED:
               case EVENT_WLAN_ROAM_SCAN_COMPLETE:
               case EVENT_WLAN_ROAM_CANDIDATE_FOUND:
               case EVENT_WLAN_ROAM_SCAN_CONFIG:
                    process_roam_event(info, id, (u8 *)payload, payloadlen);
                    break;
               case EVENT_WLAN_ADD_BLOCK_ACK_SUCCESS:
               case EVENT_WLAN_ADD_BLOCK_ACK_FAILED:
                    process_addba_event(info, id, (u8 *)payload, payloadlen);
                    break;
               default:
                    return WIFI_SUCCESS;
            }
        }
        break;
        case WLAN_DIAG_TYPE_LOG:
        {
            id = DIAG_GET_ID(header2);
            payloadlen = DIAG_GET_PAYLEN16(header2);
#ifdef QC_HAL_DEBUG
            ALOGD("DIAG_TYPE_FW_LOG: id = %d"
                  " payloadlen = %d \n", id,  payloadlen);
#endif
        }
        break;
        case WLAN_DIAG_TYPE_MSG:
        {
            id = DIAG_GET_ID(header2);
            payloadlen = DIAG_GET_PAYLEN(header2);
#ifdef QC_HAL_DEBUG
            ALOGD("%s: payloadlen = %d \n", __FUNCTION__, payloadlen);
#endif
        }
        break;
        default:
          return WIFI_SUCCESS;
        }
        count  += payloadlen + 8;
        index = count >> 2;
#ifdef QC_HAL_DEBUG
        ALOGD("Loope end:id = %d  payloadlen = %d count = %d index = %d\n",
               id,  payloadlen,  count, index);
#endif
    }
    return WIFI_SUCCESS;
}

static wifi_error remap_event(int in_event, int *out_event)
{
    int i = 0;
    while (i < MAX_CONNECTIVITY_EVENTS) {
        if (events[i].q_event == in_event) {
            *out_event = events[i].g_event;
#ifdef QC_HAL_DEBUG
            ALOGI("Event info %d", *out_event);
#endif
            return WIFI_SUCCESS;
        }
        i++;
    }
    return WIFI_ERROR_UNKNOWN;
}

static wifi_error process_wlan_pe_event(hal_info *info, u8* buf, int length)
{
    wlan_pe_event_t *pWlanPeEvent;
    pe_event_vendor_data_t *pPeEventVenData;
    wifi_ring_buffer_driver_connectivity_event *pConnectEvent;
    wifi_ring_buffer_entry *pRingBufferEntry;
    tlv_log *pTlv;
    int tot_len;
    u8 out_buf[RING_BUF_ENTRY_SIZE];
    u32 vendor_data = 0;
    wifi_error status;

    pWlanPeEvent = (wlan_pe_event_t *)buf;

#ifdef QC_HAL_DEBUG
    ALOGD(MAC_ADDR_STR, MAC_ADDR_ARRAY(pWlanPeEvent->bssid));
    ALOGD("Event type %d Sme State %d mlm_state %d"
          "Status %d Reason code %d \n",
          pWlanPeEvent->event_type,
          pWlanPeEvent->sme_state,
          pWlanPeEvent->mlm_state,
          pWlanPeEvent->status,
          pWlanPeEvent->reason_code);
#endif
    pRingBufferEntry = (wifi_ring_buffer_entry *)&out_buf[0];
    memset(pRingBufferEntry, 0, RING_BUF_ENTRY_SIZE);
    pConnectEvent = (wifi_ring_buffer_driver_connectivity_event *)
                     (pRingBufferEntry + 1);

    status = remap_event(pWlanPeEvent->event_type, (int *)&pConnectEvent->event);
    if (status != WIFI_SUCCESS)
        return status;

    pTlv = &pConnectEvent->tlvs[0];
    pTlv = addLoggerTlv(WIFI_TAG_BSSID, sizeof(pWlanPeEvent->bssid),
                        (u8 *)pWlanPeEvent->bssid, pTlv);
    tot_len = sizeof(tlv_log) + sizeof(pWlanPeEvent->bssid);
    pTlv = addLoggerTlv(WIFI_TAG_STATUS, sizeof(pWlanPeEvent->status),
                        (u8 *)&pWlanPeEvent->status, pTlv);
    tot_len += sizeof(tlv_log) + sizeof(pWlanPeEvent->status);
    pTlv = addLoggerTlv(WIFI_TAG_REASON_CODE, sizeof(pWlanPeEvent->reason_code),
                        (u8 *)&pWlanPeEvent->reason_code, pTlv);
    tot_len += sizeof(tlv_log) + sizeof(pWlanPeEvent->reason_code);

    pPeEventVenData = (pe_event_vendor_data_t *)&pWlanPeEvent->sme_state;
    memcpy(pPeEventVenData, pWlanPeEvent, sizeof(pe_event_vendor_data_t));
    pTlv = addLoggerTlv(WIFI_TAG_VENDOR_SPECIFIC, sizeof(pe_event_vendor_data_t),
                        (u8 *)pPeEventVenData, pTlv);
    tot_len += sizeof(tlv_log) + sizeof(pe_event_vendor_data_t);
    status = update_connectivity_ring_buf(info, pRingBufferEntry, tot_len);
    if (status != WIFI_SUCCESS) {
        ALOGE("Failed to write pe event into ring buffer");
    }

    return status;
}

static wifi_error process_wlan_eapol_event(hal_info *info, u8* buf, int length)
{
    wifi_ring_buffer_driver_connectivity_event *pConnectEvent;
    wlan_eapol_event_t *pWlanEapolEvent;
    wifi_ring_buffer_entry *pRingBufferEntry;
    u8 out_buf[RING_BUF_ENTRY_SIZE];
    int tot_len;
    tlv_log *pTlv;
    u32 eapol_msg_type = 0;
    wifi_error status;

    pWlanEapolEvent = (wlan_eapol_event_t *)buf;
    pRingBufferEntry = (wifi_ring_buffer_entry *)&out_buf[0];
    memset(pRingBufferEntry, 0, RING_BUF_ENTRY_SIZE);
    pConnectEvent = (wifi_ring_buffer_driver_connectivity_event *)
                     (pRingBufferEntry + 1);

    if (pWlanEapolEvent->event_sub_type ==
        WLAN_DRIVER_EAPOL_FRAME_TRANSMIT_REQUESTED)
        pConnectEvent->event = WIFI_EVENT_DRIVER_EAPOL_FRAME_TRANSMIT_REQUESTED;
    else
        pConnectEvent->event = WIFI_EVENT_DRIVER_EAPOL_FRAME_RECEIVED;

    pTlv = &pConnectEvent->tlvs[0];

    if ((pWlanEapolEvent->eapol_key_info & EAPOL_MASK) == EAPOL_M1_MASK)
        eapol_msg_type = 1;
    else if ((pWlanEapolEvent->eapol_key_info & EAPOL_MASK) == EAPOL_M2_MASK)
        eapol_msg_type = 2;
    else if ((pWlanEapolEvent->eapol_key_info & EAPOL_MASK) == EAPOL_M3_MASK)
        eapol_msg_type = 3;
    else if ((pWlanEapolEvent->eapol_key_info & EAPOL_MASK) == EAPOL_M4_MASK)
        eapol_msg_type = 4;
    else
        ALOGI("Unknow EAPOL message type \n");
#ifdef QC_HAL_DEBUG
    ALOGD("EAPOL MSG type %d msg_type 0x%x\n",
          eapol_msg_type, pWlanEapolEvent->eapol_key_info);
    ALOGD(MAC_ADDR_STR, MAC_ADDR_ARRAY(pWlanEapolEvent->dest_addr));
    ALOGD(MAC_ADDR_STR, MAC_ADDR_ARRAY(pWlanEapolEvent->src_addr));
#endif
    pTlv = addLoggerTlv(WIFI_TAG_EAPOL_MESSAGE_TYPE, sizeof(u32),
                        (u8 *)&eapol_msg_type, pTlv);
    tot_len = sizeof(tlv_log) + sizeof(u32);
    pTlv = addLoggerTlv(WIFI_TAG_ADDR1, sizeof(pWlanEapolEvent->dest_addr),
                        (u8 *)pWlanEapolEvent->dest_addr, pTlv);
    tot_len += sizeof(tlv_log) + sizeof(pWlanEapolEvent->dest_addr);
    pTlv = addLoggerTlv(WIFI_TAG_ADDR2, sizeof(pWlanEapolEvent->src_addr),
                        (u8 *)pWlanEapolEvent->src_addr, pTlv);
    tot_len += sizeof(tlv_log) + sizeof(pWlanEapolEvent->src_addr);

    status = update_connectivity_ring_buf(info, pRingBufferEntry, tot_len);
    if (status != WIFI_SUCCESS) {
        ALOGE("Failed to write eapol event into ring buffer");
    }

    return status;
}

static int process_wakelock_event(hal_info *info, u8* buf, int length)
{
    wlan_wake_lock_event_t *pWlanWakeLockEvent;
    wake_lock_event *pWakeLockEvent = NULL;
    wifi_power_event *pPowerEvent = NULL;
    wifi_ring_buffer_entry *pRingBufferEntry = NULL;
    int len_wakelock_event = 0, len_power_event = 0;
    int len_ring_buffer_entry = 0, num_records = 0;
    tlv_log *pTlv;
    struct timeval time;
    wifi_error ret = WIFI_SUCCESS;

    pWlanWakeLockEvent = (wlan_wake_lock_event_t *)(buf);
#ifdef QC_HAL_DEBUG
    ALOGD("wle status = %d reason %d timeout %d name_len %d name %s \n",
          pWlanWakeLockEvent->status, pWlanWakeLockEvent->reason,
          pWlanWakeLockEvent->timeout, pWlanWakeLockEvent->name_len,
          pWlanWakeLockEvent->name);
#endif
    len_wakelock_event = sizeof(wake_lock_event) +
                            pWlanWakeLockEvent->name_len + 1;

    pWakeLockEvent = (wake_lock_event *)malloc(len_wakelock_event);
    if (pWakeLockEvent == NULL) {
        ALOGE("%s: Failed to allocate memory", __func__);
        goto cleanup;
    }

    memset(pWakeLockEvent, 0, len_wakelock_event);

    pWakeLockEvent->status = pWlanWakeLockEvent->status;
    pWakeLockEvent->reason = pWlanWakeLockEvent->reason;
    memcpy(pWakeLockEvent->name, pWlanWakeLockEvent->name,
           pWlanWakeLockEvent->name_len);

    len_power_event = sizeof(wifi_power_event) +
                          sizeof(tlv_log) + len_wakelock_event;
    pPowerEvent = (wifi_power_event *)malloc(len_power_event);
    if (pPowerEvent == NULL) {
        ALOGE("%s: Failed to allocate memory", __func__);
        goto cleanup;
    }

    memset(pPowerEvent, 0, len_power_event);
    pPowerEvent->event = WIFI_TAG_WAKE_LOCK_EVENT;

    pTlv = &pPowerEvent->tlvs[0];
    addLoggerTlv(WIFI_TAG_WAKE_LOCK_EVENT, len_wakelock_event,
                 (u8*)pWakeLockEvent, pTlv);
    len_ring_buffer_entry = sizeof(wifi_ring_buffer_entry) + len_power_event;
    pRingBufferEntry = (wifi_ring_buffer_entry *)malloc(
                                     len_ring_buffer_entry);
    if (pRingBufferEntry == NULL) {
        ALOGE("%s: Failed to allocate memory", __func__);
        goto cleanup;
    }
    memset(pRingBufferEntry, 0, len_ring_buffer_entry);

    pRingBufferEntry->entry_size = len_power_event;
    pRingBufferEntry->flags = RING_BUFFER_ENTRY_FLAGS_HAS_BINARY |
                              RING_BUFFER_ENTRY_FLAGS_HAS_TIMESTAMP;
    pRingBufferEntry->type = ENTRY_TYPE_POWER_EVENT;
    gettimeofday(&time,NULL);
    pRingBufferEntry->timestamp = time.tv_usec + time.tv_sec * 1000 * 1000;

    memcpy(pRingBufferEntry + 1, pPowerEvent, len_power_event);
#ifdef QC_HAL_DEBUG
    ALOGI("Ring buffer Length %d \n", len_ring_buffer_entry);
#endif
    // Write if verbose and handler is set
    num_records = 1;
    if (info->rb_infos[POWER_EVENTS_RB_ID].verbose_level >= 1 &&
        info->on_ring_buffer_data)
        ring_buffer_write(&info->rb_infos[POWER_EVENTS_RB_ID],
                      (u8*)pRingBufferEntry,
                      len_ring_buffer_entry, num_records);
cleanup:
    if (pWakeLockEvent)
        free(pWakeLockEvent);
    if (pPowerEvent)
        free(pPowerEvent);
    if (pRingBufferEntry)
        free(pRingBufferEntry);
    return ret;
}

static wifi_error update_stats_to_ring_buf(hal_info *info,
                      u8 *rb_entry, u32 size)
{
    int num_records = 1;
    wifi_ring_buffer_entry *pRingBufferEntry =
        (wifi_ring_buffer_entry *)rb_entry;
    struct timeval time;

    pRingBufferEntry->entry_size = size - sizeof(wifi_ring_buffer_entry);
    pRingBufferEntry->flags = RING_BUFFER_ENTRY_FLAGS_HAS_BINARY |
                              RING_BUFFER_ENTRY_FLAGS_HAS_TIMESTAMP;
    pRingBufferEntry->type = ENTRY_TYPE_PKT;
    gettimeofday(&time,NULL);
    pRingBufferEntry->timestamp = time.tv_usec + time.tv_sec * 1000 * 1000;

    // Write if verbose and handler is set
    if ((info->rb_infos[PKT_STATS_RB_ID].verbose_level >= VERBOSE_DEBUG_PROBLEM)
        && info->on_ring_buffer_data)
        ring_buffer_write(&info->rb_infos[PKT_STATS_RB_ID],
                          (u8*)pRingBufferEntry,
                          size,
                          num_records);

    return WIFI_SUCCESS;
}

static u16 get_rate(u16 mcs_r, u8 short_gi)
{
    u16 tx_rate = 0;
    MCS mcs;
    static u16 rate_lookup[][8] = {{96, 48, 24, 12, 108, 72, 36, 18},
                            {22, 11,  4,  2,  22, 11,  4,  0}};
    static u16 MCS_rate_lookup_ht[][8] = {{ 13,  14,  27,  30,  59,  65,  117,  130},
                                   { 26,  29,  54,  60, 117, 130,  234,  260},
                                   { 39,  43,  81,  90, 176, 195,  351,  390},
                                   { 52,  58, 108, 120, 234, 260,  468,  520},
                                   { 78,  87, 162, 180, 351, 390,  702,  780},
                                   {104, 116, 216, 240, 468, 520,  936, 1040},
                                   {117, 130, 243, 270, 527, 585, 1053, 1170},
                                   {130, 144, 270, 300, 585, 650, 1170, 1300},
                                   {156, 173, 324, 360, 702, 780, 1404, 1560},
                                   {  0,   0, 360, 400, 780, 867, 1560, 1733},
                                   { 26,  29,  54,  60, 117, 130,  234,  260},
                                   { 52,  58, 108, 120, 234, 260,  468,  520},
                                   { 78,  87, 162, 180, 351, 390,  702,  780},
                                   {104, 116, 216, 240, 468, 520,  936, 1040},
                                   {156, 173, 324, 360, 702, 780, 1404, 1560},
                                   {208, 231, 432, 480, 936,1040, 1872, 2080},
                                   {234, 261, 486, 540,1053,1170, 2106, 2340},
                                   {260, 289, 540, 600,1170,1300, 2340, 2600},
                                   {312, 347, 648, 720,1404,1560, 2808, 3120},
                                   {  0,   0, 720, 800,1560,1733, 3120, 3467}};

    mcs.mcs = mcs_r;
    if ((mcs.mcs_s.preamble < 4) && (mcs.mcs_s.rate < 10)) {
        switch(mcs.mcs_s.preamble)
        {
            case 0:
            case 1:
                if(mcs.mcs_s.rate<8) {
                    tx_rate = rate_lookup [mcs.mcs_s.preamble][mcs.mcs_s.rate];
                    if (mcs.mcs_s.nss)
                        tx_rate *=2;
                } else {
                    ALOGE("Unexpected rate value");
                }
            break;
            case 2:
                if(mcs.mcs_s.rate<8) {
                    if (!mcs.mcs_s.nss)
                        tx_rate = MCS_rate_lookup_ht[mcs.mcs_s.rate][2*mcs.mcs_s.bw+short_gi];
                    else
                        tx_rate = MCS_rate_lookup_ht[10+mcs.mcs_s.rate][2*mcs.mcs_s.bw+short_gi];
                } else {
                    ALOGE("Unexpected HT mcs.mcs_s index");
                }
            break;
            case 3:
                if (!mcs.mcs_s.nss)
                    tx_rate = MCS_rate_lookup_ht[mcs.mcs_s.rate][2*mcs.mcs_s.bw+short_gi];
                else
                    tx_rate = MCS_rate_lookup_ht[10+mcs.mcs_s.rate][2*mcs.mcs_s.bw+short_gi];
            break;
            default:
                ALOGE("Unexpected preamble");
        }
    }
    return tx_rate;
}

static u16 get_rx_rate(u16 mcs)
{
    /* TODO: guard interval is not specified currently */
    return get_rate(mcs, 0);
}

static wifi_error parse_rx_stats(hal_info *info, u8 *buf, u16 size)
{
    wifi_error status;
    rb_pkt_stats_t *rx_stats_rcvd = (rb_pkt_stats_t *)buf;
    u8 rb_pkt_entry_buf[RING_BUF_ENTRY_SIZE];
    wifi_ring_buffer_entry *pRingBufferEntry;
    u32 len_ring_buffer_entry = 0;

    len_ring_buffer_entry = sizeof(wifi_ring_buffer_entry)
                            + sizeof(wifi_ring_per_packet_status_entry)
                            + RX_HTT_HDR_STATUS_LEN;

    if (len_ring_buffer_entry > RING_BUF_ENTRY_SIZE) {
        pRingBufferEntry = (wifi_ring_buffer_entry *)malloc(
                len_ring_buffer_entry);
        if (pRingBufferEntry == NULL) {
            ALOGE("%s: Failed to allocate memory", __FUNCTION__);
            return WIFI_ERROR_OUT_OF_MEMORY;
        }
    } else {
        pRingBufferEntry = (wifi_ring_buffer_entry *)rb_pkt_entry_buf;
    }

    wifi_ring_per_packet_status_entry *rb_pkt_stats =
        (wifi_ring_per_packet_status_entry *)(pRingBufferEntry + 1);

    if (size != sizeof(rb_pkt_stats_t)) {
        ALOGE("%s Unexpected rx stats event length: %d", __FUNCTION__, size);
        return WIFI_ERROR_UNKNOWN;
    }

    memset(rb_pkt_stats, 0, sizeof(wifi_ring_per_packet_status_entry));

    /* Peer tx packet and it is an Rx packet for us */
    rb_pkt_stats->flags |= PER_PACKET_ENTRY_FLAGS_DIRECTION_TX;

    if (!rx_stats_rcvd->mpdu_end.tkip_mic_err)
        rb_pkt_stats->flags |= PER_PACKET_ENTRY_FLAGS_TX_SUCCESS;

    rb_pkt_stats->flags |= PER_PACKET_ENTRY_FLAGS_80211_HEADER;

    if (rx_stats_rcvd->mpdu_start.encrypted)
        rb_pkt_stats->flags |= PER_PACKET_ENTRY_FLAGS_PROTECTED;

    rb_pkt_stats->tid = rx_stats_rcvd->mpdu_start.tid;

    if (rx_stats_rcvd->ppdu_start.preamble_type == PREAMBLE_L_SIG_RATE) {
        if (!rx_stats_rcvd->ppdu_start.l_sig_rate_select)
            rb_pkt_stats->MCS |= 1 << 6;
        rb_pkt_stats->MCS |= rx_stats_rcvd->ppdu_start.l_sig_rate % 8;
        /*BW is 0 for legacy cases*/
    } else if (rx_stats_rcvd->ppdu_start.preamble_type ==
               PREAMBLE_VHT_SIG_A_1) {
        rb_pkt_stats->MCS |= 2 << 6;
        rb_pkt_stats->MCS |=
            (rx_stats_rcvd->ppdu_start.ht_sig_vht_sig_a_1 & BITMASK(7)) %8;
        rb_pkt_stats->MCS |=
            ((rx_stats_rcvd->ppdu_start.ht_sig_vht_sig_a_1 >> 7) & 1) << 8;
    } else if (rx_stats_rcvd->ppdu_start.preamble_type ==
               PREAMBLE_VHT_SIG_A_2) {
        rb_pkt_stats->MCS |= 3 << 6;
        rb_pkt_stats->MCS |=
            (rx_stats_rcvd->ppdu_start.ht_sig_vht_sig_a_2 >> 4) & BITMASK(4);
        rb_pkt_stats->MCS |=
            (rx_stats_rcvd->ppdu_start.ht_sig_vht_sig_a_1 & 3) << 8;
    }
    rb_pkt_stats->last_transmit_rate = get_rx_rate(rb_pkt_stats->MCS);

    rb_pkt_stats->rssi = rx_stats_rcvd->ppdu_start.rssi_comb;
    rb_pkt_stats->link_layer_transmit_sequence
        = rx_stats_rcvd->mpdu_start.seq_num;

    rb_pkt_stats->firmware_entry_timestamp
        = rx_stats_rcvd->ppdu_end.wb_timestamp;

    memcpy(&rb_pkt_stats->data[0], &rx_stats_rcvd->rx_hdr_status[0],
        RX_HTT_HDR_STATUS_LEN);

    status = update_stats_to_ring_buf(info, (u8 *)pRingBufferEntry,
                                      len_ring_buffer_entry);

    if (status != WIFI_SUCCESS) {
        ALOGE("Failed to write Rx stats into the ring buffer");
    }

    if ((u8 *)pRingBufferEntry != rb_pkt_entry_buf) {
        ALOGI("Message with more than RING_BUF_ENTRY_SIZE");
        free (pRingBufferEntry);
    }

    return status;
}

static void parse_tx_rate_and_mcs(struct tx_ppdu_start *ppdu_start,
                                wifi_ring_per_packet_status_entry *rb_pkt_stats)
{
    u16 tx_rate = 0, short_gi = 0;
    MCS mcs;

    if (ppdu_start->valid_s0_bw20) {
        short_gi = ppdu_start->s0_bw20.short_gi;
        mcs.mcs_s.rate      = ppdu_start->s0_bw20.rate;
        mcs.mcs_s.nss       = ppdu_start->s0_bw20.nss;
        mcs.mcs_s.preamble  = ppdu_start->s0_bw20.preamble_type;
        mcs.mcs_s.bw        = BW_20_MHZ;
    } else if (ppdu_start->valid_s0_bw40) {
        short_gi = ppdu_start->s0_bw40.short_gi;
        mcs.mcs_s.rate      = ppdu_start->s0_bw40.rate;
        mcs.mcs_s.nss       = ppdu_start->s0_bw40.nss;
        mcs.mcs_s.preamble  = ppdu_start->s0_bw40.preamble_type;
        mcs.mcs_s.bw        = BW_40_MHZ;
    } else if (ppdu_start->valid_s0_bw80) {
        short_gi = ppdu_start->s0_bw80.short_gi;
        mcs.mcs_s.rate      = ppdu_start->s0_bw80.rate;
        mcs.mcs_s.nss       = ppdu_start->s0_bw80.nss;
        mcs.mcs_s.preamble  = ppdu_start->s0_bw80.preamble_type;
        mcs.mcs_s.bw        = BW_80_MHZ;
    } else if (ppdu_start->valid_s0_bw160) {
        short_gi = ppdu_start->s0_bw160.short_gi;
        mcs.mcs_s.rate      = ppdu_start->s0_bw160.rate;
        mcs.mcs_s.nss       = ppdu_start->s0_bw160.nss;
        mcs.mcs_s.preamble  = ppdu_start->s0_bw160.preamble_type;
        mcs.mcs_s.bw        = BW_160_MHZ;
    } else if (ppdu_start->valid_s1_bw20) {
        short_gi = ppdu_start->s1_bw20.short_gi;
        mcs.mcs_s.rate      = ppdu_start->s1_bw20.rate;
        mcs.mcs_s.nss       = ppdu_start->s1_bw20.nss;
        mcs.mcs_s.preamble  = ppdu_start->s1_bw20.preamble_type;
        mcs.mcs_s.bw        = BW_20_MHZ;
    } else if (ppdu_start->valid_s1_bw40) {
        short_gi = ppdu_start->s1_bw40.short_gi;
        mcs.mcs_s.rate      = ppdu_start->s1_bw40.rate;
        mcs.mcs_s.nss       = ppdu_start->s1_bw40.nss;
        mcs.mcs_s.preamble  = ppdu_start->s1_bw40.preamble_type;
        mcs.mcs_s.bw        = BW_40_MHZ;
    } else if (ppdu_start->valid_s1_bw80) {
        short_gi = ppdu_start->s1_bw80.short_gi;
        mcs.mcs_s.rate      = ppdu_start->s1_bw80.rate;
        mcs.mcs_s.nss       = ppdu_start->s1_bw80.nss;
        mcs.mcs_s.preamble  = ppdu_start->s1_bw80.preamble_type;
        mcs.mcs_s.bw        = BW_80_MHZ;
    } else if (ppdu_start->valid_s1_bw160) {
        short_gi = ppdu_start->s1_bw160.short_gi;
        mcs.mcs_s.rate      = ppdu_start->s1_bw160.rate;
        mcs.mcs_s.nss       = ppdu_start->s1_bw160.nss;
        mcs.mcs_s.preamble  = ppdu_start->s1_bw160.preamble_type;
        mcs.mcs_s.bw        = BW_160_MHZ;
    }

    rb_pkt_stats->MCS = mcs.mcs;
    rb_pkt_stats->last_transmit_rate = get_rate(mcs.mcs, short_gi);
}

static wifi_error parse_tx_stats(hal_info *info, void *buf,
                                 u32 buflen, u8 logtype)
{
    wifi_error status;
    wifi_ring_buffer_entry *pRingBufferEntry =
        (wifi_ring_buffer_entry *)info->pkt_stats->tx_stats;

    wifi_ring_per_packet_status_entry *rb_pkt_stats =
        (wifi_ring_per_packet_status_entry *)(pRingBufferEntry + 1);

    ALOGV("Received Tx stats: log_type : %d", logtype);
    switch (logtype)
    {
        case PKTLOG_TYPE_TX_CTRL:
        {
            if (buflen != sizeof (wh_pktlog_txctl)) {
                ALOGE("Unexpected tx_ctrl event length: %d", buflen);
                return WIFI_ERROR_UNKNOWN;
            }

            wh_pktlog_txctl *stats = (wh_pktlog_txctl *)buf;
            struct tx_ppdu_start *ppdu_start =
                (struct tx_ppdu_start *)(&stats->u.ppdu_start);

            if (ppdu_start->frame_control & BIT(DATA_PROTECTED))
                rb_pkt_stats->flags |=
                    PER_PACKET_ENTRY_FLAGS_PROTECTED;
            rb_pkt_stats->link_layer_transmit_sequence
                = ppdu_start->start_seq_num;
            rb_pkt_stats->tid = ppdu_start->qos_ctl & 0xF;
            parse_tx_rate_and_mcs(ppdu_start, rb_pkt_stats);
            info->pkt_stats->tx_stats_events |=  BIT(PKTLOG_TYPE_TX_CTRL);
        }
        break;
        case PKTLOG_TYPE_TX_STAT:
        {
            if (buflen != sizeof(struct tx_ppdu_end)) {
                ALOGE("Unexpected tx_stat event length: %d", buflen);
                return WIFI_ERROR_UNKNOWN;
            }

            /* This should be the first event for tx-stats: So,
             * previous stats are invalid. Flush the old stats and treat
             * this as new packet
             */
            if (info->pkt_stats->tx_stats_events)
                memset(rb_pkt_stats, 0,
                        sizeof(wifi_ring_per_packet_status_entry));

            struct tx_ppdu_end *tx_ppdu_end = (struct tx_ppdu_end*)(buf);

            if (tx_ppdu_end->stat.tx_ok)
                rb_pkt_stats->flags |=
                    PER_PACKET_ENTRY_FLAGS_TX_SUCCESS;
            rb_pkt_stats->transmit_success_timestamp =
                tx_ppdu_end->try_list.try_00.timestamp;
            rb_pkt_stats->rssi = tx_ppdu_end->stat.ack_rssi_ave;
            rb_pkt_stats->num_retries =
                tx_ppdu_end->stat.total_tries;

            info->pkt_stats->tx_stats_events =  BIT(PKTLOG_TYPE_TX_STAT);
        }
        break;
        case PKTLOG_TYPE_RC_UPDATE:
        case PKTLOG_TYPE_TX_MSDU_ID:
        case PKTLOG_TYPE_TX_FRM_HDR:
        case PKTLOG_TYPE_RC_FIND:
        case PKTLOG_TYPE_TX_VIRT_ADDR:
            ALOGV("%s : Unsupported log_type received : %d",
                  __FUNCTION__, logtype);
        break;
        default:
        {
            ALOGV("%s : Unexpected log_type received : %d",
                  __FUNCTION__, logtype);
            return WIFI_ERROR_UNKNOWN;
        }
    }

    if ((info->pkt_stats->tx_stats_events &  BIT(PKTLOG_TYPE_TX_CTRL))&&
        (info->pkt_stats->tx_stats_events &  BIT(PKTLOG_TYPE_TX_STAT))) {
        /* No tx payload as of now, add the length to parameter size(3rd)
         * if there is any payload
         */
        status = update_stats_to_ring_buf(info,
                                          (u8 *)pRingBufferEntry,
                                     sizeof(wifi_ring_buffer_entry) +
                                     sizeof(wifi_ring_per_packet_status_entry));

        /* Flush the local copy after writing the stats to ring buffer
         * for tx-stats.
         */
        info->pkt_stats->tx_stats_events = 0;
        memset(rb_pkt_stats, 0,
                sizeof(wifi_ring_per_packet_status_entry));

        if (status != WIFI_SUCCESS) {
            ALOGE("Failed to write into the ring buffer: %d", logtype);
            return status;
        }
    }

    return WIFI_SUCCESS;
}

static wifi_error parse_stats_record(hal_info *info, u8 *buf, u16 record_type,
                              u16 record_len)
{
    wifi_error status;
    if (record_type == PKTLOG_TYPE_RX_STAT) {
        status = parse_rx_stats(info, buf, record_len);
    } else {
        status = parse_tx_stats(info, buf, record_len, record_type);
    }
    return status;
}

static wifi_error parse_stats(hal_info *info, u8 *data, u32 buflen)
{
    wh_pktlog_hdr_t *pkt_stats_header;
    wifi_error status = WIFI_SUCCESS;

    do {
        if (buflen < sizeof(wh_pktlog_hdr_t)) {
            status = WIFI_ERROR_INVALID_ARGS;
            break;
        }

        pkt_stats_header = (wh_pktlog_hdr_t *)data;

        if (buflen < (sizeof(wh_pktlog_hdr_t) + pkt_stats_header->size)) {
            status = WIFI_ERROR_INVALID_ARGS;
            break;
        }
        status = parse_stats_record(info,
                                    (u8 *)(pkt_stats_header + 1),
                                    pkt_stats_header->log_type,
                                    pkt_stats_header->size);
        if (status != WIFI_SUCCESS) {
            ALOGE("Failed to parse the stats type : %d",
                  pkt_stats_header->log_type);
            return status;
        }
        data += (sizeof(wh_pktlog_hdr_t) + pkt_stats_header->size);
        buflen -= (sizeof(wh_pktlog_hdr_t) + pkt_stats_header->size);
    } while (buflen > 0);

    return status;
}

wifi_error diag_message_handler(hal_info *info, nl_msg *msg)
{
    tAniNlHdr *wnl = (tAniNlHdr *)nlmsg_hdr(msg);
    u8 *buf;
    wifi_error status;
#ifdef QC_HAL_DEBUG
    ALOGD("event sub type = %x", wnl->wmsg.type);
#endif
    if (wnl->wmsg.type == ANI_NL_MSG_LOG_HOST_EVENT_LOG_TYPE) {
        uint32_t diag_host_type;

        buf = (uint8_t *)(wnl + 1);
        diag_host_type = *(uint32_t *)(buf);
        ALOGV("diag type = %d", diag_host_type);

        buf +=  sizeof(uint32_t); //diag_type
        if (diag_host_type == DIAG_TYPE_HOST_EVENTS) {
            host_event_hdr_t *event_hdr =
                          (host_event_hdr_t *)(buf);
            ALOGV("diag event_id = %d length %d",
                  event_hdr->event_id, event_hdr->length);
            buf += sizeof(host_event_hdr_t);
            switch (event_hdr->event_id) {
                case EVENT_WLAN_WAKE_LOCK:
                    process_wakelock_event(info, buf, event_hdr->length);
                    break;
                case EVENT_WLAN_PE:
                    process_wlan_pe_event(info, buf, event_hdr->length);
                    break;
                case EVENT_WLAN_EAPOL:
                    process_wlan_eapol_event(info, buf, event_hdr->length);
                    break;
                default:
#ifdef QC_HAL_DEBUG
                    ALOGD(":%s: Unsupported Event %d", __FUNCTION__,
                          event_hdr->event_id);
#endif
                    return WIFI_SUCCESS;
            }
        } else if (diag_host_type == DIAG_TYPE_HOST_LOG_MSGS) {
            drv_msg_t *drv_msg = (drv_msg_t *) (buf);
            ALOGV("diag event_type = %0x length = %d",
                  drv_msg->event_type, drv_msg->length);
            if (drv_msg->event_type == WLAN_PKT_LOG_STATS) {
                if ((info->pkt_stats->prev_seq_no + 1) !=
                        drv_msg->u.pkt_stats_event.msg_seq_no) {
                    ALOGE("Few pkt stats messages missed: rcvd = %d, prev = %d",
                            drv_msg->u.pkt_stats_event.msg_seq_no,
                            info->pkt_stats->prev_seq_no);
                    if (info->pkt_stats->tx_stats_events) {
                        info->pkt_stats->tx_stats_events = 0;
                        memset(&info->pkt_stats->tx_stats, 0,
                                sizeof(wifi_ring_per_packet_status_entry));
                    }
                }

                info->pkt_stats->prev_seq_no =
                    drv_msg->u.pkt_stats_event.msg_seq_no;
                status = parse_stats(info,
                        drv_msg->u.pkt_stats_event.payload,
                        drv_msg->u.pkt_stats_event.payload_len);
                if (status != WIFI_SUCCESS) {
                    ALOGE("%s: Failed to parse Tx-Rx stats", __FUNCTION__);
                    ALOGE("Received msg Seq_num : %d",
                            drv_msg->u.pkt_stats_event.msg_seq_no);
                    hexdump((char *)drv_msg->u.pkt_stats_event.payload,
                            drv_msg->u.pkt_stats_event.payload_len);
                    return status;
                }
            }
        }
    } else {
        uint16_t diag_fw_type;
        uint32_t event_id;
        buf = (uint8_t *)NLMSG_DATA(wnl);

        fw_event_hdr_t *event_hdr =
                          (fw_event_hdr_t *)(buf);
        diag_fw_type = event_hdr->diag_type;
#ifdef QC_HAL_DEBUG
        ALOGD("diag_type  = %d diag_length %d",
              event_hdr->diag_type, event_hdr->length);
#endif
        if (diag_fw_type == DIAG_TYPE_FW_MSG) {
            dbglog_slot *slot;
            u16 length = 0;
            u32 version = 0;

            slot = (dbglog_slot *)buf;
            length = get_le32((u8 *)&slot->length);
            process_fw_diag_msg(info, &slot->payload[0], length);
        }
    }
    return WIFI_SUCCESS;
}
