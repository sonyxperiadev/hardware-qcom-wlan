/*
 * Copyright (C) 2014 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "sync.h"

#include "nan.h"
#include "nan_i.h"
#include "wifi_hal.h"
#include "common.h"
#include "cpp_bindings.h"
#include <utils/Log.h>
#include "nancommand.h"

#ifdef __GNUC__
#define PRINTF_FORMAT(a,b) __attribute__ ((format (printf, (a), (b))))
#define STRUCT_PACKED __attribute__ ((packed))
#else
#define PRINTF_FORMAT(a,b)
#define STRUCT_PACKED
#endif

#include "qca-vendor.h"

//Singleton Static Instance
NanCommand* NanCommand::mNanCommandInstance  = NULL;

//Implementation of the functions exposed in nan.h
wifi_error nan_register_handler(wifi_handle handle,
                                NanCallbackHandler handlers,
                                void* user_context)
{
    // Obtain the singleton instance
    int ret = 0;
    NanCommand *nCommand;

    nCommand = NanCommand::instance(handle);
    if (nCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __func__);
        return WIFI_ERROR_UNKNOWN;
    }
    ret = nCommand->setCallbackHandler(handlers, user_context);
    return (wifi_error)ret;
}

wifi_error nan_get_version(wifi_handle handle,
                           NanVersion* version)
{
    *version = (NAN_MAJOR_VERSION <<16 | NAN_MINOR_VERSION << 8 | NAN_MICRO_VERSION);
    return WIFI_SUCCESS;
}

/*  Function to send enable request to the wifi driver.*/
wifi_error nan_enable_request(wifi_request_id id,
                              wifi_handle handle,
                              NanEnableRequest* msg)
{
    int ret = 0;
    NanCommand *nCommand;

    nCommand = NanCommand::instance(handle);
    if (nCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __func__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nCommand->putNanEnable(msg);
    if (ret != 0) {
        ALOGE("%s: putNanEnable Error:%d",__func__, ret);
        goto cleanup;
    }
    nCommand->setId(id);
    ret = nCommand->requestEvent();
    if (ret != 0) {
        ALOGE("%s: requestEvent Error:%d",__func__, ret);
    }
cleanup:
    return (wifi_error)ret;
}

/*  Function to send disable request to the wifi driver.*/
wifi_error nan_disable_request(wifi_request_id id,
                               wifi_handle handle,
                               NanDisableRequest* msg)
{
    int ret = 0;
    NanCommand *nCommand;

    nCommand = NanCommand::instance(handle);
    if (nCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __func__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nCommand->putNanDisable(msg);
    if (ret != 0) {
        ALOGE("%s: putNanDisable Error:%d",__func__, ret);
        goto cleanup;
    }
    nCommand->setId(id);
    ret = nCommand->requestEvent();
    if (ret != 0) {
        ALOGE("%s: requestEvent Error:%d",__func__, ret);
    }
cleanup:
    return (wifi_error)ret;
}

/*  Function to send publish request to the wifi driver.*/
wifi_error nan_publish_request(wifi_request_id id,
                               wifi_handle handle,
                               NanPublishRequest* msg)
{
    int ret = 0;
    NanCommand *nCommand;

    nCommand = NanCommand::instance(handle);
    if (nCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __func__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nCommand->putNanPublish(msg);
    if (ret != 0) {
        ALOGE("%s: putNanPublish Error:%d",__func__, ret);
        goto cleanup;
    }
    nCommand->setId(id);
    ret = nCommand->requestEvent();
    if (ret != 0) {
        ALOGE("%s: requestEvent Error:%d",__func__, ret);
    }
cleanup:
    return (wifi_error)ret;
}

/*  Function to send publish cancel to the wifi driver.*/
wifi_error nan_publish_cancel_request(wifi_request_id id,
                                      wifi_handle handle,
                                      NanPublishCancelRequest* msg)
{
    int ret = 0;
    NanCommand *nCommand;

    nCommand = NanCommand::instance(handle);
    if (nCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __func__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nCommand->putNanPublishCancel(msg);
    if (ret != 0) {
        ALOGE("%s: putNanPublishCancel Error:%d",__func__, ret);
        goto cleanup;
    }
    nCommand->setId(id);
    ret = nCommand->requestEvent();
    if (ret != 0) {
        ALOGE("%s: requestEvent Error:%d",__func__, ret);
    }
cleanup:
    return (wifi_error)ret;
}

/*  Function to send Subscribe request to the wifi driver.*/
wifi_error nan_subscribe_request(wifi_request_id id,
                                 wifi_handle handle,
                                 NanSubscribeRequest* msg)
{
    int ret = 0;
    NanCommand *nCommand;

    nCommand = NanCommand::instance(handle);
    if (nCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __func__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nCommand->putNanSubscribe(msg);
    if (ret != 0) {
        ALOGE("%s: putNanSubscribe Error:%d",__func__, ret);
        goto cleanup;
    }
    nCommand->setId(id);
    ret = nCommand->requestEvent();
    if (ret != 0) {
        ALOGE("%s: requestEvent Error:%d",__func__, ret);
    }
cleanup:
    return (wifi_error)ret;
}

/*  Function to cancel subscribe to the wifi driver.*/
wifi_error nan_subscribe_cancel_request(wifi_request_id id,
                                        wifi_handle handle,
                                        NanSubscribeCancelRequest* msg)
{
    int ret = 0;
    NanCommand *nCommand;

    nCommand = NanCommand::instance(handle);
    if (nCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __func__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nCommand->putNanSubscribeCancel(msg);
    if (ret != 0) {
        ALOGE("%s: putNanSubscribeCancel Error:%d",__func__, ret);
        goto cleanup;
    }
    nCommand->setId(id);
    ret = nCommand->requestEvent();
    if (ret != 0) {
        ALOGE("%s: requestEvent Error:%d",__func__, ret);
    }
cleanup:
    return (wifi_error)ret;
}

/*  Function to send NAN follow up request to the wifi driver.*/
wifi_error nan_transmit_followup_request(wifi_request_id id,
                                         wifi_handle handle,
                                         NanTransmitFollowupRequest* msg)
{
    int ret = 0;
    NanCommand *nCommand;

    nCommand = NanCommand::instance(handle);
    if (nCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __func__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nCommand->putNanTransmitFollowup(msg);
    if (ret != 0) {
        ALOGE("%s: putNanTransmitFollowup Error:%d",__func__, ret);
        goto cleanup;
    }
    nCommand->setId(id);
    ret = nCommand->requestEvent();
    if (ret != 0) {
        ALOGE("%s: requestEvent Error:%d",__func__, ret);
    }
cleanup:
    return (wifi_error)ret;
}

/*  Function to send NAN statistics request to the wifi driver.*/
wifi_error nan_stats_request(wifi_request_id id,
                             wifi_handle handle,
                             NanStatsRequest* msg)
{
    int ret = 0;
    NanCommand *nCommand;

    nCommand = NanCommand::instance(handle);
    if (nCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __func__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nCommand->putNanStats(msg);
    if (ret != 0) {
        ALOGE("%s: putNanStats Error:%d",__func__, ret);
        goto cleanup;
    }
    nCommand->setId(id);
    ret = nCommand->requestEvent();
    if (ret != 0) {
        ALOGE("%s: requestEvent Error:%d",__func__, ret);
    }
cleanup:
    return (wifi_error)ret;
}

/*  Function to send NAN configuration request to the wifi driver.*/
wifi_error nan_config_request(wifi_request_id id,
                              wifi_handle handle,
                              NanConfigRequest* msg)
{
    int ret = 0;
    NanCommand *nCommand;

    nCommand = NanCommand::instance(handle);
    if (nCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __func__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nCommand->putNanConfig(msg);
    if (ret != 0) {
        ALOGE("%s: putNanConfig Error:%d",__func__, ret);
        goto cleanup;
    }
    nCommand->setId(id);
    ret = nCommand->requestEvent();
    if (ret != 0) {
        ALOGE("%s: requestEvent Error:%d",__func__, ret);
    }
cleanup:
    return (wifi_error)ret;
}

/*  Function to send NAN request to the wifi driver.*/
wifi_error nan_tca_request(wifi_request_id id,
                           wifi_handle handle,
                           NanTCARequest* msg)
{
    int ret = 0;
    NanCommand *nCommand;

    nCommand = NanCommand::instance(handle);
    if (nCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __func__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nCommand->putNanTCA(msg);
    if (ret != 0) {
        ALOGE("%s: putNanTCA Error:%d",__func__, ret);
        goto cleanup;
    }
    nCommand->setId(id);
    ret = nCommand->requestEvent();
    if (ret != 0) {
        ALOGE("%s: requestEvent Error:%d",__func__, ret);
    }
cleanup:
    return (wifi_error)ret;
}

/*  Function to send NAN Beacon sdf payload to the wifi driver.
    This instructs the Discovery Engine to begin publishing the
    received payload in any Beacon or Service Discovery Frame
    transmitted*/
wifi_error nan_beacon_sdf_payload_request(wifi_request_id id,
                                         wifi_handle handle,
                                         NanBeaconSdfPayloadRequest* msg)
{
    int ret = WIFI_ERROR_NOT_SUPPORTED;

    NanCommand *nCommand;

    nCommand = NanCommand::instance(handle);
    if (nCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __func__);
        return WIFI_ERROR_UNKNOWN;
    }

    ret = nCommand->putNanBeaconSdfPayload(msg);
    if (ret != 0) {
        ALOGE("%s: putNanBeaconSdfPayload Error:%d",__func__, ret);
        goto cleanup;
    }
    nCommand->setId(id);
    ret = nCommand->requestEvent();
    if (ret != 0) {
        ALOGE("%s: requestEvent Error:%d",__func__, ret);
    }

cleanup:
    return (wifi_error)ret;
}

wifi_error nan_get_sta_parameter(wifi_request_id id,
                                 wifi_handle handle,
                                 NanStaParameter* msg)
{
    int ret = WIFI_ERROR_NOT_SUPPORTED;

    NanCommand *nCommand;

    nCommand = NanCommand::instance(handle);
    if (nCommand == NULL) {
        ALOGE("%s: Error NanCommand NULL", __func__);
        return WIFI_ERROR_UNKNOWN;
    }

    nCommand->setId(id);
    ret = nCommand->getNanStaParameter(msg);
    if (ret != 0) {
        ALOGE("%s: getNanStaParameter Error:%d",__func__, ret);
        goto cleanup;
    }

cleanup:
    return (wifi_error)ret;
}

// Implementation related to nan class common functions
// Constructor
//Making the constructor private since this class is a singleton
NanCommand::NanCommand(wifi_handle handle, int id, u32 vendor_id, u32 subcmd)
        : WifiVendorCommand(handle, id, vendor_id, subcmd)
{
    ALOGV("NanCommand %p constructed", this);
    memset(&mHandler, 0,sizeof(mHandler));
    mNanVendorEvent = NULL;
    mNanDataLen = 0;
    mStaParam = NULL;
    mUserContext = NULL;
}

NanCommand* NanCommand::instance(wifi_handle handle)
{
    if (handle == NULL) {
        ALOGE("Handle is invalid");
        return NULL;
    }
    if (mNanCommandInstance == NULL) {
        mNanCommandInstance = new NanCommand(handle, 0,
                                             OUI_QCA,
                                             QCA_NL80211_VENDOR_SUBCMD_NAN);
        ALOGV("NanCommand %p created", mNanCommandInstance);
        return mNanCommandInstance;
    }
    else
    {
        if (handle != getWifiHandle(mNanCommandInstance->mInfo)) {
            /* upper layer must have cleaned up the handle and reinitialized,
               so we need to update the same */
            ALOGI("Handle different, update the handle");
            mNanCommandInstance->mInfo = (hal_info *)handle;
        }
    }
    ALOGV("NanCommand %p created already", mNanCommandInstance);
    return mNanCommandInstance;
}

NanCommand::~NanCommand()
{
    ALOGV("NanCommand %p destroyed", this);
    unregisterVendorHandler(mVendor_id, mSubcmd);
}

// This function implements creation of Vendor command
// For NAN just call base Vendor command create
int NanCommand::create() {
    return (WifiVendorCommand::create());
}

int NanCommand::handleResponse(WifiEvent reply){
    ALOGI("skipping a response");
    return NL_SKIP;
}

int NanCommand::setCallbackHandler(NanCallbackHandler nHandler,
                                   void *pUserContext)
{
    int res = 0;
    mHandler = nHandler;
    mUserContext = pUserContext;
    res = registerVendorHandler(mVendor_id, mSubcmd);
    if (res != 0) {
        //error case should not happen print log
        ALOGE("%s: Unable to register Vendor Handler Vendor Id=0x%x subcmd=%u",
              __func__, mVendor_id, mSubcmd);
    }
    return res;
}

// This function will be the main handler for incoming event
// QCA_NL80211_VENDOR_SUBCMD_NAN
//Call the appropriate callback handler after parsing the vendor data.
int NanCommand::handleEvent(WifiEvent &event)
{
    ALOGI("Got a NAN message from Driver");
    WifiVendorCommand::handleEvent(event);

    if (mSubcmd == QCA_NL80211_VENDOR_SUBCMD_NAN){
        // Parse the vendordata and get the NAN attribute
        struct nlattr *tb_vendor[QCA_WLAN_VENDOR_ATTR_MAX + 1];
        nla_parse(tb_vendor, QCA_WLAN_VENDOR_ATTR_MAX,
                  (struct nlattr *)mVendorData,
                  mDataLen, NULL);
        // Populating the mNanVendorEvent and mNanDataLen to point to NAN data.
        mNanVendorEvent = (char *)nla_data(tb_vendor[QCA_WLAN_VENDOR_ATTR_NAN]);
        mNanDataLen = nla_len(tb_vendor[QCA_WLAN_VENDOR_ATTR_NAN]);

        if (isNanResponse()) {
            //handleNanResponse will parse the data and call
            //the response callback handler with the populated
            //NanResponseMsg
            handleNanResponse();
        }
        else {
            //handleNanIndication will parse the data and call
            //the corresponding Indication callback handler
            //with the corresponding populated Indication event
            handleNanIndication();
        }
    }
    else {
        //error case should not happen print log
        ALOGE("%s: Wrong NAN subcmd received %d", __func__, mSubcmd);
    }
    return NL_SKIP;
}

/*Helper function to Write and Read TLV called in indication as well as request */
u16 NANTLV_WriteTlv(pNanTlv pInTlv, u8 *pOutTlv)
{
    u16 writeLen = 0;
    u16 i;

    if (!pInTlv)
    {
        ALOGE("NULL pInTlv");
        return writeLen;
    }

    if (!pOutTlv)
    {
        ALOGE("NULL pOutTlv");
        return writeLen;
    }

    *pOutTlv++ = pInTlv->type & 0xFF;
    *pOutTlv++ = (pInTlv->type & 0xFF00) >> 8;
    writeLen += 2;

    ALOGV("WRITE TLV type %u, writeLen %u", pInTlv->type, writeLen);

    *pOutTlv++ = pInTlv->length & 0xFF;
    *pOutTlv++ = (pInTlv->length & 0xFF00) >> 8;
    writeLen += 2;

    ALOGV("WRITE TLV length %u, writeLen %u", pInTlv->length, writeLen);

    for (i=0; i < pInTlv->length; ++i)
    {
        *pOutTlv++ = pInTlv->value[i];
    }

    writeLen += pInTlv->length;
    ALOGV("WRITE TLV value, writeLen %u", writeLen);
    return writeLen;
}

u16 NANTLV_ReadTlv(u8 *pInTlv, pNanTlv pOutTlv)
{
    u16 readLen = 0;

    if (!pInTlv)
    {
        ALOGE("NULL pInTlv");
        return readLen;
    }

    if (!pOutTlv)
    {
        ALOGE("NULL pOutTlv");
        return readLen;
    }

    pOutTlv->type = *pInTlv++;
    pOutTlv->type |= *pInTlv++ << 8;
    readLen += 2;

    ALOGV("READ TLV type %u, readLen %u", pOutTlv->type, readLen);

    pOutTlv->length = *pInTlv++;
    pOutTlv->length |= *pInTlv++ << 8;
    readLen += 2;

    ALOGV("READ TLV length %u, readLen %u", pOutTlv->length, readLen);

    if (pOutTlv->length)
    {
        pOutTlv->value = pInTlv;
        readLen += pOutTlv->length;
    }
    else
    {
        pOutTlv->value = NULL;
    }

    ALOGV("READ TLV value %u, readLen %u", pOutTlv->value, readLen);
    return readLen;
}

u8* addTlv(u16 type, u16 length, const u8* value, u8* pOutTlv)
{
   NanTlv nanTlv;
   u16 len;

   nanTlv.type = type;
   nanTlv.length = length;
   nanTlv.value = (u8*)value;

   len = NANTLV_WriteTlv(&nanTlv, pOutTlv);
   return (pOutTlv + len);
}

void NanCommand::setId(int nId)
{
    mId = nId;
}
