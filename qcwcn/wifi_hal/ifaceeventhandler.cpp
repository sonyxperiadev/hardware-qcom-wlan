/* Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of The Linux Foundation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

#include "sync.h"
#define LOG_TAG  "WifiHAL"
#include <utils/Log.h>
#include <time.h>

#include "ifaceeventhandler.h"

/* Used to handle NL command events from driver/firmware. */
IfaceEventHandlerCommand *mwifiEventHandler = NULL;

/* Set the interface event monitor handler*/
wifi_error wifi_set_iface_event_handler(wifi_request_id id,
                                        wifi_interface_handle iface,
                                        wifi_event_handler eh)
{
    int i, numAp, ret = 0;
    interface_info *ifaceInfo = getIfaceInfo(iface);
    wifi_handle wifiHandle = getWifiHandle(iface);

    ALOGE("Setting iface event handler, halHandle = %p", wifiHandle);

    /* Check if a similar request to set iface event handler was made earlier.
     * Right now we don't differentiate between the case where (i) the new
     * Request Id is different from the current one vs (ii) both new and
     * Request Ids are the same.
     */
    if (mwifiEventHandler)
    {
        if (id == mwifiEventHandler->get_request_id()) {
            ALOGE("%s: Iface Event Handler Set for request Id %d is still"
                "running. Exit", __func__, id);
            return WIFI_ERROR_TOO_MANY_REQUESTS;
        } else {
            ALOGE("%s: Iface Event Handler Set for a different Request "
                "Id:%d is requested. Not supported. Exit", __func__, id);
            return WIFI_ERROR_NOT_SUPPORTED;
        }
    }

    mwifiEventHandler = new IfaceEventHandlerCommand(
                    wifiHandle,
                    id,
                    NL80211_CMD_REG_CHANGE);
    if (mwifiEventHandler == NULL) {
        ALOGE("%s: Error mwifiEventHandler NULL", __func__);
        return WIFI_ERROR_UNKNOWN;
    }
    mwifiEventHandler->setCallbackHandler(eh);

cleanup:
    return (wifi_error)ret;
}

/* Reset monitoring for the NL event*/
wifi_error wifi_reset_iface_event_handler(wifi_request_id id,
                                          wifi_interface_handle iface)
{
    int ret = 0;

    if (mwifiEventHandler)
    {
        if (id == mwifiEventHandler->get_request_id()) {
            ALOGI("Delete Object mwifiEventHandler for id = %d", id);
            delete mwifiEventHandler;
        } else {
            ALOGE("%s: Iface Event Handler Set for a different Request "
                "Id:%d is requested. Not supported. Exit", __func__, id);
            return WIFI_ERROR_NOT_SUPPORTED;
        }
    } else {
        ALOGI("Object mwifiEventHandler for id = %d already Deleted", id);
    }

cleanup:
    return (wifi_error)ret;
}

/* This function will be the main handler for the registered incoming
 * (from driver) Commads. Calls the appropriate callback handler after
 * parsing the vendor data.
 */
int IfaceEventHandlerCommand::handleEvent(WifiEvent &event)
{
    int ret = WIFI_SUCCESS;

    wifiEventHandler::handleEvent(event);

    switch(mSubcmd)
    {
        case NL80211_CMD_REG_CHANGE:
        {
            if(!tb[NL80211_ATTR_REG_ALPHA2])
            {
                ALOGE("%s: NL80211_ATTR_REG_ALPHA2 not found", __func__);
                return WIFI_ERROR_INVALID_ARGS;
            }
            char code[2];
            memcpy(&code[0], (char *) nla_data(tb[NL80211_ATTR_REG_ALPHA2]), 2); //nla_len(tb[NL80211_ATTR_REG_ALPHA2])
            ALOGI("Country : %c%c", code[0], code[1]);
            if(mHandler.on_country_code_changed)
            {
                mHandler.on_country_code_changed(code);
            }
        }
        break;
        default:
            ALOGI("NL Event : %d Not supported", mSubcmd);
    }

    return NL_SKIP;
}

IfaceEventHandlerCommand::IfaceEventHandlerCommand(wifi_handle handle, int id, u32 subcmd)
        : wifiEventHandler(handle, id, subcmd)
{
    ALOGD("wifiEventHandler %p constructed", this);
    registerHandler(mSubcmd);
    memset(&mHandler, 0, sizeof(wifi_event_handler));
    mEventData = NULL;
    mDataLen = 0;
}

IfaceEventHandlerCommand::~IfaceEventHandlerCommand()
{
    ALOGD("IfaceEventHandlerCommand %p destructor", this);
    unregisterHandler(mSubcmd);
}

void IfaceEventHandlerCommand::setCallbackHandler(wifi_event_handler nHandler)
{
    mHandler = nHandler;
}

int wifiEventHandler::get_request_id()
{
    return mRequestId;
}

int IfaceEventHandlerCommand::get_request_id()
{
    return wifiEventHandler::get_request_id();
}

wifiEventHandler::wifiEventHandler(wifi_handle handle, int id, u32 subcmd)
        : WifiCommand(handle, id)
{
    ALOGD("wifiEventHandler %p constructed", this);
    registerHandler(mSubcmd);
    mRequestId = id;
    mSubcmd = subcmd;
}

wifiEventHandler::~wifiEventHandler()
{
    ALOGD("wifiEventHandler %p destructor", this);
    unregisterHandler(mSubcmd);
}

int wifiEventHandler::handleEvent(WifiEvent &event)
{
    ALOGI("wifiEventHandler::handleEvent");
    struct genlmsghdr *gnlh = event.header();
    mSubcmd = gnlh->cmd;
    nla_parse(tb, NL80211_ATTR_MAX, genlmsg_attrdata(gnlh, 0),
            genlmsg_attrlen(gnlh, 0), NULL);
    ALOGI("Got NL Event : %d from the Driver.", gnlh->cmd);

    return NL_SKIP;
}
