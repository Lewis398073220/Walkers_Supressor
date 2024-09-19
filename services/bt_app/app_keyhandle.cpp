/***************************************************************************
 *
 * Copyright 2015-2019 BES.
 * All rights reserved. All unpublished rights reserved.
 *
 * No part of this work may be used or reproduced in any form or by any
 * means, or stored in a database or retrieval system, without prior written
 * permission of BES.
 *
 * Use of this work is governed by a license granted by BES.
 * This work contains confidential and proprietary information of
 * BES. which is protected by copyright, trade secret,
 * trademark and other intellectual property rights.
 *
 ****************************************************************************/
//#include "mbed.h"
#include <stdio.h>
#include "cmsis.h"
#include "cmsis_os.h"
#include "hal_uart.h"
#include "hal_timer.h"
#include "audioflinger.h"
#include "lockcqueue.h"
#include "hal_trace.h"
#include "hal_cmu.h"
#include "analog.h"


#include "hfp_api.h"
#include "me_api.h"
#include "a2dp_api.h"
#include "avdtp_api.h"
#include "avctp_api.h"
#include "avrcp_api.h"

#include "besbt.h"

#include "cqueue.h"
#include "btapp.h"
#include "app_key.h"
#include "app_audio.h"

#include "apps.h"
#include "app_bt_stream.h"
#include "app_bt_media_manager.h"
#include "app_bt.h"
#include "app_bt_func.h"
#include "app_hfp.h"
#include "bt_if.h"
#if defined(APP_LINEIN_A2DP_SOURCE)||defined(APP_I2S_A2DP_SOURCE)
#include "nvrecord_env.h"
#endif
#if defined(IBRT)
#include "app_ibrt_ui.h"
#endif

#include "os_api.h"
extern struct BT_DEVICE_T  app_bt_device;
//BT_DEVICE_ID_T g_current_device_id=BT_DEVICE_NUM;  //used to change sco by one-bring-two
//BT_DEVICE_ID_T g_another_device_id=BT_DEVICE_NUM;  //used to change sco by one-bring-two


#ifdef SUPPORT_SIRI
extern int app_hfp_siri_report();
extern int app_hfp_siri_voice(bool en);
int open_siri_flag = 0;
void bt_key_handle_siri_key(enum APP_KEY_EVENT_T event)
{
     switch(event)
     {
        case  APP_KEY_EVENT_NONE:
            if(open_siri_flag == 1){
                TRACE(0,"open siri");
                app_hfp_siri_voice(true);
                open_siri_flag = 0;
            } /*else {
                TRACE(0,"evnet none close siri");
                app_hfp_siri_voice(false);
            }*/
            break;
        case  APP_KEY_EVENT_LONGLONGPRESS:
        case  APP_KEY_EVENT_UP:
            //TRACE(0,"long long/up/click event close siri");
            //app_hfp_siri_voice(false);
            break;
        default:
            TRACE(1,"unregister down key event=%x",event);
            break;
        }
}

#endif

//bool hf_mute_flag = 0;

#if defined (__HSP_ENABLE__)

extern XaStatus app_hs_handle_cmd(HsChannel *Chan,uint8_t cmd_type);

void hsp_handle_key(uint8_t hsp_key)
{
    HsCommand *hs_cmd_p;
    HsChannel *hs_channel_tmp = NULL;
    uint8_t ret= 0;
    hs_channel_tmp = &(app_bt_device.hs_channel[BT_DEVICE_ID_1]);

    if(hsp_key == HSP_KEY_CKPD_CONTROL)
    {
	TRACE(0,"hsp_key = HSP_KEY_CKPD_CONTROL");
	if(app_hs_handle_cmd(hs_channel_tmp,APP_CPKD_CMD) !=0)
            TRACE(0,"app_hs_handle_cmd err");

    }
    else if(hsp_key == HSP_KEY_CHANGE_TO_PHONE)
    {
           TRACE(0,"hsp_key = HSP_KEY_CHANGE_TO_PHONE");
	HS_DisconnectAudioLink(hs_channel_tmp);
    }
    else if(hsp_key == HSP_KEY_ADD_TO_EARPHONE)
    {
        TRACE(0,"hsp_key = HSP_KEY_ADD_TO_EARPHONE");
	HS_CreateAudioLink(hs_channel_tmp);
    }


}
#endif

void hfp_handle_key(uint8_t hfp_key)
{
    hf_chan_handle_t hf_channel_tmp = NULL;
    hf_channel_tmp = app_bt_device.hf_channel[BT_DEVICE_ID_1];

    switch(hfp_key)
    {
        case HFP_KEY_ANSWER_CALL:
            ///answer a incomming call
            TRACE(0,"avrcp_key = HFP_KEY_ANSWER_CALL\n");
            //HF_AnswerCall(hf_channel_tmp,&app_bt_device.hf_command[BT_DEVICE_ID_1]);
            btif_hf_answer_call(hf_channel_tmp);
            break;
        case HFP_KEY_HANGUP_CALL:
            TRACE(0,"avrcp_key = HFP_KEY_HANGUP_CALL\n");
            //HF_Hangup(&app_bt_device.hf_channel[app_bt_device.curr_hf_channel_id],&app_bt_device.hf_command[BT_DEVICE_ID_1]);
            hf_channel_tmp = app_bt_device.hf_channel[app_bt_device.curr_hf_channel_id];
            btif_hf_hang_up_call(hf_channel_tmp);
            break;
        case HFP_KEY_REDIAL_LAST_CALL:
            ///redail the last call
            TRACE(0,"avrcp_key = HFP_KEY_REDIAL_LAST_CALL\n");
            //HF_Redial(&app_bt_device.hf_channel[app_bt_device.curr_hf_channel_id],&app_bt_device.hf_command[BT_DEVICE_ID_1]);
            hf_channel_tmp = app_bt_device.hf_channel[app_bt_device.curr_hf_channel_id];
            btif_hf_redial_call(hf_channel_tmp);
            break;
        case HFP_KEY_CHANGE_TO_PHONE:
            ///remove sco and voice change to phone
            if(app_bt_is_hfp_audio_on())
            {
                TRACE(0,"avrcp_key = HFP_KEY_CHANGE_TO_PHONE\n");
                //HF_DisconnectAudioLink(hf_channel_tmp);
                btif_hf_disc_audio_link(hf_channel_tmp);
            }
            break;
        case HFP_KEY_ADD_TO_EARPHONE:
            ///add a sco and voice change to earphone
            if(!app_bt_is_hfp_audio_on())
            {
                TRACE(1,"avrcp_key = HFP_KEY_ADD_TO_EARPHONE ver:%x\n",  btif_hf_get_version(hf_channel_tmp));
#if defined(HFP_1_6_ENABLE)
                //if (hf_channel_tmp->negotiated_codec == HF_SCO_CODEC_MSBC){
                if (btif_hf_get_negotiated_codec(hf_channel_tmp) == BTIF_HF_SCO_CODEC_MSBC) {
                    TRACE(0,"at+bcc");
#ifdef __HFP_OK__
                    hfp_handle_add_to_earphone_1_6(hf_channel_tmp);
#endif
                    TRACE(0,"CreateAudioLink");
                    btif_hf_create_audio_link(hf_channel_tmp);
                } else
#endif
                {
                    TRACE(0,"CreateAudioLink");
                    //HF_CreateAudioLink(hf_channel_tmp);
                    btif_hf_create_audio_link(hf_channel_tmp);
                }
            }
            break;
        case HFP_KEY_MUTE:
            TRACE(0,"avrcp_key = HFP_KEY_MUTE\n");
            app_bt_device.hf_mute_flag = 1;
            break;
        case HFP_KEY_CLEAR_MUTE:
            TRACE(0,"avrcp_key = HFP_KEY_CLEAR_MUTE\n");
            app_bt_device.hf_mute_flag = 0;
            break;
        case HFP_KEY_THREEWAY_HOLD_AND_ANSWER:
            TRACE(0,"avrcp_key = HFP_KEY_THREEWAY_HOLD_AND_ANSWER\n");
            //HF_CallHold(&app_bt_device.hf_channel[app_bt_device.curr_hf_channel_id],HF_HOLD_HOLD_ACTIVE_CALLS,0,&app_bt_device.hf_command[BT_DEVICE_ID_1]);
            hf_channel_tmp = app_bt_device.hf_channel[app_bt_device.curr_hf_channel_id];
            btif_hf_call_hold(hf_channel_tmp, BTIF_HF_HOLD_HOLD_ACTIVE_CALLS, 0);
            break;
        case HFP_KEY_THREEWAY_HANGUP_AND_ANSWER:
            TRACE(0,"avrcp_key = HFP_KEY_THREEWAY_HOLD_SWAP_ANSWER\n");
            //HF_CallHold(&app_bt_device.hf_channel[app_bt_device.curr_hf_channel_id],HF_HOLD_RELEASE_ACTIVE_CALLS,0,&app_bt_device.hf_command[BT_DEVICE_ID_1]);
            hf_channel_tmp = app_bt_device.hf_channel[app_bt_device.curr_hf_channel_id];
            btif_hf_call_hold(hf_channel_tmp, BTIF_HF_HOLD_RELEASE_ACTIVE_CALLS, 0);
            break;
        case HFP_KEY_THREEWAY_HOLD_REL_INCOMING:
            TRACE(0,"avrcp_key = HFP_KEY_THREEWAY_HOLD_REL_INCOMING\n");
            //HF_CallHold(&app_bt_device.hf_channel[app_bt_device.curr_hf_channel_id],HF_HOLD_RELEASE_HELD_CALLS,0,&app_bt_device.hf_command[BT_DEVICE_ID_1]);
            hf_channel_tmp = app_bt_device.hf_channel[app_bt_device.curr_hf_channel_id];
            btif_hf_call_hold(hf_channel_tmp, BTIF_HF_HOLD_RELEASE_HELD_CALLS, 0);
            break;
        default :
            break;
    }
}

//bool a2dp_play_pause_flag = 0;
uint8_t get_avrcp_via_a2dp_id(uint8_t a2dp_id);
extern void a2dp_handleKey(uint8_t a2dp_key)
{
    btif_avrcp_channel_t* avrcp_channel_tmp = NULL;
    enum BT_DEVICE_ID_T avrcp_id = BT_DEVICE_NUM;
    if(app_bt_device.a2dp_state[app_bt_device.curr_a2dp_stream_id] == 0)
        return;

    avrcp_id = BT_DEVICE_ID_1;
    avrcp_channel_tmp = app_bt_device.avrcp_channel[avrcp_id];

    if (!btif_avrcp_is_control_channel_connected(avrcp_channel_tmp))
    {
        TRACE(1,"avrcp_key %d the channel is not connected", a2dp_key);
        return;
    }

    switch(a2dp_key)
    {
        case AVRCP_KEY_STOP:
            TRACE(0,"avrcp_key = AVRCP_KEY_STOP");
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_STOP,TRUE);
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_STOP,FALSE);
            app_bt_device.a2dp_play_pause_flag = 0;
            break;
            
            /*
             * Wrapper PLAY/PAUSE key as IBRT EVENT to avoid TWS switch interrupt
             */
#if defined(IBRT)
        case AVRCP_KEY_PLAY:
            if (!app_ibrt_ui_event_has_been_queued(IBRT_AUDIO_PLAY))
            {
                TRACE(0,"avrcp_key = AVRCP_KEY_PLAY_INTERNAL event");
                app_ibrt_ui_event_entry(IBRT_AUDIO_PLAY);
            }
            else
            {
                TRACE(0,"avrcp_key = AVRCP_KEY_PLAY_INTERNAL event duplicated");
            }
            break;
            
        case AVRCP_KEY_PAUSE:
            if (!app_ibrt_ui_event_has_been_queued(IBRT_AUDIO_PAUSE))
            {
                TRACE(0,"avrcp_key = AVRCP_KEY_PAUSE_INTERNAL event");
                app_ibrt_ui_event_entry(IBRT_AUDIO_PAUSE);
            }
            else
            {
                TRACE(0,"avrcp_key = AVRCP_KEY_PAUSE_INTERNAL event duplicated");
            }
            break;

        case AVRCP_KEY_PLAY_INTERNAL:
            TRACE(0,"avrcp_key = AVRCP_KEY_PLAY");
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_PLAY,TRUE);
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_PLAY,FALSE);
            app_bt_device.a2dp_play_pause_flag = 1;
            break;
        case AVRCP_KEY_PAUSE_INTERNAL:    
            TRACE(0,"avrcp_key = AVRCP_KEY_PAUSE");
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_PAUSE,TRUE);
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_PAUSE,FALSE);
            app_bt_device.a2dp_play_pause_flag = 0;
            break;
#else            
        case AVRCP_KEY_PLAY:
            TRACE(0,"avrcp_key = AVRCP_KEY_PLAY");
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_PLAY,TRUE);
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_PLAY,FALSE);
            app_bt_device.a2dp_play_pause_flag = 1;
            break;
            
        case AVRCP_KEY_PAUSE:
            TRACE(0,"avrcp_key = AVRCP_KEY_PAUSE");
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_PAUSE,TRUE);
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_PAUSE,FALSE);
            app_bt_device.a2dp_play_pause_flag = 0;
            break;
#endif
        case AVRCP_KEY_FORWARD:
            TRACE(0,"avrcp_key = AVRCP_KEY_FORWARD");
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_FORWARD,TRUE);
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_FORWARD,FALSE);
            app_bt_device.a2dp_play_pause_flag = 1;
            break;
        case AVRCP_KEY_BACKWARD:
            TRACE(0,"avrcp_key = AVRCP_KEY_BACKWARD");
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_BACKWARD,TRUE);
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_BACKWARD,FALSE);
            app_bt_device.a2dp_play_pause_flag = 1;
            break;
        case AVRCP_KEY_VOLUME_UP:
            TRACE(0,"avrcp_key = AVRCP_KEY_VOLUME_UP");
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_VOLUME_UP,TRUE);
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_VOLUME_UP,FALSE);
            break;
        case AVRCP_KEY_VOLUME_DOWN:
            TRACE(0,"avrcp_key = AVRCP_KEY_VOLUME_DOWN");
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_VOLUME_DOWN,TRUE);
            btif_avrcp_set_panel_key(avrcp_channel_tmp,BTIF_AVRCP_POP_VOLUME_DOWN,FALSE);
            break;
        default :
            break;
    }
}

void bt_key_handle_func_click()
{
#if defined (__HSP_ENABLE__)
    if((app_bt_device.hfchan_callSetup[BT_DEVICE_ID_1] == BTIF_HF_CALL_SETUP_NONE)&&
            (app_bt_device.hfchan_call[BT_DEVICE_ID_1] == BTIF_HF_CALL_NONE)&&
            (app_bt_device.hf_audio_state[BT_DEVICE_ID_1] == HF_AUDIO_DISCON)&&
            (app_bt_device.hs_conn_flag[app_bt_device.curr_hs_channel_id]  != 1)){
#else
     if((app_bt_device.hfchan_callSetup[BT_DEVICE_ID_1] == BTIF_HF_CALL_SETUP_NONE)&&
            (app_bt_device.hfchan_call[BT_DEVICE_ID_1] == BTIF_HF_CALL_NONE)&&
            (app_bt_device.hf_audio_state[BT_DEVICE_ID_1] == BTIF_HF_AUDIO_DISCON)){
#endif
        if(app_bt_device.a2dp_play_pause_flag == 0){   /////if no connect?
            a2dp_handleKey(AVRCP_KEY_PLAY);
        }else{
            a2dp_handleKey(AVRCP_KEY_PAUSE);
        }
    }
    if((app_bt_device.hfchan_callSetup[BT_DEVICE_ID_1] == BTIF_HF_CALL_SETUP_IN)&&(app_bt_device.hfchan_call[BT_DEVICE_ID_1] == BTIF_HF_CALL_NONE)){
        hfp_handle_key(HFP_KEY_ANSWER_CALL);
    }
    if(app_bt_device.hfchan_call[BT_DEVICE_ID_1] == BTIF_HF_CALL_ACTIVE){
        hfp_handle_key(HFP_KEY_HANGUP_CALL);
    }
    if((app_bt_device.hfchan_callSetup[BT_DEVICE_ID_1] == BTIF_HF_CALL_SETUP_OUT)||(app_bt_device.hfchan_callSetup[BT_DEVICE_ID_1] == BTIF_HF_CALL_SETUP_ALERT)){
        hfp_handle_key(HFP_KEY_HANGUP_CALL);
    }
#if defined (__HSP_ENABLE__)
    if(app_bt_device.hs_conn_flag[app_bt_device.curr_hs_channel_id]  == 1){         //now we know it is HSP active !
            hsp_handle_key(HSP_KEY_CKPD_CONTROL);
    }
#endif
#if HF_CUSTOM_FEATURE_SUPPORT & HF_CUSTOM_FEATURE_SIRI_REPORT
    //TRACE(0,"powerkey close siri");
    //app_hfp_siri_voice(false);
    open_siri_flag = 0;
#endif
}
                        
void bt_key_handle_func_doubleclick()
{
    TRACE(0,"!!!APP_KEY_EVENT_DOUBLECLICK\n");
    if((app_bt_device.hfchan_callSetup[BT_DEVICE_ID_1] == BTIF_HF_CALL_SETUP_NONE)&&(app_bt_device.hfchan_call[BT_DEVICE_ID_1] == BTIF_HF_CALL_NONE)){
        hfp_handle_key(HFP_KEY_REDIAL_LAST_CALL);
    }
    if(app_bt_device.hf_audio_state[BT_DEVICE_ID_1] == BTIF_HF_AUDIO_CON){
        if(app_bt_device.hf_mute_flag == 0){
            hfp_handle_key(HFP_KEY_MUTE);
            app_bt_device.hf_mute_flag = 1;
        }else{
            hfp_handle_key(HFP_KEY_CLEAR_MUTE);
            app_bt_device.hf_mute_flag = 0;
        }
    }
}

void bt_key_handle_func_longpress()
{
#ifdef SUPPORT_SIRI
    open_siri_flag=0;
#endif
    if((app_bt_device.hfchan_call[BT_DEVICE_ID_1] == BTIF_HF_CALL_ACTIVE) &&
    ((app_bt_device.hfchan_callSetup[BT_DEVICE_ID_1] == BTIF_HF_CALL_SETUP_IN))) {
#ifndef FPGA
        app_voice_report(APP_STATUS_INDICATION_WARNING, 0);
#endif
        hfp_handle_key(HFP_KEY_THREEWAY_HOLD_AND_ANSWER);
    } else  if(app_bt_device.hfchan_call[BT_DEVICE_ID_1] == BTIF_HF_CALL_ACTIVE){
        if(app_bt_device.phone_earphone_mark == 0){
            //call is active, switch from earphone to phone
            hfp_handle_key(HFP_KEY_CHANGE_TO_PHONE);
        }else if(app_bt_device.phone_earphone_mark == 1){
            //call is active, switch from phone to earphone
            hfp_handle_key(HFP_KEY_ADD_TO_EARPHONE);
        }
    } else if(app_bt_device.hfchan_callSetup[BT_DEVICE_ID_1] == BTIF_HF_CALL_SETUP_IN){
        hfp_handle_key(HFP_KEY_HANGUP_CALL);
    }
#if defined (__HSP_ENABLE__)
    else if(app_bt_device.hs_conn_flag[app_bt_device.curr_hs_channel_id]  == 1){         //now we know it is HSP active !
        if(app_bt_device.phone_earphone_mark == 0){
            //call is active, switch from earphone to phone
            hsp_handle_key(HSP_KEY_CHANGE_TO_PHONE);
        }else if(app_bt_device.phone_earphone_mark == 1){
            //call is active, switch from phone to earphone
            hsp_handle_key(HSP_KEY_ADD_TO_EARPHONE);
        }
    }
#endif

#ifdef BTIF_HID_DEVICE
    else if((app_bt_device.hfchan_callSetup[BT_DEVICE_ID_1] == BTIF_HF_CALL_SETUP_NONE) &&
    (app_bt_device.hfchan_call[BT_DEVICE_ID_1] == BTIF_HF_CALL_NONE)) {
        Hid_Send_capture(&app_bt_device.hid_channel[BT_DEVICE_ID_1]);
    }
#endif

#if HF_CUSTOM_FEATURE_SUPPORT & HF_CUSTOM_FEATURE_SIRI_REPORT
    else if((app_bt_device.hfchan_callSetup[BT_DEVICE_ID_1] == BTIF_HF_CALL_SETUP_NONE) &&
    (app_bt_device.hfchan_call[BT_DEVICE_ID_1] == BTIF_HF_CALL_NONE) &&
    (open_siri_flag == 0 )){
#ifndef FPGA
        app_voice_report(APP_STATUS_INDICATION_WARNING, 0);
#endif
        open_siri_flag = 1;
    }
#endif

}

void bt_key_handle_func_key(enum APP_KEY_EVENT_T event)
{
    switch(event)
    {		
        case  APP_KEY_EVENT_UP:
        case  APP_KEY_EVENT_CLICK:
            bt_key_handle_func_click();
            break;
        case  APP_KEY_EVENT_DOUBLECLICK:
            bt_key_handle_func_doubleclick();
            break;
        case  APP_KEY_EVENT_LONGPRESS:
            bt_key_handle_func_longpress();
            break;
        default:
            TRACE(1,"unregister func key event=%x",event);
            break;
    }
}

void app_bt_volumeup()
{
    app_audio_manager_ctrl_volume(APP_AUDIO_MANAGER_VOLUME_CTRL_UP, 0);
}


void app_bt_volumedown()
{
    app_audio_manager_ctrl_volume(APP_AUDIO_MANAGER_VOLUME_CTRL_DOWN, 0);
}

#if defined(__APP_KEY_FN_STYLE_A__)
void bt_key_handle_up_key(enum APP_KEY_EVENT_T event)
{
#if defined(APP_LINEIN_A2DP_SOURCE)||defined(APP_I2S_A2DP_SOURCE)
	struct nvrecord_env_t *nvrecord_env=NULL;
#endif
    switch(event)
    {
        case  APP_KEY_EVENT_UP:
        case  APP_KEY_EVENT_CLICK:
            app_bt_volumeup();
            break;
        case  APP_KEY_EVENT_LONGPRESS:
            a2dp_handleKey(AVRCP_KEY_FORWARD);
            break;
#if defined(APP_LINEIN_A2DP_SOURCE)||defined(APP_I2S_A2DP_SOURCE)
		case  APP_KEY_EVENT_DOUBLECLICK:
			//debug switch src mode
			nv_record_env_get(&nvrecord_env);
			if(app_bt_device.src_or_snk==BT_DEVICE_SRC)
			{
				nvrecord_env->src_snk_flag.src_snk_mode =BT_DEVICE_SNK;
			}
			else
			{
				nvrecord_env->src_snk_flag.src_snk_mode =BT_DEVICE_SRC;
			}
			nv_record_env_set(nvrecord_env);
			app_reset();
			break;
#endif
        default:
            TRACE(1,"unregister up key event=%x",event);
            break;
    }
}


void bt_key_handle_down_key(enum APP_KEY_EVENT_T event)
{
    switch(event)
    {
        case  APP_KEY_EVENT_UP:
        case  APP_KEY_EVENT_CLICK:
            app_bt_volumedown();
            break;
        case  APP_KEY_EVENT_LONGPRESS:
            a2dp_handleKey(AVRCP_KEY_BACKWARD);

            break;
        default:
            TRACE(1,"unregister down key event=%x",event);
            break;
    }
}
#else //#elif defined(__APP_KEY_FN_STYLE_B__)
void bt_key_handle_up_key(enum APP_KEY_EVENT_T event)
{
    TRACE(1,"%s",__func__);
    switch(event)
    {
        case  APP_KEY_EVENT_REPEAT:
            app_bt_volumeup();
            break;
        case  APP_KEY_EVENT_UP:
        case  APP_KEY_EVENT_CLICK:
            a2dp_handleKey(AVRCP_KEY_FORWARD);
            break;
        default:
            TRACE(1,"unregister up key event=%x",event);
            break;
    }
}


void bt_key_handle_down_key(enum APP_KEY_EVENT_T event)
{
    switch(event)
    {
        case  APP_KEY_EVENT_REPEAT:
            app_bt_volumedown();
            break;
        case  APP_KEY_EVENT_UP:
        case  APP_KEY_EVENT_CLICK:
            a2dp_handleKey(AVRCP_KEY_BACKWARD);
            break;
        default:
            TRACE(1,"unregister down key event=%x",event);
            break;
    }
}
#endif

#if defined(APP_LINEIN_A2DP_SOURCE)||defined(APP_I2S_A2DP_SOURCE)
void bt_key_handle_source_func_key(enum APP_KEY_EVENT_T event)
{
    TRACE(2,"%s,%d",__FUNCTION__,event);
    static bool onaudioloop = false;
    switch(event)
    {
        case  APP_KEY_EVENT_UP:
        case  APP_KEY_EVENT_CLICK:
            app_a2dp_source_find_sink();
            break;
        case APP_KEY_EVENT_DOUBLECLICK:
			if(app_bt_device.a2dp_state[0]==1)
			{
				 onaudioloop = onaudioloop?false:true;
				if (onaudioloop)
				{
					app_a2dp_start_stream();

				}
				else
				{
					app_a2dp_suspend_stream();
				}
			}
			break;
        case APP_KEY_EVENT_TRIPLECLICK:
            app_a2dp_start_stream();
            break;
        default:
            TRACE(1,"unregister down key event=%x",event);
            break;
    }
}
#endif

APP_KEY_STATUS bt_key;
static void bt_update_key_event(uint32_t code, uint8_t event)
{
    TRACE(3,"%s code:%d evt:%d",__func__, code, event);

    bt_key.code = code;
    bt_key.event = event;
    osapi_notify_evm();
}

void bt_key_send(APP_KEY_STATUS *status)
{
    uint32_t lock = int_lock();
    bool isKeyBusy = false;
    if (0xff != bt_key.code)
    {
        isKeyBusy = true;
    }
    int_unlock(lock);

    if (!isKeyBusy)
    {
        app_bt_start_custom_function_in_bt_thread(
            (uint32_t)status->code, 
            (uint32_t)status->event,
            (uint32_t)bt_update_key_event);
    }
}

void bt_key_handle(void)
{
    osapi_lock_stack();
    if(bt_key.code != 0xff)
    {
        TRACE(3,"%s code:%d evt:%d",__func__, bt_key.code, bt_key.event);
        switch(bt_key.code)
        {
            case BTAPP_FUNC_KEY:
#if defined(APP_LINEIN_A2DP_SOURCE)||defined(APP_I2S_A2DP_SOURCE)
				if(app_bt_device.src_or_snk==BT_DEVICE_SRC)
				{
					bt_key_handle_source_func_key((enum APP_KEY_EVENT_T)bt_key.event);
				}
				else
#endif
				{
					bt_key_handle_func_key((enum APP_KEY_EVENT_T)bt_key.event);
				}
                break;
            case BTAPP_VOLUME_UP_KEY:
                bt_key_handle_up_key((enum APP_KEY_EVENT_T)bt_key.event);
                break;
            case BTAPP_VOLUME_DOWN_KEY:
                bt_key_handle_down_key((enum APP_KEY_EVENT_T)bt_key.event);
                break;
#ifdef SUPPORT_SIRI
            case BTAPP_RELEASE_KEY:
                bt_key_handle_siri_key((enum APP_KEY_EVENT_T)bt_key.event);
                break;
#endif
            default:
                TRACE(0,"bt_key_handle  undefined key");
                break;
        }
        bt_key.code = 0xff;
    }
    osapi_unlock_stack();
}

void bt_key_init(void)
{
    Besbt_hook_handler_set(BESBT_HOOK_USER_2, bt_key_handle);
    bt_key.code = 0xff;
    bt_key.event = 0xff;
}

