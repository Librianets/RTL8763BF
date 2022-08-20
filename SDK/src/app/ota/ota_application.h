/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      simple_ble_peripheral_application.h
* @brief     simple_ble_peripheral_application
* @details   simple_ble_peripheral_application
* @author    jane
* @date      2015-12-22
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef _PERIPHERAL_APPLICATION__
#define _PERIPHERAL_APPLICATION__

#ifdef __cplusplus
extern "C" {
#endif
#include <app_msg.h>
#include <gap_le.h>
#include <profile_server.h>


typedef enum
{
    ADV_IDLE = 0,
    ADV_DIRECT,
    ADV_UNDIRECT_PAIR,
    ADV_UNDIRECT_NONCON,
    ADV_UNDIRECT_POWER,
    ADV_DIRECT_BEFORE_POWER,
} AdvType;

typedef enum
{
    STOP_ADV_REASON_IDLE = 0,
    STOP_ADV_REASON_PAIRING,
    STOP_ADV_REASON_TIMEOUT,
    STOP_ADV_REASON_POWERKEY,
    STOP_ADV_REASON_IRLEARN,
} StopAdvReason;

typedef enum
{
    DISCONN_REASON_IDLE = 0,
    DISCONN_REASON_PAIRING,
    DISCONN_REASON_TIMEOUT,
    DISCONN_REASON_ABNORMAL,
    DISCONN_REASON_OTA,
    DISCONN_REASON_IR_LEARN,
    DISCONN_REASON_DFU_RST,
} DisConReason;

typedef enum
{
    RCU_STATUS_IDLE = 0,                 //IDLE status no link key
    RCU_STATUS_ADVERTISING,
    RCU_STATUS_CONNECTED,                  //connect but not start paring
    RCU_STATUS_PAIRED,                   //rcu paired success
    RCU_STATUS_PAIR_FAILED,         //rcu abnormal paring,paired status = 1(paired  failed).
    RCU_STATES_IR_LEARN,            //IR Learn state
} RcuStatus;

typedef enum
{
    LANTENCY_OFF = 0,
    LANTENCY_UPDATING,
    LANTENCY_ON,
    LANTENCY_ON_CLOSE,
} LantencyStatus;


void         app_handle_io_msg(T_IO_MSG io_driver_msg_recv);
T_APP_RESULT app_profile_callback(T_SERVER_ID service_id, void *p_data);
T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data);

#ifdef __cplusplus
}
#endif

#endif

