/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      pxp_smart_app.h
   * @brief     This file handles BLE scatternet application routines.
   * @author    ken
   * @date      2017-12-08
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

#ifndef _PXP_SMART_APP_H_
#define _PXP_SMART_APP_H_

#ifdef __cplusplus
extern "C" {
#endif
/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <profile_client.h>
#include <profile_server.h>
#define GATT_UUID_IMMEDIATE_ALERT_SERVICE       0x1802
typedef enum _LinkInfo
{
    NoLink     = 0,
    MasterLink = 1,
    SlaveLink  = 2,
} LinkInfo;
typedef enum _PxpState
{
    PxpStateIdle      = 0,
    PxpStateAdv       = 1,
    PxpStateScan      = 2,
    PxpStateAdvScan   = 3,
    PxpStateLinkM     = 4,
    PxpStateLinkS     = 5,
    PxpStateLinkMADV  = 6,
    PxpStateLinkSScan = 7,
    PxpStateLinkMS    = 8,
} PxpState;

typedef enum _IoState
{
    IoStateIdle     = 0,
    IoStateAdvScanBlink = 1,
    IoStateLinkBlink = 2,
    IoStateImmAlert = 3,
    IoStateLlsAlert = 4
} IoState;

extern PxpState gPxpState;
extern IoState gMIoState;
extern IoState gSIoState;
extern bool gPowerFlg;
extern LinkInfo PXPLINK[];
/*============================================================================*
 *                              Variables
 *============================================================================*/

extern T_CLIENT_ID   gaps_client_id;        /**< gap service client id*/
extern T_CLIENT_ID   ias_client_id;         /**< immediately alert service client id*/
extern T_CLIENT_ID   lls_client_id;         /**< link loss service client id*/
extern T_CLIENT_ID   tps_client_id;         /**< tx power service client id*/
extern T_CLIENT_ID   bas_client_id;         /**< battery service client id*/
extern T_CLIENT_ID   dis_client_id;         /**< device infomation service client id*/
extern T_CLIENT_ID   kns_client_id;         /**< key notification service client id*/

extern T_SERVER_ID ias_srv_id;  /**< Immediately alert service id*/
extern T_SERVER_ID lls_srv_id;  /**< Link loss service id*/
extern T_SERVER_ID tps_srv_id;  /**< Tx power service id*/
extern T_SERVER_ID kns_srv_id;  /**< Key notification service id*/
extern T_SERVER_ID bas_srv_id;  /**< Battery service id */
extern T_SERVER_ID dis_srv_id;  /**< device infomation service id*/
/*============================================================================*
 *                              Functions
 *============================================================================*/
typedef void *TimerHandle_t;
/**
 * @brief    All the application messages are pre-handled in this function
 * @note     All the IO MSGs are sent to this function, then the event handling
 *           function shall be called according to the MSG type.
 * @param[in] io_msg  IO message data
 * @return   void
 */
void app_handle_io_msg(T_IO_MSG io_msg);
/**
  * @brief Callback for gap le to notify app
  * @param[in] cb_type callback msy type @ref GAP_LE_MSG_Types.
  * @param[in] p_cb_data point to callback data @ref T_LE_CB_DATA.
  * @retval result @ref T_APP_RESULT
  */
T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data);

/**
 * @brief  Callback will be called when data sent from profile client layer.
 * @param  client_id the ID distinguish which module sent the data.
 * @param  conn_id connection ID.
 * @param  p_data  pointer to data.
 * @retval   result @ref T_APP_RESULT
 */
T_APP_RESULT app_client_callback(T_CLIENT_ID client_id, uint8_t conn_id, void *p_data);
T_APP_RESULT app_profile_callback(T_SERVER_ID service_id, void *p_data);
void swTimerInit(void);
#if F_BT_AIRPLANE_MODE_SUPPORT
void app_gap_common_callback(uint8_t cb_type, void *p_cb_data);
#endif
#if F_BT_GAPS_CHAR_WRITEABLE
T_APP_RESULT gap_service_callback(T_SERVER_ID service_id, void *p_para);
#endif

#ifdef __cplusplus
}
#endif

#endif

