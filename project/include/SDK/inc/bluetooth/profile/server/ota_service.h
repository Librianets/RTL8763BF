/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      ota_service.h
   * @brief     Head file for using OTA service
   * @author    calvin
   * @date      2017-06-07
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

/*============================================================================*
 *                      Define to prevent recursive inclusion
 *============================================================================*/
#ifndef _OTA_SERVICE_H_
#define _OTA_SERVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include "profile_server.h"
#include "app_msg.h"

/** @addtogroup PROFILE_API Profile
  * @{
  */

/** @addtogroup Bluetooth_Services Bluetooth Services
  * @{
  */

/** @defgroup  OTA_SERVICE OTA Service
    * @brief LE Service to implement OTA feature
    * @{
    */

/*============================================================================*
 *                              Types
 *============================================================================*/
/** @defgroup OTA_SERVICE_Exported_Types OTA Service Exported Types
    * @brief
    * @{
    */

/** @brief  OTA write message */
typedef struct _TOTA_WRITE_MSG
{
    uint8_t opcode;
    uint8_t value;
} T_OTA_WRITE_MSG;

/** @brief  OTA upstream message data */
typedef union _TOTA_UPSTREAM_MSG_DATA
{
    uint8_t notification_indification_index;
    uint8_t read_value_index;
    T_OTA_WRITE_MSG write;
} T_OTA_UPSTREAM_MSG_DATA;


/** @brief  OTA service callback data to inform application */
typedef struct
{
    T_SERVICE_CALLBACK_TYPE     msg_type;                   /**<  @brief EventId defined upper */
    uint8_t                     conn_id;
    T_OTA_UPSTREAM_MSG_DATA     msg_data;
} T_OTA_CALLBACK_DATA;

/** End of OTA_SERVICE_Exported_Types
    * @}
    */

/*============================================================================*
 *                              Macros
 *============================================================================*/
/** @defgroup OTA_SERVICE_Exported_Macros OTA service Exported Macros
    * @brief
    * @{
    */

/** @brief  Demo Profile service related UUIDs. */
#define GATT_UUID_CHAR_OTA                          0xFFD1
#define GATT_UUID_CHAR_MAC                          0xFFD2
#define GATT_UUID_CHAR_PATCH                        0xFFD3
#define GATT_UUID_CHAR_APP_VERSION                  0xFFD4
#define GATT_UUID_CHAR_DSP_VERSION                  0xFFD5

/** @brief  Index of each characteristic in Demo Profile service database. */
#define BLE_SERVICE_CHAR_OTA_INDEX                  0x02
#define BLE_SERVICE_CHAR_MAC_ADDRESS_INDEX          0x04
#define BLE_SERVICE_CHAR_PATCH_INDEX                0x06
#define BLE_SERVICE_CHAR_APP_VERSION_INDEX          0x08
#define BLE_SERVICE_CHAR_DSP_VERSION_INDEX          0x0a
#define BLE_SERVICE_CHAR_DFU_PACKET_INDEX           0x0d
#define BLE_SERVICE_CHAR_DFU_CONTROL_POINT_INDEX    0x0f

/** @brief  OTA Write callback data type definition. */
#define OTA_WRITE_CHAR_VAL  0x01
#define OTA_VALUE_ENTER     0x01

/** End of OTA_SERVICE_Exported_Macros
    * @}
    */

/*============================================================================*
 *                              Functions
 *============================================================================*/
/** @defgroup OTA_SERVICE_Exported_Functions OTA service Exported Functions
    * @brief
    * @{
    */

/**
    * @brief    Add OTA BLE service to application
    * @param    p_func  Pointer of APP callback function called by profile
    * @return   Service ID auto generated by profile layer
    * @retval   A T_SERVER_ID type value
    */
extern T_SERVER_ID ota_add_service(void *p_func);

/**
    * @brief    Used to handle BT GAP messages
    * @note     OTA module will handle and only handle BT GAP conn parameter update/disc,
    *           and will leverage LE module to handle other BT GAP messages. Add this to
    *           application if you want to handle connection related IO notification.
    * @param    p_io_msg    Pointer to message to be handled
    * @return   void
    */
extern void ota_handle_gap_message(T_IO_MSG *p_io_msg);

/** End of OTA_SERVICE_Exported_Functions
    * @}
    */


/** End of OTA_SERVICE
    * @}
    */

/** End of Bluetooth_Services
    * @}
    */

/** End of PROFILE_API
    * @}
    */

static T_APP_RESULT ota_service_attr_write_cb(uint8_t conn_id, T_SERVER_ID service_id,
                                              uint16_t attr_index,
                                              T_WRITE_TYPE write_type,
                                              uint16_t length, uint8_t *p_value,
                                              P_FUN_WRITE_IND_POST_PROC *p_write_ind_post_proc);

static T_APP_RESULT ota_service_attr_read_cb(uint8_t conn_id, T_SERVER_ID service_id,
                                             uint16_t attr_index,
                                             uint16_t offset, uint16_t *p_length, uint8_t **pp_value);


#ifdef __cplusplus
}
#endif

#endif
