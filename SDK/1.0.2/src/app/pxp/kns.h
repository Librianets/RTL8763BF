/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      pxp_ext.h
* @brief
* @details
* @author    Chuanguo Xue
* @date      2015-3-27
* @version   v0.1
* *********************************************************************************************************
*/

#ifndef _KNS_H
#define _KNS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "profile_server.h"


/****************************************************************************************************************
* macros that other .c files may use all defined here
****************************************************************************************************************/
/** @brief  GATT UUID definitions. */
#define GATT_UUID_EXS  0xFFD0
#define GATT_UUID_CHAR_PARAM 0xFFD1
#define GATT_UUID_CHAR_KEY  0xFFD2

#define GATT_UUID128_KNS    0xA6, 0xF6, 0xF6, 0x07, 0x4D, 0xC4, 0x9D, 0x98, 0x6D, 0x45, 0x29, 0xBB, 0xD0, 0xFF, 0x00, 0x00,
#define GATT_UUID128_CHAR_PARAM   0xA6, 0xF6, 0xF6, 0x07, 0x4D, 0xC4, 0x9D, 0x98, 0x6D, 0x45, 0x29, 0xBB, 0xD1, 0xFF, 0x00, 0x00,
#define GATT_UUID128_CHAR_KEY     0xA6, 0xF6, 0xF6, 0x07, 0x4D, 0xC4, 0x9D, 0x98, 0x6D, 0x45, 0x29, 0xBB, 0xD2, 0xFF, 0x00, 0x00,


/** @brief  PXP Extended characteristics index definitions. */
#define KNS_PARAM_VALUE_INDEX  2
#define KNS_KEY_VALUE_INDEX  4
#define KNS_KEY_VALUE_CCCD_INDEX 5

#define KNS_READ_PARA      2
#define KNS_NOTIFY_ENABLE  1
#define KNS_NOTIFY_DISABLE 0

typedef enum
{
    KNS_PARAM_VALUE
} T_KNS_PARAM_TYPE;
/**
 * @brief PXP extended data struct for notification data to application.
 *
 * PXP extended service data to inform application.
*/
typedef union
{
    uint8_t notification_indification_index;
    uint8_t read_index;
    uint32_t write_value;
} T_KNS_UPSTREAM_MSG_DATA;

typedef struct _TOTA_CALLBACK_DATA
{
    T_SERVICE_CALLBACK_TYPE     msg_type;                    /**<  @brief EventId defined upper */
    T_KNS_UPSTREAM_MSG_DATA  msg_data;
} T_KNS_CALLBACK_DATA;

/* Add PXP Extended service to your application. */
uint8_t kns_add_service(void *pFunc);
bool kns_set_parameter(T_KNS_PARAM_TYPE param_type, uint8_t length, uint32_t *p_value);
#ifdef __cplusplus
}
#endif

#endif /* __HIDS_H */
