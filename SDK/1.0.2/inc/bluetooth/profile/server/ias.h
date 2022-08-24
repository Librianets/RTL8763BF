/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     ias.h
  * @brief    Head file for using immediate alert service.
  * @details  IAS data structs and external functions declaration.
  * @author
  * @date
  * @version  v1.0
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _IAS_H_
#define _IAS_H_

#ifdef  __cplusplus
extern "C" {
#endif      /* __cplusplus */

/* Add Includes here */
#include "profile_server.h"


/** @defgroup IAS Immediate Alert Service
  * @brief Immediate alert service
  * @details

    The Immediate Alert Service (IAS) exposes a control point to allow a peer device to cause the device to immediately alert.

    Immediate Alert Service contains an Alert Level characteristic, to which any value other than "No Alert" being written will trigger alarm in the device.

    Immediate Alert Service generally constitutes a profile collectively with other Services, such as Proximity or Find Me etc., which enables immediate alert in device.

    Application shall registger Immediate Alert Service when initialization through @ref ias_add_service function.

  * @{
  */
/*============================================================================*
 *                              Types
 *============================================================================*/

/** @defgroup IAS_Exported_Types IAS Exported Types
  * @brief
  * @{
  */
/** Message content */
typedef union
{
    uint8_t write_alert_level;
} T_IAS_UPSTREAM_MSG_DATA;

/** IAS service data to inform application */
typedef struct
{
    uint8_t                 conn_id;
    T_SERVICE_CALLBACK_TYPE msg_type;
    T_IAS_UPSTREAM_MSG_DATA msg_data;
} T_IAS_CALLBACK_DATA;

/** @} End of IAS_Exported_Types */

/*============================================================================*
 *                              Functions
 *============================================================================*/

/** @defgroup IAS_Exported_Functions IAS Exported Functions
  * @brief
  * @{
  */

/**
  * @brief Add immediate alert service to the BLE stack database.
  *
  * @param[in]   p_func  Callback when service attribute was read, write or cccd update.
  * @return Service id generated by the BLE stack: @ref T_SERVER_ID.
  * @retval 0xFF Operation failure.
  * @retval others Service id assigned by stack.
  *
  * <b>Example usage</b>
  * \code{.c}
    void profile_init()
    {
        server_init(1);
        ias_id = ias_add_service(app_handle_profile_message);
    }
  * \endcode
  */
T_SERVER_ID ias_add_service(void *p_func);

/** @} End of IAS_Exported_Functions */

/** @} End of IAS */


#ifdef  __cplusplus
}
#endif      /*  __cplusplus */

#endif //_IAS_H
