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
#include <profile_server.h>
void  app_handle_io_msg(T_IO_MSG io_driver_msg_recv);
void  dfu_service_handle_control_point_req(uint8_t opcode, uint16_t length, uint8_t *p_value);
T_APP_RESULT app_profile_callback(T_SERVER_ID service_id, void *p_data);
T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data);

#ifdef __cplusplus
}
#endif

#endif

