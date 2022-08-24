/**
*********************************************************************************************************
*               Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     uart_cmd.h
* @brief    Define user command.
* @details
* @author   jane
* @date     2019-10-24
* @version  v0.1
*********************************************************************************************************
*/
#ifndef _UART_CMD_H_
#define _UART_CMD_H_

#ifdef  __cplusplus
extern "C" {
#endif      /* __cplusplus */


typedef enum
{
    BLE_START_SCAN = 0,                           	///< BLE start scan command
    BLE_STOP_SCAN,                           		///< BLE stop scan command
    BLE_CONNECT_DEV,                            	///< BLE connect device command
    BLE_DISCONNECT_DEV,                             ///< BLE disconnect device command
    BLE_SHOW_SCAN_DEV,                              ///< BLE show scan device command
    BLE_SHOW_CONN_DEV,                              ///< BLE show connect device command
    BLE_SHOW_BOND_DEV,                              ///< BLE show bond device command
    BLE_PAIR_DEV,                                   ///< BLE pair device command
    BLE_UNPAIR_DEV,                                 ///< BLE unpair device command
    BLE_GET_DEV_NAME,                               ///< BLE get device name command
    BLE_ENABLE_NOTIFY,                              ///< BLE enable notify command
    BLE_COMM_TOTAL,                                 ///< BLE   
}BLE_COMM_t;


char *command_table[BLE_COMM_TOTAL]={"scan",
									"stopscan",
									"condev",
									"disc",
									"showdev",
									"showcon",
									"bondinfo",
									"sauth",
									"del",
									"gapread",
									"simpcccd",
									};

#ifdef  __cplusplus
}
#endif      /*  __cplusplus */

#endif