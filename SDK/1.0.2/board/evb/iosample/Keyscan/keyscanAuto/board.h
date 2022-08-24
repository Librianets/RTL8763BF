/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file     board.h
* @brief        Pin definitions
* @details
* @author   Ken Mei
* @date     2018-01-19
* @version  v0.1
* *********************************************************************************************************
*/

#ifndef _BOARD_H_
#define _BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

/* keypad row and column */
#define KEYPAD_ROW_SIZE       2
#define KEYPAD_COLUMN_SIZE    2
#define ROW0                  P4_2
#define ROW1                  P4_3

#define COLUMN0               P4_0
#define COLUMN1               P4_1

#define LED0                  P1_3
#define LED1                  P1_4
#define KEYSCAN_INTERVAL      (50)        //50ms

#ifdef __cplusplus
}
#endif

#endif

