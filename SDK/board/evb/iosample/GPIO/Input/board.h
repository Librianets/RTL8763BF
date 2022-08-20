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

#define Input_Pin            P2_4   //KEY2 In EVB board
#define GPIO_Input_Pin       GPIO_GetPin(Input_Pin)

#define Output_Pin            P1_3 //LED0
#define GPIO_Output_Pin       GPIO_GetPin(Output_Pin)

#ifdef __cplusplus
}
#endif

#endif

