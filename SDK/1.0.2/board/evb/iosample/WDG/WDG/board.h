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
#define Input_Pin               P4_0      //key 0
#define GPIO_Input_Pin          GPIO_GetPin(Input_Pin)
#define GPIO_Input_Pin_IRQ      GPIO28_IRQn
#define GPIOInputIntrHandler    GPIO28_Handler

#ifdef __cplusplus
}
#endif

#endif

