/**
*********************************************************************************************************
*               Copyright(c) 2014, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     dataTrans_uart.h
* @brief    Data uart operations for testing profiles.
* @details  Data uart init and print data through data uart.
* @author
* @date     2015-03-19
* @version  v0.1
*********************************************************************************************************
*/

#ifndef _DATA_TRANSMIT_UART_H_
#define _DATA_TRANSMIT_UART_H_

#include "rtl876x.h"

#define RECEIVE_BUF_MAX_LENGTH 2500
typedef enum
{
    BAUD_2400           = 0,
    BAUD_4800           = 1,
    BAUD_9600           = 2,
    BAUD_19200          = 3,
    BAUD_38400          = 4,
    BAUD_57600          = 5,
    BAUD_115200         = 6,
    BAUD_921600         = 7,
} BUAD_SET;

typedef struct
{
    uint8_t    buf[RECEIVE_BUF_MAX_LENGTH];
    uint16_t   WriteOffset;
    uint16_t   ReadOffset;
    uint16_t   datalen;
    uint8_t    overflow;
} ReceiveBufStruct;

typedef struct
{
    uint32_t  baudrate;
    bool  uart_flowctrl;
} transfer_info;

void DataTrans_UARTInit(void);
void DataTrans_UARTConfig(void);
void UART0_Handler(void);
uint32_t DataTrans_GetBuadrate(uint8_t baudrate_num);


#endif

