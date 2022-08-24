/**
*********************************************************************************************************
*               Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     main.c
* @brief    This file provides demo code of ADC Continuous mode.
* @details
* @author   lance
* @date     2018-02-05
* @version  v1.0
*********************************************************************************************************
*/
/* Include ---------------------------------------------------------------*/
#include "rtl876x_rcc.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_adc.h"
#include "rtl876x_nvic.h"
#include "rtl876x_tim.h"
#include "rtl876x_wdg.h"
#include "trace.h"
#include "board.h"
#include <os_sched.h>
/* Defines --------------------------------------------------------------*/
#define ADC_Channel_index               5
#define ADC_Schedule_index              0
/**
  * @brief  initialization of pinmux settings and pad settings.
  * @param   No parameter.
  * @return  void
  */
void Board_ADC_init(void)
{
    Pad_Config(ADC0 + ADC_Channel_index, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE,
               PAD_OUT_LOW);
    Pinmux_Config(ADC0 + ADC_Channel_index, IDLE_MODE);
}

/**
  * @brief  Initialize ADC peripheral.
  * @param   No parameter.
  * @return  void
  */
void Driver_ADC_init(void)
{
    ADC_DeInit(ADC);
    RCC_PeriphClockCmd(APBPeriph_ADC, APBPeriph_ADC_CLOCK, ENABLE);

    ADC_InitTypeDef adcInitStruct;
    ADC_StructInit(&adcInitStruct);
    adcInitStruct.schIndex[ADC_Schedule_index]         = EXT_SINGLE_ENDED(ADC_Channel_index);
    adcInitStruct.bitmap              = 0x01;
    adcInitStruct.adcFifoThd          = 10;
    adcInitStruct.adcSamplePeriod     = 255;
    ADC_Init(ADC, &adcInitStruct);
    ADC_INTConfig(ADC, ADC_INT_FIFO_TH, ENABLE);

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = ADC_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    ADC_Cmd(ADC, ADC_Continuous_Mode, ENABLE);
}
/**
* @brief  ADC interrupt handler function.
* @param   No parameter.
* @return  void
*/
void ADC_Handler(void)
{
    uint32_t data = 0;
    if (ADC_GetIntFlagStatus(ADC, ADC_INT_FIFO_TH) == SET)
    {
        uint8_t length = ADC_GetFifoLen(ADC);
        for (int i = 0; i < length; i++)
        {
            data = ADC_ReadFifoData(ADC);
            data = ADC_GetRes(data, EXT_SINGLE_ENDED(ADC_Channel_index));
//            DBG_DIRECT("ADC data = %d", data);
        }
        ADC_ClearINTPendingBit(ADC, ADC_INT_FIFO_TH);
    }
}
int main()
{
    WDG_Disable();
    __enable_irq();
    Board_ADC_init();
    Driver_ADC_init();
    while (1)
    {
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
    }
}
