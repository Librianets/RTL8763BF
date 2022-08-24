#include "rtl876x_tim.h"
#include "rtl876x_rcc.h"
#include "rtl876x_nvic.h"
#include "app_msg.h"
#include "app_task.h"
#include "trace.h"

#define TIMER_NUM               TIM6
#define TIMER_IRQN              TIMER6_IRQ
#define WDG_Timer_Handler       Timer6_Handler
#define TIMER_PERIOD            (2*1000*1000*40-1)  //2s for 40M clock

void driver_timer_init(void)
{
    TIM_DeInit();
    RCC_PeriphClockCmd(APBPeriph_TIMER, APBPeriph_TIMER_CLOCK, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    TIM_StructInit(&TIM_InitStruct);

    TIM_InitStruct.TIM_PWM_En = PWM_DISABLE;
    TIM_InitStruct.TIM_Period = TIMER_PERIOD ;
    TIM_InitStruct.TIM_Mode = TIM_Mode_UserDefine;
    TIM_TimeBaseInit(TIMER_NUM, &TIM_InitStruct);

    /*  Enable TIMER IRQ  */
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIMER_IRQN;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_ClearINT(TIMER_NUM);
    TIM_INTConfig(TIMER_NUM, ENABLE);
    TIM_Cmd(TIMER_NUM, ENABLE);
}

void reset_watch_dog_timer_enable(void)
{
    driver_timer_init();
}

void WDG_Timer_Handler(void)
{
    TIM_ClearINT(TIMER_NUM);
    TIM_Cmd(TIMER_NUM, DISABLE);
    //send message to app task in which reset the watch dog timer
    T_IO_MSG bee_io_msg = {0};
    bee_io_msg.type = IO_MSG_TYPE_RESET_WDG_TIMER;
    if (false == app_send_msg_to_apptask(&bee_io_msg))
    {
        APP_PRINT_ERROR0("[WDG] send IO_MSG_TYPE_RESET_WDG_TIMER message failed!");
    }

    TIM_Cmd(TIMER_NUM, ENABLE);
}
