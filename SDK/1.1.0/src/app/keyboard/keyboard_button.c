/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     keyboardbutton.c
* @brief
* @details
* @author   Elliot Chen
* @date     2015-7-20
* @version  v1.0
*********************************************************************************************************
*/
#include <stddef.h>
#include <board.h>
#include <trace.h>
#include <gap_adv.h>
#include <gap_bond_le.h>
#include <gap_conn_le.h>
#include <os_timer.h>
#include <keyboard_led.h>
#include <keyboard_button.h>
#include <keyboard_app.h>
#include <rtl876x_pinmux.h>
#include <rtl876x_rcc.h>
#include <rtl876x_gpio.h>
#include <rtl876x_nvic.h>
#include <privacy_mgnt.h>
#include <profile_init.h>

void *longpress_timer;
bool start_pair_adv = false;
bool pair_press = false;

/**
* @brief   gpio button pinmux config
* @return  void
*/
void gpio_button_pinmux_config(void)
{
    Pinmux_Config(PAIR_BUTTON, DWGPIO);
}

/**
* @brief   gpio button pad config
* @return  void
*/
void gpio_button_pad_config(void)
{
    Pad_Config(PAIR_BUTTON, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    System_WakeUpPinEnable(PAIR_BUTTON, PAD_WAKEUP_POL_LOW, PAD_WK_DEBOUNCE_ENABLE);
}

/**
* @brief   gpio button pad enter dlps config
* @return  void
*/
void gpio_button_enter_dlps_config(void)
{
    Pad_Config(PAIR_BUTTON, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
}

/**
* @brief   gpio button pad exit dlps config
* @return  void
*/
void gpio_button_exit_dlps_config(void)
{
    Pad_Config(PAIR_BUTTON, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
}

/**
* @brief  keyboard button initialization function.
* @param   GPIO_INT_PARAM.
* @return  void
*/
void keyboard_button_init(void)
{
    /* Enable GPIO and hardware timer's clock */
    RCC_PeriphClockCmd(APBPeriph_GPIO,  APBPeriph_GPIO_CLOCK,  ENABLE);

    /* Initialize GPIO as interrupt mode */
    GPIO_InitTypeDef GPIO_Param;
    GPIO_StructInit(&GPIO_Param);
    GPIO_Param.GPIO_Pin = GPIO_GetPin(PAIR_BUTTON);
    GPIO_Param.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Param.GPIO_ITCmd = ENABLE;
    GPIO_Param.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
    GPIO_Param.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
    GPIO_Param.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
    GPIO_Param.GPIO_DebounceTime = 20;
    GPIO_Init(&GPIO_Param);

    /* Enable interrupt */
    GPIO_INTConfig(GPIO_GetPin(PAIR_BUTTON), ENABLE);
    GPIO_MaskINTConfig(GPIO_GetPin(PAIR_BUTTON), DISABLE);

    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = GPIO20_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

/**
* @brief   longpress timeout callback
* @return  void
*/
void longpress_timeout(void *pxTimer)
{
    APP_PRINT_INFO0("Pair button longpress !!!");

    //if linkkey exist, clear key and start adv in the app gap callback
    if (le_get_bond_dev_num() != 0)
    {
        //when adv is enabled & resolution is enabled, could not use stop resolution cmd
        if (app_privacy_resolution_state == PRIVACY_ADDR_RESOLUTION_ENABLED && \
            gap_dev_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING)
        {
            le_adv_stop();
            start_pair_adv = true;
        }
        else if (app_privacy_resolution_state == PRIVACY_ADDR_RESOLUTION_ENABLED)
        {
            privacy_set_addr_resolution(false);
            le_bond_clear_all_keys();//maybe move this to privacy_handle_le_privacy_resolution_status_info
        }
        else if (app_privacy_resolution_state == PRIVACY_ADDR_RESOLUTION_DISABLED)
        {
            le_bond_clear_all_keys();
        }
    }
    else
    {
        if (gap_conn_state == GAP_CONN_STATE_DISCONNECTED)
        {
            if (gap_dev_state.gap_adv_state == GAP_ADV_STATE_IDLE)
            {
                keyboard_start_adv(ADV_UNDIRECT_PAIRING);
                start_adv_led_timer(false);
            }
            else if (gap_dev_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING)
            {
                reset_adv_count();
            }
        }
        else if (gap_conn_state == GAP_CONN_STATE_CONNECTED)
        {
            le_disconnect(keyboard_con_id);
            start_pair_adv = true;
        }
    }

}

/**
* @brief   longpress timer init
* @return  void
*/
void longpress_timer_init(void)
{
    os_timer_create(&longpress_timer, "long press timer", 1, 3000, false, longpress_timeout);
}

/**
* @brief   stop longpress timer
* @return  void
*/
void stop_longpress_timer(void)
{
    if (longpress_timer != NULL)
    {
        os_timer_stop(&longpress_timer);
    }
}

/**
* @brief   stop longpress timer
* @return  void
*/
void start_longpress_timer(void)
{
    if (longpress_timer != NULL)
    {
        os_timer_start(&longpress_timer);
    }
}

/**
* @brief   Pair button interrupt handler
* @return  void
*/
void GPIO20_Handler(void)
{
    /*  Mask GPIO interrupt */
    GPIO_INTConfig(GPIO_GetPin(PAIR_BUTTON), DISABLE);
    GPIO_MaskINTConfig(GPIO_GetPin(PAIR_BUTTON), ENABLE);
    GPIO_ClearINTPendingBit(GPIO_GetPin(PAIR_BUTTON));

    T_IO_MSG button_msg;
    button_msg.type = IO_MSG_TYPE_GPIO;

    if (GPIO_ReadInputDataBit(GPIO_GetPin(PAIR_BUTTON))) //Release
    {
        pair_press = false;
        button_msg.subtype = KEYBOARD_PAIR_RELEASE;
        GPIO->INTPOLARITY &= ~GPIO_GetPin(PAIR_BUTTON); //Polarity Low
        System_WakeUpPinEnable(PAIR_BUTTON, PAD_WAKEUP_POL_LOW, PAD_WK_DEBOUNCE_ENABLE);
        APP_PRINT_INFO0("Pair button release !!!");
    }
    else
    {
        pair_press = true;
        button_msg.subtype = KEYBOARD_PAIR_PRESS;
        GPIO->INTPOLARITY |= GPIO_GetPin(PAIR_BUTTON);   //Polarity High
        System_WakeUpPinEnable(PAIR_BUTTON, PAD_WAKEUP_POL_HIGH, PAD_WK_DEBOUNCE_ENABLE);
        APP_PRINT_INFO0("Pair button press !!!");
    }


    app_send_msg_to_apptask(&button_msg);

    GPIO_MaskINTConfig(GPIO_GetPin(PAIR_BUTTON), DISABLE);
    GPIO_INTConfig(GPIO_GetPin(PAIR_BUTTON), ENABLE);

}

/**
* @brief   Pair button event handler
* @return  void
*/
void handle_pair_button_event(T_IO_MSG button_msg)
{
    if (button_msg.subtype == KEYBOARD_PAIR_PRESS)
    {
        start_longpress_timer();
    }
    else if (button_msg.subtype == KEYBOARD_PAIR_RELEASE)
    {
        stop_longpress_timer();
    }
}

/**
* @brief   Pair button wakeup handler
* @return  void
*/
void handle_pair_button_wakeup(void)
{
    if (GPIO_ReadInputDataBit(GPIO_GetPin(PAIR_BUTTON)) && pair_press)
    {
        GPIO20_Handler();
    }
    else if (GPIO_ReadInputDataBit(GPIO_GetPin(PAIR_BUTTON)) == false && pair_press == false)
    {
        GPIO20_Handler();
    }
}
