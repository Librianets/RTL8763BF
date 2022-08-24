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
#include <board.h>
#include <trace.h>
#include <os_timer.h>
#include <gap_adv.h>
#include <keyboard_led.h>
#include <rtl876x_pinmux.h>
#include <rtl876x_rcc.h>
#include <rtl876x_gpio.h>


void     *adv_led_timer;
bool     adv_led_on = false;
bool     adv_reconnect = false;
uint8_t  adv_count;

/**
* @brief  adv led timeout
* @return  void
*/
void adv_led_timeout(void *pxTimer)
{
    if (adv_reconnect)
    {
        le_adv_stop();
        APP_PRINT_INFO0("ADV timeout, stop  reconnect ADV");
    }
    else
    {
        if (adv_count < 30)
        {
            if (adv_led_on)
            {
                adv_led_on = false;
                Pad_Config(ADV_LED, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
                os_timer_restart(&adv_led_timer, 900);
            }
            else
            {
                adv_count++;
                adv_led_on = true;
                Pad_Config(ADV_LED, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);
                os_timer_restart(&adv_led_timer, 100);
            }
        }
        else
        {
            adv_led_on = false;
            Pad_Config(ADV_LED, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
            le_adv_stop();
            APP_PRINT_INFO0("ADV timeout, stop pair ADV");
        }
    }
}

/**
* @brief  create adv led timer
* @return  void
*/
void adv_led_timer_init(void)
{
    os_timer_create(&adv_led_timer, "adv led timer", 1, 3000, false, adv_led_timeout);
}

/**
* @brief  start adv led timer
* @return  void
*/
void start_adv_led_timer(bool reconnect)
{
    adv_reconnect = reconnect;
    if (adv_reconnect)
    {
        os_timer_restart(&adv_led_timer, 30000);
    }
    else
    {
        adv_count = 0;
        adv_led_on = true;
        Pad_Config(ADV_LED, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);
        os_timer_restart(&adv_led_timer, 100);
    }
}

/**
* @brief  stop adv led timer
* @return  void
*/
void stop_adv_led_timer(void)
{
    adv_count = 0;
    adv_led_on = false;
    Pad_Config(ADV_LED, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
    os_timer_stop(&adv_led_timer);
}

/**
* @brief  reset adv count
* @return  void
*/
void reset_adv_count(void)
{
    adv_count = 0;
    adv_led_on = false;
    Pad_Config(ADV_LED, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
}

/**
* @brief  keyboard led initialization function.
* @return  void
*/
void keyboard_led_init(void)
{
    Pad_Config(ADV_LED, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(CAPS_LOCK_LED, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
}

/**
* @brief  turn on capslock led
* @return  void
*/
void capslock_led_on(void)
{
    Pad_Config(CAPS_LOCK_LED, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_ENABLE, PAD_OUT_HIGH);
}

/**
* @brief  turn off capslock led
* @return  void
*/
void capslock_led_off(void)
{
    Pad_Config(CAPS_LOCK_LED, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
}

