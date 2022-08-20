/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      keyboardbutton.h
* @brief
* @details
* @author    Elliot Chen
* @date      2015-7-20
* @version   v0.1
* *********************************************************************************************************
*/
#ifndef _KEYBOARD_LED__
#define _KEYBOARD_LED__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtl876x.h>

void adv_led_timer_init(void);
void start_adv_led_timer(bool reconnect);
void stop_adv_led_timer(void);
void reset_adv_count(void);
void keyboard_led_init(void);
void capslock_led_on(void);
void capslock_led_off(void);

#ifdef __cplusplus
}
#endif

#endif

