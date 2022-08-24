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
#ifndef _KEYBOARD_BUTTON__
#define _KEYBOARD_BUTTON__

#ifdef __cplusplus
extern "C" {
#endif

#include <rtl876x.h>
#include <app_task.h>

typedef enum
{
    KEYBOARD_PAIR_PRESS,
    KEYBOARD_PAIR_RELEASE
} KEYBOARD_BUTTON_MSGTYPE;

extern bool start_pair_adv;

void gpio_button_pinmux_config(void);
void gpio_button_pad_config(void);
void gpio_button_enter_dlps_config(void);
void gpio_button_exit_dlps_config(void);
void keyboard_button_init(void);
void longpress_timer_init(void);
void handle_pair_button_event(T_IO_MSG button_msg);
void handle_pair_button_wakeup(void);

#ifdef __cplusplus
}
#endif

#endif

