/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      keyscan.h
* @brief
* @details
* @author    Chuanguo Xue
* @date      2015-3-27
* @version   v1.0
* *********************************************************************************************************
*/


#ifndef _KEYSCAN_H
#define _KEYSCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "rtl876x.h"


#define KEYSCAN_SW_INTERVAL 50  /* 50ms */
#define KEYSCAN_SW_RELEASE_TIMEOUT 10 /* 10ms */

/**
 * @brief  KeyScan FIFO data struct definition.
 */
typedef struct
{
    uint32_t len;               /**< Keyscan state register        */
    struct
    {
        uint8_t column: 5;      /**< Keyscan raw buffer data       */
        uint8_t row: 3;         /**< Keyscan raw buffer data       */
        uint8_t reserved;
    } key[8];
} T_KEYSCAN_FIFIO_DATA;


/**
 * @brief  KeyScan global data struct definition.
 */
typedef struct
{
    bool is_allowed_to_repeat_report;  /* to indicate whether to allow to report repeat keyscan data event or not */
    bool is_allowed_to_enter_dlps;  /* to indicate whether to allow to enter dlps or not */
    bool is_key_pressed;  /* to indicate whether any key is pressed or not */
    bool is_all_key_released;  /* to indicate whether all keys are released or not */
    T_KEYSCAN_FIFIO_DATA pre_fifo_data;  /* to indicate the previous keyscan FIFO data */
    T_KEYSCAN_FIFIO_DATA cur_fifo_data;  /* to indicate the current keyscan FIFO data */
} T_KEYSCAN_GLOBAL_DATA;

extern T_KEYSCAN_GLOBAL_DATA keyscan_global_data;
extern void *keyscan_timer;

void keyscan_pad_config(void);
void keyscan_pinmux_config(void);
void keyscan_enter_dlps_config(void);
void keyscan_exit_dlps_config(void);
void keyscan_init_driver(uint32_t is_debounce);
void keyscan_nvic_config(void);
void keyscan_init_data(void);
void keyscan_init_timer(void);
void keyscan_timer_callback(void *pxTimer);
void handle_keyscan_wakeup(void);

#ifdef __cplusplus
}
#endif

#endif /* _KEYSCAN_H */

