/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      profile_init.h
* @brief
* @details
* @author    Chuanguo Xue
* @date      2015-3-27
* @version   v0.1
* *********************************************************************************************************
*/
#ifndef _PROFILE_INIT_H
#define _PROFILE_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ADV_IDLE = 0,
    ADV_RECONECT,
    ADV_UNDIRECT_PAIRING,
} T_ADV_TYPE;


void keyboard_start_adv(T_ADV_TYPE adv_type);
void connection_param_updata_timer_init(void);
void stop_con_update_timer(void);
void start_con_update_timer(void);

#ifdef __cplusplus
}
#endif

#endif /* _PROFILE_INIT_H */

