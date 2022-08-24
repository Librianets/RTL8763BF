/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     profile_init.c
* @brief    .
* @details
* @author   Chuanguo Xue
* @date     2015-03-27
* @version  v1.0
*********************************************************************************************************
*/
#include <rtl876x.h>
#include <trace.h>
#include <string.h>
#include <gap_adv.h>
#include <profile_init.h>
#include <gap_storage_le.h>
#include <privacy_mgnt.h>
#include <keyboard_app.h>
#include <os_timer.h>
#include <gap_conn_le.h>

void *con_update_timer = NULL;

/**
* @brief   keyboard start adv
* @return  void
*/
void keyboard_start_adv(T_ADV_TYPE adv_type)
{
    uint8_t  adv_evt_type = GAP_ADTYPE_ADV_IND;
    uint8_t  adv_direct_type = GAP_REMOTE_ADDR_LE_PUBLIC;
    uint8_t  adv_direct_addr[GAP_BD_ADDR_LEN] = {0};
    uint8_t  adv_filter_policy = GAP_ADV_FILTER_ANY;

    switch (adv_type)
    {
    case ADV_RECONECT:
        {
            T_LE_KEY_ENTRY *p_entry;
            p_entry = le_get_high_priority_bond();
            if ((p_entry->flags & LE_KEY_STORE_REMOTE_IRK_BIT) == 0)
            {
                if ((p_entry->remote_bd.remote_bd_type == GAP_REMOTE_ADDR_LE_PUBLIC) ||
                    ((p_entry->remote_bd.remote_bd_type == GAP_REMOTE_ADDR_LE_RANDOM) &&
                     ((p_entry->remote_bd.addr[5] & RANDOM_ADDR_MASK) == RANDOM_ADDR_MASK_STATIC))
                   )
                {
                    adv_evt_type = GAP_ADTYPE_ADV_HDC_DIRECT_IND;
                    memcpy(adv_direct_addr, p_entry->remote_bd.addr, GAP_BD_ADDR_LEN);
                    adv_direct_type = p_entry->remote_bd.remote_bd_type;
                }
            }
            else
            {
                if (app_privacy_resolution_state == PRIVACY_ADDR_RESOLUTION_DISABLED)
                {
                    privacy_set_addr_resolution(true);
                }
                adv_filter_policy = GAP_ADV_FILTER_WHITE_LIST_ALL;
            }
            break;
        }
    case ADV_UNDIRECT_PAIRING:
        {
            break;
        }
    default:
        break;
    }

    le_adv_set_param(GAP_PARAM_ADV_EVENT_TYPE, sizeof(adv_evt_type), &adv_evt_type);
    le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR_TYPE, sizeof(adv_direct_type), &adv_direct_type);
    le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR, sizeof(adv_direct_addr), adv_direct_addr);
    le_adv_set_param(GAP_PARAM_ADV_FILTER_POLICY, sizeof(adv_filter_policy), &adv_filter_policy);
    le_adv_start();
}


/**
* @brief   connection parameter update timeout callback
* @return  void
*/
void keyboard_update_con_para(void *pxTimer)
{
    if (gap_conn_state != GAP_CONN_STATE_CONNECTED)
    {
        return;
    }
    APP_PRINT_INFO0("connection parameter update callback!");
    uint16_t conn_interval;
    uint16_t conn_slave_latency;
    uint16_t conn_supervision_timeout;

    le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &conn_interval, keyboard_con_id);
    le_get_conn_param(GAP_PARAM_CONN_LATENCY, &conn_slave_latency, keyboard_con_id);
    le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &conn_supervision_timeout, keyboard_con_id);

    if (((conn_slave_latency + 1) * conn_interval < 16) || (conn_slave_latency == 0))
    {
        uint16_t desired_interval_min = 6;
        uint16_t desired_interval_max = 16;
        uint16_t desired_latency = 20;
        uint16_t desired_superv_tout = 3000;
        APP_PRINT_INFO0("connection parameter update request!");
        le_update_conn_param(keyboard_con_id, desired_interval_min, desired_interval_max, desired_latency,
                             desired_superv_tout / 10, desired_interval_min * 2 - 2, desired_interval_max * 2 - 2);
    }

}


/**
* @brief   create connection parameter update timer
* @return  void
*/
void connection_param_updata_timer_init(void)
{
    os_timer_create(&con_update_timer, "con param update timer", 1, 30000, false,
                    keyboard_update_con_para);
}


/**
* @brief   stop con param update timer
* @return  void
*/
void stop_con_update_timer(void)
{
    if (con_update_timer != NULL)
    {
        os_timer_stop(&con_update_timer);
    }
}

/**
* @brief   stop con param update timer
* @return  void
*/
void start_con_update_timer(void)
{
    if (con_update_timer != NULL)
    {
        os_timer_start(&con_update_timer);
    }
}

