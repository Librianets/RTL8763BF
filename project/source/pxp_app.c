/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      scatternet_app.c
   * @brief     This file handles BLE scatternet application routines.
   * @author    jane
   * @date      2017-06-06
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <trace.h>
#include <string.h>
#include <gap.h>
#include <gap_adv.h>
#include <gap_bond_le.h>
#include <profile_server.h>
#include <gap_msg.h>
#include <bas.h>
#include <app_msg.h>
#include <pxp_app.h>
#include <gap_conn_le.h>
#include <os_timer.h>
#include <ias.h>
#include <lls.h>
#include <tps.h>
#include <kns.h>
#include <bas.h>
#include <dis.h>
//#include <gatt_builtin_services.h>
#include "iohandle.h"
#include "rtl876x_wdg.h"
/** @addtogroup  PXP_DEMO
    * @{
    */

/** @defgroup  PXP Application
    * @brief This file handles BLE pxp smart application routines.
    * @{
    */
/*============================================================================*
 *                              Variables
 *============================================================================*/

PxpState gPxpState = PxpStateIdle;
IoState gIoState = IoStateIdle;
bool gPowerFlg = false;

T_SERVER_ID ias_srv_id;  /**< Immediately alert service id*/
T_SERVER_ID lls_srv_id;  /**< Link loss service id*/
T_SERVER_ID tps_srv_id;  /**< Tx power service id*/
T_SERVER_ID kns_srv_id;  /**< Key notification service id*/
T_SERVER_ID bas_srv_id;  /**< Battery service id */
T_SERVER_ID dis_srv_id;  /**< device infomation service id*/

#define LL_ASSERT_TIME gTimeParaValue//60
uint32_t gTimeParaValue = 30;
uint8_t g_pxp_immediate_alert_level;
uint8_t g_pxp_linkloss_alert_level = 1;

T_GAP_DEV_STATE gap_dev_state = {0, 0, 0, 0};                 /**< GAP device state */
T_GAP_CONN_STATE gap_conn_state = GAP_CONN_STATE_DISCONNECTED; /**< GAP connection state */

/*============================================================================*
 *                              Functions
 *============================================================================*/
void app_handle_gap_msg(T_IO_MSG  *p_gap_msg);
void Pxp_HandleButtonEvent(T_IO_MSG io_msg);
/**
 * @brief    All the application messages are pre-handled in this function
 * @note     All the IO MSGs are sent to this function, then the event handling
 *           function shall be called according to the MSG type.
 * @param[in] io_msg  IO message data
 * @return   void
 */
void app_handle_io_msg(T_IO_MSG io_msg)
{
    uint16_t msg_type = io_msg.type;

    switch (msg_type)
    {
    case IO_MSG_TYPE_BT_STATUS:
        {
            app_handle_gap_msg(&io_msg);
        }
        break;
    case IO_MSG_TYPE_GPIO:
        Pxp_HandleButtonEvent(io_msg);
        break;
    case IO_MSG_TYPE_RESET_WDG_TIMER:
        {
            APP_PRINT_INFO0("[WDG] Watch Dog Rset Timer");
            WDG_Restart();
        }
        break;
    default:
        break;
    }
}


/**
 * @brief    Handle msg GAP_MSG_LE_DEV_STATE_CHANGE
 * @note     All the gap device state events are pre-handled in this function.
 *           Then the event handling function shall be called according to the new_state
 * @param[in] new_state  New gap device state
 * @param[in] cause GAP device state change cause
 * @return   void
 */
void app_handle_dev_state_evt(T_GAP_DEV_STATE new_state, uint16_t cause)
{
    APP_PRINT_INFO3("app_handle_dev_state_evt: init state %d, adv state %d, cause 0x%x",
                    new_state.gap_init_state, new_state.gap_adv_state, cause);
    if (gap_dev_state.gap_init_state != new_state.gap_init_state)
    {
        if (new_state.gap_init_state == GAP_INIT_STATE_STACK_READY)
        {
            APP_PRINT_INFO0("GAP stack ready");
            gPowerFlg = false;
        }
    }

    if (gap_dev_state.gap_adv_state != new_state.gap_adv_state)
    {
        if (new_state.gap_adv_state == GAP_ADV_STATE_IDLE)
        {
            if (new_state.gap_adv_sub_state == GAP_ADV_TO_IDLE_CAUSE_CONN)
            {
                APP_PRINT_INFO0("GAP adv stoped: because connection created");
            }
            else
            {
                gPxpState = PxpStateIdle;
                APP_PRINT_INFO0("GAP adv stoped");
            }
        }
        else if (new_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING)
        {
            APP_PRINT_INFO0("GAP adv start");
            gPxpState = PxpStateAdv;
            if (gIoState != IoStateLlsAlert)
            {
                StartPxpIO(ALERT_LOW_PERIOD, ALERT_HIGH_PERIOD, LED_BLINK,
                           10); /*low period 0.9s, high period 0.1s,  led blink,  10times(cnt)*/
                gIoState = IoStateAdvBlink;
            }
        }
    }

    gap_dev_state = new_state;
}

/**
 * @brief    Handle msg GAP_MSG_LE_CONN_STATE_CHANGE
 * @note     All the gap conn state events are pre-handled in this function.
 *           Then the event handling function shall be called according to the new_state
 * @param[in] conn_id Connection ID
 * @param[in] new_state  New gap connection state
 * @param[in] disc_cause Use this cause when new_state is GAP_CONN_STATE_DISCONNECTED
 * @return   void
 */
void app_handle_conn_state_evt(uint8_t conn_id, T_GAP_CONN_STATE new_state, uint16_t disc_cause)
{
    APP_PRINT_INFO4("app_handle_conn_state_evt: conn_id %d old_state %d new_state %d, disc_cause 0x%x",
                    conn_id, gap_conn_state, new_state, disc_cause);
    switch (new_state)
    {
    case GAP_CONN_STATE_DISCONNECTED:
        {
            if ((disc_cause != (HCI_ERR | HCI_ERR_REMOTE_USER_TERMINATE))
                && (disc_cause != (HCI_ERR | HCI_ERR_LOCAL_HOST_TERMINATE)))
            {
                APP_PRINT_ERROR1("app_handle_conn_state_evt: connection lost cause 0x%x", disc_cause);
            }
            gPxpState = PxpStateIdle;
            if (gPowerFlg == true)
            {
                le_adv_start();
                APP_PRINT_ERROR1("g_pxp_linkloss_alert_level is %d", g_pxp_linkloss_alert_level);
                gIoState = IoStateLlsAlert;
                if (g_pxp_linkloss_alert_level == 1)
                {
                    StartPxpIO(ALERT_LOW_PERIOD, ALERT_HIGH_PERIOD, LED_BLINK, LL_ASSERT_TIME);
                }
                if (g_pxp_linkloss_alert_level == 2)
                {
                    StartPxpIO(ALERT_LOW_PERIOD, ALERT_HIGH_PERIOD, LED_BLINK | BEEP_ALERT, LL_ASSERT_TIME);
                }
                else
                {
                    //nothing to do
                }
            }
        }
        break;

    case GAP_CONN_STATE_CONNECTED:
        {
            uint16_t conn_interval;
            uint16_t conn_latency;
            uint16_t conn_supervision_timeout;
            uint8_t  remote_bd[6];
            T_GAP_REMOTE_ADDR_TYPE remote_bd_type;

            gPxpState = PxpStateLink;

            le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &conn_interval, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_LATENCY, &conn_latency, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &conn_supervision_timeout, conn_id);
            le_get_conn_addr(conn_id, remote_bd, &remote_bd_type);
            APP_PRINT_INFO5("GAP_CONN_STATE_CONNECTED:remote_bd %s, remote_addr_type %d, conn_interval 0x%x, conn_latency 0x%x, conn_supervision_timeout 0x%x",
                            TRACE_BDADDR(remote_bd), remote_bd_type,
                            conn_interval, conn_latency, conn_supervision_timeout);
        }
        break;

    default:
        break;
    }
    gap_conn_state = new_state;
}


/**
 * @brief    Handle msg GAP_MSG_LE_AUTHEN_STATE_CHANGE
 * @note     All the gap authentication state events are pre-handled in this function.
 *           Then the event handling function shall be called according to the new_state
 * @param[in] conn_id Connection ID
 * @param[in] new_state  New authentication state
 * @param[in] cause Use this cause when new_state is GAP_AUTHEN_STATE_COMPLETE
 * @return   void
 */
void app_handle_authen_state_evt(uint8_t conn_id, uint8_t new_state, uint16_t cause)
{
    APP_PRINT_INFO2("app_handle_authen_state_evt:conn_id %d, cause 0x%x", conn_id, cause);

    switch (new_state)
    {
    case GAP_AUTHEN_STATE_STARTED:
        {
            APP_PRINT_INFO0("app_handle_authen_state_evt: GAP_AUTHEN_STATE_STARTED");
        }
        break;

    case GAP_AUTHEN_STATE_COMPLETE:
        {
            if (cause == GAP_SUCCESS)
            {
                StartPxpIO(ALERT_LOW_PERIOD, ALERT_HIGH_PERIOD, LED_BLINK | BEEP_ALERT, 1);

                ChangeConnectionParameter(400, 0, 2000); //interval = 400*1.25ms
                APP_PRINT_INFO0("app_handle_authen_state_evt: GAP_AUTHEN_STATE_COMPLETE pair success");

            }
            else
            {
                APP_PRINT_INFO0("app_handle_authen_state_evt: GAP_AUTHEN_STATE_COMPLETE pair failed");
            }
        }
        break;

    default:
        {
            APP_PRINT_ERROR1("app_handle_authen_state_evt: unknown newstate %d", new_state);
        }
        break;
    }
}

/**
 * @brief    Handle msg GAP_MSG_LE_CONN_MTU_INFO
 * @note     This msg is used to inform APP that exchange mtu procedure is completed.
 * @param[in] conn_id Connection ID
 * @param[in] mtu_size  New mtu size
 * @return   void
 */
void app_handle_conn_mtu_info_evt(uint8_t conn_id, uint16_t mtu_size)
{
    APP_PRINT_INFO2("app_handle_conn_mtu_info_evt: conn_id %d, mtu_size %d", conn_id, mtu_size);
}


/**
 * @brief    Handle msg GAP_MSG_LE_CONN_PARAM_UPDATE
 * @note     All the connection parameter update change  events are pre-handled in this function.
 * @param[in] conn_id Connection ID
 * @param[in] status  New update state
 * @param[in] cause Use this cause when status is GAP_CONN_PARAM_UPDATE_STATUS_FAIL
 * @return   void
 */
void app_handle_conn_param_update_evt(uint8_t conn_id, uint8_t status, uint16_t cause)
{
    switch (status)
    {
    case GAP_CONN_PARAM_UPDATE_STATUS_SUCCESS:
        {
            uint16_t conn_interval;
            uint16_t conn_slave_latency;
            uint16_t conn_supervision_timeout;

            le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &conn_interval, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_LATENCY, &conn_slave_latency, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &conn_supervision_timeout, conn_id);
            APP_PRINT_INFO3("app_handle_conn_param_update_evt update success:conn_interval 0x%x, conn_slave_latency 0x%x, conn_supervision_timeout 0x%x",
                            conn_interval, conn_slave_latency, conn_supervision_timeout);
        }
        break;

    case GAP_CONN_PARAM_UPDATE_STATUS_FAIL:
        {
            APP_PRINT_ERROR1("app_handle_conn_param_update_evt update failed: cause 0x%x", cause);
        }
        break;

    case GAP_CONN_PARAM_UPDATE_STATUS_PENDING:
        {
            APP_PRINT_INFO0("app_handle_conn_param_update_evt update pending.");
        }
        break;

    default:
        break;
    }
}


/**
 * @brief    All the BT GAP MSG are pre-handled in this function.
 * @note     Then the event handling function shall be called according to the
 *           subtype of T_IO_MSG
 * @param[in] p_gap_msg Pointer to GAP msg
 * @return   void
 */
void app_handle_gap_msg(T_IO_MSG *p_gap_msg)
{
    T_LE_GAP_MSG gap_msg;
    uint8_t conn_id;
    memcpy(&gap_msg, &p_gap_msg->u.param, sizeof(p_gap_msg->u.param));

    APP_PRINT_TRACE1("app_handle_gap_msg: subtype %d", p_gap_msg->subtype);
    switch (p_gap_msg->subtype)
    {
    case GAP_MSG_LE_DEV_STATE_CHANGE:
        {
            app_handle_dev_state_evt(gap_msg.msg_data.gap_dev_state_change.new_state,
                                     gap_msg.msg_data.gap_dev_state_change.cause);
        }
        break;

    case GAP_MSG_LE_CONN_STATE_CHANGE:
        {
            app_handle_conn_state_evt(gap_msg.msg_data.gap_conn_state_change.conn_id,
                                      (T_GAP_CONN_STATE)gap_msg.msg_data.gap_conn_state_change.new_state,
                                      gap_msg.msg_data.gap_conn_state_change.disc_cause);
        }
        break;

    case GAP_MSG_LE_CONN_MTU_INFO:
        {
            app_handle_conn_mtu_info_evt(gap_msg.msg_data.gap_conn_mtu_info.conn_id,
                                         gap_msg.msg_data.gap_conn_mtu_info.mtu_size);
        }
        break;

    case GAP_MSG_LE_CONN_PARAM_UPDATE:
        {
            app_handle_conn_param_update_evt(gap_msg.msg_data.gap_conn_param_update.conn_id,
                                             gap_msg.msg_data.gap_conn_param_update.status,
                                             gap_msg.msg_data.gap_conn_param_update.cause);
        }
        break;

    case GAP_MSG_LE_AUTHEN_STATE_CHANGE:
        {
            app_handle_authen_state_evt(gap_msg.msg_data.gap_authen_state.conn_id,
                                        gap_msg.msg_data.gap_authen_state.new_state,
                                        gap_msg.msg_data.gap_authen_state.status);
        }
        break;

    case GAP_MSG_LE_BOND_JUST_WORK:
        {
            conn_id = gap_msg.msg_data.gap_bond_just_work_conf.conn_id;
            le_bond_just_work_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
            APP_PRINT_INFO0("GAP_MSG_LE_BOND_JUST_WORK");
        }
        break;

    case GAP_MSG_LE_BOND_PASSKEY_DISPLAY:
        {
            uint32_t display_value = 0;
            conn_id = gap_msg.msg_data.gap_bond_passkey_display.conn_id;
            le_bond_get_display_key(conn_id, &display_value);
            APP_PRINT_INFO1("GAP_MSG_LE_BOND_PASSKEY_DISPLAY:passkey %d", display_value);
            le_bond_passkey_display_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
        }
        break;

    case GAP_MSG_LE_BOND_USER_CONFIRMATION:
        {
            uint32_t display_value = 0;
            conn_id = gap_msg.msg_data.gap_bond_user_conf.conn_id;
            le_bond_get_display_key(conn_id, &display_value);
            APP_PRINT_INFO1("GAP_MSG_LE_BOND_USER_CONFIRMATION: passkey %d", display_value);
            le_bond_user_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
        }
        break;

    case GAP_MSG_LE_BOND_PASSKEY_INPUT:
        {
            uint32_t passkey = 888888;
            conn_id = gap_msg.msg_data.gap_bond_passkey_input.conn_id;
            APP_PRINT_INFO1("GAP_MSG_LE_BOND_PASSKEY_INPUT: conn_id %d", conn_id);
            le_bond_passkey_input_confirm(conn_id, passkey, GAP_CFM_CAUSE_ACCEPT);
        }
        break;

    case GAP_MSG_LE_BOND_OOB_INPUT:
        {
            uint8_t oob_data[GAP_OOB_LEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            conn_id = gap_msg.msg_data.gap_bond_oob_input.conn_id;
            APP_PRINT_INFO0("GAP_MSG_LE_BOND_OOB_INPUT");
            le_bond_set_param(GAP_PARAM_BOND_OOB_DATA, GAP_OOB_LEN, oob_data);
            le_bond_oob_input_confirm(conn_id, GAP_CFM_CAUSE_ACCEPT);
        }
        break;

    default:
        APP_PRINT_ERROR1("app_handle_gap_msg: unknown subtype %d", p_gap_msg->subtype);
        break;
    }
}
/** @} */ /* End of group PERIPH_GAP_MSG */



/** @defgroup  PXP_SMART_GAP_CALLBACK GAP Callback Event Handler
    * @brief Handle GAP callback event
    * @{
    */
/**
  * @brief Callback for gap le to notify app
  * @param[in] cb_type callback msy type @ref GAP_LE_MSG_Types.
  * @param[in] p_cb_data point to callback data @ref T_LE_CB_DATA.
  * @retval result @ref T_APP_RESULT
  */
T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data)
{
    T_APP_RESULT result = APP_RESULT_SUCCESS;
    T_LE_CB_DATA *p_data = (T_LE_CB_DATA *)p_cb_data;

    switch (cb_type)
    {
    case GAP_MSG_LE_DATA_LEN_CHANGE_INFO:
        APP_PRINT_INFO3("GAP_MSG_LE_DATA_LEN_CHANGE_INFO: conn_id %d, tx octets 0x%x, max_tx_time 0x%x",
                        p_data->p_le_data_len_change_info->conn_id,
                        p_data->p_le_data_len_change_info->max_tx_octets,
                        p_data->p_le_data_len_change_info->max_tx_time);
        break;

    case GAP_MSG_LE_MODIFY_WHITE_LIST:
        APP_PRINT_INFO2("GAP_MSG_LE_MODIFY_WHITE_LIST: operation %d, cause 0x%x",
                        p_data->p_le_modify_white_list_rsp->operation,
                        p_data->p_le_modify_white_list_rsp->cause);
        break;

    default:
        APP_PRINT_ERROR1("app_gap_callback: unhandled cb_type 0x%x", cb_type);
        break;
    }
    return result;
}
/** @} */ /* End of group PXP_GAP_CALLBACK */


/** @defgroup  PXP_SEVER_CALLBACK Profile Server Callback Event Handler
    * @brief Handle profile server callback event
    * @{
    */
/**
 * @brief    All the BT Profile service callback events are handled in this function
 * @note     Then the event handling function shall be called according to the
 *           service_id.
 * @param[in] service_id  Profile service ID
 * @param[in] p_data      Pointer to callback data
 * @return   Indicates the function call is successful or not
 * @retval   result @ref T_APP_RESULT
 */

T_APP_RESULT app_profile_callback(T_SERVER_ID service_id, void *p_data)
{
    T_APP_RESULT app_result = APP_RESULT_SUCCESS;
    if (service_id == SERVICE_PROFILE_GENERAL_ID)
    {
        T_SERVER_APP_CB_DATA *p_param = (T_SERVER_APP_CB_DATA *)p_data;
        switch (p_param->eventId)
        {
        case PROFILE_EVT_SRV_REG_COMPLETE:// srv register result event.
            APP_PRINT_INFO1("PROFILE_EVT_SRV_REG_COMPLETE: result %d",
                            p_param->event_data.service_reg_result);
            break;

        case PROFILE_EVT_SEND_DATA_COMPLETE:
            APP_PRINT_INFO5("PROFILE_EVT_SEND_DATA_COMPLETE: conn_id %d, cause 0x%x, service_id %d, attrib_idx 0x%x, credits %d",
                            p_param->event_data.send_data_result.conn_id,
                            p_param->event_data.send_data_result.cause,
                            p_param->event_data.send_data_result.service_id,
                            p_param->event_data.send_data_result.attrib_idx,
                            p_param->event_data.send_data_result.credits);
            if (p_param->event_data.send_data_result.cause == GAP_SUCCESS)
            {
                APP_PRINT_INFO0("PROFILE_EVT_SEND_DATA_COMPLETE success");
            }
            else
            {
                APP_PRINT_ERROR0("PROFILE_EVT_SEND_DATA_COMPLETE failed");
            }
            break;

        default:
            break;
        }
    }
    else if (service_id == ias_srv_id)
    {
        T_IAS_CALLBACK_DATA *p_ias_cb_data = (T_IAS_CALLBACK_DATA *)p_data;
        APP_PRINT_ERROR0("IAS CallBack.");
        if (p_ias_cb_data->msg_type == SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE)
        {
            g_pxp_immediate_alert_level = p_ias_cb_data->msg_data.write_alert_level;
            if (g_pxp_immediate_alert_level == 1)
            {
                gIoState = IoStateImmAlert;
                StopPxpIO();
                StartPxpIO(ALERT_LOW_PERIOD, ALERT_HIGH_PERIOD, LED_BLINK, gTimeParaValue);
            }
            else if (g_pxp_immediate_alert_level == 2)
            {
                gIoState = IoStateImmAlert;
                StartPxpIO(ALERT_LOW_PERIOD, ALERT_HIGH_PERIOD, LED_BLINK | BEEP_ALERT, gTimeParaValue);
            }
            else
            {
                StopPxpIO();
            }
        }
    }
    else if (service_id == lls_srv_id)
    {
        T_LLS_CALLBACK_DATA *p_lls_cb_data = (T_LLS_CALLBACK_DATA *)p_data;
        switch (p_lls_cb_data->msg_type)
        {
        case SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE:
            g_pxp_linkloss_alert_level = p_lls_cb_data->msg_data.write_alert_level;
            break;
        case SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE:
            lls_set_parameter(LLS_PARAM_LINK_LOSS_ALERT_LEVEL, 1, &g_pxp_linkloss_alert_level);
            break;
        default:
            break;
        }
    }
    else if (service_id == tps_srv_id)
    {
        T_TPS_CALLBACK_DATA *p_tps_cb_data = (T_TPS_CALLBACK_DATA *)p_data;
        if (p_tps_cb_data->msg_type == SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE)
        {
            if (p_tps_cb_data->msg_data.read_value_index == TPS_READ_TX_POWER_VALUE)
            {
                uint8_t tps_value = 0;
                tps_set_parameter(TPS_PARAM_TX_POWER, 1, &tps_value);
            }
        }
    }

    else  if (service_id == kns_srv_id)
    {
        T_KNS_CALLBACK_DATA *p_kns_cb_data = (T_KNS_CALLBACK_DATA *)p_data;
        switch (p_kns_cb_data->msg_type)
        {
        case SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION:
            {
                switch (p_kns_cb_data->msg_data.notification_indification_index)
                {
                case KNS_NOTIFY_ENABLE:
                    {
                        APP_PRINT_INFO0("KNS_NOTIFY_ENABLE");
                    }
                    break;

                case KNS_NOTIFY_DISABLE:
                    {
                        APP_PRINT_INFO0("KNS_NOTIFY_DISABLE");
                    }
                    break;
                default:
                    break;
                }
            }
            break;

        case SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE:
            {
                if (p_kns_cb_data->msg_data.read_index == KNS_READ_PARA)
                {
                    APP_PRINT_INFO0("KNS_READ_PARA");
                    kns_set_parameter(KNS_PARAM_VALUE, 4, &gTimeParaValue);
                }
            }
            break;
        case SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE:
            {
                APP_PRINT_INFO1("KNS_WRITE_PARA %x", p_kns_cb_data->msg_data.write_value);
                gTimeParaValue = p_kns_cb_data->msg_data.write_value;
            }
            break;

        default:
            break;
        }
    }
    else if (service_id == bas_srv_id)
    {
        T_BAS_CALLBACK_DATA *p_bas_cb_data = (T_BAS_CALLBACK_DATA *)p_data;
        switch (p_bas_cb_data->msg_type)
        {
        case SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION:
            {
                switch (p_bas_cb_data->msg_data.notification_indification_index)
                {
                case BAS_NOTIFY_BATTERY_LEVEL_ENABLE:
                    {
                        APP_PRINT_INFO0("BAS_NOTIFY_BATTERY_LEVEL_ENABLE");
                    }
                    break;

                case BAS_NOTIFY_BATTERY_LEVEL_DISABLE:
                    {
                        APP_PRINT_INFO0("BAS_NOTIFY_BATTERY_LEVEL_DISABLE");
                    }
                    break;
                default:
                    break;
                }
            }
            break;

        case SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE:
            {
                if (p_bas_cb_data->msg_data.read_value_index == BAS_READ_BATTERY_LEVEL)
                {
                    uint8_t battery_level = 90;
                    APP_PRINT_INFO1("BAS_READ_BATTERY_LEVEL: battery_level %d", battery_level);
                    bas_set_parameter(BAS_PARAM_BATTERY_LEVEL, 1, &battery_level);
                }
            }
            break;
        default:
            break;
        }
    }
    else if (service_id == dis_srv_id)
    {
        T_DIS_CALLBACK_DATA *p_dis_cb_data = (T_DIS_CALLBACK_DATA *)p_data;
        switch (p_dis_cb_data->msg_type)
        {
        case SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE:
            {
                if (p_dis_cb_data->msg_data.read_value_index == DIS_READ_MANU_NAME_INDEX)
                {
                    const uint8_t DISManufacturerName[] = "Realtek BT";
                    dis_set_parameter(DIS_PARAM_MANUFACTURER_NAME,
                                      sizeof(DISManufacturerName),
                                      (void *)DISManufacturerName);

                }
                else if (p_dis_cb_data->msg_data.read_value_index == DIS_READ_MODEL_NUM_INDEX)
                {
                    const uint8_t DISModelNumber[] = "Model Nbr 0.9";
                    dis_set_parameter(DIS_PARAM_MODEL_NUMBER,
                                      sizeof(DISModelNumber),
                                      (void *)DISModelNumber);
                }
                else if (p_dis_cb_data->msg_data.read_value_index == DIS_READ_SERIAL_NUM_INDEX)
                {
                    const uint8_t DISSerialNumber[] = "RTKBeeSerialNum";
                    dis_set_parameter(DIS_PARAM_SERIAL_NUMBER,
                                      sizeof(DISSerialNumber),
                                      (void *)DISSerialNumber);

                }
                else if (p_dis_cb_data->msg_data.read_value_index == DIS_READ_HARDWARE_REV_INDEX)
                {
                    const uint8_t DISHardwareRev[] = "RTKBeeHardwareRev";
                    dis_set_parameter(DIS_PARAM_HARDWARE_REVISION,
                                      sizeof(DISHardwareRev),
                                      (void *)DISHardwareRev);
                }
                else if (p_dis_cb_data->msg_data.read_value_index == DIS_READ_FIRMWARE_REV_INDEX)
                {
                    const uint8_t DISFirmwareRev[] = "RTKBeeFirmwareRev";
                    dis_set_parameter(DIS_PARAM_FIRMWARE_REVISION,
                                      sizeof(DISFirmwareRev),
                                      (void *)DISFirmwareRev);
                }
                else if (p_dis_cb_data->msg_data.read_value_index == DIS_READ_SOFTWARE_REV_INDEX)
                {
                    const uint8_t DISSoftwareRev[] = "RTKBeeSoftwareRev";
                    dis_set_parameter(DIS_PARAM_SOFTWARE_REVISION,
                                      sizeof(DISSoftwareRev),
                                      (void *)DISSoftwareRev);
                }
                else if (p_dis_cb_data->msg_data.read_value_index == DIS_READ_SYSTEM_ID_INDEX)
                {
                    const uint8_t DISSystemID[DIS_SYSTEM_ID_LENGTH] = {0, 1, 2, 0, 0, 3, 4, 5};
                    dis_set_parameter(DIS_PARAM_SYSTEM_ID,
                                      sizeof(DISSystemID),
                                      (void *)DISSystemID);

                }
                else if (p_dis_cb_data->msg_data.read_value_index == DIS_READ_IEEE_CERT_STR_INDEX)
                {
                    const uint8_t DISIEEEDataList[] = "RTKBeeIEEEDatalist";
                    dis_set_parameter(DIS_PARAM_IEEE_DATA_LIST,
                                      sizeof(DISIEEEDataList),
                                      (void *)DISIEEEDataList);
                }
                else if (p_dis_cb_data->msg_data.read_value_index == DIS_READ_PNP_ID_INDEX)
                {
                    uint8_t DISPnpID[DIS_PNP_ID_LENGTH] = {0};
                    dis_set_parameter(DIS_PARAM_PNP_ID,
                                      sizeof(DISPnpID),
                                      DISPnpID);
                }


            }
            break;
        default:
            break;
        }
    }

    return app_result;
}
/** @} */ /* End of group PXP_SERVER_CALLBACK */
/** @} */ /* End of group PXP_APP */
/** @} */ /* End of group PXP_DEMO */
