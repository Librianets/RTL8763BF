/**
*****************************************************************************************
*     Copyright(c) 2018, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      keyboard_app.c
   * @brief     This file handles BLE keyboard application routines.
   * @author    parker
   * @date      2018-04-18
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2018 Realtek Semiconductor Corporation</center></h2>
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
#include <gap_storage_le.h>
#include <profile_server.h>
#include <bas.h>
#include <gap_conn_le.h>
#include <keyboard_app.h>
#include <hids_kb.h>
#include <keyscan.h>
#include <key_handler.h>
#include <keyboard_button.h>
#include <keyboard_led.h>
#include <privacy_mgnt.h>
#include <profile_init.h>
#include "ota_service.h"
#include <dfu_api.h>
#include <rtl876x_wdg.h>

/** @defgroup  PERIPH_APP Peripheral Application
    * @brief This file handles BLE peripheral application routines.
    * @{
    */
/*============================================================================*
 *                              Variables
 *============================================================================*/
/** @addtogroup  PERIPH_SEVER_CALLBACK Profile Server Callback Event Handler
    * @brief Handle profile server callback event
    * @{
    */
T_SERVER_ID hid_srv_id; /**< HID service id*/
T_SERVER_ID bas_srv_id;  /**< Battery service id */
T_SERVER_ID dis_srv_id;  /**< Device Information service id */
T_SERVER_ID ota_srv_id;  /**< Device Information service id */
/** @} */ /* End of group PERIPH_SEVER_CALLBACK */
/** @defgroup  PERIPH_GAP_MSG GAP Message Handler
    * @brief Handle GAP Message
    * @{
    */
T_GAP_DEV_STATE gap_dev_state = {0, 0, 0, 0};                 /**< GAP device state */
T_GAP_CONN_STATE gap_conn_state = GAP_CONN_STATE_DISCONNECTED; /**< GAP connection state */
T_PRIVACY_STATE app_privacy_state = PRIVACY_STATE_INIT;
T_PRIVACY_ADDR_RESOLUTION_STATE app_privacy_resolution_state = PRIVACY_ADDR_RESOLUTION_DISABLED;

uint8_t keyboard_con_id;
bool switch_into_ota_pending = false;
bool report_cccd_enable = false;

/*============================================================================*
 *                              Functions
 *============================================================================*/
void app_handle_gap_msg(T_IO_MSG  *p_gap_msg);
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
        {
            handle_pair_button_event(io_msg);
        }
        break;
    case IO_MSG_TYPE_KEYSCAN:
        {
            if (io_msg.subtype == IO_MSG_KEYSCAN_RX_PKT)
            {
                T_KEYSCAN_FIFIO_DATA *p_key_data = io_msg.u.buf;
                APP_PRINT_INFO0("KeyScan MSG, Sub type Scan end.");
                if (passkey.enable_input)
                {
                    keyboard_handle_pair_event(p_key_data);
                }
                else if (report_cccd_enable)
                {
                    keyboard_handle_press_event(p_key_data);
                }
                else
                {
                    if (gap_conn_state == GAP_CONN_STATE_DISCONNECTED && \
                        gap_dev_state.gap_adv_state == GAP_ADV_STATE_IDLE)
                    {
                        APP_PRINT_INFO1("keyscan trigger adv, resolv_list_exist = %d", le_get_bond_dev_num());
                        if (le_get_bond_dev_num() != 0)
                        {
                            keyboard_start_adv(ADV_RECONECT);
                            start_adv_led_timer(true);
                        }
                    }
                }
            }
            else if (io_msg.subtype == IO_MSG_KEYSCAN_ALLKEYRELEASE)
            {
                APP_PRINT_INFO0("KeyScan MSG, Sub type All realse.");
                if (passkey.enable_input == false && report_cccd_enable)
                {
                    keyboard_handle_release_event();
                }
            }

        }
        break;
    default:
        break;
    }
}

/**
  * @brief Callback for BLE privacy management module to notify app
  * @param[in] type     callback msy type @ref T_PRIVACY_CB_TYPE.
  * @param[in] cb_data  callback data.
  * @retval void
  */
void app_privacy_callback(T_PRIVACY_CB_TYPE type, T_PRIVACY_CB_DATA cb_data)
{
    APP_PRINT_INFO1("app_privacy_callback: type %d", type);
    switch (type)
    {
    case PRIVACY_STATE_MSGTYPE:
        app_privacy_state = cb_data.privacy_state;
        APP_PRINT_INFO1("PRIVACY_STATE_MSGTYPE: status %d", app_privacy_state);
        break;

    case PRIVACY_RESOLUTION_STATUS_MSGTYPE:
        app_privacy_resolution_state = cb_data.resolution_state;
        APP_PRINT_INFO1("PRIVACY_RESOLUTION_STATUS_MSGTYPE: status %d", app_privacy_resolution_state);
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
    if ((new_state.gap_init_state == GAP_INIT_STATE_STACK_READY)
        && (new_state.gap_adv_state == GAP_ADV_STATE_IDLE)
        && (new_state.gap_conn_state == GAP_CONN_DEV_STATE_IDLE))
    {
        privacy_handle_resolv_list();
    }
    if (gap_dev_state.gap_init_state != new_state.gap_init_state)
    {
        if (new_state.gap_init_state == GAP_INIT_STATE_STACK_READY)
        {
            APP_PRINT_INFO0("GAP stack ready");
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
                APP_PRINT_INFO0("GAP adv stoped");
                if (start_pair_adv)
                {
                    start_pair_adv = false;
                    privacy_set_addr_resolution(false);
                    le_bond_clear_all_keys();//maybe move this to privacy_handle_le_privacy_resolution_status_info
                }
            }
        }
        else if (new_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING)
        {
            APP_PRINT_INFO0("GAP adv start");
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

            stop_con_update_timer();
            report_cccd_enable = false;
            passkey.enable_input = false;

            if (switch_into_ota_pending)
            {
                switch_into_ota_pending = false;
                dfu_switch_to_ota_mode();
                WDG_SystemReset(RESET_ALL_EXCEPT_AON, DFU_SWITCH_TO_OTA);
            }

            if (le_get_bond_dev_num() == 0)
            {
                start_pair_adv = true;
            }

            if (start_pair_adv)
            {
                APP_PRINT_INFO0("disconnect and start adv");
                start_pair_adv = false;
                keyboard_start_adv(ADV_UNDIRECT_PAIRING);
                start_adv_led_timer(false);
            }

        }
        break;

    case GAP_CONN_STATE_CONNECTED:
        {
            keyboard_con_id = conn_id;
            uint16_t conn_interval;
            uint16_t conn_latency;
            uint16_t conn_supervision_timeout;

            le_get_conn_param(GAP_PARAM_CONN_INTERVAL, &conn_interval, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_LATENCY, &conn_latency, conn_id);
            le_get_conn_param(GAP_PARAM_CONN_TIMEOUT, &conn_supervision_timeout, conn_id);
            APP_PRINT_INFO3("GAP_CONN_STATE_CONNECTED: conn_interval 0x%x, conn_latency 0x%x, conn_supervision_timeout 0x%x",
                            conn_interval, conn_latency, conn_supervision_timeout);

            stop_adv_led_timer();
            start_con_update_timer();
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
            if (((conn_slave_latency + 1) * conn_interval < 16) || (conn_slave_latency == 0))
            {
                APP_PRINT_INFO0("reupdate the connection parameter");
                start_con_update_timer();
            }
            else
            {
                APP_PRINT_INFO0("stop update the connection parameter");
                stop_con_update_timer();
            }
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
            conn_id = gap_msg.msg_data.gap_bond_passkey_input.conn_id;
            APP_PRINT_INFO1("GAP_MSG_LE_BOND_PASSKEY_INPUT: conn_id %d", conn_id);
            passkey.enable_input = true;
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

/** @defgroup  PERIPH_GAP_CALLBACK GAP Callback Event Handler
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
        if (p_data->p_le_modify_white_list_rsp->operation == GAP_WHITE_LIST_OP_CLEAR)
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
        break;

    case GAP_MSG_LE_BOND_MODIFY_INFO:
        APP_PRINT_INFO1("GAP_MSG_LE_BOND_MODIFY_INFO: type 0x%x",
                        p_data->p_le_bond_modify_info->type);
        privacy_handle_bond_modify_msg(p_data->p_le_bond_modify_info->type,
                                       p_data->p_le_bond_modify_info->p_entry, true);
    default:
        APP_PRINT_ERROR1("app_gap_callback: unhandled cb_type 0x%x", cb_type);
        break;
    }
    return result;
}
/** @} */ /* End of group PERIPH_GAP_CALLBACK */

/** @defgroup  PERIPH_SEVER_CALLBACK Profile Server Callback Event Handler
    * @brief Handle profile server callback event
    * @{
    */
/**
    * @brief    All the BT Profile service callback events are handled in this function
    * @note     Then the event handling function shall be called according to the
    *           service_id
    * @param    service_id  Profile service ID
    * @param    p_data      Pointer to callback data
    * @return   T_APP_RESULT, which indicates the function call is successful or not
    * @retval   APP_RESULT_SUCCESS  Function run successfully
    * @retval   others              Function run failed, and return number indicates the reason
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
    else  if (service_id == hid_srv_id)
    {
        T_HID_CALLBACK_DATA *p_hid_cb_data = (T_HID_CALLBACK_DATA *)p_data;

        switch (p_hid_cb_data->msg_type)
        {
        case SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION:
            {
                switch (p_hid_cb_data->msg_data.not_ind_data.index)
                {
                case GATT_SVC_HID_REPORT_INPUT_CCCD_INDEX:
                    {
                        if (p_hid_cb_data->msg_data.not_ind_data.value == NOTIFY_ENABLE)
                        {
                            report_cccd_enable = true;
                        }
                        else
                        {
                            report_cccd_enable = false;
                        }
                    }
                    break;
                case GATT_SVC_HID_BOOT_KB_IN_REPORT_CCCD_INDEX:
                    {

                    }
                    break;
                default:
                    break;
                }
            }
            break;
        case SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE:
            {
                switch (p_hid_cb_data->msg_data.read_value_index)
                {
                case GATT_SVC_HID_PROTOCOL_MODE_INDEX:
                    {
                        APP_PRINT_INFO1("HID_WRITE_PROTOCOL MODE %d\n",
                                        p_hid_cb_data->msg_data.write_msg.write_parameter.protocol_mode);
                        break;
                    }
                case GATT_SVC_HID_REPORT_OUTPUT_INDEX:
                    {
                        //callback data definition need to modify.
                        APP_PRINT_INFO1("HID_OUTPUT value %d\n", p_hid_cb_data->msg_data.write_msg.write_parameter.output);
                        if (p_hid_cb_data->msg_data.write_msg.write_parameter.output & 0x02)
                        {
                            capslock_led_on();
                        }
                        else
                        {
                            capslock_led_off();
                        }
                        break;
                    }
                default:
                    break;
                }
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
    else if (service_id == ota_srv_id)
    {

        TOTA_CALLBACK_DATA *pOTACallbackData = (TOTA_CALLBACK_DATA *)p_data;
        switch (pOTACallbackData->msg_type)
        {
        case SERVICE_CALLBACK_TYPE_WRITE_CHAR_VALUE:

            if (OTA_WRITE_CHAR_VAL == pOTACallbackData->msg_data.write.opcode &&
                OTA_VALUE_ENTER == pOTACallbackData->msg_data.write.u.value)
            {

                /*battery level is above 60 percent*/
                APP_PRINT_INFO0("Preparing switch into OTA mode\n");
                /*prepare to enter OTA mode, before switch action, we should disconnect first.*/
                switch_into_ota_pending = true;
                le_disconnect(keyboard_con_id);
            }
            else if (OTA_WRITE_IMAGE_COUNT_VAL == pOTACallbackData->msg_data.write.opcode)
            {

            }

            break;

        default:

            break;
        }
    }

    return app_result;
}

/** @} */ /* End of group PERIPH_SEVER_CALLBACK */
/** @} */ /* End of group PERIPH_APP */
