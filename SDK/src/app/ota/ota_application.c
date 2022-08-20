/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      rcu_application.c
* @brief     rcu application implementation
* @details   rcu application implementation
* @author    ken_mei
* @date      2017-07-03
* @version   v0.1
* *********************************************************************************************************
*/
#include <trace.h>
#include <string.h>
#include <gap.h>
#include <gap_adv.h>
#include <gap_bond_le.h>
#include <profile_server.h>
#include <gap_msg.h>

#include <app_msg.h>
#include <ota_application.h>
#include "bas.h"
#include "dis.h"
#include "gap_conn_le.h"
#include "ota_service.h"
#include "rtl876x_wdg.h"

#define DIS_SYSTEM_ID_LENGTH                8
#define DIS_PNP_ID_LENGTH                   7

bool gSwitchIntoOtaPending = false;
T_GAP_DEV_STATE gap_cur_state = {0, 0, 0, 0};
T_GAP_CONN_STATE gap_conn_state = GAP_CONN_STATE_DISCONNECTED;
//extern T_SERVER_ID simp_srv_id;
extern T_SERVER_ID gBASServiceId;
extern T_SERVER_ID gDISServiceId;
extern T_SERVER_ID gOTAServiceId;

uint16_t gBatVol;
uint16_t gBASBatteryLevel;

uint8_t gAdc_voice = 0;
bool IsMouseEnabled = false;
bool IsKeyboardEnabled = false;
bool gLinkKeyFlg = false;
uint8_t gDirAdvCnt = 0;
extern bool isPairAdvReady;
void periph_handle_gap_msg(T_IO_MSG  *p_gap_msg);
void SendToUart(uint32_t pBuf);
void Rmc_HandleGdmaEvent(T_IO_MSG io_driver_msg_recv);
void dfu_switch_to_ota_mode(void);
/******************************************************************
 * @fn          app_handle_io_msg
 * @brief      All the application events are pre-handled in this function.
 *                All the IO MSGs are sent to this function, Then the event handling function
 *                shall be called according to the MSG type.
 *
 * @param    io_driver_msg_recv  - bee io msg data
 * @return     void
 */
void app_handle_io_msg(T_IO_MSG io_driver_msg_recv)
{
    uint16_t msg_type = io_driver_msg_recv.type;

    switch (msg_type)
    {
    case IO_MSG_TYPE_BT_STATUS:
        {
            periph_handle_gap_msg(&io_driver_msg_recv);
        }
        break;
    case IO_MSG_TYPE_GDMA:
        {
        }
        break;
    default:
        break;
    }
}

/******************************************************************
 * @fn          peripheral_HandleBtDevStateChangeEvt
 * @brief      All the gaprole_States_t events are pre-handled in this function.
 *                Then the event handling function shall be called according to the newState.
 *
 * @param    newState  - new gap state
 * @return     void
 */
void periph_handle_dev_state_evt(T_GAP_DEV_STATE new_state, uint16_t cause)
{
    APP_PRINT_INFO4("periph_handle_dev_state_evt: init state %d, adv state %d, conn state %d, cause 0x%x",
                    new_state.gap_init_state, new_state.gap_adv_state,
                    new_state.gap_conn_state, cause);
    if (gap_cur_state.gap_init_state != new_state.gap_init_state)
    {
        if (new_state.gap_init_state == GAP_INIT_STATE_STACK_READY)
        {
            /*stack ready*/
            le_adv_start();
        }
    }

    if (gap_cur_state.gap_adv_state != new_state.gap_adv_state)
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
            }
        }
        else if (new_state.gap_adv_state == GAP_ADV_STATE_ADVERTISING)
        {
            APP_PRINT_INFO0("GAP adv start");
        }
    }

    if (gap_cur_state.gap_conn_state != new_state.gap_conn_state)
    {
        APP_PRINT_INFO2("conn state: %d -> %d",
                        gap_cur_state.gap_conn_state,
                        new_state.gap_conn_state);
    }
    gap_cur_state = new_state;
}

void periph_handle_conn_state_evt(uint8_t conn_id, T_GAP_CONN_STATE new_state, uint16_t disc_cause)
{
    APP_PRINT_INFO3("periph_handle_conn_state_evt: conn_id = %d old_state = %d new_state = %d",
                    conn_id, gap_conn_state, new_state);
    switch (new_state)
    {
    case GAP_CONN_STATE_DISCONNECTED:
        {
            if ((disc_cause != (HCI_ERR | HCI_ERR_REMOTE_USER_TERMINATE))
                && (disc_cause != (HCI_ERR | HCI_ERR_LOCAL_HOST_TERMINATE)))
            {
                APP_PRINT_ERROR1("connection lost: cause 0x%x", disc_cause);
            }
            APP_PRINT_INFO1("disc_cause is %d", disc_cause);
            if (gSwitchIntoOtaPending)
            {
                gSwitchIntoOtaPending = false;
                dfu_switch_to_ota_mode();
                WDG_SystemReset(RESET_ALL_EXCEPT_AON, DFU_SWITCH_TO_OTA);
            }
            else
            {
                le_adv_start();
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

/******************************************************************
 * @fn          peripheral_HandleBtGapAuthenStateChangeEvt
 * @brief      All the bonding state change  events are pre-handled in this function.
 *                Then the event handling function shall be called according to the newState.
 *
 * @param    newState  - new bonding state
 * @return     void
 */
void periph_handle_authen_state_evt(uint8_t conn_id, uint8_t new_state, uint16_t cause)
{
    APP_PRINT_INFO1("periph_handle_authen_state_evt:conn_id %d", conn_id);

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

/******************************************************************
 * @fn          peripheral_HandleBtGapConnParaChangeEvt
 * @brief      All the connection parameter update change  events are pre-handled in this function.
 *                Then the event handling function shall be called according to the status.
 *
 * @param    status  - connection parameter result, 0 - success, otherwise fail.
 * @return     void
 */
void periph_conn_param_update_evt(uint8_t conn_id, uint8_t status, uint16_t cause)
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

            APP_PRINT_INFO3("GAP_MSG_LE_CONN_PARAM_UPDATE update success, conn_interval = 0x%x, conn_slave_latency = 0x%x, conn_supervision_timeout = 0x%x",
                            conn_interval, conn_slave_latency, conn_supervision_timeout);
        }
        break;

    case GAP_CONN_PARAM_UPDATE_STATUS_FAIL:
        {
            APP_PRINT_ERROR1("GAP_MSG_LE_CONN_PARAM_UPDATE failed: cause 0x%x", cause);
        }
        break;

    case GAP_CONN_PARAM_UPDATE_STATUS_PENDING:
        {
            APP_PRINT_INFO0("GAP_MSG_LE_CONN_PARAM_UPDATE request success.");
        }
        break;

    default:
        break;
    }
}

/******************************************************************
 * @fn          peripheral_HandleBtGapMessage
 * @brief      All the bt gap msg  events are pre-handled in this function.
 *                Then the event handling function shall be called according to the subType
 *                of BEE_IO_MSG.
 *
 * @param    pBeeIoMsg  - pointer to bee io msg
 * @return     void
 */
void periph_handle_gap_msg(T_IO_MSG *p_gap_msg)
{
    T_LE_GAP_MSG gap_msg;
    uint8_t conn_id;
    memcpy(&gap_msg, &p_gap_msg->u.param, sizeof(p_gap_msg->u.param));

    APP_PRINT_TRACE1("periph_handle_gap_msg subtype = %d", p_gap_msg->subtype);
    switch (p_gap_msg->subtype)
    {
    case GAP_MSG_LE_DEV_STATE_CHANGE:
        {
            periph_handle_dev_state_evt(gap_msg.msg_data.gap_dev_state_change.new_state,
                                        gap_msg.msg_data.gap_dev_state_change.cause);
        }
        break;

    case GAP_MSG_LE_CONN_STATE_CHANGE:
        {
            periph_handle_conn_state_evt(gap_msg.msg_data.gap_conn_state_change.conn_id,
                                         (T_GAP_CONN_STATE)gap_msg.msg_data.gap_conn_state_change.new_state,
                                         gap_msg.msg_data.gap_conn_state_change.disc_cause);
        }
        break;

    case GAP_MSG_LE_AUTHEN_STATE_CHANGE:
        {
            periph_handle_authen_state_evt(gap_msg.msg_data.gap_authen_state.conn_id,
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

    case GAP_MSG_LE_CONN_PARAM_UPDATE:
        {
            periph_conn_param_update_evt(gap_msg.msg_data.gap_conn_param_update.conn_id,
                                         gap_msg.msg_data.gap_conn_param_update.status,
                                         gap_msg.msg_data.gap_conn_param_update.cause);
        }
        break;

    default:
        APP_PRINT_ERROR1("periph_handle_gap_msg: unknown subtype %d", p_gap_msg->subtype);
        break;
    }
}

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

    case GAP_MSG_LE_BOND_MODIFY_INFO:
        APP_PRINT_INFO1("GAP_MSG_LE_BOND_MODIFY_INFO: type 0x%x",
                        p_data->p_le_bond_modify_info->type);
        break;

    case GAP_MSG_LE_MODIFY_WHITE_LIST:
        APP_PRINT_INFO2("GAP_MSG_LE_MODIFY_WHITE_LIST: operation %d, cause 0x%x",
                        p_data->p_le_modify_white_list_rsp->operation,
                        p_data->p_le_modify_white_list_rsp->cause);
        break;

    default:
        APP_PRINT_INFO1("app_gap_callback: unhandled cb_type 0x%x", cb_type);
        break;
    }
    return result;
}

/******************************************************************
 * @fn          app_profile_callback
 * @brief      All the bt profile callbacks are handled in this function.
 *                Then the event handling function shall be called according to the serviceID
 *                of BEE_IO_MSG.
 *
 * @param    serviceID  -  service id of profile
 * @param    pData  - pointer to callback data
 * @return     void
 */
void StartMic(void);
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
            APP_PRINT_INFO5("PROFILE_EVT_SEND_DATA_COMPLETE: conn_id %d, cause 0x%x, service_id %d, attrib_idx 0x%x, credits = %d",
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
    else if (service_id == gBASServiceId)
    {
        T_BAS_CALLBACK_DATA *pBasCallbackData = (T_BAS_CALLBACK_DATA *)p_data;
        switch (pBasCallbackData->msg_type)
        {
        case SERVICE_CALLBACK_TYPE_INDIFICATION_NOTIFICATION:
            {
                if (pBasCallbackData->msg_data.notification_indification_index == BAS_NOTIFY_BATTERY_LEVEL_ENABLE)
                {
                    APP_PRINT_INFO0("Battery level notification enable");
                }
                else if (pBasCallbackData->msg_data.notification_indification_index ==
                         BAS_NOTIFY_BATTERY_LEVEL_DISABLE)
                {
                    APP_PRINT_INFO0("Battery level notification disable");
                }
            }
            break;
        case SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE:
            {
                /* update RCU battery level */
                gBatVol = 150;
                gBASBatteryLevel = 100;

                APP_PRINT_INFO2("RCU_Battery_Level_Update BatVol is %d , BASBatteryLevel is %d\n", gBatVol,
                                gBASBatteryLevel);

                bas_set_parameter(BAS_PARAM_BATTERY_LEVEL, 1, (uint8_t *)&gBASBatteryLevel);
            }
            break;
        default:
            break;
        }
    }
    else if (service_id == gDISServiceId)
    {
        T_DIS_CALLBACK_DATA *pDisCallbackData = (T_DIS_CALLBACK_DATA *)p_data;
        switch (pDisCallbackData->msg_type)
        {
        case SERVICE_CALLBACK_TYPE_READ_CHAR_VALUE:
            {
                if (pDisCallbackData->msg_data.read_value_index == DIS_READ_MANU_NAME_INDEX)
                {
                    const uint8_t DISManufacturerName[] = "Realtek BT";
                    dis_set_parameter(DIS_PARAM_MANUFACTURER_NAME,
                                      sizeof(DISManufacturerName),
                                      (void *)DISManufacturerName);
                }
                else if (pDisCallbackData->msg_data.read_value_index == DIS_READ_MODEL_NUM_INDEX)
                {
                    const uint8_t DISModelNumber[] = "Model Nbr 0.9";
                    dis_set_parameter(DIS_PARAM_MODEL_NUMBER,
                                      sizeof(DISModelNumber),
                                      (void *)DISModelNumber);
                }
                else if (pDisCallbackData->msg_data.read_value_index == DIS_READ_SERIAL_NUM_INDEX)
                {
                    const uint8_t DISSerialNumber[] = "RTKBeeSerialNum";
                    dis_set_parameter(DIS_PARAM_SERIAL_NUMBER,
                                      sizeof(DISSerialNumber),
                                      (void *)DISSerialNumber);
                }
                else if (pDisCallbackData->msg_data.read_value_index == DIS_READ_HARDWARE_REV_INDEX)
                {
                    const uint8_t DISHardwareRev[] = "RTKBeeHardwareRev";
                    dis_set_parameter(DIS_PARAM_HARDWARE_REVISION,
                                      sizeof(DISHardwareRev),
                                      (void *)DISHardwareRev);
                }
                else if (pDisCallbackData->msg_data.read_value_index == DIS_READ_FIRMWARE_REV_INDEX)
                {
                    const uint8_t DISFirmwareRev[] = "RTKBeeFirmwareRev";
                    dis_set_parameter(DIS_PARAM_FIRMWARE_REVISION,
                                      sizeof(DISFirmwareRev),
                                      (void *)DISFirmwareRev);
                }
                else if (pDisCallbackData->msg_data.read_value_index == DIS_READ_SOFTWARE_REV_INDEX)
                {
                    const uint8_t DISSoftwareRev[] = "RTKBeeSoftwareRev";
                    dis_set_parameter(DIS_PARAM_SOFTWARE_REVISION,
                                      sizeof(DISSoftwareRev),
                                      (void *)DISSoftwareRev);
                }
                else if (pDisCallbackData->msg_data.read_value_index == DIS_READ_SYSTEM_ID_INDEX)
                {
                    const uint8_t DISSystemID[DIS_SYSTEM_ID_LENGTH] = {0, 1, 2, 0, 0, 3, 4, 5};
                    dis_set_parameter(DIS_PARAM_SYSTEM_ID,
                                      sizeof(DISSystemID),
                                      (void *)DISSystemID);
                }
                else if (pDisCallbackData->msg_data.read_value_index == DIS_READ_IEEE_CERT_STR_INDEX)
                {
                    const uint8_t DISIEEEDataList[] = "RTKBeeIEEEDatalist";
                    dis_set_parameter(DIS_PARAM_IEEE_DATA_LIST,
                                      sizeof(DISIEEEDataList),
                                      (void *)DISIEEEDataList);
                }
                else if (pDisCallbackData->msg_data.read_value_index == DIS_READ_PNP_ID_INDEX)
                {
                    //uint8_t DISPnpID[DIS_PNP_ID_LENGTH] = {0x01,0xDA,0x0B,0x62,0x87,0x01,0x00};
                    uint16_t version = 0x03; //VERSION_BUILD;

                    uint8_t DISPnpID[DIS_PNP_ID_LENGTH] = {0x01, 0x5D, 0x00, 0x01, 0x00, (uint8_t)version, (uint8_t)(version >> 8)}; //VID_005D&PID_0001?

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
    else if (service_id == gOTAServiceId)
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
                gSwitchIntoOtaPending = true;
                le_disconnect(0);
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

