/**
*****************************************************************************************
*     Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file     privacy_mgnt.h
  * @brief    privacy management module.
  * @details  privacy management module.
  * @author   jane
  * @date     2018-06-19
  * @version  v0.1
  * *************************************************************************************
  */

/* Define to prevent recursive inclusion */
#ifndef _PRIVACY_MGNT_H_
#define _PRIVACY_MGNT_H_

#ifdef __cplusplus
extern "C"  {
#endif      /* __cplusplus */

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include <gap_bond_le.h>
#include <gap_privacy.h>

/** @defgroup BLE_PRIV_MODULE BLE Privacy Management Module
  * @brief Application uses this module to handle privacy procedures.
  * @{
  */
/*============================================================================*
 *                         Types
 *============================================================================*/
/** @defgroup BLE_PRIV_MODULE_Exported_Types BLE Privacy Management Module Exported Types
  * @{
  */
/** @brief Privacy management module callback type*/
typedef enum
{
    PRIVACY_STATE_MSGTYPE,              //!< Privacy management module state.
    PRIVACY_RESOLUTION_STATUS_MSGTYPE   //!< Resolution state.
} T_PRIVACY_CB_TYPE;

/** @brief Privacy management module state*/
typedef enum
{
    PRIVACY_STATE_INIT,  //!< Privacy management module is not initialization.
    PRIVACY_STATE_IDLE,  //!< Idle. No pending resolving list modification procedure.
    PRIVACY_STATE_BUSY   //!< Busy. Resolving list modification procedure is not completed.
} T_PRIVACY_STATE;
/** End of BLE_PRIV_MODULE_Exported_Types
  * @}
  */
/*============================================================================*
 *                         Functions
 *============================================================================*/
/**
 * @defgroup BLE_PRIV_MODULE_EXPORT_Functions BLE Privacy Management Module Exported Functions
 *
 * @{
 */
/**
  * @brief Callback for privacy management module to notify app
  * @param[in] type    callback msy type @ref T_PRIVACY_CB_TYPE.
  * @param[in] status  callback data.
  * @retval void
  */
typedef void(*P_FUN_PRIVACY_STATE_CB)(T_PRIVACY_CB_TYPE type, uint8_t status);

/**
 * @brief  Initialize privacy management module.
 * @param[in] p_fun     Callback function provided by the APP to handle privacy messages sent from the privacy management module.
 * @param[in] whitelist Whether manage the white list when modify the resolving list.
 * @return none
 *
 * <b>Example usage</b>
 * \code{.c}
    void app_le_gap_init(void)
    {
        ......
        privacy_init(app_privacy_callback, true);
    }
    void app_privacy_callback(T_PRIVACY_CB_TYPE type, uint8_t status)
    {
        APP_PRINT_INFO2("app_privacy_callback: type %d, status %d", type, status);
        if (type == PRIVACY_STATE_MSGTYPE)
        {
            app_privacy_state = (T_PRIVACY_STATE)status;
        }
        else if (type == PRIVACY_RESOLUTION_STATUS_MSGTYPE)
        {
            app_privacy_resolution_state = (T_LE_PRIVACY_STATE)status;
        }
    }
 * \endcode
 */
void privacy_init(P_FUN_PRIVACY_STATE_CB p_fun, bool whitelist);

/**
 * @brief  Handle the pending resolving list modification procedure.
 *
 * Application shall call this funtion when the device state is in the idle state.
 *
 * @return The current privacy management module state.
 * @retval T_PRIVACY_STATE module state
 *
 * <b>Example usage</b>
 * \code{.c}
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
                app_adv_start();
            }
        }
        ......
    }
 * \endcode
 */
T_PRIVACY_STATE privacy_handle_resolv_list(void);

/**
 * @brief  Hande the GAP_MSG_LE_BOND_MODIFY_INFO message.
 *
 * Application shall call this funtion to handle the message GAP_MSG_LE_BOND_MODIFY_INFO.
 *
 * @param[in] type        Bond modification type @ref T_LE_BOND_MODIFY_TYPE.
 * @param[in] p_entry     The key entry of the modified device.
 * @param[in] handle_add  Whether handle the type @ref LE_BOND_ADD.
 * @return none
 *
 * <b>Example usage</b>
 * \code{.c}
    T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data)
    {
        T_APP_RESULT result = APP_RESULT_SUCCESS;
        T_LE_CB_DATA *p_data = (T_LE_CB_DATA *)p_cb_data;

        switch (cb_type)
        {
        ......
        case GAP_MSG_LE_BOND_MODIFY_INFO:
            APP_PRINT_INFO1("GAP_MSG_LE_BOND_MODIFY_INFO: type 0x%x",
                            p_data->p_le_bond_modify_info->type);
            privacy_handle_bond_modify_msg(p_data->p_le_bond_modify_info->type,
                                           p_data->p_le_bond_modify_info->p_entry, true);
            break;

        default:
            APP_PRINT_ERROR1("app_gap_callback: unhandled cb_type 0x%x", cb_type);
            break;
        }
        return result;
    }
 * \endcode
 */
void privacy_handle_bond_modify_msg(T_LE_BOND_MODIFY_TYPE type, T_LE_KEY_ENTRY *p_entry,
                                    bool handle_add);

/**
 * @brief  Add the device to the resolving list.
 *
 * Application can call this funcation when the parameter handle_add of the privacy_handle_bond_modify_msg is false.
 *
 * @param[in] p_entry The key entry of the device.
 * @return result
 * @retval true  Operation success.
 * @retval false Operation failure.
 *
 */
bool privacy_add_device(T_LE_KEY_ENTRY *p_entry);

/** @} */ /* End of group BLE_PRIV_MODULE_EXPORT_Functions */
/** @} */ /* End of group BLE_PRIV_MODULE */
#ifdef __cplusplus
}
#endif

#endif /* _PRIVACY_MGNT_H_ */
