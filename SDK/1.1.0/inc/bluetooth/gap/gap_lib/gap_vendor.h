/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file    gap_vendor.h
  * @brief
  * @details
  * @author  ranhui_xia
  * @date    2017-08-02
  * @version v1.0
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
  ******************************************************************************
  */
#ifndef GAP_VNR_H
#define GAP_VNR_H

#include <gap_le.h>

typedef enum
{
    GAP_SW_RESET_WHEN_ADV = 1,
} T_GAP_SW_RESET_MODE;

/** @brief Coding scheme of LE Coded PHY when device uses LE Advertising Extensions.*/
typedef enum
{
    GAP_AE_CODING_SCHEME_S8 = 2,
    GAP_AE_CODING_SCHEME_S2 = 3,
} T_GAP_AE_CODING_SCHEME;

/** @addtogroup GAP_LE_VENDOR GAP LE vendor command API.
  * @brief GAP LE vendor command API provides extended function for controller.
  * @{
  */

/*============================================================================*
 *                         Functions
 *============================================================================*/

/** @defgroup GAP_LE_VENDOR_Exported_Functions GAP LE vendor command Exported Functions
  * @brief GAP LE vendor command Exported Functions
  * @{
  */

/**
  * @brief  Enable 3 advertising channel advertising data.
  *         Set 3 adv data please refence to @ref le_vendor_adv_3_data_enable.
  *
  * @param[in] enable   0: disable, 1: enable each adv channel with diff data.
  *
  * @return Operation result.
  * @retval GAP_CAUSE_SUCCESS: Operation success.
  * @retval GAP_CAUSE_SEND_REQ_FAILED: Operation fail.
  *
  * <b>Example usage</b>
  * \code{.c}
    void gap_vendor_test(bool enable)
    {
        ...
        cause = le_vendor_adv_3_data_enable(enable);
        return cause;
    }
    T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data)
    {
        T_APP_RESULT result = APP_RESULT_SUCCESS;
        T_LE_CB_DATA cb_data;
        memcpy(&cb_data, p_cb_data, sizeof(T_LE_CB_DATA));
        APP_PRINT_TRACE1("app_gap_callback: cb_type = %d", cb_type);
        switch (cb_type)
        {
        ...
        case GAP_MSG_LE_VENDOR_ADV_3_DATA_ENABLE:
            APP_PRINT_INFO1("GAP_MSG_LE_VENDOR_ADV_3_DATA_ENABLE: cause 0x%x",
                            p_data->le_cause.cause);
            break;
        }
        ...
    }
  * \endcode
  */
T_GAP_CAUSE le_vendor_adv_3_data_enable(bool enable);

/**
  * @brief  Set different advertising date or scan response data in 3 advertising channel.
  *         This command is used to set 38 / 39 channel data, set 37 channel data please use normal HCI command.
  *         It is necessary to enable 3 adv data with @ref le_vendor_adv_3_data_enable.
  *
  * @param[in] type   LE  vendor advertising data type @ref T_GAP_ADV_VENDOR_DATA_TYPE.
  * @param[in] len    The number of significant octets in the advertising data.
  * @param[in] p_data Pointer to data to write
  *
  * @return Operation result.
  * @retval GAP_CAUSE_SUCCESS: Operation success.
  * @retval GAP_CAUSE_SEND_REQ_FAILED: Operation fail.
  * @retval GAP_CAUSE_INVALID_PARAM: Invalid parameter.
  *
  * <b>Example usage</b>
  * \code{.c}
    T_GAP_CAUSE gap_vendor_test(T_USER_CMD_PARSED_VALUE *p_parse_value)
    {
        T_GAP_CAUSE cause;
        T_GAP_ADV_VENDOR_DATA_TYPE type = (T_GAP_ADV_VENDOR_DATA_TYPE)p_parse_value->dw_param[0];
        uint8_t len = p_parse_value->dw_param[1];
        uint8_t value = p_parse_value->dw_param[2];
        uint8_t adv_data[31];
        if (len > GAP_MAX_ADV_LEN)
        {
            return (RESULT_ERR);
        }
        memset(adv_data, value, len);

        cause = le_vendor_adv_3_data_set((T_GAP_ADV_VENDOR_DATA_TYPE)type, len, adv_data);
        return (T_GAP_CAUSE)cause;
    }
    T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data)
    {
        T_APP_RESULT result = APP_RESULT_SUCCESS;
        T_LE_CB_DATA cb_data;
        memcpy(&cb_data, p_cb_data, sizeof(T_LE_CB_DATA));
        APP_PRINT_TRACE1("app_gap_callback: cb_type = %d", cb_type);
        switch (cb_type)
        {
        ...
        case GAP_MSG_LE_VENDOR_ADV_3_DATA_SET:
            APP_PRINT_INFO2("GAP_MSG_LE_VENDOR_ADV_3_DATA_SET: type %d, cause 0x%x",
                            p_data->p_le_vendor_adv_3_data_set_rsp->type,
                            p_data->p_le_vendor_adv_3_data_set_rsp->cause);
            break;
        ...
    }
  * \endcode
  */
T_GAP_CAUSE le_vendor_adv_3_data_set(T_GAP_ADV_VENDOR_DATA_TYPE type,
                                     uint8_t len, uint8_t *p_data);

/**
  * @brief  LE Drop Acl Data
  *
  * Drop pending LE acl packet that assigned by user except the acl packet which is currently waiting for ack.
  * The packet-dropping rule is cleared if link is disconnected or HCI_RESET.
  *
  * @param[in] conn_id  Connection ID for this link.
  * @param[in] mask     Assign the mask to compare with data.
  * @param[in] pattern  Drop data that match pattern
  * @param[in] offset   The offset in bytes started from Data
  * @return Operation result.
  * @retval GAP_CAUSE_SUCCESS: Operation success.
  * @retval GAP_CAUSE_SEND_REQ_FAILED: Operation fail.
  *
  * <b>Example usage</b>
  * \code{.c}
    static T_USER_CMD_PARSE_RESULT cmd_vdropdata(T_USER_CMD_PARSED_VALUE *p_parse_value)
    {
        T_GAP_CAUSE cause;
        uint8_t conn_id = p_parse_value->dw_param[0];

        cause = le_vendor_drop_acl_data(conn_id, 0xffff, 0x0015, 5);
        return (T_USER_CMD_PARSE_RESULT)cause;
    }
    T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data)
    {
        T_APP_RESULT result = APP_RESULT_SUCCESS;
        T_LE_CB_DATA cb_data;
        memcpy(&cb_data, p_cb_data, sizeof(T_LE_CB_DATA));
        APP_PRINT_TRACE1("app_gap_callback: cb_type = %d", cb_type);
        switch (cb_type)
        {
        ...
        case GAP_MSG_LE_VENDOR_DROP_ACL_DATA:
            APP_PRINT_INFO1("GAP_MSG_LE_VENDOR_DROP_ACL_DATA: cause 0x%x",
                            p_data->le_cause.cause);
            break;
        ...
    }
  * \endcode
  */
T_GAP_CAUSE le_vendor_drop_acl_data(uint8_t conn_id, uint16_t mask, uint16_t pattern,
                                    uint8_t offset);

/**
  * @brief  Modify BT LE Fw Policy
  *
  * Modify the firmware bt le policy
  *
  * @param[in] mask     Assign the mask to compare with data.
  * @param[in] mask     Assign the mask to compare with data.
  * @return Operation result.
  * @retval GAP_CAUSE_SUCCESS: Operation success.
  * @retval GAP_CAUSE_SEND_REQ_FAILED: Operation fail.
  */
T_GAP_CAUSE le_vendor_modify_bt_le_fw_policy(uint32_t mask, uint32_t value);

/**
  * @brief  Reset BT controller
  *
  * @param[in] reset_mode  GAP software reset mode @ref T_GAP_SW_RESET_MODE.
  * @return Operation result.
  * @retval GAP_CAUSE_SUCCESS: Operation success.
  * @retval GAP_CAUSE_SEND_REQ_FAILED: Operation fail.
  *
  * <b>Example usage</b>
  * \code{.c}
    static T_USER_CMD_PARSE_RESULT cmd_reset(T_USER_CMD_PARSED_VALUE *p_parse_value)
    {
        T_GAP_CAUSE cause;
        T_GAP_SW_RESET_MODE reset_mode = (T_GAP_SW_RESET_MODE)p_parse_value->dw_param[0];
        cause = gap_sw_reset_req(reset_mode);
        return (T_USER_CMD_PARSE_RESULT)cause;
    }
    T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data)
    {
        T_APP_RESULT result = APP_RESULT_SUCCESS;
        T_LE_CB_DATA cb_data;
        memcpy(&cb_data, p_cb_data, sizeof(T_LE_CB_DATA));
        APP_PRINT_TRACE1("app_gap_callback: cb_type = %d", cb_type);
        switch (cb_type)
        {
        ...
        case GAP_MSG_GAP_SW_RESET:
            APP_PRINT_INFO1("GAP_MSG_GAP_SW_RESET: cause 0x%x",
                            p_data->le_cause.cause);
            break;
        ...
    }
  * \endcode
  */
T_GAP_CAUSE gap_sw_reset_req(T_GAP_SW_RESET_MODE reset_mode);

/**
  * @brief  Set platform bootup active time.
  *
  * Platform allowed to enter low power mode after this timeout value.
  * Can use this command in any time even low power mode is set.
  *
  * @param[in] active_time  Unit: 50msec, Range:0x03-0x800. Default value is 5 sec.
  * @return Operation result.
  * @retval true Operation success.
  * @retval false Operation fail.
  *
  * <b>Example usage</b>
  * \code{.c}
    static T_USER_CMD_PARSE_RESULT cmd_lpstime(T_USER_CMD_PARSED_VALUE *p_parse_value)
    {
        uint16_t active_time = p_parse_value->dw_param[0];

        if(gap_set_lps_bootup_active_time(active_time))
        {
            return RESULT_SUCESS;
        }
        else
        {
            return RESULT_GAP_CAUSE_INVALID_PARAM;
        }
    }
    T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data)
    {
        T_APP_RESULT result = APP_RESULT_SUCCESS;
        T_LE_CB_DATA cb_data;
        memcpy(&cb_data, p_cb_data, sizeof(T_LE_CB_DATA));
        APP_PRINT_TRACE1("app_gap_callback: cb_type = %d", cb_type);
        switch (cb_type)
        {
        ...
        case GAP_MSG_GAP_SET_LPS_BOOTUP_ACTIVE_TIME:
            APP_PRINT_INFO1("GAP_MSG_GAP_SET_LPS_BOOTUP_ACTIVE_TIME: cause 0x%x",
                            p_data->le_cause.cause);
            break;
        ...
    }
  * \endcode
  */
bool gap_set_lps_bootup_active_time(uint16_t active_time);

/**
  * @brief  Configure coding scheme of LE Coded PHY when device uses LE Advertising Extensions.
  *
  *         NOTE: Advertiser should delete advertising set before changing coding scheme.
  *
  * @param[in] coding_scheme  Coding scheme of LE Coded PHY when when device uses LE Advertising Extensions, @ref T_GAP_AE_CODING_SCHEME.
  * @return Operation result.
  * @retval GAP_CAUSE_SUCCESS Send request success.
  * @retval other             Send request failed.
  *
  * <b>Example usage</b>
  * \code{.c}
    void test(void)
    {
        le_ae_coding_scheme(GAP_AE_CODING_SCHEME_S8);
    }
    T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data)
    {
        T_APP_RESULT result = APP_RESULT_SUCCESS;
        T_LE_CB_DATA cb_data;
        memcpy(&cb_data, p_cb_data, sizeof(T_LE_CB_DATA));
        APP_PRINT_TRACE1("app_gap_callback: cb_type = %d", cb_type);
        switch (cb_type)
        {
        ...
        case GAP_MSG_LE_AE_CODING_SCHEME:
            APP_PRINT_INFO1("GAP_MSG_LE_AE_CODING_SCHEME: cause 0x%x",
                            cb_data->le_cause.cause);
        break;
        ...
    }
  * \endcode
  */
T_GAP_CAUSE le_ae_coding_scheme(T_GAP_AE_CODING_SCHEME coding_scheme);

/** End of GAP_LE_VENDOR_Exported_Functions
  * @}
  */

/** End of GAP_LE_VENDOR
  * @}
  */
#endif /* GAP_VNR_H */
