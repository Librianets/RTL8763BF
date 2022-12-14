/**
*********************************************************************************************************
*               Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file      gap_ext_adv.h
* @brief     Header file for Gap ext adv
* @details   This file defines extended advertising related API.
* @author    ranhui
* @date      2016-02-18
* @version   v1.0
* *********************************************************************************************************
*/

/*============================================================================*
 *               Define to prevent recursive inclusion
 *============================================================================*/
#ifndef GAP_EXT_ADV_H
#define GAP_EXT_ADV_H

#ifdef __cplusplus
extern "C"
{
#endif

/*============================================================================*
 *                        Header Files
 *============================================================================*/
#include "gap_le.h"

#if F_BT_LE_5_0_AE_ADV_SUPPORT

/** @addtogroup GAP GAP Module
  * @{
  */

/** @addtogroup GAP_LE GAP LE Module
  * @{
  */

/** @addtogroup GAP_LE_EXTENDED_ADV GAP LE Extended Adv Module
  * @{
  */

/*============================================================================*
 *                         Macros
 *============================================================================*/
/** @defgroup GAP_LE_EXTENDED_ADV_Exported_Macros GAP LE Extended Adv Exported Macros
  * @{
  */

/** @defgroup EXT_ADV_PARAM Extended Advertising Parameter flag
  * @brief Use the combination of macro definitions to set extended advertising related parameters
           for a specified advertising set by calling @ref le_ext_adv_start_setting.
  * @{
  */
#define EXT_ADV_SET_AUTO           0x00   /**< Automatically set extended advertising related parameters (including advertising parameters,
                                               advertising data and scan response data) according to advertising event properties. */
#define EXT_ADV_SET_ADV_PARAS      0x01   /**< Set advertising parameters supplied by @ref le_ext_adv_set_adv_param. */
#define EXT_ADV_SET_ADV_DATA       0x02   /**< Set advertising data supplied by @ref le_ext_adv_set_adv_data. */
#define EXT_ADV_SET_SCAN_RSP_DATA  0x04   /**< Set scan response data supplied by @ref le_ext_adv_set_scan_response_data. */
#define EXT_ADV_SET_RANDOM_ADDR    0x08   /**< Set random address supplied by @ref le_ext_adv_set_random. */
/** End of EXT_ADV_PARAM
  * @}
  */

/** @defgroup EXT_ADV_EVT_PROP Extended Advertising Event Properties flag
  * @brief Use the combination of macro definitions to describe the type of advertising event.
           Optional values: @ref T_LE_EXT_ADV_LEGACY_ADV_PROPERTY and @ref T_LE_EXT_ADV_EXTENDED_ADV_PROPERTY.
  * @{
  */
#define GAP_EXT_ADV_EVT_PROP_CONNECTABLE_ADV     0x01 /**< Connectable advertising. */
#define GAP_EXT_ADV_EVT_PROP_SCANNABLE_ADV       0x02 /**< Scannable advertising. */
#define GAP_EXT_ADV_EVT_PROP_DIRECTED_ADV        0x04 /**< Directed advertising. */
#define GAP_EXT_ADV_EVT_PROP_HDC_DIRECTED_ADV    0x08 /**< High Duty Cycle Directed Connectable advertising. */
#define GAP_EXT_ADV_EVT_PROP_USE_LEGACY_ADV      0x10 /**< Use legacy advertising PDUs. */
#define GAP_EXT_ADV_EVT_PROP_OMIT_ADV_ADDR       0x20 /**< Omit advertiser's address from all PDUs ("anonymous advertising"). */
#define GAP_EXT_ADV_EVT_PROP_INCLUDE_TX_POWER    0x40 /**< Include TxPower in the extended header of the advertising PDU. */
/** End of EXT_ADV_EVT_PROP
  * @}
  */

/** End of GAP_LE_EXTENDED_Exported_Macros
  * @}
  */

/*============================================================================*
 *                         Types
 *============================================================================*/
/** @defgroup GAP_LE_EXTENDED_ADV_Exported_Types GAP LE Extended Adv Exported Types
  * @{
  */

/** @brief GAP extended advertising state. */
typedef enum
{
    EXT_ADV_STATE_IDLE,         /**< Idle, no advertising. */
    EXT_ADV_STATE_START,        /**< Start Advertising. A temporary state, haven't received the result. */
    EXT_ADV_STATE_ADVERTISING,  /**< Advertising. */
    EXT_ADV_STATE_STOP,         /**< Stop Advertising. A temporary state, haven't received the result. */
} T_GAP_EXT_ADV_STATE;

/** @brief Advertising Event Properties values for legacy advertising PDUs. */
typedef enum
{
    LE_EXT_ADV_LEGACY_ADV_CONN_SCAN_UNDIRECTED           = 0x13, /**<  Connectable and scannable undirected. Advertising data or scan response data shall not exceed 31 bytes. */
    LE_EXT_ADV_LEGACY_ADV_CONN_LOW_DUTY_DIRECTED         = 0x15, /**<  Connectable directed (low duty cycle). */
    LE_EXT_ADV_LEGACY_ADV_CONN_HIGH_DUTY_DIRECTED        = 0x1D, /**<  Connectable directed (high duty cycle). */
    LE_EXT_ADV_LEGACY_ADV_SCAN_UNDIRECTED                = 0x12, /**<  Scannable undirected. Advertising data or scan response data shall not exceed 31 bytes. */
    LE_EXT_ADV_LEGACY_ADV_NON_SCAN_NON_CONN_UNDIRECTED   = 0x10, /**<  Non-connectable and non-scannable undirected. Advertising data shall not exceed 31 bytes. */
} T_LE_EXT_ADV_LEGACY_ADV_PROPERTY;

/** @brief Advertising Event Properties values for extended advertising PDUs. */
typedef enum
{
    LE_EXT_ADV_EXTENDED_ADV_NON_SCAN_NON_CONN_UNDIRECTED = 0x00, /**<  Non-connectable and non-scannable undirected. If only one advertising set is used, advertising data shall not exceed 1024 bytes. */
    LE_EXT_ADV_EXTENDED_ADV_NON_SCAN_NON_CONN_DIRECTED   = 0x04, /**<  Non-connectable and non-scannable directed. If only one advertising set is used, advertising data shall not exceed 1024 bytes. */
    LE_EXT_ADV_EXTENDED_ADV_CONN_UNDIRECTED              = 0x01, /**<  Connectable undirected. Advertising data shall not exceed 245 bytes. */
    LE_EXT_ADV_EXTENDED_ADV_CONN_DIRECTED                = 0x05, /**<  Connectable directed. Advertising data shall not exceed 239 bytes. */
    LE_EXT_ADV_EXTENDED_ADV_SCAN_UNDIRECTED              = 0x02, /**<  Scannable undirected. If only one advertising set is used, scan response data shall not exceed 991 bytes. */
    LE_EXT_ADV_EXTENDED_ADV_SCAN_DIRECTED                = 0x06, /**<  Scannable directed. If only one advertising set is used, scan response data shall not exceed 991 bytes. */
} T_LE_EXT_ADV_EXTENDED_ADV_PROPERTY;

/** @brief Supported advertising parameters type. */
typedef enum
{
    GAP_PARAM_EXT_ADV_MAX_DATA_LEN   = 0x330, /**<  Maximum length of supported data for use as advertisement data or scan
                                                    response data. Read only. Size is 2 bytes. */
    GAP_PARAM_EXT_ADV_MAX_SETS       = 0x331, /**<  Maximum number of supported advertising sets. Read only. Size is 1 byte. */
} T_LE_EXT_ADV_PARAM_TYPE;

/** End of GAP_LE_EXTENDED_ADV_Exported_Types
  * @}
  */

/*============================================================================*
 *                         Functions
 *============================================================================*/
/** @defgroup GAP_LE_EXTENDED_ADV_Exported_Functions GAP LE Extended Adv Exported Functions
  * @brief
  * @{
  */

/**
 * @brief   Get a GAP extended advertising parameter.
 *
 *          NOTE: You can call this function with a extended advertising parameter type and it will get a
 *          extended advertising parameter. Extended advertising parameters are defined in @ref T_LE_EXT_ADV_PARAM_TYPE.
 *
 * @param[in]      param   Advertising parameter type: @ref T_LE_EXT_ADV_PARAM_TYPE
 * @param[in,out]  p_value Pointer to the location to get the parameter value.  This is dependent on
 *                         the parameter type and will be cast to the appropriate data type (For example:
 *                         if data type of param is uint16_t, p_value will be cast to pointer of uint16_t).
 *
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS       Operation success.
 * @retval GAP_CAUSE_INVALID_PARAM Operation failure, invalid parameter.
 *
 * <b>Example usage</b>
 * \code{.c}
    void test(void)
    {
        uint16_t max_adv_data_len;
        le_ext_adv_get_param(GAP_PARAM_EXT_ADV_MAX_DATA_LEN, &max_adv_data_len);
    }
 * \endcode
 */
T_GAP_CAUSE le_ext_adv_get_param(T_LE_EXT_ADV_PARAM_TYPE param, void *p_value);

/**
 * @brief  Create an advertising handle which is used to identify an advertising set.
 *
 * @return Advertising handle.
 * @retval 0x00-0xFE Operation success.
 * @retval 0xFF      Operation failure.
 *
 * <b>Example usage</b>
 * \code{.c}
    void test(void)
    {
        uint8_t adv_handle;
        adv_handle = le_ext_adv_create_adv_handle();
    }
 * \endcode
 */
uint8_t le_ext_adv_create_adv_handle(void);

/**
 * @brief       Set GAP extended advertising parameters for an advertising set.
 *
 * @param[in]   adv_handle                    Identify an advertising set, which is assigned by @ref le_ext_adv_create_adv_handle.
 * @param[in]   adv_event_prop                Type of advertising event.
                                              Values for extended advertising PDUs: @ref T_LE_EXT_ADV_EXTENDED_ADV_PROPERTY.
                                              Values for legacy advertising PDUs: @ref T_LE_EXT_ADV_LEGACY_ADV_PROPERTY.
 * @param[in]   primary_adv_interval_min      Minimum advertising interval for undirected and low duty directed advertising.
                                              In units of 0.625ms, range: 0x000020 to 0xFFFFFF.
 * @param[in]   primary_adv_interval_max      Maximum advertising interval for undirected and low duty directed advertising.
                                              In units of 0.625ms, range: 0x000020 to 0xFFFFFF.
 * @param[in]   primary_adv_channel_map       A bit field that indicates the advertising channels that shall be used when
                                              transmitting advertising packets. @ref ADV_CHANNEL_MAP.
 * @param[in]   own_address_type              Local address type, @ref T_GAP_LOCAL_ADDR_TYPE.
 * @param[in]   peer_address_type             Remote address type, GAP_REMOTE_ADDR_LE_PUBLIC or GAP_REMOTE_ADDR_LE_RANDOM in @ref T_GAP_REMOTE_ADDR_TYPE.
                                              GAP_REMOTE_ADDR_LE_PUBLIC: Public Device Address or Public Identity Address.
                                              GAP_REMOTE_ADDR_LE_RANDOM: Random Device Address or Random(static) Identity Address.
 * @param[in]   p_peer_address                Remote address.
 * @param[in]   filter_policy                 Advertising filter policy: @ref T_GAP_ADV_FILTER_POLICY.
 * @param[in]   tx_power                      Advertising Tx power.
 * @param[in]   primary_adv_phy               Indicate the PHY on which the advertising packets are transmitted on the primary advertising channel.
                                              @ref T_GAP_PHYS_PRIM_ADV_TYPE.
                                              If legacy advertising PDUs are being used, the primary_adv_phy shall indicate the LE 1M PHY (@ref GAP_PHYS_PRIM_ADV_1M).
 * @param[in]   secondary_adv_max_skip        Maximum number of advertising events that can be skipped. Usually set to zero.
 * @param[in]   secondary_adv_phy             Indicate the PHY on which the advertising packets are transmitted on the secondary advertising channel.
                                              @ref T_GAP_PHYS_TYPE.
 * @param[in]   adv_sid                       Specify the value to be transmitted in Advertising SID subfield of those advertising channel
                                              PDUs that have this field. Usually set to zero.
 * @param[in]   scan_req_notification_enable  Indicate whether Host will be notified upon the receipt of a scan request PDU.
                                              Usually set to false.
 *
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS  Operation success.
 * @retval GAP_CAUSE_NOT_FIND Operation failure, the advertising handle is not found.
 *
 * <b>Example usage</b>
 * \code{.c}
     void le_init_ext_adv_params_ext_conn(void)
     {
        uint8_t adv_handle;
        T_LE_EXT_ADV_EXTENDED_ADV_PROPERTY adv_event_prop = LE_EXT_ADV_EXTENDED_ADV_CONN_UNDIRECTED;
        uint32_t primary_adv_interval_min = DEFAULT_ADVERTISING_INTERVAL_MIN;
        uint32_t primary_adv_interval_max = DEFAULT_ADVERTISING_INTERVAL_MAX;
        uint8_t  primary_adv_channel_map = GAP_ADVCHAN_ALL;
        T_GAP_LOCAL_ADDR_TYPE own_address_type = GAP_LOCAL_ADDR_LE_PUBLIC;
        T_GAP_REMOTE_ADDR_TYPE peer_address_type = GAP_REMOTE_ADDR_LE_PUBLIC;
        uint8_t p_peer_address[6] = {0};
        T_GAP_ADV_FILTER_POLICY filter_policy = GAP_ADV_FILTER_ANY;
        uint8_t tx_power = 127; //Host has no preference.
        T_GAP_PHYS_PRIM_ADV_TYPE primary_adv_phy = GAP_PHYS_PRIM_ADV_1M;
        uint8_t secondary_adv_max_skip = 0;
        T_GAP_PHYS_TYPE secondary_adv_phy = GAP_PHYS_2M;
        uint8_t adv_sid = 0;
        bool scan_req_notification_enable = false;

        adv_handle = le_ext_adv_create_adv_handle();
        le_ext_adv_set_adv_param(adv_handle,
                                 adv_event_prop,
                                 primary_adv_interval_min,
                                 primary_adv_interval_max,
                                 primary_adv_channel_map,
                                 own_address_type,
                                 peer_address_type,
                                 p_peer_address,
                                 filter_policy,
                                 tx_power,
                                 primary_adv_phy,
                                 secondary_adv_max_skip,
                                 secondary_adv_phy,
                                 adv_sid,
                                 scan_req_notification_enable);
     }
 * \endcode
 */
T_GAP_CAUSE le_ext_adv_set_adv_param(uint8_t adv_handle, uint16_t adv_event_prop,
                                     uint32_t primary_adv_interval_min, uint32_t primary_adv_interval_max,
                                     uint8_t primary_adv_channel_map, T_GAP_LOCAL_ADDR_TYPE own_address_type,
                                     T_GAP_REMOTE_ADDR_TYPE peer_address_type, uint8_t *p_peer_address,
                                     T_GAP_ADV_FILTER_POLICY filter_policy, uint8_t tx_power,
                                     T_GAP_PHYS_PRIM_ADV_TYPE primary_adv_phy, uint8_t secondary_adv_max_skip,
                                     T_GAP_PHYS_TYPE secondary_adv_phy, uint8_t adv_sid,
                                     bool scan_req_notification_enable);

/**
 * @brief       Set GAP advertising data for an advertising set.
 *
 * @param[in]   adv_handle       Identify an advertising set, which is assigned by @ref le_ext_adv_create_adv_handle.
 * @param[in]   adv_data_len     The length of advertising data.
 * @param[in]   p_adv_data       Pointer to advertising data to write.
 *
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS       Operation success.
 * @retval GAP_CAUSE_NOT_FIND      Operation failure, the advertising handle is not found.
 * @retval GAP_CAUSE_INVALID_PARAM Operation failure, the length of advertising data exceeds
                                   1024 bytes(Maximum total length of GAP extended advertising data).
 *
 * <b>Example usage</b>
 * \code{.c}
    static const uint8_t ext_adv_data[] =
    {
        // Flags
        0x02,
        GAP_ADTYPE_FLAGS,
        GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
        // Local name
        0x13,
        GAP_ADTYPE_LOCAL_NAME_COMPLETE,
        'B', 'L', 'E', '_', 'B', 'T', '5', '_', 'P', 'e', 'r', 'i', 'p', 'h', 'e', 'r', 'a', 'l',
        // Manufacturer Specific Data
        0xdd,
        GAP_ADTYPE_MANUFACTURER_SPECIFIC,
        0x5d, 0x00,
        0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
        0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
        0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
        0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f,
        0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf,
        0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9
    };

    void test(void)
    {
        le_ext_adv_set_adv_data(adv_handle, sizeof(ext_adv_data), (uint8_t *)ext_adv_data);
    }
 * \endcode
 */
T_GAP_CAUSE le_ext_adv_set_adv_data(uint8_t adv_handle, uint16_t adv_data_len, uint8_t *p_adv_data);

/**
 * @brief       Set GAP scan response data for an advertising set.
 *
 * @param[in]   adv_handle       Identify an advertising set, which is assigned by @ref le_ext_adv_create_adv_handle.
 * @param[in]   scan_data_len    The length of scan response data.
 * @param[in]   p_scan_data      Pointer to scan response data to write.
 *
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS       Operation success.
 * @retval GAP_CAUSE_NOT_FIND      Operation failure, the advertising handle is not found.
 * @retval GAP_CAUSE_INVALID_PARAM Operation failure, the length of advertising data exceeds
                                   1024 bytes(Maximum total length of GAP extended advertising data).
 *
 * <b>Example usage</b>
 * \code{.c}
    static const uint8_t ext_scan_rsp_data[] =
    {
        // Manufacturer Specific Data
        0xfc,
        GAP_ADTYPE_MANUFACTURER_SPECIFIC,
        0x5d, 0x00,
        0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
        0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
        0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
        0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f,
        0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, 0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf,
        0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf,
        0xe0, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xeb, 0xec, 0xed, 0xee, 0xef, 0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9
    };

    void test(void)
    {
        le_ext_adv_set_scan_response_data(adv_handle, sizeof(ext_scan_rsp_data), (uint8_t *)ext_scan_rsp_data);
    }
 * \endcode
 */
T_GAP_CAUSE le_ext_adv_set_scan_response_data(uint8_t adv_handle, uint16_t scan_data_len,
                                              uint8_t *p_scan_data);

/**
 * @brief       Set GAP random device address for an advertising set.
 *
 * @param[in]   adv_handle       Identify an advertising set, which is assigned by @ref le_ext_adv_create_adv_handle.
 * @param[in]   random_address   Pointer to random address to write.
 *
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS  Operation success.
 * @retval GAP_CAUSE_NOT_FIND Operation failure, the advertising handle is not found.
 *
 * <b>Example usage</b>
 * \code{.c}
    void test(void)
    {
        uint8_t rand_addr[6];
        le_gen_rand_addr(GAP_RAND_ADDR_STATIC, rand_addr);
        le_ext_adv_set_random(adv_handle, rand_addr);
    }
 * \endcode
 */
T_GAP_CAUSE le_ext_adv_set_random(uint8_t adv_handle, uint8_t *random_address);

/**
 * @brief       Set extended advertising parameters for an advertising set.
                If request success, the result of setting extended advertising parameters will be returned by
                @ref app_gap_callback with cb_type @ref GAP_MSG_LE_EXT_ADV_START_SETTING.
 *
 * @param[in]   adv_handle     Identify an advertising set, which is assigned by @ref le_ext_adv_create_adv_handle.
 * @param[in]   update_flags   A bit field that indicates extended advertising parameters that shall be set. @ref EXT_ADV_PARAM.
                               Recommendation: if random address is not used, set update_flags to @ref EXT_ADV_SET_AUTO.
 *
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS  Set parameters request success.
 * @retval GAP_CAUSE_NOT_FIND Set parameters request failed, the advertising handle is not found.
 *
 * <b>Example usage</b>
 * \code{.c}
    void le_init_ext_adv_params_ext(void)
    {
        T_LE_EXT_ADV_EXTENDED_ADV_PROPERTY adv_event_prop =
            LE_EXT_ADV_EXTENDED_ADV_NON_SCAN_NON_CONN_UNDIRECTED;
        uint32_t primary_adv_interval_min = DEFAULT_ADVERTISING_INTERVAL_MIN;
        uint32_t primary_adv_interval_max = DEFAULT_ADVERTISING_INTERVAL_MAX;
        uint8_t  primary_adv_channel_map = GAP_ADVCHAN_ALL;
        T_GAP_LOCAL_ADDR_TYPE own_address_type = GAP_LOCAL_ADDR_LE_PUBLIC;
        T_GAP_REMOTE_ADDR_TYPE peer_address_type = GAP_REMOTE_ADDR_LE_PUBLIC;
        uint8_t p_peer_address[6] = {0};
        T_GAP_ADV_FILTER_POLICY filter_policy = GAP_ADV_FILTER_ANY;
        uint8_t tx_power = 127; //Host has no preference.
        T_GAP_PHYS_PRIM_ADV_TYPE primary_adv_phy = GAP_PHYS_PRIM_ADV_1M;
        uint8_t secondary_adv_max_skip = 0;
        T_GAP_PHYS_TYPE secondary_adv_phy = GAP_PHYS_2M;
        uint8_t adv_sid = 0;
        bool scan_req_notification_enable = false;

        adv_handle = le_ext_adv_create_adv_handle();
        le_ext_adv_set_adv_param(adv_handle,
                                 adv_event_prop,
                                 primary_adv_interval_min,
                                 primary_adv_interval_max,
                                 primary_adv_channel_map,
                                 own_address_type,
                                 peer_address_type,
                                 p_peer_address,
                                 filter_policy,
                                 tx_power,
                                 primary_adv_phy,
                                 secondary_adv_max_skip,
                                 secondary_adv_phy,
                                 adv_sid,
                                 scan_req_notification_enable);

        le_ext_adv_set_adv_data(adv_handle, sizeof(ext_adv_data), (uint8_t *)ext_adv_data);
    }

    void test(void)
    {
        le_init_ext_adv_params_ext();
        le_ext_adv_start_setting(adv_handle, EXT_ADV_SET_AUTO);
    }

    T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data)
    {
        T_APP_RESULT result = APP_RESULT_SUCCESS;
        T_LE_CB_DATA *p_data = (T_LE_CB_DATA *)p_cb_data;

        switch (cb_type)
        {
        case GAP_MSG_LE_EXT_ADV_START_SETTING:
        APP_PRINT_INFO3("GAP_MSG_LE_EXT_ADV_START_SETTING:cause 0x%x, flag 0x%x, adv_handle %d",
                        p_data->p_le_ext_adv_start_setting_rsp->cause, p_data->p_le_ext_adv_start_setting_rsp->flag,
                        p_data->p_le_ext_adv_start_setting_rsp->adv_handle);

        if (p_data->p_le_ext_adv_start_setting_rsp->cause == GAP_CAUSE_SUCCESS)
        {
            // Initialize enable parameters
            le_init_ext_adv_enable_params(p_data->p_le_ext_adv_start_setting_rsp->adv_handle);
            // Enable one advertising set
            le_ext_adv_enable(1, &p_data->p_le_ext_adv_start_setting_rsp->adv_handle);
        }
        break;
        }
    }
 * \endcode
 */
T_GAP_CAUSE le_ext_adv_start_setting(uint8_t adv_handle, uint8_t update_flags);

/**
 * @brief       Set GAP extended advertising enable parameters for an advertising set.
 *
 * @param[in]   adv_handle       Identify an advertising set, which is assigned by @ref le_ext_adv_create_adv_handle.
 * @param[in]   duration         If non-zero, indicates the duration that advertising set is enabled.
                                 0x0000:        No advertising duration.
                                 0x0001-0xFFFF: Advertising duration, in units of 10ms.
 * @param[in]   max_ext_adv_evt  If non-zero, indicates the maximum extended advertising events that shall be
                                 sent prior to disabling the extended advertising set.
                                 0x00:      No maximum number of advertising events.
                                 0x01-0xFF: Maximum number of extended advertising events to send prior to terminating
                                            the extended advertising.
 *
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS  Operation success.
 * @retval GAP_CAUSE_NOT_FIND Operartion failure, the advertising handle is not found.
 *
 * <b>Example usage</b>
 * \code{.c}
    void test(void)
    {
        uint16_t duration = 0;
        uint8_t  max_ext_adv_evt  = 0;

        le_ext_adv_set_adv_enable_param(adv_handle, duration, max_ext_adv_evt);
    }
 * \endcode
 */
T_GAP_CAUSE le_ext_adv_set_adv_enable_param(uint8_t adv_handle, uint16_t duration,
                                            uint8_t max_ext_adv_evt);

/**
 * @brief       Enable extended advertising for one or more advertising sets.
                If device changes to advertising state, @ref app_handle_ext_adv_state_evt will be called, and
                @ref app_gap_callback with cb_type @ref GAP_MSG_LE_EXT_ADV_ENABLE will be called.
 *
 * @param[in]   num_of_sets      Number of advertising sets to enable.
 * @param[in]   adv_handle       Pointer to advertising set to enable.
 *
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS  Enable request success.
 * @retval GAP_CAUSE_NOT_FIND Enable request failed, the advertising handle is not found.
 *
 * <b>Example usage</b>
 * \code{.c}
    void test(void)
    {
        case GAP_MSG_LE_EXT_ADV_START_SETTING:
        APP_PRINT_INFO3("GAP_MSG_LE_EXT_ADV_START_SETTING:cause 0x%x, flag 0x%x, adv_handle %d",
                        p_data->p_le_ext_adv_start_setting_rsp->cause, p_data->p_le_ext_adv_start_setting_rsp->flag,
                        p_data->p_le_ext_adv_start_setting_rsp->adv_handle);

        if (p_data->p_le_ext_adv_start_setting_rsp->cause == GAP_CAUSE_SUCCESS)
        {
            // Initialize enable parameters
            le_init_ext_adv_enable_params(p_data->p_le_ext_adv_start_setting_rsp->adv_handle);
            // Enable one advertising set
            le_ext_adv_enable(1, &p_data->p_le_ext_adv_start_setting_rsp->adv_handle);
        }
        break;
    }

    T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data)
    {
        T_APP_RESULT result = APP_RESULT_SUCCESS;
        T_LE_CB_DATA *p_data = (T_LE_CB_DATA *)p_cb_data;

        switch (cb_type)
        {
        case GAP_MSG_LE_EXT_ADV_ENABLE:
            APP_PRINT_INFO1("GAP_MSG_LE_EXT_ADV_ENABLE:cause 0x%x", p_data->le_cause.cause);
            break;
        }
    }

    void app_handle_gap_msg(T_IO_MSG *p_gap_msg)
    {
        T_LE_GAP_MSG gap_msg;
        uint8_t conn_id;
        memcpy(&gap_msg, &p_gap_msg->u.param, sizeof(p_gap_msg->u.param));

        APP_PRINT_TRACE1("app_handle_gap_msg: subtype %d", p_gap_msg->subtype);
        switch (p_gap_msg->subtype)
        {
        case GAP_MSG_LE_EXT_ADV_STATE_CHANGE:
            {
                app_handle_ext_adv_state_evt(gap_msg.msg_data.gap_ext_adv_state_change.adv_handle,
                                             (T_GAP_EXT_ADV_STATE)gap_msg.msg_data.gap_ext_adv_state_change.new_state,
                                             gap_msg.msg_data.gap_ext_adv_state_change.cause);
            }
            break;
        }
     }

     void app_handle_ext_adv_state_evt(uint8_t adv_handle, T_GAP_EXT_ADV_STATE new_state, uint16_t cause)
     {
        for (int i = 0; i < APP_MAX_ADV_SET; i++)
        {
            if (ext_adv_state[i].adv_handle == adv_handle)
            {
                APP_PRINT_INFO2("app_handle_ext_adv_state_evt: adv_handle = %d oldState = %d",
                                ext_adv_state[i].adv_handle, ext_adv_state[i].ext_adv_state);
                ext_adv_state[i].ext_adv_state = new_state;
                break;
            }
        }
        APP_PRINT_INFO2("app_handle_ext_adv_state_evt: adv_handle = %d newState = %d",
                        adv_handle, new_state);
        switch (new_state)
        {
        // device is idle
        case EXT_ADV_STATE_IDLE:
            {
                APP_PRINT_INFO2("EXT_ADV_STATE_IDLE: adv_handle %d, cause 0x%x", adv_handle, cause);
            }
            break;

        // device is advertising
        case EXT_ADV_STATE_ADVERTISING:
            {
                APP_PRINT_INFO2("EXT_ADV_STATE_ADVERTISING: adv_handle %d, cause 0x%x", adv_handle, cause);
            }
            break;

        default:
            break;
        }
     }
 * \endcode
 */
T_GAP_CAUSE le_ext_adv_enable(uint8_t num_of_sets, uint8_t *adv_handle);

/**
 * @brief       Disable extended advertising for one or more advertising sets.
                If device changes to idle state, @ref app_handle_ext_adv_state_evt will be called, and
                @ref app_gap_callback with cb_type @ref GAP_MSG_LE_EXT_ADV_DISABLE will be called.
 *
 * @param[in]   num_of_sets      Number of advertising sets to enable.
 * @param[in]   adv_handle       Pointer to advertising set to enable.
 *
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS  Disable request success.
 * @retval GAP_CAUSE_NOT_FIND Disable request failed, the advertising handle is not found.
 *
 * <b>Example usage</b>
 * \code{.c}
    static T_USER_CMD_PARSE_RESULT cmd_stopeadv(T_USER_CMD_PARSED_VALUE *p_parse_value)
    {
        uint8_t adv_handle[4];
        uint8_t num_of_sets = 1;
        T_GAP_CAUSE cause;
        adv_handle[0] = p_parse_value->dw_param[0];

        cause = le_ext_adv_disable(num_of_sets, adv_handle);
        return (T_USER_CMD_PARSE_RESULT)cause;
    }

    T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data)
    {
        T_APP_RESULT result = APP_RESULT_SUCCESS;
        T_LE_CB_DATA *p_data = (T_LE_CB_DATA *)p_cb_data;

        switch (cb_type)
        {
        case GAP_MSG_LE_EXT_ADV_DISABLE:
        APP_PRINT_INFO1("GAP_MSG_LE_EXT_ADV_DISABLE:cause 0x%x", p_data->le_cause.cause);
        break;
        }
    }

    void app_handle_gap_msg(T_IO_MSG *p_gap_msg)
    {
        T_LE_GAP_MSG gap_msg;
        uint8_t conn_id;
        memcpy(&gap_msg, &p_gap_msg->u.param, sizeof(p_gap_msg->u.param));

        APP_PRINT_TRACE1("app_handle_gap_msg: subtype %d", p_gap_msg->subtype);
        switch (p_gap_msg->subtype)
        {
        case GAP_MSG_LE_EXT_ADV_STATE_CHANGE:
            {
                app_handle_ext_adv_state_evt(gap_msg.msg_data.gap_ext_adv_state_change.adv_handle,
                                             (T_GAP_EXT_ADV_STATE)gap_msg.msg_data.gap_ext_adv_state_change.new_state,
                                             gap_msg.msg_data.gap_ext_adv_state_change.cause);
            }
            break;
        }
     }

     void app_handle_ext_adv_state_evt(uint8_t adv_handle, T_GAP_EXT_ADV_STATE new_state, uint16_t cause)
     {
        for (int i = 0; i < APP_MAX_ADV_SET; i++)
        {
            if (ext_adv_state[i].adv_handle == adv_handle)
            {
                APP_PRINT_INFO2("app_handle_ext_adv_state_evt: adv_handle = %d oldState = %d",
                                ext_adv_state[i].adv_handle, ext_adv_state[i].ext_adv_state);
                ext_adv_state[i].ext_adv_state = new_state;
                break;
            }
        }
        APP_PRINT_INFO2("app_handle_ext_adv_state_evt: adv_handle = %d newState = %d",
                        adv_handle, new_state);
        switch (new_state)
        {
        // device is idle
        case EXT_ADV_STATE_IDLE:
            {
                APP_PRINT_INFO2("EXT_ADV_STATE_IDLE: adv_handle %d, cause 0x%x", adv_handle, cause);
            }
            break;

        // device is advertising
        case EXT_ADV_STATE_ADVERTISING:
            {
                APP_PRINT_INFO2("EXT_ADV_STATE_ADVERTISING: adv_handle %d, cause 0x%x", adv_handle, cause);
            }
            break;

        default:
            break;
        }
     }
 * \endcode
 */
T_GAP_CAUSE le_ext_adv_disable(uint8_t num_of_sets, uint8_t *adv_handle);

/**
 * @brief       Remove all existing advertising sets.
                The result of removing all existing advertising sets will be returned by
                @ref app_gap_callback with cb_type @ref GAP_MSG_LE_EXT_ADV_CLEAR_SET.
 *
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS Clear request success.
 *
 * <b>Example usage</b>
 * \code{.c}
    void test(void)
    {
        le_ext_adv_clear_set();
    }

    T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data)
    {
        T_APP_RESULT result = APP_RESULT_SUCCESS;
        T_LE_CB_DATA *p_data = (T_LE_CB_DATA *)p_cb_data;

        switch (cb_type)
        {
        case GAP_MSG_LE_EXT_ADV_CLEAR_SET:
        APP_PRINT_INFO1("GAP_MSG_LE_EXT_ADV_CLEAR_SET:cause 0x%x",
                        p_data->p_le_ext_adv_clear_set_rsp->cause);
        break;
        }
    }
 * \endcode
 */
T_GAP_CAUSE le_ext_adv_clear_set(void);

/**
 * @brief       Remove an advertising set.
                If request success, the result of removing an advertising set will be returned by
                @ref app_gap_callback with cb_type @ref GAP_MSG_LE_EXT_ADV_REMOVE_SET.
 *
 * @param[in]   adv_handle     Identify an advertising set.
 *
 * @return Operation result.
 * @retval GAP_CAUSE_SUCCESS        Remove request success.
 * @retval GAP_CAUSE_NOT_FIND       Remove request failed, the advertising handle is not found.
 * @retval GAP_CAUSE_INVALID_STATE  Remove request failed, invalid state.
 * @retval GAP_CAUSE_ALREADY_IN_REQ Remove request failed, operation is already in progress.
 *
 * <b>Example usage</b>
 * \code{.c}
    void test(void)
    {
        le_ext_adv_remove_set(adv_handle);
    }

    T_APP_RESULT app_gap_callback(uint8_t cb_type, void *p_cb_data)
    {
        T_APP_RESULT result = APP_RESULT_SUCCESS;
        T_LE_CB_DATA *p_data = (T_LE_CB_DATA *)p_cb_data;

        switch (cb_type)
        {
        case GAP_MSG_LE_EXT_ADV_REMOVE_SET:
        APP_PRINT_INFO2("GAP_MSG_LE_EXT_ADV_REMOVE_SET:cause 0x%x, adv_handle %d",
                        p_data->p_le_ext_adv_remove_set_rsp->cause, p_data->p_le_ext_adv_remove_set_rsp->adv_handle);
        break;
        }
    }
 * \endcode
 */
T_GAP_CAUSE le_ext_adv_remove_set(uint8_t adv_handle);


/** End of GAP_LE_EXTENDED_ADV_Exported_Functions
  * @}
  */

/** End of GAP_LE_EXTENDED_ADV
  * @}
  */

/** End of GAP_LE
  * @}
  */

/** End of GAP
  * @}
  */


#endif
#ifdef __cplusplus
}
#endif

#endif /* GAP_EXT_ADV_H */
