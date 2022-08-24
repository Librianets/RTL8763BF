/**
*****************************************************************************************
*     Copyright(c) 2018, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      main.c
   * @brief     Source file for BLE keyboard project, mainly used for initialize modules
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
#include <stdlib.h>
#include <os_sched.h>
#include <os_timer.h>
#include <string.h>
#include <trace.h>
#include <gap.h>
#include <gap_adv.h>
#include <gap_bond_le.h>
#include <profile_server.h>
#include <gap_msg.h>
#include <bas.h>
#include <dis.h>
#include <app_task.h>
#include <hids_kb.h>
#include <keyboard_app.h>
#include <keyscan.h>
#include <rtl876x_keyscan.h>
#include <rtl876x_io_dlps.h>
#include <keyboard_button.h>
#include <keyboard_led.h>
#include <privacy_mgnt.h>
#include <ota_service.h>
#include <profile_init.h>
#include <gatt_builtin_services.h>


/** @defgroup  PERIPH_DEMO_MAIN Peripheral Main
    * @brief Main file to initialize hardware and BT stack and start task scheduling
    * @{
    */

/*============================================================================*
 *                              Constants
 *============================================================================*/
/** @brief  Default minimum advertising interval when device is discoverable (units of 625us, 160=100ms) */
#define DEFAULT_ADVERTISING_INTERVAL_MIN            32
/** @brief  Default Maximum advertising interval */
#define DEFAULT_ADVERTISING_INTERVAL_MAX            32


/*============================================================================*
 *                              Variables
 *============================================================================*/

/** @brief  GAP - scan response data (max size = 31 bytes) */
static const uint8_t scan_rsp_data[] =
{
    0x03,                             /* length */
    GAP_ADTYPE_APPEARANCE,            /* type="Appearance" */
    LO_WORD(GAP_GATT_APPEARANCE_KEYBOARD),
    HI_WORD(GAP_GATT_APPEARANCE_KEYBOARD),
};

/** @brief  GAP - Advertisement data (max size = 31 bytes, best kept short to conserve power) */
static const uint8_t adv_data[] =
{
    /* Flags */
    0x02,             /* length */
    GAP_ADTYPE_FLAGS, /* type="Flags" */
    GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    /* Service */
    0x03,             /* length */
    GAP_ADTYPE_16BIT_COMPLETE,
    0x12,
    0x18,
    /* Local name */
    0x0D,             /* length */
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'B', 'L', 'E', '_', 'K', 'E', 'Y', 'B', 'O', 'A', 'R', 'D'
};

/*============================================================================*
 *                              Functions
 *============================================================================*/
/**
  * @brief  Initialize peripheral and gap bond manager related parameters
  * @return void
  */
void app_le_gap_init(void)
{
    /* Device name and device appearance */
    uint8_t  device_name[GAP_DEVICE_NAME_LEN] = "BLE_KEYBOARD";
    uint16_t appearance = GAP_GATT_APPEARANCE_KEYBOARD;
    uint8_t  slave_init_mtu_req = false;


    /* Advertising parameters */
    uint8_t  adv_evt_type = GAP_ADTYPE_ADV_IND;
    uint8_t  adv_direct_type = GAP_REMOTE_ADDR_LE_PUBLIC;
    uint8_t  adv_direct_addr[GAP_BD_ADDR_LEN] = {0};
    uint8_t  adv_chann_map = GAP_ADVCHAN_ALL;
    uint8_t  adv_filter_policy = GAP_ADV_FILTER_ANY;
    uint16_t adv_int_min = DEFAULT_ADVERTISING_INTERVAL_MIN;
    uint16_t adv_int_max = DEFAULT_ADVERTISING_INTERVAL_MIN;

    /* GAP Bond Manager parameters */
    uint8_t  auth_pair_mode = GAP_PAIRING_MODE_PAIRABLE;
    uint16_t auth_flags = GAP_AUTHEN_BIT_BONDING_FLAG | GAP_AUTHEN_BIT_MITM_FLAG;
    uint8_t  auth_io_cap = GAP_IO_CAP_KEYBOARD_ONLY;
    uint8_t  auth_oob = false;
    uint8_t  auth_use_fix_passkey = false;
    uint32_t auth_fix_passkey = 0;

    uint8_t  auth_sec_req_enable = false;
    uint16_t auth_sec_req_flags = GAP_AUTHEN_BIT_BONDING_FLAG | GAP_AUTHEN_BIT_MITM_FLAG;

    uint16_t conn_interval_min   = 6;
    uint16_t conn_interval_max   = 16;
    uint16_t slave_latency       = 20;
    uint16_t supervision_timeout = 3000;

    gaps_set_peripheral_preferred_conn_param(conn_interval_min, conn_interval_max,
                                             slave_latency, supervision_timeout / 10);

    /* Set device name and device appearance */
    le_set_gap_param(GAP_PARAM_DEVICE_NAME, GAP_DEVICE_NAME_LEN, device_name);
    le_set_gap_param(GAP_PARAM_APPEARANCE, sizeof(appearance), &appearance);
    le_set_gap_param(GAP_PARAM_SLAVE_INIT_GATT_MTU_REQ, sizeof(slave_init_mtu_req),
                     &slave_init_mtu_req);

    /* Set advertising parameters */
    le_adv_set_param(GAP_PARAM_ADV_EVENT_TYPE, sizeof(adv_evt_type), &adv_evt_type);
    le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR_TYPE, sizeof(adv_direct_type), &adv_direct_type);
    le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR, sizeof(adv_direct_addr), adv_direct_addr);
    le_adv_set_param(GAP_PARAM_ADV_CHANNEL_MAP, sizeof(adv_chann_map), &adv_chann_map);
    le_adv_set_param(GAP_PARAM_ADV_FILTER_POLICY, sizeof(adv_filter_policy), &adv_filter_policy);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MIN, sizeof(adv_int_min), &adv_int_min);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MAX, sizeof(adv_int_max), &adv_int_max);
    le_adv_set_param(GAP_PARAM_ADV_DATA, sizeof(adv_data), (void *)adv_data);
    le_adv_set_param(GAP_PARAM_SCAN_RSP_DATA, sizeof(scan_rsp_data), (void *)scan_rsp_data);

    /* Setup the GAP Bond Manager */
    gap_set_param(GAP_PARAM_BOND_PAIRING_MODE, sizeof(auth_pair_mode), &auth_pair_mode);
    gap_set_param(GAP_PARAM_BOND_AUTHEN_REQUIREMENTS_FLAGS, sizeof(auth_flags), &auth_flags);
    gap_set_param(GAP_PARAM_BOND_IO_CAPABILITIES, sizeof(auth_io_cap), &auth_io_cap);
    gap_set_param(GAP_PARAM_BOND_OOB_ENABLED, sizeof(auth_oob), &auth_oob);
    le_bond_set_param(GAP_PARAM_BOND_FIXED_PASSKEY, sizeof(auth_fix_passkey), &auth_fix_passkey);
    le_bond_set_param(GAP_PARAM_BOND_FIXED_PASSKEY_ENABLE, sizeof(auth_use_fix_passkey),
                      &auth_use_fix_passkey);
    le_bond_set_param(GAP_PARAM_BOND_SEC_REQ_ENABLE, sizeof(auth_sec_req_enable), &auth_sec_req_enable);
    le_bond_set_param(GAP_PARAM_BOND_SEC_REQ_REQUIREMENT, sizeof(auth_sec_req_flags),
                      &auth_sec_req_flags);

    /* register gap message callback */
    le_register_app_cb(app_gap_callback);
    privacy_init(app_privacy_callback, true);
}

/**
 * @brief  Add GATT services and register callbacks
 * @return void
 */
void app_le_profile_init(void)
{
    server_init(4);
    hid_srv_id = hids_add_service(app_profile_callback);
    bas_srv_id = bas_add_service(app_profile_callback);
    dis_srv_id = dis_add_service(app_profile_callback);
    ota_srv_id = ota_add_service(app_profile_callback);

    server_register_app_cb(app_profile_callback);

}

/**
 * @brief    Contains the initialization of pinmux settings and pad settings
 * @note     All the pinmux settings and pad settings shall be initiated in this function,
 *           but if legacy driver is used, the initialization of pinmux setting and pad setting
 *           should be peformed with the IO initializing.
 * @return   void
 */
void board_init(void)
{
    keyscan_init_data();
    keyscan_pinmux_config();
    gpio_button_pinmux_config();

    keyscan_pad_config();
    gpio_button_pad_config();
}

/**
 * @brief    Contains the initialization of peripherals
 * @note     Both new architecture driver and legacy driver initialization method can be used
 * @return   void
 */
void driver_init(void)
{
    keyscan_init_driver(KeyScan_Debounce_Enable);
    keyscan_nvic_config();
    keyscan_init_timer();

    keyboard_button_init();
    longpress_timer_init();

    adv_led_timer_init();
    keyboard_led_init();

    connection_param_updata_timer_init();
}

/**
* @brief  keyboard enter dlps callback function
* @param  No parameter.
* @return void
*/
void keyboard_enter_dlps(void)
{
    keyscan_enter_dlps_config();

    gpio_button_enter_dlps_config();
}

/**
* @brief   keyboard exit dlps callback function
* @param   No parameter.
* @return  void
*/
void keyboard_exit_dlps(void)
{
    keyscan_exit_dlps_config();

    gpio_button_exit_dlps_config();
}

/**
 * @brief    keyboard_enter_dlps_check
 * @note     APP register this callback for dlps check
 */
bool keyboard_enter_dlps_check(void)
{
    /*Here, check following condition to judge whether enter dlps or not*/
    return keyscan_global_data.is_allowed_to_enter_dlps;
}

/**
 * @brief    System_Handler
 * @note     Wake up by pad will trigger this interrupt
 * @return   void
 */
void System_Handler(void)
{
    NVIC_DisableIRQ(System_IRQn);
    //clear debounce bit
    uint8_t tmpVal;
    tmpVal = btaon_fast_read_safe(0x2b);

    if (tmpVal & BIT7)
    {
        /* pad signal wake up event */
        btaon_fast_write_safe(0x2b, (tmpVal | BIT7));
        handle_pair_button_wakeup();
        handle_keyscan_wakeup();
    }

    NVIC_ClearPendingIRQ(System_IRQn);
}

/**
 * @brief    Contains the power mode settings
 * @return   void
 */
void pwr_mgr_init(void)
{
    if (false == dlps_check_cb_reg(keyboard_enter_dlps_check))
    {
        APP_PRINT_INFO0("keyboard_enter_dlps_check reg fail!");
    }
    DLPS_IORegUserDlpsEnterCb(keyboard_enter_dlps);
    DLPS_IORegUserDlpsExitCb(keyboard_exit_dlps);
    DLPS_IORegister();

    lps_mode_set(LPM_DLPS_MODE);
}

/**
 * @brief    Contains the initialization of all tasks
 * @note     There is only one task in BLE Peripheral APP, thus only one APP task is init here
 * @return   void
 */
void task_init(void)
{
    app_task_init();
}

/**
 * @brief    Entry of APP code
 * @return   int (To avoid compile warning)
 */
int main(void)
{
    extern uint32_t random_seed_value;
    srand(random_seed_value);
    board_init();
    le_gap_init(1);
    gap_lib_init();
    app_le_gap_init();
    app_le_profile_init();
    pwr_mgr_init();
    task_init();
    os_sched_start();

    return 0;
}
/** @} */ /* End of group PERIPH_DEMO_MAIN */


