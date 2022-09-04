/**
*****************************************************************************************
*     Copyright(c) 2017, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
   * @file      main.c
   * @brief     Source file for BLE scatternet project, mainly used for initialize modules
   * @author    jane
   * @date      2017-06-12
   * @version   v1.0
   **************************************************************************************
   * @attention
   * <h2><center>&copy; COPYRIGHT 2017 Realtek Semiconductor Corporation</center></h2>
   **************************************************************************************
  */

/*============================================================================*
 *                              Header Files
 *============================================================================*/
#include <stdlib.h>
#include <os_sched.h>
#include <string.h>
#include <trace.h>
#include <gap.h>
#include <gap_adv.h>
#include <gap_bond_le.h>
#include <profile_server.h>
#include <gap_msg.h>
#include <bas.h>
#include <app_task.h>
#include <pxp_app.h>
#include <ias.h>
#include <lls.h>
#include <tps.h>
#include <kns.h>
#include <bas.h>
#include <dis.h>
#include "board.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_gpio.h"
#include "rtl876x_io_dlps.h"
#include "rtl876x_rcc.h"
#include "iohandle.h"

#include <dlps.h>

#include "otp_config.h"
#if (AON_WDG_ENABLE == 1)
#include "rtl876x_aon_wdg.h"
#endif

//#include <gatt_builtin_services.h>
#define GATT_UUID_IMMEDIATE_ALERT_SERVICE       0x1802
/** @addtogroup  BBPRO_SDK
    * @{
    */

/** @addtogroup  SAMPLE_APP
    * @{
    */

/** @defgroup  SCATTERNET_DEMO BLE Scatternet App
    * @brief The BLE scatternet project can be used as a framework for developing many different scatternet applications.
    * @{
    */

/** @defgroup  SCATTERNET_DEMO_MAIN Scatternet Main
    * @brief Main file to initialize hardware and BT stack and start task scheduling
    * @{
    */

bool allowedPxpEnterDlps = true;

/** @brief  Default minimum advertising interval when device is discoverable (units of 625us, 160=100ms) */
#define DEFAULT_ADVERTISING_INTERVAL_MIN            320
/** @brief  Default Maximum advertising interval */
#define DEFAULT_ADVERTISING_INTERVAL_MAX            320

#if 0
/** @brief  GAP - scan response data (max size = 31 bytes) */
static const uint8_t scan_rsp_data[] =
{
    0x03,
    GAP_ADTYPE_APPEARANCE,
    LO_WORD(GAP_GATT_APPEARANCE_UNKNOWN),
    HI_WORD(GAP_GATT_APPEARANCE_UNKNOWN),
};

/** @brief  GAP - Advertisement data (max size = 31 bytes, best kept short to conserve power) */
static const uint8_t adv_data[] =
{
    0x02,
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    0x03,
    GAP_ADTYPE_16BIT_COMPLETE,
    LO_WORD(GATT_UUID_IMMEDIATE_ALERT_SERVICE),//0x02,//
    HI_WORD(GATT_UUID_IMMEDIATE_ALERT_SERVICE),//0x18,//

    0x09,
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'R', 'E', 'A', 'L', '_', 'P', 'X', 'P'
};
#else
// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scan_rsp_data[] =
{
    /* BT stack appends Local Name.                                    */
    0x03,           /* length     */
    0x19,           /* type="Appearance" */
    0x40, 0x02,     /* Keyring */

    /* place holder for Local Name, filled by BT stack. if not present */
    0x09,           /* length     */
    0x09,           /* type="Complete local name" */
    'B', 'L', 'B', '_', 'P', 'R', 'O', 'X' /* BLB_PROX */
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t adv_data[] =
{
    /* Core spec. Vol. 3, Part C, Chapter 18 */
    /* Flags */
    0x02,       /* length     */
    0x01, 0x06, /* type="flags", data="bit 1: LE General Discoverable Mode" */

//    /* Adv data encrypt sample*/
//    0x07,       /* length     */
//    GAP_ADTYPE_MANUFACTURER_SPECIFIC, /* AD Type */
//    0x5d, 0x00, /*Company ID  */
//    'D', 'A', 'T', 'A', /* data */

    /* Service */
    0x11,       /* length     */
    0x07,       /* type="Complete 128-bit UUIDs available" */
    0xA6, 0xF6, 0xF6, 0x07,
    0x4D, 0xC4, 0x9D, 0x98,
    0x6D, 0x45, 0x29, 0xBB,
    0xD0, 0xFF, 0x00, 0x00
};
#endif
/*============================================================================*
 *                              Functions
 *============================================================================*/
/**
  * @brief  Initialize gap related parameters
  * @return void
  */
void app_le_gap_init(void)
{
    /* Device name and device appearance */
    uint8_t  device_name[GAP_DEVICE_NAME_LEN] = "REAL_PXP";
    uint16_t appearance = GAP_GATT_APPEARANCE_GENERIC_KEYRING;
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
    uint16_t auth_flags = GAP_AUTHEN_BIT_BONDING_FLAG;
    uint8_t  auth_io_cap = GAP_IO_CAP_NO_INPUT_NO_OUTPUT;
    uint8_t  auth_oob = false;
    uint8_t  auth_use_fix_passkey = false;
    uint32_t auth_fix_passkey = 0;

    uint8_t  auth_sec_req_enable = false;

    uint16_t auth_sec_req_flags = GAP_AUTHEN_BIT_BONDING_FLAG;

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
}

/**
 * @brief  Add GATT services, clients and register callbacks
 * @return void
 */
void app_le_profile_init(void)
{
    server_init(6);
    //simp_srv_id = simp_ble_service_add_service(app_profile_callback);
    ias_srv_id  = ias_add_service(app_profile_callback);
    lls_srv_id  = lls_add_service(app_profile_callback);
    tps_srv_id  = tps_add_service(app_profile_callback);
    kns_srv_id  = kns_add_service(app_profile_callback);
    bas_srv_id  = bas_add_service(app_profile_callback);
    dis_srv_id  = dis_add_service(app_profile_callback);
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
    RCC_Configuration();
    PINMUX_Configuration();
    PAD_Configuration();
}
/**
 * @brief    System_Handler
 * @note     system handle to judge which pin is wake source
 * @return   void
 */
void System_Handler(void)
{
    uint8_t tmpVal;

    APP_PRINT_INFO0("System_Handler");
    NVIC_DisableIRQ(System_IRQn);

    KEY_INT_Handle();//P2_4  edge triggle , can't interrupt after dlps

    // need clear debounce bit here.
    tmpVal = btaon_fast_read_safe(0x2b);
    btaon_fast_write_safe(0x2b, (tmpVal | BIT7));

//    if (System_WakeUpInterruptValue(KEY) == SET)
//    {
//        APP_PRINT_INFO0("ROW0");
//        //allowedPxpEnterDlps = false;
//        //os_timer_start(&xTimersKeyWake);
//        Pad_ClearWakeupINTPendingBit(KEY);
//        GPIO->DEBOUNCE &= ~GPIO_GetPin(KEY);
//    }

    NVIC_ClearPendingIRQ(System_IRQn);
}
/**
 * @brief    PxpEnterDlpsSet
 * @note     enter dlps to set pad&wake up pin
 * @return   void
 */
void PxpEnterDlpsSet(void)
{
    Pad_Config(KEY, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(LED, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(BEEP, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    System_WakeUpDebounceTime(0x8);
    if (keystatus)
    {
        System_WakeUpPinEnable(KEY, PAD_WAKEUP_POL_LOW, PAD_WK_DEBOUNCE_ENABLE);
    }
    else
    {
        System_WakeUpPinEnable(KEY, PAD_WAKEUP_POL_HIGH, PAD_WK_DEBOUNCE_ENABLE);
    }

#if (AON_WDG_ENABLE == 1)
    aon_wdg_enable();
#endif
}
/**
 * @brief    PxpExitDlpsInit
 * @note     Exit dlps to configure pad
 * @return   void
 */
void PxpExitDlpsInit(void)
{
    Pad_Config(LED, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(BEEP, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(KEY, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);

#if (AON_WDG_ENABLE == 1)
    aon_wdg_disable();
#endif
}
/**
 * @brief    DLPS_PxpCheck
 * @note     check app enter dlps flg.
 * @return   bool
 */
bool DLPS_PxpCheck(void)
{
    return allowedPxpEnterDlps;
}

/**
 * @brief    Contains the power mode settings
 * @return   void
 */
void pwr_mgr_init(void)
{
#if DLPS_EN
    if (false == dlps_check_cb_reg(DLPS_PxpCheck))
    {
        DBG_DIRECT("Error: dlps_check_cb_reg(DLPS_RcuCheck) failed!\n");
    }
    DLPS_IORegUserDlpsEnterCb(PxpEnterDlpsSet);
    DLPS_IORegUserDlpsExitCb(PxpExitDlpsInit);
    DLPS_IORegister();
    lps_mode_set(LPM_DLPS_MODE);
#endif
}

/**
 * @brief    Contains the initialization of all tasks
 * @note     There is only one task in BLE Scatternet APP, thus only one APP task is init here
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

    le_gap_init(APP_MAX_LINKS);
    gap_lib_init();
    app_le_gap_init();
    app_le_profile_init();
    pwr_mgr_init();
    swTimerInit();
    task_init();
    os_sched_start();

    return 0;
}
/** @} */ /* End of group SCATTERNET_DEMO_MAIN */
/** @} */ /* End of group SCATTERNET_DEMO */
/** @} */ /* End of group SAMPLE_APP */
/** @} */ /* End of group BBPRO_SDK */

