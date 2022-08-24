/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     main.c
* @brief    This is the entry of user code which the main function resides in.
* @details
* @author   ranhui
* @date     2015-03-29
* @version  v0.2
*********************************************************************************************************
*/
#include <stdlib.h>
#include <os_sched.h>
#include <string.h>
#include <trace.h>
#include <gap.h>
#include <gap_adv.h>
#include <gap_bond_le.h>
#include <profile_server.h>
#include <gap_msg.h>
#include <ota_service.h>
#include <dfu_service.h>
#include <bas.h>
#include <dis.h>
#include <app_task.h>
#include "board.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_io_dlps.h"
#include "rtl876x_rcc.h"
#include "rtl876x_nvic.h"
#include "rtl876x_gpio.h"
#include <dlps.h>
#include "dfu_application.h"

/*
********************************************************
* parameter for btstack
*
*
*********************************************************
*/
// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL_MIN            320 /* 20ms */
#define DEFAULT_ADVERTISING_INTERVAL_MAX            320 /* 30ms */


bool allowedDfuEnterDlps = true;

static uint8_t SCAN_RSP_DATA[] =
{
    /* place holder for Local Name, filled by BT stack. if not present */
    /* BT stack appends Local Name.                                    */
    0x03,           /* length     */
    0x19,           /* type="Appearance" */
    0xC1,
    0x03,           /*Key board*/
};

static uint8_t ADV_DATA[] =
{
    /* Core spec. Vol. 3, Part C, Chapter 18 */
    /* Flags */
    0x02,            /* length     */
    //XXXXMJMJ 0x01, 0x06,      /* type="flags", data="bit 1: LE General Discoverable Mode", BR/EDR not supp. */
    0x01, 0x05,      /* type="flags", data="bit 1: LE Limited Discoverable Mode" */
    /* Service */
    0x03,           /* length     */
    0x03,           /* type="More 16-bit UUIDs available" */
    0x12,
    0x18,
    /* place holder for Local Name, filled by BT stack. if not present */
    /* BT stack appends Local Name.                                    */
    0x03,           /* length     */
    0x19,           /* type="Appearance" */
    0xc1, 0x03,     /* Key Board */

    0x0B,           /* length     */
    0x09,           /* type="Complete local name" */
    'R', 'e', 'a', 'l', 'T', 'e', 'k', 'D', 'f', 'u',

    0x05,           /* length     */
    0xFF,           /* reserved */
    0x5D,
    0x00,
    0x04,
    0x00,
};
/******************************************************************
 * @fn          Initial gap parameters
 * @brief      Initialize peripheral and gap bond manager related parameters
 *
 * @return     void
 */
void app_le_gap_init(void)
{
    //device name and device appearance
    uint8_t  device_name[GAP_DEVICE_NAME_LEN] = "RealTekDfu";
    uint16_t appearance = GAP_GATT_APPEARANCE_KEYBOARD;
    uint8_t  slave_init_mtu_req = true;

    //advertising parameters
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


    //Register gap callback
    le_register_app_cb(app_gap_callback);

    //Set device name and device appearance
    le_set_gap_param(GAP_PARAM_DEVICE_NAME, GAP_DEVICE_NAME_LEN, device_name);
    le_set_gap_param(GAP_PARAM_APPEARANCE, sizeof(appearance), &appearance);
    le_set_gap_param(GAP_PARAM_SLAVE_INIT_GATT_MTU_REQ, sizeof(slave_init_mtu_req),
                     &slave_init_mtu_req);
    //Set advertising parameters
    le_adv_set_param(GAP_PARAM_ADV_EVENT_TYPE, sizeof(adv_evt_type), &adv_evt_type);
    le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR_TYPE, sizeof(adv_direct_type), &adv_direct_type);
    le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR, sizeof(adv_direct_addr), adv_direct_addr);
    le_adv_set_param(GAP_PARAM_ADV_CHANNEL_MAP, sizeof(adv_chann_map), &adv_chann_map);
    le_adv_set_param(GAP_PARAM_ADV_FILTER_POLICY, sizeof(adv_filter_policy), &adv_filter_policy);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MIN, sizeof(adv_int_min), &adv_int_min);
    le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MAX, sizeof(adv_int_max), &adv_int_max);
    le_adv_set_param(GAP_PARAM_ADV_DATA, sizeof(ADV_DATA), ADV_DATA);
    le_adv_set_param(GAP_PARAM_SCAN_RSP_DATA, sizeof(SCAN_RSP_DATA), SCAN_RSP_DATA);

    // Setup the GAP Bond Manager
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

}

/******************************************************************
 * @fn          Initial profile
 * @brief      Add simple profile service and register callbacks
 *
 * @return     void
 */
void app_le_profile_init(void)
{
    server_init(4);
    gBASServiceId = bas_add_service(app_profile_callback);
    gDISServiceId = dis_add_service(app_profile_callback);
    gDfuServiceId = dfu_add_service(app_profile_callback);
    gOtaServiceId = ota_add_service(app_profile_callback);
    server_register_app_cb(app_profile_callback);
}
/**
 * @brief    pinmux configuration
 * @return   void
 */
void PINMUX_Configuration(void)
{
    Pinmux_Config(TP0, DWGPIO);
    Pinmux_Config(KEY, DWGPIO);
    return;
}
/**
 * @brief    pad configuration
 * @return   void
 */
void PAD_Configuration(void)
{
    Pad_Config(TP0, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(KEY, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    return;
}

/**
* @brief    Driver_Init() contains the initialization of peripherals.
*
*               Both new architecture driver and legacy driver initialization method can be used.
*
* @return  void
*/
uint8_t mode;
uint8_t keystatus;
void Driver_Init(void)
{
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    GPIO_InitTypeDef Gpio_Struct;
    GPIO_StructInit(&Gpio_Struct);
    Gpio_Struct.GPIO_Pin = GPIO_GetPin(TP0);
    Gpio_Struct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(&Gpio_Struct);
    mode = !GPIO_ReadInputDataBit(GPIO_GetPin(TP0));

    Gpio_Struct.GPIO_Pin = GPIO_GetPin(KEY);
    Gpio_Struct.GPIO_Mode = GPIO_Mode_IN;
    Gpio_Struct.GPIO_ITCmd = ENABLE;
    Gpio_Struct.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
    Gpio_Struct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
    Gpio_Struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
    Gpio_Struct.GPIO_DebounceTime = 20;
    GPIO_Init(&Gpio_Struct);

    keystatus = GPIO_ReadInputDataBit(GPIO_GetPin(KEY));
    GPIO_MaskINTConfig(GPIO_GetPin(KEY), DISABLE);
    GPIO_INTConfig(GPIO_GetPin(KEY), ENABLE);

    NVIC_InitTypeDef NVIC_InitStruct;
    //NVIC_InitStruct.NVIC_IRQChannel = GPIO5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannel = KEY_IRQ;//P2_4
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}
#if SUPPORT_ERASE_SUSPEND
#include "app_section.h"
void app_flash_erase_suspend(void);
void app_flash_erase_resume(void);
extern bool erase_in_progress ;
DATA_RAM_FUNCTION
#endif
void KEY_INT_Handle(void)//P2_4
{
    //T_IO_MSG bee_io_msg;
#if SUPPORT_ERASE_SUSPEND
    app_flash_erase_suspend();
#endif
    //APP_PRINT_ERROR0("Enter GPIO20_Handler!");
    DBG_DIRECT("Enter GPIO20_Handler!");
    GPIO_MaskINTConfig(GPIO_GetPin(KEY),
                       ENABLE);
    keystatus = GPIO_ReadInputDataBit(GPIO_GetPin(KEY));

    if (keystatus == 0)
    {
        GPIO->INTPOLARITY |= GPIO_GetPin(KEY);
    }
    else
    {
        GPIO->INTPOLARITY &= ~GPIO_GetPin(KEY);
    }
    GPIO_ClearINTPendingBit(GPIO_GetPin(KEY));
    GPIO_MaskINTConfig(GPIO_GetPin(KEY), DISABLE);
#if SUPPORT_ERASE_SUSPEND
    app_flash_erase_resume();
#endif
}

/**
* @brief    Board_Init() contains the initialization of pinmux settings and pad settings.
*
*               All the pinmux settings and pad settings shall be initiated in this function.
*               But if legacy driver is used, the initialization of pinmux setting and pad setting
*               should be peformed with the IO initializing.
*
* @return  void
*/
void board_init(void)
{
//    RCC_Configuration();
    PINMUX_Configuration();
    PAD_Configuration();
}

/**
 * @brief this function will be called before enter DLPS
 *
 *  set PAD and wakeup pin config for enterring DLPS
 *
 * @param none
 * @return none
 * @retval void
*/
void RcuEnterDlpsSet(void)
{
    Pad_Config(KEY, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(TP0, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
    System_WakeUpDebounceTime(0x8);
    if (keystatus)
    {
        System_WakeUpPinEnable(KEY, PAD_WAKEUP_POL_LOW, PAD_WK_DEBOUNCE_ENABLE);
    }
    else
    {
        System_WakeUpPinEnable(KEY, PAD_WAKEUP_POL_HIGH, PAD_WK_DEBOUNCE_ENABLE);
    }
}

/**
 * @brief this function will be called after exit DLPS
 *
 *  set PAD and wakeup pin config for enterring DLPS
 *
 * @param none
 * @return none
 * @retval void
*/
void RcuExitDlpsInit(void)
{
    Pad_Config(TP0, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(KEY, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
}
/**
 * @brief    System_Handler
 * @note     system handle to judge which pin is wake source
 * @return   void
 */
#if SUPPORT_ERASE_SUSPEND
DATA_RAM_FUNCTION
#endif
void System_Handler(void)
{
    uint8_t tmpVal;
    DBG_DIRECT("System_Handler");
#if SUPPORT_ERASE_SUSPEND
    app_flash_erase_suspend();
#endif
    //APP_PRINT_INFO0("System_Handler");
    NVIC_DisableIRQ(System_IRQn);

    KEY_INT_Handle();//P2_4  edge triggle , can't interrupt after dlps

    // need clear debounce bit here.
    tmpVal = btaon_fast_read_safe(0x2b);
    btaon_fast_write_safe(0x2b, (tmpVal | BIT7));

    NVIC_ClearPendingIRQ(System_IRQn);
#if SUPPORT_ERASE_SUSPEND
    app_flash_erase_resume();
#endif
}
/**
 * @brief DLPS CallBack function
 * @param none
* @return true : allow enter dlps
 * @retval void
*/
#if SUPPORT_ERASE_SUSPEND
DATA_RAM_FUNCTION
#endif
bool DLPS_DfuCheck(void)
{
    return allowedDfuEnterDlps;
}

/**
* @brief    PwrMgr_Init() contains the setting about power mode.
*
* @return  void
*/
void pwr_mgr_init(void)
{
    /**
     * @brief Register Check CB to DlpsPlatform which will call it before entering Dlps.
     * @param  func -- DLPSEnterCheckFunc.
     * @return  Status of Operation.
     * @retval true if success, false if fail.
    */
#if DLPS_EN
    if (false == dlps_check_cb_reg(DLPS_DfuCheck))
    {
        DBG_DIRECT("Error: dlps_check_cb_reg(DLPS_RcuCheck) failed!\n");
    }
    DLPS_IORegUserDlpsEnterCb(RcuEnterDlpsSet);
    DLPS_IORegUserDlpsExitCb(RcuExitDlpsInit);
    DLPS_IORegister();
    lps_mode_set(LPM_DLPS_MODE);
#endif
}


/**
* @brief  Task_Init() contains the initialization of all the tasks.
*
*           There are four tasks are initiated.
*           Lowerstack task and upperstack task are used by bluetooth stack.
*           Application task is task which user application code resides in.
*           Emergency task is reserved.
*
* @return  void
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
    //driver_init();
    le_gap_init(1);
    gap_lib_init();
    app_le_gap_init();
    app_le_profile_init();
    pwr_mgr_init();
    task_init();
    os_sched_start();

    return 0;
}
