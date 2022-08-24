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
//#include <ota_service.h>
#include <app_task.h>
#include "board.h"
#include "rtl876x.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_io_dlps.h"
#include "rtl876x_uart.h"
#include "rtl876x_rcc.h"
#include "rtl876x_nvic.h"
#include "rtl876x_gpio.h"
#include <dlps.h>
#include "dfu_application.h"
#include "uart_application.h"
/*
********************************************************
* parameter for btstack
*
*
*********************************************************
*/

bool allowedDfuEnterDlps = true;
/**
 * @brief    pinmux configuration
 * @return   void
 */
void PINMUX_Configuration(void)
{
    Pinmux_Config(UART_TX, UART0_TX);
    Pinmux_Config(UART_RX, UART0_RX);
    Pinmux_Config(UART_CTS, UART0_CTS);
    Pinmux_Config(UART_RTS, UART0_RTS);
    return;
}
/**
 * @brief    pad configuration
 * @return   void
 */
void PAD_Configuration(void)
{
    Pad_Config(UART_TX, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(UART_RX, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(UART_CTS, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(UART_RTS, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_LOW);
    return;
}

/**
* @brief    Driver_Init() contains the initialization of peripherals.
*
*               Both new architecture driver and legacy driver initialization method can be used.
*
* @return  void
*/
void Driver_Init(void)
{
    DataTrans_UARTInit();
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
    Pad_Config(UART_TX, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(UART_RX, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(UART_CTS, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(UART_RTS, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_ENABLE, PAD_OUT_LOW);
    System_WakeUpDebounceTime(0x8);
    System_WakeUpPinEnable(UART_RX, PAD_WAKEUP_POL_LOW, PAD_WK_DEBOUNCE_ENABLE);
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
    Pad_Config(UART_TX, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(UART_RX, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(UART_CTS, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(UART_RTS, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_LOW);
}
/**
 * @brief    System_Handler
 * @note     system handle to judge which pin is wake source
 * @return   void
 */
void System_Handler(void)
{
    uint8_t tmpVal;
    DBG_DIRECT("System_Handler");
    //APP_PRINT_INFO0("System_Handler");
    NVIC_DisableIRQ(System_IRQn);

    UART0_Handler();

    // need clear debounce bit here.
    tmpVal = btaon_fast_read_safe(0x2b);
    btaon_fast_write_safe(0x2b, (tmpVal | BIT7));

    NVIC_ClearPendingIRQ(System_IRQn);
}
/**
 * @brief DLPS CallBack function
 * @param none
* @return true : allow enter dlps
 * @retval void
*/
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
    pwr_mgr_init();
    task_init();
    os_sched_start();

    return 0;
}
