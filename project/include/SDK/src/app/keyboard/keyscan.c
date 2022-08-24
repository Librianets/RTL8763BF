/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file     keyscan.c
* @brief
* @details
* @author   Chuanguo Xue
* @date     2015-03-27
* @version  v1.0
*********************************************************************************************************
*/
#include <board.h>
#include <string.h>
#include <trace.h>
#include <app_task.h>
#include <keyscan.h>
#include <os_timer.h>
#include <rtl876x_rcc.h>
#include <rtl876x_keyscan.h>
#include <rtl876x_pinmux.h>
#include <rtl876x_nvic.h>
#include <rtl876x_gpio.h>


/*============================================================================*
 *                              Macros
 *============================================================================*/
#define keyscan_interrupt_handler Keyscan_Handler

/*============================================================================*
 *                              Local Variables
 *============================================================================*/

/*============================================================================*
*                              Global Variables
*============================================================================*/
T_KEYSCAN_GLOBAL_DATA keyscan_global_data;
void *keyscan_timer = NULL;

/*============================================================================*
*                              Local Functions
*============================================================================*/

void keyscan_pad_config(void)
{
    Pad_Config(ROW0, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(ROW1, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(ROW2, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(ROW3, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);

    Pad_Config(COLUMN0, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(COLUMN1, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(COLUMN2, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(COLUMN3, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_DISABLE, PAD_OUT_LOW);
}

void keyscan_pinmux_config(void)
{
    Pinmux_Config(ROW0, KEY_ROW_0);
    Pinmux_Config(ROW1, KEY_ROW_1);
    Pinmux_Config(ROW2, KEY_ROW_2);
    Pinmux_Config(ROW3, KEY_ROW_3);

    Pinmux_Config(COLUMN0, KEY_COL_0);
    Pinmux_Config(COLUMN1, KEY_COL_1);
    Pinmux_Config(COLUMN2, KEY_COL_2);
    Pinmux_Config(COLUMN3, KEY_COL_3);
}

/******************************************************************
 * @brief    keyscan enable wakeup config function
 */
void keyscan_enable_wakeup_config(void)
{
    /* @note: no key is pressed, use PAD wake up function with debounce,
    but pad debunce time should be smaller than ble connect interval */
    System_WakeUpDebounceTime(8);

    System_WakeUpPinEnable(ROW0, PAD_WAKEUP_POL_LOW, PAD_WK_DEBOUNCE_ENABLE);
    System_WakeUpPinEnable(ROW1, PAD_WAKEUP_POL_LOW, PAD_WK_DEBOUNCE_ENABLE);
    System_WakeUpPinEnable(ROW2, PAD_WAKEUP_POL_LOW, PAD_WK_DEBOUNCE_ENABLE);
    System_WakeUpPinEnable(ROW3, PAD_WAKEUP_POL_LOW, PAD_WK_DEBOUNCE_ENABLE);
}

/******************************************************************
 * @brief    keyscan disable wakeup config function
 */
void keyscan_disable_wakeup_config(void)
{
    System_WakeUpPinDisable(ROW0);
    System_WakeUpPinDisable(ROW1);
    System_WakeUpPinDisable(ROW2);
    System_WakeUpPinDisable(ROW3);
}


/******************************************************************
 * @brief    keyscan enter DLPS config
 */
void keyscan_enter_dlps_config(void)
{
    Pad_Config(COLUMN0, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(COLUMN1, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(COLUMN2, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(COLUMN3, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);

    if (keyscan_global_data.is_all_key_released == true)
    {
        keyscan_enable_wakeup_config();
        Pad_Config(ROW0, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
        Pad_Config(ROW1, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
        Pad_Config(ROW2, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
        Pad_Config(ROW3, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    }
    else
    {
        /* any key is pressed, disable key row pins, just wait keyscan sw timer to wake */
        keyscan_disable_wakeup_config();
        Pad_Config(ROW0, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_DISABLE, PAD_OUT_LOW);
        Pad_Config(ROW1, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_DISABLE, PAD_OUT_LOW);
        Pad_Config(ROW2, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_DISABLE, PAD_OUT_LOW);
        Pad_Config(ROW3, PAD_SW_MODE, PAD_IS_PWRON, PAD_PULL_DOWN, PAD_OUT_DISABLE, PAD_OUT_LOW);
    }
}

/******************************************************************
 * @brief    keyscan exit DLPS config
 */
void keyscan_exit_dlps_config(void)
{
    keyscan_pad_config();

    if (true == keyscan_global_data.is_all_key_released)
    {
        keyscan_init_driver(KeyScan_Debounce_Enable);
    }
}


/******************************************************************
 * @fn       keyscan_init_driver
 * @brief    keyscan module initial
 *
 * @param    uint32_t is_debounce
 * @return   void
 * @note     when system in dlsp mode, keyscan debunce time should be smaller
 *           than wake up interval time (like ble interval), which can be modified
 *           through KeyScan_InitStruct.debouncecnt.
 */
void keyscan_init_driver(uint32_t is_debounce)
{
    RCC_PeriphClockCmd(APBPeriph_KEYSCAN, APBPeriph_KEYSCAN_CLOCK, DISABLE);
    /* turn on keyscan clock */
    RCC_PeriphClockCmd(APBPeriph_KEYSCAN, APBPeriph_KEYSCAN_CLOCK, ENABLE);

    keyscan_global_data.is_allowed_to_repeat_report = false;
    KEYSCAN_InitTypeDef  KeyScan_InitStruct;
    KeyScan_StructInit(&KeyScan_InitStruct);
    KeyScan_InitStruct.colSize         = KEYPAD_COLUMN_SIZE;
    KeyScan_InitStruct.rowSize         = KEYPAD_ROW_SIZE;
    KeyScan_InitStruct.scanmode        = KeyScan_Manual_Scan_Mode;

    KeyScan_InitStruct.clockdiv         = 0x26;  /* 128kHz = 5MHz/(clockdiv+1) */
    KeyScan_InitStruct.delayclk         = 0x0f;  /* 8kHz = 5MHz/(clockdiv+1)/(delayclk+1) */

    KeyScan_InitStruct.debounceEn       = is_debounce;
    KeyScan_InitStruct.scantimerEn      = KeyScan_ScanInterval_Disable;//KeyScan_ScanInterval_Enable;
    KeyScan_InitStruct.detecttimerEn    = KeyScan_Release_Detect_Disable;

    KeyScan_InitStruct.scanInterval     = 0x190;  /* 50ms = scanInterval/8kHz */
    KeyScan_InitStruct.debouncecnt      = 0x40;   //8ms = debouncecnt/8kHz
    KeyScan_InitStruct.releasecnt       = 0x01;   //0.125ms = releasecnt/8kHz

    KeyScan_Init(KEYSCAN, &KeyScan_InitStruct);
    KeyScan_INTConfig(KEYSCAN, KEYSCAN_INT_SCAN_END, ENABLE);
    KeyScan_ClearINTPendingBit(KEYSCAN, KEYSCAN_INT_SCAN_END);
    KeyScan_INTMask(KEYSCAN, KEYSCAN_INT_SCAN_END, DISABLE);  /* Mask keyscan interrupt */
    KeyScan_Cmd(KEYSCAN, ENABLE);
}

/******************************************************************
 * @brief    keyscan nvic config
 */
void keyscan_nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;

    NVIC_InitStruct.NVIC_IRQChannel = KeyScan_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStruct);
}


/**
* @fn  keyscan_init_data
* @brief  Initialize Keyscan driver data
*/
void keyscan_init_data(void)
{
    APP_PRINT_INFO0("[keyscan_init_data] init data");
    memset(&keyscan_global_data, 0, sizeof(keyscan_global_data));
    keyscan_global_data.is_allowed_to_enter_dlps = true;
    keyscan_global_data.is_all_key_released = true;
}


/******************************************************************
 * @brief    keyscan interrupt handler
 *
 * If use Manual Scan Mode, it shall clear KEYSCAN_INT_SCAN_END to start next keyscan,
 * otherwise, keyscan will not start.
 */
void keyscan_interrupt_handler(void)
{
    APP_PRINT_INFO0("[keyscan_interrupt_handler] interrupt handler");

    T_IO_MSG bee_io_msg;

    if (KeyScan_GetFlagState(KEYSCAN, KEYSCAN_INT_FLAG_SCAN_END) == SET)
    {
        keyscan_global_data.is_allowed_to_enter_dlps = true;
        KeyScan_INTMask(KEYSCAN, KEYSCAN_INT_SCAN_END, ENABLE);  /* Mask keyscan interrupt */

        keyscan_global_data.cur_fifo_data.len = KeyScan_GetFifoDataNum(KEYSCAN);
        if (keyscan_global_data.cur_fifo_data.len != 0)
        {
            /* read keyscan fifo data */
            KeyScan_Read(KEYSCAN, (uint16_t *) & (keyscan_global_data.cur_fifo_data.key[0]),
                         keyscan_global_data.cur_fifo_data.len);
            keyscan_global_data.is_key_pressed = true;
            keyscan_global_data.is_all_key_released = false;

            /* start sw timer to check press status */
            if (!os_timer_restart(&keyscan_timer, KEYSCAN_SW_INTERVAL))
            {
                APP_PRINT_ERROR0("[keyscan_interrupt_handler] restart xTimersKeyScan failed!");
                /* set flag to default status and reinit keyscan module with debounce enabled */
                keyscan_init_data();
                keyscan_init_driver(KeyScan_Debounce_Enable);
                return;
            }

            if (false == keyscan_global_data.is_allowed_to_repeat_report)
            {
                if (!memcmp(&keyscan_global_data.cur_fifo_data, &keyscan_global_data.pre_fifo_data,
                            sizeof(T_KEYSCAN_FIFIO_DATA)))
                {
                    /* some keyscan FIFO data, just return */
                    return;
                }
                else
                {
                    /* updata previous keyscan FIFO data */
                    memcpy(&keyscan_global_data.pre_fifo_data, &keyscan_global_data.cur_fifo_data,
                           sizeof(T_KEYSCAN_FIFIO_DATA));
                }
            }

            bee_io_msg.type = IO_MSG_TYPE_KEYSCAN;
            bee_io_msg.subtype = IO_MSG_KEYSCAN_RX_PKT;
            bee_io_msg.u.buf   = (void *)(&keyscan_global_data.pre_fifo_data);
            if (false == app_send_msg_to_apptask(&bee_io_msg))
            {
                APP_PRINT_ERROR0("[keyscan_interrupt_handler] send IO_MSG_KEYSCAN_RX_PKT message failed!");
                /* set flag to default status and reinit keyscan module with debounce enabled */
                keyscan_init_data();
                os_timer_stop(&keyscan_timer);
                keyscan_init_driver(KeyScan_Debounce_Enable);
                return;
            }
        }
        else
        {
            if (false == keyscan_global_data.is_all_key_released)
            {
                /* keyscan release event detected */
                APP_PRINT_INFO0("[keyscan_interrupt_handler] keyscan release event detected");
                T_IO_MSG bee_io_msg;
                bee_io_msg.type = IO_MSG_TYPE_KEYSCAN;
                bee_io_msg.subtype = IO_MSG_KEYSCAN_ALLKEYRELEASE;

                if (false == app_send_msg_to_apptask(&bee_io_msg))
                {
                    APP_PRINT_ERROR0("[keyscan_interrupt_handler] Send IO_MSG_TYPE_KEYSCAN message failed!");
                }

                keyscan_init_data();
                keyscan_init_driver(KeyScan_Debounce_Enable);
            }
            else
            {
                /*if system active, keyscan no debounce can arrive here*/
                APP_PRINT_INFO0("[keyscan_interrupt_handler] if system active, keyscan no debounce can arrive here");
                keyscan_init_data();
                keyscan_init_driver(KeyScan_Debounce_Enable);
                return;
            }
        }
    }
    else
    {
        /* if not KEYSCAN_INT_FLAG_SCAN_END interrupt */
        APP_PRINT_INFO0("[keyscan_interrupt_handler] not KEYSCAN_INT_FLAG_SCAN_END interrupt");
        keyscan_init_data();
        keyscan_init_driver(KeyScan_Debounce_Enable);
        return;
    }
}


/******************************************************************
 * @brief    keyscan init timer
 */
void keyscan_init_timer(void)
{
    APP_PRINT_INFO0("[keyscan_init_timer] init timer");
    /*xTimersKeyScan is used for keyscan dlps*/
    if (false == os_timer_create(&keyscan_timer, "keyscan_timer",  1, \
                                 KEYSCAN_SW_INTERVAL, false, keyscan_timer_callback))
    {
        APP_PRINT_ERROR0("[keyscan_init_timer] timer creat failed!");
    }
}

/******************************************************************
 * @brief    keyscan timer callback
 */
void keyscan_timer_callback(void *pxTimer)
{
    if (keyscan_global_data.is_key_pressed == true)
    {
        APP_PRINT_INFO0("[keyscan_timer_callback] start release timer");
        keyscan_global_data.is_key_pressed = false;
        keyscan_global_data.is_allowed_to_enter_dlps = false;
        keyscan_init_driver(KeyScan_Debounce_Disable);
        /* start timer to check key status */
        os_timer_restart(&pxTimer, KEYSCAN_SW_RELEASE_TIMEOUT);
    }
    else
    {
        /* keyscan release event detected */
        APP_PRINT_INFO0("[keyscan_timer_callback] keyscan release event detected");
        T_IO_MSG bee_io_msg;
        bee_io_msg.type = IO_MSG_TYPE_KEYSCAN;
        bee_io_msg.subtype = IO_MSG_KEYSCAN_ALLKEYRELEASE;

        if (false == app_send_msg_to_apptask(&bee_io_msg))
        {
            APP_PRINT_ERROR0("[keyscan_timer_callback] Send IO_MSG_TYPE_KEYSCAN message failed!");
        }

        keyscan_init_data();
        keyscan_init_driver(KeyScan_Debounce_Enable);
    }
}

/**
* @brief   keyscan wakeup handler
* @return  void
*/
void handle_keyscan_wakeup(void)
{

    Pinmux_Config(ROW0, DWGPIO);
    Pinmux_Config(ROW1, DWGPIO);
    Pinmux_Config(ROW2, DWGPIO);
    Pinmux_Config(ROW3, DWGPIO);

    Pad_Config(ROW0, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(ROW1, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(ROW2, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(ROW3, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);

    uint32_t gpio_data = 0;
    uint32_t gpio_pin = 0;
    gpio_data = GPIO_ReadInputData();
    gpio_pin = GPIO_GetPin(ROW0) | GPIO_GetPin(ROW1) | GPIO_GetPin(ROW2) | GPIO_GetPin(ROW3);

    keyscan_pinmux_config();
    keyscan_pad_config();

    if (gpio_pin != (gpio_pin & gpio_data))
    {
        APP_PRINT_INFO2("wake up by keyscan, pin =0x%x, gpio input = 0x%x", gpio_pin, gpio_data);
        keyscan_init_driver(KeyScan_Debounce_Disable);
        keyscan_global_data.is_allowed_to_enter_dlps = false;
        /* not allow to enter DLPS before interrupt or timeout */
        /* Set flag and start timer to handle unexpection condition:
           1. Not Keyscan row PAD wake up system;
           2. PAD wake up signal disappear before Keyscan works; */
        keyscan_global_data.is_key_pressed = true;
        if (!os_timer_restart(&keyscan_timer, KEYSCAN_SW_INTERVAL))
        {
            APP_PRINT_ERROR0("[System_Handler] xTimersKeyScan restart failed!");
            keyscan_global_data.is_allowed_to_enter_dlps = true;
        }
    }




}
