#include "iohandle.h"
#include <os_timer.h>
#include <os_msg.h>
#include <trace.h>
#include "app_task.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_io_dlps.h"
#include "rtl876x_rcc.h"
#include "rtl876x_gpio.h"
#include "rtl876x_nvic.h"
#include "kns.h"
#include "board.h"
#include "pxp_app.h"
#include <gap_adv.h>
#include "gap_conn_le.h"

#include "otp_config.h"

TimerHandle_t xTimerPxpIO;
TimerHandle_t xTimerLongPress;

#if (AON_WDG_ENABLE == 1)
void *xTimerPeriodWakeupDlps;
#define TIMER_WAKEUP_DLPS_PERIOD     ((AON_WDG_TIME_OUT_PERIOD_SECOND - 1) * 1000)
#endif

uint32_t xPeriodLow  = 0;
uint32_t xPeriodHigh = 0;
uint8_t gIoMode = 0;
uint32_t gActCnt = 0;

uint8_t keystatus;

/**
 * @brief    pinmux configuration
 * @return   void
 */
void PINMUX_Configuration(void)
{
    Pinmux_Config(LED, DWGPIO);
    Pinmux_Config(BEEP, DWGPIO);
    Pinmux_Config(KEY, DWGPIO);

    return;
}
/**
 * @brief    pad configuration
 * @return   void
 */
void PAD_Configuration(void)
{
    /* Keypad pad config */
    Pad_Config(LED, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(BEEP, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(KEY, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    return;
}
/**
 * @brief    rcc configuration
 * @return   void
 */
void RCC_Configuration(void)
{
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    return;
}

/**
 * @brief    Contains the initialization of peripherals
 * @note     Both new architecture driver and legacy driver initialization method can be used
 * @return   void
 */
void driver_init(void)
{
    GPIO_InitTypeDef Gpio_Struct;
    GPIO_StructInit(&Gpio_Struct);
    Gpio_Struct.GPIO_Pin = GPIO_GetPin(LED) | GPIO_GetPin(BEEP);
    Gpio_Struct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(&Gpio_Struct);
    GPIO_ResetBits(GPIO_GetPin(LED) | GPIO_GetPin(BEEP));

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

typedef enum _KeyStatus
{
    keyIdle = 0,
    keyShortPress,
    keyLongPress,
} KeyStatus;
KeyStatus gKeyStatus = keyIdle;


//void GPIO_Group0_Handler(void)
//{
//     APP_PRINT_ERROR0("Enter GPIO_Group0_Handler!");
//}
/**
* @brief    KEY_INT_Handle for key interrupt/GPIO20/P2_4
 * @note     Send short press event when read correct gpio bit port
 * @return   void
 */
//void GPIO5_Handler(void)
void KEY_INT_Handle(void)//P2_4
{

    T_IO_MSG bee_io_msg;
    APP_PRINT_ERROR0("Enter GPIO5_Handler!");
    GPIO_MaskINTConfig(GPIO_GetPin(KEY), ENABLE);
    keystatus = GPIO_ReadInputDataBit(GPIO_GetPin(KEY));

    if (keystatus == 0)
    {
        gKeyStatus = keyShortPress;
        GPIO->INTPOLARITY |= GPIO_GetPin(KEY);
        os_timer_start(&xTimerLongPress);
    }
    else
    {
        GPIO->INTPOLARITY &= ~GPIO_GetPin(KEY);
        if (gKeyStatus == keyShortPress)
        {
            os_timer_stop(&xTimerLongPress);
            bee_io_msg.type = IO_MSG_TYPE_GPIO;
            bee_io_msg.subtype = MSG_SHORT_PRESS;
            app_send_msg_to_apptask(&bee_io_msg);
        }
        gKeyStatus = keyIdle;
    }
    GPIO_ClearINTPendingBit(GPIO_GetPin(KEY));
    GPIO_MaskINTConfig(GPIO_GetPin(KEY), DISABLE);
}

/**
 * @brief    Pxp_HandleButtonEvent for key io event
 * @note     Event for Gpio interrupt and long press timer.
 * @param[in] io_msg
 * @return   void
 */
void Pxp_HandleButtonEvent(T_IO_MSG io_msg)
{
    uint8_t value_to_send;
    uint8_t keytype = io_msg.subtype ;
    if (keytype == MSG_SHORT_PRESS)
    {
        APP_PRINT_ERROR0("MSG_SHORT_PRESS");
        switch (gPxpState)
        {
        case PxpStateIdle:
            if (gIoState == IoStateIdle)
            {
                StartPxpIO(ALERT_LOW_PERIOD, ALERT_HIGH_PERIOD, LED_BLINK,
                           1); /*low period 0.9s, high period 0.1s,  led blink,  1times(cnt)*/
            }
            break;
        case PxpStateAdv:
            if (gIoState == IoStateLlsAlert)
            {
                gActCnt = 0;
                allowedPxpEnterDlps = true;
                os_timer_stop(&xTimerPxpIO);
                gIoState = IoStateIdle;
            }
            break;
        case PxpStateLink:
            if (gIoState == IoStateImmAlert)
            {
                gActCnt = 0;
                allowedPxpEnterDlps = true;
                os_timer_stop(&xTimerPxpIO);
                gIoState = IoStateIdle;
            }
            else if (gIoState == IoStateIdle)
            {
                value_to_send = 1;
                server_send_data(0, kns_srv_id, KNS_KEY_VALUE_INDEX, \
                                 &value_to_send, sizeof(uint8_t), GATT_PDU_TYPE_NOTIFICATION);
            }
            else
            {
                APP_PRINT_ERROR0("ERROR IO STATUS");//error status
            }
            break;
        default:
            break;
        }
    }
    else if (keytype == MSG_LONG_PRESS)
    {
        APP_PRINT_ERROR0("MSG_LONG_PRESS");
        switch (gPxpState)
        {
        case PxpStateIdle:
            if (gPowerFlg == false)
            {
                gPowerFlg = true;
                le_adv_start();
            }
            else
            {
                APP_PRINT_ERROR0("ERROR POWER STATUS");//error status
            }
            break;
        case PxpStateAdv:
            if (gPowerFlg == true)
            {
                gPowerFlg = false;
                if (gIoState != IoStateIdle)
                {
                    gIoState = IoStateIdle;
                    StopPxpIO();
                }
                le_adv_stop();
            }
            else
            {
                APP_PRINT_ERROR0("ERROR POWER STATUS");//error status
            }
            break;
        case PxpStateLink:
            if (gPowerFlg == true)
            {
                gPowerFlg = false;
                if (gIoState != IoStateIdle)
                {
                    gIoState = IoStateIdle;
                    StopPxpIO();
                }
                le_disconnect(0);
            }
            else
            {
                APP_PRINT_ERROR0("ERROR POWER STATUS");//error status
            }
            break;
        default:
            break;
        }
    }
    else
    {
        //nothing to do
    }
}

/**
 * @brief    StartPxpIO for led blink and beep
 * @note     Set parameter and start swtimer xTimerPxpIO
 * @param[in] lowPeroid
 * @param[in] HighPeroid
 * @param[in] mode
 * @param[in] cnt
 * @return   void
 */
void StartPxpIO(uint32_t lowPeroid, uint32_t HighPeroid, uint8_t mode, uint32_t cnt)
{
    xPeriodLow = lowPeroid;
    xPeriodHigh = HighPeroid;
    gIoMode = mode;
    gActCnt = cnt;
    if (gIoMode)
    {
        if (gIoMode & 0x1)
        {
            GPIO_SetBits(GPIO_GetPin(LED));
        }
        if (gIoMode & 0x2)
        {
            GPIO_SetBits(GPIO_GetPin(BEEP));
        }
        allowedPxpEnterDlps = false;
        os_timer_restart(&xTimerPxpIO, HighPeroid);
    }
}
/**
 * @brief    StopPxpIO
 * @note     Stop pxp io action
 * @return   void
 */
void StopPxpIO()
{
    allowedPxpEnterDlps = true;
    gActCnt = 0;
    GPIO_ResetBits(GPIO_GetPin(LED));
    GPIO_ResetBits(GPIO_GetPin(BEEP));
    os_timer_stop(&xTimerPxpIO);
}

/**
 * @brief    vTimerPxpIOCallback
 * @note     BEEP & LED act here
 * @param[in] pxTimer
 * @return   void
 */
void vTimerPxpIOCallback(TimerHandle_t pxTimer)
{
    uint8_t  status_value;
    uint32_t xNewPeriod;
    if (gIoMode & 0x1)
    {
        status_value = GPIO_ReadOutputDataBit(GPIO_GetPin(LED));
    }
    else if (gIoMode & 0x2)
    {
        status_value = GPIO_ReadOutputDataBit(GPIO_GetPin(BEEP));
    }
    else
    {
        APP_PRINT_ERROR0("ERROR IO MODE");
        return;
    }

    if (status_value & 0x1)
    {
        xNewPeriod = xPeriodLow;
        GPIO_ResetBits(GPIO_GetPin(LED));
        GPIO_ResetBits(GPIO_GetPin(BEEP));
        allowedPxpEnterDlps = true;
        gActCnt--;
    }
    else
    {
        xNewPeriod = xPeriodHigh;
        if (gIoMode & 0x1)
        {
            GPIO_SetBits(GPIO_GetPin(LED));
            allowedPxpEnterDlps = false;
        }
        if (gIoMode & 0x2)
        {
            GPIO_SetBits(GPIO_GetPin(BEEP));
            allowedPxpEnterDlps = false;
        }
    }
    if (gActCnt)
    {
        APP_PRINT_INFO0("xTimerPxpIO os_timer_restart.");
        os_timer_restart(&xTimerPxpIO, xNewPeriod);
    }
    else
    {
        gIoState = IoStateIdle;
    }
}

/**
 * @brief    vTimerLongPressCallback
 * @note     if key still pressed , send long press message
 * @param[in] pxTimer
 * @return   void
 */
void vTimerLongPressCallback(TimerHandle_t pxTimer)
{
    T_IO_MSG bee_io_msg;

    gKeyStatus = keyLongPress;

    keystatus = GPIO_ReadInputDataBit(GPIO_GetPin(KEY));
    if (keystatus == 0)
    {
        bee_io_msg.type = IO_MSG_TYPE_GPIO;
        bee_io_msg.subtype = MSG_LONG_PRESS;
        app_send_msg_to_apptask(&bee_io_msg);
    }
}

#if (AON_WDG_ENABLE == 1)
void vTimerPeriodWakeupDlpsCallback(TimerHandle_t pxTimer)
{
    APP_PRINT_INFO0("TimerPeriodWakeupDlps timeout!");
}
#endif

/**
 * @brief    swTimerInit
 * @note     creat sw timer
 * @return   void
 */
void swTimerInit()
{
    bool retval ;
    /* xTimersRmcPairBtn is used to start bt pair process after timeout */

    retval = os_timer_create(&xTimerPxpIO, "xTimerPxpIO",  1, \
                             100/*0.1s*/, false, vTimerPxpIOCallback);
    if (!retval)
    {
        APP_PRINT_INFO1("xTimerAlert retval is %d", retval);
    }
    retval = os_timer_create(&xTimerLongPress, "xTimerLongPress",  1, \
                             4000/*4s*/, false, vTimerLongPressCallback);
    if (!retval)
    {
        APP_PRINT_INFO1("xTimerLongPress retval is %d", retval);
    }

#if (AON_WDG_ENABLE == 1)
    retval = os_timer_create(&xTimerPeriodWakeupDlps, "xTimerPeriodWakeupDlps",  1, \
                             TIMER_WAKEUP_DLPS_PERIOD, true, vTimerPeriodWakeupDlpsCallback);
    if (!retval)
    {
        APP_PRINT_INFO1("xTimerPeriodWakeupDlps retval is %d", retval);
    }
    else
    {
        os_timer_start(&xTimerPeriodWakeupDlps);
        APP_PRINT_INFO0("Start xTimerPeriodWakeupDlps!");;
    }
#endif
}
/**
 * @brief change peripheral interval and latency function
 * @param interval - connection interval;
 * @param latency - peripheral connection latency;
 * @param timeout - supervision time out;
 * @return none
 */
void ChangeConnectionParameter(uint16_t interval, uint16_t latency, uint16_t timeout)
{
    le_update_conn_param(0, interval, interval, latency, timeout / 10, interval * 2 - 2,
                         interval * 2 - 2);
}
