#include "iohandle.h"
#include <trace.h>
#include "rtl876x_pinmux.h"
#include "rtl876x_io_dlps.h"
#include "rtl876x_rcc.h"
#include "rtl876x_gpio.h"
#include "rtl876x_nvic.h"
#include "ias_client.h"
#include "kns.h"
#include <os_timer.h>
#include <link_mgr.h>
#include <gap_adv.h>
#include <gap_scan.h>
#include "gap_conn_le.h"
#include "pxp_smart_app.h"
#include "board.h"
#include "app_task.h"

TimerHandle_t xTimerPxpMIO;
TimerHandle_t xTimerPxpSIO;
TimerHandle_t xTimerMkLongPress;
TimerHandle_t xTimerSkLongPress;

uint32_t xMPeriodLow  = 0;
uint32_t xMPeriodHigh = 0;
uint8_t gMIoMode = 0;
uint32_t gMActCnt = 0;

uint32_t xSPeriodLow  = 0;
uint32_t xSPeriodHigh = 0;
uint8_t gSIoMode = 0;
uint32_t gSActCnt = 0;

uint8_t keyMstatus;
uint8_t keySstatus;
bool gMBeepflg = false;
bool gSBeepflg = false;

void PINMUX_Configuration(void)
{
    Pinmux_Config(LED_M, DWGPIO);
    Pinmux_Config(LED_S, DWGPIO);
    Pinmux_Config(BEEP, DWGPIO);
    Pinmux_Config(KEY_M, DWGPIO);
    Pinmux_Config(KEY_S, DWGPIO);
    return;
}

void PAD_Configuration(void)
{
    /* Keypad pad config */
    Pad_Config(LED_M, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(LED_S, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);
    Pad_Config(BEEP, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_NONE, PAD_OUT_ENABLE, PAD_OUT_LOW);

    Pad_Config(KEY_M, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(KEY_S, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    return;
}

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
    Gpio_Struct.GPIO_Pin = GPIO_GetPin(LED_M) | GPIO_GetPin(LED_S) | GPIO_GetPin(BEEP);
    Gpio_Struct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(&Gpio_Struct);
    GPIO_ResetBits(GPIO_GetPin(LED_M) | GPIO_GetPin(LED_S) | GPIO_GetPin(BEEP));

    Gpio_Struct.GPIO_Pin = GPIO_GetPin(KEY_M) | GPIO_GetPin(KEY_S);
    Gpio_Struct.GPIO_Mode = GPIO_Mode_IN;
    Gpio_Struct.GPIO_ITCmd = ENABLE;
    Gpio_Struct.GPIO_ITTrigger = GPIO_INT_Trigger_EDGE;
    Gpio_Struct.GPIO_ITPolarity = GPIO_INT_POLARITY_ACTIVE_LOW;
    Gpio_Struct.GPIO_ITDebounce = GPIO_INT_DEBOUNCE_ENABLE;
    Gpio_Struct.GPIO_DebounceTime = 20;
    GPIO_Init(&Gpio_Struct);

    keySstatus = GPIO_ReadInputDataBit(GPIO_GetPin(KEY_S));
    keyMstatus = GPIO_ReadInputDataBit(GPIO_GetPin(KEY_M));
    GPIO_MaskINTConfig(GPIO_GetPin(KEY_M) | GPIO_GetPin(KEY_S), DISABLE);
    GPIO_INTConfig(GPIO_GetPin(KEY_M) | GPIO_GetPin(KEY_S), ENABLE);


    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = KEY_S_IRQ;//P2_4
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = KEY_M_IRQ;//P2_3
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
KeyStatus gKeySStatus = keyIdle;
KeyStatus gKeyMStatus = keyIdle;
//void gap_send_msg_to_app(T_IO_MSG *p_msg);
//#define task_send_msg_to_app gap_send_msg_to_app
//KEY_MASTER
/**
 * @brief    KEY_M_INT_Handle(Gpio19IntrHandler) for key interrupt
 * @note     Send short press event when read correct gpio bit port
 * @return   void
 */
void KEY_M_INT_Handle(void)
{
    T_IO_MSG bee_io_msg;
    APP_PRINT_INFO0("Enter GPIO19_Handler!");
    GPIO_MaskINTConfig(GPIO_GetPin(KEY_M), ENABLE);
    keyMstatus = GPIO_ReadInputDataBit(GPIO_GetPin(KEY_M));

    if (keyMstatus == 0)
    {
        gKeyMStatus = keyShortPress;
        GPIO->INTPOLARITY |= GPIO_GetPin(KEY_M);
        os_timer_start(&xTimerMkLongPress);
    }
    else
    {
        GPIO->INTPOLARITY &= ~GPIO_GetPin(KEY_M);
        if (gKeyMStatus == keyShortPress)
        {
            os_timer_stop(&xTimerMkLongPress);
            bee_io_msg.type = IO_MSG_TYPE_GPIO;
            bee_io_msg.subtype = MSG_KEYM_SHORT_PRESS;
            app_send_msg_to_apptask(&bee_io_msg);
        }
        gKeyMStatus = keyIdle;
    }
    GPIO_ClearINTPendingBit(GPIO_GetPin(KEY_M));
    GPIO_MaskINTConfig(GPIO_GetPin(KEY_M), DISABLE);
}

//KEY_SLAVE
/**
 * @brief    KEY_S_INT_Handle(Gpio20IntrHandler) for key interrupt
 * @note     Send short press event when read correct gpio bit port
 * @return   void
 */
void KEY_S_INT_Handle(void)
{
    T_IO_MSG bee_io_msg;
    APP_PRINT_ERROR0("Enter GPIO20_Handler!");
    GPIO_MaskINTConfig(GPIO_GetPin(KEY_S), ENABLE);
    keySstatus = GPIO_ReadInputDataBit(GPIO_GetPin(KEY_S));

    if (keySstatus == 0)
    {
        gKeySStatus = keyShortPress;
        GPIO->INTPOLARITY |= GPIO_GetPin(KEY_S);
        os_timer_start(&xTimerSkLongPress);
    }
    else
    {
        GPIO->INTPOLARITY &= ~GPIO_GetPin(KEY_S);
        if (gKeySStatus == keyShortPress)
        {
            os_timer_stop(&xTimerSkLongPress);
            bee_io_msg.type = IO_MSG_TYPE_GPIO;
            bee_io_msg.subtype = MSG_KEYS_SHORT_PRESS;
            app_send_msg_to_apptask(&bee_io_msg);
        }
        gKeySStatus = keyIdle;
    }
    GPIO_ClearINTPendingBit(GPIO_GetPin(KEY_S));
    GPIO_MaskINTConfig(GPIO_GetPin(KEY_S), DISABLE);
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
    uint8_t i;
    if (keytype == MSG_KEYS_SHORT_PRESS) //salve short press
    {
        switch (gPxpState)
        {
        case PxpStateIdle:
            if (gSIoState == IoStateIdle)
            {
                StartPxpSIO(ALERT_LOW_PERIOD, ALERT_HIGH_PERIOD, LED_BLINK,
                            1); /*low period 0.9s, high period 0.1s,  led blink,  1times(cnt)*/
            }
            break;
        case PxpStateAdv:
            if (gSIoState == IoStateLlsAlert) // close beep
            {
                StopPxpSIO();
                gSIoState = IoStateAdvScanBlink;
                StartPxpSIO(ALERT_LOW_PERIOD * 4, ALERT_HIGH_PERIOD, LED_BLINK,
                            0xffffffff);
            }
            break;
        case PxpStateLinkS: //link as slave
        case PxpStateLinkSScan: //link as slave
        case PxpStateLinkMS: //link as slave
            if (gSIoState == IoStateImmAlert)
            {
                StopPxpSIO();
                gSIoState = IoStateLinkBlink;
                StartPxpSIO(ALERT_LOW_PERIOD * 4, ALERT_HIGH_PERIOD, LED_BLINK,
                            0xffffffff);
            }
            else
            {
                value_to_send = 1;
                for (i = 0; i < APP_MAX_LINKS; i++)
                {
                    if (PXPLINK[i] == SlaveLink)
                        server_send_data(i, kns_srv_id, KNS_KEY_VALUE_INDEX, \
                                         &value_to_send, sizeof(uint8_t), GATT_PDU_TYPE_NOTIFICATION);
                    break;
                }

            }
            break;
        default:
            break;
        }
    }
    else if (keytype == MSG_KEYS_LONG_PRESS)
    {
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
//                StartPxpSIO(ALERT_LOW_PERIOD, ALERT_HIGH_PERIOD, LED_BLINK,
//                           10); /*low period 0.9s, high period 0.1s,  led blink,  10times(cnt)*/
//                gIoState = IoStateAdvBlink;
                le_adv_stop();
            }
            else
            {
                APP_PRINT_ERROR0("ERROR POWER STATUS");//error status
            }
            break;
        case PxpStateScan:
            le_adv_start();
            break;
        case PxpStateAdvScan:
            le_adv_stop();
            break;
        case PxpStateLinkM:
            le_adv_start();
            break;
        case PxpStateLinkS://link as slave
            if (gPowerFlg == true)
            {
                gPowerFlg = false;
                for (i = 0; i < APP_MAX_LINKS; i++)
                {
                    if (PXPLINK[i] == SlaveLink)
                    {
                        le_disconnect(i);
                    }
                    break;
                }
            }
            else
            {
                APP_PRINT_ERROR0("ERROR POWER STATUS");//error status
            }
            break;
        case PxpStateLinkMADV:
            le_adv_stop();
            break;
        case PxpStateLinkSScan:
        case PxpStateLinkMS:
            for (i = 0; i < APP_MAX_LINKS; i++)
            {
                if (PXPLINK[i] == SlaveLink)
                {
                    le_disconnect(i);
                }
                break;
            }
            break;
        default:
            break;
        }
    }
    else if (keytype == MSG_KEYM_SHORT_PRESS)
    {
        switch (gPxpState)
        {
        case PxpStateIdle:
            if (gMIoState == IoStateIdle)
            {
                StartPxpMIO(ALERT_LOW_PERIOD, ALERT_HIGH_PERIOD, LED_BLINK,
                            1); /*low period 0.9s, high period 0.1s,  led blink,  1times(cnt)*/
            }
            break;
        case PxpStateScan:
            if (gMIoState == IoStateLlsAlert) // close beep
            {
                StopPxpMIO();
                gMIoState = IoStateAdvScanBlink;
                StartPxpMIO(ALERT_LOW_PERIOD * 2, ALERT_HIGH_PERIOD, LED_BLINK,
                            0xffffffff);
            }
            break;
        case PxpStateLinkM:
        case PxpStateLinkMADV:
        case PxpStateLinkMS:
            if (gMIoState == IoStateImmAlert)
            {
                StopPxpMIO();
                gMIoState = IoStateLinkBlink;
                StartPxpMIO(ALERT_LOW_PERIOD * 4, ALERT_HIGH_PERIOD, LED_BLINK,
                            0xffffffff);
            }
            else
            {
                uint8_t alertVal = 2;
                for (i = 0; i < APP_MAX_LINKS; i++)
                {
                    if (PXPLINK[i] == MasterLink)
                    {
                        ias_client_write_char(i, sizeof(uint8_t), &alertVal, GATT_WRITE_TYPE_CMD);
                    }
                    break;
                }
            }
            break;
        default:
            break;
        }
    }
    else if (keytype == MSG_KEYM_LONG_PRESS)
    {
        switch (gPxpState)
        {
        case PxpStateIdle:
            if (gPowerFlg == false)
            {
                gPowerFlg = true;
                link_mgr_clear_device_list();
                le_scan_start();
            }
            else
            {
                APP_PRINT_ERROR0("ERROR POWER STATUS");//error status
            }
            break;
        case PxpStateAdv:
        case PxpStateLinkS:
            le_scan_start();
            break;
        case PxpStateScan:
            if (gPowerFlg == true)
            {
                gPowerFlg = false;
                le_scan_stop();
            }
            else
            {
                APP_PRINT_ERROR0("ERROR POWER STATUS");//error status
            }
            break;
        case PxpStateAdvScan:
            le_scan_stop();
            break;
        case PxpStateLinkSScan:
            le_scan_stop();
            break;
        case PxpStateLinkM://link as master
            if (gPowerFlg == true)
            {
                gPowerFlg = false;
                for (i = 0; i < APP_MAX_LINKS; i++)
                {
                    if (PXPLINK[i] == MasterLink)
                    {
                        le_disconnect(i);
                    }
                    break;
                }
            }
            else
            {
                APP_PRINT_ERROR0("ERROR POWER STATUS");//error status
            }
            break;
        case PxpStateLinkMADV:
            for (i = 0; i < APP_MAX_LINKS; i++)
            {
                if (PXPLINK[i] == MasterLink)
                {
                    le_disconnect(i);
                }
                break;
            }
            break;
        case PxpStateLinkMS:
            for (i = 0; i < APP_MAX_LINKS; i++)
            {
                if (PXPLINK[i] == MasterLink)
                {
                    le_disconnect(i);
                }
                break;
            }
            break;
        }
    }
    else
    {
        //nothing to do
    }
}
/**
 * @brief    StartPxpMIO for led blink and beep
 * @note     Set parameter and start swtimer xTimerPxpMIO
 * @param[in] lowPeroid
 * @param[in] HighPeroid
 * @param[in] mode
 * @param[in] cnt
 * @return   void
 */
void StartPxpMIO(uint32_t lowPeroid, uint32_t HighPeroid, uint8_t mode, uint32_t cnt)
{
    xMPeriodLow = lowPeroid;
    xMPeriodHigh = HighPeroid;
    gMIoMode = mode;
    gMActCnt = cnt;
    if (gMIoMode)
    {
        if (gMIoMode & LED_BLINK)
        {
            GPIO_SetBits(GPIO_GetPin(LED_M));
        }
        if (gMIoMode & BEEP_ALERT)
        {
            if (gSBeepflg == false)
            {
                gMBeepflg = true;
                GPIO_SetBits(GPIO_GetPin(BEEP));
            }
        }
        allowedPxpEnterDlps = false;
        os_timer_restart(&xTimerPxpMIO, HighPeroid);
    }
}
void StopPxpMIO()
{
    allowedPxpEnterDlps = true;
    gMActCnt = 0;
    gMIoMode = 0;
    if ((gSIoState != IoStateImmAlert) && (gSIoState != IoStateLlsAlert))
    {
        GPIO_ResetBits(GPIO_GetPin(BEEP));
    }
    GPIO_ResetBits(GPIO_GetPin(LED_M));
    os_timer_stop(&xTimerPxpMIO);
}

/**
 * @brief    StartPxpSIO for led blink and beep
 * @note     Set parameter and start swtimer xTimerPxpSIO
 * @param[in] lowPeroid
 * @param[in] HighPeroid
 * @param[in] mode
 * @param[in] cnt
 * @return   void
 */
void StartPxpSIO(uint32_t lowPeroid, uint32_t HighPeroid, uint8_t mode, uint32_t cnt)
{
    xSPeriodLow = lowPeroid;
    xSPeriodHigh = HighPeroid;
    gSIoMode = mode;
    gSActCnt = cnt;
    if (gSIoMode)
    {
        if (gSIoMode & LED_BLINK)
        {
            GPIO_SetBits(GPIO_GetPin(LED_S));
        }
        if (gSIoMode & BEEP_ALERT)
        {
            if (gMBeepflg == false)
            {
                gSBeepflg = true;
                GPIO_SetBits(GPIO_GetPin(BEEP));
            }
        }
        allowedPxpEnterDlps = false;
        os_timer_restart(&xTimerPxpSIO, HighPeroid);
    }
}

/**
 * @brief    StopPxpSIO
 * @note     Stop pxp io action
 * @return   void
 */
void StopPxpSIO(void)
{
    allowedPxpEnterDlps = true;
    gSActCnt = 0;
    gSIoMode = 0;
    if ((gMIoState != IoStateImmAlert) && (gMIoState != IoStateLlsAlert))
    {
        GPIO_ResetBits(GPIO_GetPin(BEEP));
    }
    GPIO_ResetBits(GPIO_GetPin(LED_S));
    os_timer_stop(&xTimerPxpSIO);
}
/**
 * @brief    vTimerPxpMIOCallback
 * @note     BEEP & LED act here
 * @param[in] pxTimer
 * @return   void
 */
void vTimerPxpMIOCallback(TimerHandle_t pxTimer)
{
    uint8_t  status_value = 0;
    uint32_t xNewPeriod;

    if (gMIoMode & LED_BLINK)
    {
        status_value = GPIO_ReadOutputDataBit(GPIO_GetPin(LED_M));
    }
    else if (gMIoMode & BEEP_ALERT)
    {
        if (gSBeepflg == false)
        {
            gMBeepflg = true;
            status_value = GPIO_ReadOutputDataBit(GPIO_GetPin(BEEP));
        }
    }
    else
    {
        APP_PRINT_ERROR0("ERROR IO MODE");
        return;
    }

    if (status_value & 0x1)
    {
        xNewPeriod = xMPeriodLow;
        GPIO_ResetBits(GPIO_GetPin(LED_M));
        if (gSBeepflg == false)
        {
            GPIO_ResetBits(GPIO_GetPin(BEEP));
        }
        allowedPxpEnterDlps = true;
        if (gMActCnt != 0xffffffff)
        {
            gMActCnt--;
        }
    }
    else
    {
        xNewPeriod = xMPeriodHigh;
        if (gMIoMode & LED_BLINK)
        {
            GPIO_SetBits(GPIO_GetPin(LED_M));
            allowedPxpEnterDlps = false;
        }
        if (gMIoMode & BEEP_ALERT)
        {
            if (gSBeepflg == false)
            {
                GPIO_SetBits(GPIO_GetPin(BEEP));
                allowedPxpEnterDlps = false;
            }
        }
    }
    if (gMActCnt)
    {
        os_timer_restart(&xTimerPxpMIO, xNewPeriod);
    }
    else
    {
        //set new period for display mode
        gMBeepflg = false;
        if (gPxpState == PxpStateScan || gPxpState == PxpStateAdvScan || gPxpState == PxpStateLinkSScan)
        {
            gMIoState = IoStateAdvScanBlink;
            StartPxpMIO(ALERT_LOW_PERIOD * 2, ALERT_HIGH_PERIOD, LED_BLINK, 0xffffffff);
        }
        else if (gPxpState == PxpStateLinkM || gPxpState == PxpStateLinkMADV || gPxpState == PxpStateLinkMS)
        {
            gMIoState = IoStateLinkBlink;
            StartPxpMIO(ALERT_LOW_PERIOD * 4, ALERT_HIGH_PERIOD, LED_BLINK, 0xffffffff);
        }
        //gIoState = IoStateIdle;
    }
}

/**
 * @brief    vTimerPxpMIOCallback
 * @note     BEEP & LED act here
 * @param[in] pxTimer
 * @return   void
 */
void vTimerPxpSIOCallback(TimerHandle_t pxTimer)
{
    uint8_t  status_value = 0;
    uint32_t xNewPeriod;

    APP_PRINT_INFO1("SIoMode is %x", gSIoMode);

    if (gSIoMode & LED_BLINK)
    {
        status_value = GPIO_ReadOutputDataBit(GPIO_GetPin(LED_S));
    }
    else if (gSIoMode & BEEP_ALERT)
    {
        if (gMBeepflg == false)
        {
            gSBeepflg = true;
            status_value = GPIO_ReadOutputDataBit(GPIO_GetPin(BEEP));
        }
    }
    else
    {
        APP_PRINT_ERROR0("ERROR IO MODE");
        return;
    }

    if (status_value & 0x1)
    {
        xNewPeriod = xSPeriodLow;
        GPIO_ResetBits(GPIO_GetPin(LED_S));
        if (gMBeepflg == false)
        {
            GPIO_ResetBits(GPIO_GetPin(BEEP));
        }
        allowedPxpEnterDlps = true;
        if (gSActCnt != 0xffffffff)
        {
            gSActCnt--;
        }
    }
    else
    {
        xNewPeriod = xSPeriodHigh;
        if (gSIoMode & LED_BLINK)
        {
            GPIO_SetBits(GPIO_GetPin(LED_S));
            allowedPxpEnterDlps = false;
        }
        if (gSIoMode & BEEP_ALERT)
        {
            if (gMBeepflg == false)
            {
                GPIO_SetBits(GPIO_GetPin(BEEP));
                allowedPxpEnterDlps = false;
            }
        }
    }
    if (gSActCnt)
    {
        os_timer_restart(&xTimerPxpSIO, xNewPeriod);
    }
    else
    {
        //set new period for display mode
        gSBeepflg = false;

        if (gPxpState == PxpStateLinkS || gPxpState == PxpStateLinkSScan || gPxpState == PxpStateLinkMS)
        {
            gSIoState = IoStateAdvScanBlink;
            StartPxpSIO(ALERT_LOW_PERIOD * 4, ALERT_HIGH_PERIOD, LED_BLINK,
                        0xffffffff); /*low period 0.9s, high period 0.1s,  led blink,  10times(cnt)*/
        }
        //gIoState = IoStateIdle;
    }
}
/**
 * @brief    vTimerMkLongPressCallback
 * @note     if key still pressed , send long press message
 * @param[in] pxTimer
 * @return   void
 */
void vTimerMkLongPressCallback(TimerHandle_t pxTimer)
{
    T_IO_MSG bee_io_msg;

    gKeyMStatus = keyLongPress;

    keyMstatus = GPIO_ReadInputDataBit(GPIO_GetPin(KEY_M));
    if (keyMstatus == 0)
    {
        bee_io_msg.type = IO_MSG_TYPE_GPIO;
        bee_io_msg.subtype = MSG_KEYM_LONG_PRESS;
        app_send_msg_to_apptask(&bee_io_msg);
    }
}
/**
 * @brief    vTimerSkLongPressCallback
 * @note     if key still pressed , send long press message
 * @param[in] pxTimer
 * @return   void
 */
void vTimerSkLongPressCallback(TimerHandle_t pxTimer)
{
    T_IO_MSG bee_io_msg;

    gKeySStatus = keyLongPress;

    keySstatus = GPIO_ReadInputDataBit(GPIO_GetPin(KEY_S));
    if (keySstatus == 0)
    {
        bee_io_msg.type = IO_MSG_TYPE_GPIO;
        bee_io_msg.subtype = MSG_KEYS_LONG_PRESS;
        app_send_msg_to_apptask(&bee_io_msg);
    }
}

/**
 * @brief    swTimerInit
 * @note     creat sw timer
 * @return   void
 */
void swTimerInit()
{
    bool retval ;
    /* xTimersRmcPairBtn is used to start bt pair process after timeout */

    retval = os_timer_create(&xTimerPxpMIO, "xTimerPxpMIO",  1, \
                             100/*0.1s*/, false, vTimerPxpMIOCallback);
    if (!retval)
    {
        APP_PRINT_INFO1("xTimerPxpMIO retval is %d", retval);
    }
    retval = os_timer_create(&xTimerPxpSIO, "xTimerPxpSIO",  1, \
                             100/*0.1s*/, false, vTimerPxpSIOCallback);
    if (!retval)
    {
        APP_PRINT_INFO1("xTimerPxpSIO retval is %d", retval);
    }
    retval = os_timer_create(&xTimerMkLongPress, "xTimerMkLongPress",  1, \
                             4000/*4s*/, false, vTimerMkLongPressCallback);
    if (!retval)
    {
        APP_PRINT_INFO1("xTimerMkLongPress retval is %d", retval);
    }
    retval = os_timer_create(&xTimerSkLongPress, "xTimerSkLongPress",  1, \
                             4000/*4s*/, false, vTimerSkLongPressCallback);
    if (!retval)
    {
        APP_PRINT_INFO1("xTimerSkLongPress retval is %d", retval);
    }
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
