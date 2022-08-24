#include "mxchip_qc.h"
#include "app_msg.h"
#include "gap_adv.h"
#include "gap_scan.h"
#include "os_sched.h"
#include "rtl876x.h"
#include "rtl876x_gpio.h"
#include "rtl876x_pinmux.h"
#include "rtl876x_rcc.h"
#include "rtl876x_uart.h"
#include "test_mode.h"
#include "version.h"
#include <os_msg.h>
#include <os_task.h>
#include <stdio.h>
#include <string.h>

#define MAC_STR_FORMAT "%02X-%02X-%02X-%02X-%02X-%02X"
#define BT_MAC_STR_FORMAT "%02X:%02X:%02X:%02X:%02X:%02X"

#define APP_TASK_PRIORITY 1                                                                //!< Task priorities
#define APP_TASK_STACK_SIZE 256 * 4                                                        //!<  Task stack size
#define MAX_NUMBER_OF_GAP_MESSAGE 0x20                                                     //!<  GAP message queue size
#define MAX_NUMBER_OF_IO_MESSAGE 0x20                                                      //!<  IO message queue size
#define MAX_NUMBER_OF_EVENT_MESSAGE (MAX_NUMBER_OF_GAP_MESSAGE + MAX_NUMBER_OF_IO_MESSAGE) //!< Event message queue size

/*============================================================================*
 *                              Variables
 *============================================================================*/
static void *qc_app_task_handle, *mxchp_qc_task; //!< APP Task handle
static void *qc_evt_queue_handle;                //!< Event queue handle
static void *qc_io_queue_handle;                 //!< IO queue handle

/*============================================================================*
 *                              Functions
 *============================================================================*/
void qc_app_main_task(void *p_param);

/**
 * @brief  Initialize App task
 * @return void
 */
void qc_app_task_init()
{
    os_task_create(&qc_app_task_handle, "qc_app", qc_app_main_task, 0, APP_TASK_STACK_SIZE,
                   APP_TASK_PRIORITY);
}

/**
 * @brief        App task to handle events & messages
 * @param[in]    p_param    Parameters sending to the task
 * @return       void
 */
void qc_app_main_task(void *p_param)
{
    uint8_t event;
    os_msg_queue_create(&qc_io_queue_handle, MAX_NUMBER_OF_IO_MESSAGE, sizeof(T_IO_MSG));
    os_msg_queue_create(&qc_evt_queue_handle, MAX_NUMBER_OF_EVENT_MESSAGE, sizeof(uint8_t));

    gap_start_bt_stack(qc_evt_queue_handle, qc_io_queue_handle, MAX_NUMBER_OF_GAP_MESSAGE);

    while (true)
    {
        if (os_msg_recv(qc_evt_queue_handle, &event, 0xFFFFFFFF) == true)
        {
            if (event == EVENT_GAP_MSG)
            {
                gap_handle_msg(event);
            }
        }
    }
}

static void _qc_uart_send(uint8_t *srt, uint8_t len)
{
    uint8_t count = 0, remainder = 0, i = 0;

    count = len / 16;
    remainder = len % 16;
    for (i = 0; i < count; i++)
    {
        UART_SendData(UART, &srt[16 * i], 16);
        while (UART_GetFlagState(UART, UART_FLAG_THR_TSR_EMPTY) != SET)
            ;
    }
    /* send left bytes */
    UART_SendData(UART, &srt[16 * i], remainder);
    /* wait tx fifo empty */
    while (UART_GetFlagState(UART, UART_FLAG_THR_TSR_EMPTY) != SET)
        ;
}

void mf_printf(char *str)
{
    if ((str == NULL) || (strlen(str) == 0))
        return;
    _qc_uart_send((uint8_t *)str, strlen(str));
}

void mf_putc(char ch)
{
    UART_SendByte(UART, (uint8_t)ch);
}

void _qc_uart_io_init(void)
{
    Pinmux_Config(UART_TX, UART0_TX);
    Pinmux_Config(UART_RX, UART0_RX);

    Pad_Config(UART_TX, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(UART_RX, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
}

void _qc_uart_config(void)
{
    UART_InitTypeDef UART_InitStruct;
    UART_StructInit(&UART_InitStruct);
    //    UART_InitStruct.div = 4; //921600
    //    UART_InitStruct.ovsr = 5;
    //    UART_InitStruct.ovsr_adj = 0x3F7;

    UART_Init(UART, &UART_InitStruct);
    /*  enable line status interrupt and rx data avaliable interrupt    */
    UART_INTConfig(UART, UART_INT_RD_AVA | UART_INT_LINE_STS | UART_INT_IDLE, ENABLE);
}

/****************************************************************************/
/* UART init                                                                */
/****************************************************************************/
void _qc_test_uart_init(void)
{
    RCC_PeriphClockCmd(APBPeriph_UART0, APBPeriph_UART0_CLOCK, ENABLE);
    _qc_uart_io_init();
    /*  UART Init   */
    _qc_uart_config();
    return;
}

static char *sdk_version_get(void)
{
    return "Bee2_SDK_v" VERSION_BUILD_STR;
}

static void _qc_get_mac_addr(uint8_t *mac)
{
    gap_get_param(GAP_PARAM_BD_ADDR, mac);
}

static void _qc_ble_scan_start(void)
{
    le_scan_start();
}

static void _qc_ble_scan_stop(void)
{
    le_scan_stop();
}

T_APP_RESULT _qc_gap_callback(uint8_t cb_type, void *p_cb_data)
{
    T_APP_RESULT result = APP_RESULT_SUCCESS;
    T_LE_CB_DATA *p_data = (T_LE_CB_DATA *)p_cb_data;
    char str[128] = {0};

    switch (cb_type)
    {
    case GAP_MSG_LE_SCAN_INFO:
        sprintf(str, "ADDR: " BT_MAC_STR_FORMAT ", RSSI: %d\r\n",
                p_data->p_le_scan_info->bd_addr[5], p_data->p_le_scan_info->bd_addr[4],
                p_data->p_le_scan_info->bd_addr[3], p_data->p_le_scan_info->bd_addr[2],
                p_data->p_le_scan_info->bd_addr[1], p_data->p_le_scan_info->bd_addr[0],
                p_data->p_le_scan_info->rssi);
        mf_printf(str);
    default:
        break;
    }
    return result;
}

void _qc_le_gap_init(void)
{
    //scan patameters
    uint8_t scan_mode = GAP_SCAN_MODE_ACTIVE;
    uint16_t scan_interval = 0x10;
    uint16_t scan_window = 0x10;
    uint8_t scan_filter_policy = GAP_SCAN_FILTER_ANY;
    uint8_t scan_filter_duplicate = GAP_SCAN_FILTER_DUPLICATE_ENABLE;

    //Register gap callback
    le_register_app_cb(_qc_gap_callback);

    le_scan_set_param(GAP_PARAM_SCAN_MODE, sizeof(scan_mode), &scan_mode);
    le_scan_set_param(GAP_PARAM_SCAN_INTERVAL, sizeof(scan_interval), &scan_interval);
    le_scan_set_param(GAP_PARAM_SCAN_WINDOW, sizeof(scan_window), &scan_window);
    le_scan_set_param(GAP_PARAM_SCAN_FILTER_POLICY, sizeof(scan_filter_policy),
                      &scan_filter_policy);
    le_scan_set_param(GAP_PARAM_SCAN_FILTER_DUPLICATES, sizeof(scan_filter_duplicate),
                      &scan_filter_duplicate);
}

static void _qc_ble_init(void)
{
    le_gap_init(1);
    gap_lib_init();
    _qc_le_gap_init();
}

/* MXCHIP standard QC test function main entrance, available for all modules */
static void _mxchip_qc_test(void *arg)
{
    char str[128];
    uint8_t mac[6];

    _qc_test_uart_init();
    _qc_ble_init();
    qc_app_task_init();

    mf_printf("==== MXCHIP Manufacture Test ====\r\n");
    QC_TEST_PRINT_STRING("Serial Number:", SERIAL_NUMBER);
    QC_TEST_PRINT_STRING("Library Version:", sdk_version_get());
    QC_TEST_PRINT_STRING("APP Version:", APP_VERSION_STR);

    _qc_get_mac_addr(mac);
    sprintf(str, MAC_STR_FORMAT, mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    QC_TEST_PRINT_STRING("MAC:", str);
    QC_TEST_PRINT_STRING("Local Bluetooth Address:", str);

    _qc_ble_scan_start();
    os_delay(1000);
    _qc_ble_scan_stop();

    os_task_delete(mxchp_qc_task);
}

static void _qc_init(void)
{
    RCC_PeriphClockCmd(APBPeriph_GPIO, APBPeriph_GPIO_CLOCK, ENABLE);
    Pinmux_Config(MXCHIP_QC_CHECK_PIN, DWGPIO);
    Pinmux_Config(MXCHIP_ATE_CHECK_PIN, DWGPIO);
    Pad_Config(MXCHIP_QC_CHECK_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    Pad_Config(MXCHIP_ATE_CHECK_PIN, PAD_PINMUX_MODE, PAD_IS_PWRON, PAD_PULL_UP, PAD_OUT_DISABLE, PAD_OUT_LOW);
    GPIO_InitTypeDef qc_pin_param;
    GPIO_StructInit(&qc_pin_param);
    qc_pin_param.GPIO_Pin = GPIO_GetPin(MXCHIP_QC_CHECK_PIN) | GPIO_GetPin(MXCHIP_ATE_CHECK_PIN);
    qc_pin_param.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(&qc_pin_param);
}

static uint8_t _qc_check(void)
{
    _qc_init();
    if (GPIO_ReadInputDataBit(GPIO_GetPin(MXCHIP_QC_CHECK_PIN)) == 0 && GPIO_ReadInputDataBit(GPIO_GetPin(MXCHIP_ATE_CHECK_PIN)) != 0)
    {
        return 1;
    }
    if (GPIO_ReadInputDataBit(GPIO_GetPin(MXCHIP_QC_CHECK_PIN)) == 0 && GPIO_ReadInputDataBit(GPIO_GetPin(MXCHIP_ATE_CHECK_PIN)) == 0)
    {
        return 2;
    }

    return 0;
}

void mxchip_qc_init(void)
{
    uint8_t qc_state = _qc_check();
    if (qc_state == 1)
    {
        os_task_create(&mxchp_qc_task, "qc", _mxchip_qc_test, 0, 2048,
                       APP_TASK_PRIORITY);

        os_sched_start();
    }
    else if (qc_state == 2)
    {
        switch_to_hci_mode();
    }
    else
    {
        ;
    }
}
