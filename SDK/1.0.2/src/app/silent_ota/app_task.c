/**
*****************************************************************************************
*     Copyright(c) 2016, Realtek Semiconductor Corporation. All rights reserved.
*****************************************************************************************
  * @file    app_task.c
  * @brief   application task.
  * @details
  * @author  ranhui
  * @date    2016-02-18
  * @version v1.0
  ******************************************************************************
  * @attention
  * <h2><center>&copy; COPYRIGHT 2016 Realtek Semiconductor Corporation</center></h2>
  ******************************************************************************
  */
#include <os_msg.h>
#include <os_task.h>
//#include <btif.h>
#include <gap.h>
#include <gap_le.h>
#include <gap_msg.h>
#include <trace.h>
#include <app_task.h>
#include <app_msg.h>
#include <app_task.h>
//#include <peripheral_app.h>
#include <dfu_application.h>

#define APP_TASK_PRIORITY          1   /* Task priorities. */
#define APP_TASK_STACK_SIZE        512 * 4

#define MAX_NUMBER_OF_GAP_MESSAGE    0x20
#define MAX_NUMBER_OF_IO_MESSAGE      0x20
#define MAX_NUMBER_OF_EVENT_MESSAGE   (MAX_NUMBER_OF_GAP_MESSAGE + MAX_NUMBER_OF_IO_MESSAGE)

void *app_task_handle;
void *evt_queue_handle;
void *io_queue_handle;

void app_main_task(void *p_param);

void app_task_init()
{
    os_task_create(&app_task_handle, "app", app_main_task, 0, APP_TASK_STACK_SIZE,
                   APP_TASK_PRIORITY);
}

void Driver_Init(void);
int codecInit(void);
/**
* @brief
*
*
* @param   pvParameters
* @return  void
*/
void app_main_task(void *p_param)
{
    uint8_t event;

    os_msg_queue_create(&io_queue_handle, MAX_NUMBER_OF_IO_MESSAGE, sizeof(T_IO_MSG));
    os_msg_queue_create(&evt_queue_handle, MAX_NUMBER_OF_EVENT_MESSAGE, sizeof(uint8_t));

    gap_start_bt_stack(evt_queue_handle, io_queue_handle, MAX_NUMBER_OF_GAP_MESSAGE);

    Driver_Init();

    while (true)
    {
        if (os_msg_recv(evt_queue_handle, &event, 0xFFFFFFFF) == true)
        {
            //DBG_DIRECT("***os_msg_recv***");
            if (event == EVENT_IO_TO_APP)
            {
                T_IO_MSG io_msg;
                if (os_msg_recv(io_queue_handle, &io_msg, 0) == true)
                {
                    app_handle_io_msg(io_msg);
                }
            }
            else
            {
                gap_handle_msg(event);
            }
        }
    }
}

