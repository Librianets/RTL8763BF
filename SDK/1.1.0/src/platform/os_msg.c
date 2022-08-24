/**
 * Copyright (c) 2018, Realsil Semiconductor Corporation. All rights reserved.
 */

#include "rtl876x.h"
#include <stdint.h>
#include <string.h>
#include "trace.h"
#include "os_msg.h"
#include "core_cmFunc.h"

#define pdFALSE                         ( ( BaseType_t ) 0 )
#define pdTRUE                          ( ( BaseType_t ) 1 )
#define portMAX_DELAY ( TickType_t )    0xffffffffUL
#define configTICK_RATE_HZ              ( ( TickType_t ) 100 )
#define portTICK_PERIOD_MS              ( ( TickType_t ) 1000 / configTICK_RATE_HZ )

/* Constants used with memory barrier intrinsics. */
#define portSY_FULL_READ_WRITE          ( 15 )
#define portNVIC_INT_CTRL_REG           ( * ( ( volatile uint32_t * ) 0xe000ed04 ) )
#define portNVIC_PENDSVSET_BIT          ( 1UL << 28UL )

#define portYIELD()                                                             \
    {                                                                               \
        /* Set a PendSV to request a context switch. */                             \
        portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;                             \
        \
        /* Barriers are normally not required but do ensure the code is completely  \
        within the specified behaviour for the architecture. */                        \
        __dsb( portSY_FULL_READ_WRITE );                                            \
        __isb( portSY_FULL_READ_WRITE );                                            \
    }

#define portEND_SWITCHING_ISR( xSwitchRequired ) if( xSwitchRequired != pdFALSE ) portYIELD()

/* For internal use only. */
#define queueSEND_TO_BACK       ( ( BaseType_t ) 0 )
#define queueSEND_TO_FRONT      ( ( BaseType_t ) 1 )
#define queueOVERWRITE          ( ( BaseType_t ) 2 )

#define xQueueSendToFront( xQueue, pvItemToQueue, xTicksToWait ) xQueueGenericSend( ( xQueue ), ( pvItemToQueue ), ( xTicksToWait ), queueSEND_TO_FRONT )
#define xQueueSendToFrontFromISR( xQueue, pvItemToQueue, pxHigherPriorityTaskWoken ) xQueueGenericSendFromISR( ( xQueue ), ( pvItemToQueue ), ( pxHigherPriorityTaskWoken ), queueSEND_TO_FRONT )


typedef long BaseType_t;
typedef uint32_t TickType_t;
typedef void *QueueHandle_t;

extern BaseType_t xQueueGenericSend(QueueHandle_t xQueue, const void *const pvItemToQueue,
                                    TickType_t xTicksToWait, const BaseType_t xCopyPosition);
extern BaseType_t xQueueGenericSendFromISR(QueueHandle_t xQueue, const void *const pvItemToQueue,
                                           BaseType_t *const pxHigherPriorityTaskWoken, const BaseType_t xCopyPosition);

static inline bool osif_task_context_check(void)
{
    return (__get_IPSR() == 0);
}

bool osif_msg_send_to_front(void *p_handle, void *p_msg, uint32_t wait_ms)
{
    BaseType_t ret;

    if (osif_task_context_check() == true)
    {
        TickType_t wait_ticks;

        if (wait_ms == 0xFFFFFFFFUL)
        {
            wait_ticks = portMAX_DELAY;
        }
        else
        {
            wait_ticks = (TickType_t)((wait_ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
        }

        ret = xQueueSendToFront((QueueHandle_t)p_handle, p_msg, wait_ticks);
    }
    else
    {
        BaseType_t task_woken = pdFALSE;

        ret = xQueueSendToFrontFromISR((QueueHandle_t)p_handle, p_msg, &task_woken);

        portEND_SWITCHING_ISR(task_woken);
    }

    if (ret == pdTRUE)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool os_msg_send_to_front_intern(void *p_handle, void *p_msg, uint32_t wait_ms,
                                 const char *p_func, uint32_t file_line)
{
    bool ret;

    ret = osif_msg_send_to_front(p_handle, p_msg, wait_ms);
    if (ret == false)
    {
        OSIF_PRINT_ERROR5("os_msg_send_to_front_intern: %s<%u> failed p_handle %p, p_msg %p, wait_ms %u",
                          TRACE_STRING(p_func), file_line, p_handle, p_msg, wait_ms);
    }

    return ret;
}



/* patch osif_task_signal_send with Set Bits Mode */
#include "app_section.h"

typedef enum
{
    eNoAction = 0,              /* Notify the task without updating its notify value. */
    eSetBits,                   /* Set bits in the task's notification value. */
    eIncrement,                 /* Increment the task's notification value. */
    eSetValueWithOverwrite,     /* Set the task's notification value to a specific value even if the previous value has not yet been read by the task. */
    eSetValueWithoutOverwrite   /* Set the task's notification value if the previous value has been read by the task. */
} eNotifyAction;

typedef void *TaskHandle_t;

BaseType_t xTaskGenericNotify(TaskHandle_t xTaskToNotify, uint32_t ulValue, eNotifyAction eAction,
                              uint32_t *pulPreviousNotificationValue);
#define xTaskNotify( xTaskToNotify, ulValue, eAction ) xTaskGenericNotify( ( xTaskToNotify ), ( ulValue ), ( eAction ), NULL )

BaseType_t xTaskGenericNotifyFromISR(TaskHandle_t xTaskToNotify, uint32_t ulValue,
                                     eNotifyAction eAction, uint32_t *pulPreviousNotificationValue,
                                     BaseType_t *pxHigherPriorityTaskWoken);
#define xTaskNotifyFromISR( xTaskToNotify, ulValue, eAction, pxHigherPriorityTaskWoken ) xTaskGenericNotifyFromISR( ( xTaskToNotify ), ( ulValue ), ( eAction ), NULL, ( pxHigherPriorityTaskWoken ) )

DATA_RAM_FUNCTION
bool patch_osif_task_signal_send(void *p_handle, uint32_t signal, bool *ret)
{
    BaseType_t _ret;

    if (osif_task_context_check() == true)
    {
        _ret = xTaskNotify((TaskHandle_t)p_handle, signal, eSetBits);
    }
    else
    {
        BaseType_t task_woken = pdFALSE;

        _ret = xTaskNotifyFromISR((TaskHandle_t)p_handle, signal,
                                  eSetBits, &task_woken);

        portEND_SWITCHING_ISR(task_woken);
    }

    if (_ret == pdTRUE)
    {
        *ret = true;
    }
    else
    {
        *ret = false;
    }

    return true;
}

void osif_task_signal_patch_for_set_bits(void)
{
    typedef bool (*BOOL_PATCH_FUNC)();
    extern BOOL_PATCH_FUNC patch_osif_os_task_signal_send;

    patch_osif_os_task_signal_send = patch_osif_task_signal_send;
}

