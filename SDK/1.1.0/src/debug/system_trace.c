/**
*********************************************************************************************************
*               Copyright(c) 2015, Realtek Semiconductor Corporation. All rights reserved.
**********************************************************************************************************
* @file
* @brief
* @details
* @author
* @date
* @version  v0.1
*********************************************************************************************************
*/

#include "trace_config.h"
#include "system_trace.h"
#include "os_timer.h"
#include "trace.h"

extern void *xTimerQueue;
extern void *io_queue_handle;
extern void *evt_queue_handle;

extern BOOL_PATCH_FUNC patch_vTaskSwitchContext;
void *xTimerQueue __attribute__((at(0x00200e58)));

#if (TRACE_HEAP_EN == 1)
HEAP_INFO heap_info[TRACE_HEAP_TYPE_NUM];

void print_heap_info(void)
{
    for (uint32_t i = 0; i < TRACE_HEAP_TYPE_NUM; ++i)
    {
        OS_PRINT_TRACE4("heap type: %i, total size: %d, remain size: %d, minumum ever free size: %d",
                        i, heap_info[i].total_size, heap_info[i].curr_remain_size, heap_info[i].minimum_ever_free_size);

        OS_PRINT_TRACE1("free list: free block %d.", heap_info[i].free_size_list.number);
        for (uint32_t j = 0; j < heap_info[i].free_size_list.number; ++j)
        {
            OS_PRINT_TRACE3("heap type: %i,free block: %d, free size: %d", i,
                            heap_info[i].free_size_list.number, heap_info[i].free_size_list.size[j]);
        }
    }
}
#endif

#if (TRACE_STACK_EN == 1)
STACK_INFO stack_info;

void print_stack_info(void)
{
    OS_PRINT_TRACE1("task stack: task number %d", stack_info.task_number);
    for (uint32_t i = 0; i < stack_info.task_number; ++i)
    {
        OS_PRINT_TRACE3("task id: %d, name: %s, minimum ever remain size %d bytes",
                        stack_info.task_stack_info[i].task_id, TRACE_STRING(stack_info.task_stack_info[i].task_name),
                        stack_info.task_stack_info[i].minimum_ever_remain_size);
    }
    OS_PRINT_TRACE1("main stack: minimum ever remain size %d bytes",
                    stack_info.main_stack_minimum_ever_remain_size);
}
#endif

#if (TRACE_TIMER_EN == 1)
TIMER_INFO timer_info;

void print_timer_info(void)
{
    OS_PRINT_TRACE3("timer: total %d, current used %d, minimum ever remain %d",
                    MAX_TIMER_NUM, timer_info.curr_used_num, timer_info.minimum_ever_remain_num);
}
#endif

void trace_timer_callback(void *p_timer)
{
#if (TRACE_HEAP_EN == 1)
    trace_heap();
    print_heap_info();
#endif

#if (TRACE_STACK_EN == 1)
    trace_task_stack();
    print_stack_info();
#endif

#if (TRACE_TIMER_EN == 1)
    trace_timer();
    print_timer_info();
#endif
}

void system_trace_init(void)
{
#if (TRACE_HEAP_EN == 1 || TRACE_STACK_EN == 1 || TRACE_TIMER_EN == 1)
    void *p_trace_timer = NULL;
    if (os_timer_create(&p_trace_timer, "trace_timer",  1, \
                        TRACE_PERIOD_TIME, true, trace_timer_callback))
    {
        os_timer_start(&p_trace_timer);
    }
#endif

#if (TRACE_TIMER_EN == 1)
    trace_timer_init();
#endif

#if (DEBUG_DLPS_ERROR_EN == 1)
    if (!trace_dlps_init(10000, 60000, 0))
    {
        OS_PRINT_ERROR0("trace dlps init fail!");
    }
#endif

#if (DEBUG_TASK_HANG_EN == 1)
    trace_task_hang_init(2, &xTimerQueue, 12, &evt_queue_handle, 10);
#endif
}


