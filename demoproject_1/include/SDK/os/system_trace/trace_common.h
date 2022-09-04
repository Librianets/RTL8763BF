/****************************************************************************************************//**
 * @file     trace_common.h
 *
 * @brief
 *
 * @version  v0.1
 * @date     2018-11-05
 *
 * @note
 *******************************************************************************************************/
#ifndef _TRACE_COMMON_H_
#define _TRACE_COMMON_H_

#include <stdint.h>
#include <stdbool.h>

#ifndef NULL
#define NULL (void *)0
#endif

#define portSTACK_TYPE  uint32_t
#define portBASE_TYPE   long

typedef uint32_t size_t;
typedef portSTACK_TYPE StackType_t;
typedef unsigned long UBaseType_t;

typedef bool (*BOOL_PATCH_FUNC)();

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif

#endif /*_TRACE_COMMON_H_*/

/******************* (C) COPYRIGHT 2015 Realtek Semiconductor Corporation *****END OF FILE****/
