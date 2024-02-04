/*
 * FreeRTOS V202212.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * https://www.FreeRTOS.org
 * https://github.com/FreeRTOS
 *
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/
/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
extern uint32_t SystemCoreClock;
#endif

#define configUSE_PREEMPTION                    1U
#define configUSE_IDLE_HOOK                     0U
#define configUSE_TICK_HOOK                     1U
#define configTICK_RATE_HZ                      ( 1000U )
#define configUSE_QUEUE_SETS                    1U
#define configCPU_CLOCK_HZ                      ( SystemCoreClock )
#define configMAX_PRIORITIES                    ( 16U )
#define configMINIMAL_STACK_SIZE                ( ( uint16_t) 96U )
/* Note that the heap is managed by MicroTBX. Look at tbxfreertos.c for implementation
 * details. The total heap size is controlled by configuration macro TBX_CONF_HEAP_SIZE
 * in tbx_conf.h.
 */
/*#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 8U * 1024U ) )*/
#define configMAX_TASK_NAME_LEN                 ( 16U )
#define configUSE_16_BIT_TICKS                  0U
#define configIDLE_SHOULD_YIELD                 1U
#define configUSE_MUTEXES                       1U
#define configQUEUE_REGISTRY_SIZE               5U
#define configUSE_RECURSIVE_MUTEXES             1U
#define configUSE_MALLOC_FAILED_HOOK            1U
#define configUSE_APPLICATION_TASK_TAG          1U
#define configUSE_COUNTING_SEMAPHORES           1U
#define configUSE_TICKLESS_IDLE                 0U
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS	2

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION         0U
#define configSUPPORT_DYNAMIC_ALLOCATION        1U

/* Run time and task stats gathering related definitions. */
#define configCHECK_FOR_STACK_OVERFLOW          2U
#define configGENERATE_RUN_TIME_STATS           0U
#define configUSE_TRACE_FACILITY                1U
#define configUSE_STATS_FORMATTING_FUNCTIONS    0U

/* Co-routine related definitions. */
#define configUSE_CO_ROUTINES                   0U
#define configMAX_CO_ROUTINE_PRIORITIES         1U

/* Software timer related definitions. */
#define configUSE_TIMERS                        1U
#define configTIMER_TASK_PRIORITY               ( configMAX_PRIORITIES - 1U )
#define configTIMER_QUEUE_LENGTH                10U
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet                1U
#define INCLUDE_uxTaskPriorityGet               1U
#define INCLUDE_vTaskDelete                     1U
#define INCLUDE_vTaskCleanUpResources           1U
#define INCLUDE_vTaskSuspend                    1U
#define INCLUDE_vTaskDelayUntil                 1U
#define INCLUDE_vTaskDelay                      1U
#define INCLUDE_eTaskGetState                   1U
#define INCLUDE_xTimerPendFunctionCall          0U
#define INCLUDE_xSemaphoreGetMutexHolder        1U
#define INCLUDE_xTaskGetHandle                  1U
#define INCLUDE_xTaskGetCurrentTaskHandle       1U
#define INCLUDE_xTaskGetIdleTaskHandle          1U
#define INCLUDE_xTaskAbortDelay                 1U
#define INCLUDE_xTaskGetSchedulerState          1U
#define INCLUDE_xTaskGetIdleTaskHandle          1U
#define INCLUDE_uxTaskGetStackHighWaterMark     1U

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
	/* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
	#define configPRIO_BITS                             __NVIC_PRIO_BITS
#else
	#define configPRIO_BITS                             4        /* 15 priority levels */
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY       0xf

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY               ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY          ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
	
/* Use MicroTBX assertion in FreeRTOS. */
#include "tbx_freertos.h"

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_CONFIG_H */
