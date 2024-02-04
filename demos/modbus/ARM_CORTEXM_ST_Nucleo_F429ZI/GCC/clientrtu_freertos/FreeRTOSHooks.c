/************************************************************************************//**
* \file         FreeRTOSHooks.c
* \brief        Source file that implements the FreeRTOS hook functions.
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2024 by Feaser     www.feaser.com     All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
*
* SPDX-License-Identifier: MIT
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* \endinternal
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "FreeRTOS.h"                       /* FreeRTOS                                */
#include "task.h"                           /* FreeRTOS tasks                          */
#include "stm32f4xx.h"                           /* STM32 CPU and HAL                  */


/************************************************************************************//**
** \brief     FreeRTOS hook function that gets called each time tick of the operating
**            system.
**
****************************************************************************************/
void vApplicationTickHook(void)
{
  /* Inform the HAL timebase about the event. */
  HAL_IncTick();
} /*** end of vApplicationTickHook ***/


/************************************************************************************//**
** \brief     FreeRTOS hook function that gets called when memory allocation failed.
**
****************************************************************************************/
void vApplicationMallocFailedHook(void)
{
  /* Trigger an assertion for debugging purposes. */
  configASSERT(pdFAIL);
} /*** end of vApplicationMallocFailedHook ***/


/************************************************************************************//**
** \brief     FreeRTOS hook function that gets called when a stack overflow was detected.
** \param     xTask Handle of the task that has a stack overflow.
** \param     pcTaskName Name of the task that has a stack overflow.
**
****************************************************************************************/
void vApplicationStackOverflowHook(TaskHandle_t xTask, char * pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;

  /* Trigger an assertion for debugging purposes. */
  configASSERT(pdFAIL);
} /*** end of vApplicationStackOverflowHook ***/


/*********************************** end of FreeRTOSHooks.c ****************************/
