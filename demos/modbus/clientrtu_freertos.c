/************************************************************************************//**
* \file         demos/modbus/clientrtu_freertos.c
* \brief        Demonstrates how to setup a Modbus RTU client.
* \details      This demo application creates a Modbus client, configured for RTU
*               communication at 19200 bits/sec, 8 databits, even parity and 1 stopbit.
*
*               The FreeRTOS operating system drives the application and the Modbus
*               communication stack. A task with the name ModbusTask() is created upon
*               application initialization. This task calls TbxMbEventTask() to process
*               Modbus related events. Note that the task sleeps, when no events await
*               processing.
*
*               The Modbus client communicates with a Modbus server, with assigned node
*               identifier 10 (0x0A). The Modbus server is assumed to support the
*               following data table, with zero-based table addresses:
*
*                                   |---------------------------|
*                                   |           COILS           |
*                                   |---------------------------|
*                                   |           00000           |
*                                   |           00001           |
*                                   |---------------------------|
*                                   |      DISCRETE INPUTS      |
*                                   |---------------------------|
*                                   |           10000           |
*                                   |           10001           |
*                                   |---------------------------|
*                                   |      INPUT REGISTERS      |
*                                   |---------------------------|
*                                   |           30000           |
*                                   |           30001           |
*                                   |---------------------------|
*                                   |     HOLDING REGISTERS     |
*                                   |---------------------------|
*                                   |           40000           |
*                                   |           40001           |
*                                   |---------------------------|
*
*               Note that this Modbus data table layout matches that of the Modbus RTU
*               server demo programs. As such this Modbus RTU client demo program can
*               be used as a counter-part for communicating with another node that runs
*               one of the Modbus RTU server demo programs.
*
*               Upon application startup the current state of the two discrete inputs and
*               two input registers on the Modbus server are read. Afterwards, both coils 
*               and holding registers on the Modbus server are placed in a predefined
*               state.
*
*               In the infinite loop that drives the application, the Modbus client 
*               toggles its own digital output BSP_DIGITAL_OUT1 every 500 milliseconds,
*               as well as the state of the first coil on the Modbus server.
*
*               If the communication with the Modbus server did not succeed, this toggle
*               interval of the client's own digital output and the first coil of the
*               Modbus server, changes to 2000 milliseconds.
*
*               Digital output BSP_DIGITAL_OUT1 is typically connected to an LED on the
*               Modbus client board. Likewise for the coil on the Modbus server board.
*
*               With other words: You can look at the LED on the Modbus client and server
*               boards to visually inspect if the communication between the Modbus client
*               and server works.
*
*               On the Modbus client, a fast LED toggle means that all is okay, a slow
*               LED toggle means that the communication with the Modbus server is not
*               working.
*
*               On the Modbus server, a fast LED toggle means that all is okay and an LED
*               that does not toggle means that it did not properly receive the request
*               from the Modbus client to toggle its first coil.
* 
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2023 by Feaser     www.feaser.com     All rights reserved
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
#include "FreeRTOS.h"                            /* FreeRTOS                           */
#include "task.h"                                /* FreeRTOS tasks                     */
#include "microtbx.h"                            /* MicroTBX library                   */
#include "microtbxmodbus.h"                      /* MicroTBX-Modbus library            */
#include "bsp.h"                                 /* Board support package              */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Priority of the application task. */
#define APP_TASK_PRIO             ((UBaseType_t)8U)

/** \brief Priority of the Modbus task. */
#define MODBUS_TASK_PRIO          ((UBaseType_t)4U)


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void               AppTask              (void         * pvParameters);

static void               ModbusTask           (void         * pvParameters);


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Modbus RTU transport layer handle. */
static tTbxMbTp modbusRtuTransport;

/** \brief Modbus client channel handle. */
static tTbxMbClient modbusClient;


/************************************************************************************//**
** \brief     Entry point for the demo application. Called by main() after the 
**            initialization completed.
**
****************************************************************************************/
void DemoMain(void)
{
  /* Create a Modbus RTU transport layer object. */
  modbusRtuTransport = TbxMbRtuCreate(0U, TBX_MB_UART_PORT2, TBX_MB_UART_19200BPS,
                                      TBX_MB_UART_1_STOPBITS, TBX_MB_EVEN_PARITY);
  /* Create a Modbus client channel object and link the RTU transport layer object. */
  modbusClient = TbxMbClientCreate(modbusRtuTransport, 400U, 100U);
  /* Create the application task. */
  xTaskCreate(AppTask, "AppTask", configMINIMAL_STACK_SIZE, NULL,
              APP_TASK_PRIO, NULL);
  /* Create the Modbus task. */
  xTaskCreate(ModbusTask, "ModbusTask", configMINIMAL_STACK_SIZE, NULL,
              MODBUS_TASK_PRIO, NULL);
  /* Start the RTOS scheduler. Note that this function does not return. */
  vTaskStartScheduler();
} /*** end of DemoMain ***/


/************************************************************************************//**
** \brief     Application task function.
** \param     pvParameters Pointer to optional task parameters
**
****************************************************************************************/
static void AppTask(void * pvParameters)
{
  TickType_t const toggleTicksNormal  =  500U / portTICK_PERIOD_MS;
  TickType_t const toggleTicksCommErr = 2000U / portTICK_PERIOD_MS;
  TickType_t       lastWakeTime;
  TickType_t       ledToggleTicks     = toggleTicksNormal;
  uint8_t          ledState           = TBX_OFF;
  uint16_t         inputRegs[2]       = { 0 };
  uint8_t          inputs[2]          = { 0 };
  uint8_t          coils[2]           = { TBX_OFF, TBX_OFF };
  uint16_t         holdingRegs[2]     = { 63U, 127U };

  TBX_UNUSED_ARG(pvParameters);

  /* Write coils. */
  TbxMbClientWriteCoils(modbusClient, 0x0AU, 0U, 2U, coils);

  /* Read discrete inputs. */
  TbxMbClientReadInputs(modbusClient, 0x0AU, 10000U, 2U, inputs);

  /* Read input registers. */
  TbxMbClientReadInputRegs(modbusClient, 0x0AU, 30000U, 2U, inputRegs);

  /* Write holding registers. */
  TbxMbClientWriteHoldingRegs(modbusClient, 0x0AU, 40000U, 2U, holdingRegs);

  /* Initialize the last wake time with the current time. */
  lastWakeTime = xTaskGetTickCount();

  /* Enter infinite task loop. */
  for (;;)
  {
    /* Toggle the LED at the configured interval. */
    vTaskDelayUntil(&lastWakeTime, ledToggleTicks);
    ledState = (ledState == TBX_ON) ? TBX_OFF : TBX_ON;
    BspDigitalOut(BSP_DIGITAL_OUT1, ledState);
    /* Write LED state to the first coil. */
    coils[0] = ledState;
    uint8_t commResult = TbxMbClientWriteCoils(modbusClient, 0x0AU, 0U, 1U, coils);
    /* Update LED toggle interval based on the communication result. In case of a 
     * communication error, toggle the LED at a lower rate.
     */
    ledToggleTicks = (commResult == TBX_OK) ? toggleTicksNormal : toggleTicksCommErr;
  }
} /*** end of AppTask ***/


/************************************************************************************//**
** \brief     Modbus task function.
** \param     pvParameters Pointer to optional task parameters
**
****************************************************************************************/
static void ModbusTask(void * pvParameters)
{
  TBX_UNUSED_ARG(pvParameters);

  /* Enter infinite task loop. */
  for (;;)
  {
    /* Continuously call the Modbus stack event task function. */
    TbxMbEventTask();
  }
} /*** end of ModbusTask ***/


/*********************************** end of clientrtu_freertos.c ***********************/
