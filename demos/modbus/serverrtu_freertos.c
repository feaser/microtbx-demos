/************************************************************************************//**
* \file         demos/modbus/serverrtu_freertos.c
* \brief        Demonstrates how to setup a Modbus RTU server.
* \details      This demo application creates a Modbus server, configured for RTU
*               communication at 19200 bits/sec, 8 databits, even parity and 1 stopbit.
*
*               The Modbus server is assigned node identifier 10 (0x0A).
*
*               The FreeRTOS operating system drives the application and the Modbus
*               communication stack. A task with the name ModbusTask() is created upon
*               application initialization. This task calls TbxMbEventTask() to process
*               Modbus related events. Note that the task sleeps, when no events await
*               processing.
*
*               The Modbus server supports the following data table. Note that the data
*               table addresses are zero-based:
*
*                                   |---------------------------|
*                                   |           COILS           |
*                                   |---------------------------|
*                                   |           00000           | --> BSP_DIGITAL_OUT1
*                                   |           00001           | --> BSP_DIGITAL_OUT2
*                                   |---------------------------|
*                                   |      DISCRETE INPUTS      |
*                                   |---------------------------|
*               BSP_DIGITAL_IN1 --> |           10000           |
*               BSP_DIGITAL_IN2 --> |           10001           |
*                                   |---------------------------|
*                                   |      INPUT REGISTERS      |
*                                   |---------------------------|
*                BSP_ANALOG_IN1 --> |           30000           |
*                BSP_ANALOG_IN2 --> |           30001           |
*                                   |---------------------------|
*                                   |     HOLDING REGISTERS     |
*                                   |---------------------------|
*                                   |           40000           | --> BSP_PWM_OUT1
*                                   |           40001           | --> BSP_PWM_OUT2
*                                   |---------------------------|
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
/** \brief Priority of the Modbus task. */
#define MODBUS_TASK_PRIO           ((UBaseType_t)4U)


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void               ModbusTask           (void         * pvParameters);

static tTbxMbServerResult ModbusWriteCoil      (tTbxMbServer   channel,
                                                uint16_t       addr,
                                                uint8_t        value);

static tTbxMbServerResult ModbusReadInput      (tTbxMbServer   channel,
                                                uint16_t       addr,
                                                uint8_t      * value);

static tTbxMbServerResult ModbusReadInputReg   (tTbxMbServer   channel,
                                                uint16_t       addr,
                                                uint16_t     * value);

static tTbxMbServerResult ModbusWriteHoldingReg(tTbxMbServer   channel,
                                                uint16_t       addr,
                                                uint16_t       value);


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Modbus RTU transport layer handle. */
static tTbxMbTp modbusRtuTransport;

/** \brief Modbus server channel handle. */
static tTbxMbServer modbusServer;


/************************************************************************************//**
** \brief     Entry point for the demo application. Called by main() after the 
**            initialization completed.
**
****************************************************************************************/
void DemoMain(void)
{
  /* Create a Modbus RTU transport layer object. */
  modbusRtuTransport = TbxMbRtuCreate(0x0A, TBX_MB_UART_PORT2, TBX_MB_UART_19200BPS,
                                      TBX_MB_UART_1_STOPBITS, TBX_MB_EVEN_PARITY);
  /* Create a Modbus server channel object and link the RTU transport layer object. */
  modbusServer = TbxMbServerCreate(modbusRtuTransport);
  /* Set the callbacks for accessing the Modbus data tables. */
  TbxMbServerSetCallbackWriteCoil(modbusServer, ModbusWriteCoil);
  TbxMbServerSetCallbackReadInput(modbusServer, ModbusReadInput);
  TbxMbServerSetCallbackReadInputReg(modbusServer, ModbusReadInputReg);
  TbxMbServerSetCallbackWriteHoldingReg(modbusServer, ModbusWriteHoldingReg);
  /* Create the Modbus task. */
  xTaskCreate(ModbusTask, "ModbusTask", configMINIMAL_STACK_SIZE, NULL,
              MODBUS_TASK_PRIO, NULL);
  /* Start the RTOS scheduler. Note that this function does not return. */
  vTaskStartScheduler();
} /*** end of DemoMain ***/


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


/************************************************************************************//**
** \brief     Writes a data element to the coils data table.
** \details   Note that the element is specified by its zero-based address in the range
**            0 - 65535, not its element number (1 - 65536).
** \param     channel Handle to the Modbus server channel object that triggered the 
**            callback.
** \param     addr Element address (0..65535).
** \param     value Coil value. Use TBX_ON to activate the coil, TBX_OFF otherwise.
** \return    TBX_MB_SERVER_OK if successful, TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR if the
**            specific data element address is not supported by this server, 
**            TBX_MB_SERVER_ERR_DEVICE_FAILURE otherwise.
**
****************************************************************************************/
static tTbxMbServerResult ModbusWriteCoil(tTbxMbServer   channel,
                                          uint16_t       addr,
                                          uint8_t        value)
{
  tTbxMbServerResult result = TBX_MB_SERVER_OK;

  TBX_UNUSED_ARG(channel);

  /* Filter on the requested coil address. */
  switch (addr)
  {
  case 0U:
    BspDigitalOut(BSP_DIGITAL_OUT1, value);
    break;
  
  case 1U:
    BspDigitalOut(BSP_DIGITAL_OUT2, value);
    break;
  
  default:
    /* Unsupported coil address. */
    result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
    break;
  }

  /* Give the result back to the caller. */
  return result;
} /*** end of ModbusWriteCoil ***/


/************************************************************************************//**
** \brief     Reads a data element from the discrete inputs data table.
** \details   Note that the element is specified by its zero-based address in the range
**            0 - 65535, not its element number (1 - 65536).
** \param     channel Handle to the Modbus server channel object that triggered the 
**            callback.
** \param     addr Element address (0..65535).
** \param     value Pointer to write the value of the input to. Use TBX_ON if the input
**            is on, TBX_OFF otherwise.
** \return    TBX_MB_SERVER_OK if successful, TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR if the
**            specific data element address is not supported by this server, 
**            TBX_MB_SERVER_ERR_DEVICE_FAILURE otherwise.
**
****************************************************************************************/
static tTbxMbServerResult ModbusReadInput(tTbxMbServer   channel,
                                          uint16_t       addr,
                                          uint8_t      * value)
{
  tTbxMbServerResult result = TBX_MB_SERVER_OK;

  TBX_UNUSED_ARG(channel);

  /* Filter on the requested discrete input address. */
  switch (addr)
  {
  case 10000U:
    *value = BspDigitalIn(BSP_DIGITAL_IN1);
    break;
  
  case 10001U:
    *value = BspDigitalIn(BSP_DIGITAL_IN2);
    break;
  
  default:
    /* Unsupported discrete input address. */
    result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
    break;
  }

  /* Give the result back to the caller. */
  return result;
} /*** end of ModbusReadInput ***/


/************************************************************************************//**
** \brief     Reads a data element from the input registers data table.
** \details   Write the value of the input register in your CPUs native endianess. The
**            MicroTBX-Modbus stack will automatically convert this to the big endianess
**            that the Modbus protocol requires.
**            Note that the element is specified by its zero-based address in the range
**            0 - 65535, not its element number (1 - 65536).
** \param     channel Handle to the Modbus server channel object that triggered the 
**            callback.
** \param     addr Element address (0..65535).
** \param     value Pointer to write the value of the input register to.
** \return    TBX_MB_SERVER_OK if successful, TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR if the
**            specific data element address is not supported by this server, 
**            TBX_MB_SERVER_ERR_DEVICE_FAILURE otherwise.
**
****************************************************************************************/
static tTbxMbServerResult ModbusReadInputReg(tTbxMbServer   channel,
                                             uint16_t       addr, 
                                             uint16_t     * value)
{
  tTbxMbServerResult result = TBX_MB_SERVER_OK;

  TBX_UNUSED_ARG(channel);

  /* Filter on the requested input register address. */
  switch (addr)
  {
  case 30000U:
    *value = BspAnalogIn(BSP_ANALOG_IN1);
    break;
  
  case 30001U:
    *value = BspAnalogIn(BSP_ANALOG_IN2);
    break;
  
  default:
    /* Unsupported input register address. */
    result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
    break;
  }

  /* Give the result back to the caller. */
  return result;
} /*** end of ModbusReadInputReg ***/


/************************************************************************************//**
** \brief     Writes a data element to the holding registers data table.
** \details   Note that the element is specified by its zero-based address in the range
**            0 - 65535, not its element number (1 - 65536).
**            The value of the holding register in already in your CPUs native endianess.
** \param     channel Handle to the Modbus server channel object that triggered the 
**            callback.
** \param     addr Element address (0..65535).
** \param     value Value of the holding register.
** \return    TBX_MB_SERVER_OK if successful, TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR if the
**            specific data element address is not supported by this server, 
**            TBX_MB_SERVER_ERR_DEVICE_FAILURE otherwise.
**
****************************************************************************************/
static tTbxMbServerResult ModbusWriteHoldingReg(tTbxMbServer channel,
                                                uint16_t     addr, 
                                                uint16_t     value)
{
  tTbxMbServerResult result = TBX_MB_SERVER_OK;

  TBX_UNUSED_ARG(channel);

  /* Filter on the requested holding register address. */
  switch (addr)
  {
  case 40000U:
    /* PWM supports 8-bit duty cycle. */
    if (value <= 255U)
    {
      BspPwmOut(BSP_PWM_OUT1, (uint8_t)value);
    }
    else
    {
      result = TBX_MB_SERVER_ERR_DEVICE_FAILURE;
    }
    break;
  
  case 40001U:
    /* PWM supports 8-bit duty cycle. */
    if (value <= 255U)
    {
      BspPwmOut(BSP_PWM_OUT2, (uint8_t)value);
    }
    else
    {
      result = TBX_MB_SERVER_ERR_DEVICE_FAILURE;
    }
    break;
  
  default:
    /* Unsupported holding register address. */
    result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
    break;
  }

  /* Give the result back to the caller. */
  return result;
} /*** end of ModbusWriteHoldingReg ***/


/*********************************** end of serverrtu_freertos.c ************************/
