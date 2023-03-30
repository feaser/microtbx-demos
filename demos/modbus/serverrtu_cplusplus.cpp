/************************************************************************************//**
* \file         demos/modbus/serverrtu_cplusplus.c
* \brief        Demonstrates how to setup a Modbus RTU server in C++.
* \details      This demo application creates a Modbus server, configured for RTU
*               communication at 19200 bits/sec, 8 databits, even parity and 1 stopbit.
*
*               The Modbus server is assigned node identifier 10 (0x0A).
*
*               The DemoApp class derives from class TbxMbServerRtu and override these
*               methods, when access to the Modbus data table is requested:
* 
*                 - writeCoil()
*                 - readInput()
*                 - readInputReg()
*                 - writeHoldingReg()
*
*               A traditional infinite loop, a.k.a "superloop" drives the application and
*               the Modbus communication stack. It calls TbxMbEventTask() to process
*               Modbus related events.
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
#include "microtbx.h"                            /* MicroTBX library                   */
#include "microtbxmodbus.hpp"                    /* MicroTBX-Modbus C++ library        */
#include "bsp.h"                                 /* Board support package              */


/****************************************************************************************
* Class definitions
****************************************************************************************/
/** \brief Demo application class. */
class DemoApp : public TbxMbServerRtu
{
public:
  /* Constructors and destructor. */
  DemoApp() 
    : TbxMbServerRtu(0x0A, TBX_MB_UART_PORT2, TBX_MB_UART_19200BPS, 
                     TBX_MB_UART_1_STOPBITS, TBX_MB_EVEN_PARITY) { }
  virtual ~DemoApp() { }
  /* Methods. */
  void run();

private:
  /* Methods. */
  tTbxMbServerResult writeCoil(uint16_t addr, bool value) override;
  tTbxMbServerResult readInput(uint16_t addr, bool& value) override;
  tTbxMbServerResult readInputReg(uint16_t addr, uint16_t &value) override;
  tTbxMbServerResult writeHoldingReg(uint16_t addr, uint16_t value) override;
};


/****************************************************************************************
* Class implementations
****************************************************************************************/
/************************************************************************************//**
** \brief     Runs the demo application.
**
****************************************************************************************/
void DemoApp::run()
{
  /* Enter the infinite program loop. */
  for (;;)
  {
    /* Continuously call the Modbus stack event task function. */
    TbxMbEvent::task();
  }
} /*** end of run ***/


/************************************************************************************//**
** \brief     Writes a data element to the coils data table.
** \details   Note that the element is specified by its zero-based address in the range
**            0 - 65535, not its element number (1 - 65536).
** \param     addr Element address (0..65535).
** \param     value Coil value.
** \return    TBX_MB_SERVER_OK if successful, TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR if the
**            specific data element address is not supported by this server, 
**            TBX_MB_SERVER_ERR_DEVICE_FAILURE otherwise.
**
****************************************************************************************/
tTbxMbServerResult DemoApp::writeCoil(uint16_t addr, 
                                      bool    value)
{
  tTbxMbServerResult result    = TBX_MB_SERVER_OK;
  uint8_t            coilValue = (value) ? TBX_ON : TBX_OFF;

  /* Filter on the requested coil address. */
  switch (addr)
  {
  case 0U:
    BspDigitalOut(BSP_DIGITAL_OUT1, coilValue);
    break;
  
  case 1U:
    BspDigitalOut(BSP_DIGITAL_OUT2, coilValue);
    break;
  
  default:
    /* Unsupported coil address. */
    result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
    break;
  }

  /* Give the result back to the caller. */
  return result;
} /*** end of writeCoil ***/


/************************************************************************************//**
** \brief     Reads a data element from the discrete input registers data table.
** \details   Note that the element is specified by its zero-based address in the range
**            0 - 65535, not its element number (1 - 65536).
** \param     addr Element address (0..65535).
** \param     value Reference where to store the value of the discrete input.
** \return    TBX_MB_SERVER_OK if successful, TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR if the
**            specific data element address is not supported by this server, 
**            TBX_MB_SERVER_ERR_DEVICE_FAILURE otherwise.
**
****************************************************************************************/
tTbxMbServerResult DemoApp::readInput(uint16_t addr, 
                                      bool&    value)
{
  tTbxMbServerResult result = TBX_MB_SERVER_OK;

  /* Filter on the requested discrete input address. */
  switch (addr)
  {
  case 10000U:
    value = (BspDigitalIn(BSP_DIGITAL_IN1) == TBX_ON) ? true : false;
    break;
  
  case 10001U:
    value = (BspDigitalIn(BSP_DIGITAL_IN2) == TBX_ON) ? true : false;
    break;
  
  default:
    /* Unsupported discrete input address. */
    result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
    break;
  }

  /* Give the result back to the caller. */
  return result;
} /*** end of readInput ***/


/************************************************************************************//**
** \brief     Reads a data element from the input registers data table.
** \details   Note that the element is specified by its zero-based address in the range
**            0 - 65535, not its element number (1 - 65536).
** \attention Store the value of the input register in your CPUs native endianess. The
**            MicroTBX-Modbus stack will automatically convert this to the big endianess
**            that the Modbus protocol requires.
** \param     addr Element address (0..65535).
** \param     value Reference where to store the value of the input register.
** \return    TBX_MB_SERVER_OK if successful, TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR if the
**            specific data element address is not supported by this server, 
**            TBX_MB_SERVER_ERR_DEVICE_FAILURE otherwise.
**
****************************************************************************************/
tTbxMbServerResult DemoApp::readInputReg(uint16_t  addr, 
                                         uint16_t& value)
{
  tTbxMbServerResult result = TBX_MB_SERVER_OK;

  /* Filter on the requested input register address. */
  switch (addr)
  {
  case 30000U:
    value = BspAnalogIn(BSP_ANALOG_IN1);
    break;
  
  case 30001U:
    value = BspAnalogIn(BSP_ANALOG_IN2);
    break;
  
  default:
    /* Unsupported input register address. */
    result = TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR;
    break;
  }

  /* Give the result back to the caller. */
  return result;
} /*** end of readInputReg ***/


/************************************************************************************//**
** \brief     Writes a data element to the holding registers data table.
** \details   Note that the element is specified by its zero-based address in the range
**            0 - 65535, not its element number (1 - 65536).
** \attention The value of the holding register in already in your CPUs native endianess.
** \param     addr Element address (0..65535).
** \param     value Value of the holding register.
** \return    TBX_MB_SERVER_OK if successful, TBX_MB_SERVER_ERR_ILLEGAL_DATA_ADDR if the
**            specific data element address is not supported by this server, 
**            TBX_MB_SERVER_ERR_DEVICE_FAILURE otherwise.
**
****************************************************************************************/
tTbxMbServerResult DemoApp::writeHoldingReg(uint16_t addr,
                                            uint16_t  value)
{
  tTbxMbServerResult result = TBX_MB_SERVER_OK;

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
} /*** end of writeHoldingReg ***/


extern "C"
{
  /**********************************************************************************//**
  ** \brief     Entry point for the demo application. Called by main() after the 
  **            initialization completed. 
  ** \details   This function needs C linkage because it is called from C code.
  **
  **************************************************************************************/
  void DemoMain(void)
  {
    DemoApp demoApp;

    /* Run the application. Note that this method call does not return. */
    demoApp.run();
  } /*** end of DemoMain ***/
} /* extern "C" */


/*********************************** end of serverrtu_cplusplus.c **********************/
