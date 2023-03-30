/************************************************************************************//**
* \file         demos/modbus/clientrtu_cplusplus.c
* \brief        Demonstrates how to setup a Modbus RTU client in C++.
* \details      This demo application creates a Modbus client, configured for RTU
*               communication at 19200 bits/sec, 8 databits, even parity and 1 stopbit.
*
*               The DemoApp class derives from class TbxMbClientRtu.
*
*               A traditional infinite loop, a.k.a "superloop" drives the application and
*               the Modbus communication stack. Note this it does not call 
*               TbxMbEvent::task(). On a Modbus client in combination with a superloop
*               applications, all communication with the server happens in a blocking
*               manner. The TbxMbEvent::task() method is called internally while
*               blocking. Convenient and easy, but not optimal from a run-time
*               performance. For this reason it is recommended to use an RTOS in
*               combination with a Modbus client.
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
#include "microtbx.h"                            /* MicroTBX library                   */
#include "microtbxmodbus.hpp"                    /* MicroTBX-Modbus C++ library        */
#include "bsp.h"                                 /* Board support package              */


/****************************************************************************************
* Class definitions
****************************************************************************************/
/** \brief Demo application class. */
class DemoApp : public TbxMbClientRtu
{
public:
  /* Constructors and destructor. */
  DemoApp()
    : TbxMbClientRtu(400U, 100U, TBX_MB_UART_PORT2, TBX_MB_UART_19200BPS, 
                     TBX_MB_UART_1_STOPBITS, TBX_MB_EVEN_PARITY) { }
  
  virtual ~DemoApp() { }
  /* Methods. */
  void run();
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
  uint16_t const toggleTicksNormal  =  500000U / 50U;    /* 20 kHz = 50 usec per tick. */
  uint16_t const toggleTicksCommErr = 2000000U / 50U;
  uint16_t       ledToggleTicks     = toggleTicksNormal;
  uint16_t       toggleTicksLast    = 0U;
  uint16_t       currentTicks;
  uint16_t       deltaTicks;
  uint8_t        ledState           = TBX_OFF;
  uint16_t       inputRegs[2]       = { 0 };
  uint8_t        inputs[2]          = { 0 };
  uint8_t        coils[2]           = { TBX_OFF, TBX_OFF };
  uint16_t       holdingRegs[2]     = { 63U, 127U };


  TbxMbClientRtu modbusClient(1000U, 100U, TBX_MB_UART_PORT1, TBX_MB_UART_19200BPS, 
                              TBX_MB_UART_1_STOPBITS, TBX_MB_EVEN_PARITY);

  modbusClient.writeCoils(0x0AU, 0U, 2U, coils);

  /* Write coils. */
  writeCoils(0x0AU, 0U, 2U, coils);

  /* Read discrete inputs. */
  readInputs(0x0AU, 10000U, 2U, inputs);

  /* Read input registers. */
  readInputRegs(0x0AU, 30000U, 2U, inputRegs);

  /* Write holding registers. */
  writeHoldingRegs(0x0AU, 40000U, 2U, holdingRegs);

  /* Enter the infinite program loop. */
  for (;;)
  {
    /* Calculate how many 50 us ticks passed since the last time the LED toggled. Note
     * that this also works in case the 20 kHz timer value overflowed, thanks to integer
     * math.
     */
    currentTicks = TbxMbPort::timerCount();
    deltaTicks = currentTicks - toggleTicksLast;
    /* Time to toggle the LED? */
    if ((deltaTicks) >= ledToggleTicks)
    {
      /* Store the current toggle ticks, to help determining the next time to toggle. */
      toggleTicksLast = currentTicks;
      /* Toggle the LED. */
      ledState = (ledState == TBX_ON) ? TBX_OFF : TBX_ON;
      BspDigitalOut(BSP_DIGITAL_OUT1, ledState);
      /* Write LED state to the first coil. */
      coils[0] = ledState;
      uint8_t commResult = writeCoils(0x0AU, 0U, 1U, coils);
      /* Update LED toggle interval based on the communication result. In case of a 
       * communication error, toggle the LED at a lower rate.
       */
      ledToggleTicks = (commResult == TBX_OK) ? toggleTicksNormal : toggleTicksCommErr;
    }    
  }
} /*** end of run ***/


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


/*********************************** end of clientrtu_cplusplus.c **********************/
