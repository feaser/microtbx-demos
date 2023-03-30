/************************************************************************************//**
* \file         demos/modbus/ARM_CORTEXM_ST_Nucleo_F091RC/main.c
* \brief        Program entry point source file.
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
#include <stdio.h>                               /* C standard input/output            */
#include "microtbx.h"                            /* MicroTBX library                   */
#include "microtbxmodbus.h"                      /* MicroTBX-Modbus library            */
#include "bsp.h"                                 /* Board support package              */


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void AppAssertionHandler(char const * const file,
                                uint32_t           line);

extern void DemoMain           (void);


/************************************************************************************//**
** \brief     This is the entry point for the software application and is called
**            by the reset interrupt vector after the C-startup routines executed.
** \return    Program exit code.
**
****************************************************************************************/
int main(void)
{
  /* Register the application specific assertion handler. */
  TbxAssertSetHandler(AppAssertionHandler);
  /* Initialize the board support package. */
  BspInit();
  /* Hand control over to the demo application. */
  DemoMain();
  /* Previous function is not expected to return. */
  TBX_ASSERT(TBX_FALSE);
  /* Set program exit code. Note that the program should never get here. */
  return 0;
} /*** end of main ***/


/************************************************************************************//**
** \brief     Application specific assertion handler, configured with 
**            TbxAssertSetHandler().
** \param     file The filename of the source file where the assertion occurred in.
** \param     line The line number inside the file where the assertion occurred.
**
****************************************************************************************/
static void AppAssertionHandler(char const * const file, 
                                uint32_t           line)
{
  TBX_UNUSED_ARG(file);
  TBX_UNUSED_ARG(line);

  /* Make sure interrupts are disabled to stop the program. */
  TbxCriticalSectionEnter();

  /* Hang the program by entering an infinite loop. The values for file and line can
   * then be inspected with the debugger to locate the source of the run-time assertion.
   */
  for (;;)
  {
    ;
  }

  /* Code never gets here. Just added to emphasize that TbxCriticalSectionEnter() and 
   * TbxCriticalSectionExit() should always be used pairwise. 
   */
  TbxCriticalSectionExit();
} /*** end of AppAssertionHandler ***/


/*********************************** end of main.c *************************************/
