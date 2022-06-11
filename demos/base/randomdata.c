/************************************************************************************//**
* \file         demos/base/randomdata.c
* \brief        Demonstrates how to use the random number generator to fill a data buffer
*               with random data.
* \details      The application fills two different data buffers with random data:
*                 1. A byte array declared with a fixed size at compile time.
*                 2. A byte array dynamically allocated on the heap.
*               The benefit of the 2nd approach is that you can size the data array at
*               run-time.
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2022 by Feaser     www.feaser.com     All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
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
#include "bsp.h"                                 /* Board support package header.      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Size of the buffer for storing randomly generated data. */
#define DEMO_BUFFER_SIZE    (32U)


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static uint32_t DemoSeedInitHandler(void);
static void     DemoDisplayData(uint8_t const * data, size_t len, size_t bytesPerLine);


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Buffer for storing randomly generated data. Its size is fixed at compile
 *         time.
 */
static uint8_t compileTimeBuffer[DEMO_BUFFER_SIZE];

/** \brief Buffer for storing randomly generated data. Memory for the buffer is
 *         dynamically allocated from the heap at run-time.
 */
static uint8_t * runTimeBuffer;


/************************************************************************************//**
** \brief     Entry point for the demo application. Called by main() after the 
**            initialization completed. Contains the infinite program loop.
**
****************************************************************************************/
void DemoMain(void)
{
  uint32_t * dataPtr;
  size_t     idx;

  /* Register the application specific seed initialization handler.*/
  TbxRandomSetSeedInitHandler(DemoSeedInitHandler);

  /* Fill the compile-time allocated buffer with random data, 32 bits at a time. */
  dataPtr = (uint32_t *)compileTimeBuffer;
  for (idx = 0; idx < (DEMO_BUFFER_SIZE/sizeof(uint32_t)); idx++)
  {
    dataPtr[idx] = TbxRandomNumberGet();
  }
  /* Display the generated data on the standard output. */
  printf("\nRandom data stored in the compile-time allocated buffer:\n");
  DemoDisplayData(compileTimeBuffer, DEMO_BUFFER_SIZE, 16U);

  /* Allocate memory for the run-time allocated buffer from the heap. The benefit of this
   * approach is that you can configure its size at run-time. Note that you cannot free
   * the memory using the heap module. However, such functionality is possible by using
   * the memory pool module.
   */
  runTimeBuffer = TbxHeapAllocate(DEMO_BUFFER_SIZE);
  /* Assert memory allocation. If it failed, increase the heap with configuration macro
   * TBX_CONF_HEAP_SIZE in "tbx_conf.h".
   */
  TBX_ASSERT(runTimeBuffer != NULL);
  /* Fill the run-time allocated buffer with random data, 32 bits at a time. */
  dataPtr = (uint32_t *)runTimeBuffer;
  for (idx = 0; idx < (DEMO_BUFFER_SIZE/sizeof(uint32_t)); idx++)
  {
    dataPtr[idx] = TbxRandomNumberGet();
  }
  /* Display the generated data on the standard output. */
  printf("\nRandom data stored in the run-time allocated buffer:\n");
  DemoDisplayData(runTimeBuffer, DEMO_BUFFER_SIZE, 16U);

  /* Enter infinite program loop. */
  while (1)
  {
    /* Nothing more to do for this particular demo. */
    ;  
  }
} /*** end of DemoMain ***/


/************************************************************************************//**
** \brief     Handler function that gets called by the random number generator. This
**            module requires a seed, which this function should obtain. The actual value
**            is not really important as long as it is a value that is different every
**            time the software program runs, so after each reset event.
** \return    The 32-bit value that the random number generator module uses as a seed to
**            initialize itself.
**
****************************************************************************************/
static uint32_t DemoSeedInitHandler(void)
{
  /* Pass the request on to the board support package. */
  return BspGetSeed();
} /*** end of DemoSeedInitHandler ***/


/************************************************************************************//**
** \brief     Displays the data as hexadecimal characters on the standard output.
** \param     data Pointer to the data bytes to display.
** \param     len Total number of bytes in the data buffer that should be displayed.
** \param     bytesPerLine Number of bytes to display on one line.
**
****************************************************************************************/
static void DemoDisplayData(uint8_t const * data, size_t len, size_t bytesPerLine)
{
  size_t idx;

  /* Verify parameters. */
  TBX_ASSERT((data != NULL) && (len > 0U) && (bytesPerLine > 0U));

  /* Only continue with valid parameters. */
  if ((data != NULL) && (len > 0U) && (bytesPerLine > 0U))
  {
    /* Loop through the bytes. */
    for (idx = 0; idx < len; idx++)
    {
      /* Display the byte as a hexadecimal comprising of 2 characters. */
      printf("%02X ", data[idx]);
      /* Time to move on to the next line? */
      if (((idx + 1) % bytesPerLine) == 0U)
      {
        /* Move on to the next line. */
        printf("\n");
      }
    }
    /* Final new line character needed? */
    if ((idx % bytesPerLine) != 0U)
    {
      /* Move on to the next line. */
      printf("\n");
    }
  }
} /*** end of DemoDisplayData ***/


/*********************************** end of randomdata.c *******************************/
