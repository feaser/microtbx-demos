/************************************************************************************//**
* \file         demos/base/fifobuffer.c
* \brief        Demonstrates how to use the memory pool and linked list modules to build
*               a first-in-first-out (FIFO) buffer.
* \details      Usually when implementing a FIFO buffer, you need to give the array, for
*               storing its data elements, a fixed size at compile-time. This limitation
*               no longer applies, when building a FIFO buffer based on the linked list
*               and memory pool modules of MicroTBX.
*
*               This demo implements a FIFO buffer solution that:
*                 a) Enables you to create and delete FIFO buffers at run-time.
*                 b) Enables you to specify the size of the FIFO buffer at run-time.
*                 c) Enables you to create a FIFO buffer of variable size that
*                    automatically grows in size as needed, as long as heap memory is
*                    available.
*
*               It is structured such that you can easily reuse the FIFO buffer solution.
*               You just need to copy the following functions into your own code:
*                 - FifoBufferCreate()
*                 - FifoBufferDelete()
*                 - FifoBufferStore()
*                 - FifoBufferRetrieve()
*                 - FifoBufferCount()
*                 - FifoBufferFlush()
*
*               The demo application first creates a FIFO buffer with a fixed size of 3
*               elements. It fills it with data elements and then retrieves all data
*               elements again, allowing you to verify that they are indeed in the
*               correct first-in-first-out order. Once done, the FIFO buffer is deleted.
*               Next, a same logic is repeated but then for a variable sized FIFO buffer.
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2022 by Feaser     www.feaser.com     All rights reserved
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
#include "bsp.h"                                 /* Board support package header.      */


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Maximum number of bytes for the data of a FIFO element. */
#define FIFO_ELEMENT_DATA_LEN_MAX      (8U)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Layout of the data object to be stored in the FIFO buffer. */
typedef struct
{
  uint16_t id;
  uint8_t  len;
  uint8_t  data[FIFO_ELEMENT_DATA_LEN_MAX];
} tFifoElement;

/** \brief Layout of the data object that groups all FIFO buffer related info. */
typedef struct
{
  tTbxList * listHandle;
  size_t     maxSize;
  size_t     elementSize;
} tFifoCtx;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void       DemoDisplayElement(tFifoElement const * element);

tFifoCtx * FifoBufferCreate  (size_t               maxSize, 
                              size_t               elementSize);

void       FifoBufferDelete  (tFifoCtx           * ctx);

uint8_t    FifoBufferStore   (tFifoCtx           * ctx, 
                              void         const * element);

uint8_t    FifoBufferRetrieve(tFifoCtx           * ctx,
                              void               * element);

size_t     FifoBufferCount   (tFifoCtx           * ctx);

void       FifoBufferFlush   (tFifoCtx           * ctx);


/************************************************************************************//**
** \brief     Entry point for the demo application. Called by main() after the 
**            initialization completed. Contains the infinite program loop.
**
****************************************************************************************/
void DemoMain(void)
{
  tFifoCtx     * bufferCtx;
  uint8_t        bufferRes;
  size_t         freeHeap;
  tFifoElement   element;
  tFifoElement   element1 = {.id = 0x123, .len = 8, .data = { 0, 1, 2, 3, 4, 5, 6, 7 }};
  tFifoElement   element2 = {.id = 0x456, .len = 4, .data = { 0xFF, 0xEE, 0xDD, 0xCC }};
  tFifoElement   element3 = {.id = 0x789, .len = 2, .data = { 0xAA, 0x55 }};
  tFifoElement   element4 = {.id = 0xABC, .len = 3, .data = { 0x11, 0x22, 0x33 }};

  /* ------------------ Fixed size FIFO buffer --------------------------------------- */
  /* Create a new FIFO buffer that can hold up to three elements. */
  printf("\n");
  printf("-------------------------------------------------\n");
  printf("*       Fixed size FIFO buffer                  *\n");
  printf("-------------------------------------------------\n");
  printf("Creating a new FIFO buffer of 3 elements..");
  bufferCtx = FifoBufferCreate(3U, sizeof(tFifoElement));
  TBX_ASSERT(bufferCtx != NULL);
  printf("[OK]\n");

  /* Fill the FIFO buffer with three elements. */
  printf("Adding 3 elements..\n");
  /* One. */
  bufferRes = FifoBufferStore(bufferCtx, &element1);
  TBX_ASSERT(bufferRes == TBX_OK);
  DemoDisplayElement(&element1);
  printf("[OK]\n");
  /* Two. */
  bufferRes = FifoBufferStore(bufferCtx, &element2);
  TBX_ASSERT(bufferRes == TBX_OK);
  DemoDisplayElement(&element2);
  printf("[OK]\n");
  /* Three. */
  bufferRes = FifoBufferStore(bufferCtx, &element3);
  TBX_ASSERT(bufferRes == TBX_OK);
  DemoDisplayElement(&element3);
  printf("[OK]\n");

  /* Add a fourth element, which should fail, because the max size is 3. */
  printf("Add a 4th element, which should fail..");
  bufferRes = FifoBufferStore(bufferCtx, &element4);
  TBX_ASSERT(bufferRes == TBX_ERROR);
  printf("[OK]\n");

  /* Retrieve the elements one-by-one. */
  printf("Retrieving all elements..\n");
  while (FifoBufferCount(bufferCtx) > 0U)
  {
    /* Get the oldest element currently stored in the buffer. */
    bufferRes = FifoBufferRetrieve(bufferCtx, &element);
    TBX_ASSERT(bufferRes == TBX_OK);
    DemoDisplayElement(&element);
    printf("[OK]\n");
  }

  /* Delete the FIFO buffer now that we no longer need it. */
  printf("Deleting the FIFO buffer..");
  FifoBufferDelete(bufferCtx);
  printf("[OK]\n");

  /* ------------------ Variable size FIFO buffer ------------------------------------ */
  /* Store the current number of free bytes on the heap. */
  freeHeap = TbxHeapGetFree();
  printf("\n");
  printf("-------------------------------------------------\n");
  printf("*       Variable size FIFO buffer               *\n");
  printf("-------------------------------------------------\n");
  printf("Creating a new FIFO buffer of variable size..");
  bufferCtx = FifoBufferCreate(0U, sizeof(tFifoElement));
  TBX_ASSERT(bufferCtx != NULL);
  printf("[OK]\n");

  /* Fill the FIFO buffer with three elements. */
  printf("Adding 3 elements..\n");
  /* One. */
  bufferRes = FifoBufferStore(bufferCtx, &element1);
  TBX_ASSERT(bufferRes == TBX_OK);
  DemoDisplayElement(&element1);
  printf("[OK]\n");
  /* Two. */
  bufferRes = FifoBufferStore(bufferCtx, &element2);
  TBX_ASSERT(bufferRes == TBX_OK);
  DemoDisplayElement(&element2);
  printf("[OK]\n");
  /* Three. */
  bufferRes = FifoBufferStore(bufferCtx, &element3);
  TBX_ASSERT(bufferRes == TBX_OK);
  DemoDisplayElement(&element3);
  printf("[OK]\n");

  /* Sanity check on the memory allocation from the memory pools. At this point as many
   * elements were stored as in the previous example with the fixed size FIFO buffer.
   * This means that it should have been possible to fully recycle memory from the pools.
   * With other words, no new memory should have been allocated from the heap by both
   * the linked list and memory pool modules.
   */
  TBX_ASSERT(TbxHeapGetFree() == freeHeap);

  /* Add a fourth element, which should now work, because it has a variable size. */
  printf("Add a 4th element, which should now work..\n");
  bufferRes = FifoBufferStore(bufferCtx, &element4);
  TBX_ASSERT(bufferRes == TBX_OK);
  DemoDisplayElement(&element4);
  printf("[OK]\n");

  /* Sanity check on the memory pool allocation. For storing the fourth element, both
   * the linked list and memory pool modules should have needed new memory from the
   * heap.
   */
  TBX_ASSERT(TbxHeapGetFree() < freeHeap);

  /* Retrieve the elements one-by-one. */
  printf("Retrieving all elements..\n");
  while (FifoBufferCount(bufferCtx) > 0U)
  {
    /* Get the oldest element currently stored in the buffer. */
    bufferRes = FifoBufferRetrieve(bufferCtx, &element);
    TBX_ASSERT(bufferRes == TBX_OK);
    DemoDisplayElement(&element);
    printf("[OK]\n");
  }

  /* Delete the FIFO buffer now that we no longer need it. */
  printf("Deleting the FIFO buffer..");
  FifoBufferDelete(bufferCtx);
  printf("[OK]\n");

  /* Enter infinite program loop. */
  while (1)
  {
    /* Nothing more to do for this particular demo. */
    ;  
  }
} /*** end of DemoMain ***/


/************************************************************************************//**
** \brief     Displays the element contents on the standard output.
** \param     element Pointer to the element to display.
**
****************************************************************************************/
void DemoDisplayElement(tFifoElement const * element)
{
  size_t len = FIFO_ELEMENT_DATA_LEN_MAX;
  size_t idx;

  /* Verify the parameter. */
  TBX_ASSERT(element != NULL);

  /* Start with the identifier. */
  printf("  id: %04Xh ", element->id);
  /* Store and sanitize the length. */
  if (element->len <= FIFO_ELEMENT_DATA_LEN_MAX)
  {
    len = element->len;
  }
  /* Display the data bytes. */
  for (idx = 0; idx < len; idx++)
  {
    /* Display the byte as a hexadecimal comprising of 2 characters. */
    printf("%02X ", element->data[idx]);
  }
} /*** end of DemoDisplayElement ***/


/************************************************************************************//**
** \brief     Create the FIFO buffer. All other FifoBufferXxx() functions need this
**            context.
** \param     maxSize Maximum amount of elements that the FIFO buffer is allowed to
**            store. Set it to zero to allow the FIFO buffer to automatically grow as
**            needed.
** \param     elementSize The size of one element.
** \return    Pointer to the newly created context if successful, NULL otherwise.
**
****************************************************************************************/
tFifoCtx * FifoBufferCreate(size_t maxSize, 
                            size_t elementSize)
{
  tFifoCtx * result = NULL;
  tFifoCtx * newCtx;

  /* Allocate memory for the new context from the memory pool. */
  newCtx = TbxMemPoolAllocate(sizeof(tFifoCtx));
  /* Automatically increase the memory pool if it was too small. */
  if (newCtx == NULL)
  {
    TbxMemPoolCreate(1U, sizeof(tFifoCtx));
    newCtx = TbxMemPoolAllocate(sizeof(tFifoCtx));
  }
  TBX_ASSERT(newCtx != NULL);

  /* Initialize the context. Start by storing its maximum size and the element size. */
  newCtx->maxSize = maxSize;
  newCtx->elementSize = elementSize;
  /* Ceate the linked list of the actual buffer elements. */
  newCtx->listHandle = TbxListCreate();
  TBX_ASSERT(newCtx->listHandle != NULL);

  /* Update the result. */
  if ((newCtx != NULL) && (newCtx->listHandle != NULL))
  {
    result = newCtx;
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of FifoBufferCreate ***/


/************************************************************************************//**
** \brief     Delete the FIFO buffer. Call this function when the FIFO buffer is no
**            longer needed. It releases the allocated memory back to the memory pools.
** \param     ctx Pointer to the FIFO buffer context.
**
****************************************************************************************/
void FifoBufferDelete(tFifoCtx * ctx)
{
  /* Verify the parameter. */
  TBX_ASSERT(ctx != NULL);

  /* Flush the buffer. This makes sure all memory that was allocated for its elements
   * is given back to the memory pool.
   */
  FifoBufferFlush(ctx);
  /* Delete the liked list of buffer elements. */
  TbxListDelete(ctx->listHandle);
  /* Delete the context, by giving it back to the memory pool. */
  TbxMemPoolRelease(ctx);
} /*** end of FifoBufferDelete ***/


/************************************************************************************//**
** \brief     Stores a copy of the element in the linked list. To make the buffer an
**            actual FIFO type buffer, the element is stored all the way at the back
**            of the linked list. Memory for the element is obtained from a memory pool.
** \param     ctx Pointer to the FIFO buffer context.
** \param     element Pointer to the element to store in the FIFO buffer.
** \return    TBX_OK if the element could be stored. TBX_ERROR if the buffer is full.
**
****************************************************************************************/
uint8_t FifoBufferStore(tFifoCtx       * ctx, 
                        void     const * element)
{
  uint8_t         result    = TBX_ERROR;
  uint8_t         stillRoom = TBX_TRUE;
  void          * newElement;
  uint8_t const * srcPtr;
  uint8_t       * destPtr;
  size_t          idx;

  /* Verify the parameters. */
  TBX_ASSERT((ctx != NULL) && (element != NULL));

  /* Check if the FIFO buffer is full, in case of a fixed size FIFO buffer. */
  if ((ctx->maxSize > 0U) && (FifoBufferCount(ctx) == ctx->maxSize))
  {
    stillRoom = TBX_FALSE;
  }
  /* Only continue if there is still room available in the FIFO buffer. */
  if (stillRoom == TBX_TRUE)
  {
    /* Obtain memory to store the new element from the memory pool. */
    newElement = TbxMemPoolAllocate(ctx->elementSize);
    /* Automatically increase the memory pool if it was too small. */
    if (newElement == NULL)
    {
      TbxMemPoolCreate(1U, ctx->elementSize);
      newElement = TbxMemPoolAllocate(ctx->elementSize);
    }
    TBX_ASSERT(newElement != NULL);
    /* Copy the element. */
    srcPtr = element;
    destPtr = newElement;
    for (idx = 0; idx < ctx->elementSize; idx++)
    {
      destPtr[idx] = srcPtr[idx];
    }
    /* Insert the element at the end of the linked list. */
    result = TbxListInsertItemBack(ctx->listHandle, newElement);
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of FifoBufferStore ***/


/************************************************************************************//**
** \brief     Obtains an element from the linked list. To make the buffer an actual FIFO
**            type buffer, the element is retrieved from the front of the linked list.
**            Afterwards, memory for the element release back to the memory pool for
**            future reuse.
** \param     ctx Pointer to the FIFO buffer context.
** \param     element Pointer to where the data of the retrieved element is written to.
** \return    TBX_OK if an element was retrieved. TBX_ERROR if the buffer was empty.
**
****************************************************************************************/
uint8_t FifoBufferRetrieve(tFifoCtx * ctx, 
                           void     * element)
{
  uint8_t result = TBX_ERROR;
  void * currentElement;
  uint8_t const * srcPtr;
  uint8_t * destPtr;
  size_t idx;

  /* Verify the parameters. */
  TBX_ASSERT((ctx != NULL) && (element != NULL));

  /* Obtain the elment at the start of the linked list (oldest). */
  currentElement = TbxListGetFirstItem(ctx->listHandle);
  if (currentElement != NULL)
  {
    /* Delete it from the linked list, now that we read it. */
    TbxListRemoveItem(ctx->listHandle, currentElement);
    /* Copy the element to the caller's provided storage. */
    srcPtr = currentElement;
    destPtr = element;
    for (idx = 0; idx < ctx->elementSize; idx++)
    {
      destPtr[idx] = srcPtr[idx];
    }
    /* Give the allocate memory back to the pool. */
    TbxMemPoolRelease(currentElement);
    /* Update the result. */
    result = TBX_OK;
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of FifoBufferRetrieve ***/


/************************************************************************************//**
** \brief     Obtains the count of elements currently stored in the FIFO buffer.
** \param     ctx Pointer to the FIFO buffer context.
** \return    Element count.
**
****************************************************************************************/
size_t FifoBufferCount(tFifoCtx * ctx)
{
  size_t result;

  /* Verify the parameter. */
  TBX_ASSERT(ctx != NULL);

  /* Obtain the count of elements that are currently stored in the linked list. */
  result = TbxListGetSize(ctx->listHandle);
  /* Give the result back to the caller. */
  return result;
} /*** end of FifoBufferCount ***/


/************************************************************************************//**
** \brief     Flushes the FIFO buffer by clearing all its elements.
** \param     ctx Pointer to the FIFO buffer context.
**
****************************************************************************************/
void FifoBufferFlush(tFifoCtx * ctx)
{
  void * currentElement;

  /* Verify the parameter. */
  TBX_ASSERT(ctx != NULL);

  /* Empty the list one element at a time. */
  currentElement = TbxListGetFirstItem(ctx->listHandle);
  while (currentElement != NULL)
  {
    /* Delete it from the linked list. */
    TbxListRemoveItem(ctx->listHandle, currentElement);
    /* Give the allocated memory back to the pool. */
    TbxMemPoolRelease(currentElement);
    /* Continue with the next item in the list, if any. */
    currentElement = TbxListGetFirstItem(ctx->listHandle);
  }
} /*** end of FifoBufferFlush ***/


/*********************************** end of fifobuffer.c *******************************/
