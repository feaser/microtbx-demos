/************************************************************************************//**
* \file         demos/base/dynamicarray.c
* \brief        Demonstrates how to use the memory pool and linked list modules to build
*               a dynamic array.
* \details      Usually when implementing an array, you need to give it a fixed size at
*               compile time. With the help of the linked list and memory pool modules,
*               you can build a dynamic array that bypasses this limitation. A dynamic
*               array is one that automatically increases in size and that can
*               be created (and deleted) at run-time.
*
*               The dynamic array presented in this demo application, is such that it:
*                 a) Enables you to create and delete arrays at run-time.
*                 b) Enables you to grow the array size automatically, as long as heap
*                    memory is available.
*
*               It is structured such that you can easily reuse the dynamic array
*               solution. You just need to copy the following functions into your own
*               code:
*                 - ArrayCreate()
*                 - ArrayDelete()
*                 - ArrayAdd()
*                 - ArrayRemove()
*                 - ArrayGet()
*                 - ArraySet()
*                 - ArrayCount()
*                 - ArrayFlush()
*
*               To demonstrate the use of these functions, the application first creates
*               a dynamic array and performs the following operations on it:
*                 - Add three items to the array.
*                 - Change an item at a specific index.
*                 - Swap the first and the last items.
*                 - Removing the first and last items.
*                 - Adding four more items to the array.
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
/** \brief Maximum number of bytes for the data of an array element. */
#define ARRAY_ELEMENT_DATA_LEN_MAX     (8U)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Layout of the data object to be stored in the array. */
typedef struct
{
  uint16_t id;
  uint8_t  len;
  uint8_t  data[ARRAY_ELEMENT_DATA_LEN_MAX];
} tArrayElement;

/** \brief Layout of the data object that groups all array related info. */
typedef struct
{
  tTbxList * listHandle;
  size_t     elementSize;
} tArrayCtx;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void        DemoDisplayElement(tArrayElement const * element);

tArrayCtx * ArrayCreate       (size_t                elementSize);

void        ArrayDelete       (tArrayCtx           * ctx);

uint8_t     ArrayAdd          (tArrayCtx           * ctx,
                               void          const * element);

uint8_t     ArrayRemove       (tArrayCtx           * ctx, 
                               size_t                idx);

uint8_t     ArrayGet          (tArrayCtx           * ctx, 
                               size_t                idx,
                               void                * element);

uint8_t     ArraySet          (tArrayCtx           * ctx,
                               size_t                idx,
                               void          const * element);

size_t      ArrayCount        (tArrayCtx           * ctx);

void        ArrayFlush        (tArrayCtx           * ctx);


/************************************************************************************//**
** \brief     Entry point for the demo application. Called by main() after the 
**            initialization completed. Contains the infinite program loop.
**
****************************************************************************************/
void DemoMain(void)
{
  tArrayCtx     * arrayCtx;
  uint8_t         arrayRes;
  size_t          idx;
  tArrayElement   element;
  tArrayElement   elementBackup;
  tArrayElement   elementA = {.id = 0x123, .len = 8, .data = { 0, 1, 2, 3, 4, 5, 6, 7 }};
  tArrayElement   elementB = {.id = 0x456, .len = 4, .data = { 0xFF, 0xEE, 0xDD, 0xCC }};
  tArrayElement   elementC = {.id = 0x789, .len = 2, .data = { 0xAA, 0x55 }};

  /* Create a new dynamic array. */
  printf("Creating a new dynamic array..");
  arrayCtx = ArrayCreate(sizeof(tArrayElement));
  TBX_ASSERT(arrayCtx != NULL);
  printf("[OK]\n");

  /* Add a few to the array. */
  printf("Adding a few items to the array..\n");
  arrayRes = ArrayAdd(arrayCtx, &elementA);
  TBX_ASSERT(arrayRes == TBX_OK);
  arrayRes = ArrayAdd(arrayCtx, &elementB);
  TBX_ASSERT(arrayRes == TBX_OK);
  arrayRes = ArrayAdd(arrayCtx, &elementC);
  TBX_ASSERT(arrayRes == TBX_OK);
  /* Display the current contents of the array. */
  for (idx = 0U; idx < ArrayCount(arrayCtx); idx++)
  {
    arrayRes = ArrayGet(arrayCtx, idx, &element);
    TBX_ASSERT(arrayRes == TBX_OK);
    printf("  [%d] ", idx);
    DemoDisplayElement(&element);
    printf("[OK]\n");
  }

  /* Changing the element at index 1. */
  printf("Changing the item at index 1..\n");
  element.id = 0xFFFF;
  element.len = 2;
  element.data[0] = 0xFF;
  element.data[1] = 0xFF;
  arrayRes = ArraySet(arrayCtx, 1U, &element);
  TBX_ASSERT(arrayRes == TBX_OK);
  /* Display the current contents of the array. */
  for (idx = 0U; idx < ArrayCount(arrayCtx); idx++)
  {
    arrayRes = ArrayGet(arrayCtx, idx, &element);
    TBX_ASSERT(arrayRes == TBX_OK);
    printf("  [%d] ", idx);
    DemoDisplayElement(&element);
    printf("[OK]\n");
  }

  /* Swap the first (index 0) and the last (index 2) elements. */
  printf("Swapping the first and the last items..\n");
  /* Backup element at index 2 before overwriting it. */
  arrayRes = ArrayGet(arrayCtx, 2U, &elementBackup);
  TBX_ASSERT(arrayRes == TBX_OK);
  /* Readout the element at index 0. */
  arrayRes = ArrayGet(arrayCtx, 0U, &element);
  TBX_ASSERT(arrayRes == TBX_OK);
  /* Store the element that is currently at index 0, at index 2. */
  arrayRes = ArraySet(arrayCtx, 2U, &element);
  TBX_ASSERT(arrayRes == TBX_OK);
  /* Store the element that was at index 2, at index 0. */
  arrayRes = ArraySet(arrayCtx, 0U, &elementBackup);
  TBX_ASSERT(arrayRes == TBX_OK);
  /* Display the current contents of the array. */
  for (idx = 0U; idx < ArrayCount(arrayCtx); idx++)
  {
    arrayRes = ArrayGet(arrayCtx, idx, &element);
    TBX_ASSERT(arrayRes == TBX_OK);
    printf("  [%d] ", idx);
    DemoDisplayElement(&element);
    printf("[OK]\n");
  }

  /* Remove the first and the last items. */
  printf("Removing the first and the last items..\n");
  /* Remove the first item. */
  arrayRes = ArrayRemove(arrayCtx, 0U);
  TBX_ASSERT(arrayRes == TBX_OK);
  /* Remove the last item. */
  arrayRes = ArrayRemove(arrayCtx, ArrayCount(arrayCtx) - 1U);
  TBX_ASSERT(arrayRes == TBX_OK);
  /* Display the current contents of the array. */
  for (idx = 0U; idx < ArrayCount(arrayCtx); idx++)
  {
    arrayRes = ArrayGet(arrayCtx, idx, &element);
    TBX_ASSERT(arrayRes == TBX_OK);
    printf("  [%d] ", idx);
    DemoDisplayElement(&element);
    printf("[OK]\n");
  }

  /* Add a few more items to the array. */
  printf("Adding four more items to the array..\n");
  for (idx = 1U; idx <= 4U; idx++)
  {
    element.id = idx;
    element.len = 1;
    element.data[0] = idx;
    arrayRes = ArrayAdd(arrayCtx, &element);
    TBX_ASSERT(arrayRes == TBX_OK);
  }
  /* Display the current contents of the array. */
  for (idx = 0U; idx < ArrayCount(arrayCtx); idx++)
  {
    arrayRes = ArrayGet(arrayCtx, idx, &element);
    TBX_ASSERT(arrayRes == TBX_OK);
    printf("  [%d] ", idx);
    DemoDisplayElement(&element);
    printf("[OK]\n");
  }

  /* Delete the entire array. */
  printf("Deleting the array..");
  ArrayDelete(arrayCtx);
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
void DemoDisplayElement(tArrayElement const * element)
{
  size_t len = ARRAY_ELEMENT_DATA_LEN_MAX;
  size_t idx;

  /* Verify the parameter. */
  TBX_ASSERT(element != NULL);

  /* Start with the identifier. */
  printf("id: %04Xh ", element->id);
  /* Store and sanitize the length. */
  if (element->len <= ARRAY_ELEMENT_DATA_LEN_MAX)
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
** \brief     Create a dynamic array at run-time. Think of it as the constructor.
** \param     elementSize The size of one array element.
**
****************************************************************************************/
tArrayCtx * ArrayCreate(size_t elementSize)
{
  tArrayCtx * result = NULL;
  tArrayCtx * newCtx;

  /* Verify parameter. */
  TBX_ASSERT(elementSize > 0U);

  /* Allocate memory for the new context from the memory pool. */
  newCtx = TbxMemPoolAllocate(sizeof(tArrayCtx));
  /* Automatically increase the memory pool if it was too small. */
  if (newCtx == NULL)
  {
    TbxMemPoolCreate(1U, sizeof(tArrayCtx));
    newCtx = TbxMemPoolAllocate(sizeof(tArrayCtx));
  }
  TBX_ASSERT(newCtx != NULL);

  /* Initialize the context. */
  newCtx->elementSize = elementSize;
  /* Ceate the linked list of the actual array elements. */
  newCtx->listHandle = TbxListCreate();
  TBX_ASSERT(newCtx->listHandle != NULL);

  /* Update the result. */
  if ((newCtx != NULL) && (newCtx->listHandle != NULL))
  {
    result = newCtx;
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of ArrayCreate ***/


/************************************************************************************//**
** \brief     Delete the array. Call this function when the array is no longer needed. It
**            releases the allocated memory back to the memory pools.
** \param     ctx Pointer to the array context.
**
****************************************************************************************/
void ArrayDelete(tArrayCtx * ctx)
{
  /* Verify the parameter. */
  TBX_ASSERT(ctx != NULL);

  /* Flush the buffer. This makes sure all memory that was allocated for its elements
   * is given back to the memory pool.
   */
  ArrayFlush(ctx);
  /* Delete the liked list of array elements. */
  TbxListDelete(ctx->listHandle);
  /* Delete the context, by giving it back to the memory pool. */
  TbxMemPoolRelease(ctx);
} /*** end of ArrayDelete ***/


/************************************************************************************//**
** \brief     Stores a copy of the element as a new array element all the way at the end.
**            Memory for the element is obtains from a memory pool
** \param     ctx Pointer to the array context.
** \param     element Pointer to the element to add to the array.
**
****************************************************************************************/
uint8_t ArrayAdd(tArrayCtx       * ctx, 
                 void      const * element)
{
  uint8_t         result = TBX_ERROR;
  void          * newElement;
  uint8_t const * srcPtr;
  uint8_t       * destPtr;
  size_t          dataIdx;

  /* Verify parameters. */
  TBX_ASSERT((ctx != NULL) && (element != NULL));

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
  for (dataIdx = 0; dataIdx < ctx->elementSize; dataIdx++)
  {
    destPtr[dataIdx] = srcPtr[dataIdx];
  }
  /* Insert the element at the end of the linked list. */
  result = TbxListInsertItemBack(ctx->listHandle, newElement);
  /* Give the result back to the caller. */
  return result;
} /*** end of ArrayAdd ***/


/************************************************************************************//**
** \brief     Removes the element at the specified index from the array.
** \param     ctx Pointer to the array context.
** \param     idx Index into the array of the element to remove.
** \return    TBX_OK if the element was removed. TBX_ERROR if there was no element at the
**            specified index.
**
****************************************************************************************/
uint8_t ArrayRemove(tArrayCtx * ctx, 
                    size_t      idx)
{
  uint8_t   result = TBX_ERROR;
  void    * currentElement;
  size_t    currentIdx = 0U;
  uint8_t   foundElement = TBX_FALSE;

  /* Verify parameter. */
  TBX_ASSERT(ctx != NULL);

  /* Iterate over the list to get to the element with the specified index. */
  currentElement = TbxListGetFirstItem(ctx->listHandle);
  while (currentElement != NULL)
  {
    /* Is this at the specified index? */
    if (currentIdx == idx)
    {
      /* Set flag that we found the element and stop the loop. */
      foundElement = TBX_TRUE;
      break;
    }
    /* Continue with the next element. */
    currentElement = TbxListGetNextItem(ctx->listHandle, currentElement);
    currentIdx++;
  }
  /* Only continue with the removal, if the element at the specified index was found. */
  if (foundElement == TBX_TRUE)
  {
    /* Remove it from the linked list. */
    TbxListRemoveItem(ctx->listHandle, currentElement);
    /* Give the allocate memory back to the pool. */
    TbxMemPoolRelease(currentElement);
    /* Update the result. */
    result = TBX_OK;
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of ArrayRemove ***/


/************************************************************************************//**
** \brief     Reads the element at the specified index from the array.
** \param     ctx Pointer to the array context.
** \param     idx Index into the array of the element to read.
** \return    TBX_OK if the element was read. TBX_ERROR if there was no element at the
**            specified index.
**
****************************************************************************************/
uint8_t ArrayGet(tArrayCtx * ctx, 
                 size_t      idx,
                 void      * element)
{
  uint8_t         result = TBX_ERROR;
  void          * currentElement;
  size_t          currentIdx = 0U;
  uint8_t         foundElement = TBX_FALSE;
  uint8_t const * srcPtr;
  uint8_t       * destPtr;
  size_t          dataIdx;

  /* Verify parameters. */
  TBX_ASSERT((ctx != NULL) && (element != NULL));

  /* Iterate over the list to get to the element with the specified index. */
  currentElement = TbxListGetFirstItem(ctx->listHandle);
  while (currentElement != NULL)
  {
    /* Is this at the specified index? */
    if (currentIdx == idx)
    {
      /* Set flag that we found the element and stop the loop. */
      foundElement = TBX_TRUE;
      break;
    }
    /* Continue with the next element. */
    currentElement = TbxListGetNextItem(ctx->listHandle, currentElement);
    currentIdx++;
  }
  /* Only continue with the reading, if the element at the specified index was found. */
  if (foundElement == TBX_TRUE)
  {
    /* Copy the element to the caller's provided storage. */
    srcPtr = currentElement;
    destPtr = element;
    for (dataIdx = 0; dataIdx < ctx->elementSize; dataIdx++)
    {
      destPtr[dataIdx] = srcPtr[dataIdx];
    }
    /* Update the result. */
    result = TBX_OK;
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of ArrayGet ***/


/************************************************************************************//**
** \brief     Changes the contents of the element at the specified index in the array.
** \param     ctx Pointer to the array context.
** \param     idx Index into the array of the element to modify.
** \return    TBX_OK if the element was set. TBX_ERROR if there was no element at the
**            specified index.
**
****************************************************************************************/
uint8_t ArraySet(tArrayCtx       * ctx, 
                size_t             idx, 
                void       const * element)
{
  uint8_t         result = TBX_ERROR;
  void          * currentElement;
  size_t          currentIdx = 0U;
  uint8_t         foundElement = TBX_FALSE;
  uint8_t const * srcPtr;
  uint8_t       * destPtr;
  size_t          dataIdx;

  /* Verify parameters. */
  TBX_ASSERT((ctx != NULL) && (element != NULL));

  /* Iterate over the list to get to the element with the specified index. */
  currentElement = TbxListGetFirstItem(ctx->listHandle);
  while (currentElement != NULL)
  {
    /* Is this at the specified index? */
    if (currentIdx == idx)
    {
      /* Set flag that we found the element and stop the loop. */
      foundElement = TBX_TRUE;
      break;
    }
    /* Continue with the next element. */
    currentElement = TbxListGetNextItem(ctx->listHandle, currentElement);
    currentIdx++;
  }
  /* Only continue with the writing, if the element at the specified index was found. */
  if (foundElement == TBX_TRUE)
  {
    /* Update the element's content. */
    srcPtr = element;
    destPtr = currentElement;
    for (dataIdx = 0; dataIdx < ctx->elementSize; dataIdx++)
    {
      destPtr[dataIdx] = srcPtr[dataIdx];
    }
    /* Update the result. */
    result = TBX_OK;
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of ArraySet ***/


/************************************************************************************//**
** \brief     Obtains the count of elements currently stored in the array.
** \param     ctx Pointer to the array context.
** \return    Element count.
**
****************************************************************************************/
size_t ArrayCount(tArrayCtx * ctx)
{
  size_t result = TBX_ERROR;

  /* Verify parameter. */
  TBX_ASSERT(ctx != NULL);

  /* Obtain the count of elements that are currently stored in the linked list. */
  result = TbxListGetSize(ctx->listHandle);
  /* Give the result back to the caller. */
  return result;
} /*** end of ArrayCount ***/


/************************************************************************************//**
** \brief     Flushes the array by clearing all its elements.
** \param     ctx Pointer to the array context.
**
****************************************************************************************/
void ArrayFlush(tArrayCtx * ctx)
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
} /*** end of ArrayFlush ***/


/*********************************** end of dynamicarray.c *****************************/
