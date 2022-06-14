/************************************************************************************//**
* \file         demos/base/sortlist.c
* \brief        Demonstrates how to sort items in a linked list.
* \details      The demo application creates a linked list and fills it with three data
*               elements. The contents of a data element can be defined by the user. In
*               this case, an element consists of an identifier field, a data length
*               field and an array of data bytes.
*
*               This demo implements two data element comparison functions that can be
*               passed as a parameter to TbxListSortItems() for sorting the contents of
*               the linked list:
*                 - DemoCompareElementByAscId
*                 - DemoCompareElementByDescId
*
*               The first one compares the identifier fields of to elements such that
*               TbxListSortItems() can sort the list by ascending order of the
*               identifiers.
*               The second one does the opposite, enabling you to sort the list by
*               descending order of the identifiers.
*
*               After filling the list with data, the data is first sorted by ascending
*               identifiers. The resulting list contents is sent to the standard output
*               for verification purposes. Next, the date is sorted by descending
*               identifiers.
*
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
/** \brief Maximum number of bytes for the data of a list element. */
#define LIST_ELEMENT_DATA_LEN_MAX      (8U)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Layout of the data object to be stored in the linked list. */
typedef struct
{
  uint16_t id;
  uint8_t  len;
  uint8_t  data[LIST_ELEMENT_DATA_LEN_MAX];
} tListElement;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
uint8_t DemoCompareElementByAscId(void const * item1, void const * item2);
uint8_t DemoCompareElementByDescId(void const * item1, void const * item2);
void    DemoDisplayElement(tListElement const * element);


/************************************************************************************//**
** \brief     Entry point for the demo application. Called by main() after the 
**            initialization completed. Contains the infinite program loop.
**
****************************************************************************************/
void DemoMain(void)
{
  tTbxList * dataList;
  tListElement * dataElement;
  tListElement dataEntries[] =
  {
    { .id = 0x200, .len = 8, .data = { 1, 2, 3, 4, 5, 6, 7, 8 }  },
    { .id = 0x300, .len = 4, .data = { 0xFF, 0xEE, 0xDD, 0xCC }  },
    { .id = 0x100, .len = 6, .data = { 0xAA, 0x55, 0x11, 0x22, 0x33, 0x44 }  },
  };
  size_t idx = 0U;
  uint8_t listRes;

  /* Create a new linked list. */
  printf("Creating a new linked list..");
  dataList = TbxListCreate();
  TBX_ASSERT(dataList != NULL);
  printf("[OK]\n");

  /* Fill the list with pointers to the data entries. */
  printf("Filling the list with data entries..\n");
  while (idx < (sizeof(dataEntries)/sizeof(dataEntries[0])))
  {
    listRes = TbxListInsertItemBack(dataList, &dataEntries[idx]);
    TBX_ASSERT(listRes == TBX_OK);
    DemoDisplayElement(&dataEntries[idx]);
    printf("[OK]\n");
    idx++;
  }

  /* Sort the list by ascending order of the element identifier. */
  printf("Sorting the list by ascending ID..\n");
  TbxListSortItems(dataList, DemoCompareElementByAscId);
  dataElement = TbxListGetFirstItem(dataList);
  while (dataElement != NULL)
  {
    DemoDisplayElement(dataElement);
    printf("[OK]\n");
    dataElement = TbxListGetNextItem(dataList, dataElement);
  }

  /* Sort the list by descending order of the element identifier. */
  printf("Sorting the list by descending ID..\n");
  TbxListSortItems(dataList, DemoCompareElementByDescId);
  dataElement = TbxListGetFirstItem(dataList);
  while (dataElement != NULL)
  {
    DemoDisplayElement(dataElement);
    printf("[OK]\n");
    dataElement = TbxListGetNextItem(dataList, dataElement);
  }

  /* Delete the list now that it is no longer needed. */
  printf("Deleting the linked list..");
  TbxListDelete(dataList);
  printf("[OK]\n");

  /* Enter infinite program loop. */
  while (1)
  {
    /* Nothing more to do for this particular demo. */
    ;  
  }
} /*** end of DemoMain ***/


/************************************************************************************//**
** \brief     Data element comparison function for sorting the list by element
**            identifier in ascending order.
** \param     item1 The first item for the comparison.
** \param     item2 The seconds item for the comparison.
** \return    TBX_TRUE if item1's identifier is greater than item2's identifier,
**            TBX_FALSE otherwise.
**
****************************************************************************************/
uint8_t DemoCompareElementByAscId(void const * item1, void const * item2)
{
  uint8_t result = TBX_FALSE;
  tListElement const * element1 = item1;
  tListElement const * element2 = item2;

  /* Verify the parameter. */
  TBX_ASSERT((item1 != NULL) && (item2 != NULL));

  /* Is the identifier of the first element greater than the second? */
  if (element1->id > element2->id)
  {
    /* Update the result accordingly. */
    result = TBX_TRUE;
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of DemoCompareElementByAscId ***/


/************************************************************************************//**
** \brief     Data element comparison function for sorting the list by element
**            identifier in descending order.
** \param     item1 The first item for the comparison.
** \param     item2 The seconds item for the comparison.
** \return    TBX_TRUE if item1's identifier is smaller than item2's identifier,
**            TBX_FALSE otherwise.
**
****************************************************************************************/
uint8_t DemoCompareElementByDescId(void const * item1, void const * item2)
{
  uint8_t result = TBX_FALSE;
  tListElement const * element1 = item1;
  tListElement const * element2 = item2;

  /* Verify the parameter. */
  TBX_ASSERT((item1 != NULL) && (item2 != NULL));

  /* Is the identifier of the first element smaller than the second? */
  if (element1->id < element2->id)
  {
    /* Update the result accordingly. */
    result = TBX_TRUE;
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of DemoCompareElementByDescId ***/


/************************************************************************************//**
** \brief     Displays the element contents on the standard output.
** \param     element Pointer to the element to display.
**
****************************************************************************************/
void DemoDisplayElement(tListElement const * element)
{
  size_t len = LIST_ELEMENT_DATA_LEN_MAX;
  size_t idx;

  /* Verify the parameter. */
  TBX_ASSERT(element != NULL);

  /* Start with the identifier. */
  printf("  id: %04Xh ", element->id);
  /* Store and sanitize the length. */
  if (element->len <= LIST_ELEMENT_DATA_LEN_MAX)
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


/*********************************** end of sortlist.c *********************************/
