/************************************************************************************//**
* \file         demos/base/securedata.c
* \brief        Demonstrates how to use the checksum and cryptography modules to store
*               and retrieve a block of data in a secure manner.
* \details      The demo application implements a data vault for securily storing a block
*               of 256 bytes. The actual location is just a variable in RAM. Yet, this
*               could easily be adjusted for EEPROM, FRAM or a file on a filesystem.
*
*               Upon data storage, a CRC16 checksum value is calculated and stored with
*               the data. That way the correctness of the data can be validated upon
*               retrieval. Next, the data is encrypted using an AES-256 ECB algorithm.
*
*               When retrieving the data, it is first decrypted and afterwards the CRC16
*               is recalculated and compared with the stored CRC16 value.
*
*               The demo application generates 256 bytes of test data and stores it in
*               a secure manner. Afterwards, it is retrieved again and verified that the
*               original data did not change.
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
/** \brief Number of data bytes in a secure data block. */
#define SECURE_DATA_BLOCK_SIZE         (256U)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Layout of the data object for storing a block of data in a secure manner. */
typedef struct
{
  uint8_t  data[SECURE_DATA_BLOCK_SIZE];
  uint16_t crc;
} tSecureDataBlock;


/****************************************************************************************
* Local constant declarations
****************************************************************************************/
/** \brief The 256-bit key that will be used to encrypt and decrypt data. */
static const uint8_t secureKey[] =
{
  0x32, 0x72, 0x35, 0x75, 0x38, 0x78, 0x21, 0x41,
  0x25, 0x44, 0x2A, 0x47, 0x2D, 0x1A, 0x61, 0x50,
  0x64, 0x53, 0x67, 0x56, 0x6B, 0x59, 0x70, 0x33,
  0x73, 0x36, 0x76, 0x39, 0x79, 0x24, 0x42, 0x5E
};


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Variable for storing data in a secure manner. */
static tSecureDataBlock secureDataBlock;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void    DemoStoreData(uint8_t const * data);
static uint8_t DemoRetrieveData(uint8_t * data);
static void    DemoDisplayData(uint8_t const * data, size_t len, size_t bytesPerLine);


/************************************************************************************//**
** \brief     Entry point for the demo application. Called by main() after the 
**            initialization completed. Contains the infinite program loop.
**
****************************************************************************************/
void DemoMain(void)
{
  uint8_t testData[SECURE_DATA_BLOCK_SIZE];
  size_t  idx;
  uint8_t res;
  uint8_t dataOkay = TBX_TRUE;

  /* Generate some data for testing purposes and dislay it on the standard output. */
  printf("\nGenerating test data:\n");
  for (idx= 0U; idx < SECURE_DATA_BLOCK_SIZE; idx++)
  {
    testData[idx] = idx;
  }
  DemoDisplayData(testData, SECURE_DATA_BLOCK_SIZE, 32);
  printf("[OK]\n");

  /* Store the data in the secure data block and display its contents. */
  printf("\nStoring the data in a secure manner:\n");
  DemoStoreData(testData);
  DemoDisplayData(secureDataBlock.data, SECURE_DATA_BLOCK_SIZE, 32);
  printf("CRC16 = %04Xh\n", secureDataBlock.crc);
  printf("[OK]\n");

  /* Retrieve the data from the secure data block and display its contents. */
  printf("\nRetrieving the data:\n");
  res = DemoRetrieveData(testData);
  TBX_ASSERT(res == TBX_OK);
  DemoDisplayData(testData, SECURE_DATA_BLOCK_SIZE, 32);
  printf("[OK]\n");

  /* Verify that the data did not change. */
  printf("\nVerifying that the data did not change:\n");
  for (idx= 0U; idx < SECURE_DATA_BLOCK_SIZE; idx++)
  {
    /* Is this byte value as expected? */
    if (testData[idx] != idx)
    {
      /* Set flag to indicate the error. */
      dataOkay = TBX_FALSE;
      break;
    }
  }
  if (dataOkay == TBX_TRUE)
  {
    printf("[OK]\n");
  }
  else
  {
    printf("[ERROR]\n");
  }

  /* Enter infinite program loop. */
  while (1)
  {
    /* Nothing more to do for this particular demo. */
    ;  
  }
} /*** end of DemoMain ***/


/************************************************************************************//**
** \brief     Stores data bytes with a length of SECURE_DATA_BLOCK_SIZE in a secure
**            manner. First, a CRC16 checksum is calculated and stored with the data.
**            Next, the data is encrypted using the AES-256 ECB algorithm.
** \param     data Pointer to the array with data bytes to store.
**
****************************************************************************************/
static void DemoStoreData(uint8_t const * data)
{
  size_t idx;

  /* Verify parameter. */
  TBX_ASSERT(data != NULL);

  /* Obtain mutual exclusive access to secureDataBlock, because it could potentially be
   * shared between an ISR and the main program loop.
   */
  TbxCriticalSectionEnter();
  /* Start by copying the data to the storage location. */
  for (idx = 0U; idx < SECURE_DATA_BLOCK_SIZE; idx++)
  {
    secureDataBlock.data[idx] = data[idx];
  }
  /* Next calculate the CRC16 over the data and store it. */
  secureDataBlock.crc = TbxChecksumCrc16Calculate(secureDataBlock.data,
                                                  SECURE_DATA_BLOCK_SIZE);
  /* As a last step encrypt the data. */
  TbxCryptoAes256Encrypt(secureDataBlock.data, SECURE_DATA_BLOCK_SIZE, secureKey);
  /* Release mutual exclusive access to secureDataBlock. */
  TbxCriticalSectionExit();
} /*** end of DemoStoreData ***/


/************************************************************************************//**
** \brief     Retrieves data bytes with a length of SECURE_DATA_BLOCK_SIZE in a secure
**            manner. First, the data is decrypted using the AES-256 ECB algorithm. Next,
**            the results are verified using a CRC16 checksum value. This makes sure the
**            data didn't actually change somehow, while it was stored.
** \param     data Pointer to the byte array where the retrieved bytes will be written
**            to.
** \return    TBX_OK if the data retrieval was successful, TBX_ERROR otherwise. This
**            could happen if the stored data somehow changed (tampering / aging).
**
****************************************************************************************/
static uint8_t DemoRetrieveData(uint8_t * data)
{
  uint8_t  result = TBX_ERROR;
  size_t   idx;
  uint16_t crc;

  /* Verify parameter. */
  TBX_ASSERT(data != NULL);

  /* Obtain mutual exclusive access to secureDataBlock, because it could potentially be
   * shared between an ISR and the main program loop.
   */
  TbxCriticalSectionEnter();
  /* Start by copying the stored data to the array specified by the parameter. Note that
   * the data is still encrypted at this point.
   */
  for (idx = 0U; idx < SECURE_DATA_BLOCK_SIZE; idx++)
  {
    data[idx] = secureDataBlock.data[idx];
  }
  /* Next decrypt the data. */
  TbxCryptoAes256Decrypt(data, SECURE_DATA_BLOCK_SIZE, secureKey);
  /* As the last step perform the checksum verification. */
  crc = TbxChecksumCrc16Calculate(data, SECURE_DATA_BLOCK_SIZE);
  if (crc == secureDataBlock.crc)
  {
    /* Checksum was as expected. Update the result for success. */
    result =  TBX_OK;
  }
  /* Release mutual exclusive access to secureDataBlock. */
  TbxCriticalSectionExit();
  /* Give the result back to the caller. */
  return result;
} /*** end of DemoRetrieveData ***/


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


/*********************************** end of securedata.c *******************************/
