/************************************************************************************//**
* \file         demos/base/ARM_CORTEXM_ST_Nucleo_F091RC/bsp.c
* \brief        Board support package source file.
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
#include "stm32f0xx.h"                           /* STM32 CPU and HAL header           */
#include "stm32f0xx_ll_usart.h"                  /* STM32 LL USART header              */


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Handle for accessing the ADC peripheral with the HAL drivers. */
static ADC_HandleTypeDef adcHandle;


/************************************************************************************//**
** \brief     Initialized the board support package.
**
****************************************************************************************/
void BspInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  ADC_ChannelConfTypeDef ADC_ChannelConfStruct = { 0 };
  LL_USART_InitTypeDef USART_InitStruct = { 0 };

  /* GPIO port, ADC and UART peripheral clock enable. */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();

  /* Configure PA0 (AN0 on the board) as an analog input. */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure the global features of the ADC such as the clock, resolution, data
   * alignment and number of conversions.
   */
  adcHandle.Instance = ADC1;
  adcHandle.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  adcHandle.Init.Resolution = ADC_RESOLUTION_12B;
  adcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  adcHandle.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  adcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  adcHandle.Init.LowPowerAutoWait = DISABLE;
  adcHandle.Init.LowPowerAutoPowerOff = DISABLE;
  adcHandle.Init.ContinuousConvMode = DISABLE;
  adcHandle.Init.DiscontinuousConvMode = DISABLE;
  adcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  adcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  adcHandle.Init.DMAContinuousRequests = DISABLE;
  adcHandle.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  (void)HAL_ADC_Init(&adcHandle);

  /* Configure the ADC regular channel (PA0 = ADC channel 0) to be converted. */
  ADC_ChannelConfStruct.Channel = ADC_CHANNEL_0;
  ADC_ChannelConfStruct.Rank = ADC_RANK_CHANNEL_NUMBER;
  ADC_ChannelConfStruct.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  (void)HAL_ADC_ConfigChannel(&adcHandle, &ADC_ChannelConfStruct);

  /* UART TX and RX GPIO pin configuration. */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure UART peripheral. Used for printf(). */
  USART_InitStruct.BaudRate = 57600;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  /* Initialize the UART peripheral */
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_Enable(USART2);
} /*** end of BspInit ***/


/************************************************************************************//**
** \brief     Transmits the character on the communication interface.
** \param     ch The value of the character to transmit.
** \return    The value of the transmitted character if successful, -1 otherwise.
**
****************************************************************************************/
int BspPutChar(int ch)
{
  int result = ch;
  uint8_t c;

  /* Convert to 8-bit character. */
  c = ch & 0x00FF;
  /* Write byte to transmit holding register */
  LL_USART_TransmitData8(USART2, c);
  /* Wait for tx holding register to be empty */
  while (LL_USART_IsActiveFlag_TXE(USART2) == 0)
  {
    ;
  }
  /* Automatically send carriage return with each newline. */
  if (ch == '\n')
  {
    (void)BspPutChar('\r');
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of BspPutChar ***/


/************************************************************************************//**
** \brief     Obtains a seed, which is used to initialize the seed for the random number
**            generator. The actual value is not really important as long as it is a
**            value that is different every time the software program runs, so after each
**            reset event.
** \details   This example implementation set the seed based on the value of a floating
**            analog input. Such a floating analog input will pick up noise, so the
**            analog to digital conversion results always vary slightly. Other options
**            would be to:
**            * Increment a 32-bit value in EEPROM or a non-volatile register, if
**              supported by the microcontroller, each time this function is called.
**              Keep in mind though that these data storage options have a limited amount
**              of write cycles. A better option might be to use external FRAM.
**            * If the system has access to an external file system such as an SD-card,
**              you could increment a 32-bit value in a file each time this function is
**              called.
** \return    The 32-bit value that the random number generator module uses as a seed to
**            initialize itself.
**
****************************************************************************************/
uint32_t BspGetSeed(void)
{
  uint16_t result = 0;
  uint16_t conversionResults[2] = { 0 };
  uint8_t  idx;

  /* Perform two analog to digital conversions on the conigured channel. In this case on
   * pin PA0, which is not connected to anything. Such a floating channel will pick up
   * noise, so the conversion results always vary slightly.
   */
  for (idx = 0; idx < sizeof(conversionResults)/sizeof(conversionResults[0]); idx++)
  {
		if (HAL_ADC_Start(&adcHandle) == HAL_OK)
		{
		  /* Wait for the analog to digital conversion to complete. */
		  if (HAL_ADC_PollForConversion(&adcHandle, 10) == HAL_OK)
		  {
		    /* Store the conversion results. */
		    conversionResults[idx] = HAL_ADC_GetValue(&adcHandle);
		  }
		}
  }
  /* Create a 32-bit seed value by combining two reads of the floating analog pin. */
  result =  (conversionResults[0] << 16u) | conversionResults[1];
  /* Give the result back to the caller. */
  return result;
} /*** end of BspGetSeed ***/


/*********************************** end of bsp.c **************************************/
