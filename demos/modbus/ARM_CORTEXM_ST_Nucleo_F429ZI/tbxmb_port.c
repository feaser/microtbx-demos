/************************************************************************************//**
* \file         demos/modbus/ARM_CORTEXM_ST_Nucleo_F429ZI/tbxmb_port.c
* \brief        Modbus hardware specific port source file.
* \details      This MicroTBX-Modbus port for the Nucleo-F429ZI board supports three
*               serial ports: 
*
*                 - TBX_MB_UART_PORT1 = USART6
*                 - TBX_MB_UART_PORT2 = USART3
*                 - TBX_MB_UART_PORT3 = USART2
*               
*               On the Nucleo-F429ZI board, USART3 on PD8 and PD9 is connected to the
*               on-board ST-Link/V2.1 debugger interface, which exposes it as a virtual
*               COM-port on the PC using the USB-CDC class.
*
*               USART6 is configured for PG9 and PG14 with the idea that it's used in
*               combination with a Waveshare RS485/CAN shield. This Arduino type shield
*               offers an RS485 transceiver, allowing MicroTBX-Modbus to be tested on
*               an RS485 network. Make sure to set the UART jumpers of RX2 and TX2 on the
*               shield.
*
*               The RS485 transceiver on this shield has the usual Driver Enable (DE) and
*               Receiver Enable (RE) inputs. These inputs are connected together and can
*               be controlled with the single PF13 digital output.
*
*               USART2 is configured for PD5 and PD6 with the idea that you can connect
*               these pins to USART6 PG9 and PG14 for a loopback between USART6 and
*               USART2:
*                 - PD5 = USART2_TX <--> USART6_RX = PG9
*                 - PD6 = USART2_RX <--> USART6_TX = PG14
*                
*               In such a loopback configuration you could for example have both a client
*               and a server on one and the same board, for testing and simulation
*               purposes.
*
*               The 20 kHz free running timer counter, needed for exact Modbus-RTU
*               timings, is realized with the help of TIM7.
*
*               Note that the implementation of this MicroTBX-Modbus port assumes that
*               the following topics are handled by the application, upon initialization:
*
*                 - Enabling of the USART2, USART3, USART6 and TIM7 peripheral clocks.
*                 - Configuration of the USART2, USART3, USART6 and RS485 DE/NRE GPIO
*                   pins. 
*                 - Enabling the USART2, USART3 and USART6 interrupts in the NVIC.
*
*               In the demo programs, this is handled by function BspInit(). 
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2024 by Feaser     www.feaser.com     All rights reserved
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
#include "microtbxmodbus.h"                      /* MicroTBX-Modbus library            */
#include "stm32f4xx.h"                           /* STM32 CPU and HAL                  */


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void TbxMbPortUartDriverEnable(tTbxMbUartPort port,
                                      uint8_t        value);


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Variable that groups together UART channel information, including data to
 *         control a serial transmission for each serial port.
 */
static struct
{
  UART_HandleTypeDef         handle;     /**< USART channel handle.                    */
  USART_TypeDef            * instance;   /**< USART instance pointer.                  */
  uint8_t                    rxByte;     /**< USART single byte reception buffer.      */
} uartChannel[] =
{
  { .instance = USART6 },                /* TBX_MB_UART_PORT1 mapped to USART6.        */
  { .instance = USART3 },                /* TBX_MB_UART_PORT2 mapped to USART3.        */
  { .instance = USART2 }                 /* TBX_MB_UART_PORT3 mapped to USART2.        */
};


/************************************************************************************//**
** \brief     Initializes the UART channel.
** \attention It is assumed that the following items were already handled by the
**            application, upon initialization:
**            - Enabling of the UART channel's peripheral clock.
**            - Configuration of the UART Tx and Rx GPIO pins.
**            - Enabling the UART interrupts in the NVIC.
** \param     port The serial port to use. The actual meaning of the serial port is
**            hardware dependent. It typically maps to the UART peripheral number. E.g. 
**            TBX_MB_UART_PORT1 = USART1 on an STM32, although this is not mandatory.
** \param     baudrate The desired communication speed.
** \param     databits Number of databits for a character.
** \param     stopbits Number of stop bits at the end of a character.
** \param     parity Parity bit type to use.
**
****************************************************************************************/
void TbxMbPortUartInit(tTbxMbUartPort     port, 
                       tTbxMbUartBaudrate baudrate,
                       tTbxMbUartDatabits databits, 
                       tTbxMbUartStopbits stopbits,
                       tTbxMbUartParity   parity)
{
  const uint32_t baudrateLookup[TBX_MB_UART_NUM_BAUDRATE] =
  {
    1200,                                                  /* TBX_MB_UART_1200BPS      */
    2400,                                                  /* TBX_MB_UART_2400BPS      */
    4800,                                                  /* TBX_MB_UART_4800BPS      */
    9600,                                                  /* TBX_MB_UART_9600BPS      */
    19200,                                                 /* TBX_MB_UART_19200BPS     */
    38400,                                                 /* TBX_MB_UART_38400BPS     */
    57600,                                                 /* TBX_MB_UART_57600BPS     */
    115200                                                 /* TBX_MB_UART_115200BPS    */
  };
  const uint32_t stopbitsLookup[TBX_MB_UART_NUM_STOPBITS] =
  {
    UART_STOPBITS_1,                                       /* TBX_MB_UART_1_STOPBITS   */
    UART_STOPBITS_2                                        /* TBX_MB_UART_2_STOPBITS   */
  };
  const uint32_t parityLookup[TBX_MB_UART_NUM_PARITY] =
  {
    UART_PARITY_ODD,                                       /* TBX_MB_ODD_PARITY        */
    UART_PARITY_EVEN,                                      /* TBX_MB_EVEN_PARITY       */
    UART_PARITY_NONE                                       /* TBX_MB_NO_PARITY         */
  };

  /* Make sure the requested serial port is actually supported by this module. */
  TBX_ASSERT(port <= (sizeof(uartChannel)/sizeof(uartChannel[0])));

  /* Initialize the channel's handle. */
  uartChannel[port].handle.Instance = uartChannel[port].instance;
  uartChannel[port].handle.Init.Mode = UART_MODE_TX_RX;
  uartChannel[port].handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uartChannel[port].handle.Init.OverSampling = UART_OVERSAMPLING_16;
  uartChannel[port].handle.Init.BaudRate = baudrateLookup[baudrate];
  uartChannel[port].handle.Init.Parity = parityLookup[parity];
  uartChannel[port].handle.Init.StopBits = stopbitsLookup[stopbits];

  /* Initialize the number of databits to 8. This covers these configurations:
   * - 7 databits with parity enabled.
   * - 8 databits with parity disabled.
   */
  uint32_t wordLength = UART_WORDLENGTH_8B;
  /* The STM32F4's UART peripheral only supports 7 databits if you also have parity
   * enabled. Meaning that this configuration is not possible:
   * - 7 databits with parity disabled.
   */
  if ((parity == TBX_MB_NO_PARITY) && (databits == TBX_MB_UART_7_DATABITS))
  {
    /* Trigger assertion error due to an invalid configuration. */
    TBX_ASSERT(TBX_FALSE);
  }
  /* One other configuration might still be possible:
   * - 8 databits with parity enabled.
   */
  else if ((databits == TBX_MB_UART_8_DATABITS) && (parity != TBX_MB_NO_PARITY))
  {
    wordLength = UART_WORDLENGTH_9B;
  }
  uartChannel[port].handle.Init.WordLength = wordLength;
  /* Initialize the channel. */
  HAL_UART_Init(&uartChannel[port].handle);
  /* Kick off first byte reception. */
  HAL_UART_Receive_IT(&uartChannel[port].handle, &uartChannel[port].rxByte, 1);
} /*** end of TbxMbPortUartInit ***/


/************************************************************************************//**
** \brief     Starts the transfer of len bytes from the data array on the specified 
**            serial port.
** \attention This function has mutual exclusive access to the bytes in the data[] array,
**            until this port module calls TbxMbUartTransmitComplete(). This means that
**            you do not need to copy the data bytes to a local buffer. This approach 
**            keeps RAM requirements low and benefits the run-time performance. Just make
**            sure to call TbxMbUartTransmitComplete() once all bytes are transmitted or
**            an error was detected, to release access to the data[] array.
** \param     port The serial port to start the data transfer on.
** \param     data Byte array with data to transmit.
** \param     len Number of bytes to transmit.
** \return    TBX_OK if successful, TBX_ERROR otherwise.  
**
****************************************************************************************/
uint8_t TbxMbPortUartTransmit(tTbxMbUartPort         port, 
                              uint8_t        const * data, 
                              uint16_t               len)
{
  uint8_t result = TBX_ERROR;

  /* Make sure the requested serial port is actually supported by this module. */
  TBX_ASSERT(port <= (sizeof(uartChannel)/sizeof(uartChannel[0])));

  /* Switch the hardware from reception to transmission mode. */
  TbxMbPortUartDriverEnable(port, TBX_ON);
  /* Start the interrupt driven transmission of the data bytes. */
  if (HAL_UART_Transmit_IT(&uartChannel[port].handle, (uint8_t *)data, len) == HAL_OK)
  {
    result = TBX_OK;
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of TbxMbPortUartTransmit ***/


/************************************************************************************//**
** \brief     Changes the state of the driver enable pin for the available serial port.
**            This is only applicable for RS485 networks, which require an RS485 
**            transceiver.
** \attention It is assumed that the following items were already handled by the
**            application, upon initialization:
**            - Configuration of the DE/NRE GPIO pin(s) as a digital output. 
** \details   When communicating via RS485, the transceiver typically has a driver enable
**            (DE) pin which is active high, and a receiver enable (NRE) pin which is
**            active low. Both pins must be set high when transmitting data and low when
**            receiving data. Note that on most PCBs, the DE and NRE pins are connected
**            together, in which case you only need to change one output pin, as opposed
**            to two.
** \param     port The serial port to change the state of the driver enable pin for.
** \param     value TBX_ON to set the DE/NRE pins to logic high, TBX_OFF for logic low.
**
****************************************************************************************/
static void TbxMbPortUartDriverEnable(tTbxMbUartPort port,
                                      uint8_t        value)
{
  /* Make sure the requested serial port is actually supported by this module. */
  TBX_ASSERT(port <= (sizeof(uartChannel)/sizeof(uartChannel[0])));
  
  /* Only continue with a supported port. */
  if (port <= (sizeof(uartChannel)/sizeof(uartChannel[0])))
  {
    /* Only USART6 on this board is configured for RS485, with an RS485 transceiver on
     * the Waveshare RS485/CAN shield. The transceiver's DE/NRE pin is connected to D7 of
     * the Arduino connector, which is GPIO PF13.
     */
    if (port == TBX_MB_UART_PORT1)
    {
      if (value == TBX_ON)
      {
        /* Set the DE/NRE pin(s) to logic high to switch to transmission mode. */
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
      }
      else
      {
        /* Set the DE/NRE pin(s) to logic low to switch to reception mode. */
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
      }
    }
  }
} /*** end of TbxMbPortUartDriverEnable ***/


/************************************************************************************//**
** \brief     Obtains the free running counter value of a timer that runs at 20 kHz.
** \attention It is assumed that the following items were already handled by the
**            application, upon initialization:
**            - Enabling of the TIM7 peripheral clock.
** \details   The Modbus RTU communication makes use of 1.5 (T1_5) and 3.5 (T3_5)
**            character timeouts. Time between packet characters should be <= T1_5 and
**            the time between packets should be >= T3_5. 
** 
**            To get these timings right a free running counter, incrementing every 50
**            microseconds, provides a time reference. This function obtains the current
**            value of this counter.
**
**            Ideally a timer is initialized to have its free running counter running at
**            20 kHz for this. The benefit of this approach is that the timer can still
**            be reused for other purposes (input capture, pwm, output compare, etc.) and
**            does not need a periodic timer interrupt.
**
**            Timers are a scarce resource on microcontrollers. Therefore it is also
**            possible to use the free running counter of a timer that runs at a
**            different frequency. Just make sure to adjust the counter value in this
**            function accordingly. For example, if you choose to reuse your RTOS'
**            1 millisecond SysTick, you need to multiply its tick counter value by 20 to
**            simulate a 20 kHz timer. This does of course have a negative impact on the
**            accuracy of the T1_5 and T3_5 timings, so there's a trade off involved.
** \return    Free running counter value.
**
****************************************************************************************/
uint16_t TbxMbPortTimerCount(void)
{
  static uint8_t           initialized = TBX_FALSE;
  static TIM_HandleTypeDef timHandle   = {0};

  /* Perform one-time initialization. */
  if (initialized == TBX_FALSE)
  {
    initialized = TBX_TRUE;
    /* According to the clock tree diagram in the RCC chapter of the reference manual,
     * the PCLK1-TIM frequency = PLCK1 * 1, when the APB1 prescaler is 1, otherwise it is
     * PCLK1 * 2.
     */
    uint8_t timMultiplier = 1U;
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
    {
      timMultiplier = 2U;
    }
    uint32_t timFreq = HAL_RCC_GetPCLK1Freq() * timMultiplier;
    timHandle.Instance = TIM7;
    timHandle.Init.Prescaler = (timFreq / 20000U) - 1U;
    timHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
    timHandle.Init.Period = 65535;
    timHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&timHandle);
    __HAL_TIM_ENABLE(&timHandle);
  }
  /* Read out the current value of the timer's free running counter and return it. */
  return (uint16_t)__HAL_TIM_GET_COUNTER(&timHandle);
} /*** end of TbxMbPortTimerCount ***/


/****************************************************************************************
*            C A L L B A C K   R O U T I N E S
****************************************************************************************/
/************************************************************************************//**
** \brief     UART transmit complete callback.
** \param     handle Pointer to the channel's handle.
**
****************************************************************************************/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef * handle)
{
  uint8_t port;

  /* Locate the port that this handle belongs to. */
  for (port = 0; port < (sizeof(uartChannel)/sizeof(uartChannel[0])); port++)
  {
    if (&uartChannel[port].handle == handle)
    {
      /* Switch the hardware from transmission to reception mode. */
      TbxMbPortUartDriverEnable(port, TBX_OFF);
      /* Inform the Modbus UART module about the transmission completed event. */
      TbxMbUartTransmitComplete(port);
      /* Stop the loop, now that the port was located. */
      break;
    }
  }
} /*** end of HAL_UART_TxCpltCallback ***/


/************************************************************************************//**
** \brief     UART reception complete callback.
** \param     handle Pointer to the channel's handle.
**
****************************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef * handle)
{
  uint8_t port;

  /* Locate the port that this handle belongs to. */
  for (port = 0; port < (sizeof(uartChannel)/sizeof(uartChannel[0])); port++)
  {
    if (&uartChannel[port].handle == handle)
    {
      uint32_t errorCode = HAL_UART_GetError(&uartChannel[port].handle);
      /* Only process the byte if no noise, framing or parity error was detected. */
      if ((errorCode & (HAL_UART_ERROR_NE | HAL_UART_ERROR_PE | HAL_UART_ERROR_FE)) == 0)
      {
        /* Inform the Modbus UART module about the newly received data byte. */
        TbxMbUartDataReceived(port, &uartChannel[port].rxByte, 1U);
      }
      /* Restart reception for the next byte. */
      HAL_UART_Receive_IT(&uartChannel[port].handle, &uartChannel[port].rxByte, 1);
      /* Stop the loop, now that the port was located. */
      break;
    }
  }
} /*** end of HAL_UART_RxCpltCallback ***/


/****************************************************************************************
*            I N T E R R U P T   S E R V I C E   R O U T I N E S
****************************************************************************************/
/************************************************************************************//**
** \brief     USART6 interrupt service routine.
**
****************************************************************************************/
void USART6_IRQHandler(void)
{
  /* Pass event on to the HAL driver for further handling. */
  HAL_UART_IRQHandler(&uartChannel[TBX_MB_UART_PORT1].handle);
} /*** end of USART6_IRQHandler ***/


/************************************************************************************//**
** \brief     USART3 interrupt service routine.
**
****************************************************************************************/
void USART3_IRQHandler(void)
{
  /* Pass event on to the HAL driver for further handling. */
  HAL_UART_IRQHandler(&uartChannel[TBX_MB_UART_PORT2].handle);
} /*** end of USART3_IRQHandler ***/


/************************************************************************************//**
** \brief     USART2 interrupt service routine.
**
****************************************************************************************/
void USART2_IRQHandler(void)
{
  /* Pass event on to the HAL driver for further handling. */
  HAL_UART_IRQHandler(&uartChannel[TBX_MB_UART_PORT3].handle);
} /*** end of USART2_IRQHandler ***/


/*********************************** end of tbxmb_port.c *******************************/
