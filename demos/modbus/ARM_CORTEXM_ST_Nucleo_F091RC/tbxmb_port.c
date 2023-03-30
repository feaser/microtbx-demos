/************************************************************************************//**
* \file         demos/modbus/ARM_CORTEXM_ST_Nucleo_F091RC/tbxmb_port.c
* \brief        Modbus hardware specific port source file.
* \details      This MicroTBX-Modbus port for the Nucleo-F091RC board supports three
*               serial ports:
*
*                 - TBX_MB_UART_PORT1 = USART1
*                 - TBX_MB_UART_PORT2 = USART2
*                 - TBX_MB_UART_PORT3 = USART3
*               
*               On the Nucleo-F091RC board, USART2 on PA2 and PA3 is connected to the
*               on-board ST-Link/V2.1 debugger interface, which exposes it as a virtual
*               COM-port on the PC using the USB-CDC class.
*
*               USART1 is configured for PA9 and PA10 with the idea that it's used in
*               combination with a Waveshare RS485/CAN shield. This Arduino type shield
*               offers an RS485 transceiver, allowing MicroTBX-Modbus to be tested on
*               an RS485 network.
*
*               The RS485 transceiver on this shield has the usual Driver Enable (DE) and
*               Receiver Enable (RE) inputs. These inputs are connected together and can
*               be controlled with the single PA8 digital output.
*
*               USART3 is configured for PC10 and PC11 with the idea that you can connect
*               these pins to USART1 PA9 and PA10 for a loopback between USART1 and
*               USART3:
*                 - PC10 = USART3_TX <--> USART1_RX = PA10
*                 - PC11 = USART3_RX <--> USART1_TX = PA9
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
*                 - Enabling of the USART1, USART2, USART3 and TIM7 peripheral clocks.
*                 - Configuration of the USART1, USART2, USART3 and RS485 DE/NRE GPIO
*                   pins. 
*                 - Configuration and enabling of the TIM7 free running counter to count
*                   upwards at a speed of 20 kHz.
*                 - Enabling the USART1, USART2 and USART3 interrupts in the NVIC.
*
*               In the demo programs, this is handled by function BspInit(). 
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
#include "microtbxmodbus.h"                      /* MicroTBX-Modbus library            */
#include "stm32f0xx_ll_usart.h"                  /* STM32 LL USART                     */
#include "stm32f0xx_ll_gpio.h"                   /* STM32 LL GPIO                      */
#include "stm32f0xx_ll_tim.h"                    /* STM32 LL TIM                       */


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void TbxMbPortUartDriverEnable(tTbxMbUartPort port,
                                      uint8_t        value);

static void TbxMbPortUartInterrupt   (tTbxMbUartPort port);


/****************************************************************************************
* Local constant declarations
****************************************************************************************/
/** \brief Lookup table for mapping the UART port to its peripheral channel. */
static USART_TypeDef * const uartChannel[] =
{
  USART1,                                  /* Idx 0: TBX_MB_UART_PORT1                 */
  USART2,                                  /* Idx 1: TBX_MB_UART_PORT2                 */
  USART3                                   /* Idx 2: TBX_MB_UART_PORT3                 */
};

/** \brief Variable that groups together information to control a serial transmission
 *         for each serial port. Can be indexed with a tTbxMbUartPort variable, i.e.
 *         a TBX_MB_UART_PORTx value.
 */
static volatile struct
{
  uint8_t  const * data;                   /* Pointer of the transmit data byte array. */
  uint16_t         nextIdx;                /* Index of the next byte to transmit.      */
  uint16_t         totalLen;               /* Total number of bytes to transmit.       */
} transmitInfo[sizeof(uartChannel)/sizeof(uartChannel[0])];


/************************************************************************************//**
** \brief     Initializes the UART channel.
** \attention It is assumed that the following items were already handled by the
**            application, upon initialization:
**            - Enabling of the UART channel's peripheral clock.
**            - Configuration of the UART Tx and Rx GPIO pins.
**            - Enabling the UART interrupts in the NVIC.
** \param     port The serial port to use. The actual meaning of the serial port is
**            hardware dependent. It typically maps to the UART peripheral number. E.g. 
**            TBX_MB_UART_PORT1 = USART1 on an STM32.
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
  LL_USART_InitTypeDef USART_InitStruct = { 0 };
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
    LL_USART_STOPBITS_1,                                   /* TBX_MB_UART_1_STOPBITS   */
    LL_USART_STOPBITS_2                                    /* TBX_MB_UART_2_STOPBITS   */
  };
  const uint32_t parityLookup[TBX_MB_UART_NUM_PARITY] =
  {
    LL_USART_PARITY_ODD,                                   /* TBX_MB_ODD_PARITY        */
    LL_USART_PARITY_EVEN,                                  /* TBX_MB_EVEN_PARITY       */
    LL_USART_PARITY_NONE                                   /* TBX_MB_NO_PARITY         */
  };

  /* Make sure the requested serial port is actually supported by this module. */
  TBX_ASSERT(port <= (sizeof(uartChannel)/sizeof(uartChannel[0])));
  
  /* Only continue with a supported port. */
  if (port <= (sizeof(uartChannel)/sizeof(uartChannel[0])))
  {
    /* Disable and reset the UART peripheral. */
    LL_USART_Disable(uartChannel[port]);
    LL_USART_DeInit(uartChannel[port]);
    /* Configure UART peripheral. */
    USART_InitStruct.BaudRate = baudrateLookup[baudrate];
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    USART_InitStruct.StopBits = stopbitsLookup[stopbits];
    USART_InitStruct.Parity = parityLookup[parity];
    /* One extra data bit needs to be configured for the parity bit, if enabled. */
    switch (databits)
    {
    case TBX_MB_UART_7_DATABITS:
      USART_InitStruct.DataWidth = (parity == TBX_MB_NO_PARITY) ? LL_USART_DATAWIDTH_7B : 
                                                                  LL_USART_DATAWIDTH_8B;
      break;

    case TBX_MB_UART_8_DATABITS:
    default:
      USART_InitStruct.DataWidth = (parity == TBX_MB_NO_PARITY) ? LL_USART_DATAWIDTH_8B : 
                                                                  LL_USART_DATAWIDTH_9B;
      break;
    }
    /* Initialize and enable the UART peripheral. */
    LL_USART_Init(uartChannel[port], &USART_InitStruct);
    LL_USART_Enable(uartChannel[port]);
    /* Enable the receiver interrupt. */
    LL_USART_EnableIT_RXNE(uartChannel[port]);
  }
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
  uint8_t result = TBX_OK;

  /* Make sure the requested serial port is actually supported by this module. */
  TBX_ASSERT(port <= (sizeof(uartChannel)/sizeof(uartChannel[0])));
  
  /* Only continue with a supported port. */
  if (port <= (sizeof(uartChannel)/sizeof(uartChannel[0])))
  {
    /* Prepare the transmit information taking into account that this function will already
     * start the transmission of the first byte.
    */
    transmitInfo[port].data = data;
    transmitInfo[port].totalLen = len;
    transmitInfo[port].nextIdx = 1U;
    /* Switch the hardware from reception to transmission mode. */
    TbxMbPortUartDriverEnable(port, TBX_ON);
    /* Kick off the transmission with the first data byte. */
    LL_USART_TransmitData8(uartChannel[port], transmitInfo[port].data[0]);
    /* Enable the transmit complete interrupt if just transmitting one byte, otherwise
     * enable the transmit data register empty interrupt.
     */
    if (len == 1U)
    {
      LL_USART_EnableIT_TC(uartChannel[port]);
    }
    else
    {
      LL_USART_EnableIT_TXE(uartChannel[port]);
    }
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
    /* Only USART1 on this board is configured for RS485, with an RS485 transceiver on
     * the Waveshare RS485/CAN shield. The transceiver's DE/NRE pin is connected to D7 of
     * the Arduino connector, which is GPIO PA8.
     */
    if (port == TBX_MB_UART_PORT1)
    {
      if (value == TBX_ON)
      {
        /* Set the DE/NRE pin(s) to logic high to switch to transmission mode. */
        LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
      }
      else
      {
        /* Set the DE/NRE pin(s) to logic low to switch to reception mode. */
        LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
      }
    }
  }
} /*** end of TbxMbPortUartDriverEnable ***/


/************************************************************************************//**
** \brief     UART transmit and reception interrupt handler. Should be called from your
**            UART interrupt handler for the specified serial port.
** \param     port The serial port that generated the interrupt.
**
****************************************************************************************/
static void TbxMbPortUartInterrupt(tTbxMbUartPort port)
{
  uint32_t intFlags = uartChannel[port]->ISR;
  
  /* ------------------- Reception related data or error interrupt? ------------------ */
  if (LL_USART_IsEnabledIT_RXNE(uartChannel[port]) != 0U)
  {
    if ((intFlags & (USART_ISR_RXNE | USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | 
        USART_ISR_PE)) != 0U)
    {
      /* Clear the error flags. */
      uartChannel[port]->ICR = USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | 
                              USART_ISR_PE;
      /* Read out the newly received data, which also clears the RXNE flag. */
      uint8_t rxByte = LL_USART_ReceiveData8(uartChannel[port]);
      /* In case of a parity, framing or noise error, ignore the received data because it
       * will be invalid. Nothing that can be done to save or correct the data. It is
       * assumed that a higher module will be able to detect that data went missing. For
       * example due to a data stream being shorter than expected or an incorrect
       * checksum. In case of an overrun error, the data is still valid. Only the data in
       * the reception shift register gets lost.
       */
      if ((intFlags & (USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)) == 0U)
      {
        /* Inform the Modbus UART module about the newly received data byte. */
        TbxMbUartDataReceived(port, &rxByte, 1U);
      }
    }
  }

  /* ------------------- Transmission complete related interrupt? -------------------- */
  if ( (LL_USART_IsEnabledIT_TC(uartChannel[port]) != 0U) || 
       (LL_USART_IsEnabledIT_TXE(uartChannel[port]) != 0U) )
  {
    if ((intFlags & (USART_ISR_TC | USART_ISR_TXE)) != 0U)
    {
      /* Still data left to send? */
      if (transmitInfo[port].nextIdx < transmitInfo[port].totalLen)
      {
        /* Continue the transmission with the next data byte. */
        LL_USART_TransmitData8(uartChannel[port], 
                               transmitInfo[port].data[transmitInfo[port].nextIdx]);
        /* Currently transmitting the last byte of the entire transfer? */
        if (transmitInfo[port].nextIdx == (transmitInfo[port].totalLen - 1U))
        {
          /* Switch from data register empty to transmit complete interrupt. */
          LL_USART_DisableIT_TXE(uartChannel[port]);
          LL_USART_EnableIT_TC(uartChannel[port]);
        }
        /* Update the indexer to point to the next byte to transmit. */
        transmitInfo[port].nextIdx++;
      }
      /* No more data left to send. */
      else
      {
        /* Disable the transmit complete interrupt. */
        LL_USART_DisableIT_TC(uartChannel[port]);
        /* Switch the hardware from transmission to reception mode. */
        TbxMbPortUartDriverEnable(port, TBX_OFF);
        /* Inform the Modbus UART module about the transmission completed event. */
        TbxMbUartTransmitComplete(port);
      }
    }
  }
} /*** end of TbxMbPortUartInterrupt ***/


/************************************************************************************//**
** \brief     Obtains the free running counter value of a timer that runs at 20 kHz.
** \attention It is assumed that the following items were already handled by the
**            application, upon initialization:
**            - Enabling of the TIM7 peripheral clock.
**            - Configuration of the TIM7 free running counter to count upwards at a 
**              speed of 20 kHz.
**            - Enabling of the TIM7 free running counter.
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
  /* Obtain the current value of TIM7's free running counter. */
  return (uint16_t)LL_TIM_GetCounter(TIM7);
} /*** end of TbxMbPortTimerCount ***/


/****************************************************************************************
*            I N T E R R U P T   S E R V I C E   R O U T I N E S
****************************************************************************************/
/************************************************************************************//**
** \brief     Interrupt service routine of the USART1 peripheral.
**
****************************************************************************************/
void USART1_IRQHandler(void)
{
  /* Reroute to the generic handler, while taking into account the serial port mapping
   * from uartChannel[].
   */
  TbxMbPortUartInterrupt(TBX_MB_UART_PORT1);
} /*** end of USART1_IRQHandler ***/


/************************************************************************************//**
** \brief     Interrupt service routine of the USART2 peripheral.
**
****************************************************************************************/
void USART2_IRQHandler(void)
{
  /* Reroute to the generic handler, while taking into account the serial port mapping
   * from uartChannel[].
   */
  TbxMbPortUartInterrupt(TBX_MB_UART_PORT2);
} /*** end of USART2_IRQHandler ***/


/************************************************************************************//**
** \brief     Interrupt service routine of the USART3 peripheral.
**
****************************************************************************************/
void USART3_8_IRQHandler(void)
{
  /* Reroute to the generic handler, while taking into account the serial port mapping
   * from uartChannel[].
   */
  TbxMbPortUartInterrupt(TBX_MB_UART_PORT3);
} /*** end of USART3_8_IRQHandler ***/


/*********************************** end of tbxmb_port.c *******************************/
