/************************************************************************************//**
* \file         demos/modbus/ARM_CORTEXM_ST_Nucleo_F091RC/bsp.c
* \brief        Board support package source file.
* \details      This board support package for the Nucleo-F091RC implements the following
*               pin mapping:
*
*               Digital Outputs
*               ---------------
*               D13 = PA5  = LED = BSP_DIGITAL_OUT1
*               D12 = PA6        = BSP_DIGITAL_OUT2
*               
*               PWM Outputs
*               -----------
*               D5  = PB4  = TIM3_CH1 = BSP_PWM_OUT1
*               D11 = PA7  = TIM3_CH2 = BSP_PWM_OUT2
*               
*               Digital Inputs
*               --------------
*               n/a = PC13 = Pushbutton (low active)      = BSP_DIGITAL_IN1
*               D4  = PB5  = Pull-up enabled (low active) = BSP_DIGITAL_IN2
*               
*               Analog Inputs
*               -------------
*               A5  = PC0  = ADC_IN10 = BSP_ANALOG_IN1
*               A4  = PC1  = ADC_IN11 = BSP_ANALOG_IN2
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
#include "microtbx.h"                            /* MicroTBX base library              */
#include "microtbxmodbus.h"                      /* MicroTBX-Modbus library            */
#include "bsp.h"                                 /* Board support package              */
#include "stm32f0xx.h"                           /* STM32 CPU and HAL                  */
#include "stm32f0xx_ll_rcc.h"                    /* STM32 LL RCC                       */
#include "stm32f0xx_ll_system.h"                 /* STM32 LL SYSTEM                    */
#include "stm32f0xx_ll_utils.h"                  /* STM32 LL UTILS                     */
#include "stm32f0xx_ll_bus.h"                    /* STM32 LL BUS                       */
#include "stm32f0xx_ll_gpio.h"                   /* STM32 LL GPIO                      */
#include "stm32f0xx_ll_tim.h"                    /* STM32 LL TIM                       */
#include "stm32f0xx_ll_dma.h"                    /* STM32 LL DMA                       */
#include "stm32f0xx_ll_adc.h"                    /* STM32 LL ADC                       */


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void SystemClock_Config(void);


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Buffer where the DMA stores the analog to digital conversion results. */
static volatile uint16_t adcResult[BSP_NUM_ANALOG_IN] = { 0 };


/************************************************************************************//**
** \brief     Initializes the microcontroller clocks, periperals and interrupts for the 
**            purpose of the demo application.
**
****************************************************************************************/
void BspInit(void)
{
  LL_GPIO_InitTypeDef    GPIO_InitStruct    = { 0 };
  LL_TIM_InitTypeDef     TIM_InitStruct     = { 0 };
  LL_TIM_OC_InitTypeDef  TIM_OC_InitStruct  = { 0 };
  LL_RCC_ClocksTypeDef   RCC_ClocksStruct   = { 0 };
  LL_ADC_InitTypeDef     ADC_InitStruct     = { 0 };
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = { 0 };

  /* Disable the interrupts. */
  __disable_irq();

  /* Configure the system clock. */
  SystemClock_Config();

  /* Enable peripheral clocks. */
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM7);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_ADC1);

  /* Configure interrupt priorities. Note that 3 is lowest priority on the STM32F0. */
  NVIC_SetPriority(PendSV_IRQn,    3U);
  NVIC_SetPriority(SysTick_IRQn,   3U);
  NVIC_SetPriority(USART1_IRQn,    3U);
  NVIC_SetPriority(USART2_IRQn,    3U);
  NVIC_SetPriority(USART3_8_IRQn,  3U);

  /* Configure the digital input GPIO pins:
   *   - PC13 = Pushbutton (has external pull-up) = BSP_DIGITAL_IN1
   *   - PB5  = D4 (internal pull-up enabled)     = BSP_DIGITAL_IN2
   */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure the analog input GPIO pins:
   *   - PC0 = A5  = ADC_IN10 = BSP_ANALOG_IN1
   *   - PC1 = A4  = ADC_IN11 = BSP_ANALOG_IN2
   */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0 | LL_GPIO_PIN_1;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure the digital output GPIO pins:
   *   - PA5 = D13 = LED  = BSP_DIGITAL_OUT1
   *   - PA6 = D12        = BSP_DIGITAL_OUT2
   */
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5 | LL_GPIO_PIN_6);
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Configure the USART1 RS485 transceiver DE/NRE GPIO pin. */
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);  
  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 TX and RX GPIO pin configuration. */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9 | LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 TX and RX GPIO pin configuration. */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART3 TX and RX GPIO pin configuration. */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_10 | LL_GPIO_PIN_11;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* TIM3 GPIO configuration for PWM outputs:
   *   - PB4 = TIM3 channel 1 = D5  = BSP_PWM_OUT1
   *   - PA7 = TIM3 channel 2 = D11 = BSP_PWM_OUT2
   */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_1;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LL_GPIO_PIN_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TIM3 configuration for 1 kHz PWM outputs with an 8-bit duty cycle, driven by 
   * APB1/PCLK1.
   */
  LL_RCC_GetSystemClocksFreq(&RCC_ClocksStruct);
  TIM_InitStruct.Prescaler = (RCC_ClocksStruct.PCLK1_Frequency / (1000U * 256U)) - 1U;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 255U;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_EnableARRPreload(TIM3);
  /* TIM3 channel 1 and 2 configuration for PWM output generation. */
  TIM_OC_InitStruct.OCMode       = LL_TIM_OCMODE_PWM1;
  TIM_OC_InitStruct.OCState      = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.OCNState     = LL_TIM_OCSTATE_DISABLE;
  TIM_OC_InitStruct.CompareValue = 0U;
  TIM_OC_InitStruct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCNPolarity  = LL_TIM_OCPOLARITY_HIGH;
  TIM_OC_InitStruct.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;
  TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
  LL_TIM_OC_Init(TIM3, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH2);
  LL_TIM_EnableCounter(TIM3);
  LL_TIM_GenerateEvent_UPDATE(TIM3);

  /* TIM7 configuration for a free running counter at 20 kHz, driven by APB1/PCLK1. */
  LL_RCC_GetSystemClocksFreq(&RCC_ClocksStruct);
  TIM_InitStruct.Prescaler = (RCC_ClocksStruct.PCLK1_Frequency / 20000U) - 1U;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535U;
  LL_TIM_Init(TIM7, &TIM_InitStruct);
  LL_TIM_EnableCounter(TIM7);

  /* DMA1 channel 1 configuration for storing the ADC1 conversion results. */
  LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1,
                        LL_DMA_DIRECTION_PERIPH_TO_MEMORY |
                        LL_DMA_MODE_CIRCULAR              |
                        LL_DMA_PERIPH_NOINCREMENT         |
                        LL_DMA_MEMORY_INCREMENT           |
                        LL_DMA_PDATAALIGN_HALFWORD        |
                        LL_DMA_MDATAALIGN_HALFWORD        |
                        LL_DMA_PRIORITY_HIGH);
  /* Set DMA transfer addresses of source and destination. */
  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
                         LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA),
                         (uint32_t)adcResult, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
  /* Set DMA transfer size. */
  LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, BSP_NUM_ANALOG_IN);
  /* Enable the DMA transfer. */
  LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

  /* Configure the global features of the ADC for continuous scan mode of the ADC
   * channels 10 and 11 in combination with DMA for storing the results.
   */
  ADC_InitStruct.Clock = LL_ADC_CLOCK_ASYNC;
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_10B;
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_41CYCLES_5);
  /* Configure the regular channels. */
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_10);
  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_11);
  /* Enable ADC and start the continuous conversion. */
  LL_ADC_Enable(ADC1);
  LL_ADC_REG_StartConversion(ADC1);

  /* Enable interrupts in the NVIC. Note that FreeRTOS handles PendSV and SysTick. */
  NVIC_EnableIRQ(USART1_IRQn);
  NVIC_EnableIRQ(USART2_IRQn);
  NVIC_EnableIRQ(USART3_8_IRQn);

  /* Enable the interrupts. */
  __enable_irq();
} /*** end of BspInit ***/


/************************************************************************************//**
** \brief     Changes the state of the specified digital output.
** \param     pin Digital output identifier.
** \param     value TBX_ON to set the digital output to logic high, TBX_OFF for logic
**            low.
**
****************************************************************************************/
void BspDigitalOut(tBspDigitalOut pin,
                   uint8_t        value)
{
  uint32_t const pinMask[BSP_NUM_DIGITAL_OUT] = { LL_GPIO_PIN_5, LL_GPIO_PIN_6 };

  /* Verify parameters. */
  TBX_ASSERT(pin < BSP_NUM_DIGITAL_OUT);

  /* Only continue with valid parameters. */
  if (pin < BSP_NUM_DIGITAL_OUT)
  {
    if (value != TBX_OFF)
    {
      LL_GPIO_SetOutputPin(GPIOA, pinMask[pin]);
    }
    else
    {
      LL_GPIO_ResetOutputPin(GPIOA, pinMask[pin]);
    }
  }
} /*** end of BspDigitalOut ***/


/************************************************************************************//**
** \brief     Reads the state of the specified digital input.
** \param     pin Digital input identifier.
** \return    TBX_ON if the digital input is on, TBX_OFF if off.
**
****************************************************************************************/
uint8_t BspDigitalIn(tBspDigitalIn pin)
{
  uint8_t              result = TBX_OFF;
  uint32_t       const pinMask[BSP_NUM_DIGITAL_IN]  = { LL_GPIO_PIN_13, LL_GPIO_PIN_5 };
  GPIO_TypeDef * const portMask[BSP_NUM_DIGITAL_IN] = { GPIOC, GPIOB };

  /* Verify parameters. */
  TBX_ASSERT(pin < BSP_NUM_DIGITAL_IN);

  /* Only continue with valid parameters. */
  if (pin < BSP_NUM_DIGITAL_IN)
  {
    /* The digital inputs have a pull-up, making them low-active. When the pin reads 
     * logic high, it is considered to be off. When it reads logic low, it is considered
     * to be on.
     */
    if (LL_GPIO_IsInputPinSet(portMask[pin], pinMask[pin]) == 0U)
    {
      /* Update the result for the on-state. */
      result = TBX_ON;
    }
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of BspDigitalIn ***/


/************************************************************************************//**
** \brief     Changes the duty cycle of the specified PWM output.
** \param     pin PWM output identifier.
** \param     duty New duty-cycle in the range 0..255 for 0..100%.
**
****************************************************************************************/
void BspPwmOut(tBspPwmOut pin,
               uint8_t    duty)
{
  static uint8_t currentDuty[BSP_NUM_PWM_OUT] = { 0 };

  /* Verify parameters. */
  TBX_ASSERT(pin < BSP_NUM_PWM_OUT);

  /* Only continue with valid parameters. */
  if (pin < BSP_NUM_PWM_OUT)
  {
    /* Did the duty-cycle actually change? */
    if (currentDuty[pin] != duty)
    {
      /* Update the duty-cycle. */
      if (pin == BSP_PWM_OUT1)
      {
        LL_TIM_OC_SetCompareCH1(TIM3, duty);
      }
      else
      {
        LL_TIM_OC_SetCompareCH2(TIM3, duty);
      }
      /* Store the new duty-cycle for change detection. */
      currentDuty[pin] = duty;
    }
  }
} /*** end of BspPwmOut ***/


/************************************************************************************//**
** \brief     Reads the state of the specified analog input.
** \param     pin Analog input identifier.
** \return    Latest analog to digital conversion result as a 10-bit value (0..1023).
**
****************************************************************************************/
uint16_t BspAnalogIn(tBspAnalogIn pin)
{
  uint16_t result = 0U;

  /* Verify parameters. */
  TBX_ASSERT(pin < BSP_NUM_ANALOG_IN);

  /* Only continue with valid parameters. */
  if (pin < BSP_NUM_ANALOG_IN)
  {
    /* Store the ADC conversion result of the specific analog input. */
    result = adcResult[pin];
  }
  /* Give the result back to the caller. */
  return result;
} /*** end of BspAnalogIn ***/


/************************************************************************************//**
** \brief     System Clock Configuration. This code was created by CubeMX and configures
**            the system clock.
**
****************************************************************************************/
static void SystemClock_Config(void)
{
  /* Set flash latency. */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  /* Verify flash latency setting. */
  if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
    /* Error setting flash latency. */
    TBX_ASSERT(TBX_FALSE);
  }

  /* Enable the HSE clock and bypass as the MCO signal from the ST-link is used. */
  LL_RCC_HSE_EnableBypass();
  LL_RCC_HSE_Enable();
  /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    ;
  }
  /* Configure and enable the PLL. */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLL_MUL_12, LL_RCC_PREDIV_DIV_2);
  LL_RCC_PLL_Enable();
  /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    ;
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
    ;
  }
  /* Update the system clock speed setting. */
  LL_SetSystemCoreClock(48000000UL);
} /*** end of SystemClock_Config ***/


/*********************************** end of bsp.c **************************************/
