/************************************************************************************//**
* \file         demos/modbus/ARM_CORTEXM_ST_Nucleo_F429ZI/bsp.c
* \brief        Board support package source file.
* \details      This board support package for the Nucleo-F429ZI implements the following
*               pin mapping:
*
*               Digital Outputs
*               ---------------
*               D33 = PB0  = Green LED = BSP_DIGITAL_OUT1
*               D22 = PB5              = BSP_DIGITAL_OUT2
*               
*               PWM Outputs
*               -----------
*               D16 = PC6  = TIM3_CH1 = BSP_PWM_OUT1
*               D21 = PC7  = TIM3_CH2 = BSP_PWM_OUT2
*               
*               Digital Inputs
*               --------------
*               n/a = PC13 = Pushbutton (high active)        = BSP_DIGITAL_IN1
*               D4  = PF14 = Pull-down enabled (high active) = BSP_DIGITAL_IN2
*               
*               Analog Inputs
*               -------------
*               A5  = PF10 = ADC3_IN8 = BSP_ANALOG_IN1
*               A3  = PF3  = ADC3_IN9 = BSP_ANALOG_IN2
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
#include "microtbx.h"                            /* MicroTBX base library              */
#include "microtbxmodbus.h"                      /* MicroTBX-Modbus library            */
#include "bsp.h"                                 /* Board support package              */
#include "stm32f4xx.h"                           /* STM32 CPU and HAL                  */


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Handle to the TIM peripheral which will be used for the PWM outputs. */
static TIM_HandleTypeDef pwmTimHandle = {0};

/** \brief Handle to the ADC peripheral which will conver the analog inputs. */
static ADC_HandleTypeDef adcHandle = {0};

/** \brief Handle to the DMA peripheral which will be used for storing ADC conversions.*/
static DMA_HandleTypeDef adcDmaHandle = {0};

/** \brief Buffer where the DMA stores the analog to digital conversion results. */
static volatile uint16_t adcResult[BSP_NUM_ANALOG_IN] = { 0 };


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void SystemClock_Config(void);


/************************************************************************************//**
** \brief     Initializes the microcontroller clocks, periperals and interrupts for the 
**            purpose of the demo application.
**
****************************************************************************************/
void BspInit(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Disable the interrupts. */
  __disable_irq();

  /* Initialize the HAL. */
  HAL_Init();

  /* Configure the system clock. */
  SystemClock_Config();

  /* Enable peripheral clocks. */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_USART6_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM7_CLK_ENABLE();
  __HAL_RCC_ADC3_CLK_ENABLE(); 
  __HAL_RCC_DMA2_CLK_ENABLE();   
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /* Set interrupt group priority. Needs to be NVIC_PRIORITYGROUP_4 for FreeRTOS. */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);  

  /* Configure interrupt priorities. Note that 15 is the lowest priority on the STM32F4.
   * When using FreeRTOS, the NVIC priority number should be >=
   * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY if the associated interrupt handlers
   * make use of FreeRTOS API calls.
   */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn,  0U,  0U);
  HAL_NVIC_SetPriority(BusFault_IRQn,          0U,  0U);
  HAL_NVIC_SetPriority(UsageFault_IRQn,        0U,  0U);
  HAL_NVIC_SetPriority(SVCall_IRQn,            0U,  0U);
  HAL_NVIC_SetPriority(DebugMonitor_IRQn,      0U,  0U);
  HAL_NVIC_SetPriority(PendSV_IRQn,            15U, 0U);
  HAL_NVIC_SetPriority(SysTick_IRQn,           15U, 0U);
  HAL_NVIC_SetPriority(USART2_IRQn,            10U, 0U);
  HAL_NVIC_SetPriority(USART3_IRQn,            10U, 0U);
  HAL_NVIC_SetPriority(USART6_IRQn,            10U, 0U);

  /* Configure the digital input GPIO pins:
   *   - PC13 = Pushbutton (has external pull-down) = BSP_DIGITAL_IN1
   *   - PF14 = D4 (internal pull-down enabled)     = BSP_DIGITAL_IN2
   */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* Configure the digital output GPIO pins:
   *   - PB0 = D33 = Green LED  = BSP_DIGITAL_OUT1
   *   - PB5 = D22              = BSP_DIGITAL_OUT2
   */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_5, GPIO_PIN_RESET);

  /* Configure the analog input GPIO pins:
   *   - PF10 = A5  = ADC3_IN8 = BSP_ANALOG_IN1
   *   - PF3  = A3  = ADC3_IN9 = BSP_ANALOG_IN2
   */
  GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* TIM3 GPIO configuration for PWM outputs:
   *   - PC6 = TIM3 channel 1 = D16 = BSP_PWM_OUT1
   *   - PC7 = TIM3 channel 2 = D21 = BSP_PWM_OUT2
   */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* TIM3 configuration for 1 kHz PWM outputs with an 8-bit duty cycle, driven by 
   * APB1/PCLK1. Start by setting the TIM peripheral to use.
   */
  pwmTimHandle.Instance = TIM3;
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
  pwmTimHandle.Init.Prescaler         = (timFreq / (1000U * 256U)) - 1U;
  pwmTimHandle.Init.Period            = 255U;
  pwmTimHandle.Init.ClockDivision     = 0U;
  pwmTimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  pwmTimHandle.Init.RepetitionCounter = 0U;
  pwmTimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_PWM_Init(&pwmTimHandle);
  /* TIM3 channel 1 and 2 configuration for PWM output generation. */
  TIM_OC_InitTypeDef pwmChannelConfig = {0};
  pwmChannelConfig.OCMode       = TIM_OCMODE_PWM1;
  pwmChannelConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  pwmChannelConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  pwmChannelConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  pwmChannelConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  pwmChannelConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  pwmChannelConfig.Pulse        = 0U;
  HAL_TIM_PWM_ConfigChannel(&pwmTimHandle, &pwmChannelConfig, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&pwmTimHandle, &pwmChannelConfig, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&pwmTimHandle, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&pwmTimHandle, TIM_CHANNEL_2);

  /* USART2 TX and RX GPIO pin configuration. */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USART3 TX and RX GPIO pin configuration. */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Configure the USART6 RS485 transceiver DE/NRE GPIO pin. */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /* USART6 TX and RX GPIO pin configuration. */
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* DMA2 stream 0 channel 2 configuration for storing the ADC3 conversion results. */
  adcDmaHandle.Instance = DMA2_Stream0;
  adcDmaHandle.Init.Channel = DMA_CHANNEL_2;
  adcDmaHandle.Init.Direction = DMA_PERIPH_TO_MEMORY;
  adcDmaHandle.Init.PeriphInc = DMA_PINC_DISABLE;
  adcDmaHandle.Init.MemInc = DMA_MINC_ENABLE;
  adcDmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  adcDmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  adcDmaHandle.Init.Mode = DMA_CIRCULAR;
  adcDmaHandle.Init.Priority = DMA_PRIORITY_HIGH;
  adcDmaHandle.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&adcDmaHandle);
  /* Link the ADC and DMA handles. */
  __HAL_LINKDMA(&adcHandle, DMA_Handle, adcDmaHandle);
  /* Configure the global features of the ADC for continuous scan mode of the ADC
   * channels 8 and 9 in combination with DMA for storing the results.
   */
  adcHandle.Instance = ADC3;
  adcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  adcHandle.Init.Resolution = ADC_RESOLUTION_10B;
  adcHandle.Init.ScanConvMode = ENABLE;
  adcHandle.Init.ContinuousConvMode = ENABLE;
  adcHandle.Init.DiscontinuousConvMode = DISABLE;
  adcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  adcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  adcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  adcHandle.Init.NbrOfConversion = 2;
  adcHandle.Init.DMAContinuousRequests = ENABLE;
  adcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&adcHandle);
  /* Configure the regular channels. */
  ADC_ChannelConfTypeDef adcChannelConfig = {0};
  adcChannelConfig.Channel = ADC_CHANNEL_8;
  adcChannelConfig.Rank = 1;
  adcChannelConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&adcHandle, &adcChannelConfig);
  adcChannelConfig.Channel = ADC_CHANNEL_9;
  adcChannelConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&adcHandle, &adcChannelConfig);
  /* Enable ADC and start the continuous conversion. */
  HAL_ADC_Start_DMA(&adcHandle, (uint32_t *)&adcResult, 2);

  /* Enable interrupts in the NVIC. Note that FreeRTOS handles PendSV and SysTick. */
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  HAL_NVIC_EnableIRQ(USART6_IRQn);

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
  uint32_t const pinMask[BSP_NUM_DIGITAL_OUT] = { GPIO_PIN_0, GPIO_PIN_5 };

  /* Verify parameters. */
  TBX_ASSERT(pin < BSP_NUM_DIGITAL_OUT);

  /* Only continue with valid parameters. */
  if (pin < BSP_NUM_DIGITAL_OUT)
  {
    if (value != TBX_OFF)
    {
      HAL_GPIO_WritePin(GPIOB, pinMask[pin], GPIO_PIN_SET);
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, pinMask[pin], GPIO_PIN_RESET);
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
  uint32_t       const pinMask[BSP_NUM_DIGITAL_IN]  = { GPIO_PIN_13, GPIO_PIN_14 };
  GPIO_TypeDef * const portMask[BSP_NUM_DIGITAL_IN] = { GPIOC, GPIOF };

  /* Verify parameters. */
  TBX_ASSERT(pin < BSP_NUM_DIGITAL_IN);

  /* Only continue with valid parameters. */
  if (pin < BSP_NUM_DIGITAL_IN)
  {
    /* The digital inputs have a pull-down, making them high-active. When the pin reads 
     * logic low, it is considered to be off. When it reads logic high, it is considered
     * to be on.
     */
    if (HAL_GPIO_ReadPin(portMask[pin], pinMask[pin]) == GPIO_PIN_SET)
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
  static uint8_t currentDuty[BSP_NUM_PWM_OUT] = {0};

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
        __HAL_TIM_SET_COMPARE(&pwmTimHandle, TIM_CHANNEL_1, duty);
      }
      else
      {
        __HAL_TIM_SET_COMPARE(&pwmTimHandle, TIM_CHANNEL_2, duty);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure the main internal regulator output voltage. */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Initializes the RCC Oscillators according to the specified parameters in the 
   * RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    TBX_ASSERT(TBX_FALSE);
  }

  /* Initializes the CPU, AHB and APB buses clocks. */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    TBX_ASSERT(TBX_FALSE);    
  }
} /*** end of SystemClock_Config ***/


/****************************************************************************************
*            I N T E R R U P T   S E R V I C E   R O U T I N E S
****************************************************************************************/
/************************************************************************************//**
** \brief     SysTick interrupt handler. 
** \attention Weak attribute was added, because in FreeRTOS demos, a function with the
**            same name is used to process the system tick interrupt. The FreeRTOS
**            implementation should have priority. In this case, the HAL tick is 
**            handled in vApplicationTickHook().
**
****************************************************************************************/
__weak void SysTick_Handler(void)
{
  /* Inform the HAL timebase about the event. */
  HAL_IncTick();
} /*** end of SysTick_Handler ***/


/*********************************** end of bsp.c **************************************/
