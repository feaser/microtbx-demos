/************************************************************************************//**
* \file         demos/modbus/ARM_CORTEXM_ST_Nucleo_F429ZI/bsp.c
* \brief        Board support package source file.
* \details      This board support package for the Nucleo-F091RC implements the following
*               pin mapping:
*
*               Digital Outputs
*               ---------------
*               D33 = PB0  = Green LED = BSP_DIGITAL_OUT1
*               D22 = PB5              = BSP_DIGITAL_OUT2
*               
*               PWM Outputs
*               -----------
*               D5  = PB4  = TIM3_CH1 = BSP_PWM_OUT1  TODO ##Vg Update
*               D11 = PA7  = TIM3_CH2 = BSP_PWM_OUT2  TODO ##Vg Update
*               
*               Digital Inputs
*               --------------
*               n/a = PC13 = Pushbutton (high active)        = BSP_DIGITAL_IN1
*               D4  = PF14 = Pull-down enabled (high active) = BSP_DIGITAL_IN2
*               
*               Analog Inputs
*               -------------
*               A5  = PC0  = ADC_IN10 = BSP_ANALOG_IN1 TODO ##Vg Update
*               A4  = PC1  = ADC_IN11 = BSP_ANALOG_IN2 TODO ##Vg Update
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

  /* Configure the system clock. */
  SystemClock_Config();

  /* Enable peripheral clocks. */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* Set interrupt group priority. Needs to be NVIC_PRIORITYGROUP_4 for FreeRTOS. */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);  

  /* Configure interrupt priorities. Note that 15 is the lowest priority on the STM32F4.
   * When using FreeRTOS, the NVIC priority number should be >=
   * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY if the associated interrupt handlers
   * make use of FreeRTOS API calls.
   */
  HAL_NVIC_SetPriority(PendSV_IRQn,  15U, 0U);
  HAL_NVIC_SetPriority(SysTick_IRQn, 15U, 0U);
  HAL_NVIC_SetPriority(USART3_IRQn,  10U, 0U);

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

  /* USART3 TX and RX GPIO pin configuration. */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Enable interrupts in the NVIC. Note that FreeRTOS handles PendSV and SysTick. */
  HAL_NVIC_EnableIRQ(USART3_IRQn);

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
  /* TODO ##Vg Implement BspPwmOut(). */
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

  /* TODO ##Vg Implement BspAnalogIn(). */

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
**
****************************************************************************************/
void SysTick_Handler(void)
{
  /* Inform the HAL timebase about the event. */
  HAL_IncTick();
}


/*********************************** end of bsp.c **************************************/
