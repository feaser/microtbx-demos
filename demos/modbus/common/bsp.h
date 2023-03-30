/************************************************************************************//**
* \file         demos/modbus/common/bsp.h
* \brief        Board support package header file.
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
#ifndef BSP_H
#define BSP_H

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Enumerated type with all supported digital inputs. */
typedef enum
{
  BSP_DIGITAL_IN1 = 0U,
  BSP_DIGITAL_IN2,
  BSP_NUM_DIGITAL_IN
} tBspDigitalIn;


/** \brief Enumerated type with all supported digital outputs. */
typedef enum
{
  BSP_DIGITAL_OUT1 = 0U,
  BSP_DIGITAL_OUT2,
  BSP_NUM_DIGITAL_OUT
} tBspDigitalOut;


/** \brief Enumerated type with all supported analog inputs. */
typedef enum
{
  BSP_ANALOG_IN1 = 0U,
  BSP_ANALOG_IN2,
  BSP_NUM_ANALOG_IN
} tBspAnalogIn;


/** \brief Enumerated type with all supported PWM outputs. */
typedef enum
{
  BSP_PWM_OUT1 = 0U,
  BSP_PWM_OUT2,
  BSP_NUM_PWM_OUT
} tBspPwmOut;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void     BspInit          (void);

void     BspDigitalOut    (tBspDigitalOut pin,
                           uint8_t        value);

uint8_t  BspDigitalIn     (tBspDigitalIn  pin);

void     BspPwmOut        (tBspPwmOut     pin,
                           uint8_t        duty);

uint16_t BspAnalogIn      (tBspAnalogIn   pin);


#ifdef __cplusplus
}
#endif

#endif /* BSP_H */
/*********************************** end of bsp.h **************************************/
