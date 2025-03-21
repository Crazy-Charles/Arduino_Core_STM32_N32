/*
 *******************************************************************************
 * Copyright (c) 2018, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 * Based on mbed-os/target/TARGET_STM/TARGET_STMYY/pin_device.h
 */
#ifndef _PINCONFIG_H
#define _PINCONFIG_H

#include "PinAF_STM32F1.h"
#include "stm32f1xx_ll_gpio.h"

static inline void pin_DisconnectDebug(PinName pin)
{

  pinF1_DisconnectDebug(pin);

}

static inline void pin_PullConfig(GPIO_TypeDef *gpio, uint32_t ll_pin, uint32_t pull_config)
{

uint32_t function = LL_GPIO_GetPinMode(gpio, ll_pin);

  switch (pull_config) {
    case GPIO_PULLUP:
      if (function == LL_GPIO_MODE_FLOATING) {
        LL_GPIO_SetPinMode(gpio, ll_pin, LL_GPIO_MODE_INPUT);
      }

      LL_GPIO_SetPinPull(gpio, ll_pin, LL_GPIO_PULL_UP);
      break;
    case GPIO_PULLDOWN:
      if (function == LL_GPIO_MODE_FLOATING) {
        LL_GPIO_SetPinMode(gpio, ll_pin, LL_GPIO_MODE_INPUT);
      }

      LL_GPIO_SetPinPull(gpio, ll_pin, LL_GPIO_PULL_DOWN);
      break;
    default:
      /*  Input+NoPull = Floating for F1 family */
      if (function == LL_GPIO_MODE_INPUT) {
        LL_GPIO_SetPinMode(gpio, ll_pin, LL_GPIO_MODE_FLOATING);
      }
      break;
  }
}

static inline void pin_SetAFPin(GPIO_TypeDef *gpio, PinName pin, uint32_t afnum)
{

  UNUSED(gpio);
  UNUSED(pin);
  pin_SetF1AFPin(afnum);

}

#endif
