/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "opentx.h"

void pwrInit()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = PWR_ON_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(PWR_ON_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = PWR_SWITCH_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(PWR_SWITCH_GPIO, &GPIO_InitStructure);

  pwrOn();
}

void pwrOn()
{
  GPIO_SetBits(PWR_ON_GPIO, PWR_ON_GPIO_PIN);
}

void pwrOff()
{
  GPIO_ResetBits(PWR_ON_GPIO, PWR_ON_GPIO_PIN);
}

bool pwrPressed()
{
  return GPIO_ReadInputDataBit(PWR_SWITCH_GPIO, PWR_SWITCH_GPIO_PIN) == Bit_SET;
}

void pwrResetHandler()
{
  RCC->AHB1ENR |= PWR_RCC_AHB1Periph;

  // these two NOPs are needed (see STM32F errata sheet) before the peripheral
  // register can be written after the peripheral clock was enabled
  __ASM volatile ("nop");
  __ASM volatile ("nop");

  if (WAS_RESET_BY_WATCHDOG_OR_SOFTWARE()) {
    pwrOn();
  }
}
