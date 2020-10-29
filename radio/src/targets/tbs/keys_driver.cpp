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

#if defined(ROTARY_ENCODER_NAVIGATION)
uint32_t rotencPositionValue;
#endif

#if defined(PCBTANGO)
uint8_t  g_trimState = 0;
#elif defined(PCBMAMBO)
#define POT_MULTIPOS_SWITCH_ADC_TOLERANCE   128
#define POT_3POS_SWITCH_POS1_ADC  0
#define POT_3POS_SWITCH_POS2_ADC  1024
#define POT_3POS_SWITCH_POS3_ADC  2048

#define TRIM_ADC_OFFSET             20
#define IS_TRIM_PRESSED_STATE(x)    ((x - TRIM_ADC_OFFSET) < trimValue && trimValue < (x + TRIM_ADC_OFFSET))
#endif
uint32_t readKeys()
{
  uint32_t result = 0;

  if (~KEYS_GPIO_REG_ENTER & KEYS_GPIO_PIN_ENTER)
    result |= 1 << KEY_ENTER;

#if defined(KEYS_GPIO_PIN_MENU)
  if (~KEYS_GPIO_REG_MENU & KEYS_GPIO_PIN_MENU)
    result |= 1 << KEY_MENU;
#endif

#if defined(KEYS_GPIO_PIN_PAGE)
  if (~KEYS_GPIO_REG_PAGE & KEYS_GPIO_PIN_PAGE)
    result |= 1 << KEY_PAGE;
#endif

  if (~KEYS_GPIO_REG_EXIT & KEYS_GPIO_PIN_EXIT)
    result |= 1 << KEY_EXIT;

#if defined(KEYS_GPIO_PIN_PLUS)
  if (~KEYS_GPIO_REG_PLUS & KEYS_GPIO_PIN_PLUS)
    result |= 1 << KEY_PLUS;
  if (~KEYS_GPIO_REG_MINUS & KEYS_GPIO_PIN_MINUS)
    result |= 1 << KEY_MINUS;
#endif

#if defined(KEYS_GPIO_PIN_LEFT)
  if (~KEYS_GPIO_REG_LEFT & KEYS_GPIO_PIN_LEFT)
    result |= 1 << KEY_LEFT;
  if (~KEYS_GPIO_REG_RIGHT & KEYS_GPIO_PIN_RIGHT)
    result |= 1 << KEY_RIGHT;
  if (~KEYS_GPIO_REG_UP & KEYS_GPIO_PIN_UP)
    result |= 1 << KEY_UP;
  if (~KEYS_GPIO_REG_DOWN & KEYS_GPIO_PIN_DOWN)
    result |= 1 << KEY_DOWN;
#endif

  return result;
}

uint32_t readTrims()
{
  uint32_t result = 0;

#if defined(PCBTANGO)  
  // the trim state from the events of per10ms()
  result = g_trimState;
  g_trimState = 0;
#elif defined(PCBMAMBO)
  #if !defined(SIMU) && !defined (BOOT)
  uint16_t trimValue = anaIn(TX_TRIM);
  if (trimValue < 1024) {
    if (IS_TRIM_PRESSED_STATE(991))
      result |= 0x01;
    if (IS_TRIM_PRESSED_STATE(719))
      result |= 0x02;
    if (IS_TRIM_PRESSED_STATE(587))
      result |= 0x04;
    if (IS_TRIM_PRESSED_STATE(504))
      result |= 0x08;

    if (IS_TRIM_PRESSED_STATE(387))
      result |= 0x10;
    if (IS_TRIM_PRESSED_STATE(445))
      result |= 0x20;
    if (IS_TRIM_PRESSED_STATE(320))
      result |= 0x40;
    if (IS_TRIM_PRESSED_STATE(245))
      result |= 0x80;
  }
  #endif
#endif

  return result;
}

bool trimDown(uint8_t idx)
{
  return readTrims() & (1 << idx);
}

bool keyDown()
{
  return readKeys() || readTrims();
}

/* TODO common to ARM */
void readKeysAndTrims()
{
  uint8_t index = 0;
  uint32_t keys_input = readKeys();
  for (uint8_t i = 1; i != uint8_t(1 << TRM_BASE); i <<= 1) {
    keys[index++].input(keys_input & i);
  }

  uint32_t trims_input = readTrims();
  for (uint8_t i = 1; i != uint8_t(1 << 8); i <<= 1) {
    keys[index++].input(trims_input & i);
  }

#if defined(PWR_BUTTON_PRESS)
  if ((keys_input || trims_input || pwrPressed()) && (g_eeGeneral.backlightMode & e_backlight_mode_keys)) {
#else
  if ((keys_input || trims_input) && (g_eeGeneral.backlightMode & e_backlight_mode_keys)) {
#endif
    // on keypress turn the light on
    resetBacklightTimeout();
  }
}

#define ADD_2POS_CASE(x) \
  case SW_S ## x ## 0: \
    xxx = SWITCHES_GPIO_REG_ ## x  & SWITCHES_GPIO_PIN_ ## x ; \
    break; \
  case SW_S ## x ## 2: \
    xxx = ~SWITCHES_GPIO_REG_ ## x  & SWITCHES_GPIO_PIN_ ## x ; \
    break;

#if defined(PCBTANGO)  
#define ADD_3POS_CASE(x, i) \
  case SW_S ## x ## 0: \
    xxx = (SWITCHES_GPIO_REG_ ## x ## _H & SWITCHES_GPIO_PIN_ ## x ## _H); \
    if (IS_CONFIG_3POS(i)) { \
      xxx = xxx && (~SWITCHES_GPIO_REG_ ## x ## _L & SWITCHES_GPIO_PIN_ ## x ## _L); \
    } \
    break; \
  case SW_S ## x ## 1: \
    xxx = (SWITCHES_GPIO_REG_ ## x ## _H & SWITCHES_GPIO_PIN_ ## x ## _H) && (SWITCHES_GPIO_REG_ ## x ## _L & SWITCHES_GPIO_PIN_ ## x ## _L); \
    break; \
  case SW_S ## x ## 2: \
    xxx = (~SWITCHES_GPIO_REG_ ## x ## _H & SWITCHES_GPIO_PIN_ ## x ## _H); \
    if (IS_CONFIG_3POS(i)) { \
      xxx = xxx && (SWITCHES_GPIO_REG_ ## x ## _L & SWITCHES_GPIO_PIN_ ## x ## _L); \
    } \
    break
#elif defined(PCBMAMBO)
#define ADD_3POS_CASE(x) \
  case SW_S ## x ## 0: \
    xxx = (anaIn(SWITCH_ ## x) < POT_3POS_SWITCH_POS1_ADC + POT_MULTIPOS_SWITCH_ADC_TOLERANCE) ? 1 : 0; \
    break; \
  case SW_S ## x ## 1: \
    xxx = (anaIn(SWITCH_ ## x) < POT_3POS_SWITCH_POS2_ADC + POT_MULTIPOS_SWITCH_ADC_TOLERANCE && \
           anaIn(SWITCH_ ## x) > POT_3POS_SWITCH_POS2_ADC - POT_MULTIPOS_SWITCH_ADC_TOLERANCE) ? 1 : 0; \
    break; \
  case SW_S ## x ## 2: \
    xxx = (anaIn(SWITCH_ ## x) > POT_3POS_SWITCH_POS3_ADC - POT_MULTIPOS_SWITCH_ADC_TOLERANCE) ? 1 : 0; \
    break
#endif

uint8_t keyState(uint8_t index)
{
  return keys[index].state();
}

#if !defined(BOOT)
uint32_t switchState(uint8_t index)
{
  uint32_t xxx = 0;

  switch (index) {
    ADD_2POS_CASE(A);
#if defined(PCBTANGO)
    ADD_3POS_CASE(B, 1);
    ADD_3POS_CASE(C, 2);
    ADD_2POS_CASE(D);
    ADD_2POS_CASE(E);
    ADD_2POS_CASE(F);
#elif defined(PCBMAMBO)
    ADD_3POS_CASE(B);
    ADD_3POS_CASE(C);
    ADD_3POS_CASE(D);
    ADD_3POS_CASE(E);
    ADD_2POS_CASE(F);
#endif

    default:
      break;
  }

  // TRACE("switch %d => %d", index, xxx);
  return xxx;
}
#endif

void keysInit()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;

#if defined(KEYS_GPIOA_PINS)
  INIT_KEYS_PINS(GPIOA);
#endif

#if defined(KEYS_GPIOB_PINS)
  INIT_KEYS_PINS(GPIOB);
#endif

#if defined(KEYS_GPIOC_PINS)
  INIT_KEYS_PINS(GPIOC);
#endif

#if defined(KEYS_GPIOD_PINS)
  INIT_KEYS_PINS(GPIOD);
#endif

#if defined(KEYS_GPIOE_PINS)
  INIT_KEYS_PINS(GPIOE);
#endif

#if defined(KEYS_GPIOF_PINS)
  INIT_KEYS_PINS(GPIOF);
#endif

#if defined(KEYS_GPIOG_PINS)
  INIT_KEYS_PINS(GPIOG);
#endif

#if defined(ROTARY_ENCODER_NAVIGATION)
  rotencPositionValue = ROTARY_ENCODER_POSITION();
#endif
}
