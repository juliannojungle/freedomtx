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

void backlightEnable(uint8_t level)
{
  // the scale is divided into two groups since the affect of contrast configuration is not so linear
  // system brightness 0  to 84  map to screen contrast 0   to 127
  // system brightness 81 to 100 map to screen contrast 127 to 255

  uint8_t value = 100-level;
  if(value >= 84)
    value = ((value-84) << 3) + 127;        // (value-84)*128/16+127;
  else
    value = (value << 5) / 21;              // value*128/84

  lcdAdjustContrast(value);
  lcdOn();
}