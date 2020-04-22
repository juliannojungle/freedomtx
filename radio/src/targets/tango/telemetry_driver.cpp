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

uint32_t telemetryErrors = 0;
Fifo<uint8_t, TELEMETRY_BUFFER_SIZE> crsf_telemetry_buffer;

void telemetryPortInit(uint32_t baudrate, uint8_t mode)
{
}

void telemetryPortSetDirectionOutput()
{
}

void sportSendBuffer(const uint8_t * buffer, uint32_t count)
{
}

uint8_t telemetryGetByte(uint8_t * byte)
{
  bool res = crsf_telemetry_buffer.pop(*byte);
#if defined(LUA)
  if (telemetryProtocol == PROTOCOL_TELEMETRY_CROSSFIRE)
  {
    static uint8_t prevdata;
    if (prevdata == 0x7E && outputTelemetryBufferSize > 0 && *byte == outputTelemetryBufferTrigger) {
      sportSendBuffer(outputTelemetryBuffer, outputTelemetryBufferSize);
    }
    prevdata = *byte;
  }
#endif
  return res;
}

