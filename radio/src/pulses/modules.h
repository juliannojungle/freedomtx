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

#ifndef _MODULES_H_
#define _MODULES_H_

#include "myeeprom.h"
#include "bitfield.h"

#define CROSSFIRE_CHANNELS_COUNT        16

#if defined(MULTIMODULE)
inline bool isModuleMultimodule(uint8_t idx)
{
  return idx == EXTERNAL_MODULE && g_model.moduleData[EXTERNAL_MODULE].type == MODULE_TYPE_MULTIMODULE;
}

inline bool isModuleMultimoduleDSM2(uint8_t idx)
{
  return isModuleMultimodule(idx) && g_model.moduleData[idx].getMultiProtocol() == MM_RF_PROTO_DSM2;
}
#else
inline bool isModuleMultimodule(uint8_t)
{
  return false;
}

inline bool isModuleMultimoduleDSM2(uint8_t)
{
  return false;
}
#endif

inline bool isModuleXJT(uint8_t idx)
{
  return g_model.moduleData[idx].type == MODULE_TYPE_PXX_XJT;
}

inline bool isModuleXJT2(uint8_t idx)
{
  return g_model.moduleData[idx].type == MODULE_TYPE_ACCESS_ISRM;
}

inline bool isModuleXJTVariant(uint8_t idx)
{
  return g_model.moduleData[idx].type == MODULE_TYPE_PXX_XJT || g_model.moduleData[idx].type == MODULE_TYPE_ACCESS_ISRM;
}


#if defined(CROSSFIRE)
inline bool isModuleCrossfire(uint8_t idx)
{
  return idx == EXTERNAL_MODULE && g_model.moduleData[EXTERNAL_MODULE].type == MODULE_TYPE_CROSSFIRE;
}
#else
inline bool isModuleCrossfire(uint8_t idx)
{
  return false;
}
#endif

#if defined(EXTRA_MODULE)
inline bool isExtraModule(uint8_t idx)
{
  return idx == EXTRA_MODULE);
}
#else
inline bool isExtraModule(uint8_t)
{
  return false;
}
#endif

#if defined(TARANIS_INTERNAL_PPM)
inline bool isModulePPM(uint8_t idx)
{
  return (idx == INTERNAL_MODULE && g_model.moduleData[INTERNAL_MODULE].type == MODULE_TYPE_PPM) ||
         (idx == EXTERNAL_MODULE && g_model.moduleData[EXTERNAL_MODULE].type == MODULE_TYPE_PPM);
}
#else
inline bool isModulePPM(uint8_t idx)
{
  return isExtraModule(idx) ||
         (idx == EXTERNAL_MODULE && g_model.moduleData[EXTERNAL_MODULE].type == MODULE_TYPE_PPM);
}
#endif

inline bool isModuleR9M(uint8_t idx)
{
  return g_model.moduleData[idx].type == MODULE_TYPE_PXX_R9M || g_model.moduleData[idx].type == MODULE_TYPE_PXX_R9M_LITE;
}

inline bool isModuleR9M2(uint8_t idx)
{
  return g_model.moduleData[idx].type == MODULE_TYPE_ACCESS_R9M || g_model.moduleData[idx].type == MODULE_TYPE_ACCESS_R9M_LITE || g_model.moduleData[idx].type == MODULE_TYPE_ACCESS_R9M_LITE_PRO;
}


inline bool isModuleR9M_FCC(uint8_t idx)
{
  return isModuleR9M(idx) && g_model.moduleData[idx].subType == MODULE_SUBTYPE_R9M_FCC;
}

inline bool isModuleR9M_LBT(uint8_t idx)
{
  return isModuleR9M(idx) && g_model.moduleData[idx].subType == MODULE_SUBTYPE_R9M_EU;
}

inline bool isModuleR9M_FCC_VARIANT(uint8_t idx)
{
  return isModuleR9M(idx) && g_model.moduleData[idx].subType != MODULE_SUBTYPE_R9M_EU;
}

inline bool isModuleR9M_EUPLUS(uint8_t idx)
{
  return isModuleR9M(idx) && g_model.moduleData[idx].subType == MODULE_SUBTYPE_R9M_EUPLUS;
}

inline bool isModuleR9M_AU_PLUS(uint8_t idx)
{
  return isModuleR9M(idx) && g_model.moduleData[idx].subType == MODULE_SUBTYPE_R9M_AUPLUS;
}

inline bool isModulePXX(uint8_t idx)
{
  return isModuleXJT(idx) || isModuleR9M(idx);
}

inline bool isModulePXX2(uint8_t idx)
{
  return isModuleXJT2(idx) || isModuleR9M2(idx);
}

#if defined(DSM2)
inline bool isModuleDSM2(uint8_t idx)
{
  return idx == EXTERNAL_MODULE && g_model.moduleData[EXTERNAL_MODULE].type == MODULE_TYPE_DSM2;
}

inline bool isModuleSBUS(uint8_t idx)
{
  return idx == EXTERNAL_MODULE && g_model.moduleData[EXTERNAL_MODULE].type == MODULE_TYPE_SBUS;
}
#else
inline bool isModuleDSM2(uint8_t idx)
{
  return false;
}
#endif

// order is the same as in enum Protocols in myeeprom.h (none, ppm, pxx, pxx2, dsm, crossfire, multi, r9m, r9m2, sbus)
static const int8_t maxChannelsModules[] = { 0, 8, 8, 16, -2, 8, 4, 8, 16, 8}; // relative to 8!
static const int8_t maxChannelsXJT[] = { 0, 8, 0, 4 }; // relative to 8!

constexpr int8_t MAX_TRAINER_CHANNELS_M8 = MAX_TRAINER_CHANNELS - 8;
constexpr int8_t MAX_EXTRA_MODULE_CHANNELS_M8 = 8; // only 16ch PPM

inline int8_t maxModuleChannels_M8(uint8_t idx)
{
  if (isExtraModule(idx))
    return MAX_EXTRA_MODULE_CHANNELS_M8;
  else if (isModuleXJT(idx))
    return maxChannelsXJT[1 + g_model.moduleData[idx].rfProtocol];
  else
    return maxChannelsModules[g_model.moduleData[idx].type];
}

inline int8_t defaultModuleChannels_M8(uint8_t idx)
{
  if (isModulePPM(idx))
    return 0; // 8 channels
  else if (isModuleDSM2(idx))
    return 0; // 8 channels
  else if (isModuleMultimoduleDSM2(idx))
    return -1; // 7 channels
  else if (isModulePXX2(idx))
    return 8; // 16 channels
  else
    return maxModuleChannels_M8(idx);
}

inline int8_t sentModuleChannels(uint8_t idx)
{
  if (isModuleCrossfire(idx))
    return CROSSFIRE_CHANNELS_COUNT;
  else if (isModuleMultimodule(idx) && !isModuleMultimoduleDSM2(idx))
    return 16;
  else
    return 8 + g_model.moduleData[idx].channelsCount;
}

enum {
  MODULE_OPTION_RF_PROTOCOL,
  MODULE_OPTION_EXTERNAL_ANTENNA,
  MODULE_OPTION_POWER,
  MODULE_OPTION_SPECTRUM_ANALYSER,
  MODULE_OPTION_POWER_METER,
};

/* Options order:
 * - RF Protocol (0x01)
 * - External antenna (0x02)
 * - Power (0x04)
 * - Spektrum analyser (0x08)
 * - Power meter (0x10)
 */
static const uint8_t moduleOptions[] = {
#if defined(SIMU)
  0b11111111, // None = display all options on SIMU
#else
  0b00000000, // None = display all options on SIMU
#endif
  0b11100010, // XJT
  0b11100010, // ISRM
  0b11111010, // ISRM-PRO
  0b11101010, // ISRM-S
  0b11100100, // R9M
  0b11100100, // R9MLite
  0b11111100, // R9MLite-PRO
  0b11101000, // ISRM-N
};

inline bool isModuleOptionAvailable(uint8_t modelId, uint8_t option)
{
  return moduleOptions[modelId] & (1 << option);
}

inline bool isDefaultModelRegistrationID()
{
  return memcmp(g_model.modelRegistrationID, g_eeGeneral.ownerRegistrationID, PXX2_LEN_REGISTRATION_ID) == 0;
}

#endif // _MODULES_H_
