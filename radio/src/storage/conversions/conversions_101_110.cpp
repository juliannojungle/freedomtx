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
#include "string.h"
#include "datastructs_101.h"

#define INPUT_SOURCE_C1_101           79
#define INPUT_SOURCE_C2_101           80
#define INPUT_SOURCE_MAX_101          81
#define INPUT_SOURCE_CYC1_101         82
#define INPUT_SOURCE_CYC3_101         84
#define INPUT_SOURCE_TRIM_RUD_101     85
#define INPUT_SOURCE_L64_101          158
#define INPUT_SOURCE_TR1_101          159
#define INPUT_SOURCE_TR16_101         174
#define INPUT_SOURCE_CH1_101          175
#define INPUT_SOURCE_CH32_101         206
#define INPUT_SOURCE_CH1_110          157
#define INPUT_SOURCE_TELEM_FIRST_101  226
#define INPUT_SOURCE_TELEM_LAST_101   405
#define INPUT_SOURCE_TELEM_FIRST_110  208

#define MIX_SOURCE_C1_101             79
#define MIX_SOURCE_C2_101             80
#define MIX_SOURCE_MAX_101            81
#define MIX_SOURCE_CYC1_101           82
#define MIX_SOURCE_CYC3_101           84
#define MIX_SOURCE_TRIM_RUD_101       85
#define MIX_SOURCE_L64_101            158
#define MIX_SOURCE_TR1_101            159
#define MIX_SOURCE_TR16_101           174
#define MIX_SOURCE_CH1_101            175
#define MIX_SOURCE_CH32_101           206
#define MIX_SOURCE_CH1_110            157

#define LOG_SOURCE_C1_101             79
#define LOG_SOURCE_C2_101             80
#define LOG_SOURCE_MAX_101            81
#define LOG_SOURCE_CYC1_101           82
#define LOG_SOURCE_CYC3_101           84
#define LOG_SOURCE_TRIM_RUD_101       85
#define LOG_SOURCE_L64_101            158
#define LOG_SOURCE_TR1_101            159
#define LOG_SOURCE_TR16_101           174
#define LOG_SOURCE_CH1_101            175
#define LOG_SOURCE_CH32_101           206
#define LOG_SOURCE_CH1_110            157
#define LOG_SOURCE_G1_101             207
#define LOG_SOURCE_G9_101             215
#define LOG_SOURCE_G1_110             189
#define LOG_SOURCE_BATT_101           216
#define LOG_SOURCE_BATT_110           198
#define LOG_SOURCE_TIME_101           217
#define LOG_SOURCE_TIME_110           199
#define LOG_SOURCE_TMR1_101           223
#define LOG_SOURCE_TMR3_101           225
#define LOG_SOURCE_TMR1_110           205
#define LOG_SOURCE_TELEM_FIRST_101    226
#define LOG_SOURCE_TELEM_LAST_101     405
#define LOG_SOURCE_TELEM_FIRST_110    208

#define SPE_FUNC_TRAINER_101          1
#define SPE_FUNC_SET_FAILSFE_101      7


int convertLogicalSource_101_to_110(int source)
{
  if (source == LOG_SOURCE_C1_101 || source == LOG_SOURCE_C2_101)
  {
    source = MIXSRC_NONE;
  }
  else if (source == LOG_SOURCE_MAX_101) {
    source -= 2;
  }
  else if (source >= LOG_SOURCE_CYC1_101 && source <= LOG_SOURCE_CYC3_101)
  {
    source = MIXSRC_NONE;
  }
  else if (source >= LOG_SOURCE_TRIM_RUD_101 && source <= LOG_SOURCE_L64_101) {
    source -= 2;
  }
  else if (source >= LOG_SOURCE_TR1_101 && source <= LOG_SOURCE_TR16_101)
  {
    source = MIXSRC_NONE;
  }
  else if (source >= LOG_SOURCE_CH1_101 && source <= LOG_SOURCE_CH32_101)
  {
    source = LOG_SOURCE_CH1_110 + (source - LOG_SOURCE_CH1_101);
  }
  else if (source >= LOG_SOURCE_G1_101 && source <= LOG_SOURCE_G9_101)
  {
    source = LOG_SOURCE_G1_110 + (source - LOG_SOURCE_G1_101);
  }
  else if (source == LOG_SOURCE_BATT_101)
  {
    source = LOG_SOURCE_BATT_110;
  }
  else if (source == LOG_SOURCE_TIME_101)
  {
    source = LOG_SOURCE_TIME_110;
  }
  else if (source >= LOG_SOURCE_TMR1_101 && source <= LOG_SOURCE_TMR3_101)
  {
    source = LOG_SOURCE_TMR1_110 + (source - LOG_SOURCE_TMR1_101);
  }
  else if (source >= LOG_SOURCE_TELEM_FIRST_101 && source <= LOG_SOURCE_TELEM_LAST_101)
  {
    source = LOG_SOURCE_TELEM_FIRST_110 + (source - LOG_SOURCE_TELEM_FIRST_101);
  }

  return source;
}

int convertMixSource_101_to_110(int source, int *new_source)
{
  if (source == MIX_SOURCE_C1_101 || source == MIX_SOURCE_C2_101)
  {
    source = MIXSRC_NONE;
    return -1;
  }
  else if (source == MIX_SOURCE_MAX_101)
  {
    source -= 2;
  }
  else if (source >= MIX_SOURCE_CYC1_101 && source <= MIX_SOURCE_CYC3_101)
  {
    source = MIXSRC_NONE;
    return -1;
  }
  else if (source >= MIX_SOURCE_TRIM_RUD_101 && source <= MIX_SOURCE_L64_101)
  {
    source -= 2;
  }
  else if (source >= MIX_SOURCE_TR1_101 && source <= MIX_SOURCE_TR16_101)
  {
    source = MIXSRC_NONE;
    return -1;
  }
  else if (source >= MIX_SOURCE_CH1_101 && source <= MIX_SOURCE_CH32_101)
  {
    source = MIX_SOURCE_CH1_110 + (source - MIX_SOURCE_CH1_101);
  }

  *new_source = source;
  return 0;
}

int convertInputSource_101_to_110(int source)
{
  if (source >= INPUT_SOURCE_C1_101 && source <= INPUT_SOURCE_C2_101)
  {
    source = MIXSRC_NONE;
  }
  if (source == INPUT_SOURCE_MAX_101)
  {
    source -= 2;
  }
  else if (source >= INPUT_SOURCE_CYC1_101 && source <= INPUT_SOURCE_CYC3_101)
  {
    source = MIXSRC_NONE;
  }
  else if (source >= INPUT_SOURCE_TRIM_RUD_101 && source <= INPUT_SOURCE_L64_101)
  {
    source -= 2;
  }
  else if (source >= INPUT_SOURCE_TR1_101 && source <= INPUT_SOURCE_TR16_101)
  {
    source = MIXSRC_NONE;
  }
  else if (source >= INPUT_SOURCE_CH1_101 && source <= INPUT_SOURCE_CH32_101)
  {
    source = INPUT_SOURCE_CH1_110 + (source - INPUT_SOURCE_CH1_101);
  }
  else if (source >= INPUT_SOURCE_TELEM_FIRST_101 && source <= INPUT_SOURCE_TELEM_LAST_101)
  {
    source = INPUT_SOURCE_TELEM_FIRST_110 + (source - INPUT_SOURCE_TELEM_FIRST_101);
  }

  return source;
}

void convertModelData_101_to_110(ModelData &model)
{
  int new_source;
  int ret = 0;
  static_assert(sizeof(Conversion_101::ModelData_v101) <= sizeof(ModelData), "ModelData size has been reduced");

  Conversion_101::ModelData_v101 oldModel;
  memcpy(&oldModel, &model, sizeof(oldModel));
  ModelData & newModel = (ModelData &)model;
  memset(&newModel, 0, sizeof(ModelData));
  memcpy(&newModel, &oldModel, offsetof(Conversion_101::ModelData_v101, scriptsData) - offsetof(Conversion_101::ModelData_v101, header));
  memcpy(&newModel.inputNames, &oldModel.inputNames, offsetof(Conversion_101::ModelData_v101, screensType) - offsetof(Conversion_101::ModelData_v101, inputNames));
  memcpy(&newModel.modelRegistrationID, &oldModel.modelRegistrationID, PXX2_LEN_REGISTRATION_ID);

  //customer script data
  for (int i = 0; i < MAX_SCRIPTS; i++)
  {
    memcpy(newModel.scriptsData[i].file, oldModel.scriptsData[i].file, LEN_SCRIPT_FILENAME);
    memcpy(newModel.scriptsData[i].name, oldModel.scriptsData[i].name, LEN_SCRIPT_NAME);

    for (int input = 0; input < MAX_SCRIPT_INPUTS; input++)
    {
      newModel.scriptsData[i].inputs[input].source = convertLogicalSource_101_to_110(oldModel.scriptsData[i].inputs[input].source);
    }
  }
  //customer screens data
  newModel.screensType = oldModel.screensType;
  newModel.view = oldModel.view;
  for (int i = 0; i < MAX_TELEMETRY_SCREENS; i++)
  {
    uint8_t screenType = (oldModel.screensType >> (2*i)) & 0x03;

    if (screenType == TELEMETRY_SCREEN_TYPE_BARS)
    {
      //bars
      for (int j = 0; j < 4; j++)
      {
        newModel.screens[i].bars[j].source = convertLogicalSource_101_to_110(oldModel.screens[i].bars[j].source);
        newModel.screens[i].bars[j].barMin = oldModel.screens[i].bars[j].barMin;
        newModel.screens[i].bars[j].barMax = oldModel.screens[i].bars[j].barMax;
      }
    }
    else if (screenType == TELEMETRY_SCREEN_TYPE_VALUES)
    {
      //numbers
      for (int line = 0; line < 4; line++)
      {
        for (int k = 0; k < NUM_LINE_ITEMS; k++)
        {
          newModel.screens[i].lines[line].sources[k] = convertLogicalSource_101_to_110(oldModel.screens[i].lines[line].sources[k]);
        }
      }
    }
    else if (screenType == TELEMETRY_SCREEN_TYPE_SCRIPT)
    {
      memcpy(newModel.screens[i].script.file, oldModel.screens[i].script.file, LEN_SCRIPT_FILENAME);
      memcpy(newModel.screens[i].script.inputs, oldModel.screens[i].script.inputs, sizeof(oldModel.screens[i].script.inputs));
    }
  };


  for (int i = 0; i < MAX_EXPOS; i++)
  {
    newModel.expoData[i].srcRaw = convertInputSource_101_to_110(oldModel.expoData[i].srcRaw);
  }

  for (int i = 0; i < MAX_MIXERS; i++)
  {
    ret = convertMixSource_101_to_110(newModel.mixData[i].srcRaw, &new_source);

    if (ret == -1)
    {
      MixData * mix = &newModel.mixData[i];;
      memmove(mix, mix + 1, (MAX_MIXERS - (i + 1)) * sizeof(MixData));
      memclear(&newModel.mixData[MAX_MIXERS - 1], sizeof(MixData));
      storageDirty(EE_MODEL);
      i--;
    }
    else
    {
      newModel.mixData[i].srcRaw = new_source;
    }
  }

  for (int i = 0; i < MAX_LOGICAL_SWITCHES; i++)
  {
    newModel.logicalSw[i].v1 = convertLogicalSource_101_to_110(oldModel.logicalSw[i].v1);
  }

  for (int i = 0; i < MAX_SPECIAL_FUNCTIONS; i++)
  {
    if (oldModel.customFn[i].func == SPE_FUNC_TRAINER_101 || oldModel.customFn[i].func == SPE_FUNC_SET_FAILSFE_101)
    {
      //disable useless function
      newModel.customFn[i].swtch = 0;
    }
  }
}

void convertRadioData_101_to_110(RadioData &radio)
{
  for (int i = 0; i < MAX_SPECIAL_FUNCTIONS; i++)
  {
    if (radio.customFn[i].func == SPE_FUNC_TRAINER_101 || radio.customFn[i].func == SPE_FUNC_SET_FAILSFE_101)
    {
      //disable useless function
      radio.customFn[i].swtch = 0;
    }
  }
}