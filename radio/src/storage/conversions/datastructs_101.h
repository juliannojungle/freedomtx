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

// No include guards here, this file may be included many times in different namespaces
// i.e. BACKUP RAM Backup/Restore functions

#ifndef OPENTX_DATASTRUCTS_101_H
#define OPENTX_DATASTRUCTS_101_H

namespace Conversion_101 {
#include "datastructs.h"

#define NOBACKUP(...)                __VA_ARGS__
#define SWITCHES_WARNING_DATA \
    swarnstate_t  switchWarningState; \
    swarnenable_t switchWarningEnable;

#define CUSTOM_SCREENS_DATA \
  uint8_t screensType; /* 2bits per screen (None/Gauges/Numbers/Script) */ \
  TelemetryScreenData screens[MAX_TELEMETRY_SCREENS]; \
  uint8_t view;

typedef uint8_t source_t_v101;

#if MAX_SCRIPTS > 0
  union ScriptDataInput_v101 {
    int16_t value;
    source_t_v101 source;
  };

  PACK(struct ScriptData_v101 {
      char            file[LEN_SCRIPT_FILENAME];
      char            name[LEN_SCRIPT_NAME];
      ScriptDataInput_v101 inputs[MAX_SCRIPT_INPUTS];
  });
#endif

#define SCRIPT_DATA_v101 \
    NOBACKUP(ScriptData_v101 scriptsData[MAX_SCRIPTS]);

PACK(struct FrSkyBarData_v101 {
   source_t_v101 source;
   ls_telemetry_value_t barMin;           // minimum for bar display
   ls_telemetry_value_t barMax;           // ditto for max display (would usually = ratio)
 });

PACK(struct FrSkyLineData_v101 {
       source_t_v101 sources[NUM_LINE_ITEMS];
     });

union TelemetryScreenData_v101 {
  FrSkyBarData_v101  bars[4];
  FrSkyLineData_v101 lines[4];
  TelemetryScriptData script;
};

#define CUSTOM_SCREENS_DATA_v101 \
  uint8_t screensType;      \
  TelemetryScreenData_v101 screens[MAX_TELEMETRY_SCREENS]; \
  uint8_t view;

PACK(struct ModelData_v101 {
       ModelHeader header;
       TimerData timers[MAX_TIMERS];
       uint8_t   telemetryProtocol:3;
       uint8_t   thrTrim:1;            // Enable Throttle Trim
       uint8_t   noGlobalFunctions:1;
       uint8_t   displayTrims:2;
       uint8_t   ignoreSensorIds:1;
       int8_t    trimInc:3;            // Trim Increments
       uint8_t   disableThrottleWarning:1;
       uint8_t   displayChecklist:1;
       uint8_t   extendedLimits:1;
       uint8_t   extendedTrims:1;
       uint8_t   throttleReversed:1;
       BeepANACenter beepANACenter;
       MixData   mixData[MAX_MIXERS];
       LimitData limitData[MAX_OUTPUT_CHANNELS];
       ExpoData  expoData[MAX_EXPOS];

       CurveData curves[MAX_CURVES];
       int8_t    points[MAX_CURVE_POINTS];

       LogicalSwitchData logicalSw[MAX_LOGICAL_SWITCHES];
       CustomFunctionData customFn[MAX_SPECIAL_FUNCTIONS];
       SwashRingData swashR;
       FlightModeData flightModeData[MAX_FLIGHT_MODES];

       NOBACKUP(uint8_t thrTraceSrc);

       SWITCHES_WARNING_DATA

       GVarData gvars[MAX_GVARS];

       NOBACKUP(VarioData varioData);
       NOBACKUP(uint8_t rssiSource);

       TOPBAR_DATA

       NOBACKUP(RssiAlarmData rssiAlarms);

       NOBACKUP(uint8_t spare1:6);
       NOBACKUP(uint8_t potsWarnMode:2);

       ModuleData moduleData[NUM_MODULES];
       int16_t failsafeChannels[MAX_OUTPUT_CHANNELS];
       TrainerModuleData trainerData;

       SCRIPT_DATA_v101

       NOBACKUP(char inputNames[MAX_INPUTS][LEN_INPUT_NAME]);
       NOBACKUP(uint8_t potsWarnEnabled);
       NOBACKUP(int8_t potsWarnPosition[STORAGE_NUM_POTS+STORAGE_NUM_SLIDERS]);

       NOBACKUP(TelemetrySensor telemetrySensors[MAX_TELEMETRY_SENSORS];)

       TARANIS_PCBX9E_FIELD(uint8_t toplcdTimer)

       CUSTOM_SCREENS_DATA_v101

       char modelRegistrationID[PXX2_LEN_REGISTRATION_ID];
     });
};
#endif

