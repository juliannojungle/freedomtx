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

#ifndef _IO_CROSSFIRE_H_
#define _IO_CROSSFIRE_H_

#include <stdint.h>
#include "dataconstants.h"
#include "definitions.h"
#include "fifo.h"
#include "ff.h"
#include "sdcard.h"
#include "crsf.h"


typedef enum {
  DEVICE_INTERNAL = 0,
  USB_HID,
  CRSF_SHARED_FIFO,
  CRSF_ESP,
  LAST_CRSF_PORT
} _CRSF_PORT_NAME;

typedef enum {
  DIO_IRQ_TRAMPOLINE = 0,
  NOTIFICATION_TIMER_IRQ_TRAMPOLINE,
  DEBUG_UART_IRQ_TRAMPOLINE,
  RTOS_WAIT_FLAG_TRAMPOILINE,
  RTOS_CLEAR_FLAG_TRAMPOILINE,
  TRAMPOLINE_INDEX_COUNT
} TRAMPOLINE_INDEX;

typedef enum {
  XF_TASK_FLAG = 0,
  CRSF_SD_TASK_FLAG,
  BOOTLOADER_ICON_WAIT_FLAG,
  TASK_FLAG_COUNT,
  TASK_FLAG_MAX = 10
} TASK_FLAG_INDEX;

typedef enum {
  CRSF_FLAG_SHOW_BOOTLOADER_ICON = 0,
  CRSF_FLAG_EEPROM_SAVE,
  CRSF_FLAG_BOOTUP,
  CRSF_FLAG_POWER_OFF,
  CRSF_FLAG_COUNT,
  CRSF_FLAG_MAX = 32,
} CRSF_FLAG_INDEX;

#define get_task_flag(i)    crossfireSharedData.taskFlag[i]
#define get_crsf_flag(i)    (crossfireSharedData.crsfFlag & ( 1UL << i ) ? true : false )
#define set_crsf_flag(i)    (crossfireSharedData.crsfFlag |= ( 1UL << i ))
#define clear_crsf_flag(i)  (crossfireSharedData.crsfFlag &= ~( 1UL << i ))

#define RTOS_API_VERSION                0x102
#define TELEMETRY_BUFFER_SIZE           128
#define CROSSFIRE_FIFO_SIZE             256
#define CROSSFIRE_CHANNELS_COUNT        16

struct CrossfireSharedData {
  uint32_t rtosApiVersion;
  uint32_t *trampoline;
  uint8_t taskFlag[TASK_FLAG_MAX];
  uint32_t crsfFlag;
  uint32_t reserverd1[16];
  Fifo<uint8_t, CROSSFIRE_FIFO_SIZE> crsf_tx;   //from XF to OpenTX
  Fifo<uint8_t, CROSSFIRE_FIFO_SIZE> crsf_rx;   //from OpenTX to XF
  uint32_t reserverd2[64];
  int16_t sticks[NUM_STICKS];
  uint8_t stick_state:5;
  uint8_t gim_select:2;
  uint8_t mixer_schedule:1;
  int32_t channels[CROSSFIRE_CHANNELS_COUNT];
};

#ifdef LIBCRSF_ENABLE_OPENTX_RELATED
//#define CRSF_ENABLE_SD_DEBUG
#ifdef CRSF_ENABLE_SD_DEBUG
#define CRSF_SD_PRINTF  TRACE_NOCRLF
#else
#define CRSF_SD_PRINTF( ... )
#endif

extern bool set_model_id_needed;
extern uint8_t current_crsf_model_id;
void crsfRemoteRelatedHandler( uint8_t *p_arr );
#endif

typedef struct CrossfireSharedData CrossfireSharedData_t;
#define crossfireSharedData (*((CrossfireSharedData_t*)SHARED_MEMORY_ADDRESS))

#define isMixerTaskScheduled()     (crossfireSharedData.mixer_schedule)
#define clearMixerTaskSchedule()  {crossfireSharedData.mixer_schedule = 0;}
void CRSF_Init( void );
void crsfSharedFifoHandler( void );
void crsfSetModelID(void);
void crsfGetModelID(void);
void crsfEspHandler( void );
void CRSF_to_Shared_FIFO( uint8_t *p_arr );
void CRSF_to_ESP( uint8_t *p_arr );
void CRSF_This_Device( uint8_t *p_arr );
void AgentLegacyCalls( uint8_t *arr );

#endif // _CROSSFIRE_H_
