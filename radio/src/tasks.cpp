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

RTOS_TASK_HANDLE menusTaskId;
RTOS_DEFINE_STACK(menusStack, MENUS_STACK_SIZE);

RTOS_TASK_HANDLE mixerTaskId;
RTOS_DEFINE_STACK(mixerStack, MIXER_STACK_SIZE);

RTOS_TASK_HANDLE audioTaskId;
RTOS_DEFINE_STACK(audioStack, AUDIO_STACK_SIZE);

#if defined(CROSSFIRE_TASK)
RTOS_TASK_HANDLE crossfireTaskId;
RTOS_DEFINE_STACK(crossfireStack, CROSSFIRE_STACK_SIZE);

RTOS_TASK_HANDLE systemTaskId;
RTOS_DEFINE_STACK(systemStack, SYSTEM_STACK_SIZE);
#endif

RTOS_MUTEX_HANDLE audioMutex;
RTOS_MUTEX_HANDLE mixerMutex;

enum TaskIndex {
  MENU_TASK_INDEX,
  MIXER_TASK_INDEX,
  AUDIO_TASK_INDEX,
  CLI_TASK_INDEX,
  BLUETOOTH_TASK_INDEX,
  TASK_INDEX_COUNT,
  MAIN_TASK_INDEX = 255
};

void stackPaint()
{
  menusStack.paint();
  mixerStack.paint();
  audioStack.paint();
#if defined(CLI)
  cliStack.paint();
#endif
}

volatile uint16_t timeForcePowerOffPressed = 0;

bool isForcePowerOffRequested()
{
  if (pwrOffPressed()) {
    if (timeForcePowerOffPressed == 0) {
      timeForcePowerOffPressed = get_tmr10ms();
    }
    else {
      uint16_t delay = (uint16_t)get_tmr10ms() - timeForcePowerOffPressed;
      if (delay > 1000/*10s*/) {
        return true;
      }
    }
  }
  else {
    resetForcePowerOffRequest();
  }
  return false;
}

bool isModuleSynchronous(uint8_t module)
{
  uint8_t protocol = moduleState[module].protocol;
  if (protocol == PROTOCOL_CHANNELS_PXX2_HIGHSPEED || protocol == PROTOCOL_CHANNELS_PXX2_LOWSPEED || protocol == PROTOCOL_CHANNELS_CROSSFIRE || protocol == PROTOCOL_CHANNELS_NONE)
    return true;
#if defined(INTMODULE_USART) || defined(EXTMODULE_USART)
  if (protocol == PROTOCOL_CHANNELS_PXX1_SERIAL)
    return true;
#elif defined(PCBTANGO)
    return true;
#endif
  return false;
}

void sendSynchronousPulses()
{
#if defined(HARDWARE_INTERNAL_MODULE)
  if (isModuleSynchronous(INTERNAL_MODULE)) {
    if (setupPulsesInternalModule())
      intmoduleSendNextFrame();
  }
#endif

  if (isModuleSynchronous(EXTERNAL_MODULE)) {
    if (setupPulsesExternalModule())
      extmoduleSendNextFrame();
  }
}

uint32_t nextMixerTime[NUM_MODULES];

TASK_FUNCTION(mixerTask)
{
  static uint32_t lastRunTime;
  s_pulses_paused = true;

  while (true) {
#if defined(PCBTARANIS) && defined(SBUS)
    // SBUS trainer
    processSbusInput();
#endif

#if defined(GYRO)
    gyro.wakeup();
#endif

#if defined(BLUETOOTH)
    bluetooth.wakeup();
#endif

    RTOS_WAIT_TICKS(1);

#if defined(SIMU)
    if (pwrCheck() == e_power_off) {
      TASK_RETURN();
    }
#else
    if (isForcePowerOffRequested()) {
      boardOff();
    }
#endif

    uint32_t now = RTOS_GET_MS();
    bool run = false;

#if defined(CROSSFIRE_TASK) && !defined(SIMU)
    if (isMixerTaskScheduled()) {
      clearMixerTaskSchedule();
      run = true;
    }
#endif

    if (now - lastRunTime >= 10) {
      // run at least every 10ms
      run = true;
    }

#if defined(INTMODULE_USART) && defined(INTMODULE_HEARTBEAT)
    if ((moduleState[INTERNAL_MODULE].protocol == PROTOCOL_CHANNELS_PXX2_HIGHSPEED || moduleState[INTERNAL_MODULE].protocol == PROTOCOL_CHANNELS_PXX1_SERIAL) && heartbeatCapture.valid && heartbeatCapture.timestamp > lastRunTime) {
      run = true;
    }
#endif

    if (now == nextMixerTime[0]) {
      run = true;
    }

#if NUM_MODULES >= 2
    if (now == nextMixerTime[1]) {
      run = true;
    }
#endif

    if (!run) {
      continue;  // go back to sleep
    }

    lastRunTime = now;

    if (!s_pulses_paused) {
      uint16_t t0 = getTmr2MHz();

      DEBUG_TIMER_START(debugTimerMixer);
      RTOS_LOCK_MUTEX(mixerMutex);
      doMixerCalculations();
      DEBUG_TIMER_START(debugTimerMixerCalcToUsage);
      DEBUG_TIMER_SAMPLE(debugTimerMixerIterval);
      RTOS_UNLOCK_MUTEX(mixerMutex);
      DEBUG_TIMER_STOP(debugTimerMixer);

#if defined(STM32) && !defined(SIMU)
      if (getSelectedUsbMode() == USB_JOYSTICK_MODE) {
        usbJoystickUpdate();
      }
#if defined(CROSSFIRE_TASK)
      UpdateCrossfireChannels();
#endif
#endif

#if defined(PCBSKY9X) && !defined(SIMU)
      usbJoystickUpdate();
#endif

      DEBUG_TIMER_START(debugTimerTelemetryWakeup);
      telemetryWakeup();
      DEBUG_TIMER_STOP(debugTimerTelemetryWakeup);

      if (heartbeat == HEART_WDT_CHECK) {
        wdt_reset();
        heartbeat = 0;
      }

      t0 = getTmr2MHz() - t0;
      if (t0 > maxMixerDuration)
        maxMixerDuration = t0;

      sendSynchronousPulses();
    }
  }
}

void scheduleNextMixerCalculation(uint8_t module, uint16_t period_ms)
{
  // Schedule next mixer calculation time,

  if (isModuleSynchronous(module)) {
    nextMixerTime[module] += period_ms / RTOS_MS_PER_TICK;
    if (nextMixerTime[module] < RTOS_GET_TIME()) {
      // we are late ... let's add some small delay
      nextMixerTime[module] = (uint32_t) RTOS_GET_TIME() + (period_ms / RTOS_MS_PER_TICK);
    }
  }
  else {
    // for now assume mixer calculation takes 2 ms.
    nextMixerTime[module] = (uint32_t) RTOS_GET_TIME() + (period_ms / RTOS_MS_PER_TICK);
  }

  DEBUG_TIMER_STOP(debugTimerMixerCalcToUsage);
}

#define MENU_TASK_PERIOD_TICKS         (50 / RTOS_MS_PER_TICK)    // 50ms

#if defined(COLORLCD) && defined(CLI)
bool perMainEnabled = true;
#endif

TASK_FUNCTION(menusTask)
{
  opentxInit();

#if defined(PWR_BUTTON_PRESS)
  while (true) {
    uint32_t pwr_check = pwrCheck();
    if (pwr_check == e_power_off) {
      break;
    }
    else if (pwr_check == e_power_press) {
      RTOS_WAIT_TICKS(MENU_TASK_PERIOD_TICKS);
      continue;
    }
#else
  while (pwrCheck() != e_power_off) {
#endif
    uint32_t start = (uint32_t)RTOS_GET_TIME();
    DEBUG_TIMER_START(debugTimerPerMain);
#if defined(COLORLCD) && defined(CLI)
    if (perMainEnabled) {
      perMain();
    }
#else
    perMain();
#endif
    DEBUG_TIMER_STOP(debugTimerPerMain);
    // TODO remove completely massstorage from sky9x firmware
    uint32_t runtime = ((uint32_t)RTOS_GET_TIME() - start);
    // deduct the thread run-time from the wait, if run-time was more than
    // desired period, then skip the wait all together
    if (runtime < MENU_TASK_PERIOD_TICKS) {
      RTOS_WAIT_TICKS(MENU_TASK_PERIOD_TICKS - runtime);
    }

    resetForcePowerOffRequest();
  }

#if defined(CROSSFIRE_TASK) && defined(LIBCRSF_ENABLE_OPENTX_RELATED) && defined(LIBCRSF_ENABLE_SD)
  if((*(uint32_t *)CROSSFIRE_TASK_ADDRESS != 0xFFFFFFFF) &&
    getSelectedUsbMode() != USB_MASS_STORAGE_MODE && sdMounted()){
    set_crsf_flag( CRSF_FLAG_EEPROM_SAVE );
    uint32_t time = get_tmr10ms();
    while( get_crsf_flag( CRSF_FLAG_EEPROM_SAVE ) && get_tmr10ms() - time <= 100 ){
      // with 1s timeout
      RTOS_WAIT_TICKS(1);
    }
  }
#endif

#if defined(PCBX9E)
  toplcdOff();
#endif

#if defined(PCBHORUS)
  ledOff();
#endif

  drawSleepBitmap();
  opentxClose();
  boardOff(); // Only turn power off if necessary
  
  TASK_RETURN();
}

#if defined(CROSSFIRE_TASK) && !defined(SIMU)
TASK_FUNCTION(systemTask)
{
  static uint32_t get_modelid_delay = 0;
  set_model_id_needed = true;

  while(1) {
    if( get_crsf_flag( CRSF_FLAG_SHOW_BOOTLOADER_ICON )){
      static uint32_t delayCount = 0;
      if(delayCount == 0){
        delayCount = RTOS_GET_TIME();
        RTOS_DEL_TASK(menusTaskId);
        lcdOn();
        drawDownload();
      }
      if(RTOS_GET_TIME() - delayCount > 100){
        NVIC_SystemReset();
      }
    }

    crsfSharedFifoHandler();
    crsfEspHandler();
#if defined(AGENT) && !defined(SIMU)
    AgentHandler();
#endif
    if (set_model_id_needed && g_model.header.modelId[EXTERNAL_MODULE] != 0 && get_tmr10ms() - get_modelid_delay > 100 ) {
      crsfSetModelID();
      crsfGetModelID();
      if(current_crsf_model_id == g_model.header.modelId[EXTERNAL_MODULE])
        set_model_id_needed = false;
      get_modelid_delay = get_tmr10ms();
    }
  }
  TASK_RETURN();
}

void crossfireTasksCreate()
{
  RTOS_CREATE_TASK(crossfireTaskId, (FUNCPtr)CROSSFIRE_TASK_ADDRESS, "crossfire", crossfireStack, CROSSFIRE_STACK_SIZE, CROSSFIRE_TASK_PRIORITY);
  RTOS_CREATE_TASK(systemTaskId, systemTask, "system", systemStack, SYSTEM_STACK_SIZE, RTOS_SYS_TASK_PRIORITY);
}

void crossfireTasksStart()
{
  uint8_t taskFlag[TASK_FLAG_MAX] = {0};
  // Test if crossfire task is available and start it
  if (*(uint32_t *)CROSSFIRE_TASK_ADDRESS != 0xFFFFFFFF ) {
    crossfireTasksCreate();
    RTOS_CREATE_FLAG( taskFlag[XF_TASK_FLAG] );
    RTOS_CREATE_FLAG( taskFlag[CRSF_SD_TASK_FLAG] );
    RTOS_CREATE_FLAG( taskFlag[BOOTLOADER_ICON_WAIT_FLAG] );

    for( uint8_t i = 0; i < TASK_FLAG_MAX; i++ ){
      crossfireSharedData.taskFlag[i] = taskFlag[i];
    }
  }
}

void crossfireTasksStop()
{
  NVIC_DisableIRQ(INTERRUPT_EXTI_IRQn);
  NVIC_DisableIRQ(INTERRUPT_NOT_TIMER_IRQn);
  RTOS_DEL_TASK(crossfireTaskId);
  RTOS_DEL_TASK(systemTaskId);
}
#endif

void tasksStart()
{
  RTOS_INIT();

#if defined(CLI)
  cliStart();
#endif

  RTOS_CREATE_TASK(mixerTaskId, mixerTask, "mixer", mixerStack, MIXER_STACK_SIZE, MIXER_TASK_PRIO);
  RTOS_CREATE_TASK(menusTaskId, menusTask, "menus", menusStack, MENUS_STACK_SIZE, MENUS_TASK_PRIO);

#if defined(CROSSFIRE_TASK) && !defined(SIMU)
  crossfireTasksStart();
#endif

#if !defined(SIMU)
  RTOS_CREATE_TASK(audioTaskId, audioTask, "audio", audioStack, AUDIO_STACK_SIZE, AUDIO_TASK_PRIO);
#endif

  RTOS_CREATE_MUTEX(audioMutex);
  RTOS_CREATE_MUTEX(mixerMutex);

  RTOS_START();
}
