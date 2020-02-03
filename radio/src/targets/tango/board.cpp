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
#include "rtos.h"
#include "stm32f4xx_flash.h"

bool set_model_id_needed = false;

HardwareOptions hardwareOptions;

static uint32_t trampoline[TRAMPOLINE_INDEX_COUNT] = {0};

#if defined(__cplusplus) && !defined(SIMU)
extern "C" {
#endif
#include "usb_dcd_int.h"
#include "usb_bsp.h"
#if defined(__cplusplus) && !defined(SIMU)
}
#endif

#if !defined(SIMU)
static uint8_t boardOffState = 0;
uint8_t getBoardOffState();
static uint8_t isDisableBoardOff();
static uint8_t checkDefaultWord();
uint8_t getFlag(uint32_t flag);
void setFlag(uint32_t flag);
void clearFlag(uint32_t flag);
#endif

void watchdogInit(unsigned int duration)
{
  IWDG->KR = 0x5555;      // Unlock registers
  IWDG->PR = 3;           // Divide by 32 => 1kHz clock
  IWDG->KR = 0x5555;      // Unlock registers
  IWDG->RLR = duration;       // 1.5 seconds nominal
  IWDG->KR = 0xAAAA;      // reload
  IWDG->KR = 0xCCCC;      // start
}

// Starts TIMER at 2MHz
void init2MhzTimer()
{
  TIMER_2MHz_TIMER->PSC = (PERI1_FREQUENCY * TIMER_MULT_APB1) / 2000000 - 1 ;       // 0.5 uS, 2 MHz
  TIMER_2MHz_TIMER->ARR = 65535;
  TIMER_2MHz_TIMER->CR2 = 0;
  TIMER_2MHz_TIMER->CR1 = TIM_CR1_CEN;
}

// Starts TIMER at 200Hz (5ms)
void init5msTimer()
{
  INTERRUPT_xMS_TIMER->ARR = 4999 ; // 5mS in uS
  INTERRUPT_xMS_TIMER->PSC = (PERI1_FREQUENCY * TIMER_MULT_APB1) / 1000000 - 1 ; // 1uS
  INTERRUPT_xMS_TIMER->CCER = 0 ;
  INTERRUPT_xMS_TIMER->CCMR1 = 0 ;
  INTERRUPT_xMS_TIMER->EGR = 0 ;
  INTERRUPT_xMS_TIMER->CR1 = 5 ;
  INTERRUPT_xMS_TIMER->DIER |= 1 ;
  NVIC_EnableIRQ(INTERRUPT_xMS_IRQn) ;
  NVIC_SetPriority(INTERRUPT_xMS_IRQn, 7);
}

void stop5msTimer( void )
{
  INTERRUPT_xMS_TIMER->CR1 = 0 ;        // stop timer
  NVIC_DisableIRQ(INTERRUPT_xMS_IRQn) ;
}

// TODO use the same than board_sky9x.cpp
void interrupt5ms()
{
  static uint32_t pre_scale ;       // Used to get 10 Hz counter

  AUDIO_HEARTBEAT();

#if defined(HAPTIC)
  HAPTIC_HEARTBEAT();
#endif
  if (++pre_scale >= 2) {
    pre_scale = 0 ;
    DEBUG_TIMER_START(debugTimerPer10ms);
    DEBUG_TIMER_SAMPLE(debugTimerPer10msPeriod);
    per10ms();
    DEBUG_TIMER_STOP(debugTimerPer10ms);
  }
}

// Starts TIMER at 1KHz (1ms)
void init1msTimer()
{
  INTERRUPT_1MS_TIMER->ARR = 999 ; // 1mS in uS
  INTERRUPT_1MS_TIMER->PSC = (PERI2_FREQUENCY * TIMER_MULT_APB2) / 1000000 - 1 ; // 1uS
  INTERRUPT_1MS_TIMER->CCER = 0 ;
  INTERRUPT_1MS_TIMER->CCMR1 = 0 ;
  INTERRUPT_1MS_TIMER->EGR = 0 ;
  INTERRUPT_1MS_TIMER->CR1 = 5 ;
  INTERRUPT_1MS_TIMER->DIER |= 1 ;
  NVIC_SetPriority(INTERRUPT_1MS_IRQn, INTERRUPT_1MS_IRQ_PRI);
  NVIC_EnableIRQ(INTERRUPT_1MS_IRQn) ;
}

#if !defined(SIMU)
extern "C" void INTERRUPT_xMS_IRQHandler()
{
  INTERRUPT_xMS_TIMER->SR &= ~TIM_SR_UIF ;
  interrupt5ms() ;
  DEBUG_INTERRUPT(INT_5MS);
}

extern "C" void INTERRUPT_1MS_IRQHandler()
{
  INTERRUPT_1MS_TIMER->SR &= ~TIM_SR_UIF ;
}

#if defined(PWR_BUTTON_PRESS)
  #define PWR_PRESS_DURATION_MIN        100 // 1s
  #define PWR_PRESS_DURATION_MAX        500 // 5s
#endif

const unsigned char bmp_startup[]  = {
  #include "startup.lbm"
};

const unsigned char bmp_lock[]  = {
  #include "lock.lbm"
};

static void chargerInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = CHARGER_STATE_GPIO_PIN | CHARGER_FAULT_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(CHARGER_STATE_GPIO, &GPIO_InitStructure);
}

static void detectChargingMode(void)
{
  tmr10ms_t tm10ms = g_tmr10ms;
  tmr10ms_t tm100ms = g_tmr10ms;
  uint8_t  wt_cnt = 5;

  while (wt_cnt > 0) {
    if ((g_tmr10ms- tm100ms) > 9) {
      usbPlugged(); //call several times continuously for debouncing and ensure stable connection
      wt_cnt--;
      tm100ms = g_tmr10ms;
    }
  }

  if (usbPlugged() && !pwrPressed()) {
    if (!rambackupRestore()) {
      g_eeGeneral.txVoltageCalibration = BATT_CALIB_OFFSET; //ram backup failed, use the default calibration value for battery voltage
    }
  }

  while (IS_CHARGING_STATE() && !IS_CHARGING_FAULT() && usbPlugged() && !pwrPressed()) {
    if ((g_tmr10ms - tm10ms) > 9) {
      getADC();
      tm10ms = g_tmr10ms;
    }
    if ((g_tmr10ms- tm100ms) > 49) {
      // lcdClear();
      checkBattery();
      // drawChargingState();
      // lcdRefresh();
      LED_CHARGING_IN_PROGRESS();
      tm100ms = g_tmr10ms;
    }
  }

  wt_cnt = 5;
  while (wt_cnt > 0) {
    if ((g_tmr10ms - tm10ms) > 9) {
      getADC();
      tm10ms = g_tmr10ms;
    }
    if ((g_tmr10ms- tm100ms) > 49) {
      checkBattery();
      wt_cnt--;
    }
  }

  while (!IS_CHARGING_STATE() && usbPlugged() && !pwrPressed() && g_vbat100mV >= 40) {
    if ((g_tmr10ms - tm10ms) > 9) {
      getADC();
      tm10ms = g_tmr10ms;
    }
    if ((g_tmr10ms- tm100ms) > 49) {
      // lcdClear();
      checkBattery();
      // drawFullyCharged();
      // lcdRefresh();
      LED_CHARGING_DONE();
      tm100ms = g_tmr10ms;
    }
  }
}
#endif  // !defined(SIMU)

void boardInit()
{
#if !defined(SIMU)
  RCC_AHB1PeriphClockCmd(PWR_RCC_AHB1Periph | 
                         KEYS_RCC_AHB1Periph | LCD_RCC_AHB1Periph |
                         AUDIO_RCC_AHB1Periph |
                         ADC_RCC_AHB1Periph | I2C_RCC_AHB1Periph |
                         SD_RCC_AHB1Periph | HAPTIC_RCC_AHB1Periph |
                         TELEMETRY_RCC_AHB1Periph | LED_RCC_AHB1Periph |
                         SERIAL_RCC_AHB1Periph, ENABLE);

  RCC_APB1PeriphClockCmd(LCD_RCC_APB1Periph | AUDIO_RCC_APB1Periph | ADC_RCC_APB1Periph |
                         HAPTIC_RCC_APB1Periph | INTERRUPT_xMS_RCC_APB1Periph |
                         TIMER_2MHz_RCC_APB1Periph | LED_RCC_APB1Periph |
                         SD_RCC_APB1Periph |
                         TELEMETRY_RCC_APB1Periph | SERIAL_RCC_APB1Periph, ENABLE);

  RCC_APB2PeriphClockCmd( ADC_RCC_APB2Periph |
                         HAPTIC_RCC_APB2Periph |
                         INTERRUPT_1MS_RCC_APB1Periph, ENABLE);

  pwrInit();
  keysInit();
  delaysInit();
  adcInit();
  lcdInit(); // delaysInit() must be called before
  audioInit();
  init2MhzTimer();
  init5msTimer();
  init1msTimer();
  CRSF_Init();
  usbInit();
  #if defined(CHARGING_LEDS)
    ledInit();
  #endif
  chargerInit();
  __enable_irq();

#if defined(DEBUG) && defined(SERIAL_GPIO)
  serial2Init(0, 0); // default serial mode (None if DEBUG not defined)
  TRACE("Tango board started :)\r\n");
#endif

#if !defined(SIMU)
  if(!isDisableBoardOff()){
#endif
    detectChargingMode();
#if !defined(SIMU)
  }
#endif

#if defined(ESP_SERIAL)
  espInit(ESP_UART_BAUDRATE, false);
#endif

#if defined(HAPTIC)
  hapticInit();
#endif

#if defined(ROTARY_ENCODER_NAVIGATION) && defined(ROTARY_ENCODER_EXTI_IRQHandler1)
  rotaryEncoderInit();
#endif

#if defined(DEBUG)
  DBGMCU_APB1PeriphConfig(DBGMCU_IWDG_STOP|DBGMCU_TIM1_STOP|DBGMCU_TIM2_STOP|DBGMCU_TIM3_STOP|DBGMCU_TIM6_STOP|DBGMCU_TIM8_STOP|DBGMCU_TIM10_STOP|DBGMCU_TIM13_STOP|DBGMCU_TIM14_STOP, ENABLE);
#endif

#if defined(PWR_BUTTON_PRESS)
  if (!WAS_RESET_BY_WATCHDOG()) {
    lcdClear();
    lcdDrawFilledRect(LCD_W / 2 - 18, LCD_H / 2 - 3, 6, 6, SOLID, 0);
    lcdRefresh();
    lcdRefreshWait();

    tmr10ms_t start = get_tmr10ms();
    tmr10ms_t duration = 0;
    uint8_t pwr_on = 0;
    while (pwrPressed()) {
      duration = get_tmr10ms() - start;
      if (duration < PWR_PRESS_DURATION_MIN) {
        unsigned index = duration / (PWR_PRESS_DURATION_MIN / 4);
        lcdClear();
        for(uint8_t i= 0; i < 4; i++) {
          if (index >= i) {
            lcdDrawFilledRect(LCD_W / 2 - 18 + 10 * i, LCD_H / 2 - 3, 6, 6, SOLID, 0);
          }
        }
      }
      else if (duration >= PWR_PRESS_DURATION_MAX) {
        drawSleepBitmap();
        backlightDisable();
      }
      else {
        if (pwr_on != 1) {
          pwr_on = 1;
          pwrInit();
          backlightInit();
          haptic.play(15, 3, PLAY_NOW);
          break;
        }
      }
      lcdRefresh();
      lcdRefreshWait();
    }
    if (duration < PWR_PRESS_DURATION_MIN || duration >= PWR_PRESS_DURATION_MAX) {
      if(!getBoardOffState()){
        if (IS_CHARGING_STATE() && !IS_CHARGING_FAULT() && usbPlugged()) {
          NVIC_SystemReset();
        }
        TRACE("power off\n");
        boardOff();
      }
    }
  }
  else {
    backlightInit();
  }
#else // defined(PWR_BUTTON_PRESS)
  backlightInit();
#endif
#endif // !defined(SIMU)

#if defined(LIBCRSF_ENABLE_OPENTX_RELATED) && defined(LIBCRSF_ENABLE_SD)
  sdInit();
#endif
  vbattRTC = getRTCBattVoltage();
}

void boardOff()
{
#if !defined(SIMU)
  crossfireOff();
#endif

#if defined(ESP_SERIAL)
  espOff();
#endif

  BACKLIGHT_DISABLE();

#if defined(CHARGING_LEDS)
  ledOff();
#endif
  lcdOff();
  SysTick->CTRL = 0; // turn off systick
  pwrOff();
}

uint16_t getBatteryVoltage()
{
#if !defined(SIMU)
  // set the flag when opentx finish bootup
  if( !get_crsf_flag( CRSF_FLAG_BOOTUP )){
    set_crsf_flag( CRSF_FLAG_BOOTUP );
  }
#endif
  int32_t instant_vbat = anaIn(TX_VOLTAGE); // using filtered ADC value on purpose
  instant_vbat = instant_vbat / BATT_SCALE + g_eeGeneral.txVoltageCalibration;
  instant_vbat = instant_vbat > BATTERY_MAX * 10 ? BATTERY_MAX * 10 : instant_vbat;
  return (uint16_t)instant_vbat;
}

#if !defined(SIMU)
uint32_t readBackupReg(uint8_t index){
  return *(__IO uint32_t *) (BKPSRAM_BASE + index*4);
}

void writeBackupReg(uint8_t index, uint32_t data){
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  PWR_BackupRegulatorCmd(ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_BKPSRAM, ENABLE);
  PWR_BackupAccessCmd(ENABLE);
  while(PWR_GetFlagStatus(PWR_FLAG_BRR) == RESET);
  *(__IO uint32_t *) (BKPSRAM_BASE + index*4) = data;
  // PWR_BackupAccessCmd(DISABLE);
}

void boot2bootloader(uint32_t isNeedFlash, uint32_t HwId, uint32_t sn){
  usbStop();
#if !defined(SIMU)
  crossfireOff();
  RTOS_SET_FLAG(get_task_flag(XF_TASK_FLAG));
#endif
  writeBackupReg(BOOTLOADER_IS_NEED_FLASH_ADDR, isNeedFlash);
  writeBackupReg(BOOTLOADER_HW_ID_ADDR, HwId);
  writeBackupReg(BOOTLOADER_SERIAL_NO_ADDR, sn);
  setFlag( DEVICE_RESTART_WITHOUT_WARN_FLAG );
  NVIC_SystemReset();
}

uint8_t getBoardOffState(){
  return boardOffState;
}

static uint8_t isDisableBoardOff(){
  uint8_t value = (uint8_t)readBackupReg(BOOTLOADER_IS_SKIP_BOARD_OFF_ADDR);
  writeBackupReg(BOOTLOADER_IS_SKIP_BOARD_OFF_ADDR, 0);
  boardOffState = getFlag(DEVICE_RESTART_WITHOUT_WARN_FLAG);
  clearFlag(DEVICE_RESTART_WITHOUT_WARN_FLAG);
  boardOffState |= value;
  if(!checkDefaultWord()){
    boardOffState = 0;
  }
  return boardOffState;
}

static uint8_t checkDefaultWord(){
  union{
    uint8_t b[4];
    uint32_t word;
  } defaultWord;
  defaultWord.b[0] = (uint8_t)'t';
  defaultWord.b[1] = (uint8_t)'a';
  defaultWord.b[2] = (uint8_t)'n';
  defaultWord.b[3] = (uint8_t)'g';
  uint32_t value = (uint32_t)readBackupReg(BOOTLOADER_DEFAULT_WORD_ADDR);
  if(value != defaultWord.word){
    writeBackupReg(BOOTLOADER_DEFAULT_WORD_ADDR, defaultWord.word);
    return 0;
  }
  return 1;
}

uint8_t getFlag(uint32_t flag){
  uint32_t value = (uint32_t)readBackupReg(BOOTLOADER_FLAG_ADDR);
  return (uint8_t)(value & (1 << flag) >> flag);
}

void setFlag(uint32_t flag){
  uint32_t value = (uint32_t)readBackupReg(BOOTLOADER_FLAG_ADDR);
  value |= (1 << flag);
  writeBackupReg(BOOTLOADER_FLAG_ADDR, value);
}

void clearFlag(uint32_t flag){
  uint32_t value = (uint32_t)readBackupReg(BOOTLOADER_FLAG_ADDR);
  value &= ~(1 << flag);
  writeBackupReg(BOOTLOADER_FLAG_ADDR, value);
}

void crossfireOff( void ){
  set_crsf_flag( CRSF_FLAG_POWER_OFF );
}
#endif

void PrintData(char* header, uint8_t* data){
  TRACE_NOCRLF("\r\n%s: ", header);
  for(int i = 0; i < data[1] + 2; i++){
    TRACE_NOCRLF("%02X ", data[i]);
  }
  TRACE_NOCRLF("\r\n");
}

void trampolineInit( void )
{
  memset( trampoline, 0, sizeof(uint32_t) * TRAMPOLINE_INDEX_COUNT );
#if !defined(SIMU)
  trampoline[RTOS_WAIT_FLAG_TRAMPOILINE] = (uint32_t)(&CoWaitForSingleFlag);
  trampoline[RTOS_CLEAR_FLAG_TRAMPOILINE] = (uint32_t)(&CoClearFlag);
  crossfireSharedData.trampoline = trampoline;
#endif
}

void tangoUpdateChannel( void )
{
  uint8_t i;
  for ( i = 0; i < NUM_STICKS + NUM_SWITCHES; ++i)
    crossfireSharedData.channels[i] = channelOutputs[i];
}

void loadTangoRadioSettingsSettings(void)
{
  // this is to reset incorrect radio settings. should be removed later.
  g_eeGeneral.lightAutoOff = g_eeGeneral.lightAutoOff < BACKLIGHT_TIMEOUT_MIN ? 6 : g_eeGeneral.lightAutoOff;
  g_eeGeneral.backlightMode = g_eeGeneral.backlightMode < e_backlight_mode_keys ? e_backlight_mode_keys : g_eeGeneral.backlightMode;

  FILINFO info;
  FRESULT result;
  result = f_stat(RADIO_SETTINGS_PATH, &info);
  if(result == FR_OK && info.fsize == 728 && (info.fdate == 60449 || info.fdate < 20513 ) &&  
    g_eeGeneral.stickMode == 0 && g_eeGeneral.lightAutoOff == 2)
  {
    g_eeGeneral.stickMode = DEFAULT_MODE - 1;
    g_eeGeneral.view = VIEW_INPUTS;
    g_eeGeneral.lightAutoOff = 12;
  }
}

#if !defined(SIMU) && defined(DEBUG)
#define UART_INT_MODE_TX     1
#define UART_INT_MODE_RX     2
extern Fifo<uint8_t, 512> serial2TxFifo;
extern "C" void SERIAL_USART_IRQHandler(void)
{
  DEBUG_INTERRUPT(INT_SER2);
  bool xf_active = false;
  bool (*uart_irq)( uint8_t, uint8_t );
  bool xf_valid = false;
  uint8_t data = 0;

  if ( crossfireSharedData.trampoline[DEBUG_UART_IRQ_TRAMPOLINE] ){
    uart_irq = (bool (*)( uint8_t, uint8_t ))crossfireSharedData.trampoline[DEBUG_UART_IRQ_TRAMPOLINE];
    xf_valid = true;
  }

  // Send
  if (USART_GetITStatus(SERIAL_USART, USART_IT_TXE) != RESET) {
    if( xf_valid )
      xf_active = uart_irq( UART_INT_MODE_TX, 0);
    if( !xf_active ){
      if ( !serial2TxFifo.isEmpty() ) {
        /* Write one byte to the transmit data register */
        serial2TxFifo.pop(data);
        USART_SendData(SERIAL_USART, data);
      }
      else {
        USART_ITConfig(SERIAL_USART, USART_IT_TXE, DISABLE);
      }
    }
  }

  if ( USART_GetITStatus(SERIAL_USART, USART_IT_RXNE) != RESET ) {
    if ( xf_valid ) {
      // Receive
      data = USART_ReceiveData(SERIAL_USART);
      uart_irq( UART_INT_MODE_RX, data);
    }
    else
      data = USART_ReceiveData(SERIAL_USART);
  }
}
#endif

#if !defined(SIMU)
extern "C" {
void EXTI15_10_IRQHandler(void)
{
  CoEnterISR();
  void (*exti_irq)(void);
  if ( crossfireSharedData.trampoline[DIO_IRQ_TRAMPOLINE] ){
    exti_irq = (void (*)(void))crossfireSharedData.trampoline[DIO_IRQ_TRAMPOLINE];
    /* call DIOCN handler of crossfire */
    exti_irq();
    isr_SetFlag( get_task_flag( XF_TASK_FLAG ));
  }
  CoExitISR();
}

void TIM8_UP_TIM13_IRQHandler()
{
  CoEnterISR();
  if( TIM13->SR & TIM_SR_UIF )
  {
    TIM13->SR &= ~TIM_SR_UIF;
    void (*timer_irq)(void);
    if( crossfireSharedData.trampoline[NOTIFICATION_TIMER_IRQ_TRAMPOLINE] ){
      timer_irq = (void (*)(void))crossfireSharedData.trampoline[NOTIFICATION_TIMER_IRQ_TRAMPOLINE];
      /* call notification timer handler of crossfire */
      timer_irq();
      isr_SetFlag( get_task_flag( XF_TASK_FLAG ));
    }
  }
  CoExitISR();
}

#if defined(DEBUG) && defined(SERIAL_GPIO)
#include <stdio.h>
#include <stdarg.h>
void uart_tx(uint8_t byte)
{
  while(USART_GetFlagStatus(SERIAL_USART, USART_FLAG_TXE) == RESET);
    USART_SendData(SERIAL_USART, byte);
}

void hf_printf(const char * TxBuf, ...)
{
  uint8_t UartBuf[200];
  va_list arglist;
  volatile uint8_t *fp;

  va_start(arglist, TxBuf);
  vsprintf((char*)UartBuf, (const char*)TxBuf, arglist);
  va_end(arglist);

  fp = UartBuf;
  while(*fp)
  {
    uart_tx(*fp);
    fp++;
  }
}

void _general_exception_handler (unsigned int * hardfault_args)
{
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;

  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);

  hf_printf ("\r\n\n***OpenTx Hard Fault Handler Debug Printing***\r\n");
  hf_printf ("R0\t\t= 0x%.8x\r\n", stacked_r0);
  hf_printf ("R1\t\t= 0x%.8x\r\n", stacked_r1);
  hf_printf ("R2\t\t= 0x%.8x\r\n", stacked_r2);
  hf_printf ("R3\t\t= 0x%.8x\r\n", stacked_r3);
  hf_printf ("R12\t\t= 0x%.8x\r\n", stacked_r12);
  hf_printf ("LR [R14]\t= 0x%.8x\r\n", stacked_lr);
  hf_printf ("PC [R15]\t= 0x%.8x\r\n", stacked_pc);
  hf_printf ("PSR\t\t= 0x%.8x\r\n", stacked_psr);
  hf_printf ("BFAR\t\t= 0x%.8x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
  hf_printf ("CFSR\t\t= 0x%.8x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
  hf_printf ("HFSR\t\t= 0x%.8x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
  hf_printf ("AFSR\t\t= 0x%.8x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));
  hf_printf ("SCB_SHCSR\t= 0x%.8x\r\n", SCB->SHCSR);

  while (1);
}
#endif

void HardFault_Handler(void)
{
#if defined(DEBUG)
  __asm("TST LR, #4");
  __asm("ITE EQ");
  __asm("MRSEQ R0, MSP");
  __asm("MRSNE R0, PSP");
  __asm("B _general_exception_handler");
#else
  NVIC_SystemReset();
#endif
}
}  //extern "C" {
#endif
