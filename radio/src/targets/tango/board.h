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

#ifndef _BOARD_H_
#define _BOARD_H_

#include <inttypes.h>
#include "definitions.h"
#include "opentx_constants.h"
#include "board_common.h"
#include "hal.h"


#if defined(ROTARY_ENCODER_NAVIGATION)
// Rotary Encoder driver
void rotaryEncoderInit();
void rotaryEncoderCheck();
#endif


#if !defined(LUA_EXPORT_GENERATION)
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_sdio.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dma2d.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_ltdc.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_fmc.h"
#endif

#define MY_DEVICE_NAME                  "Tango II"
#define FLASHSIZE                       0xC0000
#define BOOTLOADER_SIZE                 0xC000
#define FIRMWARE_ADDRESS                0x08000000
#define CROSSFIRE_TASK_ADDRESS          0x080C0020
#define SHARED_MEMORY_ADDRESS           0x10000000

#define LUA_MEM_MAX                     (0)    // max allowed memory usage for complete Lua  (in bytes), 0 means unlimited

#define PERI1_FREQUENCY                 42000000
#define PERI2_FREQUENCY                 84000000

#define MODEL_DATA_SIZE_101             6245
#define MODEL_DATA_SIZE_110             6253


#define TIMER_MULT_APB1                 2
#define TIMER_MULT_APB2                 2

extern uint16_t sessionTimer;

// Board driver
void boardInit();
void boardOff();

// Timers driver
void init2MhzTimer();
void init5msTimer();

// PCBREV driver
enum {
  // Tango2
  PCBREV_Tango2_Unknown = 0,
  PCBREV_Tango2_V1,
  PCBREV_Tango2_V2,
};

// SD driver
#define BLOCK_SIZE                      512 /* Block Size in Bytes */
#if !defined(SIMU) || defined(SIMU_DISKIO)
uint32_t sdIsHC();
uint32_t sdGetSpeed();
#define SD_IS_HC()                      (sdIsHC())
#define SD_GET_SPEED()                  (sdGetSpeed())
#define SD_GET_FREE_BLOCKNR()           (sdGetFreeSectors())
#else
#define SD_IS_HC()                      (0)
#define SD_GET_SPEED()                  (0)
#endif

#if defined(DISK_CACHE)
#include "diskio.h"
DRESULT __disk_read(BYTE drv, BYTE * buff, DWORD sector, UINT count);
DRESULT __disk_write(BYTE drv, const BYTE * buff, DWORD sector, UINT count);
#else
#define __disk_read                     disk_read
#define __disk_write                    disk_write
#define DISK_OPERATION_TIMEOUT          5
#define SD_MAX_VOLT_TRIAL               ((uint32_t)0x000000FF)
#endif

#if defined(SIMU)
  #if !defined(SIMU_DISKIO)
    #define sdInit()
    #define sdDone()
  #endif
  #define sdMount()
  #define SD_CARD_PRESENT()               true
#else
void sdInit();
void sdMount();
void sdDone();
#define sdPoll10ms()
uint32_t sdMounted();
#define SD_CARD_PRESENT()                 (true)
#endif

// Flash Write driver
#define FLASH_PAGESIZE 256
void unlockFlash();
void lockFlash();
void flashWrite(uint32_t * address, const uint32_t * buffer);
uint32_t isFirmwareStart(const uint8_t * buffer);
uint32_t isBootloaderStart(const uint8_t * buffer);

#define IS_INTERNAL_MODULE_ON()         false

// Keys driver
enum EnumKeys
{
  KEY_MENU,
  KEY_EXIT,
  KEY_ENTER,
  KEY_PAGE,
  KEY_PLUS,
  KEY_MINUS,
  KEY_COUNT,
  KEY_MAX = KEY_COUNT - 1,
  TRM_BASE,
  TRM_LH_DWN = TRM_BASE,
  TRM_LH_UP,
  TRM_LV_DWN,
  TRM_LV_UP,
  TRM_RV_DWN,
  TRM_RV_UP,
  TRM_RH_DWN,
  TRM_RH_UP,
  TRM_LAST = TRM_RH_UP,

  NUM_KEYS
};

enum EnumTrimMode{
  EDIT_TRIM_DISABLED = 0,
  EDIT_TRIM_1,
  EDIT_TRIM_2,
  EDIT_TRIM_3,
  EDIT_TRIM_4,
  EDIT_TRIM_MAX = EDIT_TRIM_4,
};

#define KEY_UP                          KEY_PLUS
#define KEY_DOWN                        KEY_MINUS
#define KEY_RIGHT                       KEY_MINUS
#define KEY_LEFT                        KEY_PLUS

#define IS_SHIFT_KEY(index)             (false)
#define IS_SHIFT_PRESSED()              (false)

enum EnumSwitches
{
  SW_SA,
  SW_SB,
  SW_SC,
  SW_SD,
  SW_SE,
  SW_SF,
};
#define IS_3POS(x)                      ((x) != SW_SA && (x) != SW_SD && (x) != SW_SE && (x) != SW_SF )

enum EnumSwitchesPositions
{
  SW_SA0,
  SW_SA1,
  SW_SA2,
  SW_SB0,
  SW_SB1,
  SW_SB2,
  SW_SC0,
  SW_SC1,
  SW_SC2,
  SW_SD0,
  SW_SD1,
  SW_SD2,
  SW_SE0,
  SW_SE1,
  SW_SE2,
  SW_SF0,
  SW_SF1,
  SW_SF2,
};

#define NUM_SWITCHES                    6
#define STORAGE_NUM_SWITCHES            NUM_SWITCHES
#define DEFAULT_SWITCH_CONFIG           (SWITCH_TOGGLE << 10) + (SWITCH_TOGGLE << 8) + (SWITCH_2POS << 6) + (SWITCH_3POS << 4) + (SWITCH_3POS << 2) + (SWITCH_2POS << 0)

#define STORAGE_NUM_SWITCHES_POSITIONS  (STORAGE_NUM_SWITCHES * 3)

void keysInit();
uint8_t keyState(uint8_t index);
uint32_t switchState(uint8_t index);
uint32_t readKeys();
uint32_t readTrims();

extern uint8_t g_trimEditMode;
extern uint8_t g_trimState;


#define TRIMS_PRESSED()                 (readTrims())
#define KEYS_PRESSED()                  (readKeys())

// WDT driver
#define WDG_DURATION                      500 /*ms*/
#if !defined(WATCHDOG) || defined(SIMU)
  #define WDG_ENABLE(x)
  #define WDG_RESET()
  #define WAS_RESET_BY_WATCHDOG()               (false)
  #define WAS_RESET_BY_SOFTWARE()               (false)
  #define WAS_RESET_BY_WATCHDOG_OR_SOFTWARE()   (false)
#else
  void watchdogInit(unsigned int duration);
  #define WDG_ENABLE(x)                 watchdogInit(x)
  #define WDG_RESET()                   IWDG->KR = 0xAAAA
  #define WAS_RESET_BY_SOFTWARE()             (RCC->CSR & RCC_CSR_SFTRSTF)
  #define WAS_RESET_BY_WATCHDOG()             (RCC->CSR & (RCC_CSR_WDGRSTF | RCC_CSR_WWDGRSTF))
  #define WAS_RESET_BY_WATCHDOG_OR_SOFTWARE() (RCC->CSR & (RCC_CSR_WDGRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_SFTRSTF))
#endif

// ADC driver
enum Analogs {
  STICK1,
  STICK2,
  STICK3,
  STICK4,
  TX_VOLTAGE,
  TX_RTC_VOLTAGE,
  NUM_ANALOGS
};

#define NUM_POTS                        0
#define NUM_XPOTS                       0
#define NUM_SLIDERS                     0
#define NUM_TRIMS                       4
#define NUM_MOUSE_ANALOGS               0
#define STORAGE_NUM_MOUSE_ANALOGS       0

#define STORAGE_NUM_POTS                0
#define STORAGE_NUM_SLIDERS             0


PACK(typedef struct {
  uint8_t pcbrev:2;
  uint8_t sticksPwmDisabled:1;
  uint8_t pxx2Enabled:1;
}) HardwareOptions;

extern HardwareOptions hardwareOptions;

#define NUM_TRIMS_KEYS                  8
#define STICKS_PWM_ENABLED()            false

enum CalibratedAnalogs {
  CALIBRATED_STICK1,
  CALIBRATED_STICK2,
  CALIBRATED_STICK3,
  CALIBRATED_STICK4,
  NUM_CALIBRATED_ANALOGS
};

#define IS_POT(x)                     (false)
#define IS_SLIDER(x)                  (false)

extern uint16_t adcValues[NUM_ANALOGS];

// Battery driver
uint16_t getBatteryVoltage();   // returns current battery voltage in 10mV steps
// 1 x Li-Ion
#define BATTERY_WARN                  35 // 3.5V
#define BATTERY_MIN                   34 // 3.4V
#define BATTERY_MAX                   42 // 4.2V

#define BATT_CALIB_OFFSET             5
#define BATT_SCALE                    (4.446f)
#define BATT_SCALE2                   (4.162f)
// BATT_SCALE = 12-bit max value * pd / ANALOG_MULTIPLIER / vref / multiplication
//            = 4095 * 2/3 / 2 / vref / 100

#if defined(__cplusplus)
enum PowerReason {
  SHUTDOWN_REQUEST = 0xDEADBEEF,
  SOFTRESET_REQUEST = 0xCAFEDEAD,
};

constexpr uint32_t POWER_REASON_SIGNATURE = 0x0178746F;

inline bool UNEXPECTED_SHUTDOWN()
{
#if defined(SIMU) || defined(NO_UNEXPECTED_SHUTDOWN)
  return false;
#else
  if (WAS_RESET_BY_WATCHDOG())
    return true;
  else if (WAS_RESET_BY_SOFTWARE())
    return RTC->BKP0R != SOFTRESET_REQUEST;
  else
    return RTC->BKP1R == POWER_REASON_SIGNATURE && RTC->BKP0R != SHUTDOWN_REQUEST;
#endif
}

inline void SET_POWER_REASON(uint32_t value)
{
  RTC->BKP0R = value;
  RTC->BKP1R = POWER_REASON_SIGNATURE;
}
#endif

#if defined(__cplusplus) && !defined(SIMU)
extern "C" {
#endif

// Power driver
#define SOFT_PWR_CTRL
void pwrInit();
uint32_t pwrCheck();
void pwrOn();
void pwrOff();
bool pwrPressed();
#define STARTUP_ANIMATION
uint32_t pwrPressedDuration();
void pwrResetHandler();

#if defined(SIMU)
#define UNEXPECTED_SHUTDOWN()           false
#else
#define UNEXPECTED_SHUTDOWN()           (WAS_RESET_BY_WATCHDOG() || g_eeGeneral.unexpectedShutdown)
#endif

// Backlight driver
#define BACKLIGHT_TIMEOUT_MIN           2
#define backlightInit()
void backlightEnable(uint8_t level);
#if defined(SIMU)
  #define backlightDisable()
  #define BACKLIGHT_DISABLE()
  #define isBacklightEnabled()            true
  #define backlightEnable(level)
  #define BACKLIGHT_ENABLE()
#else
  #define backlightDisable()              lcdOff()
  #define BACKLIGHT_DISABLE()             backlightDisable()
  #define isBacklightEnabled()            isLcdOn()
  #define BACKLIGHT_ENABLE()              backlightEnable(g_eeGeneral.backlightBright)
#endif

void usbJoystickUpdate();
#define USB_NAME                        "TBS"
#define USB_MANUFACTURER                'T', 'B', 'S', ' ', ' ', ' ', ' ', ' '  /* 8 bytes */
#define USB_PRODUCT                     'T', 'a', 'n', 'g', 'o', ' ', '2', ' '  /* 8 Bytes */

#if defined(__cplusplus) && !defined(SIMU)
}
#endif


// Debug driver
void debugPutc(const char c);

// Telemetry driver
void telemetryPortInit(uint32_t baudrate, uint8_t mode);
void telemetryPortSetDirectionOutput();
void sportSendBuffer(const uint8_t * buffer, uint32_t count);
bool telemetryGetByte(uint8_t * byte);
extern uint32_t telemetryErrors;
void telemetryPortInit(uint32_t baudrate, uint8_t mode);
void telemetryPortSetDirectionInput();
void telemetryPortSetDirectionOutput();
void sportSendByte(uint8_t byte);
void sportSendByteLoop(uint8_t byte);
void sportStopSendByteLoop();
void sportSendBuffer(const uint8_t * buffer, uint32_t count);
bool telemetryGetByte(uint8_t * byte);
void telemetryClearFifo();
extern uint32_t telemetryErrors;
// soft-serial
void telemetryPortInvertedInit(uint32_t baudrate);

// external module driver
void intmoduleStop();
void intmoduleSerialStart(uint32_t baudrate, uint8_t rxEnable);
void extmoduleStop();
void extmodulePpmStart();
void extmodulePxxPulsesStart();
void extmodulePxxSerialStart();
void extmodulePxx2Start();
void extmoduleSerialStart(uint32_t baudrate, uint32_t period_half_us, bool inverted);
void extmoduleInvertedSerialStart(uint32_t baudrate);
void extmoduleSendBuffer(const uint8_t * data, uint8_t size);
void extmoduleSendNextFrame();
void extmoduleSendInvertedByte(uint8_t byte);

// Sport update driver
#define SPORT_UPDATE_POWER_ON()
#define SPORT_UPDATE_POWER_OFF()
#define INTERNAL_MODULE_OFF()

#define EXTERNAL_MODULE_ON()          EXTERNAL_MODULE_PWR_ON()
#if defined(EXTMODULE_USART)
  #define EXTERNAL_MODULE_OFF()       extmoduleStop()
#else
  #define EXTERNAL_MODULE_OFF()       EXTERNAL_MODULE_PWR_OFF()
#endif
#define IS_EXTERNAL_MODULE_ON()         (GPIO_ReadInputDataBit(EXTMODULE_PWR_GPIO, EXTMODULE_PWR_GPIO_PIN) == Bit_SET)

// PCBREV driver
#define IS_PCBREV_01()                (hardwareOptions.pcbrev == PCBREV_Tango2_V1)
#define IS_PCBREV_02()                (hardwareOptions.pcbrev == PCBREV_Tango2_V2)

// Charger
#define IS_CHARGING_STATE()           (GPIO_ReadInputDataBit( CHARGER_STATE_GPIO, CHARGER_STATE_GPIO_PIN ) == Bit_RESET)
#define IS_CHARGING_FAULT()           (GPIO_ReadInputDataBit( CHARGER_FAULT_GPIO, CHARGER_FAULT_GPIO_PIN ) == Bit_RESET)

// Audio driver
void audioInit() ;
void audioEnd() ;
void dacStart();
void dacStop();
void setSampleRate(uint32_t frequency);
#define VOLUME_LEVEL_MAX  23
#define VOLUME_LEVEL_DEF  12
#if !defined(SOFTWARE_VOLUME)
void setScaledVolume(uint8_t volume);
void setVolume(uint8_t volume);
int32_t getVolume();
#endif
void audioConsumeCurrentBuffer();
#define audioDisableIrq()               __disable_irq()
#define audioEnableIrq()                __enable_irq()

// Haptic driver
void hapticInit();
void hapticOff();
void hapticOn();

// Second serial port driver
#if defined(AUX_SERIAL_GPIO)
#define DEBUG_BAUDRATE                  500000
#if defined(DEBUG)
#define AUX_SERIAL
#endif
extern uint8_t auxSerialMode;
void auxSerialInit(unsigned int mode, unsigned int protocol);
void auxSerialPutc(char c);
#define auxSerialTelemetryInit(protocol) auxSerialInit(UART_MODE_TELEMETRY, protocol)
void auxSerialSbusInit();
void auxSerialStop();
#define AUX_SERIAL_POWER_ON()
#define AUX_SERIAL__POWER_OFF()
#endif

// BT driver
#define IS_BLUETOOTH_CHIP_PRESENT()     (false)

// Led driver
void ledInit(void);
void ledOff(void);
void ledRed(void);
void ledBlue(void);
void ledGreen(void);
void ledWhite(void);
#if defined(CHARGING_LEDS)
  #define LED_CHARGING_IN_PROGRESS()    ledRed()
  #define LED_CHARGING_DONE()           ledGreen()
#else
  #define LED_CHARGING_IN_PROGRESS()
  #define LED_CHARGING_DONE()
#endif

// LCD driver
#define LCD_W                           128
#define LCD_H                           96
#define LCD_DEPTH                       4
#define IS_LCD_RESET_NEEDED()           true
#define LCD_CONTRAST_MIN                0
#define LCD_CONTRAST_MAX                45
#define LCD_CONTRAST_DEFAULT            20
void lcdInit();
void lcdOn();
void lcdOff();
bool isLcdOn();
void lcdAdjustContrast(uint8_t val);

// TODO lcdRefreshWait() stub in simpgmspace and remove LCD_DUAL_BUFFER
#if defined(LCD_DMA) && !defined(LCD_DUAL_BUFFER) && !defined(SIMU)
void lcdRefreshWait();
#else
#define lcdRefreshWait()
#endif
#if defined(SIMU) || !defined(__cplusplus)
void lcdRefresh();
#else
void lcdRefresh(bool wait=true); // TODO uint8_t wait to simplify this
#endif
void lcdSetRefVolt(unsigned char val);
void lcdSetContrast();

// Wifi driver
#define WIFI_IS_ON()                    GPIO_ReadOutputDataBit(ESP_EN_GPIO, ESP_EN_GPIO_PIN)
#define WIFI_OFF()                      GPIO_ResetBits(ESP_EN_GPIO, ESP_EN_GPIO_PIN)

#define USART_FLAG_ERRORS (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)

#if defined(__cplusplus)
#include "fifo.h"
#include "dmafifo.h"

#define TELEMETRY_FIFO_SIZE             128
extern Fifo<uint8_t, TELEMETRY_FIFO_SIZE> telemetryFifo;
typedef DMAFifo<32> AuxSerialRxFifo;
extern AuxSerialRxFifo auxSerialRxFifo;
#endif

#if defined(SIMU)
  #define getBoardOffState(...)                 0
#else
  uint8_t getBoardOffState();
#endif

void trampolineInit(void);
void boardReboot2bootloader(uint32_t isNeedFlash, uint32_t HwId, uint32_t sn);
void loadDefaultRadioSettings(void);

#endif // _BOARD_H_
