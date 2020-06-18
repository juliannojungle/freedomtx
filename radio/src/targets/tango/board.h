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
#include "../definitions.h"
#include "../opentx_constants.h"
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

#define FLASHSIZE                       0xC0000
#define BOOTLOADER_SIZE                 0xC000
#define FIRMWARE_ADDRESS                0x08000000

#define LUA_MEM_MAX                     (0)    // max allowed memory usage for complete Lua  (in bytes), 0 means unlimited

#define PERI1_FREQUENCY                 42000000
#define PERI2_FREQUENCY                 84000000

#define MODEL_DATA_SIZE_101             6245
#define MODEL_DATA_SIZE_110             6253

#if defined(__cplusplus) && !defined(SIMU)
extern "C" {
#endif



#include "usb_driver.h"
#if !defined(SIMU)
  #include "usbd_cdc_core.h"
  #include "usbd_msc_core.h"
  #include "usbd_hid_core.h"
  #include "usbd_usr.h"
  #include "usbd_desc.h"
  #include "usb_conf.h"
  #include "usbd_conf.h"
#endif

#include "hal.h"

#if defined(__cplusplus) && !defined(SIMU)
}
#endif



#define TIMER_MULT_APB1                 2
#define TIMER_MULT_APB2                 2

extern uint16_t sessionTimer;

// Board driver
void boardPreInit(void);
void boardInit();
void boardOff();

// Timers driver
void init2MhzTimer();
void init5msTimer();


// Delays driver
#ifdef __cplusplus
extern "C" {
#endif
void delaysInit(void);
void delay_01us(uint16_t nb);
void delay_us(uint16_t nb);
void delay_ms(uint32_t ms);
#ifdef __cplusplus
}
#endif

// CPU Unique ID
#define LEN_CPU_UID                     (3*8+2)
void getCPUUniqueID(char * s);

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
void flashWrite(uint32_t * address, uint32_t * buffer);
uint32_t isFirmwareStart(const uint8_t * buffer);
uint32_t isBootloaderStart(const uint8_t * buffer);

// Pulses driver
#define disable_serial(...)
#define init_ppm(...)
#define disable_ppm(...)
#define extmoduleSerialStart(...)
#define extmoduleSendNextFrame()

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
#define WDTO_500MS                      500
#if !defined(WATCHDOG) || defined(SIMU)
  #define wdt_enable(x)
  #define wdt_reset()
#else
  #define wdt_enable(x)                 watchdogInit(x)
  #define wdt_reset()                   IWDG->KR = 0xAAAA
#endif
void watchdogInit(unsigned int duration);
#define WAS_RESET_BY_SOFTWARE()             (RCC->CSR & RCC_CSR_SFTRSTF)
#define WAS_RESET_BY_WATCHDOG()             (RCC->CSR & (RCC_CSR_WDGRSTF | RCC_CSR_WWDGRSTF))
#define WAS_RESET_BY_WATCHDOG_OR_SOFTWARE() (RCC->CSR & (RCC_CSR_WDGRSTF | RCC_CSR_WWDGRSTF | RCC_CSR_SFTRSTF))

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


typedef struct
{
#if NUM_PWMSTICKS > 0
  uint8_t sticksPwmDisabled:1;
#endif
  uint8_t pxx2Enabled:1;
} HardwareOptions;

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
uint16_t getAnalogValue(uint8_t index);

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
#if defined(PWR_BUTTON_PRESS)
#define STARTUP_ANIMATION
uint32_t pwrPressedDuration();
#endif
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

#if !defined(SIMU)
  void usbJoystickUpdate();
#endif
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
uint8_t telemetryGetByte(uint8_t * byte);
extern uint32_t telemetryErrors;

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
void hapticOff(void);
void hapticOn();

// Second serial port driver
#if defined(AUX_SERIAL_GPIO)
#define DEBUG_BAUDRATE                  500000
#define AUX_SERIAL
extern uint8_t auxSerialMode;
void auxSerialInit(unsigned int mode, unsigned int protocol);
void auxSerialPutc(char c);
#define auxSerialTelemetryInit(protocol) auxSerialInit(UART_MODE_TELEMETRY, protocol)
void auxSerialSbusInit();
void auxSerialStop();
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

#define USART_FLAG_ERRORS (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)

#if defined(__cplusplus)
#include "fifo.h"
#include "dmafifo.h"

#if defined(CROSSFIRE)
#define TELEMETRY_FIFO_SIZE             128
#else
#define TELEMETRY_FIFO_SIZE             64
#endif

#define SHARED_MEMORY_ADDRESS           0x10000000

#define CROSSFIRE_STACK_SIZE            700
#define CROSSFIRE_TASK_PRIORITY         0
#define CROSSFIRE_TASK_ADDRESS          0x080C0020
#define SYSTEM_STACK_SIZE               500
#define RTOS_SYS_TASK_PRIORITY          10


extern Fifo<uint8_t, TELEMETRY_FIFO_SIZE> telemetryFifo;
typedef DMAFifo<32> AuxSerialRxFifo;
extern AuxSerialRxFifo auxSerialRxFifo;
#endif

#define DISPLAY_PROGRESS_BAR(...) do{} while(0)

void CRSF_Init(void);
void trampolineInit(void);

#if defined(ESP_SERIAL)
#define ESP_TX_BUFFER_SIZE                    256
#define ESP_RX_BUFFER_SIZE                    256
#define ESP_UART_BAUDRATE                     500000
void espInit(uint32_t baudrate, bool use_dma);
void espOff();
void espWriteBuffer(uint8_t* buf, uint16_t len);
void ESP_WriteHandler(void);
uint8_t espReadBuffer(uint8_t* buf);
#endif

#define BOOTLOADER_IS_NEED_FLASH_ADDR         0x0
#define BOOTLOADER_HW_ID_ADDR                 0x1
#define BOOTLOADER_IS_SKIP_BOARD_OFF_ADDR     0x2
#define BOOTLOADER_SERIAL_NO_ADDR             0x3
#define BOOTLOADER_HW_ID_ADDR_OPENTX          0x4
#define BOOTLOADER_HW_ID_ADDR_XF              0x5
#define BOOTLOADER_SERIAL_NO_ADDR_OPENTX      0x6
#define BOOTLOADER_SERIAL_NO_ADDR_XF          0x7
#define BOOTLOADER_FLAG_ADDR                  0x8
#define BOOTLOADER_DEFAULT_WORD_ADDR          0x9

typedef enum {
  DEVICE_RESTART_WITHOUT_WARN_FLAG = 0x0,
  STORAGE_ERASE_STATUS,
  BOOTLOADER_FLAG_COUNT,
} BOOTLOADER_FLAG_INDEX;

void boardTurnOffRf();
void boardSetSkipWarning();
uint32_t readBackupReg(uint8_t index);
void writeBackupReg(uint8_t index, uint32_t data);
void boot2bootloader(uint32_t isNeedFlash, uint32_t HwId, uint32_t sn);
uint8_t getBoardOffState();
uint8_t getStatusFlag(uint32_t flag);
void setStatusFlag(uint32_t flag);
void clrStatusFlag(uint32_t flag);

void crossfireOff( void );

void PrintData(char* header, uint8_t* data);

void loadDefaultRadioSettings(void);

#endif // _BOARD_H_
