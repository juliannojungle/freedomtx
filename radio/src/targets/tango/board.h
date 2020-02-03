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

#include "stddef.h"
#include "stdbool.h"

#if defined(__cplusplus) && !defined(SIMU)
extern "C" {
#endif

#if __clang__
// clang is very picky about the use of "register"
// tell it to ignore for the STM32 includes instead of modyfing the orginal files
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-register"
#endif

#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rcc.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_syscfg.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_exti.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_tim.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_spi.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_i2c.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rtc.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_pwr.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dma.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_usart.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_flash.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dbgmcu.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/misc.h"
#include "STM32F4xx_DSP_StdPeriph_Lib_V1.4.0/Libraries/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_sdio.h"


#if __clang__
// Restore warnings about registers
#pragma clang diagnostic pop
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

#define FLASHSIZE                       0xC0000
#define BOOTLOADER_SIZE                 0xC000
#define FIRMWARE_ADDRESS                0x08000000

#define LUA_MEM_MAX                     (0)    // max allowed memory usage for complete Lua  (in bytes), 0 means unlimited

#define PERI1_FREQUENCY                 42000000
#define PERI2_FREQUENCY                 84000000

#define TIMER_MULT_APB1                 2
#define TIMER_MULT_APB2                 2

extern uint16_t sessionTimer;

// Board driver
void boardPreInit(void);
void boardInit(void);
void boardOff(void);

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
uint32_t sdIsHC(void);
uint32_t sdGetSpeed(void);
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
#define __disk_read                    disk_read
#define __disk_write                   disk_write
#endif

#if defined(SIMU)
  #if !defined(SIMU_DISKIO)
    #define sdInit()
    #define sdDone()
  #endif
  #define sdMount()
  #define SD_CARD_PRESENT()               true
#else
void sdInit(void);
void sdMount(void);
void sdDone(void);
void sdPoll10ms(void);
uint32_t sdMounted(void);
#define SD_CARD_PRESENT()                 (true)
#endif

// Flash Write driver
#define FLASH_PAGESIZE 256
void unlockFlash(void);
void lockFlash(void);
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

#define NUM_SWITCHES                        6

void keysInit(void);
uint8_t keyState(uint8_t index);
uint32_t switchState(uint8_t index);
uint32_t readKeys(void);
uint32_t readTrims(void);

extern uint8_t g_trimEditMode;
extern uint8_t g_trimState;


#define TRIMS_PRESSED()                     (readTrims())
#define KEYS_PRESSED()                      (readKeys())

// Rotary Encoder driver
#define ROTARY_ENCODER_NAVIGATION
void rotaryEncoderInit(void);
void rotaryEncoderCheck(void);

// WDT driver
#define WDTO_500MS                          500
#if defined(WATCHDOG_DISABLED) || defined(SIMU)
  #define wdt_enable(x)
  #define wdt_reset()
#else
  #define wdt_enable(x)                     watchdogInit(x)
  #define wdt_reset()                       IWDG->KR = 0xAAAA
#endif
#define wdt_disable()
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
  POT_FIRST,
  POT1 = POT_FIRST,
  POT2,
  POT3,
  POT_LAST = POT3,
  SLIDER1,
  SLIDER2,
  TX_VOLTAGE,
  NUM_ANALOGS,
  TX_RTC = NUM_ANALOGS
};

#define NUM_POTS                        0
#define NUM_XPOTS                       0
#define NUM_SLIDERS                     0
#define NUM_TRIMS                       4
#define NUM_MOUSE_ANALOGS               0
#define NUM_DUMMY_ANAS                  0

#define NUM_TRIMS_KEYS                  8
#define STICKS_PWM_ENABLED()            false


// Hardware options
typedef struct
{
#if NUM_PWMSTICKS > 0
  uint8_t sticksPwmDisabled:1;
#endif
  uint8_t pxx2Enabled:1;
} HardwareOptions;


extern HardwareOptions hardwareOptions;

enum CalibratedAnalogs {
  CALIBRATED_STICK1,
  CALIBRATED_STICK2,
  CALIBRATED_STICK3,
  CALIBRATED_STICK4,
  NUM_CALIBRATED_ANALOGS
};


#define IS_POT(x)                     (false)
#define IS_SLIDER(x)                  (false)
void adcInit(void);
void adcRead(void);
extern uint16_t adcValues[NUM_ANALOGS + 1/*RTC*/];
uint16_t getAnalogValue(uint8_t index);
uint16_t getRTCBattVoltage();

// Battery driver
uint16_t getBatteryVoltage();   // returns current battery voltage in 10mV steps
// 1 x Li-Ion
#define BATTERY_WARN                  35 // 3.5V
#define BATTERY_MIN                   34 // 3.4V
#define BATTERY_MAX                   42 // 4.2V

#define BATT_CALIB_OFFSET             5
#define BATT_SCALE                    (4.446f)
// BATT_SCALE = 12-bit max value * pd / ANALOG_MULTIPLIER / vref / multiplication
//            = 4095 * 2/3 / 2 / 3.07 / 100

#if defined(__cplusplus) && !defined(SIMU)
extern "C" {
#endif

// Power driver
#define SOFT_PWR_CTRL
void pwrInit(void);
uint32_t pwrCheck(void);
void pwrOn(void);
void pwrOff(void);
uint32_t pwrPressed(void);
#if defined(PWR_BUTTON_PRESS)
uint32_t pwrPressedDuration(void);
#endif
void pwrResetHandler(void);

#if defined(SIMU)
#define UNEXPECTED_SHUTDOWN()           false
#else
#define UNEXPECTED_SHUTDOWN()           (WAS_RESET_BY_WATCHDOG() || g_eeGeneral.unexpectedShutdown)
#endif

// Backlight driver
#define backlightInit()
#define backlightDisable()              lcdOff() 
#define BACKLIGHT_DISABLE()             backlightDisable()
#define isBacklightEnabled()            isLcdOn()
#define backlightEnable(level)          lcdOn()
#define BACKLIGHT_ENABLE()              backlightEnable(g_eeGeneral.backlightBright)
#define BACKLIGHT_TIMEOUT_MIN           2

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
void telemetryPortSetDirectionOutput(void);
void sportSendBuffer(const uint8_t * buffer, uint32_t count);
uint8_t telemetryGetByte(uint8_t * byte);
extern uint32_t telemetryErrors;

// Charger
#define IS_CHARGING_STATE()           (GPIO_ReadInputDataBit( CHARGER_STATE_GPIO, CHARGER_STATE_GPIO_PIN ) == Bit_RESET)
#define IS_CHARGING_FAULT()           (GPIO_ReadInputDataBit( CHARGER_FAULT_GPIO, CHARGER_FAULT_GPIO_PIN ) == Bit_RESET)

// Audio driver
void audioInit(void) ;
void audioEnd(void) ;
void dacStart(void);
void dacStop(void);
void setSampleRate(uint32_t frequency);
#define VOLUME_LEVEL_MAX  23
#define VOLUME_LEVEL_DEF  12
#if !defined(SOFTWARE_VOLUME)
void setScaledVolume(uint8_t volume);
void setVolume(uint8_t volume);
int32_t getVolume(void);
#endif
void audioConsumeCurrentBuffer();
#define audioDisableIrq()               __disable_irq()
#define audioEnableIrq()                __enable_irq()

// Haptic driver
void hapticInit(void);
void hapticOff(void);
void hapticOn(void);

// Second serial port driver
#if defined(SERIAL_GPIO)
#define DEBUG_BAUDRATE                  500000
#define SERIAL2
extern uint8_t serial2Mode;
void serial2Init(unsigned int mode, unsigned int protocol);
void serial2Putc(char c);
#define serial2TelemetryInit(protocol) serial2Init(UART_MODE_TELEMETRY, protocol)
void serial2SbusInit(void);
void serial2Stop(void);
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
void lcdInit(void);
void lcdOn(void);
void lcdOff(void);
bool isLcdOn(void);

// TODO lcdRefreshWait() stub in simpgmspace and remove LCD_DUAL_BUFFER
#if defined(LCD_DMA) && !defined(LCD_DUAL_BUFFER) && !defined(SIMU)
void lcdRefreshWait();
#else
#define lcdRefreshWait()
#endif
#if defined(SIMU) || !defined(__cplusplus)
void lcdRefresh(void);
#else
void lcdRefresh(bool wait=true); // TODO uint8_t wait to simplify this
#endif
void lcdSetRefVolt(unsigned char val);
void lcdSetContrast(void);

#define USART_FLAG_ERRORS (USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE)

#define checkTrainerSettings()

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
extern DMAFifo<32> serial2RxFifo;
#endif

#define DISPLAY_PROGRESS_BAR(...) do{} while(0)

void CRSF_Init(void);
void tangoUpdateChannel(void);
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

enum {
  DEVICE_RESTART_WITHOUT_WARN_FLAG = 0x0,
};

uint32_t readBackupReg(uint8_t index);
void writeBackupReg(uint8_t index, uint32_t data);
uint8_t getFlag(uint32_t flag);
void setFlag(uint32_t flag);
void clearFlag(uint32_t flag);
void boot2bootloader(uint32_t isNeedFlash, uint32_t HwId, uint32_t sn);
uint8_t getBoardOffState();

void crossfireOff( void );

void PrintData(char* header, uint8_t* data);

void loadTangoRadioSettingsSettings(void);

#endif // _BOARD_H_
