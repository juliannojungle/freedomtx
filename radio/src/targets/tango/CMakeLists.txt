option(DISK_CACHE "Enable SD card disk cache" ON)
option(SHUTDOWN_CONFIRMATION "Shutdown confirmation" OFF)
option(LCD_DUAL_BUFFER "Dual LCD Buffer" OFF)


option(CROSSFIRE_TASK "Crossfire task" YES)
set(PWR_BUTTON "PRESS" CACHE STRING "Pwr button type (PRESS/SWITCH)")
set(PWR_BUTTON "PRESS" CACHE STRING "Pwr button type (PRESS/SWITCH)")
set(STICKS "STANDARD" CACHE STRING "Sticks type (STANDARD/HORUS)")
set(CPU_TYPE STM32F4)
set(CPU_TYPE_FULL STM32F407xE)  # for size report
set(LINKER_SCRIPT targets/tango/stm32f4_flash.ld)
set(HAPTIC YES)
set(LUA_EXPORT lua_export_tango)
set(FLAVOUR tango)
set(RAMBACKUP YES)
add_definitions(-DSTM32F40_41xxx -DPCBTANGO)
add_definitions(-DEEPROM_VARIANT=0x8000)
add_definitions(-DPWR_BUTTON_${PWR_BUTTON})
add_definitions(-DSOFTWARE_VOLUME)

set(FIRMWARE_TARGET_SRC
  ${FIRMWARE_TARGET_SRC}
  startup_stm32f40_41xxx.s
  )

set(SERIAL2_DRIVER ../common/arm/stm32/serial2_driver.cpp)

if (CROSSFIRE_TASK)
  add_definitions(-DCROSSFIRE_TASK)
endif()

add_definitions(-DCROSSFIRE)
add_definitions(-DLIBCRSF_ENABLE_OPENTX_RELATED -DLIBCRSF_ENABLE_SD -DLIBCRSF_ENABLE_COMMAND -DLIBCRSF_ENABLE_PARAMETER)
add_definitions(-DDEFAULT_MODE=2)

set(GUI_DIR 128x96)
set(NAVIGATION_TYPE tango)
set(BITMAPS_TARGET tango_bitmaps)
set(BITMAPS_TARGET_DIR tango)
set(FONTS_TARGET tango_fonts_1bit)
set(LCD_DRIVER lcd_driver_spi.cpp)
set(GVAR_SCREEN model_gvars.cpp)


if(PCB STREQUAL TANGO)
  set(TARGET_SRC
    ${TARGET_SRC}
    ../common/arm/stm32/rotary_encoder_driver.cpp
    )
endif()

add_definitions(-DESP_SERIAL)
set(TARGET_SRC
  ${TARGET_SRC}
  ../tango/esp_driver.cpp
  )
add_definitions(-DAGENT)
set(TARGET_SRC
  ${TARGET_SRC}
  ../tango/usbd_hid_agent.c
  )
set(SRC
  ${SRC}
  )
set(FIRMWARE_TARGET_SRC
  ${FIRMWARE_TARGET_SRC}
  ../../io/crsf/crossfire.cpp
  ../../io/crsf/crsf.cpp
  ../../io/crsf/crsf_write.cpp
  ../../io/crsf/crsf_utilities.cpp
  ../../io/crsf/crc8.cpp
  )

set(HSE_VALUE 8000000)
set(SDCARD YES)
set(EEPROM SDCARD)
set(TARGET_DIR tango)
set(PPM_LIMITS_SYMETRICAL YES)

if(SPLASH STREQUAL OFF)
  message(FATAL_ERROR "Option SPLASH=OFF not supported on Tango2")
else()
  add_definitions(-DTBS_RELEASE)
endif()

add_definitions(-DAUDIO -DVOICE -DRTCLOCK)

set(GUI_SRC ${GUI_SRC}
  model_input_edit.cpp
  model_mix_edit.cpp
  model_display.cpp
  radio_diagkeys.cpp
  radio_diaganas.cpp
  radio_hardware.cpp
  view_channels.cpp
  view_telemetry.cpp
  view_about.cpp
  bmp.cpp
  ../screenshot.cpp
  )

set(TARGET_SRC
  ${TARGET_SRC}
  board.cpp
  led_driver.cpp
  backlight_driver.cpp
  ../common/arm/stm32/audio_dac_driver.cpp
  ../common/arm/stm32/adc_driver.cpp
  )


set(FIRMWARE_TARGET_SRC
  ${FIRMWARE_TARGET_SRC}
  ${LCD_DRIVER}
  pwr_driver.cpp
  ../common/arm/stm32/sdio_sd.c
  )

if(LCD_DUAL_BUFFER)
  add_definitions(-DLCD_DUAL_BUFFER)
endif()


set(STM32LIB_SRC
  STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.c
  STM32F4xx_StdPeriph_Driver/src/stm32f4xx_ltdc.c
  STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c
  STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c
  )

set(FIRMWARE_DEPENDENCIES ${FIRMWARE_DEPENDENCIES} ${BITMAPS_TARGET})
