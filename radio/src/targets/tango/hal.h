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

#ifndef _HAL_H_
#define _HAL_H_

  // Keys
  #define KEYS_GPIO_REG_MENU            GPIOD->IDR
  #define KEYS_GPIO_PIN_MENU            GPIO_Pin_13  // PD.13
  #define KEYS_GPIO_REG_EXIT            GPIOD->IDR
  #define KEYS_GPIO_PIN_EXIT            GPIO_Pin_14  // PD.14
  #define KEYS_GPIO_REG_PAGE            GPIOD->IDR
  #define KEYS_GPIO_PIN_PAGE            GPIO_Pin_12  // PD.12
  #define KEYS_GPIO_REG_ENTER           GPIOD->IDR
  #define KEYS_GPIO_PIN_ENTER           GPIO_Pin_4   // PD.4

  // Rotary Encoder
  #define ENC_GPIO                      GPIOA
  #define ENC_GPIO_PIN_A                GPIO_Pin_8    // PA.8
  #define ENC_GPIO_PIN_B                GPIO_Pin_10   // PA.10
  #define ROTARY_ENCODER_POSITION()     ((ENC_GPIO->IDR >> 9) & 0x02) + ((ENC_GPIO->IDR >> 8) & 0x01)
  #define ROTARY_ENCODER_EXTI_LINE1     EXTI_Line8
  #define ROTARY_ENCODER_EXTI_IRQn1        EXTI9_5_IRQn
  #define ROTARY_ENCODER_EXTI_IRQHandler1  EXTI9_5_IRQHandler
  #define ROTARY_ENCODER_EXTI_PortSource  EXTI_PortSourceGPIOA
  #define ROTARY_ENCODER_EXTI_PinSource1  EXTI_PinSource8

  // This is for SIMU: reuse rotary encoder pins to map UP and DOWN keyboard keys
  #if defined(SIMU)
    #define KEYS_GPIO_REG_PLUS            ENC_GPIO->IDR
    #define KEYS_GPIO_PIN_PLUS            ENC_GPIO_PIN_A
    #define KEYS_GPIO_REG_MINUS           ENC_GPIO->IDR
    #define KEYS_GPIO_PIN_MINUS           ENC_GPIO_PIN_B
  #endif

  // Trims
  #if defined(SIMU)
    #define TRIMS_GPIO_REG_LHL            GPIOG->IDR
    #define TRIMS_GPIO_PIN_LHL            GPIO_Pin_0  // PG.00
    #define TRIMS_GPIO_REG_LHR            GPIOG->IDR
    #define TRIMS_GPIO_PIN_LHR            GPIO_Pin_1  // PG.01
    #define TRIMS_GPIO_REG_LVD            GPIOG->IDR
    #define TRIMS_GPIO_PIN_LVD            GPIO_Pin_2  // PG.02
    #define TRIMS_GPIO_REG_LVU            GPIOG->IDR
    #define TRIMS_GPIO_PIN_LVU            GPIO_Pin_3  // PG.03
    #define TRIMS_GPIO_REG_RVD            GPIOG->IDR
    #define TRIMS_GPIO_PIN_RVD            GPIO_Pin_4  // PG.04
    #define TRIMS_GPIO_REG_RHL            GPIOG->IDR
    #define TRIMS_GPIO_PIN_RHL            GPIO_Pin_5  // PG.05
    #define TRIMS_GPIO_REG_RVU            GPIOG->IDR
    #define TRIMS_GPIO_PIN_RVU            GPIO_Pin_6  // PG.06
    #define TRIMS_GPIO_REG_RHR            GPIOG->IDR
    #define TRIMS_GPIO_PIN_RHR            GPIO_Pin_7  // PG.07
  #endif

  // Switches
  #define SWITCHES_GPIO_REG_A           GPIOC->IDR
  #define SWITCHES_GPIO_PIN_A           GPIO_Pin_4  // PC.04
  #define SWITCHES_GPIO_REG_B_H         GPIOA->IDR
  #define SWITCHES_GPIO_PIN_B_H         GPIO_Pin_5  // PA.05
  #define SWITCHES_GPIO_REG_B_L         GPIOA->IDR
  #define SWITCHES_GPIO_PIN_B_L         GPIO_Pin_6  // PA.06
  #define SWITCHES_GPIO_REG_C_H         GPIOE->IDR
  #define SWITCHES_GPIO_PIN_C_H         GPIO_Pin_3  // PE.03
  #define SWITCHES_GPIO_REG_C_L         GPIOE->IDR
  #define SWITCHES_GPIO_PIN_C_L         GPIO_Pin_2  // PE.02
  #define SWITCHES_GPIO_REG_D           GPIOE->IDR
  #define SWITCHES_GPIO_PIN_D           GPIO_Pin_0  // PE.00
  #define SWITCHES_GPIO_REG_E           GPIOE->IDR
  #define SWITCHES_GPIO_PIN_E           GPIO_Pin_1  // PE.01
  #define SWITCHES_GPIO_REG_F           GPIOA->IDR
  #define SWITCHES_GPIO_PIN_F           GPIO_Pin_7  // PA.07

  #define KEYS_RCC_AHB1Periph           (RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOF|RCC_AHB1Periph_GPIOG)
  #define KEYS_GPIOA_PINS               (SWITCHES_GPIO_PIN_F | SWITCHES_GPIO_PIN_B_H | SWITCHES_GPIO_PIN_B_L | ENC_GPIO_PIN_A | ENC_GPIO_PIN_B)
  #define KEYS_GPIOC_PINS               SWITCHES_GPIO_PIN_A
  #define KEYS_GPIOD_PINS               (KEYS_GPIO_PIN_ENTER | KEYS_GPIO_PIN_MENU | KEYS_GPIO_PIN_PAGE | KEYS_GPIO_PIN_EXIT)
  #define KEYS_GPIOE_PINS               (SWITCHES_GPIO_PIN_C_H | SWITCHES_GPIO_PIN_C_L | SWITCHES_GPIO_PIN_D | SWITCHES_GPIO_PIN_E)

  // ADC
  #define ADC_MAIN                      ADC1
  #define ADC_DMA                       DMA2
  #define ADC_DMA_SxCR_CHSEL            0
  #define ADC_DMA_Stream                DMA2_Stream4
  #define ADC_SET_DMA_FLAGS()           ADC_DMA->HIFCR = (DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4)
  #define ADC_TRANSFER_COMPLETE()       (ADC_DMA->HISR & DMA_HISR_TCIF4)
  #define ADC_SAMPTIME                  2   // sample time = 28 cycles
  #define ADC_CHANNEL_RTC               ADC_Channel_18 // ADC1_IN18
  #define ADC_RCC_AHB1Periph            (RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2)
  #define ADC_RCC_APB1Periph            0
  #define ADC_RCC_APB2Periph            RCC_APB2Periph_ADC1
  #define ADC_GPIO_PIN_BATT             GPIO_Pin_1  // PB.01
  #define ADC_GPIOB_PINS                ADC_GPIO_PIN_BATT
  #define ADC_CHANNEL_BATT              ADC_Channel_9  // ADC1_IN9

  // PWR driver
  #define PWR_RCC_AHB1Periph            (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE)
  #define PWR_SWITCH_GPIO               GPIOB
  #define PWR_SWITCH_GPIO_PIN           GPIO_Pin_14 // PB.14
  #define PWR_ON_GPIO                   GPIOB
  #define PWR_ON_GPIO_PIN               GPIO_Pin_12 // PB.12

  // Charger
  #define CHARGER_STATE_GPIO            GPIOD
  #define CHARGER_STATE_GPIO_PIN        GPIO_Pin_10 // PD.10
  #define CHARGER_FAULT_GPIO            GPIOD
  #define CHARGER_FAULT_GPIO_PIN        GPIO_Pin_11 // PD.10

  // Internal Module
  #define INTMODULE_PULSES
  #define INTMODULE_RCC_AHB1Periph      0
  #define INTMODULE_RCC_APB1Periph      0
  #define INTMODULE_RCC_APB2Periph      0
  #define INTMODULE_PWR_GPIO            0
  #define INTMODULE_PWR_GPIO_PIN        0 

  // External Module
  #define EXTERNAL_MODULE_ON()
  #define EXTERNAL_MODULE_OFF()
  #define IS_EXTERNAL_MODULE_ON()         (true)

  // Trainer Port
  #define TRAINER_RCC_AHB1Periph          0
  #define TRAINER_RCC_APB1Periph          0

  // Led
  #define CHARGING_LEDS
  #define LED_RCC_AHB1Periph              (RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1)
  #define LED_RCC_APB1Periph              RCC_APB1Periph_TIM2
  #define LED_GPIO                        GPIOB
  #define LED_GPIO_PIN                    GPIO_Pin_11  // PB.11
  #define LED_GPIO_PinSource              GPIO_PinSource11
  #define LED_TIMER                       TIM2
  #define LED_GPIO_AF                     GPIO_AF_TIM2
  #define LED_DMA_STREAM                  DMA1_Stream6
  #define LED_DMA_CHANNEL                 DMA_Channel_3
  #define LED_DMA_FLAG_TC                 DMA_FLAG_TCIF6
  #define LED_DMA_FLAG_ERRORS             (DMA_FLAG_FEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_HTIF6)

  // Serial Port
  #define SERIAL_RCC_AHB1Periph           (RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1)
  #define SERIAL_RCC_APB1Periph           RCC_APB1Periph_USART3
  #define SERIAL_GPIO                     GPIOD
  #define SERIAL_GPIO_PIN_TX              GPIO_Pin_8 // PD.08
  #define SERIAL_GPIO_PIN_RX              GPIO_Pin_9 // PD.09
  #define SERIAL_GPIO_PinSource_TX        GPIO_PinSource8
  #define SERIAL_GPIO_PinSource_RX        GPIO_PinSource9
  #define SERIAL_GPIO_AF                  GPIO_AF_USART3
  #define SERIAL_USART                    USART3
  #define SERIAL_USART_IRQHandler         USART3_IRQHandler
  #define SERIAL_USART_IRQn               USART3_IRQn
  #define SERIAL_DMA_Stream_RX            DMA1_Stream1
  #define SERIAL_DMA_Channel_RX           DMA_Channel_4

  // Telemetry
  #define TELEMETRY_RCC_AHB1Periph        0
  #define TELEMETRY_RCC_APB1Periph        0

  // Heartbeat
  #define HEARTBEAT_RCC_AHB1Periph        0
  #define HEARTBEAT_RCC_APB2Periph        0
  #define HEARTBEAT_DMA_Stream            0

  // USB
  #define USB_RCC_AHB1Periph_GPIO         RCC_AHB1Periph_GPIOA
  #define USB_GPIO                        GPIOA
  #define USB_GPIO_PIN_VBUS               GPIO_Pin_9  // PA.09
  #define USB_GPIO_PIN_DM                 GPIO_Pin_11 // PA.11
  #define USB_GPIO_PIN_DP                 GPIO_Pin_12 // PA.12
  #define USB_GPIO_PinSource_DM           GPIO_PinSource11
  #define USB_GPIO_PinSource_DP           GPIO_PinSource12
  #define USB_GPIO_AF                     GPIO_AF_OTG1_FS

  // BackLight
  #define BACKLIGHT_RCC_AHB1Periph        0
  #define BACKLIGHT_RCC_APB1Periph        0
  #define BACKLIGHT_RCC_APB2Periph        0

  // LCD driver
  #define LCD_RCC_AHB1Periph              (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA1)
  #define LCD_RCC_APB1Periph              RCC_APB1Periph_SPI3
  #define LCD_SPI_GPIO                    GPIOB
  #define LCD_MOSI_GPIO_PIN               GPIO_Pin_5 // PB.5
  #define LCD_MOSI_GPIO_PinSource         GPIO_PinSource5
  #define LCD_CLK_GPIO_PIN                GPIO_Pin_3 // PB.3
  #define LCD_CLK_GPIO_PinSource          GPIO_PinSource3
  #define LCD_NCS_GPIO                    GPIOD
  #define LCD_NCS_GPIO_PIN                GPIO_Pin_1 // PD.1
  #define LCD_RST_GPIO                    GPIOD
  #define LCD_RST_GPIO_PIN                GPIO_Pin_3 // PD.3
  #define LCD_DC_GPIO                     GPIOD
  #define LCD_DC_GPIO_PIN                 GPIO_Pin_6 // PD.6
  #define LCD_DMA                         DMA1
  #define LCD_DMA_Stream                  DMA1_Stream7
  #define LCD_DMA_Stream_IRQn             DMA1_Stream7_IRQn
  #define LCD_DMA_Stream_IRQHandler       DMA1_Stream7_IRQHandler
  #define LCD_DMA_FLAGS                   (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7)
  #define LCD_DMA_FLAG_INT                DMA_HIFCR_CTCIF7
  #define LCD_SPI                         SPI3
  #define LCD_GPIO_AF                     GPIO_AF_SPI3
  #define LCD_RCC_APB2Periph              0

  // I2C
  #define I2C_RCC_APB1Periph              0
  #define I2C_RCC_AHB1Periph              0

  // SD
  #define SD_RCC_AHB1Periph               (RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA2)
  #define SD_RCC_APB1Periph               0
  #define SD_PRESENT_GPIO                 GPIOC
  #define SD_PRESENT_GPIO_PIN             GPIO_Pin_5  // PC.05
  #define SD_SDIO_DMA_STREAM              DMA2_Stream3
  #define SD_SDIO_DMA_CHANNEL             DMA_Channel_4
  #define SD_SDIO_DMA_FLAG_FEIF           DMA_FLAG_FEIF3
  #define SD_SDIO_DMA_FLAG_DMEIF          DMA_FLAG_DMEIF3
  #define SD_SDIO_DMA_FLAG_TEIF           DMA_FLAG_TEIF3
  #define SD_SDIO_DMA_FLAG_HTIF           DMA_FLAG_HTIF3
  #define SD_SDIO_DMA_FLAG_TCIF           DMA_FLAG_TCIF3
  #define SD_SDIO_DMA_IRQn                DMA2_Stream3_IRQn
  #define SD_SDIO_DMA_IRQHANDLER          DMA2_Stream3_IRQHandler
  #define SD_SDIO_FIFO_ADDRESS            ((uint32_t)0x40012C80)
  #define SD_SDIO_CLK_DIV(fq)             ((48000000 / (fq)) - 2)
  #define SD_SDIO_INIT_CLK_DIV            SD_SDIO_CLK_DIV(400000)
  #define SD_SDIO_TRANSFER_CLK_DIV        SD_SDIO_CLK_DIV(24000000)

  // Audio
  #define AUDIO_RCC_AHB1Periph            (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1)
  #define AUDIO_RCC_APB1Periph            (RCC_APB1Periph_TIM6 | RCC_APB1Periph_DAC)
  #define AUDIO_OUTPUT_GPIO               GPIOA
  #define AUDIO_OUTPUT_GPIO_PIN           GPIO_Pin_4  // PA.04
  #define AUDIO_DMA_Stream                DMA1_Stream5
  #define AUDIO_DMA_Stream_IRQn           DMA1_Stream5_IRQn
  #define AUDIO_TIM_IRQn                  TIM6_DAC_IRQn
  #define AUDIO_TIM_IRQHandler            TIM6_DAC_IRQHandler
  #define AUDIO_DMA_Stream_IRQHandler     DMA1_Stream5_IRQHandler
  #define AUDIO_TIMER                     TIM6
  #define AUDIO_DMA                       DMA1
  #define AUDIO_MUTE_GPIO                 GPIOD
  #define AUDIO_MUTE_GPIO_PIN             GPIO_Pin_5  // PD.05

  // Haptic
  #define HAPTIC_RCC_AHB1Periph           RCC_AHB1Periph_GPIOB
  #define HAPTIC_RCC_APB2Periph           0
  #define HAPTIC_RCC_APB1Periph           0
  #define HAPTIC_GPIO                     GPIOB
  #define HAPTIC_GPIO_PIN                 GPIO_Pin_0  // PB.00

  // ESP
  #if defined(ESP_SERIAL)
    #define ESP_RCC_AHB1Periph            (RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1)
    #define ESP_RCC_APB1Periph            RCC_APB1Periph_UART4
    #define ESP_EN_GPIO                   GPIOA
    #define ESP_EN_GPIO_PIN               GPIO_Pin_3 // PA.03
    #define ESP_TX_GPIO                   GPIOA
    #define ESP_TX_GPIO_PIN               GPIO_Pin_0 // PA.00
    #define ESP_RX_GPIO                   GPIOA
    #define ESP_RX_GPIO_PIN               GPIO_Pin_1 // PA.01
    #define ESP_TX_GPIO_PinSource         GPIO_PinSource0
    #define ESP_RX_GPIO_PinSource         GPIO_PinSource1
    #define ESP_GPIO_AF                   GPIO_AF_UART4
    #define ESP_USART                     UART4
    #define ESP_USART_IRQHandler          UART4_IRQHandler
    #define ESP_USART_IRQn                UART4_IRQn
    #define ESP_TX_DMA_Stream             DMA1_Stream4
    #define ESP_TX_DMA_Channel            DMA_Channel_4
    #define ESP_RX_DMA_Stream             DMA1_Stream2
    #define ESP_RX_DMA_Channel            DMA_Channel_4
    #define ESP_TX_DMA_STREAM_IRQHandler  DMA1_Stream4_IRQHandler
    #define ESP_TX_DMA_IRQn               DMA1_Stream4_IRQn
    #define ESP_TX_DMA_IT_TC              DMA_IT_TCIF4
    #define ESP_RX_DMA_STREAM_IRQHandler  DMA1_Stream2_IRQHandler
    #define ESP_RX_DMA_IRQn               DMA1_Stream2_IRQn
    #define ESP_RX_DMA_IT_TC              DMA_IT_TCIF2
  #endif

  // Xms Interrupt
  #define INTERRUPT_xMS_RCC_APB1Periph    RCC_APB1Periph_TIM14
  #define INTERRUPT_xMS_TIMER             TIM14
  #define INTERRUPT_xMS_IRQn              TIM8_TRG_COM_TIM14_IRQn
  #define INTERRUPT_xMS_IRQHandler        TIM8_TRG_COM_TIM14_IRQHandler

  // 2MHz Timer
  #define TIMER_2MHz_RCC_APB1Periph       RCC_APB1Periph_TIM7
  #define TIMER_2MHz_TIMER                TIM7

  // 1ms Interrupt
  #define INTERRUPT_1MS_RCC_APB1Periph    RCC_APB2Periph_TIM11
  #define INTERRUPT_1MS_TIMER             TIM11
  #define INTERRUPT_1MS_IRQn              TIM1_TRG_COM_TIM11_IRQn
  #define INTERRUPT_1MS_IRQHandler        TIM1_TRG_COM_TIM11_IRQHandler
  #define INTERRUPT_1MS_IRQ_PRI           1

#endif // _HAL_H_
