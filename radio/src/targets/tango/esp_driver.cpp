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

Fifo<uint8_t, ESP_TX_BUFFER_SIZE> espTxFifo;
Fifo<uint8_t, ESP_RX_BUFFER_SIZE> espRxFifo;

void espInit(uint32_t baudrate, bool use_dma)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  USART_DeInit(ESP_USART);

  DMA_Cmd(ESP_TX_DMA_Stream, DISABLE);

  RCC_AHB1PeriphClockCmd(ESP_RCC_AHB1Periph, ENABLE);
  RCC_APB1PeriphClockCmd(ESP_RCC_APB1Periph, ENABLE);

  GPIO_InitStructure.GPIO_Pin = ESP_EN_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(ESP_EN_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = ESP_TX_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ESP_TX_GPIO, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = ESP_RX_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(ESP_RX_GPIO, &GPIO_InitStructure);

  GPIO_PinAFConfig(ESP_TX_GPIO, ESP_TX_GPIO_PinSource, ESP_GPIO_AF);
  GPIO_PinAFConfig(ESP_RX_GPIO, ESP_RX_GPIO_PinSource, ESP_GPIO_AF);

  USART_InitStructure.USART_BaudRate = baudrate;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(ESP_USART, &USART_InitStructure);

  USART_ITConfig(ESP_USART, USART_IT_RXNE, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = ESP_USART_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_Cmd(ESP_USART, ENABLE);

  espTxFifo.clear();
  espRxFifo.clear();

  GPIO_SetBits(ESP_EN_GPIO, ESP_EN_GPIO_PIN); // open esp
}

void espOff()
{
  GPIO_ResetBits(ESP_EN_GPIO, ESP_EN_GPIO_PIN); // close esp
}

void espWriteBuffer(uint8_t* buf, uint16_t len){
  if(len == 0 || len > ESP_TX_BUFFER_SIZE){
    return;
  }

  for(int i = 0; i < len; i++){
    espTxFifo.push(buf[i]);
  }
  uint8_t data;
  if(espTxFifo.pop(data)){
    USART_SendData(ESP_USART, data);
  }
}

uint8_t espReadBuffer(uint8_t* buf){
  uint8_t size = 0;
  while(espRxFifo.pop(buf[size])){
    size++;
  }
  return size;
}

void ESP_WriteHandler(void){
  while(1){
    uint8_t data;
    if(espTxFifo.pop(data)){
      while(USART_GetFlagStatus(ESP_USART, USART_FLAG_TXE) == RESET);
      USART_SendData(ESP_USART, data);
    }
    else{
      break;
    }
  }
}

extern "C" void ESP_USART_IRQHandler(void){
  uint8_t data;
  uint32_t status = ESP_USART->SR;
  if( status & USART_SR_RXNE ){
    data = ESP_USART->DR;
    espRxFifo.push( data );
  }
  else if( status & USART_FLAG_ERRORS ){
    status = ESP_USART->DR;
    status = ESP_USART->SR;    
  }
}
