#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "comms.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

//volatile char char_buf[300];
volatile SERIAL_USART_BUFFER usart2_it_TXbuffer;

#define UART_DMA_CHANNEL DMA1_Channel7

volatile uint8_t uart_buf[200];
volatile int16_t ch_buf[8];

volatile SERIAL_USART_BUFFER usart3_it_TXbuffer;
volatile SERIAL_USART_BUFFER usart3_it_RXbuffer;

void setScopeChannel(uint8_t ch, int16_t val) {
  ch_buf[ch] = val;
}

void consoleScope(void) {
    memset((void *)(uintptr_t)uart_buf, 0, sizeof(uart_buf));
    int strLength;
    strLength = sprintf((char *)(uintptr_t)uart_buf,
                "angle:%5i step:%3i sr:%5i sl:%5i adcbat:%5i vbat:%5i adctemp:%5i temp:%5i\r\n",
                ch_buf[0], ch_buf[1], ch_buf[2], ch_buf[3], ch_buf[4], ch_buf[5], ch_buf[6], ch_buf[7]);
    consoleLogLowPrio((char*)&uart_buf);
}

uint8_t dma_buffer[4000];
uint8_t wait_buffer[3900];
unsigned int buffer_position = 0;

void consoleLog(char *message) 
{
if (huart2.gState == HAL_UART_STATE_READY)
  {  
    // copy wait buffer into send buffer
    for (unsigned int i = 0; i< buffer_position; i++)
    {
      dma_buffer[i] = wait_buffer[i];
    }
    //append new message
    for (unsigned int i = 0; i<strlen(message); i++)
    {
      dma_buffer[buffer_position+i] = message[i];
    }  
    // send whole buffer
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*) dma_buffer, buffer_position+strlen(message)) ;
    //clear wait buffer
    buffer_position = 0;
  }
  else // uart busy
  {
    //copy new message to end of wait buffer
    if (buffer_position + strlen(message) < 3900)
    {
      for (unsigned int i = 0; i<strlen(message); i++)
      {
        wait_buffer[buffer_position + i] = message[i];
      }
      // update wait buffer length
      buffer_position = buffer_position + strlen(message);
    } else { //buffer full
      for (unsigned int i = 0; i<(3900/2); i++)
      {
        wait_buffer[i] = wait_buffer[i+(3900/2)]; // save last half
      }
      buffer_position = 3900/2; // clear half of the buffer (older ones gone)
      wait_buffer[3900/2 - 2] = '\r';
      wait_buffer[3900/2 - 1] = '\n';
      for (unsigned int i = 0; i<strlen(message); i++)
      {
        wait_buffer[buffer_position + i] = message[i];
      }
    }
  }
}

void consoleLogLowPrio(char *message)
{
  if (huart2.gState == HAL_UART_STATE_READY)
  {  
    // copy wait buffer into send buffer
    // for (unsigned int i = 0; i< buffer_position; i++)
    // {
    //   dma_buffer[i] = wait_buffer[i];
    // }
    //append new message
    // for (unsigned int i = 0; i<strlen(message); i++)
    // {
    //   dma_buffer[buffer_position+i] = message[i];
    // }  
    // send whole buffer
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*) message, strlen(message)) ;
    //clear wait buffer
    //buffer_position = 0;
  }
  else // uart busy
  {
    ; // drop message
  }
}

//////////////////////////////////////////////////////
// called from actual IRQ routines
void USART2_IT_IRQ(USART_TypeDef *us) {
  volatile uint32_t *SR     = &us->SR;  // USART Status register
  volatile uint32_t *DR     = &us->DR;  // USART Data register
  volatile uint32_t *CR1    = &us->CR1; // USART Control register 1

  // Transmit
  if ((*SR) & UART_FLAG_TXE) {
    if (serial_usart_buffer_count(&usart2_it_TXbuffer) == 0) {
      *CR1 = (*CR1 & ~(USART_CR1_TXEIE | USART_CR1_TCIE));
    } else {
      *DR = (serial_usart_buffer_pop(&usart2_it_TXbuffer) & 0x1ff);
    }
  }

  // Receive
  // if (((*SR) & UART_FLAG_RXNE)) {
  //   SERIAL_USART_IT_BUFFERTYPE rword = (*DR) & 0x01FF;
  //   serial_usart_buffer_push(&usart2_it_RXbuffer, rword);
  // }

  return;
}

int USART3_IT_starttx() {
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_TXE);
    return 1;
}

int USART3_IT_send(unsigned char *data, int len) {

    int count = serial_usart_buffer_count(&usart3_it_TXbuffer);
    if (count + len + 1 > SERIAL_USART_BUFFER_SIZE-3){
        usart3_it_TXbuffer.overflow++;
        return -1;
    }

    for (int i = 0; i < len; i++){
        serial_usart_buffer_push(&usart3_it_TXbuffer, (SERIAL_USART_IT_BUFFERTYPE) data[i]);
    }

    return USART3_IT_starttx();
}

//////////////////////////////////////////////////////
// called from actual IRQ routines
void USART3_IT_IRQ(USART_TypeDef *us) {
  volatile uint32_t *SR     = &us->SR;  // USART Status register
  volatile uint32_t *DR     = &us->DR;  // USART Data register
  volatile uint32_t *CR1    = &us->CR1; // USART Control register 1

  // Transmit
  if ((*SR) & UART_FLAG_TXE) {
    if (serial_usart_buffer_count(&usart3_it_TXbuffer) == 0) {
      *CR1 = (*CR1 & ~(USART_CR1_TXEIE | USART_CR1_TCIE));
    } else {
      *DR = (serial_usart_buffer_pop(&usart3_it_TXbuffer) & 0x1ff);
    }
  }

  // Receive
  if (((*SR) & UART_FLAG_RXNE)) {
    SERIAL_USART_IT_BUFFERTYPE rword = (*DR) & 0x01FF;
    serial_usart_buffer_push(&usart3_it_RXbuffer, rword);
  }

  return;
}



int serial_usart_buffer_count(volatile SERIAL_USART_BUFFER *usart_buf) {
    int count = usart_buf->head - usart_buf->tail;
    if (count < 0) count += SERIAL_USART_BUFFER_SIZE;
    return count;
}

void serial_usart_buffer_push(volatile SERIAL_USART_BUFFER *usart_buf, SERIAL_USART_IT_BUFFERTYPE value) {
    int count = serial_usart_buffer_count(usart_buf);
    if (count >=  SERIAL_USART_BUFFER_SIZE-2){
        usart_buf->overflow++;
        return;
    }

    usart_buf->buff[usart_buf->head] = value;
    usart_buf->head = ((usart_buf->head + 1 ) % SERIAL_USART_BUFFER_SIZE);
}

SERIAL_USART_IT_BUFFERTYPE serial_usart_buffer_pop(volatile SERIAL_USART_BUFFER *usart_buf) {
  SERIAL_USART_IT_BUFFERTYPE t = 0;
  if (usart_buf->head != usart_buf->tail){
      t = usart_buf->buff[usart_buf->tail];
      usart_buf->tail = ((usart_buf->tail + 1 ) % SERIAL_USART_BUFFER_SIZE);
  }
  return t;
}
