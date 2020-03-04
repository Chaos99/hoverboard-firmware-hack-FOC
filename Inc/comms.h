/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Define to prevent recursive inclusion
#ifndef COMMS_H
#define COMMS_H

#include "stm32f1xx_hal.h"

#define SERIAL_USART_BUFFER_SIZE 1024 // TODO: implement send_wait routine..
typedef struct tag_serial_usart_buffer {
    SERIAL_USART_IT_BUFFERTYPE buff[SERIAL_USART_BUFFER_SIZE];
    int head;
    int tail;

    // count of buffer overflows
    unsigned int overflow;

} SERIAL_USART_BUFFER;

extern volatile SERIAL_USART_BUFFER usart3_it_TXbuffer;
extern volatile SERIAL_USART_BUFFER usart3_it_RXbuffer;

int   USART3_IT_starttx();
int   USART3_IT_send(unsigned char *data, int len);
void  USART3_IT_IRQ(USART_TypeDef *us);

int                        serial_usart_buffer_count(volatile SERIAL_USART_BUFFER *usart_buf);
void                       serial_usart_buffer_push (volatile SERIAL_USART_BUFFER *usart_buf, SERIAL_USART_IT_BUFFERTYPE value);
SERIAL_USART_IT_BUFFERTYPE serial_usart_buffer_pop  (volatile SERIAL_USART_BUFFER *usart_buf);



void setScopeChannel(uint8_t ch, int16_t val);
void consoleScope(void);
void consoleLog(char *message);

#endif

