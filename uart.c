/*************************************************************************
Title:    Interrupt UART library with receive/transmit circular buffers
Author:   Peter Fleury <pfleury@gmx.ch>   http://tinyurl.com/peterfleury
File:     $Id: uart.c,v 1.15.2.4 2015/09/05 18:33:32 peter Exp $
Software: AVR-GCC 4.x
Hardware: any AVR with built-in UART, 
License:  GNU General Public License 
          
DESCRIPTION:
    An interrupt is generated when the UART has finished transmitting or
    receiving a byte. The interrupt handling routines use circular buffers
    for buffering received and transmitted data.
    
    The UART_RX_BUFFER_SIZE and UART_TX_BUFFER_SIZE variables define
    the buffer size in bytes. Note that these variables must be a 
    power of 2.
    
USAGE:
    Refere to the header file uart.h for a description of the routines. 
    See also example test_uart.c.

NOTES:
    Based on Atmel Application Note AVR306
                    
LICENSE:
    Copyright (C) 2015 Peter Fleury, GNU General Public License Version 3

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
                        
*************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "uart.h"




/*****************************************************/
void uart_init1(uint32_t baudrate, uint8_t double_speed) {
    uint16_t ubrr = 0;
    if (double_speed) {
        UCSR0A = _BV(U2X0);  //Enable 2x speed
        ubrr = (F_CPU / (8UL * baudrate)) - 1;
    } else {
        ubrr = (F_CPU / (16UL * baudrate)) - 1;
    }
    UBRR0H = ubrr >> 8;
    UBRR0L = ubrr;

    UCSR0C &= ~(_BV(UMSEL01) | _BV(UMSEL00)); // enable asynchronous USART
    UCSR0C &= ~(_BV(UPM01) | _BV(UPM00)); // disable parity mode
    UCSR0C &= ~_BV(USBS0); // set 1-bit stop
    UCSR0C &= ~_BV(UCSZ02);
    UCSR0C |= _BV(UCSZ01) | _BV(UCSZ00); // set 8-bit data

    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   // Enable RX and TX
}


uint8_t uart_getchar() {
    loop_until_bit_is_set(UCSR0A, RXC0);
    return UDR0;
}

void uart_read_line(uint8_t *value, uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        value[i] = uart_getchar();
        if (value[i] == '\r') {
            value[i] = '\0';
            break;
        }
    }
}

void uart_putchar(const uint8_t data) {
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = data;
}


void uart_print(const char *value) {
    while (*value != '\0') {
        uart_putchar(*value++);
    }
}

void uart_nprint(const char *value, unsigned int n) {
    unsigned int i=0;
	while (i<n) {
        uart_putchar(*value++);
        i++;
    }
}
/*****************************************************************/

