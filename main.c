/*-------------------------------------------------------------------------
  Example main program for LCD PCD8544 (LCD of Nokia 3310)


  Copyright (c) 2008, Fandi Gunawan <fandigunawan@gmail.com>
   http://fandigunawan.wordpress.com

   This program is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the
   Free Software Foundation; either version 2, or (__at your option) any
   later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

   In other words, you are welcome to use, share and improve this program.
   You are forbidden to forbid anyone else to use, share and improve
   what you give them.   Help stamp out software-hoarding!

-------------------------------------------------------------------------*/
/*
   Many thanks to Jakub Lasinski for reviewing and for giving 
   suggestion to my code 
*/



#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>


#include <avr/interrupt.h>


#include "uart.h"


#include "pcd8544.h"
#include "main.h"
#include <string.h>

/* Define UART buad rate here */
#define UART_BAUD_RATE      38400
//#define UART_BAUD_RATE      9600

#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "obd2.h"



int main()
{
	char data[32];
	int line=1;

	initialize();
	//LcdInit();
	        LcdClear();

	        LcdGotoXYFont(1,line++);
	        LcdFStr(FONT_1X,(unsigned char*)PSTR(">"));


LcdUpdate();

//uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
uart_init1(UART_BAUD_RATE, 1);


     while(1){
    	 state_machine();

     }

    return 0;
}
