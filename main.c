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
#include "adc.h"

extern char voltage_str[10];
extern char voltage_analog_str[10];
extern char velocity[10];
extern char rpm_str[10];
extern char coolantTemp_str[10];
extern char map_str[10];
extern char iat_str[10];
extern char lp100km_str[10];
extern char lp100kmAvg_str[10];

volatile uint8_t refreshLCD=0;

void displayData (void){



	switch (refreshLCD) {

	case 1:
		LcdGotoXYFont(5, 1);

		LcdStr(FONT_1X, (unsigned char*) velocity);
		LcdStr(FONT_1X, (unsigned char*) "km/h");
		refreshLCD = 2;
		break;

	case 2:
		LcdGotoXYFont(5, 2);
		LcdStr(FONT_1X, (unsigned char*) rpm_str);
		refreshLCD = 3;
		break;

	case 3:


		LcdGotoXYFont(5, 3);
		LcdStr(FONT_1X, (unsigned char*) voltage_analog_str);
		LcdStr(FONT_1X, (unsigned char*) " V ");
		/*
		LcdGotoXYFont(5, 4);
		LcdStr(FONT_1X, (unsigned char*) voltage_str);
		LcdStr(FONT_1X, (unsigned char*) " V ");
		*/
		refreshLCD = 4;

		break;

	case 4:
			LcdGotoXYFont(5, 4);
			LcdStr(FONT_1X, (unsigned char*) map_str);
			//LcdStr(FONT_1X, (unsigned char*) "C");
			refreshLCD = 5;
			break;

	case 5:
			LcdGotoXYFont(5, 5);
			//LcdStr(FONT_1X, (unsigned char*) iat_str);
			LcdStr(FONT_1X, (unsigned char*) lp100kmAvg_str);
			//LcdStr(FONT_1X, (unsigned char*) "C");
			refreshLCD = 6;
			break;


	case 6:
		/*
		LcdGotoXYFont(6, 6);
		LcdStr(FONT_1X, (unsigned char*) coolantTemp_str);
		LcdStr(FONT_1X, (unsigned char*) "C");
		*/
		LcdGotoXYFont(5, 6);
		LcdStr(FONT_1X, (unsigned char*) lp100km_str);
		//LcdStr(FONT_1X, (unsigned char*) "l/100km");

		refreshLCD = 0;
		break;



	}
	if(refreshLCD>0)
		LcdUpdate();
}



ISR (TIMER1_COMPA_vect)
{
    // action to be done every 1 sec
	refreshLCD=1;

}

int main()
{
	char data[32];
	int line=1;

	initialize();
	InitADC();
	//LcdInit();




//uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
uart_init1(UART_BAUD_RATE, 1);


     while(1){
    	 state_machine();
    	 displayData();
     }

    return 0;
}
