#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include <avr/interrupt.h>

#include "uart.h"

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

extern char coolantTemp_str[10];

extern char lp100km_str[10];
extern char lp100kmAvg_str[10];
extern char lp100kmAvgLong_str[10];

volatile uint8_t refreshLCD = 0;

static FILE mydata =
FDEV_SETUP_STREAM(ili9341_putchar_printf, NULL, _FDEV_SETUP_WRITE);

void displayData(void) {

	switch (refreshLCD) {

	case 1:

		ili9341_setcursor(0, 0);
		ili9341_settextcolour(GREEN, BLACK);

		ili9341_settextsize(4);
		printf("Ak:%s", (unsigned char*) voltage_analog_str);

		refreshLCD = 2;

		break;

	case 2:

		ili9341_setcursor(0, 32);
		ili9341_settextcolour(BLUE, BLACK);

		ili9341_settextsize(4);
		printf("CC:%s", (unsigned char*) lp100km_str);

		refreshLCD = 3;
		break;

	case 3:
		ili9341_setcursor(0, 64);
		ili9341_settextcolour(RED, BLACK);

		ili9341_settextsize(4);
		printf("AC:%s", (unsigned char*) lp100kmAvg_str);

		refreshLCD = 4;
		break;
	case 4:
		ili9341_setcursor(0, 96);
		ili9341_settextcolour(ORANGE, BLACK);

		ili9341_settextsize(4);
		printf("LC:%s", (unsigned char*) lp100kmAvgLong_str);

		refreshLCD = 5;
		break;
	case 5:

		ili9341_setcursor(0, 138);
		ili9341_settextcolour(PINK, BLACK);
		//_delay_ms(2);

		ili9341_settextsize(4);
		printf("Te: %s", (unsigned char*) coolantTemp_str);

		refreshLCD = 0;
		break;

	}

}

ISR (TIMER1_COMPA_vect) {
	// action to be done every 1 sec
	refreshLCD = 1;

}

int main() {
	char data[32];
	int line = 1;

	stdout = &mydata;
	ili9341_init(); //initial driver setup to drive ili9341
	ili9341_clear(BLACK); //fill screen with black colour

	ili9341_setRotation(3); //rotate screen

	ili9341_settextcolour(GREEN, BLACK);
	ili9341_setcursor(185, 8);
	ili9341_settextsize(3);
	printf("V");

	ili9341_settextcolour(BLUE, BLACK);
	ili9341_setcursor(185, 41);
	ili9341_settextsize(3);
	printf("l/100km");

	ili9341_settextcolour(RED, BLACK);
	ili9341_setcursor(185, 75);
	ili9341_settextsize(3);
	printf("l/100km");

	ili9341_settextcolour(ORANGE, BLACK);
	ili9341_setcursor(185, 105);
	ili9341_settextsize(3);
	printf("l/100km");

	ili9341_settextcolour(PINK, BLACK);
	ili9341_setcursor(172, 145);
	ili9341_settextsize(3);
	printf(" C");

	initialize();

	//display_init();//display initial data

	InitADC();
	//LcdInit();

//uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );
	uart_init1(UART_BAUD_RATE, 1);

	while (1) {
		state_machine();
		displayData();
	}

	return 0;
}
