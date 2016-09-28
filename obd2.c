#include <stdint.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "pcd8544.h"
#include "uart.h"
#include "obd2.h"
#include "adc.h"

//State Machine States
#define Initial 1
#define Trans_EchoOff 2
#define Rec_EchoOff 3
#define Trans_Reset 4
#define Rec_Reset 5
#define Trans_VSS 6
#define Rec_VSS 7
#define Trans_MPG 8
#define Rec_MPG	9
#define Trans_Protocol 10
#define Rec_Protocol 11
#define Trans_RPM 12
#define Rec_RPM	13
#define Trans_Coolant 14
#define Rec_Coolant	15
#define Save_sd	16
#define Trans_atsp	17
#define Rec_atsp 18
#define Trans_Voltage 19
#define Rec_Voltage 20
#define Trans_MAP 21
#define Rec_MAP	22
#define Trans_IAT 23
#define Rec_IAT	24
#define N_DECIMAL_POINTS_PRECISION (100) // n = 3. Three decimal points.

//Data conversion
#define airflowrate_convert(c,b)	floor((256*(c))+(b))*0.01;
#define speed_convert(c)			floor(c);
#define rpm_convert(c,b)			floor(((256*(c))+(b))/4);
#define temp_convert(c)				floor((c)-40); 
#define lph_convert(c)				floor((c)*3.7854); 

//Macro timers

#define t2 50//increment system time ms
#define t3 25 //debouncer
#define t4 100 //LCD timer

//State machine state names
#define NoPush 1
#define MaybePush 2
#define Pushed 3
#define MaybeNoPush 4

#define countmax 3

unsigned char PushFlag;		//message indicating a button push
unsigned char PushState;	//state machine

//Time Keeping
volatile unsigned int realTimer, buttonTimer, lcdTimer;

/* Global Variables */
uint8_t rx_buffer[128];
uint8_t rx_buffer_index;

uint8_t lcd_buffer[17];	// LCD display buffer
//enum {MCU_STATE_IDLE, MCU_STATE_TX};
//uint8_t error_buffer[40];

//LCD Messages
const int8_t LCD_product[] PROGMEM = "Komputer sam.";
const int8_t LCD_name[] PROGMEM = "    by Pawel    ";
const int8_t LCD_VSS[] PROGMEM = "km/h\0";
const int8_t LCD_RPM[] PROGMEM = "RPM\0";
const int8_t LCD_Temp[] PROGMEM = "Temp\0";
const int8_t LCD_Dist[] PROGMEM = "Dis\0";
const int8_t LCD_F[] PROGMEM = " C\0";
const int8_t LCD_empty[] PROGMEM = " \0";

//PID Meassages

uint8_t Reset[4] = "ATZ\r";
uint8_t EchoOff[5] = "ATE0\r";
uint8_t ATL[5] = "ATL1\r";
uint8_t VSS[6] = "01 0D\r";
uint8_t MAF[6] = "01 10\r";
uint8_t MAP[6] = "01 0B\r";
uint8_t IAT[6] = "01 0F\r";
uint8_t VOL[6] = "AT RV\r";
uint8_t RPM[6] = "01 0C\r";
uint8_t TEMP[6] = "01 05\r";
uint8_t FUEL[6] = "01 2F\r";
uint8_t ATSP[9] = "AT SP A4\r";

volatile char velocity[10] = "0";
char maf_str[10];
volatile char coolantTemp_str[10] = "  0";
volatile char rpm_str[10];
volatile char voltage_str[10] = "0.0";
volatile char voltage_analog_str[10] = "0";
volatile char map_str[10] = "0";
volatile char iat_str[10] = "0";
volatile char mpg1_str[10] = "0";

//void uart0_send_byte(uint8_t data);

//State Machine Marker
int state;

//Car data
int kph, temperature, rpm, map, iat;
int sec;
int temp_sec;
volatile unsigned int mph;
volatile float dist1;
volatile unsigned int dist;
double mpg,lp100km, maf,maf1, gph, lph, kpg;

int temp, decimalPart;

uint8_t filter_vss, filter_41, filter_maf, filter_rpm, filter_temp, filter_map,
		filter_iat;

unsigned int temp_maf1, temp_maf2;
unsigned int temp_rpm1, temp_rpm2;

//Number of times the file has been written to

unsigned char flag, wFlag, smaf[10];

//time
volatile unsigned int millis;
volatile unsigned int secs;
volatile unsigned int mins;
volatile unsigned int hours;
volatile unsigned int day;
volatile unsigned int month;
volatile unsigned int year;

/******************************************************************************************/
// /**
/*
 ISR (TIMER1_COMPA_vect) {
 //Decrement the time if not already zero
 if (realTimer > 0)
 --realTimer;
 if (realTimer == 0) {
 realTimer = t2;
 IncrementTime();
 }
 if (buttonTimer > 0)
 --buttonTimer;
 if (lcdTimer > 0)
 --lcdTimer;

 }
 */

//Parsing function based on current state
void uart0_parse_rx(uint8_t rx_data) {

	uint16_t vAdc;
	float vAdcf;

	if (state == Rec_EchoOff) {
		if (rx_data == 0x3E) {
			rx_buffer_index = 0;
			state = Trans_VSS;
		} else if (rx_data >= 0x20 && rx_data <= 0x5F) {
			rx_buffer[rx_buffer_index++] = rx_data;
			//state = Trans_Protocol;
		}
	} else if (state == Rec_Protocol) {
		if (rx_data == 0x3E) {
			rx_buffer_index = 0;
			state = Trans_EchoOff;
		} else if (rx_data >= 0x20 && rx_data <= 0x5F) {
			rx_buffer[rx_buffer_index++] = rx_data;
		}
	} else if (state == Rec_Reset) {
		if (rx_data == 0x3E) {
			rx_buffer_index = 0;
			state = Trans_atsp;
		} else if (rx_data >= 0x20 && rx_data <= 0x5F) {
			rx_buffer[rx_buffer_index++] = rx_data;
		}
	} else if (state == Rec_atsp) {
		if (rx_data == 0x3E) {
			rx_buffer_index = 0;
			state = Trans_Protocol;
		} else if (rx_data >= 0x20 && rx_data <= 0x5F) {
			rx_buffer[rx_buffer_index++] = rx_data;
		}
	} else if (state == Rec_Voltage) {
		if (rx_data == 0x3E) {
			//display voltage
			rx_buffer[rx_buffer_index] = 0;
			strcpy(voltage_str, rx_buffer);

			vAdc = ReadADC(0);
			vAdcf = (float) vAdc / 63.0;
			//vAdc = 123;
			sprintf(voltage_analog_str, "%2.1f", vAdcf);

			rx_buffer_index = 0;
			state = Trans_MPG;
		} else if (rx_data >= 0x20 && rx_data <= 0x3A) {
			rx_buffer[rx_buffer_index++] = rx_data;
		}
	}

	else if (state == Rec_VSS) {

		if (rx_data == 0x3E) {// 3E es el simbolo < que prmptea el elm327 en codigo ascii
			sscanf(rx_buffer, "%X %X", &filter_41, &filter_vss);

			sscanf(rx_buffer, "%*s %*s %X %X", &temp_maf1, &temp_maf2);

			if ((filter_41 == 0x41) && (filter_vss == 0x0D)) {

				sscanf(rx_buffer, "%*s %*s %X", &kph);

				sprintf(velocity, "%3d  ", kph);

			}

			rx_buffer_index = 0;
			state = Trans_RPM;
		} else if (rx_data >= 0x20 && rx_data <= 0x5F) {
			rx_buffer[rx_buffer_index++] = rx_data;
		}

	} else if (state == Rec_RPM) {

		if (rx_data == 0x3E) {
			sscanf(rx_buffer, "%X %X", &filter_41, &filter_rpm);
			//fprintf(stdout,"%X %X\n\r", filter_41, filter_vss);

			if ((filter_41 == 0x41) && (filter_rpm == 0x0C)) {
				LcdGotoXYFont(5, 2);

				sscanf(rx_buffer, "%*s %*s %X %X", &temp_rpm1, &temp_rpm2);
				rpm = rpm_convert(temp_rpm1, temp_rpm2)
				;
				sprintf(rpm_str, "%4d  ", rpm);

			}
			rx_buffer_index = 0;
			state = Trans_Coolant;
		} else if (rx_data >= 0x20 && rx_data <= 0x5F) {
			rx_buffer[rx_buffer_index++] = rx_data;
		}
	}

	else if (state == Rec_Coolant) {

		if (rx_data == 0x3E) {

			_delay_ms(30); //was 50
			sscanf(rx_buffer, "%X %X", &filter_41, &filter_temp);

			if ((filter_41 == 0x41) && (filter_temp == 0x05)) {

				sscanf(rx_buffer, "%*s %*s %X", &temperature);
				temperature = temp_convert(temperature)
				;
				sprintf(coolantTemp_str, "%3d  ", temperature);

			}
			rx_buffer_index = 0;
			//state = Trans_MPG;
			state = Trans_MAP;
		} else if (rx_data >= 0x20 && rx_data <= 0x5F) {
			rx_buffer[rx_buffer_index++] = rx_data;
		}
	}

	else if (state == Rec_MAP) {

		if (rx_data == 0x3E) {

			_delay_ms(30); //was 50
			sscanf(rx_buffer, "%X %X", &filter_41, &filter_map);

			if ((filter_41 == 0x41) && (filter_map == 0x0B)) {

				sscanf(rx_buffer, "%*s %*s %X", &map);

				sprintf(map_str, "%3d  ", map);

			}
			rx_buffer_index = 0;
			//state = Trans_MPG;
			state = Trans_IAT;
		} else if (rx_data >= 0x20 && rx_data <= 0x5F) {
			rx_buffer[rx_buffer_index++] = rx_data;
		}
	}

	else if (state == Rec_IAT) {

		if (rx_data == 0x3E) {

			_delay_ms(30); //was 50
			sscanf(rx_buffer, "%X %X", &filter_41, &filter_iat);

			if ((filter_41 == 0x41) && (filter_iat == 0x0F)) {

				sscanf(rx_buffer, "%*s %*s %X", &iat);

				sprintf(iat_str, "%3d  ", iat);

				if(map >0 && iat >0){
				maf1 =  14.7 * rpm * map / iat;

				lp100km = kph * 7.718/maf1; //miles per gallon

				lp100km = kph * 7.718/(maf1*0.621317); //km per gallon

				lp100km =  (maf1*0.621317)/(kph *7.718); //gallons per km

				lp100km =  (maf1*0.621317)*3.785411784/(kph *7.718); //litres per km

				lp100km = (maf1*0.621317)*3.785411784*100/(kph *7.718); //litres per 100 km

				if(lp100km < 100.0f)
					sprintf(mpg1_str, "%2.2f  ", lp100km);
				}

			}
			rx_buffer_index = 0;
			//state = Trans_MPG;
			state = Trans_Voltage;
		} else if (rx_data >= 0x20 && rx_data <= 0x5F) {
			rx_buffer[rx_buffer_index++] = rx_data;
		}
	}

	else if (state == Rec_MPG) {

		if (rx_data == 0x3E) {

			sscanf(rx_buffer, "%X %X", &filter_41, &filter_maf);
			//fprintf(stdout,"%X %X\n\r", filter_41, filter_vss);

			if ((filter_41 == 0x41) && (filter_maf == 0x10)) {

				//LcdStr(FONT_1X,(unsigned char*)"MPG: ");
				/*
				 *
				 * Fuel Flow = Air Flow / Stoichiometric Ratio.

				 For a gasoline (petrol) engine, the stoichiometric ratio is 14.7.
				 The air flow is obtained by a MAF (Mass Air Flow) sensor, and it is available over OBD2.

				 If there is no MAF sensor, then the MAP (Manifold Absolute Pressure) sensor can be used:

				 Air Flow = C x RPM x MAP / Absolute Temperature.
				 MPG =VSS * 7.718/MAF
				 */

				sscanf(rx_buffer, "%*s %*s %X %X", &temp_maf1, &temp_maf2);
				maf = airflowrate_convert(temp_maf1, temp_maf2);
				gph = (maf * 0.0805);
				lph = lph_convert(gph);
				kpg = (double) (kph / gph);

				temp = (int) (kpg);
				decimalPart = ((int) (kpg * N_DECIMAL_POINTS_PRECISION)
						% N_DECIMAL_POINTS_PRECISION);

			}
			rx_buffer_index = 0;
			state = Trans_VSS;
		} else if (rx_data >= 0x20 && rx_data <= 0x5F) {
			rx_buffer[rx_buffer_index++] = rx_data;

		}
	}
}

void initialize(void) {
	/*
	 TIMSK1 = 2;		//turn on timer 0 cmp match ISR
	 OCR1A = 249;  	//set the compare reg to 250 time ticks
	 TCCR1A = 0b00000010; // turn on clear-on-match
	 TCCR1B = 0b00000011;	// clock prescalar to 64

	 //FRESULT res;
	 OCR0A = 0xB3; // avrcalc says that at 3.6864MHz that /8 and CTC 0xB3 will give 10ms
	 TIMSK0 = (1 << OCIE0A); // use COMP interrupt
	 TCCR0A = (1 << WGM01) | (1 << CS01); // CTC with div 8

	 DDRD = 0b00001010;	// PORT D is an input except for TX and LED
	 PORTD =0b11111100;	// PORT D pullup MAKE SURE THATS OK FOR UART
	 */
	OCR1A = 0x3D08; //1 sec

	TCCR1B |= (1 << WGM12);
	// Mode 4, CTC on OCR1A

	TIMSK1 |= (1 << OCIE1A);
	//Set interrupt on compare match

	TCCR1B |= (1 << CS12) | (1 << CS10);
	// set prescaler to 1024 and start the timer

	sei();
	// enable interrupts

	LcdInit();
	LcdClear();

	state = Initial;

	kph = 0;
	mph = 0;
	dist = 0;
	dist1 = 0;

	rpm = 0;

	temperature = 0;

	PushFlag = 0;
	PushState = NoPush;

	//init the task timer

	realTimer = t2;
	buttonTimer = t3;
	lcdTimer = t4;

	//init time
	millis = 0;
	secs = 0;
	sec = 0;
	mins = 0;
	hours = 0;
	day = 22; //dummy day
	month = 4; //dummy month
	year = 20; //dummy year

	temp_sec = sec;

	//ADMUX = (1 << ADLAR) | 0b01000010;
	//ADCSRA = (1<<ADEN) | (1<<ADSC) + 7 ;
	_delay_ms(500);

	sei();
}
/******************************************************************************************/
void state_machine(void) {

	switch (state) {
	case Initial:

		//CopyStringtoLCD(LCD_product, 0, 0);
		//CopyStringtoLCD(LCD_name, 0, 1);

		LcdGotoXYFont(1, 1);
		LcdFStr(FONT_1X, (unsigned char*) LCD_product);

		LcdGotoXYFont(1, 2);
		LcdFStr(FONT_1X, (unsigned char*) LCD_name);
		LcdUpdate();

		_delay_ms(800);

		LcdClear();
		LcdGotoXYFont(1, 1);
		LcdStr(FONT_1X, (unsigned char*) "V: ");

		LcdGotoXYFont(1, 2);
		LcdStr(FONT_1X, (unsigned char*) "RPM: ");

		LcdGotoXYFont(1, 3);
		LcdStr(FONT_1X, (unsigned char*) "AD:");

		LcdGotoXYFont(1, 4);
		LcdStr(FONT_1X, (unsigned char*) "AO:");

		LcdGotoXYFont(1, 5);
		LcdStr(FONT_1X, (unsigned char*) "MAP: ");

		LcdGotoXYFont(1, 6);
		LcdStr(FONT_1X, (unsigned char*) "IAT: ");

		/*
		LcdGotoXYFont(1, 6);
		LcdStr(FONT_1X, (unsigned char*) "FC:");
*/
		LcdUpdate();
		state = Trans_Reset;

		break;

	case Trans_atsp:
		//uart0_send_byte_array(ATSP,9);
		uart_nprint(ATSP, 9);
		_delay_ms(500);
		state = Rec_atsp;
		break;

	case Rec_atsp:
		if (UCSR0A & (1 << RXC0)) {
			uart0_parse_rx(UDR0);

		}

		break;

	case Trans_EchoOff:
		//uart0_send_byte_array(EchoOff,5);
		uart_nprint(EchoOff, 5);
		_delay_ms(500);
		state = Rec_EchoOff;
		break;

	case Rec_EchoOff:
		if (UCSR0A & (1 << RXC0)) {
			uart0_parse_rx(UDR0);

		}

		break;

	case Trans_Protocol:
		uart_nprint(ATL, 5);
		_delay_ms(500);
		//UBRR0L = 25;
		state = Rec_Protocol;
		break;

	case Rec_Protocol:
		if (UCSR0A & (1 << RXC0)) {
			uart0_parse_rx(UDR0);

		}
		//state = Trans_VSS;// esto no estaba aca solo lo puse para ver la salida uart.

		break;

	case Trans_Reset:
		uart_nprint(Reset, 4);
		temp_sec = sec;
		state = Rec_Reset;

		break;

	case Rec_Reset:
		if (UCSR0A & (1 << RXC0)) {
			uart0_parse_rx(UDR0);

		}

		break;

	case Trans_VSS:

		uart_nprint(VSS, 6);

		state = Rec_VSS;
		break;

	case Rec_VSS:
		if (UCSR0A & (1 << RXC0)) {
			uart0_parse_rx(UDR0);

		}

		break;

	case Trans_MPG:
		uart_nprint(MAF, 6);

		state = Rec_MPG;
		break;

	case Rec_MPG:
		if (UCSR0A & (1 << RXC0)) {

			uart0_parse_rx(UDR0);

		}
		break;

	case Trans_Voltage:
		uart_nprint(VOL, 6);

		state = Rec_Voltage;
		break;

	case Rec_Voltage:
		if (UCSR0A & (1 << RXC0)) {
			uart0_parse_rx(UDR0);

		}
		break;

	case Trans_RPM:

		uart_nprint(RPM, 6);

		state = Rec_RPM;
		break;

	case Rec_RPM:
		if (UCSR0A & (1 << RXC0)) {
			uart0_parse_rx(UDR0);

		}
		break;

	case Trans_MAP:

			uart_nprint(MAP, 6);

			state = Rec_MAP;
			break;

		case Rec_MAP:
			if (UCSR0A & (1 << RXC0)) {
				uart0_parse_rx(UDR0);

			}
			break;

		case Trans_IAT:

				uart_nprint(IAT, 6);

				state = Rec_IAT;
				break;

			case Rec_IAT:
				if (UCSR0A & (1 << RXC0)) {
					uart0_parse_rx(UDR0);

				}
				break;

	case Trans_Coolant:
		uart_nprint(TEMP, 6);

		state = Rec_Coolant;
		break;

	case Rec_Coolant:
		if (UCSR0A & (1 << RXC0)) {
			uart0_parse_rx(UDR0);

		}
		break;

	}
}
/*
 void init_lcd(void) {
 LCDinit();	//initialize the display
 LCDcursorOFF();
 LCDclr();				//clear the display
 }
 */

//*******************************   
//Button Debouncer  
/*
 void debouncer(void)
 {
 switch (PushState)
 {
 case NoPush:
 if (~PIND & 0x04) PushState=MaybePush; //0x04 = 0b00000100 0b11111011
 else PushState=NoPush;
 break;
 case MaybePush:
 if (~PIND & 0x04)
 {
 PushState=Pushed;
 PushFlag=1;
 }
 else PushState=NoPush;
 break;
 case Pushed:
 if (~PIND & 0x04) PushState=Pushed;
 else PushState=MaybeNoPush;
 break;
 case MaybeNoPush:
 if (~PIND & 0x04) PushState=Pushed;
 else
 {
 PushState=NoPush;
 PushFlag=0;
 }
 break;
 }
 }

 */

void IncrementTime() {
	millis++;
	if (millis == 10) {
		dist1 = (dist1 + (((float) mph * 1000.0) / 3600.0));// ACA SE HACE LA DEDUCCION DE LA ODOMETRIA.
		dist = speed_convert(dist1)
		;
		secs++;
		sec++;
		millis = 0;
		if (secs == 60) {
			//Toggle the led state
			mins++;
			secs = 0;
			if (mins == 60) {
				hours++;
				mins = 0;
				if (hours == 24) {
					hours = 0;
				}
			}
		}
	}
}

