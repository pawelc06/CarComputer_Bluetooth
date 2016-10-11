#include <stdint.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <math.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "uart.h"
#include "obd2.h"
#include "adc.h"

#include "ili9341.h"
#include "ili9341gfx.h"
#include "eeprom.h"

#define POINTCOLOUR PINK

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



/* Global Variables */
uint8_t rx_buffer[128];
uint8_t rx_buffer_index;


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
volatile char lp100km_str[10] = "0";
volatile char lp100kmAvg_str[10] = "0";
double fcBuffer[100];
uint8_t fcn=0;

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
double mpg,lp100km, maf,maf1, gph, lph, kpg,imap,ff,lp100kmAvg;

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
extern uint16_t vsetx,vsety,vactualx,vactualy,isetx,isety,iactualx,iactualy;
static FILE mydata = FDEV_SETUP_STREAM(ili9341_putchar_printf, NULL, _FDEV_SETUP_WRITE);

double calculateAvgFuelConfumption(double * buffer){
	double avgFC, sum;
	uint8_t i;

	sum=0;
	for(i=0;i<100;i++){
		sum=sum+buffer[i];
	}
	avgFC = sum/100.0;

	return avgFC;
}

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


			lp100kmAvg = readDoubleFromEEPROM(0);
			sprintf(lp100kmAvg_str, "%2.1f", lp100kmAvg);

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
			state = Trans_VSS;
		} else if (rx_data >= 0x20 && rx_data <= 0x3A) {
			rx_buffer[rx_buffer_index++] = rx_data;
		}
	}

	else if (state == Rec_VSS) {

		if (rx_data == 0x3E) {// 3E - ">" prompt
			sscanf(rx_buffer, "%X %X", &filter_41, &filter_vss);



			if ((filter_41 == 0x41) && (filter_vss == 0x0D)) {

				sscanf(rx_buffer, "%*s %*s %X", &kph);

				//kph=18;

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


				sscanf(rx_buffer, "%*s %*s %X %X", &temp_rpm1, &temp_rpm2);
				rpm = rpm_convert(temp_rpm1, temp_rpm2);
				//rpm = 1500;
				//sprintf(rpm_str, "%4d  ", rpm);

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
				temperature = temp_convert(temperature);
				sprintf(coolantTemp_str, "%3d", temperature);

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


			sscanf(rx_buffer, "%X %X", &filter_41, &filter_map);

			if ((filter_41 == 0x41) && (filter_map == 0x0B)) {

				sscanf(rx_buffer, "%*s %*s %X", &map);
				//map = 43;
				sprintf(map_str, "%3d", map);

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


			sscanf(rx_buffer, "%X %X", &filter_41, &filter_iat);

			if ((filter_41 == 0x41) && (filter_iat == 0x0F)) {

				sscanf(rx_buffer, "%*s %*s %X", &iat);
				//iat = 63;
				sprintf(iat_str, "%3d  ", iat);

				if(map >0 && iat >0){

				imap = (double)rpm*map/(iat-40+273)/2;
				maf1 = (imap/60.0)*(0.6)*1.58*28.97/8.314;
				ff = (maf1*3600.0)/(14.7*820.0);
				lp100km = ff*100.0/kph;

				if(lp100km < 100.0f)
					fcBuffer[fcn] = lp100km;
					fcn = fcn+1;
					if(fcn==100){
						lp100kmAvg = calculateAvgFuelConfumption(fcBuffer);
						saveDoubleInEEPROM(0,lp100kmAvg);
						sprintf(lp100kmAvg_str, "%2.1f", lp100kmAvg);
						fcn = 0;
					}

					if(lp100km<100)
					sprintf(lp100km_str, "%2.1f", lp100km);
					//sprintf(lp100km_str, "%3.1f  ", imap);
					//sprintf(lp100km_str, "%1.2f  ", maf1);
					//sprintf(lp100km_str, "%1.2f  ", ff);
				}

			}
			rx_buffer_index = 0;
			//state = Trans_MPG;
			state = Trans_Voltage;
		} else if (rx_data >= 0x20 && rx_data <= 0x5F) {
			rx_buffer[rx_buffer_index++] = rx_data;
		}
	}


}

void initialize(void) {

	OCR1A = 0x3D08; //1 sec

	TCCR1B |= (1 << WGM12);
	// Mode 4, CTC on OCR1A

	TIMSK1 |= (1 << OCIE1A);
	//Set interrupt on compare match

	TCCR1B |= (1 << CS12) | (1 << CS10);
	// set prescaler to 1024 and start the timer

	sei();
	// enable interrupts

	stdout = & mydata;


	state = Initial;

	kph = 0;
	mph = 0;
	dist = 0;
	dist1 = 0;

	rpm = 0;

	temperature = 0;





	sei();
}

void state_machine(void) {

	switch (state) {
	case Initial:


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



