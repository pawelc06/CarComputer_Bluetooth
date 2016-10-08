/*
 * adc.h
 *
 *  Created on: 28 wrz 2016
 *      Author: Pawe³
 */

#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>

void InitADC();
uint16_t ReadADC(uint8_t ch);

#endif /* ADC_H_ */
