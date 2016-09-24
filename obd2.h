/*
 * obd2.h
 *
 *  Created on: 10 wrz 2016
 *      Author: Pawe³
 */

#ifndef OBD2_H_
#define OBD2_H_

void state_machine(void);
void uart0_parse_rx(uint8_t rx_data);

void initialize(void);
void state_machine(void);
void debounce_machine(void);

//Update the current system in days/hours/minutes at a 1ms
void IncrementTime();

#endif /* OBD2_H_ */
