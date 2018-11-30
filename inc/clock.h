/*
 * clock.h
 *
 *  Created on: 6 May 2018
 *      Author: gaspe
 */

#ifndef CLOCK_H_
#define CLOCK_H_

#include "stdint.h"

// Timer is set up at 125 KHz
#define FAST_CLOCK				125			// Update the clock every millisecond
#define SLOW_CLOCK				62500		// Update the clock ever half a second

void clock_timer_init();
void clock_timer_deinit();

void clock_timer_start(uint16_t timeout);

uint32_t clock_get_ms();

uint32_t clock_get_ms_since(uint32_t timestamp);

#endif /* CLOCK_H_ */
