/*
 * clock.c
 *
 *  Created on: 6 May 2018
 *      Author: gaspe
 */


#include "clock.h"
#include <stdint.h>

#include "app_simple_timer.h"
#include "app_error.h"

static volatile uint32_t internal_ms = 0;

static uint16_t	interval_per_interrupt;		// How many milliseconds pass per interrupt
static uint8_t	initialized;				// Keeps track of whether we have initialized the timer

static void timeout_handler(void * p_context) {
	internal_ms += interval_per_interrupt;
}

void clock_timer_init() {
	if (initialized == 0) {
		uint32_t err_code = app_simple_timer_init();
		APP_ERROR_CHECK(err_code);

		initialized = 1;

		internal_ms = 0;
		clock_timer_start(FAST_CLOCK);
	}
}

void clock_timer_start(uint16_t timeout) {
	uint32_t err_code = app_simple_timer_start(APP_SIMPLE_TIMER_MODE_REPEATED, timeout_handler, timeout, NULL);
	APP_ERROR_CHECK(err_code);

	if (timeout == FAST_CLOCK) {
		interval_per_interrupt = 1;
	}
	else if (timeout == SLOW_CLOCK) {
		interval_per_interrupt = 500;
	}
}

void clock_timer_deinit() {
	uint32_t err_code = app_simple_timer_uninit();
	APP_ERROR_CHECK(err_code);

	initialized = 0;
}

uint32_t clock_get_ms() {
	return internal_ms;
}

uint32_t clock_get_ms_since(uint32_t timestamp) {
	return internal_ms - timestamp;
}

