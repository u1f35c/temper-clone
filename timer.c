/*
 * Timer1 functions for ATTiny85
 *
 * Heavily based on code from Ardunio:
 * https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring.c
 *
 * Copyright (c) 2005-2006 David A. Mellis
 * Copyright 2018 Jonathan McDowell <noodles@earth.li>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdbool.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include <avr/pgmspace.h>

#include "timer.h"

#define clockCyclesPerMicrosecond() (F_CPU / 1000000L)
/* Prescaler of 64 from CPU_CLK */
#define MICROSECONDS_PER_TIMER1_OVERFLOW (64 * 256 / clockCyclesPerMicrosecond())
#define MILLIS_INC (MICROSECONDS_PER_TIMER1_OVERFLOW / 1000)
#define FRACT_INC ((MICROSECONDS_PER_TIMER1_OVERFLOW % 1000) >> 3)
#define FRACT_MAX (1000 >> 3)

volatile unsigned long timer1_overflow_count = 0;
volatile unsigned long timer1_millis = 0;
static unsigned char timer1_fract = 0;

void timer_init(void)
{
	/*
	 * bit 7:   0  (CTC1: Disabled CTC)
	 * bit 6/5: 00 (COM1A1/COM1A0: Disconnect counter from OC1A output)
	 * bit 3-0: 0111 (CS13/CS12/CS11/CS10: CPU_CK/64)
	 */
	TCCR1 = (1 << CS12) | (1 << CS11) | (1 << CS10);

	/* Initialise counter */
	TCNT1 = 0;

	/* Disable Timer 1 output compare match interrupt */
	TIMSK &= ~(OCIE1A | OCIE1B);
	/* Enable Timer 1 overflow interrupt */
	TIMSK |= _BV(TOIE1);
}

unsigned long micros(void)
{
	unsigned long m;
	uint8_t oldSREG = SREG, t;

	cli();
	m = timer1_overflow_count;
	t = TCNT1;

	if ((TIFR & _BV(TOV1)) & (t < 255))
		m++;

	SREG = oldSREG;

	return ((m << 8) + t) * (64 / clockCyclesPerMicrosecond());
}

unsigned long millis(void)
{
	unsigned long m;
	uint8_t oldSREG = SREG;

	cli();
	m = timer1_millis;
	SREG = oldSREG;

	return m;
}

SIGNAL(TIMER1_OVF_vect)
{
	unsigned long m = timer1_millis;
	unsigned char f = timer1_fract;

	m += MILLIS_INC;
	f += FRACT_INC;
	if (f >= FRACT_MAX) {
		f -= FRACT_MAX;
		m++;
	}

	timer1_fract = f;
	timer1_millis = m;
	timer1_overflow_count++;
}
