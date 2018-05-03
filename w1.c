/*
 * Basic routines to bit-bang standard 1-Wire via a GPIO pin
 *
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
#include <avr/io.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "w1.h"

uint8_t w1_crc(uint8_t *buf, uint8_t len)
{
	uint8_t i, j, crc;

	crc = 0;
	for (i = 0; i < len; i++)
	{
		crc ^= buf[i];
		for (j = 0; j < 8; j++)
		{
			crc = crc >> 1 ^ ((crc & 1) ? 0x8c : 0);
		}
	}

	return crc;
}

void w1_write(uint8_t val)
{
	uint8_t i;

	for (i = 0; i < 8; i++) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			/* Pull low for 6µs for 1, 60µs for 0 */
			DDRB |= 1 << W1_PIN;
			if (val & 1)
				_delay_us(6);
			else
				_delay_us(60);
			/* Release to make up to 70µs total */
			DDRB &= ~(1 << W1_PIN);
			if (val & 1)
				_delay_us(64);
			else
				_delay_us(10);
		}

		val >>= 1;
	}
}

uint8_t w1_read_byte()
{
	uint8_t i, val;

	val = 0;
	for (i = 0; i < 8; i++) {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			/* Pull low for 6µs */
			DDRB |= 1 << W1_PIN;
			_delay_us(6);
			/* Release for 9µs */
			DDRB &= ~(1 << W1_PIN);
			_delay_us(9);

			/* Read the line state */
			val |= ((PINB >> W1_PIN) & 1) << i;
		}

		_delay_us(55);
	}

	return val;
}

void w1_read(uint8_t *buf, uint8_t len)
{
	uint8_t i;

	for (i = 0; i < len; i++) {
		buf[i] = w1_read_byte();
	}
}

bool w1_reset(void)
{
	bool present;

	/* Pull low for 480µs */
	DDRB |= 1 << W1_PIN;
	_delay_us(480);
	/* Release for 70µs */
	DDRB &= ~(1 << W1_PIN);
	_delay_us(70);

	/* If there's a device present it'll have pulled the line low */
	present = !((PINB >> W1_PIN) & 1);

	/* Wait for reset to complete */
	_delay_us(410);

	return present;
}

void w1_setup(void)
{
	/* Set 1w pin to low */
	PORTB &= (1 << W1_PIN);
}
