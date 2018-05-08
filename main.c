/*
 * Basic firmware to emulate a USB TEMPer device (which uses an FM75 I2C
 * sensor internally) using a Digispark + a DS18B20 1-wire sensor.
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
#include <stdlib.h>
#include <string.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <util/delay.h>

#include <avr/pgmspace.h>
#include "usbdrv.h"
#include "libs-device/osccal.h"

#include "w1.h"

typedef struct {
	uint8_t modifier;
	uint8_t reserved;
	uint8_t keycode[6];
} keyboard_report_t;
keyboard_report_t keyboard_report;

uint8_t temp_state = 0;

uint8_t temp_report[8];
bool have_temp_int = false;

#define NUM_LOCK	1
#define CAPS_LOCK	2
#define SCROLL_LOCK	4
volatile static uchar ledstate = 0xff;
static uchar idlerate;

/* We populate this with the 1-wire device ROM ID when we get it */
int serno_str[] = {
	USB_STRING_DESCRIPTOR_HEADER(16),
	'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F',
	'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F',
};

#define USB_INDEX_KEYBOARD	0
#define USB_INDEX_MOUSE		1

PROGMEM const char usbHidKeyboardReportDescriptor[] = {
	0x05, 0x01,		// USAGE_PAGE (Generic Desktop)
	0x09, 0x06,		// USAGE (Keyboard)
	0xa1, 0x01,		// COLLECTION (Application)
	0x85, 0x01,		//   REPORT_ID (1)
	0x05, 0x07,		//   USAGE_PAGE (Keyboard) (Key Codes)
	0x19, 0xe0,		//   USAGE_MINIMUM (Keyboard LeftControl)(224)
	0x29, 0xe7,		//   USAGE_MAXIMUM (Keyboard Right GUI)(231)
	0x15, 0x00,		//   LOGICAL_MINIMUM (0)
	0x25, 0x01,		//   LOGICAL_MAXIMUM (1)
	0x75, 0x01,		//   REPORT_SIZE (1)
	0x95, 0x08,		//   REPORT_COUNT (8)
	0x81, 0x02,		//   INPUT (Data,Var,Abs) ; Modifier byte
	0x95, 0x01,		//   REPORT_COUNT (1)
	0x75, 0x08,		//   REPORT_SIZE (8)
	0x81, 0x01,		//   INPUT (Cnst,Arr,Abs) ; Reserved byte
	0x95, 0x03,		//   REPORT_COUNT (3)
	0x75, 0x01,		//   REPORT_SIZE (1)
	0x05, 0x08,		//   USAGE_PAGE (LEDs)
	0x19, 0x01,		//   USAGE_MINIMUM (Num Lock)
	0x29, 0x03,		//   USAGE_MAXIMUM (Scroll Lock)
	0x91, 0x02,		//   OUTPUT (Data,Var,Abs) ; LED report
	0x95, 0x05,		//   REPORT_COUNT (5)
	0x75, 0x01,		//   REPORT_SIZE (1)
	0x91, 0x01,		//   OUTPUT (Cnst,Arr,Abs)
	0x95, 0x05,		//   REPORT_COUNT (5)
	0x75, 0x08,		//   REPORT_SIZE (8)
	0x15, 0x00,		//   LOGICAL_MINIMUM (0)
	0x25, 0xff,		//   LOGICAL_MAXIMUM (255)
	0x05, 0x07,		//   USAGE_PAGE (Keyboard)(Key Codes)
	0x19, 0x00,		//   USAGE_MINIMUM (Reserved (no event))(0)
	0x29, 0xff,		//   USAGE_MAXIMUM (Keyboard Application)(null)
	0x81, 0x00,		//   INPUT (Data,Ary,Abs)
	0xc0			// END_COLLECTION
};

PROGMEM const char usbHidMouseReportDescriptor[] = {
	0x06, 0x00, 0xff,	// USAGE_PAGE (65280)
	0x09, 0x01,		// USAGE (1)
	0xa1, 0x01,		// COLLECTION (Application)
	0x09, 0x01,		// USAGE (1)
	0x15, 0x00,		//   LOGICAL_MINIMUM (0)
	0x26, 0xff, 0x00,	//   LOGICAL_MAXIMUM (255)
	0x75, 0x08,		//   REPORT_SIZE (8)
	0x95, 0x08,		//   REPORT_COUNT (8)
	0x81, 0x02,		//   INPUT (Data,Var,Abs) ; Modifier byte
	0x09, 0x01,		// USAGE (Pointer)
	0x95, 0x08,		//   REPORT_COUNT (8)
	0x91, 0x02,		//   OUTPUT (Data,Var,Abs) ; Modifier byte
	0x05, 0x0c,		// USAGE_PAGE (Consumer)
	0x09, 0x00,		//   USAGE (0)
	0x15, 0x80,		//   LOGICAL_MINIMUM (128)
	0x25, 0x7f,		//   LOGICAL_MAXIMUM (127)
	0x75, 0x08,		//   REPORT_SIZE (8)
	0x95, 0x08,		//   REPORT_COUNT (8)
	0xb1, 0x02,		//   FEATURE (Data,Var,Abs)
	0xc0			// END_COLLECTION
};

/* USB configuration descriptor */
PROGMEM const char usbDescriptorConfiguration[] = {
	9,				/* sizeof(usbDescriptorConfiguration) */
	USBDESCR_CONFIG,		/* descriptor type */
	/* total length of data returned (including inlined descriptors) */
	18 + 9 + 9 + 7 + 9 + 7, 0,
	2,				/* number of interfaces in this cfg */
	1,				/* index of this configuration */
	0,				/* configuration name string index */
	(1 << 7) | USBATTR_REMOTEWAKE,	/* attributes */
	USB_CFG_MAX_BUS_POWER/2,	/* max USB current in 2mA units */

	/* Keyboard interface descriptor follows inline: */

	/* sizeof(usbDescrInterface): length of descriptor in bytes */
	9,
	USBDESCR_INTERFACE,	/* descriptor type */
	0,			/* index of this interface */
	0,			/* alternate setting for this interface */
	/* endpoints excl 0: number of endpoint descriptors to follow */
	1,
	3,			/* Interface class: HID */
	1,			/* SubClass: Boot Interface */
	1,			/* Interface protocol: Keyboard */
	0,			/* string index for interface */
	9,			/* sizeof(usbDescrHID) */
	USBDESCR_HID,		/* descriptor type: HID */
	0x10, 0x01,		/* BCD representation of HID version */
	0x00,			/* target country code */
	/* number of HID Report (or other HID class) Descriptors to follow */
	0x01,
	0x22,			/* descriptor type: report */
	sizeof(usbHidKeyboardReportDescriptor),
	0,		/* total length of report descriptor */

	/* endpoint descriptor for endpoint 1 */
	7,				/* sizeof(usbDescrEndpoint) */
	USBDESCR_ENDPOINT,		/* descriptor type = endpoint */
	(char)0x81,			/* IN endpoint number 1 */
	0x03,				/* attrib: Interrupt endpoint */
	8, 0,				/* maximum packet size */
	USB_CFG_INTR_POLL_INTERVAL,	/* in ms */

	/* Mouse interface descriptor follows inline: */

	/* sizeof(usbDescrInterface): length of descriptor in bytes */
	9,
	USBDESCR_INTERFACE,	/* descriptor type */
	1,			/* index of this interface */
	0,			/* alternate setting for this interface */
	/* endpoints excl 0: number of endpoint descriptors to follow */
	1,
	3,			/* Interface class: HID */
	1,			/* SubClass: Boot Interface */
	2,			/* Interface protocol: Mouse */
	0,			/* string index for interface */
	9,			/* sizeof(usbDescrHID) */
	USBDESCR_HID,		/* descriptor type: HID */
	0x10, 0x01,		/* BCD representation of HID version */
	0x00,			/* target country code */
	/* number of HID Report (or other HID class) Descriptors to follow */
	0x01,
	0x22,			/* descriptor type: report */
	/* total length of report descriptor */
	sizeof(usbHidMouseReportDescriptor), 0,

	/* endpoint descriptor for endpoint 2 */
	7,				/* sizeof(usbDescrEndpoint) */
	USBDESCR_ENDPOINT,		/* descriptor type = endpoint */
	(char)0x82,			/* IN endpoint number 2 */
	0x03,				/* attrib: Interrupt endpoint */
	8, 0,				/* maximum packet size */
	USB_CFG_INTR_POLL_INTERVAL,	/* in ms */
};

inline char hexdigit(unsigned int i)
{
	return (i < 10) ? ('0' + i) : ('A' - 10 + i);
}

/* Look for a 1-Wire device and use its ROMID to set the serial ID */
void set_serial(void)
{
	uint8_t buf[8];
	uint8_t i;

	if (!w1_reset()) {
		return;
	}

	w1_write(0x33);		/* READ ROM */
	w1_read(buf, 8);

	for (i = 0; i < 8; i++) {
		serno_str[i * 2 + 1] = hexdigit(buf[i] >> 4);
		serno_str[i * 2 + 2] = hexdigit(buf[i] & 0xF);
	}
}

uint16_t read_temp(void)
{
	uint8_t buf[9];

	cli();
	if (!w1_reset()) {
		return 0xFFFF;
		sei();
	}

	w1_write(0xCC);		/* SKIP ROM */
	w1_write(0x44);		/* Convert T */

	do {
		w1_read(buf, 1);
	} while (buf[0] != 0xFF);

	if (!w1_reset()) {
		return 0xFFFF;
		sei();
	}

	w1_write(0xCC);		/* SKIP ROM */
	w1_write(0xBE);		/* Read Scratchpad */
	w1_read(buf, 9);
	sei();

	return buf[2] << 8 | buf[1];
}

usbMsgLen_t usbFunctionDescriptor(usbRequest_t *rq)
{
	if (rq->wValue.bytes[1] == USBDESCR_STRING &&
			rq->wValue.bytes[0] == 3) {
		usbMsgPtr = (usbMsgPtr_t) serno_str;
		return sizeof(serno_str);
	} else if (rq->wValue.bytes[1] == USBDESCR_HID) {
		if (rq->wIndex.word == USB_INDEX_MOUSE) {
			/* "Mouse" */
			usbMsgPtr = (usbMsgPtr_t)
				&usbDescriptorConfiguration[43];
		} else {
			/* "Keyboard" */
			usbMsgPtr = (usbMsgPtr_t)
				&usbDescriptorConfiguration[18];
		}
	} else if (rq->wValue.bytes[1] == USBDESCR_HID_REPORT) {
		if (rq->wIndex.word == USB_INDEX_MOUSE) {
			usbMsgPtr = (usbMsgPtr_t)
				&usbHidMouseReportDescriptor[0];
			return sizeof(usbHidMouseReportDescriptor);
		} else {
			usbMsgPtr = (usbMsgPtr_t)
				&usbHidKeyboardReportDescriptor[0];
			return sizeof(usbHidKeyboardReportDescriptor);
		}
	}

	return 0;
}

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t *rq = (void *) data;

	if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {
		if (rq->bRequest == USBRQ_HID_GET_REPORT) {
			/* No keys pressed, no mouse event */
			usbMsgPtr = (void *) &keyboard_report;
			keyboard_report.modifier = 0;
			keyboard_report.reserved = 0;
			keyboard_report.keycode[0] = 0;
			return sizeof(keyboard_report);
		} else if (rq->bRequest == USBRQ_HID_SET_REPORT) {
			if (rq->wIndex.word == USB_INDEX_MOUSE) {
				return (rq->wLength.word == 8) ?
						USB_NO_MSG : 0;
			} else {
				/* Keyboard */
				return (rq->wLength.word == 1) ?
						USB_NO_MSG : 0;
			}
		} else if (rq->bRequest == USBRQ_HID_GET_IDLE) {
			usbMsgPtr = &idlerate;
			return 1;
		} else if (rq->bRequest == USBRQ_HID_SET_IDLE) {
			idlerate = rq->wValue.bytes[1];
		}
	}

	return 0;
}

usbMsgLen_t usbFunctionWrite(uint8_t * data, uchar len)
{
	if (len == 1) {
		PORTB |= 1 << PB1; // LED on
		if (data[0] != ledstate) {
			ledstate = data[0];

			if (ledstate & CAPS_LOCK) {
				PORTB |= 1 << PB1; // LED on
			} else {
				PORTB &= ~(1 << PB1); // LED off
			}
		}
		return 1;
	} else if (len == 8) {
		/* Silently consume if unexpected*/
		if (data[0] != 1)
			return 8;
		if ((data[4] | data[5] | data[6] | data[7]) != 0)
			return 8;

		if (data[1] == 0x80 && data[2] == 0x33 && data[3] == 1) {
			/* Temperature query */
			memset(temp_report, 0, 8);
			temp_state = 1;
			temp_report[0] = 0x80;
			temp_report[1] = 2;
		} else if (data[1] == 0x82 && data[2] == 0x77 &&
				data[3] == 1) {
			/* Initialisation Query #1 */
			have_temp_int = true;
			temp_report[0] = 0x82;
			temp_report[1] = 1;
			temp_report[2] = 0;
		} else if (data[1] == 0x86 && data[2] == 0xff &&
				data[3] == 1) {
			/* Initialisation Query #2 */
			have_temp_int = true;
			memcpy(temp_report, "TEMPerF1", 8);
		}

		return 8;
	}

	return 0;
}

void hadUsbReset(void)
{
	/* Reset our state machine back to having nothing to send */
	temp_state = 0;
	have_temp_int = false;
}

int main(void)
{
	unsigned char i;
	uint8_t buf[9];

	wdt_enable(WDTO_1S);

	w1_setup();
	set_serial();

	usbInit();
	usbDeviceDisconnect();

	i = 0;
	while (--i) {
		wdt_reset();
		_delay_ms(1);
	}

	usbDeviceConnect();

	/* PB1 as output for LED */
	DDRB |= 1 << PB1;

	sei(); /* We're ready to go; enable interrupts */

	keyboard_report.modifier = 0;
	keyboard_report.reserved = 0;
	keyboard_report.keycode[0] = 0;

	while (1) {
		wdt_reset();
		usbPoll();

		if (have_temp_int && usbInterruptIsReady3()) {
			usbSetInterrupt3((void *) temp_report,
					sizeof(temp_report));
			if (temp_report[0] == 'T') {
				/* Hack up second response for 0x86 query */
				memcpy(temp_report, ".2", 2);
			} else {
				have_temp_int = false;
			}
		}

		if (temp_state == 1) {
			if (w1_reset()) {
				temp_state = 2;
			} else {
				temp_report[2] = 0xFF;
				temp_report[3] = 0xFF;
				have_temp_int = true;
				temp_state = 0;
			}
		} else if (temp_state == 2) {
			w1_write(0xCC);		/* SKIP ROM */
			temp_state = 3;
		} else if (temp_state == 3) {
			w1_write(0x44);		/* Convert T */
			temp_state = 4;
		} else if (temp_state == 4) {
			if (w1_read_byte() == 0xFF)
				temp_state = 5;
		} else if (temp_state == 5) {
			if (w1_reset()) {
				temp_state = 6;
			} else {
				temp_report[2] = 0xFF;
				temp_report[3] = 0xFE;
				have_temp_int = true;
				temp_state = 0;
			}
		} else if (temp_state == 6) {
			w1_write(0xCC);		/* SKIP ROM */
			temp_state = 7;
		} else if (temp_state == 7) {
			w1_write(0xBE);		/* Read Scratchpad */
			temp_state = 8;
		} else if (temp_state > 7 && temp_state < 17) {
			buf[temp_state - 8] = w1_read_byte();
			temp_state++;
		} else if (temp_state == 17) {
			temp_report[2] = buf[1] << 4 | buf[0] >> 4;
			temp_report[3] = buf[0] << 4;
			have_temp_int = true;
			temp_state = 0;
		}
	}
}
