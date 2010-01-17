/*
 * main.c
 *	Wireless Telemetry module
 *
 *  Created on: 2009-11-20
 *  Author: John Hughes <jondo2010@gmail.com>
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <usart.h>
#include <xbee.h>

#include "telemetry.h"

ISR (TIMER2_OVF_vect)
{
	PORTE ^= _BV (PE4);
}

void
init_timer (void)
{
	ASSR = _BV (AS2);		/// External crystal source
	TCCR2A = _BV (CS22) | _BV (CS20);		/// Clock enabled, scale by 128 => 1 s interrupts
	TIMSK2 = _BV (TOIE2);	/// Interrupt on overflow
}

int
main (void)
{
	DDRE = _BV (PE4);
	PORTE = 0;

	//init_timer ();

	sei ();

	telemetry_init ();

	while ( xbee_get_driver_status () != xbee_driver_status_ready );

	xbee_send_data (0xffff, "READY!", 6);

	for (;;)
	{
		telemetry_process ();

		_delay_ms(50);
		PORTE ^= _BV (PE4);
	}

	return 0;
}
