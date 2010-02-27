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
#include <util/atomic.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <usart.h>

#include <spi.h>

#include "max3100.h"
#include "telemetry.h"
#include "xbee.h"

ISR(BADISR_vect)
{
	int i = 0;
    for (;;);
}

void
test_rx (uint8_t b)
{
	usart0_write_to_tx_buf(&b, 1);
}

int
main (void)
{
	DDRG = _BV (PG2);
	PORTG &= ~_BV (PG2);

	//init_timer ();
	//max3100_init ();

	//usart0_init (57600, 0);
	//usart0_enable_rx ();
	//usart0_enable_tx ();
	//usart0_set_rx_byte_callback (test_rx);

	//usart1_init (57600,0);
	//usart1_enable_rx();

	//_delay_ms(100);
	telemetry_init ();
	telemetry_set_daq_address (0x0001);
	telemetry_set_dta_address (0x0005);

	sei ();

	while ( xbee_get_driver_status () != xbee_driver_status_ready );
	PORTG = _BV (PG2);

/*	uint8_t buf[64];
	uint8_t n = 0, i=0;
	uint8_t str[16];

	str[0] = 0;
	str[15] = 0xff;
	for (i=1; i<15; i++)
		str[i] = n;
*/
	for (;;)
	{
		//n = usart1_read_from_rx_buf(buf, 64, 0);
		//max3100_write_to_tx_buf(buf, n);

		//n = max3100_read_from_rx_buf(buf, 64, 0);
		//usart1_write_to_tx_buf(buf, n);
		//usart0_write_to_tx_buf (buf, n);

		telemetry_process ();
/*
		while (max3100_tx_bytes_free () > 24)
		{
			//for (i=1; i<15; i++)
			//	str[i] = n;
			//n++;
			//xbee_send_data (0xffff, str, 16);

			max3100_write_to_tx_buf (str, 16);

			i = usart0_read_from_rx_buf(buf, 64, 0);
			if (i>0)
				usart0_write_to_tx_buf(buf,i);

			//_delay_ms(50);
		}

		i = 0;

		_delay_ms(50);
*/
	}
	return 0;
}
