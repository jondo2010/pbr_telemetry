/*
 * main.c
 *	Wireless Telemetry module
 *
 *  Created on: 2009-11-20
 *  Author: John Hughes <jondo2010@gmail.com>
 */

#include <usart.h>
#include <xbee.h>

void
init_lib_xbee ()
{
	usart0_init (115200);
	xbee_set_uart_read_function (&usart0_read_from_rx_buf);
	xbee_set_uart_write_function (&usart0_write_to_tx_buf);
	xbee_set_uart_flush_rx_function (&usart0_flush_rx_buf);
	xbee_set_uart_flush_tx_function (&usart0_flush_tx_buf);
	usart0_set_rx_byte_callback (&xbee_receive_byte_from_uart);

	xbee_init ();
}

void main(void)
{
	init_lib_xbee ();

	for (;;)
	{
		if ( xbee_get_driver_status () == xbee_driver_status_ready )
		{
			xbee_digest_incoming_packets ();
		}
	}
}
