/*
 * telemetry.c
 *
 *  Created on: 2009-12-20
 *  Author: John Hughes <jondo2010@gmail.com>
 */

#include <stdlib.h>
#include <string.h>

#include <usart.h>
#include <xbee.h>

#include "telemetry.h"
#include "dac.h"

static uint16_t dta_node_address = 0;
static uint16_t daq_node_address = 0;

void rx_overrun_callback ()
{
	dta_node_address = 0;
}

void
telemetry_xbee_command_response_callback
(
	xbee_api_packet *p
)
{
	xbee_at_command_response_struct *response =
			(xbee_at_command_response_struct *) p->data;

	p->data_length;
}

void
telemetry_xbee_rx_callback
(
	xbee_api_packet *p
)
{
	xbee_rx_struct *response = (xbee_rx_struct *) p->data;

	// Send RSSI message out over CAN
	//response->rssi;

	if (response->source == dta_node_address)
	{
		/// Pass data on to DTA UART
	}
	else if (response->source == daq_node_address)
	{
		/// Possibly implement some interface here
	}
	else
	{
		/// Someone unknown is sending a weird message.
	}

	xbee_send_data (0xffff, response->data, p->data_length - sizeof (xbee_rx_struct) + 1);

	//uint8_t n;
	//for (n = 0; n < (p->data_length - sizeof (xbee_rx_struct) + 1); n++)
	//	dac_decode_data (*(response->data + n), &dbuffer, &dacData, telemetry_dac_channel_decoded_callback);
}

void
telemetry_dac_channel_decoded_callback
(
	uint8_t data[],
	int length,
	DACData* dacData,
	enum DAC_Channel channel
)
{
	char msg[64];

	if (channel == TIME_STAMP)
	{
	//	snprintf(msg, 64, "Time Stamp: %lu\r\n", dacData->timeStamp);
	//	xbee_send_data(0xffff, msg, strlen(msg));
	}
	else if (channel >= ANALOGUE_FIRST && channel <= ANALOGUE_LAST)
	{
		uint8_t cn = getUserAnalogueChannel(channel - ANALOGUE_FIRST);
		snprintf(msg, 64, "Channel = %d, Voltage = %d mv\r\n", cn, dacData->analogueVoltage[cn - 1] );
		xbee_send_data(0xffff, msg, strlen(msg));
	}
}

void
telemetry_dac_rx_callback
(
	uint8_t byte
)
{
	dac_buffer_serial_data (byte);
}

void
telemetry_init ()
{
	usart0_init (57600);

	xbee_set_uart_tx_bytes_free_function (&usart0_tx_bytes_free);
	xbee_set_uart_flush_rx_function (&usart0_flush_rx_buf);
	xbee_set_uart_flush_tx_function (&usart0_flush_tx_buf);
	xbee_set_uart_read_function (&usart0_read_from_rx_buf);
	xbee_set_uart_write_function (&usart0_write_to_tx_buf);

	usart0_set_rx_byte_callback (&xbee_receive_byte_from_uart);
	usart0_set_rx_overrun_callback (&rx_overrun_callback);

	xbee_set_at_command_response_callback (&telemetry_xbee_command_response_callback);
	xbee_set_rx_callback (&telemetry_xbee_rx_callback);

	usart0_enable_rx ();
	usart0_enable_tx ();

	xbee_init ();

	dac_init_buffer ();
	dac_set_decode_callback (&telemetry_dac_channel_decoded_callback);

	usart1_init (57600);
	usart1_set_rx_byte_callback (&telemetry_dac_rx_callback);
	usart1_enable_rx ();
}

void
telemetry_process (void)
{
	xbee_digest_incoming_packets ();
	dac_decode_data ();
}

void
telemetry_set_dta_address (uint16_t address)
{
	dta_node_address = address;
}

uint16_t
telemetry_get_dta_address (void)
{
	return dta_node_address;
}

void
telemetry_set_daq_address (uint16_t address)
{
	daq_node_address = address;
}

uint16_t
telemetry_get_daq_address (void)
{
	return daq_node_address;
}

void
telemetry_set_dta_interface_enabled (uint8_t b);

void
telemetry_set_daq_interface_enabled (uint8_t b);
