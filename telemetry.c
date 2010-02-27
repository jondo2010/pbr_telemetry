/*
 * telemetry.c
 *
 *  Created on: 2009-12-20
 *  Author: John Hughes <jondo2010@gmail.com>
 */

#include <stdlib.h>
#include <string.h>
#include <util/delay.h>

#include <usart.h>

#include "telemetry.h"
#include "dac.h"
#include "max3100.h"
#include "xbee.h"

static uint16_t dta_node_address;
static uint16_t dac_node_address;

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

	p->length;
}

void
telemetry_xbee_rx_callback
(
	xbee_api_packet *p
)
{
	xbee_rx_header *header = (xbee_rx_header *) p->header;

	// Send RSSI message out over CAN
	//response->rssi;

	if (header->source_msb == (dta_node_address & 0xff00) >> 8 &&
			header->source_lsb == (dta_node_address & 0x00ff))
	{
		usart0_write_to_tx_buf (p->data, p->length);
		/// Pass data on to DTA UART
	}
	else if (header->source_lsb == dac_node_address)
	{
		/// Possibly implement some interface here
	}
	else
	{
		/// Someone unknown is sending a weird message.
	}

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
	uint8_t analog[4];
	uint8_t n = encodeAnalogueData(analog, dacData->timeStamp, 9);
	xbee_send_data (dac_node_address, analog, n);
/*
	char msg[64];
	if (channel == TIME_STAMP)
	{
	//	snprintf(msg, 64, "Time Stamp: %lu\r\n", dacData->timeStamp);
	//	xbee_send_data(0xffff, msg, strlen(msg));
	}
	else if (channel >= ANALOGUE_FIRST && channel <= ANALOGUE_LAST)
	{
		//uint8_t cn = getUserAnalogueChannel(channel - ANALOGUE_FIRST);
		//snprintf(msg, 64, "C = %d, V = %d mv\r\n", cn, dacData->analogueVoltage[cn - 1] );
		//usart0_write_to_tx_buf (msg, strlen(msg));
		//xbee_send_data(0xffff, msg, strlen(msg));
	}
*/
	xbee_send_data (dac_node_address, data, length);
	//usart0_write_to_tx_buf(data, length);
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
	max3100_init ();

	xbee_set_uart_tx_bytes_free_function (max3100_tx_bytes_free);
	xbee_set_uart_flush_rx_function (max3100_flush_rx_buf);
	xbee_set_uart_flush_tx_function (max3100_flush_tx_buf);
	xbee_set_uart_read_function (max3100_read_from_rx_buf);
	xbee_set_uart_write_function (max3100_write_to_tx_buf);

	max3100_set_rx_byte_callback (xbee_receive_byte_from_uart);
	max3100_set_rx_overrun_callback (rx_overrun_callback);

	xbee_set_at_command_response_callback (telemetry_xbee_command_response_callback);
	xbee_set_rx_callback (telemetry_xbee_rx_callback);

	xbee_init ();

	dac_init_buffer ();
	dac_set_decode_callback (telemetry_dac_channel_decoded_callback);

	/// DAC is on usart1
	usart1_init (38400, 0);
	usart1_set_rx_byte_callback (&telemetry_dac_rx_callback);
	usart1_enable_rx ();

	/// ECU is on usart0
	usart0_init (57600, 1);
	usart0_enable_rx ();
	usart0_enable_tx ();
}

void
telemetry_process (void)
{
	static uint8_t dta_buf[64];

	uint8_t n = usart0_rx_bytes_ready ();
	if (n>0)
	{
		n = usart0_read_from_rx_buf (dta_buf, 64, 0);
		xbee_send_data (dta_node_address, (uint8_t *)dta_buf, n);
	}

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
	dac_node_address = address;
}

uint16_t
telemetry_get_daq_address (void)
{
	return dac_node_address;
}

void
telemetry_set_dta_interface_enabled (uint8_t b);

void
telemetry_set_daq_interface_enabled (uint8_t b);
