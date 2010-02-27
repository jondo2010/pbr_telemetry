/*
 * xbee.c
 * Implementation for libxbee, communicates with XBee modems. Uses API mode 1, and 16-bit addressing.
 *  Created on: 2009-11-18
 *  Author: John Hughes <jondo2010@gmail.com>
 */

#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdlib.h>
#include <string.h>

#include "xbee.h"

static uint8_t xbee_frame_id = 1;
static volatile xbee_driver_status driver_status = xbee_driver_status_unknown;
static xbee_modem_mode modem_mode = xbee_modem_mode_unknown;

static xbee_api_packet packet_buf [XBEE_PACKET_BUF_LEN];
static uint8_t current_packet;
static volatile uint8_t packets_received;
static volatile uint8_t packets_digested;

/// Callback pointers
static void (*at_command_response_callback)(xbee_api_packet *);
static void (*tx_status_callback)(xbee_api_packet *);
static void (*rx_callback)(xbee_api_packet *);

/// UART function pointers
static uint8_t (*uart_write)(uint8_t *, uint8_t);
static uint8_t (*uart_read)(uint8_t *, uint8_t, int8_t);
static void (*flush_rx_buffer)(void);
static void (*flush_tx_buffer)(void);
static uint8_t (*uart_rx_bytes_free)(void);
static uint8_t (*uart_tx_bytes_free)(void);

static void (*xbee_timeout_callback) (void);

ISR (TIMER2_COMP_vect)
{
	if (xbee_timeout_callback)
		xbee_timeout_callback ();

	TIMSK2 = 0;
}

void
xbee_set_timeout
(
	void (*callback_function)(void),
	uint8_t timeout
)
{
	xbee_timeout_callback = callback_function;

	/// External crystal source
	ASSR = _BV (AS2);

	/// CTC Mode, Clock enabled, scale by 1024 => 1 second = 32 counts
	TCCR2A = _BV (WGM21) | _BV (CS22) | _BV (CS21) | _BV (CS20);
	TIMSK2 |= _BV (OCIE2A); /// Interrupt on compare match
	OCR2A = timeout;
}

void
xbee_break_timeout (void)
{
	TCCR2A &= ~(_BV (CS22) | _BV (CS21) | _BV (CS20));
	TIMSK2 &= ~_BV (OCIE2A);
}

void
xbee_set_uart_write_function
(
	uint8_t (*callback_ptr)(uint8_t *, uint8_t)
)
{
	uart_write = callback_ptr;
}

void
xbee_set_uart_read_function
(
	uint8_t (*callback_ptr)(uint8_t *, uint8_t, int8_t)
)
{
	uart_read = callback_ptr;
}

void
xbee_set_uart_flush_rx_function
(
	void (*callback_ptr)(void)
)
{
	flush_rx_buffer = callback_ptr;
}

void
xbee_set_uart_flush_tx_function
(
	void (*callback_ptr)(void)
)
{
	flush_tx_buffer = callback_ptr;
}

void
xbee_set_uart_rx_bytes_free_function
(
	uint16_t (*callback_ptr)(void)
)
{
	uart_rx_bytes_free = callback_ptr;
}

void
xbee_set_uart_tx_bytes_free_function
(
	uint16_t (*callback_ptr)(void)
)
{
	uart_tx_bytes_free = callback_ptr;
}

/**
 * Start initialisation process
 */
void
xbee_init (void)
{
	uart_write (XBEE_AT_COMMAND_SEQUENCE, strlen (XBEE_AT_COMMAND_SEQUENCE));
	driver_status = xbee_driver_status_init1;
	xbee_set_timeout (&xbee_init, 40);
}

const xbee_driver_status
xbee_get_driver_status (void)
{
	return driver_status;
}

void
xbee_set_at_command_response_callback
(
	void (*callback_ptr)(xbee_api_packet *)
)
{
	at_command_response_callback = callback_ptr;
}

void
xbee_set_tx_status_callback
(
	void (*callback_ptr)(xbee_api_packet *)
)
{
	tx_status_callback = callback_ptr;
}

void
xbee_set_rx_callback
(
	void (*callback_ptr)(xbee_api_packet *)
)
{
	rx_callback = callback_ptr;
}
/*
void
xbee_send_packet
(
	xbee_api_packet *packet
)
{
	uint8_t buf[128];
	uint8_t n, r, q;

	n = xbee_serialize_packet (packet, (uint8_t *)buf);
	//q = uart_tx_bytes_free ();
	//if (n <= q)
		r = uart_write ((uint8_t *)buf, n);
	//else
	//	r = 0;
}
*/
void
xbee_receive_byte_from_uart
(
	uint8_t byte
)
{
	static volatile xbee_rx_fsm_state_t state = waiting;
	static volatile uint8_t bytes_received = 0;

	xbee_api_packet *packet = &packet_buf [current_packet];

	if (driver_status == xbee_driver_status_init1 && byte == '\r')
	{
		xbee_break_timeout ();
		modem_mode = xbee_modem_mode_command;
		driver_status = xbee_driver_status_init2;
		uart_write ("ATAP1\n\r", 7);
		flush_rx_buffer ();
		return;
	}
	else if (driver_status == xbee_driver_status_init2 && byte == '\r')
	{
		driver_status = xbee_driver_status_init3;
		uart_write ("ATCN\n\r", 6);
		flush_rx_buffer ();
		return;
	}
	else if (driver_status == xbee_driver_status_init3 && byte == '\r')
	{
		modem_mode = xbee_modem_mode_api1;
		driver_status = xbee_driver_status_ready;
		flush_rx_buffer ();
		return;
	}
	else if (driver_status == xbee_driver_status_ready)
	{
		if (packets_received - packets_digested > XBEE_PACKET_BUF_LEN)
		{
			flush_rx_buffer ();
			return; /// Drop the packet since the buffer is full
		}

		switch (state)
		{
			case waiting:

				if (byte == XBEE_API_START_DELIM)
				{
					flush_rx_buffer ();
					state = data_length_1;
				}
				break;

			case data_length_1:

				flush_rx_buffer ();
				state = data_length_2;
				break;

			case data_length_2:

				packet->length = byte - 5;
				packet->data = (uint8_t *) malloc (sizeof(uint8_t) * packet->length);
				bytes_received = 0;
				flush_rx_buffer ();
				state = header;
				break;

			case header:

				if (++bytes_received == 5)
				{
					uart_read (packet->header, 5, 0);
					bytes_received = 0;
					flush_rx_buffer ();
					state = data;
				}
				break;

			case data:

				if (++bytes_received == packet->length)
				{
					uart_read (packet->data, packet->length, 0);
					flush_rx_buffer ();
					state = checksum;
				}
				break;

			case checksum:

				packets_received++;

				if (++current_packet == XBEE_PACKET_BUF_LEN)
					current_packet = 0;

				flush_rx_buffer ();
				state = waiting;
				break;
		}
	}
}

void
xbee_digest_incoming_packets ()
{
	while (packets_received - packets_digested)
	{
		xbee_api_packet *packet = &packet_buf [packets_digested % XBEE_PACKET_BUF_LEN];

		switch (packet->header[0])
		{
		case at_command_response:
			if (at_command_response_callback)
				at_command_response_callback (packet);
			break;
		case tx_status:
			if (tx_status_callback)
				tx_status_callback (packet);
			break;
		case rx_16:
			if (rx_callback)
				rx_callback (packet);
			break;
		case rx_64:
			/// Not implemented
			break;
		default:
			/// Dunno!
			break;
		}

		free (packet->data);
		packets_digested++;
	}
}
/*
const uint8_t
xbee_calculate_checksum
(
	xbee_api_packet *packet
)
{
	uint8_t i;
	uint16_t count = packet->api_id;

	for (i=0; i<packet->data_length; i++)
		count += packet->data [i];
	return  0xff - (count & 0xff);
}

const uint8_t
xbee_serialize_packet
(
	xbee_api_packet *packet,
	uint8_t *buf				/// Preinitialized data
)
{
	*buf++ = XBEE_API_START_DELIM;
	*buf++ = 0x00;
	*buf++ = packet->data_length + 1;
	*buf++ = packet->api_id;
	strncpy (buf, packet->data, packet->data_length);
	buf+= packet->data_length;
	*buf = xbee_calculate_checksum (packet);

	return packet->data_length+5;
}
*/
/*
xbee_api_packet *
xbee_deserialize_packet
(
	uint8_t *buf,
	uint8_t data_length			/// Detected data length
)
{
	xbee_api_packet *packet;
	uint8_t *data;

	if (*buf++ != XBEE_API_START_DELIM)
		return 0;

	data = (uint8_t *) malloc ( data_length - 1);					/// Pull data length from packet
	strncpy (data, (int8_t *) buf+3, data_length - 1);						/// Pull data from packet
	packet = xbee_create_packet ((xbee_api_command_id_t) *(buf+2), data_length - 1, data);

	return packet;
}
*/
/*
const int8_t
xbee_send_at_command
(
	uint8_t *command,	/// Always 2 bytes
	uint8_t *value,		/// Length bytes
	uint8_t length
)
{
	xbee_api_header api;
	xbee_tx_header tx;
	uint8_t *tx_ptr = (uint8_t *) &tx;
	uint16_t count;
	uint8_t checksum;
	uint8_t i;

	if (driver_status != xbee_driver_status_ready)
		return -1;

	xbee_api_packet packet;
	xbee_at_command_struct *cmd = 0;

	if (driver_status != xbee_driver_status_ready)
		return -1;

	cmd = malloc (sizeof (xbee_at_command_struct) + length);

	cmd->at_command[0] = command[0];
	cmd->at_command[1] = command[1];
	cmd->frame_id = xbee_frame_id++;
	strncpy (cmd->value, value, length);

	packet.api_id = at_command_request;
	packet.data = (uint8_t *) cmd;
	packet.data_length = length + 3;

	xbee_send_packet (&packet);

	free (cmd);

	return 1;
}
*/
const int8_t
xbee_send_data
(
	uint16_t address,
	uint8_t *data,
	uint8_t length
)
{
	xbee_api_header api;
	xbee_tx_header tx;
	uint8_t *tx_ptr = (uint8_t *) &tx;
	uint16_t count, data_length;
	uint8_t checksum;
	uint8_t i;

	if (driver_status != xbee_driver_status_ready)
		return -1;

	data_length = sizeof (xbee_tx_header) + length;

	api.api_delimiter = XBEE_API_START_DELIM;
	api.data_length_msb = (data_length & 0xff00) >> 8;
	api.data_length_lsb = (data_length & 0x00ff);

	tx.api_id = tx_16_request;
	tx.frame_id = 0x00;
	tx.destination_msb = (address & 0xff00) >> 8;
	tx.destination_lsb = (address & 0x00ff);
	tx.options = disable_ack;

	/// Calculate checksum
	count = tx_ptr[0] + tx_ptr[1] + tx_ptr[2] + tx_ptr[3] + tx_ptr[4];
	for (i = 0; i < length; i++)
		count += data[i];

	checksum = 0xff - (count & 0xff);

	/// Transmit
	uart_write ((uint8_t *) &api, sizeof(xbee_api_header));
	uart_write ((uint8_t *) &tx, sizeof (xbee_tx_header));
	uart_write (data, length);
	uart_write (&checksum, 1);

	return 1;
}
