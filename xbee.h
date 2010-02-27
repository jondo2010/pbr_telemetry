/*
 * xbee.h
 *
 * Header for libxbee, communicates with XBee modems. Uses API mode 1, and 16-bit addressing.
 * Created on: 2009-11-18
 * Author: John Hughes <jondo2010@gmail.com>
 */

#ifndef XBEE_H_
#define XBEE_H_

#include <inttypes.h>

#define XBEE_API_START_DELIM			0x7e

typedef enum XBEE_DRIVER_STATUS
{
	xbee_driver_status_unknown,
	xbee_driver_status_init1,
	xbee_driver_status_init2,
	xbee_driver_status_init3,
	xbee_driver_status_ready,
	xbee_driver_status_choked
}
xbee_driver_status;

typedef enum XBEE_MODEM_MODE
{
	xbee_modem_mode_unknown,
	xbee_modem_mode_command,
	xbee_modem_mode_api1,
	xbee_modem_mode_api2
}
xbee_modem_mode;

/// API Identifiers
typedef enum XBEE_API_COMMAND_ID_T
{
	at_command_request	= 0x08,
	at_command_response = 0x88,
	tx_64_request		= 0x00,			/// 64-bit addressing transmit request
	tx_16_request		= 0x01,			/// 16-bit addressing transmit request
	tx_status			= 0x89,			/// Transmit status message
	rx_64				= 0x80,
	rx_16				= 0x81,
}
xbee_api_command_id_t;

/// Abstract packet type
typedef struct XBEE_API_HEADER
{
	uint8_t		api_delimiter;
	uint8_t		data_length_msb;
	uint8_t		data_length_lsb;
}
xbee_api_header;

/// Concrete command struct type
typedef struct XBEE_AT_COMMAND_HEADER
{
	uint8_t	frame_id;
	uint8_t	at_command[2];
	uint8_t value[1];			/// Variable length
}
xbee_at_command_header;

/// Response status types for AT command response packets
typedef enum XBEE_AT_COMMAND_RESPONSE_STATUS_T
{
	ok					= 0x00,
	error				= 0x01,
	invalid_command		= 0x02,
	invalid_parameter	= 0x03
}
xbee_at_command_response_status_t;

typedef struct XBEE_AT_COMMAND_RESPONSE_STRUCT
{
	uint8_t								frame_id;
	uint8_t								at_command[2];
	xbee_at_command_response_status_t	status;
	uint8_t								value[1];
}
xbee_at_command_response_struct;

/// Transmit options. Mutually exclusive.
typedef enum XBEE_TX_OPTION_T
{
	disable_ack		= 0x01,
	pan_broadcast	= 0x04
}
xbee_tx_option_t;

typedef struct XBEE_TX_HEADER
{
	xbee_api_command_id_t		api_id;
	uint8_t						frame_id;
	uint8_t						destination_msb;	/// 16-bit destination address
	uint8_t						destination_lsb;
	xbee_tx_option_t			options;
}
xbee_tx_header;

/// Transmit status types for TX status packets
typedef enum XBEE_TX_STATUS_T
{
	success = 0x00,
	no_ack = 0x01,
	cca_fail = 0x02,
	purged = 0x03
}
xbee_tx_status_t;

typedef struct XBEE_TX_STATUS_STRUCT
{
	uint8_t				frame_id;
	xbee_tx_status_t	status;
}
xbee_tx_status_struct;

/// Receive options defines
#define XBEE_RX_ADDRESS_BROADCAST	0x02
#define XBEE_RX_PAN_BROADCAST		0x04

typedef struct XBEE_RX_HEADER
{
	xbee_api_command_id_t	api_id;
	uint8_t					source_msb;	/// 16-bit source address
	uint8_t					source_lsb;
	uint8_t					rssi;		/// Receive signal strength
	uint8_t					options;
}
xbee_rx_header;

typedef struct XBEE_API_PACKET
{
	uint8_t		header[5];				/// 5 byte header must be cast
	uint8_t		length;					/// length of data portion of packet
	uint8_t		*data;
}
xbee_api_packet;

/// Select AT commands
#define XBEE_AT_COMMAND_SEQUENCE	"+++"
#define XBEE_AT_API					"AP"
#define XBEE_AT_CHANNEL				"CH"
#define XBEE_AT_PAN_ID				"ID"

#define XBEE_PACKET_BUF_LEN			32

typedef enum XBEE_RX_FSM_STATE_T
{
	waiting,
	data_length_1,
	data_length_2,
	header,
	data,
	checksum
}
xbee_rx_fsm_state_t;

void
xbee_set_uart_write_function
(
	uint8_t (*callback_ptr)(uint8_t *, uint8_t)
);

void
xbee_set_uart_read_function
(
	uint8_t (*callback_ptr)(uint8_t *, uint8_t, int8_t)
);

void
xbee_set_uart_flush_rx_function
(
	void (*callback_ptr)(void)
);

void
xbee_set_uart_flush_tx_function
(
	void (*callback_ptr)(void)
);

void
xbee_set_uart_rx_bytes_free_function
(
	uint16_t (*callback_ptr)(void)
);

void
xbee_set_uart_tx_bytes_free_function
(
	uint16_t (*callback_ptr)(void)
);

void
xbee_init ();

const xbee_driver_status
xbee_get_driver_status ();

/**
 * Set the callback to be called when an incoming AT command response packet is digested
 */
void
xbee_set_at_command_response_callback
(
	void (*callback_ptr)(xbee_api_packet *)
);

/**
 * Set the callback to be called when an incoming TX status packet is digested
 */
void
xbee_set_tx_status_callback
(
	void (*callback_ptr)(xbee_api_packet *)
);

/**
 * Set the callback to be called when an incoming data packet is digested
 */
void
xbee_set_rx_callback
(
	void (*callback_ptr)(xbee_api_packet *)
);

/**
 * Send a packet
 */
void
xbee_send_packet
(
	xbee_api_packet *packet
);

/**
 * Process a received packet after it's been deserialized
 */
void
xbee_receive_packet
(
	xbee_api_packet *packet
);

/**
 * Receive a byte from the XBee
 * Set up as a callback for the uart code to call.
 */
void
xbee_receive_byte_from_uart
(
	uint8_t byte
);

/**
 * Digest all packets in the incoming packet buffer. Calls the packet handler callbacks.
 */
void
xbee_digest_incoming_packets ();

/**
 * Fill preinitialized buffer *buf with packet data.
 * Returns the length of the serialized data in bytes.
 */
const uint8_t
xbee_serialize_packet
(
	xbee_api_packet *packet,
	uint8_t *buf				/// Preinitialized data
);

/**
 * Deserialize a received packet into a packet structure
 */
/*
xbee_api_packet *
xbee_deserialize_packet
(
	uint8_t *buf,
	uint8_t data_length			/// Detected data length
);
*/

const int8_t
xbee_send_at_command
(
	uint8_t command[2],
	uint8_t *value,
	uint8_t length
);

const int8_t
xbee_send_data
(
	uint16_t address,
	uint8_t *data,
	uint8_t length
);

#endif /* XBEE_H_ */
