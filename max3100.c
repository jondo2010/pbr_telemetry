/*
 * max3100.c -	Interfaces with MAX3100 SPI USART. Software interface and
 * 				circular buffering code adapted from Mike Jean's USART driver.
 *
 *  Created on: 2010-02-03
 *  Author: John Hughes <jondo2010@gmail.com>
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include <spi.h>

#include "max3100.h"

// RX buffer (circular)

static 			uint8_t		rx_buf[MAX3100_RX_BUF_LEN];		// buffer ptr
static volatile	uint8_t		*rx_start = rx_buf;		// start of valid data ptr
static volatile	uint8_t 	*rx_end = rx_buf;		// end of valid data ptr

static volatile uint8_t 	rx_read_count;		// total elements read
static volatile uint8_t		rx_write_count;		// total elements written

// TX buffer (circular)

static 			uint8_t		tx_buf[MAX3100_TX_BUF_LEN];		// buffer ptr
static volatile	uint8_t		*tx_start = tx_buf;	// start of valid data ptr
static volatile	uint8_t 	*tx_end = tx_buf;		// end of valid data ptr

static volatile uint8_t 	tx_read_count;		// total elements read
static volatile uint8_t		tx_write_count;		// total elements written

static volatile uint8_t		tx_running;

// Callback function pointers

static void (*rx_byte_callback)			(uint8_t byte);
static void	(*rx_newline_callback)		(void);
static void	(*rx_error_callback)		(void);
static void	(*rx_full_callback)			(void);
static void	(*rx_overrun_callback)		(void);
static void	(*tx_complete_callback)		(void);

/**
 * External interrupt from max3100 usart
 */
ISR (INT7_vect)
{
	/// Disable INT7 and then re-enable global interrupts
	EIMSK &= ~_BV (INT7);
	sei ();

	/// Read the chips' config to determine what caused the interrupt
	spi_slave_select (0);
	uint16_t conf_word = spi_putword (READ_CONFIG);
	spi_slave_deselect (0);

	if (conf_word & 0x4000) /// Transmit buffer is empty
	{
		if (tx_write_count - tx_read_count)
		{
			spi_slave_select (0);
			spi_putword (WRITE_DATA | *(tx_start++));
			spi_slave_deselect (0);

			if (tx_start == tx_buf + MAX3100_TX_BUF_LEN)
				tx_start = tx_buf;

			tx_read_count++;

			/*if (!(tx_write_count - tx_read_count))
			{
				if (tx_complete_callback)
					tx_complete_callback();
			}*/
		}
		else if (!(conf_word & 0x8000)) /// The TX Buf empty flag needs to be cleared manually since there's no data to be read
		{
			spi_slave_select (0);
			spi_putword (READ_DATA);
			spi_slave_deselect (0);

			tx_running = 0;
		}
	}

	if (conf_word & 0x8000)		/// Data in input FIFO
	{
		spi_slave_select (0);
		uint8_t data = spi_putword (READ_DATA) & 0xff;
		spi_slave_deselect (0);

		int8_t	buf_full 	= rx_write_count - rx_read_count == MAX3100_RX_BUF_LEN - 1;
		int8_t 	buf_overrun = rx_write_count - rx_read_count == MAX3100_RX_BUF_LEN;

		if (buf_overrun)
		{
			rx_start++;
			rx_read_count++;

			if (rx_start == rx_buf + MAX3100_RX_BUF_LEN)
				rx_start = rx_buf;
		}

		*(rx_end++) = data;
		rx_write_count++;

		if (rx_end == rx_buf + MAX3100_RX_BUF_LEN)
			rx_end = rx_buf;

		if (rx_byte_callback)
			rx_byte_callback (data);
/*
		if ((data == '\r' || data == '\n') && rx_newline_callback)
			rx_newline_callback ();

		if (buf_full && rx_full_callback)
		{
			rx_full_callback ();
		}
		else if (buf_overrun && rx_overrun_callback)
		{
			rx_overrun_callback ();
		}
		*/
	}

	/// Re-enable INT7
	EIMSK |= _BV (INT7);
}

void
max3100_init (void)
{
	spi_slave_desc_t desc;

	spi_init (0);

	/**
	 * Line documentation:
	 * PB0 CS line
	 */

	DDRB |= _BV (PB0);

	desc.port = &PORTB;
	desc.select_delay = 0;
	desc.deselect_delay = 0;
	desc.spi_mode = SPI_MODE_0;

	desc.pin = PB0;
	spi_setup_slave (0, &desc);

	/// Set up the external interrupt line
	/// Enable low level triggered interrupts on INT7
	EIMSK |= _BV (INT7);

	//EICRB |= _BV (ISC71);

	spi_slave_select (0);
	spi_putword (WRITE_CONFIG | INT_READ_ENABLE | INT_WRITE_ENABLE | BAUD_115_2);
	spi_slave_deselect (0);
}

uint8_t
max3100_read_from_rx_buf
(
	uint8_t		*dst,
	uint8_t	n,
	int8_t		append_null
)
{
	uint8_t	bytes_read 	= 0;

	while ((rx_write_count - rx_read_count) && bytes_read < n)
	{
		*(dst++) = *(rx_start++);
		if (rx_start == rx_buf + MAX3100_RX_BUF_LEN)
			rx_start = rx_buf;

		rx_read_count++;
		bytes_read++;
	}

	if (append_null)
		*(dst) = '\0';

	return bytes_read;
}

uint8_t
max3100_write_to_tx_buf
(
	uint8_t		*src,
	uint8_t	n
)
{
	uint8_t bytes_written = 0;

	if (n==0)
		return 0;

	while (bytes_written < n)
	{
		/*if (tx_running)
		{
			cli ();
		}*/

		ATOMIC_BLOCK (ATOMIC_FORCEON)
		{
			if (tx_write_count - tx_read_count == MAX3100_TX_BUF_LEN)
			{
				tx_start++;
				if (tx_start == tx_buf + MAX3100_TX_BUF_LEN)
					tx_start = tx_buf;

				tx_read_count++;
			}

			*(tx_end++) = *(src++);
			if (tx_end == tx_buf + MAX3100_TX_BUF_LEN)
				tx_end = tx_buf;

			tx_write_count++;
		}

		/*if (tx_running)
		{
			sei ();
		}*/

		bytes_written++;
	}

	//if (tx_write_count - tx_read_count == bytes_written)		/* 1 */
	if (tx_running == 0)
	{
		ATOMIC_BLOCK (ATOMIC_FORCEON)
		{
			spi_slave_select (0);
			spi_putword (WRITE_DATA | *(tx_start++));
			spi_slave_deselect (0);

			if (tx_start == tx_buf + MAX3100_TX_BUF_LEN)
				tx_start = tx_buf;

			tx_read_count++;

			tx_running = 1;

			if (bytes_written == 1 && tx_complete_callback)
				tx_complete_callback();
		}
	}

	return bytes_written;
}

//
//	1.	The UART was previously not transmitting when we
//		buffered this data, so start by feeding it one byte.

uint16_t
max3100_rx_bytes_free ()
{
	uint16_t free;

	ATOMIC_BLOCK (ATOMIC_FORCEON)
	{
		free = MAX3100_RX_BUF_LEN - rx_write_count + rx_read_count;
	}
	return free;
}

uint16_t
max3100_tx_bytes_free ()
{
	uint16_t free;

	ATOMIC_BLOCK (ATOMIC_FORCEON)
	{
		free = MAX3100_TX_BUF_LEN - tx_write_count + tx_read_count;
	}
	return free;
}

void
max3100_flush_rx_buf ()
{
	rx_start = rx_end = rx_buf;
	rx_read_count = rx_write_count = 0;
}

void
max3100_flush_tx_buf ()
{
	tx_start = tx_end = tx_buf;
	tx_read_count = tx_write_count = 0;
}

void
max3100_set_rx_byte_callback
(
	void (*callback_func)(uint8_t byte)
)
{
	rx_byte_callback = callback_func;
}

void
max3100_set_rx_newline_callback
(
	void (*callback_func)(void)
)
{
	rx_newline_callback = callback_func;
}

void
max3100_set_rx_error_callback
(
	void (*callback_func)(void)
)
{
	rx_error_callback = callback_func;
}

void
max3100_set_rx_full_callback
(
	void (*callback_func)(void)
)
{
	rx_full_callback = callback_func;
}

void
max3100_set_rx_overrun_callback
(
	void (*callback_func)(void)
)
{
	rx_overrun_callback = callback_func;
}

void
max3100_set_tx_complete_callback
(
	void (*callback_func)(void)
)
{
	tx_complete_callback = callback_func;
}
