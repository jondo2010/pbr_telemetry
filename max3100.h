/*
 * max3100.h
 *
 *  Created on: 2010-02-04
 *      Author: john
 */

#ifndef MAX3100_H_
#define MAX3100_H_

#define BAUD_115_2		0x01
#define BAUD_57_6		0x02

#define INT_RAM_ENABLE		(1 << 8)
#define INT_PARITY_ENABLE	(1 << 9)
#define	INT_READ_ENABLE		(1 << 10)
#define INT_WRITE_ENABLE	(1 << 11)
#define FIFO_DISABLE		(1 << 13)
#define ENABLE_PARITY		(1 << 5)

#define WRITE_CONFIG	0xc000
#define READ_CONFIG		0x4000
#define WRITE_DATA		0x8000
#define READ_DATA		0x0000

#define	MAX3100_RX_BUF_LEN 64
#define MAX3100_TX_BUF_LEN 64

void
max3100_init (void);

uint8_t
max3100_read_from_rx_buf
(
	uint8_t		*dst,
	uint8_t	n,
	int8_t		append_null
);

uint8_t
max3100_write_to_tx_buf
(
	uint8_t		*src,
	uint8_t	n
);

uint16_t
max3100_rx_bytes_free ();

uint16_t
max3100_tx_bytes_free ();

void
max3100_flush_rx_buf ();

void
max3100_flush_tx_buf ();

void
max3100_set_rx_byte_callback
(
	void (*callback_func)(uint8_t byte)
);

void
max3100_set_rx_newline_callback
(
	void (*callback_func)(void)
);

void
max3100_set_rx_error_callback
(
	void (*callback_func)(void)
);

void
max3100_set_rx_full_callback
(
	void (*callback_func)(void)
);

void
max3100_set_rx_overrun_callback
(
	void (*callback_func)(void)
);

void
max3100_set_tx_complete_callback
(
	void (*callback_func)(void)
);

#endif /* MAX3100_H_ */
