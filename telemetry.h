/*
 * telemetry.h
 *
 *  Created on: 2009-12-20
 *  Author: John Hughes <jondo2010@gmail.com>
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include "dac.h"
#include "xbee.h"

void
telemetry_xbee_command_response_callback (xbee_api_packet *p);

void
telemetry_xbee_rx_callback (xbee_api_packet *p);

void
telemetry_dac_channel_decoded_callback
(
	uint8_t data[],
	int length,
	DACData* dacData,
	enum DAC_Channel channel
);

/**
 * Initialize the telemetry module
 */
void
telemetry_init (void);

/**
 * Process telemetry data, call from main() loop
 */
void
telemetry_process (void);

/**
 * Set the address of the modem used for DTA Communication
 */
void
telemetry_set_dta_address (uint16_t address);

/**
 * Get the address of the modem used for DTA Communication
 */
uint16_t
telemetry_get_dta_address (void);

/**
 * Set the address of the modem used for DAQ Communication
 */
void
telemetry_set_daq_address (uint16_t address);

/**
 * Get the address of the modem used for DAQ Communication
 */
uint16_t
telemetry_get_daq_address (void);

void
telemetry_set_dta_interface_enabled (uint8_t b);

void
telemetry_set_daq_interface_enabled (uint8_t b);

#endif /* TELEMETRY_H_ */
