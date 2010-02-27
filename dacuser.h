/*
 *  dacuser.h
 *
 *  Author: David Schilling
 *  Date: December 30, 2009
 */

#ifndef DAC_USER_H
#define DAC_USER_H

/* Will turn printfs on in decoder code */
#define DEBUG_DAC_DECODER 0

/* Specify here which types of DAC messages you want decoded */

/* no clue what the data is. Only turn on for debugging */
#define DECODE_RUN_INFO 0

#define DECODE_RUN_STATUS_MESSAGES 1
#define DECODE_RAW_GPS_DATA 0

/* doubt we need this, docs are also pretty confusing */
#define DECODE_NEW_SECTOR_TIME 0
#define DECODE_NEW_LAP_MARKER 0

#define DECODE_LOGGER_STORAGE_CHANNEL 0
#define DECODE_GPS_TIME_STORAGE_CHANNEL 0
#define DECODE_ACCELERATIONS 0
#define DECODE_TIME_STAMP 1
#define DECODE_GPS_POSITIONAL_DATA 0
#define DECODE_SPEED_DATA 0
#define DECODE_BEACON_PULSE_PRESENT 0
#define DECODE_GPS_PULSE_PRESENT 0
#define DECODE_FREQUENCY_INPUT 0
#define DECODE_SERIAL_DATA_INPUT 0
#define DECODE_ANALOGUE_INPUT 1

/* highly doubt we need this, needs fixes in code anyways 
 * due to PIC format floats*/
#define DECODE_CHANNEL_DATA_CHANNEL 0
#define DECODE_DISPLAY_DATA_CHANNEL 0
/* only does something if debugging is on, 
 * will print which device was specified to be reflashed */
#define DECODE_REFLASH_CHANNEL 0

#define DECODE_DATE_STORAGE_CHANNEL 0
#define DECODE_GPS_COURSE_DATA 0
#define DECODE_GPS_ALT_AND_SPEED 0
#define DECODE_EXTENDED_FREQUENCY_INPUTS 0

/* no message to decode, just allows the debug message to be printed */
#define DECODE_START_OF_RUN 0

#define DECODE_PROCESSED_SPEED 0
#define DECODE_GEAR_SETUP_DATA 0

/* doubt we want this, only here for debugging */
#define DECODE_BARGRAPH_SETUP_DATA 0
#define DECODE_DASHBOARD_SETUP_DATA 0
#define DECODE_DASHBOARD_SETUP_DATA_2 0

#define DECODE_NEW_TARGET_SECTOR_TIME 0
#define DECODE_NEW_TARGET_MARKER_TIME 0

/* For debugging. Specifies which module is attached
 * to the auxiliary input */
#define DECODE_AUXILIARY_MODULE_NUMBER 0

#define DECODE_EXT_TEMP 0
#define DECODE_EXT_FREQ 0
#define DECODE_EXT_PERCENTAGE 0
#define DECODE_EXT_TIME 0

/* Not implemented, turn only to see if it's called during debug*/
#define DECODE_LCD_DATA 0

#define DECODE_LED_DATA 0
#define DECODE_PRECALCULATED_DISTANCE 0
#define DECODE_YAW_RATE 0
#define DECODE_CALCULATED_YAW 0
#define DECODE_PITCH_RATE 0
#define DECODE_PITCH_ANGLE 0
#define DECODE_ROLL_RATE 0
#define DECODE_ROLL_ANGLE 0
#define DECODE_GRADIENT_CHANNEL 0
#define DECODE_PULSE_COUNT 0
#define DECODE_BASELINE 0

#define DECODE_UNIT_CONTROL_CHANNEL 0
#define DECODE_Z_ACCELERATIONS 0

#define DECODE_EXT_ANGLE 0
#define DECODE_EXT_PRESSURE 0
#define DECODE_EXT_MISC 0

#define DECODE_TIME_INTO_LAP_AND_SECTOR 0

#define DECODE_HIGH_RESOLUTION_EVENT_TIMER 0
#define DECODE_NED_VELOCITIES 0

/* The following are not implemented */
#define DECODE_SECTOR_DEFINITION 0
#define DECODE_BRAKEBOX_TO_PC 0
#define DECODE_DVR_COMMUNICATION 0
#define DECODE_VIDEO_FRAME_INDEX 0
#define DECODE_GENERAL_CONFIG 0
#define DECODE_GENERAL_CONFIG_REFLASH 0

#endif
