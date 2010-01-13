/*
 *  dac.c
 *
 *  Author: David Schilling
 *  Date: October 22, 2009
 */

#include "dac.h"
#include "fixedpoint.h"

#if DEBUG_DAC_DECODER
#include <stdio.h>
#else
#define NULL 0
#endif

#define NUM_CHANNELS 108
/* length values for variable length and unused channels */
#define VARIABLE_LENGTH -1
#define NOT_USED_LENGTH 0

#define ANALOGUE_CHANNEL_OFFSET 20
#define FREQUENCY_CHANNEL_OFFSET 14
#define EXTENDED_FREQUENCY_CHANNEL_OFFSET 58
#define PULSE_CHANNEL_OFFSET 86

/* Every DAC channel has a length in bytes and specific
 * decode function */
struct Channel
{
	int length;
	void (*decode)(uint8_t[], DACData*);
};

/* Function Prototypes */

uint8_t calcCheckSum(uint8_t data[], int length);
uint8_t isValidCheckSum(uint8_t data[], int length);

uint8_t decodeBufferData(uint16_t lastIndex);

uint16_t getUInt16(uint8_t[]);
uint32_t getUInt24(uint8_t[]);
uint32_t getUInt32(uint8_t[]);
uint64_t getUInt40(uint8_t[]);

uint16_t getUInt16LittleEndian(uint8_t[]);
uint32_t getUInt24LittleEndian(uint8_t[]);
uint32_t getUInt32LittleEndian(uint8_t[]);

int16_t getInt16(uint8_t[]);
int32_t getInt24(uint8_t[]);
int32_t getInt32(uint8_t[]);

int16_t getInt16LittleEndian(uint8_t[]);

#if DECODE_RUN_INFO
void decodeRunInfo(uint8_t[], DACData*);
#endif
#if DECODE_RUN_STATUS_MESSAGES
void decodeRunStatusMessages(uint8_t[], DACData*);
#endif
#if DECODE_RAW_GPS_DATA
void decodeRawGPSDataInput(uint8_t[], DACData*);
#endif
#if DECODE_NEW_SECTOR_TIME
void decodeNewSectorTime(uint8_t[], DACData*);
#endif
#if DECODE_NEW_LAP_MARKER
void decodeNewLapMarker(uint8_t[], DACData*);
#endif
#if DECODE_LOGGER_STORAGE_CHANNEL
void decodeLoggerStorageChannel(uint8_t[], DACData*);
#endif
#if DECODE_GPS_TIME_STORAGE_CHANNEL
void decodeGPSTimeStorageChannel(uint8_t[], DACData*);
#endif
#if DECODE_ACCELERATIONS
void decodeAccelerations(uint8_t[], DACData*);
#endif
#if DECODE_TIME_STAMP
void decodeTimeStamp(uint8_t[], DACData*);
#endif
#if DECODE_GPS_POSITIONAL_DATA
void decodeGPSPositionalData(uint8_t[], DACData*);
#endif
#if DECODE_SPEED_DATA
void decodeSpeedData(uint8_t[], DACData*);
#endif
#if DECODE_BEACON_PULSE_PRESENT
void decodeBeaconPulsePresent(uint8_t[], DACData*);
#endif
#if DECODE_GPS_PULSE_PRESENT
void decodeGPSPulsePresent(uint8_t[], DACData*);
#endif
#if DECODE_FREQUENCY_INPUT
void decodeFrequencyInput(uint8_t[], DACData*);
#endif
#if DECODE_SERIAL_DATA_INPUT
void decodeSerialDataInput(uint8_t[], DACData*);
#endif
#if DECODE_ANALOGUE_INPUT
void decodeAnalogueInput(uint8_t[], DACData*);
#endif
#if DECODE_CHANNEL_DATA_CHANNEL
void decodeChannelDataChannel(uint8_t[], DACData*);
#endif
#if DECODE_DISPLAY_DATA_CHANNEL
void decodeDisplayDataChannel(uint8_t[], DACData*);
#endif
#if DECODE_REFLASH_CHANNEL
void decodeReflashChannel(uint8_t[], DACData*);
#endif
#if DECODE_DATE_STORAGE_CHANNEL
void decodeDateStorageChannel(uint8_t[], DACData*);
#endif
#if DECODE_GPS_COURSE_DATA
void decodeGPSCourseData(uint8_t[], DACData*);
#endif
#if DECODE_GPS_ALT_AND_SPEED
void decodeGPSAltitudeAndSpeedAccuracy(uint8_t[], DACData*);
#endif
#if DECODE_EXTENDED_FREQUENCY_INPUTS
void decodeExtendedFrequencyInput(uint8_t[], DACData*);
#endif
#if DECODE_START_OF_RUN
void decodeStartOfRunChannel(uint8_t[], DACData*);
#endif
#if DECODE_PROCESSED_SPEED
void decodeProcessedSpeedData(uint8_t[], DACData*);
#endif
#if DECODE_GEAR_SETUP_DATA
void decodeGearSetUpData(uint8_t[], DACData*);
#endif
#if DECODE_BARGRAPH_SETUP_DATA
void decodeBargraphSetUpData(uint8_t[], DACData*);
#endif
#if DECODE_DASHBOARD_SETUP_DATA
void decodeDashboardSetUpData(uint8_t[], DACData*);
#endif
#if DECODE_DASHBOARD_SETUP_DATA_2
void decodeDashboardSetUpData2(uint8_t[], DACData*);
#endif
#if DECODE_NEW_TARGET_SECTOR_TIME
void decodeNewTargetSectorTime(uint8_t[], DACData*);
#endif
#if DECODE_NEW_TARGET_MARKER_TIME
void decodeNewTargetMarkerTime(uint8_t[], DACData*);
#endif
#if DECODE_AUXILIARY_MODULE_NUMBER
void decodeAuxiliaryInputModuleNumber(uint8_t[], DACData*);
#endif
#if DECODE_EXT_TEMP
void decodeExternalTemperatureChannel(uint8_t[], DACData*);
#endif
#if DECODE_EXT_FREQ
void decodeExternalFrequencyChannel(uint8_t[], DACData*);
#endif
#if DECODE_EXT_PERCENTAGE
void decodeExternalPercentageChannel(uint8_t[], DACData*);
#endif
#if DECODE_EXT_TIME
void decodeExternalTimeChannel(uint8_t[], DACData*);
#endif
#if DECODE_LCD_DATA
void decodeNewLCDDataChannel(uint8_t[], DACData*);
#endif
#if DECODE_LED_DATA
void decodeNewLEDDataChannel(uint8_t[], DACData*);
#endif
#if DECODE_PRECALCULATED_DISTANCE
void decodePreCalculatedDistanceDataChannel(uint8_t[], DACData*);
#endif
#if DECODE_YAW_RATE
void decodeYawRateChannel(uint8_t[], DACData*);
#endif
#if DECODE_CALCULATED_YAW
void decodeCalculatedYawChannel(uint8_t[], DACData*);
#endif
#if DECODE_PITCH_RATE
void decodePitchRateChannel(uint8_t[], DACData*);
#endif
#if DECODE_PITCH_ANGLE
void decodePitchAngleChannel(uint8_t[], DACData*);
#endif
#if DECODE_ROLL_RATE
void decodeRollRateChannel(uint8_t[], DACData*);
#endif
#if DECODE_ROLL_ANGLE
void decodeRollAngleChannel(uint8_t[], DACData*);
#endif
#if DECODE_GRADIENT_CHANNEL
void decodeGradientChannel(uint8_t[], DACData*);
#endif
#if DECODE_PULSE_COUNT
void decodePulseCount(uint8_t[], DACData*);
#endif
#if DECODE_BASELINE
void decodeBaselineChannel(uint8_t[], DACData*);
#endif
#if DECODE_UNIT_CONTROL_CHANNEL
void decodeUnitControlChannel(uint8_t[], DACData*);
#endif
#if DECODE_Z_ACCELERATIONS
void decodeZAccelerations(uint8_t[], DACData*);
#endif
#if DECODE_EXT_ANGLE
void decodeExternalAngleChannel(uint8_t[], DACData*);
#endif
#if DECODE_EXT_PRESSURE
void decodeExternalPressureChannel(uint8_t[], DACData*);
#endif
#if DECODE_EXT_MISC
void decodeExternalMiscellaneousChannel(uint8_t[], DACData*);
#endif
#if DECODE_TIME_INTO_LAP_AND_SECTOR
void decodeTimeIntoLapAndSectorChannel(uint8_t[], DACData*);
#endif
#if DECODE_HIGH_RESOLUTION_EVENT_TIMER
void decodeHighResolutionEventTimerChannel(uint8_t[], DACData*);
#endif
#if DECODE_SECTOR_DEFINITION
void decodeSectorDefinitionChannel(uint8_t[], DACData*);
#endif
#if DECODE_BRAKEBOX_TO_PC
void decodeBRAKEBOXToPcCommunicationChannel(uint8_t[], DACData*);
#endif
#if DECODE_DVR_COMMUNICATION
void decodeDVRCommunicationChannel(uint8_t[], DACData*);
#endif
#if DECODE_VIDEO_FRAME_INDEX
void decodeVideoFrameIndex(uint8_t[], DACData*);
#endif
#if DECODE_NED_VELOCITIES
void decodeNEDVelocities(uint8_t[], DACData*);
#endif
#if DECODE_GENERAL_CONFIG
void decodeGeneralConfigChannel(uint8_t[], DACData*);
#endif
#if DECODE_GENERAL_CONFIG_REFLASH
void decodeGeneralConfig(uint8_t[], DACData*);
#endif


/* serial data decoding function table */

const struct Channel channel_table[] = 
{
	{NOT_USED_LENGTH, NULL},					/* 0 Not Used */
	{9, 
	#if (DECODE_RUN_INFO)
	decodeRunInfo
	#else
	NULL
	#endif
	},											/* 1 Run Information */
	{11, 
	#if (DECODE_RUN_STATUS_MESSAGES)
	decodeRunStatusMessages
	#else
	NULL
	#endif
	},											/* 2 Run start/stop info */
	{VARIABLE_LENGTH, 
	#if (DECODE_RAW_GPS_DATA)
	decodeRawGPSDataInput
	#else
	NULL
	#endif
	},											/* 3 Raw GPS Data Input */
	{7 /* or 12*/, 
	#if (DECODE_NEW_SECTOR_TIME)
	decodeNewSectorTime
	#else
	NULL
	#endif
	},											/* 4 New Sector Time */
	{21, 
	#if (DECODE_NEW_LAP_MARKER)
	decodeNewLapMarker
	#else
	NULL
	#endif
	},											/* 5 New Lap Marker */
	{6,
	#if (DECODE_LOGGER_STORAGE_CHANNEL) 
	decodeLoggerStorageChannel
	#else
	NULL
	#endif
	},											/* 6 Logger Storage Channel */
	{6,
	#if (DECODE_GPS_TIME_STORAGE_CHANNEL)
	decodeGPSTimeStorageChannel
	#else
	NULL
	#endif
	},											/* 7 GPS Time Storage Channel */
	{6,
	#if (DECODE_ACCELERATIONS) 
	decodeAccelerations
	#else
	NULL
	#endif
	},											/* 8 Accelerations */
	{5,
	#if (DECODE_TIME_STAMP)
	decodeTimeStamp
	#else
	NULL
	#endif
	},											/* 9 Time Stamp */
	{14,
	#if (DECODE_GPS_POSITIONAL_DATA)
	decodeGPSPositionalData
	#else
	NULL
	#endif
	},											/* 10 GPS Positional Data */
	{10,
	#if (DECODE_SPEED_DATA) 
	decodeSpeedData
	#else
	NULL
	#endif
	},											/* 11 GPS Raw Speed Data */
	{3,
	#if (DECODE_BEACON_PULSE_PRESENT) 
	decodeBeaconPulsePresent
	#else
	NULL
	#endif
	},											/* 12 Beacon Pulse Present */
	{3,
	#if (DECODE_GPS_PULSE_PRESENT)
	decodeGPSPulsePresent
	#else
	NULL
	#endif
	},											/* 13 GPS Pulse Present */
	{5, 
	#if (DECODE_FREQUENCY_INPUT)
	decodeFrequencyInput
	#else
	NULL
	#endif
	},											/* 14 Frequency 1 */
	{5,
	#if (DECODE_FREQUENCY_INPUT)
	decodeFrequencyInput
	#else
	NULL
	#endif
	},											/* 15 Frequency 2 */
	{5,
	#if (DECODE_FREQUENCY_INPUT)
	decodeFrequencyInput
	#else
	NULL
	#endif
	},											/* 16 Frequency 3 */
	{5,
	#if (DECODE_FREQUENCY_INPUT)
	decodeFrequencyInput
	#else
	NULL
	#endif
	},											/* 17 Frequency 4 */
	{5,
	#if (DECODE_FREQUENCY_INPUT)
	decodeFrequencyInput
	#else
	NULL
	#endif
	},											/* 18 RPM Data */
	{VARIABLE_LENGTH,
	#if (DECODE_SERIAL_DATA_INPUT)
	decodeSerialDataInput
	#else
	NULL
	#endif
	},											/* 19 Serial Data Input */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 20 Analogue 1 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 21 Analogue 2 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 22 Analogue 3 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 23 Analogue 4 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 24 Analogue 5 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 25 Analogue 6 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 26 Analogue 7 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 27 Analogue 8 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 28 Analogue 9 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 29 Analogue 10 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 30 Analogue 11 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 31 Analogue 12 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 32 Analogue 13 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 33 Analogue 14 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 34 Analogue 15 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 35 Analogue 16 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 36 Analogue 17 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 37 Analogue 18 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 38 Analogue 19 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 39 Analogue 20 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 40 Analogue 21 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 41 Analogue 22 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 42 Analogue 23 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 43 Analogue 24 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 44 Analogue 25 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 45 Analogue 26 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 46 Analogue 27 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 47 Analogue 28 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 48 Analogue 29 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 49 Analogue 30 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 50 Analogue 31 */
	{4, 
	#if (DECODE_ANALOGUE_INPUT)
	decodeAnalogueInput
	#else
	NULL
	#endif
	},											/* 51 Analogue 32 */
	{67,
	#if (DECODE_CHANNEL_DATA_CHANNEL)
	decodeChannelDataChannel
	#else
	NULL
	#endif
	},											/* 52 Channel Data Channel */
	{11,
	#if (DECODE_DISPLAY_DATA_CHANNEL) 
	decodeDisplayDataChannel
	#else
	NULL
	#endif
	},											/* 53 Display Data Channel */
	{6,
	#if (DECODE_REFLASH_CHANNEL)
	decodeReflashChannel
	#else
	NULL
	#endif
	},											/* 54 Reflash Channel */
	{10,
	#if (DECODE_DATE_STORAGE_CHANNEL)
	decodeDateStorageChannel
	#else
	NULL
	#endif
	},											/* 55 Date Storage Channel */
	{10,
	#if (DECODE_GPS_COURSE_DATA) 
	decodeGPSCourseData
	#else
	NULL
	#endif
	},											/* 56 GPS Course Data */
	{10,
	#if (DECODE_GPS_ALT_AND_SPEED)
	decodeGPSAltitudeAndSpeedAccuracy
	#else
	NULL
	#endif
	},											/* 57 GPS Altitude and Speed Accuracy */
	{11,
	#if (DECODE_EXTENDED_FREQUENCY_INPUTS)
	decodeExtendedFrequencyInput
	#else
	NULL
	#endif
	},											/* 58 Extended Frequency 1 */
	{11,
	#if (DECODE_EXTENDED_FREQUENCY_INPUTS)
	decodeExtendedFrequencyInput
	#else
	NULL
	#endif
	},											/* 59 Extended Frequency 2 */
	{11,
	#if (DECODE_EXTENDED_FREQUENCY_INPUTS)
	decodeExtendedFrequencyInput
	#else
	NULL
	#endif
	},											/* 60 Extended Frequency 3 */
	{11,
	#if (DECODE_EXTENDED_FREQUENCY_INPUTS)
	decodeExtendedFrequencyInput
	#else
	NULL
	#endif
	},											/* 61 Extended Frequency 4 */
	{11,
	#if (DECODE_EXTENDED_FREQUENCY_INPUTS)
	decodeExtendedFrequencyInput
	#else
	NULL
	#endif
	},											/* 62 Extended RPM */
	{3,
	#if (DECODE_START_OF_RUN)
	decodeStartOfRunChannel
	#else
	NULL
	#endif
	},											/* 63 Start of Run Channel */
	{5,
	#if (DECODE_PROCESSED_SPEED)
	decodeProcessedSpeedData
	#else
	NULL
	#endif
	},											/* 64 Processed Speed Data */
	{30,
	#if (DECODE_GEAR_SETUP_DATA)
	decodeGearSetUpData
	#else
	NULL
	#endif
	},											/* 65 Gear Set Up Data */
	{11,
	#if (DECODE_BARGRAPH_SETUP_DATA)
	decodeBargraphSetUpData
	#else
	NULL
	#endif
	},											/* 66 Bargraph Set Up Data */
	{4,
	#if (DECODE_DASHBOARD_SETUP_DATA)
	decodeDashboardSetUpData
	#else
	NULL
	#endif
	},											/* 67 Dashboard Set Up Data */
	{4,
	#if (DECODE_DASHBOARD_SETUP_DATA_2)
	decodeDashboardSetUpData2
	#else
	NULL
	#endif
	},											/* 68 Dashboard Set Up Data Two */
	{42,
	#if (DECODE_NEW_TARGET_SECTOR_TIME)
	decodeNewTargetSectorTime
	#else
	NULL
	#endif
	},											/* 69 New Target Sector Time */
	{42,
	#if (DECODE_NEW_TARGET_MARKER_TIME)
	decodeNewTargetMarkerTime
	#else
	NULL
	#endif
	},											/* 70 New Target Marker Time */
	{3,
	#if (DECODE_AUXILIARY_MODULE_NUMBER)
	decodeAuxiliaryInputModuleNumber
	#else
	NULL
	#endif
	},											/* 71 Auxiliary Input Module Number */
	{5,
	#if (DECODE_EXT_TEMP)
	decodeExternalTemperatureChannel
	#else
	NULL
	#endif
	},											/* 72 External Temperature Channel */
	{5,
	#if (DECODE_EXT_FREQ)
	decodeExternalFrequencyChannel
	#else
	NULL
	#endif
	},											/* 73 External Frequency Channel */
	{5,
	#if (DECODE_EXT_PERCENTAGE) 
	decodeExternalPercentageChannel
	#else
	NULL
	#endif
	},											/* 74 External Percentage Channel */
	{6,
	#if (DECODE_EXT_TIME)
	decodeExternalTimeChannel
	#else
	NULL
	#endif
	},											/* 75 External Time Channel */
	{24,
	#if (DECODE_LCD_DATA)
	decodeNewLCDDataChannel
	#else
	NULL
	#endif
	},											/* 76 New LCD Data Channel */
	{3,
	#if (DECODE_LED_DATA) 
	decodeNewLEDDataChannel
	#else
	NULL
	#endif
	},											/* 77 New LED Data Channel */
	{6,
	#if (DECODE_PRECALCULATED_DISTANCE)
	decodePreCalculatedDistanceDataChannel
	#else
	NULL
	#endif
	},											/* 78 Pre Calculated Distance Data Channel */
	{4,
	#if (DECODE_YAW_RATE)
	decodeYawRateChannel
	#else
	NULL
	#endif
	},											/* 79 Yaw Rate Channel */
	{4,
	#if (DECODE_CALCULATED_YAW)
	decodeCalculatedYawChannel
	#else
	NULL
	#endif
	},											/* 80 Calculated Yaw Channel */
	{5,
	#if (DECODE_PITCH_RATE)
	decodePitchRateChannel
	#else
	NULL
	#endif
	},											/* 81 Pitch Rate Channel */
	{5,
	#if (DECODE_PITCH_ANGLE)
	decodePitchAngleChannel
	#else
	NULL
	#endif
	},											/* 82 Pitch Angle Channel */
	{5,
	#if (DECODE_ROLL_RATE) 
	decodeRollRateChannel
	#else
	NULL
	#endif
	},											/* 83 Roll Rate Channel */
	{5,
	#if (DECODE_ROLL_ANGLE) 
	decodeRollAngleChannel
	#else
	NULL
	#endif
	},											/* 84 Roll Angle Channel */
	{10,
	#if (DECODE_GRADIENT_CHANNEL) 
	decodeGradientChannel
	#else
	NULL
	#endif
	},											/* 85 Gradient Channel */
	{5,
	#if (DECODE_PULSE_COUNT) 
	decodePulseCount
	#else
	NULL
	#endif
	},											/* 86 Pulse Count 1 */
	{5,
	#if (DECODE_PULSE_COUNT) 
	decodePulseCount
	#else
	NULL
	#endif
	},											/* 87 Pulse Count 2 */
	{5,
	#if (DECODE_PULSE_COUNT) 
	decodePulseCount
	#else
	NULL
	#endif
	},											/* 88 Pulse Count 3 */
	{5,
	#if (DECODE_PULSE_COUNT) 
	decodePulseCount
	#else
	NULL
	#endif
	},											/* 89 Pulse Count 4 */
	{6,
	#if (DECODE_BASELINE) 
	decodeBaselineChannel
	#else
	NULL
	#endif
	},											/* 90 Baseline Channel */
	{5,
	#if (DECODE_UNIT_CONTROL_CHANNEL)
	decodeUnitControlChannel
	#else
	NULL
	#endif
	},											/* 91 Unit Control Channel */
	{4,
	#if (DECODE_Z_ACCELERATIONS) 
	decodeZAccelerations
	#else
	NULL
	#endif
	},											/* 92 Z Acceleration */
	{5,
	#if (DECODE_EXT_ANGLE) 
	decodeExternalAngleChannel
	#else
	NULL
	#endif
	},											/* 93 External Angle Channel */
	{6,
	#if (DECODE_EXT_PRESSURE)
	decodeExternalPressureChannel
	#else
	NULL
	#endif
	},											/* 94 External Pressure Channel */
	{5,
	#if (DECODE_EXT_MISC) 
	decodeExternalMiscellaneousChannel
	#else
	NULL
	#endif
	},											/* 95 External Miscellaneous Channel */
	{10,
	#if (DECODE_TIME_INTO_LAP_AND_SECTOR) 
	decodeTimeIntoLapAndSectorChannel
	#else
	NULL
	#endif
	},											/* 96 Time into current lap and sector */
	{8,
	#if (DECODE_HIGH_RESOLUTION_EVENT_TIMER) 
	decodeHighResolutionEventTimerChannel
	#else
	NULL
	#endif
	},											/* 97 High resolution event timer */
	{NOT_USED_LENGTH, NULL},					/* 98 Not Used */
	{NOT_USED_LENGTH, NULL},					/* 99 Not Used */
	{NOT_USED_LENGTH, NULL},					/* 100 Not Used */
	{19,
	#if (DECODE_SECTOR_DEFINITION) 
	decodeSectorDefinitionChannel
	#else
	NULL
	#endif
	},											/* 101 Sector Definition Channel */
	{VARIABLE_LENGTH, 
	#if (DECODE_BRAKEBOX_TO_PC) 
	decodeBRAKEBOXToPcCommunicationChannel
	#else
	NULL
	#endif
	},											/* 102 BRAKEBOX to PC Communication Channel */
	{17,
	#if (DECODE_DVR_COMMUNICATION) 
	decodeDVRCommunicationChannel
	#else
	NULL
	#endif
	},											/* 103 DVR Communication Channel */
	{9,
	#if (DECODE_VIDEO_FRAME_INDEX) 
	decodeVideoFrameIndex
	#else
	NULL
	#endif
	},											/* 104 Video frame index */
	{11,
	#if (DECODE_NED_VELOCITIES) 
	decodeNEDVelocities
	#else
	NULL
	#endif
	},											/* 105 Local NED velocities */
	{NOT_USED_LENGTH, NULL},					/* 106 Not Used */
	{VARIABLE_LENGTH, 
	#if (DECODE_GENERAL_CONFIG) 
	decodeGeneralConfigChannel
	#else
	NULL
	#endif
	},											/* 107 General Configuration Message */
	{VARIABLE_LENGTH, 
	#if (DECODE_GENERAL_CONFIG_REFLASH) 
	decodeGeneralConfig
	#else
	NULL
	#endif
	}											/* 108 General Configuration and Reflash Message */
};


/* internal to external (user) analogue channel mappings */
const uint8_t extern_analogue_channel[] = 
{
	8,	/* DL1 Analogue 0 */
	6,	/* DL1 Analogue 1 */
	7,	/* DL1 Analogue 2 */
	5,	/* DL1 Analogue 3 */
	4,	/* DL1 Analogue 4 */
	2,	/* DL1 Analogue 5 */
	3,	/* DL1 Analogue 6 */
	1,	/* DL1 Analogue 7 */
	16,	/* DL1 Analogue 8 */
	14,	/* DL1 Analogue 9 */
	15,	/* DL1 Analogue 10 */
	13,	/* DL1 Analogue 11 */
	12,	/* DL1 Analogue 12 */
	10,	/* DL1 Analogue 13 */
	11,	/* DL1 Analogue 14 */
	9,	/* DL1 Analogue 15 */
	17,	/* DL1 Analogue 16 */
	18,	/* DL1 Analogue 17 */
	19,	/* DL1 Analogue 18 */
	20,	/* DL1 Analogue 19 */
	21,	/* DL1 Analogue 20 */
	22,	/* DL1 Analogue 21 */
	23,	/* DL1 Analogue 22 */
	24,	/* DL1 Analogue 23 */
	25,	/* DL1 Analogue 24 */
	26,	/* DL1 Analogue 25 */
	27,	/* DL1 Analogue 26 */
	28,	/* DL1 Analogue 27 */
	29,	/* DL1 Analogue 28 */
	30,	/* DL1 Analogue 29 */
	31,	/* DL1 Analogue 30 */
	32,	/* DL1 Analogue 31 */
};

/* external (user) to internal analogue channel mappings 
 * Note: for array indexing purposes, the index is the user channel - 1
 */
const uint8_t intern_analogue_channel[] = 
{
	7,  /* User Analogue 1 */
	5,  /* User Analogue 2 */
	6,  /* User Analogue 3 */
	4,  /* User Analogue 4 */
	3,  /* User Analogue 5 */
	1,  /* User Analogue 6 */
	2,  /* User Analogue 7 */
	0,  /* User Analogue 8 */
	15, /* User Analogue 9 */
	13, /* User Analogue 10 */
	14, /* User Analogue 11 */
	12, /* User Analogue 12 */
	11, /* User Analogue 13 */
	9,  /* User Analogue 14 */
	10, /* User Analogue 15 */
	8,  /* User Analogue 16 */
	16, /* User Analogue 17 */
	17,	/* User Analogue 18 */
	18,	/* User Analogue 19 */
	19,	/* User Analogue 20 */
	20,	/* User Analogue 21 */
	21,	/* User Analogue 22 */
	22,	/* User Analogue 23 */
	23,	/* User Analogue 24 */
	24,	/* User Analogue 25 */
	25,	/* User Analogue 26 */
	26,	/* User Analogue 27 */
	27,	/* User Analogue 28 */
	28,	/* User Analogue 29 */
	29,	/* User Analogue 30 */
	30,	/* User Analogue 31 */
	31,	/* User Analogue 32 */
};

/* internal variables */
static decodeBuffer dbuff;
static DACData dacData;
static ChannelDecodedFunc *channelDecodedFunc = NULL;

/* For RaceTechnology internal use only 
 * So we don't know what data the message contains
 */
#if (DECODE_RUN_INFO)
void decodeRunInfo(uint8_t data[], DACData *dacData)
{
	#if (DEBUG_DAC_DECODER)
	/* Only here to see if it's getting called */
	int i;
	
	printf("Decoding run info...\n");
	
	for (i = 0; i < 9; ++i)
		printf("data[%d] = 0x%x\n", i, data[i + 1]);
	#endif
}
#endif

/* Decodes DAC Run Status Messages */
#if DECODE_RUN_STATUS_MESSAGES
void decodeRunStatusMessages(uint8_t data[], DACData *dacData)
{	
	uint8_t startMethod = data[1];
	uint8_t stopMethod = data[2];
	uint8_t pretriggerLoopExit = data[3];
	uint8_t pretriggerTime = data[4];
	uint8_t postTriggerTime = data[5];
	uint8_t autoStartSrc = data[6];
	uint8_t autoStopSrc = data[7];
	uint16_t lowestBufferVal = getUInt16(&data[8]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding run status messages...\n");
	#endif
	
	if (data[1] == 4)
	{
		startMethod = data[2] >> 4;
		stopMethod = data[2] & 0x0f;
	}
	
	#if DEBUG_DAC_DECODER
	if (data[1] <= 4)
	{
		if ((startMethod > 0) && (startMethod < 4))
			printf("Start method: ");
			
		switch (startMethod)
		{
			case 1:
				printf("Button press\n");
				break;
				
			case 2:
				printf("Autostart\n");
				
				printf("Autostart source: ");
				if ((autoStartSrc >= 1) && (autoStartSrc <= 8))
					printf("ADC channel %d\n", autoStartSrc);
				else if (autoStartSrc == 15)
					printf("Lateral g-force\n");
				else if (autoStartSrc == 16)
					printf("Longitudinal g-force\n");
				else
					printf("Unrecognized autostart source = %d\n", autoStartSrc);
				break;
				
			case 3:
				printf("Starting pretrigger loop (On VIDEO4: DVR serial command from PC commander   OR   Remote RT serial command from DASH)\n");
				
				printf("Pretrigger loop exit: ");
				switch (pretriggerLoopExit)
				{
					case 1:
						printf("Button press\n");
						break;
						
					case 2:
						printf("Autostart\n");
						break;
						
					default:
						printf("Unrecognized pretrigger loop exit val = %d\n", pretriggerLoopExit);
						break;
				}
				
				printf("Pretrigger time = %d minutes.\n", pretriggerTime);
				break;
				
			default:
				break;
		}
		
		if ((stopMethod > 0) && (stopMethod < 11))
			printf("Stop method: ");
	
		switch (stopMethod)
		{
			case 1:
				printf("Button press\n");
				break;
				
			case 2:
				printf("Autostop condition met\n");
				
				printf("Autostop source: ");
				if ((autoStopSrc >= 1) && (autoStopSrc <= 8))
					printf("ADC channel %d\n", autoStopSrc);
				else if (autoStopSrc == 15)
					printf("Lateral g-force\n");
				else if (autoStopSrc == 16)
					printf("Longitudinal g-force\n");
				else
					printf("Unrecognized autostop source = %d\n", autoStopSrc);
				break;
				
			case 3:
				printf("post trigger expired\n");
				printf("Posttrigger time = %d minutes.\n", postTriggerTime);
				break;
				
			case 4:
				printf("low battery voltage\n");
				break;
				
			case 5:
				printf("low buffer space, most likely slow CF card\n");
				printf("Lowest buffer value = %d.\n", lowestBufferVal);
				break;
				
			case 6:
				printf("GSM command\n");
				break;
				
			case 7:
				printf("Disk full\n");
				break;
				
			case 8:
				printf("DVR serial command from PC commander\n");
				break;
				
			case 9:
				printf("Remote RT serial command from DASH\n");
				break;
				
			case 10:
				printf("4 GB file limit exceeded (pDrive only)\n");
				break;
				
			default:
				break;
		}
	}
	else if (data[1] == 5)
	{
		printf("Failed to add a track marker on a logger. The reason was: \n");
		switch (data[2])
		{
			case 1:
				printf("Marker already exists at this location.\n");
				break;
			
			case 2:
				printf("Too many markers defined.\n");
				break;
			
			case 3:
				printf("Not currently logging data.\n");
				break;
			
			case 4:
				printf("No card inserted.\n");
				break;
			
			case 5:
				printf("GPS data is too inaccurate. Therefore, positional accuracy is greater than the threshold.\n");
				break;
			
			case 6:
				printf("Correct GPS data is not available. Therefore, DL1 was not setup correctly.\n");
				break;
		}
	}
	#endif
	
	dacData->statusMsg.startMethod = startMethod;
	dacData->statusMsg.stopMethod = stopMethod;
	dacData->statusMsg.pretriggerLoopExit = pretriggerLoopExit;
	dacData->statusMsg.pretriggerTime = pretriggerTime;
	dacData->statusMsg.postTriggerTime = postTriggerTime;
	dacData->statusMsg.autoStartSrc = autoStartSrc;
	dacData->statusMsg.autoStopSrc = autoStopSrc;
	dacData->statusMsg.lowestBufferVal = lowestBufferVal;
}
#endif

/* Decode Raw GPS data */
#if DECODE_RAW_GPS_DATA
void decodeRawGPSDataInput(uint8_t data[], DACData *dacData)
{
	int i;
	dacData->rawGPSData.length = data[1];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding raw gps data input...\n");
	printf("Received %d bytes of gps data:\n", data[1]);
	#endif
	
	/* data is in reverse order from which it was received by the DAC*/
	for (i = data[1] + 1; i > 1; --i)
	{
		dacData->rawGPSData.data[data[1] + 1 - i] = data[i];
		
		#if DEBUG_DAC_DECODER
		printf("0x%x\n", data[i]);
		#endif
	}
}
#endif

/* Decode the new sector time 
 * definition is screwy since docs say length is 7 but decoding
 * requires length of 12
 */
#if DECODE_NEW_SECTOR_TIME
void decodeNewSectorTime(uint8_t data[], DACData *dacData)
{
	dacData->sectTime.marker = data[1];
	dacData->sectTime.markerTime = getUInt32LittleEndian(&data[2]);
	dacData->sectTime.sectorNum = data[6];
	dacData->sectTime.sectorTime = getUInt32LittleEndian(&data[7]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding new sector time...\n");
	printf("Matched Marker = %d, Time at marker = %u ms, Matched Sector Num = %d, Sector Time = %u ms\n", dacData->sectTime.marker, dacData->sectTime.markerTime, dacData->sectTime.sectorNum, dacData->sectTime.sectorTime);
	#endif
}
#endif

/* Decode the new lap marker
 * Can't find where this is specified.
 */
#if DECODE_NEW_LAP_MARKER
void decodeNewLapMarker(uint8_t data[], DACData *dacData)
{
	dacData->lapMarker.num = data[1];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding new lap marker...\n");
	printf("Marker number = %d\n", data[1]);
	#endif
}
#endif

/* Decode the data logger info */
#if DECODE_LOGGER_STORAGE_CHANNEL
void decodeLoggerStorageChannel(uint8_t data[], DACData *dacData)
{
	dacData->loggerStorage.serialNumber = getUInt16(&data[1]);
	dacData->loggerStorage.softwareVer = data[3];
	dacData->loggerStorage.bootloadVer = data[4];

	#if DEBUG_DAC_DECODER
	printf("Decoding logger storage channel...\n");
	printf("Serial Num = %d, Software Version = %d, Bootload Version = %d\n", dacData->loggerStorage.serialNumber, dacData->loggerStorage.softwareVer, dacData->loggerStorage.bootloadVer);
	#endif
}
#endif

/* Decode the GPS time (ms time of week)
 * Time of week is reset to zero at midnight
 * between Saturday and Sunday
 */
#if DECODE_GPS_TIME_STORAGE_CHANNEL
void decodeGPSTimeStorageChannel(uint8_t data[], DACData *dacData)
{
	dacData->gpsTime = getUInt32(&data[1]);

	#if DEBUG_DAC_DECODER
	printf("Decoding gps time storage channel...\n");
	printf("GPS time of week = %u ms\n", dacData->gpsTime);
	#endif
}
#endif

/* Decode the Accelerometer data 
 * Sign Convention
 * Longitudinal is positive for acceleration, negative for braking.
 * Lateral is positive for cornering around a RH bend, negative for cornering
 * around a LH bend.
 */
#if DECODE_ACCELERATIONS
void decodeAccelerations(uint8_t data[], DACData *dacData)
{
	int16_t lateral = getUInt16(&data[1]) & 0x7fff;
	int16_t longitudinal = getUInt16(&data[3]) & 0x7fff;
	
	if ((data[1] & 0x80) == 0)
		lateral = -lateral;
	
	if ((data[3] & 0x80) == 0)
		longitudinal = -longitudinal;
	
	#if DEBUG_DAC_DECODER
	printf("Decoding accelerations...\n");
	printf("Lateral = %f, Longitudinal = %f\n", fpToReal(lateral), fpToReal(longitudinal));
	#endif
	
	dacData->accels.lateral = lateral;
	dacData->accels.longitudinal = longitudinal;
}
#endif

/* Decode the time stamp */
#if DECODE_TIME_STAMP
void decodeTimeStamp(uint8_t data[], DACData *dacData)
{
	dacData->timeStamp = getUInt24(&data[1]);

	#if DEBUG_DAC_DECODER
	printf("Decoding time stamp...\n");
	printf("Time stamp = %d\n", dacData->timeStamp);
	#endif
}
#endif

/* Decode GPS positional data */
#if DECODE_GPS_POSITIONAL_DATA
void decodeGPSPositionalData(uint8_t data[], DACData *dacData)
{
	dacData->gpsPos.longitude = getInt32(&data[1]);
	dacData->gpsPos.latitude = getInt32(&data[5]);
	dacData->gpsPos.accuracy = getUInt32(&data[9]);

	#if DEBUG_DAC_DECODER
	printf("Decoding gps positional data...\n");
	printf("Gps Longitude = %f degrees, Gps Latitude = %f degrees, Positional Accuracy Estimate = %u mm\n", dacData->gpsPos.longitude * 0.0000001, dacData->gpsPos.latitude * 0.0000001, dacData->gpsPos.accuracy);
	#endif
}
#endif

/* Decode Speed data in mm/s */
#if DECODE_SPEED_DATA
void decodeSpeedData(uint8_t data[], DACData *dacData)
{
	dacData->speed.speed = getUInt32(&data[1]);
	dacData->speed.accuracy = getUInt24(&data[6]);
	dacData->speed.source = data[5];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding speed data...\n");
	printf("Speed = %f m/s, Speed accuracy = %f m/s, source = %d\n", dacData->speed.speed * 0.01, dacData->speed.accuracy * 0.01, dacData->speed.source);
	#endif
}
#endif

/* Decode the beacon pulse state */
#if DECODE_BEACON_PULSE_PRESENT
void decodeBeaconPulsePresent(uint8_t data[], DACData *dacData)
{
	dacData->beaconPulseState = data[1];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding beacon pulse present...\n");
	printf("Pulse state = %d\n", dacData->beaconPulseState);
	#endif
}
#endif

/* Decode the GPS pulse state */
#if DECODE_GPS_PULSE_PRESENT
void decodeGPSPulsePresent(uint8_t data[], DACData *dacData)
{
	dacData->gpsPulseState = data[1];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding gps pulse present...\n");
	printf("Pulse state = %d\n", dacData->gpsPulseState);
	#endif
}
#endif

/* Decode Frequency Input data, Frequency input 5 is RPM */
#if DECODE_FREQUENCY_INPUT
void decodeFrequencyInput(uint8_t data[], DACData *dacData)
{	
	#if DEBUG_DAC_DECODER
	double freq;
	#endif
	
	dacData->frequency[data[0] = FREQUENCY_CHANNEL_OFFSET] = getUInt24(&data[1]);
	
	#if DEBUG_DAC_DECODER
	freq = dacData->frequency[data[0] - FREQUENCY_CHANNEL_OFFSET];
	
	if (freq != 0)
		freq = 1 / (freq * TICK_PERIOD);
		
	if ( (data[0]  - FREQUENCY_CHANNEL_OFFSET) == 4)
		printf("Decoding rpm input...\n");
	else
	{
		printf("Decoding frequency input...\n");
		printf("Input = %d, ", (data[0] - FREQUENCY_CHANNEL_OFFSET + 1));
	}
	printf("Freq = %f\n", freq);
	#endif
}
#endif

/* Decode raw serial data input */
#if DECODE_SERIAL_DATA_INPUT
void decodeSerialDataInput(uint8_t data[], DACData *dacData)
{
	int i;
	dacData->serialData.length = data[1];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding serial data...\n");
	printf("Received %d bytes of serial data:\n", data[1]);
	#endif
	
	/* Data is in reverse order from which it was received by the DAC */
	for (i = data[1] + 1; i > 1; --i)
	{
		dacData->serialData.data[data[1] + 1 - i] = data[i];
		
		#if DEBUG_DAC_DECODER
		printf("0x%x \n", data[i]);
		#endif
	}
}
#endif

/* Decode analogue channel input in mili-volts */
#if DECODE_ANALOGUE_INPUT
void decodeAnalogueInput(uint8_t data[], DACData *dacData)
{
	dacData->analogueVoltage[extern_analogue_channel[data[0] - ANALOGUE_CHANNEL_OFFSET] - 1] = getUInt16(&data[1]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding analog input...\n");
	printf("Channel = %d, Voltage = %f volts\n", extern_analogue_channel[data[0] - ANALOGUE_CHANNEL_OFFSET], dacData->analogueVoltage[extern_analogue_channel[data[0] - ANALOGUE_CHANNEL_OFFSET] - 1] / 1000.0);
	#endif
}
#endif

/* Decode channel data 
 * Floating point values are in PIC format:
 * 3 byte mantissa (LSB first) followed by 1 byte exponent
 *
 * The change in value between the previous and current value is then limited by the value in Maximum rate of change. The filter is then applied in the form:
 * New value = Filter value x current value + (1-filter value) x last value
 * So a value of 1 puts no filtering on the signal.
 * Values of 0 in the filter rate and maximum rate of change are ignored.
 *
 * Bytes 48-65 are reserved for future use
 */
#if DECODE_CHANNEL_DATA_CHANNEL
void decodeChannelDataChannel(uint8_t data[], DACData *dacData)
{
	int i;
	
	#if DEBUG_DAC_DECODER
	double output;
	#endif
	
	uint8_t channel = data[1];
	dacData->channelData.channel = channel;
	
	/* need to interpret these in PIC floating point format */
	dacData->channelData.a = getUInt32(&data[2]);
	dacData->channelData.b = getUInt32(&data[6]);
	dacData->channelData.c = getUInt32(&data[10]);
	dacData->channelData.d = getUInt32(&data[14]);
	
	/* these are in PIC format as well */
	dacData->channelData.upperAlarm = getUInt32(&data[18]);
	dacData->channelData.lowerAlarm = getUInt32(&data[22]);
	
	for (i = 0; i < 8; ++i)
		dacData->channelData.longName[i]= data[26 + i];
	
	for (i = 0; i < 3; ++i)
		dacData->channelData.shortName[i] = data[33 + i];
	
	for (i = 0; i < 3; ++i)
		dacData->channelData.unitsName[i] = data[36 + i];
		
	dacData->channelData.numPlacesAfterDecimal = data[39];
	
	/* these also need to be interpreted in PIC floating point format */
	dacData->channelData.maxChangeRate = getUInt24(&data[41]);
	dacData->channelData.filterRate = getUInt32(&data[44]);
	
	#if DEBUG_DAC_DECODER
	output = dacData->channelData.a * (channel * channel * channel) + dacData->channelData.b * (channel * channel) + dacData->channelData.c * channel + dacData->channelData.d;

	printf("Decoding channel data channel...\n");
	printf("Long name = %s, short name = %s, units name = %s, output = %f\n", dacData->channelData.longName, dacData->channelData.shortName, dacData->channelData.unitsName, output);
	#endif
}
#endif

/* Decode display data channel 
 * See the DL1 documentation for further information on what
 * the channel and view numbers refer to. 
 * Y:\superusers\Projects\Products\DL1\Documentation\DL1 Documentation.doc
 */
#if DECODE_DISPLAY_DATA_CHANNEL	
void decodeDisplayDataChannel(uint8_t data[], DACData *dacData)
{
	dacData->displayData.menuNum = data[1];
	dacData->displayData.topLeftChannel = data[2];
	dacData->displayData.topLeftView = data[3];
	dacData->displayData.topRightChannel = data[4];
	dacData->displayData.topRightView = data[5];
	dacData->displayData.bottomLeftChannel = data[6];
	dacData->displayData.bottomLeftView = data[7];
	dacData->displayData.bottomRightChannel = data[8];
	dacData->displayData.bottomRightView = data[9];

	#if DEBUG_DAC_DECODER
	printf("Decoding display data channel...\n");
	printf("Menu Number = %d, Top-Left channel = %d, Top-Left View = %d\n", dacData->displayData.menuNum, dacData->displayData.topLeftChannel, dacData->displayData.topLeftView);
	printf("Top-Right channel = %d, Top-Right View = %d\n", dacData->displayData.topRightChannel, dacData->displayData.topRightView);
	printf("Bottom-Left channel = %d, Bottom-Left View = %d\n", dacData->displayData.bottomLeftChannel, dacData->displayData.bottomLeftView);
	printf("Bottom-Right channel = %d, Bottom-Right View = %d\n", dacData->displayData.bottomRightChannel, dacData->displayData.bottomRightView);
	#endif
}
#endif

/* Decode Reflash Channel
 * Product specific message to reflash different
 * products from a DL1/DL2 
 */
#if (DECODE_REFLASH_CHANNEL)
void decodeReflashChannel(uint8_t data[], DACData *dacData)
{
	#if (DEBUG_DAC_DECODER)
	printf("Decoding reflash channel...\n");
	if ( (data[1] == 73) && (data[2] == 56) && (data[3] == 183) && (data[4] == 26) )
		printf("Reflash DASH1\n");
	else if ( (data[1] == 62) && (data[2] == 153) && (data[3] == 223) && (data[4] == 25) )
		printf("Reflash VOB\n");
	else if ( (data[1] == 88) && (data[2] == 136) && (data[3] == 218) && (data[4] == 22) )
		printf("Reflash SPEEDBOX2\n");
	#endif
}
#endif

/* Decode Date Storage Channel 
 * Offset from GMT is given by 15 minutes increments or decrements
 * For example (-22) = (- 5:30 GMT)
 */
#if DECODE_DATE_STORAGE_CHANNEL
void decodeDateStorageChannel(uint8_t data[], DACData *dacData)
{
	dacData->date.seconds = data[1];
	dacData->date.minutes = data[2];
	dacData->date.hours = data[3];
	dacData->date.day = data[4];
	dacData->date.month = data[5];
	dacData->date.year = getUInt16(&data[6]);
	dacData->date.offset = data[8];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding date storage channel...\n");
	printf("GMT is %d/%d/%d %d:%d:%d (day/month/year hour:minute:seconds)\n", dacData->date.day, dacData->date.month, dacData->date.year, dacData->date.hours, dacData->date.minutes, dacData->date.seconds);
	printf("offset is %d\n", dacData->date.offset);
	#endif
}
#endif

/* Decode GPS Course Data 
 * GPS Heading and heading accuracy are in degrees * 10^-5
 */
#if DECODE_GPS_COURSE_DATA
void decodeGPSCourseData(uint8_t data[], DACData *dacData)
{
	dacData->gpsCourseData.heading = getInt32(&data[1]);
	dacData->gpsCourseData.accuracy = getUInt32(&data[5]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding gps course data...\n");
	printf("GPS Heading = %d (degrees x 10^-5), GPS Heading Accuracy = %u (degrees x 10^-5)\n", dacData->gpsCourseData.heading, dacData->gpsCourseData.accuracy);
	#endif
}
#endif

/* Decode GPS Altitude And Speed Accuracy in milimeters */
#if DECODE_GPS_ALT_AND_SPEED
void decodeGPSAltitudeAndSpeedAccuracy(uint8_t data[], DACData *dacData)
{
	dacData->gpsAltAndSpeed.altitude = getInt32(&data[1]);
	dacData->gpsAltAndSpeed.accuracy = getInt32(&data[5]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding gps altitude and speed accuracy...\n");
	printf("Altitude = %f m, Altitude Accuracy = %f m\n", dacData->gpsAltAndSpeed.altitude / 1000.0, dacData->gpsAltAndSpeed.accuracy / 1000.0);
	#endif
}
#endif

/* Decode Extended Frequency Inputs 
 * Input 5 is extended RPM
 */
#if DECODE_EXTENDED_FREQUENCY_INPUTS
void decodeExtendedFrequencyInput(uint8_t data[], DACData *dacData)
{	
	#if DEBUG_DAC_DECODER
	uint8_t channel;
	double periodIntoInt;
	double lowPeriod;
	double highPeriod;
	#endif
	
	dacData->extFrequency[data[0] - EXTENDED_FREQUENCY_CHANNEL_OFFSET].periodIntoInt = getUInt24(&data[1]);
	dacData->extFrequency[data[0] - EXTENDED_FREQUENCY_CHANNEL_OFFSET].lowPeriod = getUInt24(&data[4]);
	dacData->extFrequency[data[0] - EXTENDED_FREQUENCY_CHANNEL_OFFSET].highPeriod = getUInt24(&data[7]);
	
	#if DEBUG_DAC_DECODER
	channel = data[0] - EXTENDED_FREQUENCY_CHANNEL_OFFSET;
	periodIntoInt = (dacData->extFrequency[channel].periodIntoInt * TICK_PERIOD) - 10;
	lowPeriod = dacData->extFrequency[channel].periodIntoInt * TICK_PERIOD;
	highPeriod = dacData->extFrequency[channel].periodIntoInt * TICK_PERIOD;
	
	if (channel == 4)
		printf("Decoding extended rpm input...\n");
	else
	{
		printf("Decoding extended frequency input...\n");
		printf("Input = %d, ", channel + 1);
	}
	printf("Period into 10ms interrupt = %f\n", periodIntoInt);
	printf("Input low period = %f, Input high period = %f\n", lowPeriod, highPeriod);
	#endif
}
#endif

/* When received, this channel signifies that a new data run has
 * started. The data in the message doesn't signify anything. */
#if (DECODE_START_OF_RUN)
void decodeStartOfRunChannel(uint8_t data[], DACData *dacData)
{
	#if (DEBUG_DAC_DECODER)
	printf("A new data run has started...\n");
	#endif
}
#endif

/* Decodes the processed speed data 
 * Speed is in Kph / 0.001379060159 */
#if (DECODE_PROCESSED_SPEED)
void decodeProcessedSpeedData(uint8_t data[], DACData *dacData)
{
	dacData->processedSpeed = getUInt24(&data[1]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding processed speed data...\n");
	printf("Processed Speed = %f kph\n", dacData->processedSpeed * PROCESSED_SPEED_CONST);
	#endif
}
#endif

/* Decode lower and upper check vals for gears 1 to 7
 * Gear is active when RPM x 2/speed(KPH) is between check values */
#if DECODE_GEAR_SETUP_DATA
void decodeGearSetUpData(uint8_t data[], DACData *dacData)
{
	int i;
	for (i = 0; i < 7; ++i)
	{
		dacData->gearCheckVals[i].lower = getUInt16LittleEndian(&data[i * 4 + 1]);
		dacData->gearCheckVals[i].upper = getUInt16LittleEndian(&data[i * 4 + 3]);
	}
	
	#if DEBUG_DAC_DECODER
	printf("Decoding gear setup data...\n");
	for (i = 0; i < 7; ++i)
	{
		printf("Lower Gear %d Check value = %d\n", i + 1, dacData->gearCheckVals[i].lower);
		printf("Upper Gear %d Check value = %d\n", i + 1, dacData->gearCheckVals[i].upper);
	}
	#endif
}
#endif

/* Decode Bargraph set up data 
 * Function doesn't store anything in dacData */
#if (DECODE_BARGRAPH_SETUP_DATA)
void decodeBargraphSetUpData(uint8_t data[], DACData *dacData)
{
	#if (DEBUG_DAC_DECODER)
	int i;
	
	printf("Decoding bargraph setup data...\n");
	for (i = 0; i < 9; ++i)
	{
		double displayVal = (data[i + 1] & 0x0f) + ((data[i + 1] & 0xf0) * 10.0 / 16);
		printf("Display Value %d = %f\n", i + 1, displayVal);
	}
	#endif
}
#endif

/* Decode Dashboard setup data */
#if DECODE_DASHBOARD_SETUP_DATA
void decodeDashboardSetUpData(uint8_t data[], DACData *dacData)
{	
	dacData->dashboardData.warningHoldTime = data[1];
	dacData->dashboardData.sectorHoldTime = data[2];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding dashboard setup data...\n");
	printf("Warning hold time = %d seconds, Sector hold time = %d seconds\n", dacData->dashboardData.warningHoldTime, dacData->dashboardData.sectorHoldTime);
	#endif
}
#endif

/* Decode dashboard setup data 2 */
#if DECODE_DASHBOARD_SETUP_DATA_2
void decodeDashboardSetUpData2(uint8_t data[], DACData *dacData)
{	
	dacData->dashboardData2.highestRPMHoldTime = data[1] * 40;
	dacData->dashboardData2.rpmReductionRate = data[2];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding dashboard setup data 2...\n");
	printf("Highest RPM hold time = %d ms\n", dacData->dashboardData2.highestRPMHoldTime);
	printf("RPM reduction rate (per 40ms) = %d\n", dacData->dashboardData2.rpmReductionRate);
	#endif
}
#endif

/* Decode New Target Sector time */
#if DECODE_NEW_TARGET_SECTOR_TIME
void decodeNewTargetSectorTime(uint8_t data[], DACData *dacData)
{
	int i;
	for (i = 0; i < 10; ++i)
			dacData->sectorTargetTime[i] = getUInt32LittleEndian(&data[i * 4 + 1]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding new target sector time...\n");
	for (i = 0; i < 10; ++i)
		printf("Sector %d target time = %u ms\n", i + 1, dacData->sectorTargetTime[i]);
	#endif
}
#endif

/* Decode New Target Marker time */
#if DECODE_NEW_TARGET_MARKER_TIME
void decodeNewTargetMarkerTime(uint8_t data[], DACData *dacData)
{
	int i;
	for (i = 0; i < 10; ++i)
			dacData->markerTargetTime[i] = getUInt32LittleEndian(&data[i * 4 + 1]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding new target marker time...\n");
	for (i = 0; i < 10; ++i)
		printf("Marker %d target time = %u ms\n", i + 1, dacData->markerTargetTime[i]);
	#endif
}
#endif

/* Decode Auxiliary Input Module Number
 * Module number attached to auxiliary data input */
#if (DECODE_AUXILIARY_MODULE_NUMBER)
void decodeAuxiliaryInputModuleNumber(uint8_t data[], DACData *dacData)
{
	#if (DEBUG_DAC_DECODER)
	printf("Decoding auxiliary input module number...\n");
	switch (data[1])
	{
		case 1:
			printf("Input module is MoTeC M4\n");
			break;
			
		case 2:
			printf("Input module is MBE941\n");
			break;
			
		case 3:
			printf("Input module is IMU06\n");
			break;
			
		case 4:
			printf("Input module is EMS Stinger and 8860\n");
			break;
			
		case 5:
			printf("Input module is Motec M800 Data set 3\n");
			break;
			
		case 6:
			printf("Input module is OMEX 941\n");
			break;
			
		case 7:
			printf("Input module is OBDII interface\n");
			break;
			
		case 8:
			printf("Input module is Pectel\n");
			break;
			
		case 9:
			printf("Input module is Hydra\n");
			break;
			
		case 10:
			printf("Input module is Hondata\n");
			break;
			
		case 11:
			printf("Input module is Vems\n");
			break;
			
		case 12:
			printf("Input module is SMC1\n");
			break;
			
		case 13:
			printf("Input module is Emerald\n");
			break;
			
		case 14:
			printf("Input module is KMS\n");
			break;
		
		default:
			break;
	}
	#endif
}
#endif

/* Decode External Temperature */
#if DECODE_EXT_TEMP
void decodeExternalTemperatureChannel(uint8_t data[], DACData *dacData)
{
	dacData->extTemp.sensorLocation = data[1];
	dacData->extTemp.temp = getInt16LittleEndian(&data[2]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding external temperature channel...\n");
	switch (dacData->extTemp.sensorLocation) 
	{
		case 1:
			printf("Ambient Air Temperature = ");
			break;
		
		case 2:
			printf("Inlet Pre Turbo Temp 1 = ");
			break;
		
		case 3:
			printf("Inlet Pre Turbo Temp 2 = ");
			break;
		
		case 4:
			printf("Inlet Post Turbo Temp 1 = ");
			break;
		
		case 5:
			printf("Inlet Post Turbo Temp 2 = ");
			break;
		
		case 6:
			printf("Inlet Post Intercooler Temp 1 = ");
			break;
		
		case 7:
			printf("Inlet Post Intercooler Temp 2 = ");
			break;
		
		case 8:
			printf("Water Temp = ");
			break;
		
		case 9:
			printf("Oil Temp = ");
			break;
		
		case 10:
			printf("Gearbox Temp = ");
			break;
		
		case 11:
			printf("Gearbox Temp Post Cooler = ");
			break;
		
		case 12:
			printf("Tyre Temp 1 = ");
			break;
		
		case 13:
			printf("Tyre Temp 2 = ");
			break;
		
		case 14:
			printf("Tyre Temp 3 = ");
			break;
		
		case 15:
			printf("Tyre Temp 4 = ");
			break;
		
		case 16:
			printf("ECU Temperature = ");
			break;
		
		case 17:
			printf("Exhaust Temp 1 = ");
			break;
		
		case 18:
			printf("Exhaust Temp 2 = ");
			break;
		
		case 19:
			printf("Exhaust Temp 3 = ");
			break;
		
		case 20:
			printf("Exhaust Temp 4 = ");
			break;
		
		case 21:
			printf("Exhaust Temp 5 = ");
			break;
		
		case 22:
			printf("Exhaust Temp 6 = ");
			break;
		
		case 23:
			printf("Exhaust Temp 7 = ");
			break;
		
		case 24:
			printf("Exhaust Temp 8 = ");
			break;
		
		case 25:
			printf("Auxiliary Temp 1 = ");
			break;
		
		default:
			break;
	}
	
	printf("%f degrees celcius\n", dacData->extTemp.temp / 10.0);
	#endif
}
#endif

/* Decode External Frequency 
 * For more information on sensor locations, see “External ECU interface modules”
 */
#if DECODE_EXT_FREQ
void decodeExternalFrequencyChannel(uint8_t data[], DACData *dacData)
{
	dacData->extFreq.sensorLocation = data[1];
	dacData->extFreq.freq = getUInt16LittleEndian(&data[2]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding external frequency channel...\n");
	switch (dacData->extFreq.sensorLocation) 
	{
		case 1:
			printf("RPM Frequency = ");
			break;
		
		case 2:
			printf("EGR Frequency = ");
			break;
			
		case 3:
			printf("ISBV Frequency = ");
			break;
		
		case 4:
			printf("Nitrous Solenoid Frequency = ");
			break;
			
		default:
			break;
	}
	
	printf("%f Hz\n", dacData->extFreq.freq / 10.0);
	#endif
}
#endif

/* Decode External Percentage */
#if DECODE_EXT_PERCENTAGE
void decodeExternalPercentageChannel(uint8_t data[], DACData *dacData)
{
	dacData->extPercentage.sensorLocation = data[1];
	dacData->extPercentage.percentage = getInt16LittleEndian(&data[2]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding external percentage channel...\n");
	switch (dacData->extPercentage.sensorLocation) 
	{
		case 1:
			printf("Throttle Position Percent = ");
			break;
		
		case 2:
			printf("Lambda 1 Short Term Trim Percent = ");
			break;
		
		case 3:
			printf("Lambda 2 Short Term Trim Percent = ");
			break;
		
		case 4:
			printf("Lambda 1 Long Term Trim Percent = ");
			break;
		
		case 5:
			printf("Lambda 2 Long Term Trim Percent = ");
			break;
		
		case 6:
			printf("Fuel Inj 1 Pulse Width Percent = ");
			break;
		
		case 7:
			printf("Fuel Inj 2 Pulse Width Percent = ");
			break;
		
		case 8:
			printf("Fuel Inj 3 Pulse Width Percent = ");
			break;
		
		case 9:
			printf("Fuel Inj 4 Pulse Width Percent = ");
			break;
		
		case 10:
			printf("Fuel Inj 5 Pulse Width Percent = ");
			break;
		
		case 11:
			printf("Fuel Inj 6 Pulse Width Percent = ");
			break;
		
		case 12:
			printf("Fuel Inj 7 Pulse Width Percent = ");
			break;
		
		case 13:
			printf("Fuel Inj 8 Pulse Width Percent = ");
			break;
		
		case 14:
			printf("Fuel Inj 1 Cut Level Percent = ");
			break;
		
		case 15:
			printf("Fuel Inj 2 Cut Level Percent = ");
			break;
		
		case 16:
			printf("Fuel Inj 3 Cut Level Percent = ");
			break;
		
		case 17:
			printf("Fuel Inj 4 Cut Level Percent = ");
			break;
		
		case 18:
			printf("Fuel Inj 5 Cut Level Percent = ");
			break;
		
		case 19:
			printf("Fuel Inj 6 Cut Level Percent = ");
			break;
		
		case 20:
			printf("Fuel Inj 7 Cut Level Percent = ");
			break;
		
		case 21:
			printf("Fuel Inj 8 Cut Level Percent = ");
			break;
		
		case 22:
			printf("Ignition Cut Level Percent = ");
			break;
		
		case 23:
			printf("ISBV 1 Open Percent = ");
			break;
		
		case 24:
			printf("ISBV 2 Open Percent = ");
			break;
		
		case 25:
			printf("Nitrous Percent = ");
			break;
		
		case 26:
			printf("Auxiliary 1 Percent = ");
			break;
			
		case 27:
			printf("Auxiliary 2 Percent = ");
			break;
			
		case 28:
			printf("Auxiliary 3 Percent = ");
			break;
			
		case 29:
			printf("Auxiliary 4 Percent = ");
			break;
		
		case 30:
			printf("Fuel Aux Temp Comp Percent = ");
			break;
		
		case 31:
			printf("Fuel Aux Volt Comp Percent = ");
			break;
		
		default:
			break;
	}
	
	printf("%f percentage\n", dacData->extPercentage.percentage / 10.0);
	#endif
}
#endif

/* Decode External Time channel */
#if DECODE_EXT_TIME
void decodeExternalTimeChannel(uint8_t data[], DACData *dacData)
{
	dacData->extTime.sensorLocation = data[1];
	dacData->extTime.time = getUInt16LittleEndian(&data[3]);
	
	if (data[2] & 0x80)
		dacData->extTime.exponent = -(256 - data[2]);
	else
		dacData->extTime.exponent = data[2];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding external time channel...\n");
	switch (dacData->extTime.sensorLocation) 
	{
		case 1:
			printf("Injector 1 on time = ");
			break;
		
		case 2:
			printf("Injector 2 on time = ");
			break;
			
		case 3:
			printf("Injector 3 on time = ");
			break;
		
		case 4:
			printf("Injector 4 on time = ");
			break;
		
		case 5:
			printf("Injector 5 on time = ");
			break;
		
		case 6:
			printf("Injector 6 on time = ");
			break;
			
		case 7:
			printf("Injector 7 on time = ");
			break;
		
		case 8:
			printf("Injector 8 on time = ");
			break;
		
		case 9:
			printf("Coil 1 charge time = ");
			break;
		
		case 10:
			printf("Coil 2 charge time = ");
			break;
			
		case 11:
			printf("Coil 3 charge time = ");
			break;
		
		case 12:
			printf("Coil 4 charge time = ");
			break;
		
		default:
			break;
	}
	
	printf("%d x 10^%d ms\n", dacData->extTime.time, dacData->extTime.exponent);
	#endif
}
#endif

/* Decode LCD Data, not implemented */
#if (DECODE_LCD_DATA)
void decodeNewLCDDataChannel(uint8_t data[], DACData *dacData)
{
	#if (DEBUG_DAC_DECODER)
	printf("Decoding new lcd data channel...\n");
	#endif
}
#endif

/* Decode LED Data
 * 1 means it should be on, 0 off
 * Bit 0 General warning LED
 * Bit 1 Lowest shift light LED (shift light 1)
 * Bit 2 Shift light 2
 * Bit 3 Shift light 3
 * Bit 4 Shift light 4
 * Bit 5 Shift light 5
 * Bit 6 Shift light 6 
 */
#if DECODE_LED_DATA
void decodeNewLEDDataChannel(uint8_t data[], DACData *dacData)
{	
	#if DEBUG_DAC_DECODER
	int i;
	#endif
	
	dacData->ledData = data[1];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding new led data channel...\n");
	printf("General warning LED is ");
	if (dacData->ledData & 0x01)
		printf("on\n");
	else
		printf("off\n");
	
	for (i = 1; i < 6; ++i)
	{
		printf("Shift Light LED %d is ", i);
		if (dacData->ledData & (1 << i))
			printf("on\n");
		else
			printf("off\n");
	}
	#endif
}
#endif

/* Decode precalculated distance, in mm */
#if DECODE_PRECALCULATED_DISTANCE
void decodePreCalculatedDistanceDataChannel(uint8_t data[], DACData *dacData)
{
	dacData->precalcDistance = getUInt32(&data[1]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding precalculated distance data channel...\n");
	printf("Distance = %u mm\n", dacData->precalcDistance);
	#endif
}
#endif

/* Decode the yaw rate */
#if DECODE_YAW_RATE
void decodeYawRateChannel(uint8_t data[], DACData *dacData)
{
	dacData->yawRate = 32768 - getUInt16(&data[1]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding yaw rate channel...\n");
	printf("Yaw rate is %f (degrees/s)\n", dacData->yawRate / 100.0);
	#endif
}
#endif

/* Decode calculated yaw */
#if DECODE_CALCULATED_YAW
void decodeCalculatedYawChannel(uint8_t data[], DACData *dacData)
{
	dacData->calculatedYaw.angle = getInt16(&data[1]);
	dacData->calculatedYaw.accuracy = data[3];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding calculated yaw channel...\n");
	printf("Yaw angle is %f degrees, accuracy = %d\n", dacData->calculatedYaw.angle / 100.0, dacData->calculatedYaw.accuracy);
	#endif
}
#endif

/* Decode Pitch Rate */
#if DECODE_PITCH_RATE
void decodePitchRateChannel(uint8_t data[], DACData *dacData)
{
	dacData->pitchRate.rate = 32768 - getUInt16(&data[1]);
	dacData->pitchRate.accuracy = data[3];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding pitch rate channel...\n");
	printf("Pitch rate = %f (degrees/s), accuracy = %d\n", dacData->pitchRate.rate / 100.0, dacData->pitchRate.accuracy);
	#endif
}
#endif

/* Decode Pitch Angle */
#if DECODE_PITCH_ANGLE
void decodePitchAngleChannel(uint8_t data[], DACData *dacData)
{
	dacData->pitchAngle.angle = getInt16(&data[1]);
	dacData->pitchAngle.accuracy = data[3];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding pitch angle channel...\n");
	printf("Pitch angle is %f degrees, accuracy = %d\n", dacData->pitchAngle.angle / 100.0, dacData->pitchAngle.accuracy);
	#endif
}
#endif

/* Decode Roll Rate */
#if DECODE_ROLL_RATE
void decodeRollRateChannel(uint8_t data[], DACData *dacData)
{
	dacData->rollRate.rate = 32768 - getUInt16(&data[1]);
	dacData->rollRate.accuracy = data[3];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding roll rate channel...\n");
	printf("Roll rate = %f (degrees/s), accuracy = %d\n", dacData->rollRate.rate / 100.0, dacData->rollRate.accuracy);
	#endif
}
#endif

/* Decode Rol Angle */
#if DECODE_ROLL_ANGLE
void decodeRollAngleChannel(uint8_t data[], DACData *dacData)
{
	dacData->rollAngle.angle = getInt16(&data[1]);
	dacData->rollAngle.accuracy = data[3];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding roll angle channel...\n");
	printf("Roll angle is %f degrees, accuracy = %d\n", dacData->rollAngle.angle / 100.0, dacData->rollAngle.accuracy);
	#endif
}
#endif

/* Decode Gradient Channel
 * Gradient and accuracy are in degrees * 10^-5
 */
#if DECODE_GRADIENT_CHANNEL
void decodeGradientChannel(uint8_t data[], DACData *dacData)
{
	dacData->gradient.gradient = getInt32(&data[1]);
	dacData->gradient.accuracy = getUInt32(&data[5]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding gradient channel...\n");
	printf("Gradient = %d (degrees x 10^-5), Accuracy = %u (degrees x 10^-5)\n", dacData->gradient.gradient, dacData->gradient.accuracy);
	#endif
}
#endif

/* Decode pulse count */
#if DECODE_PULSE_COUNT
void decodePulseCount(uint8_t data[], DACData *dacData)
{
	dacData->pulseCount[data[0] - PULSE_CHANNEL_OFFSET] = getUInt24(&data[1]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding pulse count channel...\n");
	printf("Pulse Count %d = %d\n", data[0] - PULSE_CHANNEL_OFFSET + 1, dacData->pulseCount[data[0] - PULSE_CHANNEL_OFFSET]);
	#endif
}
#endif

/* Decode baseline 
 * Estimated baseline in mm, Rtk relative accuracy in (mm * 10)
 */
#if DECODE_BASELINE
void decodeBaselineChannel(uint8_t data[], DACData *dacData)
{
	dacData->baseline.baseline = getUInt16(&data[1]);
	dacData->baseline.accuracy = getUInt16(&data[3]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding baseline channel...\n");
	printf("Estimated baseline = %d mm, Rtk relative accuracy = %d (mm * 10)\n", dacData->baseline.baseline, dacData->baseline.accuracy);
	#endif
}
#endif

/* Decode Unit Control Channel*/
#if DECODE_UNIT_CONTROL_CHANNEL
void decodeUnitControlChannel(uint8_t data[], DACData *dacData)
{
	dacData->unitControl.loggerSerialNum = getUInt16(&data[1]);
	dacData->unitControl.command = data[3];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding unit control channel...\n");
	printf("Logger serial number = %d\n", dacData->unitControl.loggerSerialNum);
	switch (dacData->unitControl.command) 
	{
		case 0x01:
			printf("Command = Start Logging\n");
			break;
		
		case 0x02:
			printf("Command = Stop Logging\n");
			break;

		case 0x03:
			printf("Command = Add Marker\n");
			break;

		default:
			printf("Unknown comand\n");
			break;
	}
	#endif
}
#endif

/* Decode the Accelerometer data 
 * Sign Convention
 * Lateral is positive for accelerating upwards (e.g. driving through a dip),
 * negative for accelerating downwards (e.g. driving over a brow). 
 * This results in a stationary and level vertical acceleration of +1g (due to gravity).
 */
#if DECODE_Z_ACCELERATIONS
void decodeZAccelerations(uint8_t data[], DACData *dacData)
{
	dacData->zAccel = getUInt16(&data[1]) & 0x7fff;
	
	if ((data[1] & 0x80) == 0)
		dacData->zAccel = -dacData->zAccel;
	
	#if DEBUG_DAC_DECODER
	printf("Decoding Z-accelerations channel...\n");
	printf("Z-Accel = %f\n", fpToReal(dacData->zAccel));
	#endif
}
#endif

/* Decode External Angle */
#if DECODE_EXT_ANGLE
void decodeExternalAngleChannel(uint8_t data[], DACData *dacData)
{
	dacData->extAngle.sensorLocation = data[1];
	dacData->extAngle.angle = getInt16LittleEndian(&data[2]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding external angle channel...\n");
	switch (dacData->extAngle.sensorLocation)
	{
		case 1:
			printf("Throttle Angle = ");
			break;
		
		case 2:
			printf("Ignition Angle = ");
			break;
		
		case 3:
			printf("Steering Angle = ");
			break;
		
		default:
			break;
	}
	printf("%f degrees\n", dacData->extAngle.angle / 10.0);
	#endif
}
#endif

/* Decode External Pressure */
#if DECODE_EXT_PRESSURE
void decodeExternalPressureChannel(uint8_t data[], DACData *dacData)
{
	dacData->extPressure.sensorLocation = data[1];
	dacData->extPressure.pressure = getUInt16LittleEndian(&data[3]);
	
	if (data[2] & 0x80)
		dacData->extPressure.scalingFactor = -(256 - data[2]);
	else
		dacData->extPressure.scalingFactor = data[2];
	
	#if DEBUG_DAC_DECODER
	printf("Decoding external pressure channel...\n");
	switch (dacData->extPressure.sensorLocation)
	{
		case 1:
			printf("Ambient Air Pressure = ");
			break;
		
		case 2:
			printf("Oil Pressure = ");
			break;
		
		case 3:
			printf("Water Fuel Pressure = ");
			break;
		
		case 4:
			printf("Boost Pressure = ");
			break;

		default:
			break;
	}
	printf("%d x 10^%d mBar\n", dacData->extPressure.pressure, dacData->extPressure.scalingFactor);
	#endif
}
#endif

/* Decode Miscellaneous external data */
#if DECODE_EXT_MISC
void decodeExternalMiscellaneousChannel(uint8_t data[], DACData *dacData)
{
	dacData->extMisc.sensorLocation = data[1];
	dacData->extMisc.data = getUInt16LittleEndian(&data[2]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding external miscellaneous channel...\n");
	switch (dacData->extMisc.sensorLocation) 
	{
		case 1:
			printf("Lambda 1 = ");
			break;
		
		case 2:
			printf("Lambda 2 = ");
			break;
		
		case 3:
			printf("Battery Voltage (v) = ");
			break;
		
		default:
			break;
	}
	printf("%f\n", dacData->extMisc.data / 10.0);
	#endif
}
#endif

/* Decode the time into the lap and sector 
 * Bytes 7 and 8 are reserved for future use.
 */
#if DECODE_TIME_INTO_LAP_AND_SECTOR
void decodeTimeIntoLapAndSectorChannel(uint8_t data[], DACData *dacData)
{
	dacData->timeIntoLapAndSector.timeIntoLap = getUInt24(&data[1]);
	dacData->timeIntoLapAndSector.timeIntoSector = getUInt24(&data[4]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding time into lap and sector channel...\n");
	printf("Time Into Lap = %d\n", dacData->timeIntoLapAndSector.timeIntoLap);
	printf("Time Into Sector = %d\n", dacData->timeIntoLapAndSector.timeIntoSector);
	#endif
}
#endif

/* Decode High Resolution Event Timer
 *
 * For Data1, bits 0 - 2 (the 3 least significant bits) are used to determine which 
 * trigger on a logger the signal has originated from. Trigger numbers are 1-based 
 * on the logger fascias, but zero-based in this message, so:
 * Trigger number = (Data1 & 0x07) + 1
 * Bit 7, the msb, is used to indicate the polarity of the originating event.
 * if bit7 = 0 (i.e. Data1 & 0x80 == 0) then the event was triggered by a falling edge.
 * if bit7 = 1 (i.e. Data1 & 0x80 == 1) then the event was triggered by a rising edge.
 *
 * This channel provides a method of outputting an event time with micro-second 
 * resolution referenced to GPS time.
 */
#if DECODE_HIGH_RESOLUTION_EVENT_TIMER
void decodeHighResolutionEventTimerChannel(uint8_t data[], DACData *dacData)
{
	dacData->eventTimer.eventDetails = data[0];
	dacData->eventTimer.gpsTimeOfWeek = getUInt40(&data[2]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding high resolution event timer channel...\n");
	printf("Trigger Number = %d\n", (dacData->eventTimer.eventDetails & 0x07) + 1);
	
	if (dacData->eventTimer.eventDetails & 0x80)
		printf("Event was triggered by a falling edge.\n");
	else
		printf("Event was triggered by a rising edge.\n");
	
	printf("GPS micro-second time of week = %jd\n", (intmax_t) dacData->eventTimer.gpsTimeOfWeek);
	#endif
}
#endif

/* Decode Sector Definition 
 * NOT IMPLEMENTED */
#if (DECODE_SECTOR_DEFINITION)
void decodeSectorDefinitionChannel(uint8_t data[], DACData *dacData)
{
	#if (DEBUG_DAC_DECODER)
	printf("Decoding sector definition channel...\n");
	#endif
}
#endif

/* Decode BRAKEBOX To PC Communications 
 * NOT IMPLEMENTED */
#if (DECODE_BRAKEBOX_TO_PC)
void decodeBRAKEBOXToPcCommunicationChannel(uint8_t data[], DACData *dacData)
{
	#if (DEBUG_DAC_DECODER)
	printf("Decoding BRAKEBOX to PC communication channel...\n");
	#endif
}
#endif

/* Decode DVR Communictions
 * NOT IMPLEMENTED */
#if (DECODE_DVR_COMMUNICATION)
void decodeDVRCommunicationChannel(uint8_t data[], DACData *dacData)
{
	#if (DEBUG_DAC_DECODER)
	printf("Decoding DVR communication channel...\n");
	#endif
}
#endif

/* Decode Video Frame Index 
 * NOT IMPLEMENTED */
#if (DECODE_VIDEO_FRAME_INDEX)
void decodeVideoFrameIndex(uint8_t data[], DACData *dacData)
{
	#if (DEBUG_DAC_DECODER)
	printf("Decoding video frame index...\n");
	#endif
}
#endif

/* Decode Local North, East and Down Velocities 
 * Values are in (m/s) * 10000
 *
 * This message provides North, East and down velocities at the local Earth 
 * position of the receiver.
 * In all cases checksum is the sum of all bytes including the channel number,
 * modulo 256
 */
#if DECODE_NED_VELOCITIES
void decodeNEDVelocities(uint8_t data[], DACData *dacData)
{
	dacData->nedVelocities.northVelocity = getInt24(&data[1]);
	dacData->nedVelocities.eastVelocity = getInt24(&data[4]);
	dacData->nedVelocities.downVelocity = getInt24(&data[7]);
	
	#if DEBUG_DAC_DECODER
	printf("Decoding NED velocities...\n");
	printf("Local North velocity = %f m/s\n", dacData->nedVelocities.northVelocity / 10000.0);
	printf("Local East velocity = %f m/s\n", dacData->nedVelocities.eastVelocity / 10000.0);
	printf("Local down velocity = %f m/s\n", dacData->nedVelocities.downVelocity / 10000.0);
	#endif
}
#endif

/* Decode General Config message 
 * NOT IMPLEMENTED */
#if (DECODE_GENERAL_CONFIG)
void decodeGeneralConfigChannel(uint8_t data[], DACData *dacData)
{
	#if (DEBUG_DAC_DECODER)
	printf("Decoding general configuration channel...\n");
	#endif
}
#endif

/* Decode General Configuration and reflash message
 * NOT IMPLEMENTED */
#if (DECODE_GENERAL_CONFIG_REFLASH)
void decodeGeneralConfig(uint8_t data[], DACData *dacData)
{
	#if (DEBUG_DAC_DECODER)
	printf("Decoding general configuration and reflash...\n");
	#endif
}
#endif

/* utility functions for interpreting data */

/* interpret as unsigned 16-bit integer, in big-endian format */
uint16_t getUInt16(uint8_t data[])
{
	return (uint16_t) ( (data[0] << 8) + data[1] );
}

/* interpret as unsigned 24-bit integer, in big-endian format */
uint32_t getUInt24(uint8_t data[])
{
	return (uint32_t) ( (data[0] << 16) + (data[1] << 8) + data[2] );
}

/* interpret as unsigned 32-bit integer, in big-endian format */
uint32_t getUInt32(uint8_t data[])
{
	return (uint32_t) ( (data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3] );
}

/* interpret as unsigned 40-bit integer, in big-endian format */
uint64_t getUInt40(uint8_t data[])
{
	/* cast data[0] to a uint64_t to make compiler gives enough room for left shift
	 * NOTE: may have to cast other values to uint32_t if compilers complain
	 */
	return (uint64_t) ( ( ((uint64_t) data[0]) << 32 ) + (data[1] << 24) + (data[2] << 16) + (data[3] << 8) + data[4] );
}

/* interpret as unsigned 16-bit integer, in little-endian format */
uint16_t getUInt16LittleEndian(uint8_t data[])
{
	return (uint16_t) ( data[0] + (data[1] << 8) );
}

/* interpret as unsigned 24-bit integer, in little-endian format */
uint32_t getUInt24LittleEndian(uint8_t data[])
{
	return (uint32_t) ( data[0] + (data[1] << 8) + (data[2] << 16) );
}

/* interpret as unsigned 32-bit integer, in little-endian format */
uint32_t getUInt32LittleEndian(uint8_t data[])
{
	return (uint32_t) ( data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24) );
}

/* interpret as signed 16-bit integer, in big-endian format 
 *
 * Currently using casting methods described in docs, I can
 * always change them to direct casting (if the compiler supports it)
 * or simply sign-extension (like in the signed 24-bit case) 
 * if the current implementation is too slow.
 */
int16_t getInt16(uint8_t data[])
{
	int16_t val = getUInt16(data) & 0x7fff;
	
	if (data[0] & 0x80)
		val = val - 32768;
	
	return val;
}

/* interpret as signed 24-bit integer, in big-endian format */
int32_t getInt24(uint8_t data[])
{
	int32_t val = getUInt24(data) & 0x00ffffff;
	
	if (data[0] & 0x80)
		val = val | 0xff000000;
	
	return val;
}

/* interpret as signed 32-bit integer, in big-endian format */
int32_t getInt32(uint8_t data[])
{
	int32_t val = getUInt32(data) & 0x7fffffff;
	
	if (data[0] & 0x80)
		val = val - (1 << 31);
	
	return val;
}

/* interpret as signed 16-bit integer, in little-endian format */
int16_t getInt16LittleEndian(uint8_t data[])
{
	int16_t val = getUInt16LittleEndian(data) & 0x7fff;
	
	if (data[0] & 0x80)
		val = val - 32768;
	
	return val;
}

/* Encode analogue voltage and store it in the data array.
 *
 * data: the array in which to store the encoded data (should be pre-allocated)
 *       The size of the array should be 4.
 *
 * voltage: the volatage (in milli-volts) to be decoded
 *
 * channel: the physical analogue channel (the one on the outside of the DAC)
 *          should be a value from 1 - 32
 *          The voltage should be in mili-volts 
 *
 * returns -1 on error, otherwise returns the number of bytes encoded (which should be 4)
 */
 
int encodeAnalogueData(uint8_t data[], uint16_t voltage, uint8_t channel)
{
	int numEncoded = -1;
	
	if ((channel >= 1) && (channel <= 32))
	{
		data[0] = intern_analogue_channel[channel - 1] + ANALOGUE_CHANNEL_OFFSET;
		data[1] = (voltage >> 8);
		data[2] = (uint8_t) (voltage & 0xff);
		data[3] = calcCheckSum(data, 3);
		numEncoded = 4;
	}
	
	return numEncoded;
}


/* Initializes the variables and buffer used for decoding.
 * Must be called before calling dac_decode_data() 
 */
void dac_init_buffer()
{
	dbuff.startIndex = 0;
	dbuff.currIndex = 0;
	dbuff.dataLength = 0;
	dbuff.newMessage = 0;
}

/* calculates the checksum of the data 
 * length: the length of the data array
 *         to calculate the checksum over
 * 
 * returns the checksum
 */
uint8_t calcCheckSum(uint8_t data[], int length)
{
	int i;
	uint32_t sum = 0;
	
	for (i = 0; i < length; ++i)
	{
		sum += (data[i] & 0xff);
	}
	
	return (uint8_t) sum;
}

/* checks if the checksum at data[length - 1] is valid 
 * returns 1 if it is, 0 if it isn't
 */
uint8_t isValidCheckSum(uint8_t data[], int length)
{
	return (data[length - 1] == calcCheckSum(data, length - 1));
}

/* Sets the function to call once a message has been decoded.
 *
 * channelDecodedFunc: will be called if an entire message (of a channel) has been decoded.
 */
 
void dac_set_decode_callback(ChannelDecodedFunc *func)
{
	channelDecodedFunc = func;
}

/* Returns already decoded data. */
 
DACData *dac_get_DACData()
{
	return &dacData;
}

/* Adds data to the buffer to be decoded.
 *
 * data: the data to add to the buffer.
 *
 * returns 0 if data is successfully added, 
 * -1 on buffer overflow (meaning some message probably won't be decoded)
 */
int8_t dac_buffer_serial_data(uint8_t data)
{
	int8_t overflow = 0;
	if (dbuff.currIndex == BUFFER_SIZE)
	{
		dbuff.currIndex = 0;
		overflow = -1;
	}
	dbuff.buffer[dbuff.currIndex++] = data;
	
	if (!dbuff.newMessage)
	{
		/* check if there's a new message */
		if (data <= NUM_CHANNELS)
		{
			dbuff.startIndex = dbuff.currIndex - 1;
			dbuff.dataLength = channel_table[data].length;
		
			if (dbuff.dataLength != NOT_USED_LENGTH)
				dbuff.newMessage = 1;
		}
	}
	
	return overflow;
}

/* decode the buffered data between dbuff.startIndex and lastIndex */
uint8_t decodeBufferData(uint16_t lastIndex)
{
	uint8_t messageLeftover = 0;
	
	while ((dbuff.startIndex < lastIndex) && !messageLeftover)
	{
		if (dbuff.buffer[dbuff.startIndex] <= NUM_CHANNELS)
		{
			dbuff.dataLength = channel_table[dbuff.buffer[dbuff.startIndex]].length;
			
			if ((dbuff.dataLength == VARIABLE_LENGTH) && (lastIndex >= dbuff.startIndex + 2))
			{
				dbuff.dataLength = dbuff.buffer[dbuff.startIndex + 1] + 2;
			}
			
			/* if the entire message has already been read then attempt to decode it */
			if ((dbuff.dataLength > 0) && ((lastIndex - dbuff.startIndex) >= dbuff.dataLength))
			{
				if (isValidCheckSum(&(dbuff.buffer[dbuff.startIndex]), dbuff.dataLength))
				{
					if (channel_table[dbuff.buffer[dbuff.startIndex]].decode != NULL)
						channel_table[dbuff.buffer[dbuff.startIndex]].decode(&(dbuff.buffer[dbuff.startIndex]), &dacData);
					
					/* let the caller know what's been decoded */
					if (channelDecodedFunc != NULL)
						channelDecodedFunc(&(dbuff.buffer[dbuff.startIndex]), dbuff.dataLength, &dacData, dbuff.buffer[dbuff.startIndex]);
					dbuff.startIndex = dbuff.startIndex + dbuff.dataLength;
				}
				else
					dbuff.startIndex++;
			}
			else if (((lastIndex - dbuff.startIndex) < dbuff.dataLength) || (dbuff.dataLength == VARIABLE_LENGTH))
			{
				/* if we're still waiting for some bytes then we can't decode anything else */
				messageLeftover = 1;
			}
			else
				dbuff.startIndex++;
		}
		else
			dbuff.startIndex++;
	}
	
	return messageLeftover;
}

/* Decodes the buffered serial data and stores the decoded data in dacData. */ 
void dac_decode_data()
{
	if (dbuff.newMessage)
	{
		if (dbuff.currIndex < dbuff.startIndex)
		{
			decodeBufferData(BUFFER_SIZE);
			dbuff.startIndex = 0;
		}
		dbuff.newMessage = decodeBufferData(dbuff.currIndex);
		if (!dbuff.newMessage)
			dbuff.currIndex = 0;
	}
}

/* gets the user analogue channel (the number on the DAC) from the internal channel
 * number. 
 *
 * channel: the internal channel (should be a value from 0 to 31)
 *          Note: function assumes value is from 0 to 31.
 * returns the user channel (a value from 1 to 32)
 */
uint8_t getUserAnalogueChannel(uint8_t channel)
{
	return extern_analogue_channel[channel];
}
