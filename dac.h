/*
 *  dac.h
 *
 *  Author: David Schilling
 *  Date: October 22, 2009
 */

#ifndef DAC_H
#define DAC_H

#include <stdint.h>
#include "dacuser.h"

/* Maximum amount of raw GPS or serial data */
#define MAX_DATA_LENGTH 255

/* the maximum length of a message */
#define MAX_MESSAGE_SIZE 258
/* buffer size is 2 * maximum message length (MAX_MESSAGE_SIZE << 1)*/
#define BUFFER_SIZE 516

#define TICK_PERIOD (1.66666666666667E-07)
#define PROCESSED_SPEED_CONST (0.001379060159)

/* the names of the different channels coming from the DAC */
enum DAC_Channel
{
	RUN_INFO =						1,
	RUN_STATUS =					2,
	RAW_GPS_DATA =					3,
	NEW_SECTOR_TIME =				4,
	NEW_LAP_MARKER =				5,
	LOGGER_STORAGE =				6,
	GPS_TIME_STORAGE =				7,
	ACCELERATIONS =					8,
	TIME_STAMP =					9,
	GPS_POSITION =					10,
	SPEED =							11,
	BEACON_PULSE_PRESENT =			12,
	GPS_PULSE_PRESENT =				13,
	FREQUENCY_FIRST =				14,
	FREQUENCY_LAST =				18,
	RPM =							18,
	SERIAL_DATA =					19,
	ANALOGUE_FIRST =				20,
	ANALOGUE_LAST =					51,
	CHANNEL_DATA =					52,
	DISPLAY_DATA =					53,
	REFLASH_CHANNEL =				54,
	DATE_STORAGE =					55,
	GPS_COURSE =					56,
	GPS_ALTITUDE_AND_SPEED =		57,
	EXTENDED_FREQUENCY_FIRST =		58,
	EXTENDED_FREQUENCY_LAST =		62,
	EXTENDED_RPM =					62,
	START_OF_RUN =					63,
	PROCESSED_SPEED =				64,
	GEAR_SET_UP_DATA =				65,
	BARGRAPH_SET_UP_DATA =			66,
	DASHBOARD_SET_UP_DATA =			67,
	DASHBOARD_SET_UP_DATA_2 =		68,
	NEW_TARGET_SECTOR_TIME =		69,
	NEW_TARGET_MARKER_TIME =		70,
	AUXILIARY_INPUT_MODULE_NUM =	71,
	EXTERNAL_TEMPERATURE =			72,
	EXTERNAL_FREQUENCY =			73,
	EXTERNAL_PERCENTAGE =			74,
	EXTERNAL_TIME =					75,
	NEW_LCD_DATA =					76,
	NEW_LED_DATA =					77,
	PRECALCULATED_DISTANCE =		78,
	YAW_RATE =						79,
	CALCULATED_YAW =				80,
	PITCH_RATE =					81,
	PITCH_ANGLE =					82,
	ROLL_RATE =						83,
	ROLL_ANGLE =					84,
	GRADIENT =						85,
	PULSE_COUNT_FIRST =				86,
	PULSE_COUNT_LAST =				89,
	BASELINE =						90,
	UNIT_CONTROL =					91,
	Z_ACCELERATION =				92,
	EXTERNAL_ANGLE =				93,
	EXTERNAL_PRESSURE =				94,
	EXTERNAL_MISCELLANEOUS =		95,
	TIME_INTO_LAP_AND_SECTOR =		96,
	HIGH_RESOLUTION_EVENT_TIMER =	97,
	SECTOR_DEFINITION =				101,
	BRAKEBOX_TO_PC_COM =			102,
	DVR_COMMUNICATION =				103,
	VIDEO_FRAME_INDEX =				104,
	LOCAL_NED_VELOCITIES =			105,
	GENERAL_CONFIG_MSG =			107,
	GENERAL_CONFIG_AND_REFLASH =	108
};

#if DECODE_RUN_STATUS_MESSAGES
struct StatusMessage
{
	uint8_t startMethod;
	uint8_t stopMethod;
	uint8_t pretriggerLoopExit;
	uint8_t pretriggerTime; /* in minutes */
	uint8_t postTriggerTime; /* in minutes */
	uint8_t autoStartSrc;
	uint8_t autoStopSrc;
	uint16_t lowestBufferVal;
};
#endif

#if (DECODE_RAW_GPS_DATA || DECODE_SERIAL_DATA_INPUT)
struct RawData
{
	uint8_t length;
	uint8_t data[MAX_DATA_LENGTH];
};
#endif

#if DECODE_NEW_SECTOR_TIME
struct SectorTime
{
	/* Definition of Sector Time is messed up. Some parts of
	 * docs (and sample code) say that length is 7 bytes but
	 * decoding requires a length of 12 bytes */
	 
	uint8_t marker; /* Matched Marker, Range 0-9 */
	uint32_t markerTime; /* Time at marker in miliseconds */
	uint8_t sectorNum; /* Matched Sector Number */
	uint32_t sectorTime; /* Sector time in miliseconds */
};
#endif

#if DECODE_NEW_LAP_MARKER
struct LapMarker
{
	uint8_t num; /* the marker number */
	
	/* There should be 18 more bytes of data
	 but I have no clue what they are */
};
#endif

#if DECODE_LOGGER_STORAGE_CHANNEL
struct LoggerStorage
{
	uint16_t serialNumber;
	uint8_t softwareVer;
	uint8_t bootloadVer;
};
#endif

#if DECODE_ACCELERATIONS
struct Accelerations
{
	/* Sign Convention
	 * Longitudinal is positive for acceleration, negative for braking.
	 * Lateral is positive for cornering around a RH bend, negative for cornering
	 * around a LH bend.
     *
	 * Both are in fixed-point 7.8 with 1 sign bit */
	int16_t lateral;
	int16_t longitudinal;
};
#endif

#if DECODE_GPS_POSITIONAL_DATA
struct GPSPosition 
{
	/* to convert longitude and latitude to degrees
	 * multiply by 0.0000001 */
	int32_t longitude;
	int32_t latitude;
	uint32_t accuracy;
};
#endif

#if DECODE_SPEED_DATA
struct Speed
{
	/* in milimeters per second */
	uint32_t speed;
	uint32_t accuracy;
	uint8_t source; /* 0 means DL1, DL2 or AX22 GPS module, 1 means CAN data */
};
#endif

#if DECODE_CHANNEL_DATA_CHANNEL
/* Don't think we need this. Also, don't think I'm decoding it properly */
struct ChannelData
{
	/* Channel being set up */
	uint8_t channel;
	
	/* the following are in PIC floating point format */
	uint32_t a;
	uint32_t b;
	uint32_t c;
	uint32_t d;
	uint32_t upperAlarm;
	uint32_t lowerAlarm;
	
	uint8_t numPlacesAfterDecimal;
	/* also in PIC format */
	uint32_t maxChangeRate;
	uint32_t filterRate;
	
	/* ascii strings */
	char longName[8];
	char shortName[3];
	char unitsName[3];
	
	/* Output = ax^3 + bx^2 + c^x + d 
	 * The change in value between the previous and current value is then 
	 * limited by the value in Maximum rate of change. The filter is then 
	 * applied in the form:
	 * New value = Filter value x current value + (1-filter value) x last value
	 * So a value of 1 puts no filtering on the signal.
	 * Values of 0 in the filter rate and maximum rate of change are ignored.
	 *
	 * NOTE: Floating point values in the following section consists of three 
	 * byte mantissa (LSB first) followed by 1 byte exponent, these are in PIC format.
	 */
};
#endif

#if DECODE_DISPLAY_DATA_CHANNEL
/* Don't think we need this one either */
struct DisplayData
{
	uint8_t menuNum;
	uint8_t topLeftChannel;
	uint8_t topLeftView;
	uint8_t topRightChannel;
	uint8_t topRightView;
	uint8_t bottomLeftChannel;
	uint8_t bottomLeftView;
	uint8_t bottomRightChannel;
	uint8_t bottomRightView;
};
#endif

#if DECODE_DATE_STORAGE_CHANNEL
struct Date
{
	/* current GMT time */
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day;
	uint8_t month;
	uint16_t year;
	
	/* offset from GMT
	 * Offset from GMT is given by 15 minute increments
	 * or decrements.
	 * ex: (-22) = (-5:30 GMT) */
	int8_t offset;
};
#endif

#if DECODE_GPS_COURSE_DATA
struct GPSCourseData
{
	/* heading and heading accuracy in
	 * degrees * 10^-5 */
	int32_t heading;
	uint32_t accuracy;
};
#endif

#if DECODE_GPS_ALT_AND_SPEED
struct GPSAltAndSpeed
{
	/* GPS altitude and speed accuracy in milimeters */
	int32_t altitude;
	int32_t accuracy;
};
#endif

#if DECODE_EXTENDED_FREQUENCY_INPUTS
struct ExtendedFrequency
{
	/* Period into 10ms interrupt = (periodIntoInt * TickPeriod) - 10 */
	uint32_t periodIntoInt;
	/* Input low period = lowPeriod * TickPeriod */
	uint32_t lowPeriod;
	/* Input high period = highPeriod * TickPeriod */
	uint32_t highPeriod;
};
#endif

#if DECODE_GEAR_SETUP_DATA
struct GearCheckVals 
{
	uint16_t upper;
	uint16_t lower;
};
#endif

#if DECODE_DASHBOARD_SETUP_DATA
struct DashboardData
{
	/* both are in seconds */
	uint8_t warningHoldTime;
	uint8_t sectorHoldTime;
};
#endif

#if DECODE_DASHBOARD_SETUP_DATA_2
struct DashboardData2
{
	/* highest RPM hold time in ms */
	uint16_t highestRPMHoldTime;
	/* RPM reduction rate per 40ms */
	uint8_t rpmReductionRate;
};
#endif

#if DECODE_EXT_TEMP
struct ExtTemp 
{
	uint8_t sensorLocation;
	/* temperature in degrees celcius * 10 */
	int16_t temp;
};
#endif

#if DECODE_EXT_FREQ
struct ExtFreq
{
	uint8_t sensorLocation;
	/* Frequency is in Hz * 10 (so deci-Hertz) */
	uint16_t freq;
};
#endif

#if DECODE_EXT_PERCENTAGE
struct ExtPercentage
{
	uint8_t sensorLocation;
	/* percentage is in percentage * 10 */
	int16_t percentage;
};
#endif

#if DECODE_EXT_TIME
struct ExtTime
{
	uint8_t sensorLocation;
	int8_t exponent;
	/* time in ms is time * 10^exponent */
	uint16_t time;
};
#endif

#if (DECODE_CALCULATED_YAW || DECODE_PITCH_ANGLE || DECODE_ROLL_ANGLE)
struct RotationAngle
{
	/* angle in degrees * 100 */
	int16_t angle;
	uint8_t accuracy;
};
#endif

#if (DECODE_PITCH_RATE || DECODE_ROLL_RATE)
struct RotationRate
{
	/* in (degrees/second) * 100 */
	int16_t rate;
	uint8_t accuracy;
};
#endif

#if DECODE_GRADIENT_CHANNEL
struct Gradient
{
	/* gradient and accuracy in
	 * degrees * 10^-5 */
	int32_t gradient;
	uint32_t accuracy;
};
#endif

#if DECODE_BASELINE
struct BaseLine
{
	/* estimated baseline (mm) */
	uint16_t baseline;
	/* Rtk relative accuracy (mm * 10) */
	uint16_t accuracy;
};
#endif

#if DECODE_UNIT_CONTROL_CHANNEL
struct UnitControl
{
	uint16_t loggerSerialNum;
	uint8_t command;
};
#endif

#if DECODE_EXT_ANGLE
struct ExtAngle
{
	uint8_t sensorLocation;
	/* in degrees * 10 */
	int16_t angle;
};
#endif

#if DECODE_EXT_PRESSURE
struct ExtPressure
{
	uint8_t sensorLocation;
	int8_t scalingFactor;
	/* pressure in mBar is pressure * 10 ^ scalingFactor */
	uint16_t pressure;
};
#endif

#if DECODE_EXT_MISC
struct ExtMisc
{
	uint8_t sensorLocation;
	/* multiples of 0.01 */
	uint16_t data;
};
#endif

#if DECODE_TIME_INTO_LAP_AND_SECTOR
struct TimeIntoLapSector
{
	uint32_t timeIntoSector;
	uint32_t timeIntoLap;
};
#endif

#if DECODE_HIGH_RESOLUTION_EVENT_TIMER
/* This channel provides a method of outputting an event time with micro-second 
 * resolution referenced to GPS time. */
struct EventTimer
{	
	/* bits 0 - 2 (the 3 least significant bits) are used to determine which trigger 
	 * on a logger the signal has originated from. Trigger numbers are 1-based on the
	 * logger fascias, but zero-based in this message, so:
	 * Trigger number = (eventDetails & 0x07) + 1 
	 *
	 * Bit 7, the msb, is used to indicate the polarity of the originating event.
	 * if bit7 = 0 (i.e. eventDetails & 0x80 == 0) then the event was triggered by a falling edge.
	 * if bit7 = 1 (i.e. eventDetails & 0x80 == 1) then the event was triggered by a rising edge.
	 */
	uint8_t eventDetails;
	
	/* in micro-seconds */
	uint64_t gpsTimeOfWeek;	
};
#endif

#if DECODE_NED_VELOCITIES
/* This message provides North, East and down 
 * velocities at the local Earth position of the receiver. */
struct NEDVelocities
{
	/* velocities are in (m/s) * 10000 */
	int32_t northVelocity;
	int32_t eastVelocity;
	int32_t downVelocity;
};
#endif

/* structure to store decoded DAC data */
struct DAC_Data
{
	#if DECODE_RUN_STATUS_MESSAGES
	struct StatusMessage statusMsg;
	#endif
	
	#if DECODE_RAW_GPS_DATA
	struct RawData rawGPSData;
	#endif
	
	#if DECODE_NEW_SECTOR_TIME
	struct SectorTime sectTime;
	#endif
	
	#if DECODE_NEW_LAP_MARKER
	struct LapMarker lapMarker;
	#endif
	
	#if DECODE_LOGGER_STORAGE_CHANNEL
	struct LoggerStorage loggerStorage;
	#endif
	
	#if DECODE_GPS_TIME_STORAGE_CHANNEL
	/* GPS ms time of week. Time of week is reset to zero at
	 * midnight between Saturday and Sunday */
	uint32_t gpsTime;
	#endif
	
	#if DECODE_ACCELERATIONS
	struct Accelerations accels;
	#endif
	
	#if DECODE_TIME_STAMP
	uint32_t timeStamp;
	#endif
	
	#if DECODE_GPS_POSITIONAL_DATA
	struct GPSPosition gpsPos;
	#endif
	
	#if DECODE_SPEED_DATA
	struct Speed speed;
	#endif
	
	#if DECODE_BEACON_PULSE_PRESENT
	uint8_t beaconPulseState; /* 1 if it's present, 0 if it's not */
	#endif
	
	#if DECODE_GPS_PULSE_PRESENT
	uint8_t gpsPulseState; /* 1 if it's present, 0 if it's not */
	#endif
	
	#if DECODE_FREQUENCY_INPUT
	/* Actual_Frequency[i] = 1 / (frequency[i] * TICK_PERIOD)
	 * Input 5 (frequency[4]) is RPM */
	uint32_t frequency[5];
	#endif
	
	#if DECODE_SERIAL_DATA_INPUT
	struct RawData serialData;
	#endif
	
	#if DECODE_ANALOGUE_INPUT
	uint16_t analogueVoltage[32]; /* voltage is in milli-volts */
	#endif
	
	#if DECODE_CHANNEL_DATA_CHANNEL
	struct ChannelData channelData;
	#endif
	
	#if DECODE_DISPLAY_DATA_CHANNEL
	struct DisplayData displayData;
	#endif
	
	#if DECODE_DATE_STORAGE_CHANNEL
	struct Date date;
	#endif
	
	#if DECODE_GPS_COURSE_DATA
	struct GPSCourseData gpsCourseData;
	#endif
	
	#if DECODE_GPS_ALT_AND_SPEED
	struct GPSAltAndSpeed gpsAltAndSpeed;
	#endif
	
	#if DECODE_EXTENDED_FREQUENCY_INPUTS
	/* Input 5 (extFrequency[4]) is extended RPM */
	struct ExtendedFrequency extFrequency[5];
	#endif
	
	#if DECODE_PROCESSED_SPEED
	/* to get speed in Kph, multiply by PROCESSED_SPEED_CONST */
	uint32_t processedSpeed;
	#endif
	
	#if DECODE_GEAR_SETUP_DATA
	/* Gear is active when RPM x 2/speed(KPH) is between check values */
	struct GearCheckVals gearCheckVals[7];
	#endif

	#if DECODE_DASHBOARD_SETUP_DATA
	struct DashboardData dashboardData;
	#endif
	
	#if DECODE_DASHBOARD_SETUP_DATA_2
	struct DashboardData2 dashboardData2;
	#endif
	
	#if DECODE_NEW_TARGET_SECTOR_TIME
	uint32_t sectorTargetTime[10];
	#endif
	
	#if DECODE_NEW_TARGET_MARKER_TIME
	uint32_t markerTargetTime[10];
	#endif
	
	#if DECODE_EXT_TEMP
	struct ExtTemp extTemp;
	#endif
	
	#if DECODE_EXT_FREQ
	struct ExtFreq extFreq;
	#endif
	
	#if DECODE_EXT_PERCENTAGE
	struct ExtPercentage extPercentage;
	#endif
	
	#if DECODE_EXT_TIME
	struct ExtTime extTime;
	#endif
	
	#if DECODE_LED_DATA
	/* 1 means it should be on, 0 off
	 * Bit 0 General warning LED
	 * Bit 1 Lowest shift light LED (shift light 1)
	 * Bit 2 Shift light 2
	 * Bit 3 Shift light 3
	 * Bit 4 Shift light 4
	 * Bit 5 Shift light 5
	 * Bit 6 Shift light 6 */
	uint8_t ledData;
	#endif
	
	#if DECODE_PRECALCULATED_DISTANCE
	uint32_t precalcDistance;
	#endif
	
	#if DECODE_YAW_RATE
	/* yaw rate in 100 * degrees/s */
	int16_t yawRate;
	#endif
	
	#if DECODE_CALCULATED_YAW
	struct RotationAngle calculatedYaw;
	#endif
	
	#if DECODE_PITCH_RATE
	struct RotationRate pitchRate;
	#endif
	
	#if DECODE_PITCH_ANGLE
	struct RotationAngle pitchAngle;
	#endif
	
	#if DECODE_ROLL_RATE
	struct RotationRate rollRate;
	#endif
	
	#if DECODE_ROLL_ANGLE
	struct RotationAngle rollAngle;
	#endif
	
	#if DECODE_GRADIENT_CHANNEL
	struct Gradient gradient;
	#endif
	
	#if DECODE_PULSE_COUNT
	uint32_t pulseCount[4];
	#endif
	
	#if DECODE_BASELINE
	struct BaseLine baseline;
	#endif
	
	#if DECODE_UNIT_CONTROL_CHANNEL
	struct UnitControl unitControl;
	#endif
	
	#if DECODE_Z_ACCELERATIONS
	/* Sign Convention
	 * Lateral is positive for accelerating upwards (e.g. driving through a dip),
	 * negative for accelerating downwards (e.g. driving over a brow). 
	 * This results in a stationary and level vertical acceleration of +1g (due to gravity).
	 *
	 * In fixed-point 7.8 with 1 sign bit
	 */
	int16_t zAccel;
	#endif
	
	#if DECODE_EXT_ANGLE
	struct ExtAngle extAngle;
	#endif
	
	#if DECODE_EXT_PRESSURE
	struct ExtPressure extPressure;
	#endif
	
	#if DECODE_EXT_MISC
	struct ExtMisc extMisc;
	#endif
	
	#if DECODE_TIME_INTO_LAP_AND_SECTOR
	struct TimeIntoLapSector timeIntoLapAndSector;
	#endif
	
	#if DECODE_HIGH_RESOLUTION_EVENT_TIMER
	/* This channel provides a method of outputting an event time with micro-second 
	 * resolution referenced to GPS time. */
	struct EventTimer eventTimer;
	#endif
	
	#if DECODE_NED_VELOCITIES
	/* This message provides North, East and down 
	 * velocities at the local Earth position of the receiver. */
	struct NEDVelocities nedVelocities;
	#endif
};

typedef struct DAC_Data DACData;

/* structure used for decoding DAC data
 * Simply call initialize on this object,
 * Do not modify the contents.
 *
 * Structure is only provided to make the decode
 * function more re-entrant.
 */
struct DecodeBuffer 
{
	uint8_t buffer[BUFFER_SIZE];
	uint16_t startIndex;
	uint16_t currIndex;
	int16_t dataLength;
	uint8_t newMessage;
};

typedef struct DecodeBuffer decodeBuffer;


/* A function to be called when a channel message has been decoded.
 *
 * uint8_t[]: the encoded message that was just decoded.
 *       if you would like to store this, you must copy it out right away as it
 *       might be overwritten by the next call to dac_decode_data.
 *
 * int: the length of the array containing the encoded message.
 *
 * DACData*: contains the variables updated by the decodeding.
 *
 * DAC_Channel: the channel that was just decoded.
 */ 
typedef void (ChannelDecodedFunc)(uint8_t[], int length, DACData* dacData, enum DAC_Channel);


/* Initializes the variables and buffer used for decoding. 
 * Must be called before calling dac_decode_data() 
 */
void dac_init_buffer();


/* Decodes the buffered serial data and stores the decoded data in dacData. */ 
void dac_decode_data();

/* Adds data to the buffer to be decoded.
 *
 * data: the data to add to the buffer.
 *
 * returns 0 if data is successfully added, 
 * -1 on buffer overflow (meaning some message probably won't be decoded)
 */
int8_t dac_buffer_serial_data(uint8_t data);

/* Sets the function to call once a message has been decoded.
 *
 * channelDecodedFunc: will be called if an entire message (of a channel) has been decoded.
 */
 
void dac_set_decode_callback(ChannelDecodedFunc *func);

/* Returns already decoded data. */
DACData *dac_get_DACData();


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
int encodeAnalogueData(uint8_t data[], uint16_t voltage, uint8_t channel);

/* gets the user analogue channel (the number on the DAC) from the internal channel
 * number. 
 *
 * channel: the internal channel (should be a value from 0 to 31)
 *          Note: function assumes value is from 0 to 31.
 * returns the user channel (a value from 1 to 32)
 */
uint8_t getUserAnalogueChannel(uint8_t channel);

#endif
