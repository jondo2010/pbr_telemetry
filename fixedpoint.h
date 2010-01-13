/*
 *  fixedpoint.h
 *
 */

#ifndef FIXEDPOINT_H

#include <stdint.h>

/* 16 bit 7.8 with 1 sign bit */

/* fraction portion is 8 bits
 *  so we need to divide or multiply by 2^8 */
#define F 256

static inline int16_t realToFP(double r) { return (int16_t) ( r * F + (r >= 0 ? 0.5 : -.5) ); }
static inline int16_t intToFP(int16_t i) { return i * F; }
static inline double fpToReal(int16_t i) { return (double) i / F; }
static inline int16_t fpToInt(int16_t i) { return i / F; }
static inline int16_t fpToIntRound(int16_t i) { return ( i >= 0 ? ( (i + F / 2) / F ) : ( (i - F / 2) / F ) ); }
static inline int16_t fpAdd(int16_t x, int16_t y) { return x + y; }
static inline int16_t fpSub(int16_t x, int16_t y) { return x - y; }
static inline int16_t fpMul(int16_t x, int16_t y) { return (int16_t) ( ( ( (int32_t) x ) * y ) / F ); }
static inline int16_t fpDiv(int16_t x, int16_t y) { return (int16_t) ( ( ( (int32_t) x ) * F ) / y ); }

#endif
