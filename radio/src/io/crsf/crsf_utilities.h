/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __UTILITIES_H__  /* Guard against multiple inclusion */
#define __UTILITIES_H__

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <limits.h>
#include <stdbool.h>
#include "crc8.h"

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

#define BITMASK_8( b )                    ( 1 << ( ( b ) % CHAR_BIT ) )
#define BITSLOT( b )                      ( ( b ) / CHAR_BIT )
#define BITSET( a, b )                    ( ( a )[ BITSLOT( b ) ] |= BITMASK_8( b ) )
#define BITCLEAR( a, b )                  ( ( a )[ BITSLOT( b ) ] &= ~BITMASK_8( b ) )
#define BITTEST( a, b )                   ( ( a )[ BITSLOT( b ) ] & BITMASK_8( b ) )
#define BITNSLOTS( nb )                   ( ( nb + CHAR_BIT -1 ) / CHAR_BIT )

#define LIBUTIL_ARRAY_SIZE( array )   ( sizeof( array ) / sizeof( array[ 0 ] ) )

/* ******************** Public Functions and Procedures ******************** */
  extern uint32_t ( *libUtil_systemtime )( void );


  #define LIBUTIL_MS2TICKS( ms )              ( ms )

/* *********************** Write Byte Array Functions ********************** */
void libUtil_ClearBuffer( uint8_t *arrayPointer, uint32_t *increment, uint32_t arraySize );
void libUtil_Write8( uint8_t *arrayPointer, uint32_t *increment, uint8_t inputData );
void libUtil_Write16( uint8_t *arrayPointer, uint32_t *increment, uint16_t inputData );
void libUtil_Write24( uint8_t *arrayPointer, uint32_t *increment, uint32_t inputData );
void libUtil_Write32( uint8_t *arrayPointer, uint32_t *increment, uint32_t inputData );
uint32_t libUtil_WriteString( uint8_t *arrayPointer, uint32_t *increment, char *inputStr, bool nullendbyte );
void libUtil_WriteBytes( uint8_t *out_array, uint32_t *increment, uint8_t *in_array, uint32_t needBytes );
void libUtil_WriteEnd_8( uint8_t *arrayPointer, uint32_t frameLengthAt, uint8_t cmdSize, uint8_t polynom );

/* *********************** Shift Byte Array Functions ********************** */
void libUtil_RightShift( uint8_t *arrayPointer, uint16_t start, uint16_t end, uint16_t right_shift );

/* *********************** Read Byte Array Functions *********************** */
uint8_t libUtil_Read8( uint8_t *arrayPointer, uint32_t *increment );
uint16_t libUtil_Read16( uint8_t *arrayPointer, uint32_t *increment );
uint32_t libUtil_Read24( uint8_t *arrayPointer, uint32_t *increment );
uint32_t libUtil_Read32( uint8_t *arrayPointer, uint32_t *increment );
int8_t libUtil_ReadInt8( uint8_t *arrayPointer, uint32_t *increment );
int16_t libUtil_ReadInt16( uint8_t *arrayPointer, uint32_t *increment );
int32_t libUtil_ReadInt24( uint8_t *arrayPointer, uint32_t *increment );
int32_t libUtil_ReadInt32( uint8_t *arrayPointer, uint32_t *increment );
float libUtil_ReadFloat( uint8_t *arrayPointer, uint32_t *increment, uint8_t decimalPoint );
uint32_t libUtil_ReadString( uint8_t *arrayPointer, uint32_t *increment, char *outputStr, bool skipNullChar );

/* ************************************************************************* */
void libUtil_hex_to_string( uint8_t *hex, uint8_t sizeof_hex, char *separator, bool capital, char *string );
uint8_t libUtil_ReverseUint8( uint8_t data );
uint16_t libUtil_ReverseUint16( uint16_t data );
uint32_t libUtil_ReverseUint32( uint32_t data );
/* ************************************************************************* */

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* __UTILITIES_H__ */

/* ****************************************************************************
 End of File
 */
