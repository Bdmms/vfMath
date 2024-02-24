#pragma once
#ifndef VF_PRIMITIVES_H
#define VF_PRIMITIVES_H

#include <stdint.h>
#include <bit>

// -----------------------
// --- Type Shorthands ---
// -----------------------

typedef int8_t   i8;
typedef int16_t  i16;
typedef int32_t  i32;
typedef int64_t  i64;

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

// ----------------------------------
// --- Floating-Point Definitions ---
// ----------------------------------

// NON-STANDARD (3/4) smmm ffff (offset=0x03)
typedef unsigned char float8;
// IEEE 754-2008 (5/10) smmm mmff ffff ffff (offset=0x0F)
typedef unsigned short float16;
// IEEE 754-1985 (8/23) smmm mmmm mfff ffff ffff ffff ffff ffff (offset=0x7F)
typedef float float32;
// IEEE 754-1985 (11/52) smmm mmmm mmmm ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff (offset=0x3FF)
typedef double float64;

typedef float8  f8;
typedef float16 f16;
typedef float32 f32;
typedef float64 f64;


constexpr float8 toFloat8( float8 value )
{
	return value;
}

constexpr float8 toFloat8( float16 value )
{
	uint32_t e16 = ( ( value >> 10u ) & 0x1Fu ) - 0x0Fu;
	uint32_t e8 = ( ( e16 + 0x3u ) & 0x7u ) << 4u;
	return ( ( value >> 6u ) & 0xFu ) | e8 | ( ( value >> 8u ) & 0x80u );
}

constexpr float8 toFloat8( float32 value )
{
	uint32_t uVal = std::bit_cast<uint32_t>( value );
	uint32_t e32 = ( ( uVal >> 23u ) & 0xFFu ) - 0x7Fu;
	uint32_t e8 = ( ( e32 + 0x3u ) & 0x7u ) << 4u;
	return ( ( uVal >> 19u ) & 0xFu ) | e8 | ( ( uVal >> 24u ) & 0x80u );
}

constexpr float8 toFloat8( float64 value )
{
	uint64_t uVal = std::bit_cast<uint64_t>( value );
	uint64_t e64 = ( ( uVal >> 52u ) & 0x7FFu ) - 0x3FFu;
	uint64_t e8 = ( ( e64 + 0x3u ) & 0x7u ) << 4u;
	return static_cast<float8>( ( ( uVal >> 48u ) & 0xFu ) | e8 | ( ( uVal >> 56u ) & 0x80u ) );
}

constexpr float16 toFloat16( float8 value )
{
	uint32_t e8 = ( ( value >> 4u ) & 0x7u ) - 0x3u;
	uint32_t e16 = ( ( e8 + 0x0Fu ) & 0x1Fu ) << 10u;
	return ( ( value& 0xFu ) << 6u ) | e8 | ( ( value & 0x80u ) << 8u );
}

constexpr float16 toFloat16( float16 value )
{
	return value;
}

constexpr float16 toFloat16( float32 value )
{
	uint32_t uVal = std::bit_cast<uint32_t>( value );
	uint32_t e32 = ( ( uVal >> 23 ) & 0xFF ) - 0x7F;
	uint32_t e16 = ( ( e32 + 0x0F ) & 0x1F ) << 10;
	return ( ( uVal >> 13u ) & 0x3FFu ) | e16 | ( ( uVal >> 16u ) & 0x8000u );
}

constexpr float16 toFloat16( float64 value )
{
	uint64_t uVal = std::bit_cast<uint64_t>( value );
	uint64_t e64 = ( ( uVal >> 52 ) & 0x7FF ) - 0x3FF;
	uint64_t e16 = ( ( e64 + 0x0F ) & 0x1F ) << 10;
	return static_cast<float16>( ( ( uVal >> 42u ) & 0x3FFu ) | e16 | ( ( uVal >> 48u ) & 0x8000u ) );
}

constexpr float32 toFloat32( float8 value )
{
	uint32_t e8 = ( ( value >> 4u ) & 0x7u ) - 0x3u;
	uint32_t e32 = ( ( e8 + 0x7Fu ) & 0xFFu ) << 23u;
	uint32_t uVal = ( ( value & 0xF ) << 19u ) | e32 | ( ( value & 0x80u ) << 24u );
	return std::bit_cast<float32>( uVal );
}

constexpr float32 toFloat32( float16 value )
{
	uint32_t e16 = ( ( value >> 10u ) & 0x1Fu ) - 0x0Fu;
	uint32_t e32 = ( ( e16 + 0x7Fu ) & 0xFFu ) << 23u;
	uint32_t uVal = ( ( value & 0x3FFu ) << 13u ) | e32 | ( ( value & 0x8000u ) << 16u );
	return std::bit_cast<float32>( uVal );
}

constexpr float32 toFloat32( float32 value )
{
	return value;
}

constexpr float32 toFloat32( float64 value )
{
	return static_cast<float32>( value );
}

constexpr float64 toFloat64( float8 value )
{
	uint64_t fVal = static_cast<uint64_t>( value );
	uint64_t e8 = ( ( fVal >> 4u ) & 0x7u ) - 0x3u;
	uint64_t e64 = ( ( e8 + 0x3FFu ) & 0x7FFu ) << 52u;
	uint64_t uVal = ( ( fVal & 0xF ) << 48u ) | e64 | ( ( fVal & 0x80u ) << 56u );
	return std::bit_cast<float64>( uVal );
}

constexpr float64 toFloat64( float16 value )
{
	uint64_t fVal = static_cast<uint64_t>( value );
	uint64_t e16 = ( ( fVal >> 10u ) & 0x1Fu ) - 0x0Fu;
	uint64_t e64 = ( ( e16 + 0x3FFu ) & 0x7FFu ) << 52u;
	uint64_t uVal = ( ( fVal & 0x3FFu ) << 42u ) | e64 | ( ( fVal & 0x8000u ) << 48u );
	return std::bit_cast<float64>( uVal );
}

constexpr float64 toFloat64( float32 value )
{
	return static_cast<float64>( value );
}

constexpr float64 toFloat64( float64 value )
{
	return value;
}

#endif