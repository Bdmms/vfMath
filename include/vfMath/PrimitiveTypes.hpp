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
struct float8_t
{
	constexpr static uint8_t toFloat8( uint16_t value ) noexcept
	{
		uint32_t e16 = ( ( value >> 10u ) & 0x1Fu );

		if( e16 == 0x00u ) return ( ( value >> 6u ) & 0xFu ) | ( ( value >> 8u ) & 0x80u );
		if( e16 == 0x1Fu ) return ( ( value >> 6u ) & 0xFu ) | 0x70u | ( ( value >> 8u ) & 0x80u );

		uint32_t e8 = ( ( e16 - 0x0Fu + 0x3u ) & 0x7u ) << 4u;
		return ( ( value >> 6u ) & 0xFu ) | e8 | ( ( value >> 8u ) & 0x80u );
	}

	constexpr static uint8_t toFloat8( float value ) noexcept
	{
		uint32_t uVal = std::bit_cast<uint32_t>( value );
		uint32_t e32 = ( ( uVal >> 23u ) & 0xFFu );

		if( e32 == 0x00u ) return ( ( uVal >> 19u ) & 0xFu ) | ( ( uVal >> 24u ) & 0x80u );
		if( e32 == 0xFFu ) return ( ( uVal >> 19u ) & 0xFu ) | 0x70u | ( ( uVal >> 24u ) & 0x80u );

		uint32_t e8 = ( ( e32 - 0x7Fu + 0x3u ) & 0x7u ) << 4u;
		return ( ( uVal >> 19u ) & 0xFu ) | e8 | ( ( uVal >> 24u ) & 0x80u );
	}

	constexpr static uint8_t toFloat8( double value ) noexcept
	{
		uint64_t uVal = std::bit_cast<uint64_t>( value );
		uint64_t e64 = ( ( uVal >> 52u ) & 0x7FFu );

		if( e64 == 0x000u ) return static_cast<uint8_t>( ( ( uVal >> 48u ) & 0xFu ) | ( ( uVal >> 56u ) & 0x80u ) );
		if( e64 == 0x7FFu ) return static_cast<uint8_t>( ( ( uVal >> 48u ) & 0xFu ) | 0x7u | ( ( uVal >> 56u ) & 0x80u ) );

		uint64_t e8 = ( ( e64 - 0x3FFu + 0x3u ) & 0x7u ) << 4u;
		return static_cast<uint8_t>( ( ( uVal >> 48u ) & 0xFu ) | e8 | ( ( uVal >> 56u ) & 0x80u ) );
	}

	// TODO: This assumes long double is always equivalent to double
	constexpr static uint8_t toFloat8( long double value ) noexcept
	{
		uint64_t uVal = std::bit_cast<uint64_t>( value );
		uint64_t e64 = ( ( uVal >> 52u ) & 0x7FFu );

		if( e64 == 0x000u ) return static_cast<uint8_t>( ( ( uVal >> 48u ) & 0xFu ) | ( ( uVal >> 56u ) & 0x80u ) );
		if( e64 == 0x7FFu ) return static_cast<uint8_t>( ( ( uVal >> 48u ) & 0xFu ) | 0x7u | ( ( uVal >> 56u ) & 0x80u ) );

		uint64_t e8 = ( ( e64 - 0x3FFu + 0x3u ) & 0x7u ) << 4u;
		return static_cast<uint8_t>( ( ( uVal >> 48u ) & 0xFu ) | e8 | ( ( uVal >> 56u ) & 0x80u ) );
	}

	constexpr static float8_t fromBits( uint8_t bits ) noexcept
	{
		float8_t copy;
		copy.bits = bits;
		return copy;
	}

	uint8_t bits;

	constexpr float8_t() noexcept : bits( 0u ) {}
	constexpr float8_t( const float8_t& val ) noexcept : bits( val.bits ) { }
	constexpr float8_t( float val ) noexcept : bits( toFloat8( val ) ) { }
	constexpr float8_t( double val ) noexcept : bits( toFloat8( val ) ) { }
	constexpr float8_t( long double val ) noexcept : bits( toFloat8( val ) ) { }

	constexpr float8_t( int8_t val ) noexcept : bits( val ) { }
	constexpr float8_t( int16_t val ) noexcept : bits( toFloat8( static_cast<float>( val ) ) ) { }
	constexpr float8_t( int32_t val ) noexcept : bits( toFloat8( static_cast<float>( val ) ) ) { }
	constexpr float8_t( int64_t val ) noexcept : bits( toFloat8( static_cast<double>( val ) ) ) { }

	constexpr float8_t( uint8_t val ) noexcept : bits( val ) { }
	constexpr float8_t( uint16_t val ) noexcept : bits( toFloat8( static_cast<float>( val ) ) ) { }
	constexpr float8_t( uint32_t val ) noexcept : bits( toFloat8( static_cast<float>( val ) ) ) { }
	constexpr float8_t( uint64_t val ) noexcept : bits( toFloat8( static_cast<double>( val ) ) ) { }

	[[nodiscard]] constexpr float8_t operator-() const noexcept
	{
		return fromBits( bits ^ 0x80 );
	}

	[[nodiscard]] constexpr explicit operator float() const noexcept
	{
		uint32_t e8 = ( ( bits >> 4u ) & 0x7u );

		if( e8 == 0x0u ) return std::bit_cast<float>( ( ( bits & 0xF ) << 19u ) | ( ( bits & 0x80u ) << 24u ) );
		if( e8 == 0x7u ) return std::bit_cast<float>( ( ( bits & 0xF ) << 19u ) | 0x7F800000u | ( ( bits & 0x80u ) << 24u ) );

		uint32_t e32 = ( ( e8 - 0x3u + 0x7Fu ) & 0xFFu ) << 23u;
		return std::bit_cast<float>( ( ( bits & 0xF ) << 19u ) | e32 | ( ( bits & 0x80u ) << 24u ) );
	}

	[[nodiscard]] constexpr explicit operator double() const noexcept
	{
		uint64_t fVal = static_cast<uint64_t>( bits );
		uint64_t e8 = ( ( fVal >> 4u ) & 0x7u );

		if( e8 == 0x0u ) return std::bit_cast<double>( ( ( fVal & 0xF ) << 48u ) | ( ( fVal & 0x80u ) << 56u ) );
		if( e8 == 0x7u ) return std::bit_cast<double>( ( ( fVal & 0xF ) << 48u ) | 0x7FF0000000000000u | ( ( fVal & 0x80u ) << 56u ) );

		uint64_t e64 = ( ( e8 - 0x3u + 0x3FFu ) & 0x7FFu ) << 52u;
		return std::bit_cast<double>( ( ( fVal & 0xF ) << 48u ) | e64 | ( ( fVal & 0x80u ) << 56u ) );
	}

	// TODO: This assumes long double is always equivalent to double
	[[nodiscard]] constexpr explicit operator long double() const noexcept
	{
		uint64_t fVal = static_cast<uint64_t>( bits );
		uint64_t e8 = ( ( fVal >> 4u ) & 0x7u );

		if( e8 == 0x0u ) return std::bit_cast<double>( ( ( fVal & 0xF ) << 48u ) | ( ( fVal & 0x80u ) << 56u ) );
		if( e8 == 0x7u ) return std::bit_cast<double>( ( ( fVal & 0xF ) << 48u ) | 0x7FF0000000000000u | ( ( fVal & 0x80u ) << 56u ) );

		uint64_t e64 = ( ( e8 - 0x3u + 0x3FFu ) & 0x7FFu ) << 52u;
		return std::bit_cast<double>( ( ( fVal & 0xF ) << 48u ) | e64 | ( ( fVal & 0x80u ) << 56u ) );
	}

	[[nodiscard]] constexpr explicit operator int8_t() const noexcept { return static_cast<int8_t>( operator float() ); }
	[[nodiscard]] constexpr explicit operator int16_t() const noexcept { return static_cast<int16_t>( operator float() ); }
	[[nodiscard]] constexpr explicit operator int32_t() const noexcept { return static_cast<int32_t>( operator float() ); }
	[[nodiscard]] constexpr explicit operator int64_t() const noexcept { return static_cast<int64_t>( operator double() ); }
	[[nodiscard]] constexpr explicit operator uint8_t() const noexcept { return static_cast<uint8_t>( operator float() ); }
	[[nodiscard]] constexpr explicit operator uint16_t() const noexcept { return static_cast<uint16_t>( operator float() ); }
	[[nodiscard]] constexpr explicit operator uint32_t() const noexcept { return static_cast<uint32_t>( operator float() ); }
	[[nodiscard]] constexpr explicit operator uint64_t() const noexcept { return static_cast<uint64_t>( operator double() ); }
};

// IEEE 754-2008 (5/10) smmm mmff ffff ffff (offset=0x0F)
struct float16_t
{
	constexpr static uint16_t toFloat16( uint8_t value ) noexcept
	{
		uint32_t e8 = ( ( value >> 4u ) & 0x7u );

		if( e8 == 0x00u ) return ( ( value & 0xFu ) << 6u ) | ( ( value & 0x80u ) << 8u );
		if( e8 == 0x1Fu ) return ( ( value & 0xFu ) << 6u ) | 0x7C00u | ( ( value & 0x80u ) << 8u );

		uint32_t e16 = ( ( e8 - 0x3u + 0x0Fu ) & 0x1Fu ) << 10u;
		return ( ( value & 0xFu ) << 6u ) | e8 | ( ( value & 0x80u ) << 8u );
	}

	constexpr static uint16_t toFloat16( float value ) noexcept
	{
		uint32_t uVal = std::bit_cast<uint32_t>( value );
		uint32_t e32 = ( ( uVal >> 23 ) & 0xFF );

		if( e32 == 0x00u ) return ( ( uVal >> 13u ) & 0x3FFu ) | ( ( uVal >> 16u ) & 0x8000u );
		if( e32 == 0xFFu ) return ( ( uVal >> 13u ) & 0x3FFu ) | 0x7C00u | ( ( uVal >> 16u ) & 0x8000u );

		uint32_t e16 = ( ( e32 - 0x7F + 0x0F ) & 0x1F ) << 10;
		return ( ( uVal >> 13u ) & 0x3FFu ) | e16 | ( ( uVal >> 16u ) & 0x8000u );
	}

	constexpr static uint16_t toFloat16( double value ) noexcept
	{
		uint64_t uVal = std::bit_cast<uint64_t>( value );
		uint64_t e64 = ( ( uVal >> 52 ) & 0x7FF );

		if( e64 == 0x000u ) return static_cast<uint16_t>( ( ( uVal >> 42u ) & 0x3FFu ) | ( ( uVal >> 48u ) & 0x8000u ) );
		if( e64 == 0x7FFu ) return static_cast<uint16_t>( ( ( uVal >> 42u ) & 0x3FFu ) | 0x7C00u | ( ( uVal >> 48u ) & 0x8000u ) );

		uint64_t e16 = ( ( e64 - 0x3FF + 0x0F ) & 0x1F ) << 10;
		return static_cast<uint16_t>( ( ( uVal >> 42u ) & 0x3FFu ) | e16 | ( ( uVal >> 48u ) & 0x8000u ) );
	}

	// TODO: This assumes long double is always equivalent to double
	constexpr static uint16_t toFloat16( long double value ) noexcept
	{
		uint64_t uVal = std::bit_cast<uint64_t>( value );
		uint64_t e64 = ( ( uVal >> 52 ) & 0x7FF );

		if( e64 == 0x000u ) return static_cast<uint16_t>( ( ( uVal >> 42u ) & 0x3FFu ) | ( ( uVal >> 48u ) & 0x8000u ) );
		if( e64 == 0x7FFu ) return static_cast<uint16_t>( ( ( uVal >> 42u ) & 0x3FFu ) | 0x7C00u | ( ( uVal >> 48u ) & 0x8000u ) );

		uint64_t e16 = ( ( e64 - 0x3FF + 0x0F ) & 0x1F ) << 10;
		return static_cast<uint16_t>( ( ( uVal >> 42u ) & 0x3FFu ) | e16 | ( ( uVal >> 48u ) & 0x8000u ) );
	}

	constexpr static float16_t fromBits( uint16_t bits ) noexcept
	{
		float16_t copy;
		copy.bits = bits;
		return copy;
	}

	uint16_t bits;

	constexpr float16_t() noexcept : bits( 0u ) {}
	constexpr float16_t( float8_t val ) noexcept : bits( toFloat16( val.bits ) ) { }
	constexpr float16_t( const float16_t& val ) noexcept : bits( val.bits ) { }
	constexpr float16_t( float val ) noexcept : bits( toFloat16( val ) ) { }
	constexpr float16_t( double val ) noexcept : bits( toFloat16( val ) ) { }
	constexpr float16_t( long double val ) noexcept : bits( toFloat16( val ) ) { }

	constexpr float16_t( int8_t val ) noexcept : bits( val ) { }
	constexpr float16_t( int16_t val ) noexcept : bits( toFloat16( static_cast<float>( val ) ) ) { }
	constexpr float16_t( int32_t val ) noexcept : bits( toFloat16( static_cast<float>( val ) ) ) { }
	constexpr float16_t( int64_t val ) noexcept : bits( toFloat16( static_cast<double>( val ) ) ) { }

	constexpr float16_t( uint8_t val ) noexcept : bits( val ) { }
	constexpr float16_t( uint16_t val ) noexcept : bits( toFloat16( static_cast<float>( val ) ) ) { }
	constexpr float16_t( uint32_t val ) noexcept : bits( toFloat16( static_cast<float>( val ) ) ) { }
	constexpr float16_t( uint64_t val ) noexcept : bits( toFloat16( static_cast<double>( val ) ) ) { }

	[[nodiscard]] constexpr float16_t operator-() const noexcept
	{ 
		return float16_t::fromBits( bits ^ 0x8000 );
	}

	[[nodiscard]] constexpr explicit operator float8_t() const noexcept
	{
		return float8_t::fromBits( float8_t::toFloat8( bits ) );
	}

	[[nodiscard]] constexpr explicit operator float() const noexcept
	{
		uint32_t e16 = ( ( bits >> 10u ) & 0x1Fu );

		if( e16 == 0x00u ) return std::bit_cast<float>( ( ( bits & 0x3FFu ) << 13u ) | ( ( bits & 0x8000u ) << 16u ) );
		if( e16 == 0x1Fu ) return std::bit_cast<float>( ( ( bits & 0x3FFu ) << 13u ) | 0x7F800000u | ( ( bits & 0x8000u ) << 16u ) );

		uint32_t e32 = ( ( e16 - 0x0Fu + 0x7Fu ) & 0xFFu ) << 23u;
		return std::bit_cast<float>( ( ( bits & 0x3FFu ) << 13u ) | e32 | ( ( bits & 0x8000u ) << 16u ) );
	}

	[[nodiscard]] constexpr explicit operator double() const noexcept
	{
		uint64_t fVal = static_cast<uint64_t>( bits );
		uint64_t e16 = ( ( fVal >> 10u ) & 0x1Fu ) - 0x0Fu;

		if( e16 == 0x00u ) return std::bit_cast<double>( ( ( fVal & 0x3FFu ) << 42u ) | ( ( fVal & 0x8000u ) << 48u ) );
		if( e16 == 0x1Fu ) return std::bit_cast<double>( ( ( fVal & 0x3FFu ) << 42u ) | 0x7FF0000000000000u | ( ( fVal & 0x8000u ) << 48u ) );

		uint64_t e64 = ( ( e16 + 0x3FFu ) & 0x7FFu ) << 52u;
		return std::bit_cast<double>( ( ( fVal & 0x3FFu ) << 42u ) | e64 | ( ( fVal & 0x8000u ) << 48u ) );
	}

	// TODO: This assumes long double is always equivalent to double
	[[nodiscard]] constexpr explicit operator long double() const noexcept
	{
		uint64_t fVal = static_cast<uint64_t>( bits );
		uint64_t e16 = ( ( fVal >> 10u ) & 0x1Fu ) - 0x0Fu;

		if( e16 == 0x00u ) return std::bit_cast<double>( ( ( fVal & 0x3FFu ) << 42u ) | ( ( fVal & 0x8000u ) << 48u ) );
		if( e16 == 0x1Fu ) return std::bit_cast<double>( ( ( fVal & 0x3FFu ) << 42u ) | 0x7FF0000000000000u | ( ( fVal & 0x8000u ) << 48u ) );

		uint64_t e64 = ( ( e16 + 0x3FFu ) & 0x7FFu ) << 52u;
		return std::bit_cast<double>( ( ( fVal & 0x3FFu ) << 42u ) | e64 | ( ( fVal & 0x8000u ) << 48u ) );
	}

	[[nodiscard]] constexpr explicit operator int8_t() const noexcept { return static_cast<int8_t>( operator float() ); }
	[[nodiscard]] constexpr explicit operator int16_t() const noexcept { return static_cast<int16_t>( operator float() ); }
	[[nodiscard]] constexpr explicit operator int32_t() const noexcept { return static_cast<int32_t>( operator float() ); }
	[[nodiscard]] constexpr explicit operator int64_t() const noexcept { return static_cast<int64_t>( operator double() ); }
	[[nodiscard]] constexpr explicit operator uint8_t() const noexcept { return static_cast<uint8_t>( operator float() ); }
	[[nodiscard]] constexpr explicit operator uint16_t() const noexcept { return static_cast<uint16_t>( operator float() ); }
	[[nodiscard]] constexpr explicit operator uint32_t() const noexcept { return static_cast<uint32_t>( operator float() ); }
	[[nodiscard]] constexpr explicit operator uint64_t() const noexcept { return static_cast<uint64_t>( operator double() ); }
};

// IEEE 754-1985 (8/23) smmm mmmm mfff ffff ffff ffff ffff ffff (offset=0x7F)
typedef float float32_t;
// IEEE 754-1985 (11/52) smmm mmmm mmmm ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff ffff (offset=0x3FF)
typedef double float64_t;

typedef float8_t  f8;
typedef float16_t f16;
typedef float32_t f32;
typedef float64_t f64;

// 8-bit Float Literal
constexpr float8_t operator ""_f8( long double value )
{
	return static_cast<float8_t>( value );
}

// 16-bit Float Literal
constexpr float16_t operator ""_f16( long double value )
{
	return static_cast<float16_t>( value );
}

#endif