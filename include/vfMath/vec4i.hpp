#pragma once
#ifndef VF_VEC_4D_INT_HPP
#define VF_VEC_4D_INT_HPP

#include "vec2i.hpp"

/**
 * @brief 4D signed-integer vector
*/
union vec4i
{
	using ElementType = int;

	int v[4];
	__m128i simd;
	struct { int x, y, z, w; };
	struct { unsigned int u_x, u_y, u_z, u_w; };

	// Constructors (Fixes issue with usability)
	constexpr vec4i( const __m128i& vec ) : simd( vec ) {}
	constexpr vec4i( const int x ) : x( x ), y( 0 ), z( 0 ), w( 0 ) {}
	constexpr vec4i( const int x, const int y ) : x( x ), y( y ), z( 0 ), w( 0 ) {}
	constexpr vec4i( const int x, const int y, const int z ) : x( x ), y( y ), z( z ), w( 0 ) {}
	constexpr vec4i( const int x, const int y, const int z, const int w ) : x( x ), y( y ), z( z ), w( w ) {}

	// Array Operators
	constexpr int operator[]( const unsigned char i ) const { return v[i]; }
	constexpr int& operator[]( const unsigned char i ) { return v[i]; }

	// Arithmetic Operators
	vec4i& operator+=( const vec4i& b ) noexcept { simd = _mm_add_epi32( simd, b.simd ); return *this; }
	vec4i& operator-=( const vec4i& b ) noexcept { simd = _mm_sub_epi32( simd, b.simd ); return *this; }
	vec4i& operator*=( const vec4i& b ) noexcept { simd = _mm_mul_epi32( simd, b.simd ); return *this; }
	vec4i& operator/=( const vec4i& b ) noexcept { simd = _mm_div_epi32( simd, b.simd ); return *this; }

	vec4i& operator+=( const int b ) noexcept { simd = _mm_add_epi32( simd, _mm_set1_epi32( b ) ); return *this; }
	vec4i& operator-=( const int b ) noexcept { simd = _mm_sub_epi32( simd, _mm_set1_epi32( b ) ); return *this; }
	vec4i& operator*=( const int b ) noexcept { simd = _mm_mul_epi32( simd, _mm_set1_epi32( b ) ); return *this; }
	vec4i& operator/=( const int b ) noexcept { simd = _mm_div_epi32( simd, _mm_set1_epi32( b ) ); return *this; }

	// Unary Operators
	[[nodiscard]] vec4i operator-() const { return { _mm_mul_epi32( simd, _mm_set1_epi32( -1 ) ) }; }
	[[nodiscard]] vec4i operator~() const { return { _mm_xor_epi32( simd, SIMD_4i_NEG ) }; }

	// Logical Operators
	vec4i& operator&=( const vec4i& b ) noexcept { simd = _mm_and_si128( simd, b.simd ); return *this; }
	vec4i& operator|=( const vec4i& b ) noexcept { simd = _mm_or_si128( simd, b.simd ); return *this; }
	vec4i& operator^=( const vec4i& b ) noexcept { simd = _mm_xor_si128( simd, b.simd ); return *this; }

	vec4i& operator&=( const int b ) noexcept { simd = _mm_and_si128( simd, _mm_set1_epi32( b ) ); return *this; }
	vec4i& operator|=( const int b ) noexcept { simd = _mm_or_si128( simd, _mm_set1_epi32( b ) ); return *this; }
	vec4i& operator^=( const int b ) noexcept { simd = _mm_xor_si128( simd, _mm_set1_epi32( b ) ); return *this; }

	// Conversions
	[[nodiscard]] explicit operator vec2i& ( ) noexcept { return reinterpret_cast<vec2i&>( *this ); }
	[[nodiscard]] explicit operator const vec2i& ( ) const noexcept { return reinterpret_cast<const vec2i&>( *this ); }
	[[nodiscard]] explicit operator vec4f() const;
};

// Arithmetic Operators
[[nodiscard]] static vec4i operator+( const vec4i& a, const vec4i& b ) noexcept { return { _mm_add_epi32( a.simd, b.simd ) }; }
[[nodiscard]] static vec4i operator-( const vec4i& a, const vec4i& b ) noexcept { return { _mm_sub_epi32( a.simd, b.simd ) }; }
[[nodiscard]] static vec4i operator*( const vec4i& a, const vec4i& b ) noexcept { return { _mm_mul_epi32( a.simd, b.simd ) }; }
[[nodiscard]] static vec4i operator/( const vec4i& a, const vec4i& b ) noexcept { return { _mm_div_epi32( a.simd, b.simd ) }; }

[[nodiscard]] static vec4i operator+( const vec4i& a, const int b ) noexcept { return { _mm_add_epi32( a.simd, _mm_set1_epi32( b ) ) }; }
[[nodiscard]] static vec4i operator-( const vec4i& a, const int b ) noexcept { return { _mm_sub_epi32( a.simd, _mm_set1_epi32( b ) ) }; }
[[nodiscard]] static vec4i operator*( const vec4i& a, const int b ) noexcept { return { _mm_mul_epi32( a.simd, _mm_set1_epi32( b ) ) }; }
[[nodiscard]] static vec4i operator/( const vec4i& a, const int b ) noexcept { return { _mm_div_epi32( a.simd, _mm_set1_epi32( b ) ) }; }

[[nodiscard]] static vec4i operator+( const int a, const vec4i& b ) noexcept { return { _mm_add_epi32( _mm_set1_epi32( a ), b.simd ) }; }
[[nodiscard]] static vec4i operator-( const int a, const vec4i& b ) noexcept { return { _mm_sub_epi32( _mm_set1_epi32( a ), b.simd ) }; }
[[nodiscard]] static vec4i operator*( const int a, const vec4i& b ) noexcept { return { _mm_mul_epi32( _mm_set1_epi32( a ), b.simd ) }; }
[[nodiscard]] static vec4i operator/( const int a, const vec4i& b ) noexcept { return { _mm_div_epi32( _mm_set1_epi32( a ), b.simd ) }; }

// Comparison Operators
[[nodiscard]] static vec4i operator>=( const vec4i& a, const vec4i& b ) noexcept { return { _mm_cmpge_epi32_mask( a.simd, b.simd ) }; }
[[nodiscard]] static vec4i operator<=( const vec4i& a, const vec4i& b ) noexcept { return { _mm_cmple_epi32_mask( a.simd, b.simd ) }; }
[[nodiscard]] static vec4i operator>( const vec4i& a, const vec4i& b ) noexcept { return { _mm_cmpgt_epi32_mask( a.simd, b.simd ) }; }
[[nodiscard]] static vec4i operator<( const vec4i& a, const vec4i& b ) noexcept { return { _mm_cmplt_epi32_mask( a.simd, b.simd ) }; }
[[nodiscard]] static vec4i operator&&( const vec4i& a, const vec4i& b ) noexcept { return { _mm_and_si128( a.simd, b.simd ) }; }
[[nodiscard]] static vec4i operator||( const vec4i& a, const vec4i& b ) noexcept { return { _mm_or_si128( a.simd, b.simd ) }; }

[[nodiscard]] static vec4i operator>=( const vec4i& a, const int b ) noexcept { return { _mm_cmpge_epi32_mask( a.simd, _mm_set1_epi32( b ) ) }; }
[[nodiscard]] static vec4i operator<=( const vec4i& a, const int b ) noexcept { return { _mm_cmple_epi32_mask( a.simd, _mm_set1_epi32( b ) ) }; }
[[nodiscard]] static vec4i operator>( const vec4i& a, const int b ) noexcept { return { _mm_cmpgt_epi32_mask( a.simd, _mm_set1_epi32( b ) ) }; }
[[nodiscard]] static vec4i operator<( const vec4i& a, const int b ) noexcept { return { _mm_cmplt_epi32_mask( a.simd, _mm_set1_epi32( b ) ) }; }
[[nodiscard]] static vec4i operator&&( const vec4i& a, const int b ) noexcept { return { _mm_and_si128( a.simd, _mm_set1_epi32( b ) ) }; }
[[nodiscard]] static vec4i operator||( const vec4i& a, const int b ) noexcept { return { _mm_or_si128( a.simd, _mm_set1_epi32( b ) ) }; }

[[nodiscard]] static vec4i operator>=( const int a, const vec4i& b ) noexcept { return { _mm_cmpge_epi32_mask( _mm_set1_epi32( a ), b.simd)}; }
[[nodiscard]] static vec4i operator<=( const int a, const vec4i& b ) noexcept { return { _mm_cmple_epi32_mask( _mm_set1_epi32( a ), b.simd ) }; }
[[nodiscard]] static vec4i operator>( const int a, const vec4i& b ) noexcept { return { _mm_cmpgt_epi32_mask( _mm_set1_epi32( a ), b.simd ) }; }
[[nodiscard]] static vec4i operator<( const int a, const vec4i& b ) noexcept { return { _mm_cmplt_epi32_mask( _mm_set1_epi32( a ), b.simd ) }; }
[[nodiscard]] static vec4i operator&&( const int a, const vec4i& b ) noexcept { return { _mm_and_si128( _mm_set1_epi32( a ), b.simd ) }; }
[[nodiscard]] static vec4i operator||( const int a, const vec4i& b ) noexcept { return { _mm_or_si128( _mm_set1_epi32( a ), b.simd ) }; }

// Logical Operators
[[nodiscard]] static vec4i operator&( const vec4i& a, const vec4i& b ) noexcept { return { _mm_and_si128( a.simd, b.simd ) }; }
[[nodiscard]] static vec4i operator|( const vec4i& a, const vec4i& b ) noexcept { return { _mm_or_si128( a.simd, b.simd ) }; }
[[nodiscard]] static vec4i operator^( const vec4i& a, const vec4i& b ) noexcept { return { _mm_xor_si128( a.simd, b.simd ) }; }

[[nodiscard]] static vec4i operator&( const vec4i& a, const int b ) noexcept { return { _mm_and_si128( a.simd, _mm_set1_epi32( b ) ) }; }
[[nodiscard]] static vec4i operator|( const vec4i& a, const int b ) noexcept { return { _mm_or_si128( a.simd, _mm_set1_epi32( b ) ) }; }
[[nodiscard]] static vec4i operator^( const vec4i& a, const int b ) noexcept { return { _mm_xor_si128( a.simd, _mm_set1_epi32( b ) ) }; }

[[nodiscard]] static vec4i operator&( const int a, const vec4i& b ) noexcept { return { _mm_and_si128( _mm_set1_epi32( a ), b.simd ) }; }
[[nodiscard]] static vec4i operator|( const int a, const vec4i& b ) noexcept { return { _mm_or_si128( _mm_set1_epi32( a ), b.simd ) }; }
[[nodiscard]] static vec4i operator^( const int a, const vec4i& b ) noexcept { return { _mm_xor_si128( _mm_set1_epi32( a ), b.simd ) }; }

typedef vec4i vec3i;

#endif