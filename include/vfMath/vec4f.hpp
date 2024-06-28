#pragma once
#ifndef VF_VEC_4D_FLOAT_HPP
#define VF_VEC_4D_FLOAT_HPP

#include "vec2f.hpp"

#define VECTOR_FORWARD(x) { x }
//#define VECTOR_FORWARD(x) reinterpret_cast<vec4f&&>(std::forward<__m128>(x))

/**
 * @brief 4D floating-point vector
*/
union vec4f
{
	using ElementType = float;
	static constexpr uint32_t ElementSize = 4u;

	__m128 simd;
	float v[4];
	struct { vec2f front, back; };
	struct { float x, y, z, w; };
	struct { float r, g, b, a; };

	// Array Operators
	constexpr float operator[]( uint32_t i ) const { return v[i]; }
	constexpr float& operator[]( uint32_t i ) { return v[i]; }

	// Arithmetic Operators
	constexpr vec4f& operator+=( const vec4f& b ) noexcept { simd = SIMD_4F_ADD( simd, b.simd ); return *this; }
	constexpr vec4f& operator-=( const vec4f& b ) noexcept { simd = SIMD_4F_SUB( simd, b.simd ); return *this; }
	constexpr vec4f& operator*=( const vec4f& b ) noexcept { simd = SIMD_4F_MUL( simd, b.simd ); return *this; }
	constexpr vec4f& operator/=( const vec4f& b ) noexcept { simd = SIMD_4F_DIV( simd, b.simd ); return *this; }
	vec4f& operator%=( const vec4f& b ) noexcept { simd = _mm_fmod_ps( simd, b.simd ); return *this; }

	constexpr vec4f& operator+=( const float b ) noexcept { simd = SIMD_4F_ADD( simd, SIMD_4F_SET1( b ) ); return *this; }
	constexpr vec4f& operator-=( const float b ) noexcept { simd = SIMD_4F_SUB( simd, SIMD_4F_SET1( b ) ); return *this; }
	constexpr vec4f& operator*=( const float b ) noexcept { simd = SIMD_4F_MUL( simd, SIMD_4F_SET1( b ) ); return *this; }
	constexpr vec4f& operator/=( const float b ) noexcept { simd = SIMD_4F_DIV( simd, SIMD_4F_SET1( b ) ); return *this; }
	vec4f& operator%=( const float b ) noexcept { simd = _mm_fmod_ps( simd, SIMD_4F_SET1( b ) ); return *this; }

	// Unary Operators
	[[nodiscard]] constexpr vec4f operator-() const { return VECTOR_FORWARD( SIMD_4F_MUL( simd, SIMD_4f_NEG ) ); }
	[[nodiscard]] constexpr vec4f operator~() const { return VECTOR_FORWARD( SIMD_4F_XOR( simd, SIMD_4f_MASK ) ); }

	// Logical Operators
	constexpr vec4f& operator&=( const vec4f& b ) noexcept { simd = SIMD_4F_AND( simd, b.simd ); return *this; }
	constexpr vec4f& operator|=( const vec4f& b ) noexcept { simd = SIMD_4F_OR( simd, b.simd ); return *this; }
	constexpr vec4f& operator^=( const vec4f& b ) noexcept { simd = SIMD_4F_XOR( simd, b.simd ); return *this; }

	constexpr vec4f& operator&=( const float b ) noexcept { simd = SIMD_4F_AND( simd, SIMD_4F_SET1( b ) ); return *this; }
	constexpr vec4f& operator|=( const float b ) noexcept { simd = SIMD_4F_OR( simd, SIMD_4F_SET1( b ) ); return *this; }
	constexpr vec4f& operator^=( const float b ) noexcept { simd = SIMD_4F_XOR( simd, SIMD_4F_SET1( b ) ); return *this; }

	// Conversions
	[[nodiscard]] explicit constexpr operator bool() const { return x && y && z && w; }
	[[nodiscard]] explicit operator vec2f& ( ) noexcept { return reinterpret_cast<vec2f&>( *this ); }
	[[nodiscard]] explicit operator const vec2f& ( ) const noexcept { return reinterpret_cast<const vec2f&>( *this ); }
	[[nodiscard]] explicit operator vec4i() const;
};

// Arithmetic Operators
[[nodiscard]] constexpr vec4f operator+( const vec4f& a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_ADD( a.simd, b.simd ) ); }
[[nodiscard]] constexpr vec4f operator-( const vec4f& a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_SUB( a.simd, b.simd ) ); }
[[nodiscard]] constexpr vec4f operator*( const vec4f& a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_MUL( a.simd, b.simd ) ); }
[[nodiscard]] constexpr vec4f operator/( const vec4f& a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_DIV( a.simd, b.simd ) ); }
[[nodiscard]] static vec4f operator%( const vec4f& a, const vec4f& b ) noexcept { return VECTOR_FORWARD( _mm_fmod_ps( a.simd, b.simd ) ); }

[[nodiscard]] constexpr vec4f operator+( const vec4f& a, const float b ) noexcept { return VECTOR_FORWARD( SIMD_4F_ADD( a.simd, SIMD_4F_SET1( b ) ) ); }
[[nodiscard]] constexpr vec4f operator-( const vec4f& a, const float b ) noexcept { return VECTOR_FORWARD( SIMD_4F_SUB( a.simd, SIMD_4F_SET1( b ) ) ); }
[[nodiscard]] constexpr vec4f operator*( const vec4f& a, const float b ) noexcept { return VECTOR_FORWARD( SIMD_4F_MUL( a.simd, SIMD_4F_SET1( b ) ) ); }
[[nodiscard]] constexpr vec4f operator/( const vec4f& a, const float b ) noexcept { return VECTOR_FORWARD( SIMD_4F_DIV( a.simd, SIMD_4F_SET1( b ) ) ); }
[[nodiscard]] static vec4f operator%( const vec4f& a, const float b ) noexcept { return VECTOR_FORWARD( _mm_fmod_ps( a.simd, _mm_set1_ps( b ) ) ); }

[[nodiscard]] constexpr vec4f operator+( const float a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_ADD( SIMD_4F_SET1( a ), b.simd ) ); }
[[nodiscard]] constexpr vec4f operator-( const float a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_SUB( SIMD_4F_SET1( a ), b.simd ) ); }
[[nodiscard]] constexpr vec4f operator*( const float a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_MUL( SIMD_4F_SET1( a ), b.simd ) ); }
[[nodiscard]] constexpr vec4f operator/( const float a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_DIV( SIMD_4F_SET1( a ), b.simd ) ); }
[[nodiscard]] static vec4f operator%( const float a, const vec4f& b ) noexcept { return VECTOR_FORWARD( _mm_fmod_ps( _mm_set1_ps( a ), b.simd ) ); }

// Comparison Operators
[[nodiscard]] constexpr vec4f operator>=( const vec4f& a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_CMP_GE( a.simd, b.simd ) ); }
[[nodiscard]] constexpr vec4f operator<=( const vec4f& a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_CMP_LE( a.simd, b.simd ) ); }
[[nodiscard]] constexpr vec4f operator>( const vec4f& a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_CMP_GT( a.simd, b.simd ) ); }
[[nodiscard]] constexpr vec4f operator<( const vec4f& a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_CMP_LT( a.simd, b.simd ) ); }
[[nodiscard]] constexpr vec4f operator&&( const vec4f& a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_AND( a.simd, b.simd ) ); }
[[nodiscard]] constexpr vec4f operator||( const vec4f& a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_OR( a.simd, b.simd ) ); }

[[nodiscard]] constexpr vec4f operator>=( const vec4f& a, const float b ) noexcept { return VECTOR_FORWARD( SIMD_4F_CMP_GE( a.simd, SIMD_4F_SET1( b ) ) ); }
[[nodiscard]] constexpr vec4f operator<=( const vec4f& a, const float b ) noexcept { return VECTOR_FORWARD( SIMD_4F_CMP_LE( a.simd, SIMD_4F_SET1( b ) ) ); }
[[nodiscard]] constexpr vec4f operator>( const vec4f& a, const float b ) noexcept { return VECTOR_FORWARD( SIMD_4F_CMP_GT( a.simd, SIMD_4F_SET1( b ) ) ); }
[[nodiscard]] constexpr vec4f operator<( const vec4f& a, const float b ) noexcept { return VECTOR_FORWARD( SIMD_4F_CMP_LT( a.simd, SIMD_4F_SET1( b ) ) ); }

[[nodiscard]] constexpr vec4f operator>=( const float a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_CMP_GE( SIMD_4F_SET1( a ), b.simd ) ); }
[[nodiscard]] constexpr vec4f operator<=( const float a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_CMP_LE( SIMD_4F_SET1( a ), b.simd ) ); }
[[nodiscard]] constexpr vec4f operator>( const float a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_CMP_GT( SIMD_4F_SET1( a ), b.simd ) ); }
[[nodiscard]] constexpr vec4f operator<( const float a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_CMP_LT( SIMD_4F_SET1( a ), b.simd ) ); }

// Logical Operators
[[nodiscard]] constexpr vec4f operator&( const vec4f& a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_AND( a.simd, b.simd ) ); }
[[nodiscard]] constexpr vec4f operator|( const vec4f& a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_OR( a.simd, b.simd ) ); }
[[nodiscard]] constexpr vec4f operator^( const vec4f& a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_XOR( a.simd, b.simd ) ); }

[[nodiscard]] constexpr vec4f operator&( const vec4f& a, const float b ) noexcept { return VECTOR_FORWARD( SIMD_4F_AND( a.simd, SIMD_4F_SET1( b ) ) ); }
[[nodiscard]] constexpr vec4f operator|( const vec4f& a, const float b ) noexcept { return VECTOR_FORWARD( SIMD_4F_OR( a.simd, SIMD_4F_SET1( b ) ) ); }
[[nodiscard]] constexpr vec4f operator^( const vec4f& a, const float b ) noexcept { return VECTOR_FORWARD( SIMD_4F_XOR( a.simd, SIMD_4F_SET1( b ) ) ); }

[[nodiscard]] constexpr vec4f operator&( const float a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_AND( SIMD_4F_SET1( a ), b.simd ) ); }
[[nodiscard]] constexpr vec4f operator|( const float a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_OR( SIMD_4F_SET1( a ), b.simd ) ); }
[[nodiscard]] constexpr vec4f operator^( const float a, const vec4f& b ) noexcept { return VECTOR_FORWARD( SIMD_4F_XOR( SIMD_4F_SET1( a ), b.simd ) ); }

typedef vec4f vec3f;

#endif