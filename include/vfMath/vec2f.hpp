#pragma once
#ifndef VF_VEC_2D_FLOAT_HPP
#define VF_VEC_2D_FLOAT_HPP

#include "vec.hpp"

/**
 * @brief 2D floating-point vector
*/
union vec2f
{
	using ElementType = float;

	float v[2];
	__m64 simd;
	struct { float x, y; };

	// Operators
	constexpr float operator[]( const unsigned char i ) const { return v[i]; }
	constexpr float& operator[]( const unsigned char i ) { return v[i]; }
	constexpr vec2f& operator+=( const vec2f b ) noexcept { x += b.x; y += b.y; return *this; }
	constexpr vec2f& operator-=( const vec2f b ) noexcept { x -= b.x; y -= b.y; return *this; }
	constexpr vec2f& operator*=( const vec2f b ) noexcept { x *= b.x; y *= b.y; return *this; }
	constexpr vec2f& operator/=( const vec2f b ) noexcept { x /= b.x; y /= b.y; return *this; }
			  vec2f& operator%=( const vec2f b ) noexcept { x = fmodf( x, b.x ); y = fmodf( y, b.y ); return *this; }
	constexpr vec2f& operator+=( const float b ) noexcept { x += b; y += b; return *this; }
	constexpr vec2f& operator-=( const float b ) noexcept { x -= b; y -= b; return *this; }
	constexpr vec2f& operator*=( const float b ) noexcept { x *= b; y *= b; return *this; }
	constexpr vec2f& operator/=( const float b ) noexcept { x /= b; y /= b; return *this; }
			  vec2f& operator%=( const float b ) noexcept { x = fmodf( x, b ); y = fmodf( y, b ); return *this; }

	[[nodiscard]] constexpr vec2f operator-() const { return vec2f{ -x, -y }; }

	// Conversions
	[[nodiscard]] operator vec4f() const;
	[[nodiscard]] operator vec2i() const;
};

// Arithmetic
[[nodiscard]] constexpr vec2f operator+( const vec2f a, const vec2f b ) noexcept { return { a.x + b.x, a.y + b.y }; }
[[nodiscard]] constexpr vec2f operator-( const vec2f a, const vec2f b ) noexcept { return { a.x - b.x, a.y - b.y }; }
[[nodiscard]] constexpr vec2f operator*( const vec2f a, const vec2f b ) noexcept { return { a.x * b.x, a.y * b.y }; }
[[nodiscard]] constexpr vec2f operator/( const vec2f a, const vec2f b ) noexcept { return { a.x / b.x, a.y / b.y }; }
[[nodiscard]] static	vec2f operator%( const vec2f a, const vec2f b ) noexcept { return { fmodf( a.x, b.x ), fmodf( a.y, b.y ) }; }
[[nodiscard]] constexpr vec2f operator+( const vec2f a, const float b ) noexcept { return { a.x + b, a.y + b }; }
[[nodiscard]] constexpr vec2f operator-( const vec2f a, const float b ) noexcept { return { a.x - b, a.y - b }; }
[[nodiscard]] constexpr vec2f operator*( const vec2f a, const float b ) noexcept { return { a.x * b, a.y * b }; }
[[nodiscard]] constexpr vec2f operator/( const vec2f a, const float b ) noexcept { return { a.x / b, a.y / b }; }
[[nodiscard]] static	vec2f operator%( const vec2f a, const float b ) noexcept { return { fmodf( a.x, b ), fmodf( a.y, b ) }; }
[[nodiscard]] constexpr vec2f operator+( const float a, const vec2f b ) noexcept { return { a + b.x, a + b.y }; }
[[nodiscard]] constexpr vec2f operator-( const float a, const vec2f b ) noexcept { return { a - b.x, a - b.y }; }
[[nodiscard]] constexpr vec2f operator*( const float a, const vec2f b ) noexcept { return { a * b.x, a * b.y }; }
[[nodiscard]] constexpr vec2f operator/( const float a, const vec2f b ) noexcept { return { a / b.x, a / b.y }; }
[[nodiscard]] static	vec2f operator%( const float a, const vec2f b ) noexcept { return { fmodf( a, b.x ), fmodf( a, b.y ) }; }

#endif