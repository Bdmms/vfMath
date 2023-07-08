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
	static constexpr uint32_t ElementSize = 2u;

	float v[2];
	__m64 simd;
	struct { float x, y; };

	// Array Operators
	constexpr float operator[]( const unsigned char i ) const { return v[i]; }
	constexpr float& operator[]( const unsigned char i ) { return v[i]; }

	// Arithmetic Operators
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

	// Unary Operators
	[[nodiscard]] constexpr vec2f operator-() const { return vec2f{ -x, -y }; }
	[[nodiscard]]		    vec2f operator~() const 
	{ 
		int a = ~reinterpret_cast<const int&>( x );
		int b = ~reinterpret_cast<const int&>( y );
		return vec2f{ reinterpret_cast<const float&>( a ), reinterpret_cast<const float&>( b ) };
	}

	// Logical Operators
	vec2f operator&=( const vec2f v ) noexcept 
	{ 
		int a = reinterpret_cast<const int&>( x ) & reinterpret_cast<const int&>( v.x );
		int b = reinterpret_cast<const int&>( y ) & reinterpret_cast<const int&>( v.y );
		x = reinterpret_cast<float&>( a );
		y = reinterpret_cast<float&>( b );
		return *this;
	}

	vec2f operator|=( const vec2f v ) noexcept
	{
		int a = reinterpret_cast<const int&>( x ) | reinterpret_cast<const int&>( v.x );
		int b = reinterpret_cast<const int&>( y ) | reinterpret_cast<const int&>( v.y );
		x = reinterpret_cast<float&>( a );
		y = reinterpret_cast<float&>( b );
		return *this;
	}

	vec2f operator^=( const vec2f v ) noexcept
	{
		int a = reinterpret_cast<const int&>( x ) ^ reinterpret_cast<const int&>( v.x );
		int b = reinterpret_cast<const int&>( y ) ^ reinterpret_cast<const int&>( v.y );
		x = reinterpret_cast<float&>( a );
		y = reinterpret_cast<float&>( b );
		return *this;
	}

	vec2f operator&=( const float v ) noexcept
	{
		int a = reinterpret_cast<const int&>( x ) & reinterpret_cast<const int&>( v );
		int b = reinterpret_cast<const int&>( y ) & reinterpret_cast<const int&>( v );
		x = reinterpret_cast<float&>( a );
		y = reinterpret_cast<float&>( b );
		return *this;
	}

	vec2f operator|=( const float v ) noexcept
	{
		int a = reinterpret_cast<const int&>( x ) | reinterpret_cast<const int&>( v );
		int b = reinterpret_cast<const int&>( y ) | reinterpret_cast<const int&>( v );
		x = reinterpret_cast<float&>( a );
		y = reinterpret_cast<float&>( b );
		return *this;
	}

	vec2f operator^=( const float v ) noexcept
	{
		int a = reinterpret_cast<const int&>( x ) ^ reinterpret_cast<const int&>( v );
		int b = reinterpret_cast<const int&>( y ) ^ reinterpret_cast<const int&>( v );
		x = reinterpret_cast<float&>( a );
		y = reinterpret_cast<float&>( b );
		return *this;
	}

	// Conversions
	[[nodiscard]] explicit constexpr operator bool() const { return x && y; }
	[[nodiscard]] explicit operator vec4f() const;
	[[nodiscard]] explicit operator vec2i() const;
};

// Arithmetic Operators
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

// Comparison Operators
[[nodiscard]] static vec2f operator>=( const vec2f a, const vec2f b ) noexcept 
{ 
	int MASK = 0xFFFFFFFF;
	return { 
		a.x >= b.x ? reinterpret_cast<const float&>( MASK ) : 0.0f,
		a.y >= b.y ? reinterpret_cast<const float&>( MASK ) : 0.0f,
	}; 
}

[[nodiscard]] static vec2f operator<=( const vec2f a, const vec2f b ) noexcept
{
	int MASK = 0xFFFFFFFF;
	return {
		a.x <= b.x ? reinterpret_cast<const float&>( MASK ) : 0.0f,
		a.y <= b.y ? reinterpret_cast<const float&>( MASK ) : 0.0f,
	};
}

[[nodiscard]] static vec2f operator>( const vec2f a, const vec2f b ) noexcept
{
	int MASK = 0xFFFFFFFF;
	return {
		a.x > b.x ? reinterpret_cast<const float&>( MASK ) : 0.0f,
		a.y > b.y ? reinterpret_cast<const float&>( MASK ) : 0.0f,
	};
}

[[nodiscard]] static vec2f operator<( const vec2f a, const vec2f b ) noexcept
{
	int MASK = 0xFFFFFFFF;
	return {
		a.x < b.x ? reinterpret_cast<const float&>( MASK ) : 0.0f,
		a.y < b.y ? reinterpret_cast<const float&>( MASK ) : 0.0f,
	};
}

[[nodiscard]] static vec2f operator&&( const vec2f a, const vec2f b ) noexcept
{
	int MASK = 0xFFFFFFFF;
	return {
		a.x && b.x ? reinterpret_cast<const float&>( MASK ) : 0.0f,
		a.y && b.y ? reinterpret_cast<const float&>( MASK ) : 0.0f,
	};
}

[[nodiscard]] static vec2f operator||( const vec2f a, const vec2f b ) noexcept
{
	int MASK = 0xFFFFFFFF;
	return {
		a.x || b.x ? reinterpret_cast<const float&>( MASK ) : 0.0f,
		a.y || b.y ? reinterpret_cast<const float&>( MASK ) : 0.0f,
	};
}

[[nodiscard]] static vec2f operator>=( const vec2f a, const float b ) noexcept
{
	int MASK = 0xFFFFFFFF;
	return {
		a.x >= b ? reinterpret_cast<const float&>( MASK ) : 0.0f,
		a.y >= b ? reinterpret_cast<const float&>( MASK ) : 0.0f,
	};
}

[[nodiscard]] static vec2f operator<=( const vec2f a, const float b ) noexcept
{
	int MASK = 0xFFFFFFFF;
	return {
		a.x <= b ? reinterpret_cast<const float&>( MASK ) : 0.0f,
		a.y <= b ? reinterpret_cast<const float&>( MASK ) : 0.0f,
	};
}

[[nodiscard]] static vec2f operator>( const vec2f a, const float b ) noexcept
{
	int MASK = 0xFFFFFFFF;
	return {
		a.x > b ? reinterpret_cast<const float&>( MASK ) : 0.0f,
		a.y > b ? reinterpret_cast<const float&>( MASK ) : 0.0f,
	};
}

[[nodiscard]] static vec2f operator<( const vec2f a, const float b ) noexcept
{
	int MASK = 0xFFFFFFFF;
	return {
		a.x < b ? reinterpret_cast<const float&>( MASK ) : 0.0f,
		a.y < b ? reinterpret_cast<const float&>( MASK ) : 0.0f,
	};
}

[[nodiscard]] static vec2f operator>=( const float a, const vec2f b ) noexcept
{
	int MASK = 0xFFFFFFFF;
	return {
		a >= b.x ? reinterpret_cast<const float&>( MASK ) : 0.0f,
		a >= b.y ? reinterpret_cast<const float&>( MASK ) : 0.0f,
	};
}

[[nodiscard]] static vec2f operator<=( const float a, const vec2f b ) noexcept
{
	int MASK = 0xFFFFFFFF;
	return {
		a <= b.x ? reinterpret_cast<const float&>( MASK ) : 0.0f,
		a <= b.y ? reinterpret_cast<const float&>( MASK ) : 0.0f,
	};
}

[[nodiscard]] static vec2f operator>( const float a, const vec2f b ) noexcept
{
	int MASK = 0xFFFFFFFF;
	return {
		a > b.x ? reinterpret_cast<const float&>( MASK ) : 0.0f,
		a > b.y ? reinterpret_cast<const float&>( MASK ) : 0.0f,
	};
}

[[nodiscard]] static vec2f operator<( const float a, const vec2f b ) noexcept
{
	int MASK = 0xFFFFFFFF;
	return {
		a < b.x ? reinterpret_cast<const float&>( MASK ) : 0.0f,
		a < b.y ? reinterpret_cast<const float&>( MASK ) : 0.0f,
	};
}

// Logical Operators
[[nodiscard]] static vec2f operator&( const vec2f a, const vec2f b ) noexcept
{ 
	int x = reinterpret_cast<const int&>( a.x ) & reinterpret_cast<const int&>( b.x );
	int y = reinterpret_cast<const int&>( a.y ) & reinterpret_cast<const int&>( b.y );
	return { reinterpret_cast<float&>( x ), reinterpret_cast<float&>( y ) };
}

[[nodiscard]] static vec2f operator|( const vec2f a, const vec2f b ) noexcept
{
	int x = reinterpret_cast<const int&>( a.x ) | reinterpret_cast<const int&>( b.x );
	int y = reinterpret_cast<const int&>( a.y ) | reinterpret_cast<const int&>( b.y );
	return { reinterpret_cast<float&>( x ), reinterpret_cast<float&>( y ) };
}

[[nodiscard]] static vec2f operator^( const vec2f a, const vec2f b ) noexcept
{
	int x = reinterpret_cast<const int&>( a.x ) ^ reinterpret_cast<const int&>( b.x );
	int y = reinterpret_cast<const int&>( a.y ) ^ reinterpret_cast<const int&>( b.y );
	return { reinterpret_cast<float&>( x ), reinterpret_cast<float&>( y ) };
}

[[nodiscard]] static vec2f operator&( const vec2f a, const float b ) noexcept
{
	int x = reinterpret_cast<const int&>( a.x ) & reinterpret_cast<const int&>( b );
	int y = reinterpret_cast<const int&>( a.y ) & reinterpret_cast<const int&>( b );
	return { reinterpret_cast<float&>( x ), reinterpret_cast<float&>( y ) };
}

[[nodiscard]] static vec2f operator|( const vec2f a, const float b ) noexcept
{
	int x = reinterpret_cast<const int&>( a.x ) | reinterpret_cast<const int&>( b );
	int y = reinterpret_cast<const int&>( a.y ) | reinterpret_cast<const int&>( b );
	return { reinterpret_cast<float&>( x ), reinterpret_cast<float&>( y ) };
}

[[nodiscard]] static vec2f operator^( const vec2f a, const float b ) noexcept
{
	int x = reinterpret_cast<const int&>( a.x ) ^ reinterpret_cast<const int&>( b );
	int y = reinterpret_cast<const int&>( a.y ) ^ reinterpret_cast<const int&>( b );
	return { reinterpret_cast<float&>( x ), reinterpret_cast<float&>( y ) };
}

[[nodiscard]] static vec2f operator&( const float a, const vec2f b ) noexcept
{
	int x = reinterpret_cast<const int&>( a ) & reinterpret_cast<const int&>( b.x );
	int y = reinterpret_cast<const int&>( a ) & reinterpret_cast<const int&>( b.y );
	return { reinterpret_cast<float&>( x ), reinterpret_cast<float&>( y ) };
}

[[nodiscard]] static vec2f operator|( const float a, const vec2f b ) noexcept
{
	int x = reinterpret_cast<const int&>( a ) | reinterpret_cast<const int&>( b.x );
	int y = reinterpret_cast<const int&>( a ) | reinterpret_cast<const int&>( b.y );
	return { reinterpret_cast<float&>( x ), reinterpret_cast<float&>( y ) };
}

[[nodiscard]] static vec2f operator^( const float a, const vec2f b ) noexcept
{
	int x = reinterpret_cast<const int&>( a ) ^ reinterpret_cast<const int&>( b.x );
	int y = reinterpret_cast<const int&>( a ) ^ reinterpret_cast<const int&>( b.y );
	return { reinterpret_cast<float&>( x ), reinterpret_cast<float&>( y ) };
}

#endif