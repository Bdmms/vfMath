#pragma once
#ifndef VF_VECTOR_2D_HPP
#define VF_VECTOR_2D_HPP

#include "vec.hpp"

/**
 * @brief 2D vector
*/
template<typename T>
union vec2
{
	using ElementType = T;
	static constexpr size_t ElementSize = 2LLU;

	struct { T x, y; };
	T v[2];

	// Array Operators
	constexpr T operator[]( size_t i ) const { return v[i]; }
	constexpr T& operator[]( size_t i ) { return v[i]; }

	// Arithmetic Operators
	constexpr vec2<T>& operator+=( const vec2<T> b ) noexcept { x += b.x; y += b.y; return *this; }
	constexpr vec2<T>& operator-=( const vec2<T> b ) noexcept { x -= b.x; y -= b.y; return *this; }
	constexpr vec2<T>& operator*=( const vec2<T> b ) noexcept { x *= b.x; y *= b.y; return *this; }
	constexpr vec2<T>& operator/=( const vec2<T> b ) noexcept { x /= b.x; y /= b.y; return *this; }
	constexpr vec2<T>& operator%=( const vec2<T> b ) noexcept { x = vf::primitives::modulo( x, b.x ); y = vf::primitives::modulo( y, b.y ); return *this; }

	constexpr vec2<T>& operator+=( T b ) noexcept { x += b; y += b; return *this; }
	constexpr vec2<T>& operator-=( T b ) noexcept { x -= b; y -= b; return *this; }
	constexpr vec2<T>& operator*=( T b ) noexcept { x *= b; y *= b; return *this; }
	constexpr vec2<T>& operator/=( T b ) noexcept { x /= b; y /= b; return *this; }
	constexpr vec2<T>& operator%=( T b ) noexcept { x = vf::primitives::modulo( x, b ); y = vf::primitives::modulo( y, b ); return *this; }

	// Unary Operators
	[[nodiscard]] constexpr vec2<T> operator-() const { return vec2<T>{ -x, -y }; }
	[[nodiscard]] constexpr vec2<T> operator~() const { return vec2<T>{ vf::primitives::bitwise_not( x ), vf::primitives::bitwise_not( y ) }; }

	// Logical Operators
	constexpr vec2<T> operator&=( const vec2<T> v ) noexcept
	{
		x = vf::primitives::bitwise_and( x, v.x );
		y = vf::primitives::bitwise_and( y, v.y );
		return *this;
	}

	constexpr vec2<T> operator|=( const vec2<T> v ) noexcept
	{
		x = vf::primitives::bitwise_or( x, v.x );
		y = vf::primitives::bitwise_or( y, v.y );
		return *this;
	}

	constexpr vec2<T> operator^=( const vec2<T> v ) noexcept
	{
		x = vf::primitives::bitwise_xor( x, v.x );
		y = vf::primitives::bitwise_xor( y, v.y );
		return *this;
	}

	constexpr vec2<T> operator&=( T v ) noexcept
	{
		x = vf::primitives::bitwise_and( x, v );
		y = vf::primitives::bitwise_and( y, v );
		return *this;
	}

	constexpr vec2<T> operator|=( T v ) noexcept
	{
		x = vf::primitives::bitwise_or( x, v );
		y = vf::primitives::bitwise_or( y, v );
		return *this;
	}

	constexpr vec2<T> operator^=( T v ) noexcept
	{
		x = vf::primitives::bitwise_xor( x, v );
		y = vf::primitives::bitwise_xor( y, v );
		return *this;
	}

	// Conversions
	[[nodiscard]] explicit constexpr operator bool() const { return x && y; }

	template<typename T>
	constexpr explicit operator vec2<T>() const
	{
		return { static_cast<T>( x ), static_cast<T>( y ) };
	};
};

// Arithmetic Operators
template<typename T> constexpr static vec2<T> operator+( const vec2<T> a, const vec2<T> b ) noexcept { return { a.x + b.x, a.y + b.y }; }
template<typename T> constexpr static vec2<T> operator-( const vec2<T> a, const vec2<T> b ) noexcept { return { a.x - b.x, a.y - b.y }; }
template<typename T> constexpr static vec2<T> operator*( const vec2<T> a, const vec2<T> b ) noexcept { return { a.x * b.x, a.y * b.y }; }
template<typename T> constexpr static vec2<T> operator/( const vec2<T> a, const vec2<T> b ) noexcept { return { a.x / b.x, a.y / b.y }; }
template<typename T> constexpr static vec2<T> operator%( const vec2<T> a, const vec2<T> b ) noexcept { return { vf::primitives::modulo( a.x, b.x ), vf::primitives::modulo( a.y, b.y ) }; }

template<typename T> constexpr static vec2<T> operator+( const vec2<T> a, const T b ) noexcept { return { a.x + b, a.y + b }; }
template<typename T> constexpr static vec2<T> operator-( const vec2<T> a, const T b ) noexcept { return { a.x - b, a.y - b }; }
template<typename T> constexpr static vec2<T> operator*( const vec2<T> a, const T b ) noexcept { return { a.x * b, a.y * b }; }
template<typename T> constexpr static vec2<T> operator/( const vec2<T> a, const T b ) noexcept { return { a.x / b, a.y / b }; }
template<typename T> constexpr static vec2<T> operator%( const vec2<T> a, const T b ) noexcept { return { vf::primitives::modulo( a.x, b ), vf::primitives::modulo( a.y, b ) }; }

template<typename T> constexpr static vec2<T> operator+( const T a, const vec2<T> b ) noexcept { return { a + b.x, a + b.y }; }
template<typename T> constexpr static vec2<T> operator-( const T a, const vec2<T> b ) noexcept { return { a - b.x, a - b.y }; }
template<typename T> constexpr static vec2<T> operator*( const T a, const vec2<T> b ) noexcept { return { a * b.x, a * b.y }; }
template<typename T> constexpr static vec2<T> operator/( const T a, const vec2<T> b ) noexcept { return { a / b.x, a / b.y }; }
template<typename T> constexpr static vec2<T> operator%( const T a, const vec2<T> b ) noexcept { return { vf::primitives::modulo( a, b.x ), vf::primitives::modulo( a, b.y ) }; }

// Comparison Operators
template<typename T> constexpr vec2<T> operator>=( const vec2<T> a, const vec2<T> b ) noexcept
{
	return {
		a.x >= b.x ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
		a.y >= b.y ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
	};
}

template<typename T> constexpr vec2<T> operator<=( const vec2<T> a, const vec2<T> b ) noexcept
{
	return {
		a.x <= b.x ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
		a.y <= b.y ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
	};
}

template<typename T> constexpr vec2<T> operator>( const vec2<T> a, const vec2<T> b ) noexcept
{
	return {
		a.x > b.x ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
		a.y > b.y ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
	};
}

template<typename T> constexpr vec2<T> operator<( const vec2<T> a, const vec2<T> b ) noexcept
{
	return {
		a.x < b.x ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
		a.y < b.y ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
	};
}

template<typename T> constexpr vec2<T> operator&&( const vec2<T> a, const vec2<T> b ) noexcept
{
	return {
		a.x && b.x ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
		a.y && b.y ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
	};
}

template<typename T> constexpr vec2<T> operator||( const vec2<T> a, const vec2<T> b ) noexcept
{
	return {
		a.x || b.x ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
		a.y || b.y ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
	};
}

template<typename T> constexpr vec2<T> operator>=( const vec2<T> a, const T b ) noexcept
{
	return {
		a.x >= b ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
		a.y >= b ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
	};
}

template<typename T> constexpr vec2<T> operator<=( const vec2<T> a, const T b ) noexcept
{
	return {
		a.x <= b ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
		a.y <= b ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
	};
}

template<typename T> constexpr vec2<T> operator>( const vec2<T> a, const T b ) noexcept
{
	return {
		a.x > b ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
		a.y > b ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
	};
}

template<typename T> constexpr vec2<T> operator<( const vec2<T> a, const T b ) noexcept
{
	return {
		a.x < b ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
		a.y < b ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
	};
}

template<typename T> constexpr vec2<T> operator>=( const T a, const vec2<T> b ) noexcept
{
	return {
		a >= b.x ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
		a >= b.y ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
	};
}

template<typename T> constexpr vec2<T> operator<=( const T a, const vec2<T> b ) noexcept
{
	return {
		a <= b.x ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
		a <= b.y ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
	};
}

template<typename T> constexpr vec2<T> operator>( const T a, const vec2<T> b ) noexcept
{
	return {
		a > b.x ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
		a > b.y ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
	};
}

template<typename T> constexpr vec2<T> operator<( const T a, const vec2<T> b ) noexcept
{
	return {
		a < b.x ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
		a < b.y ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T>,
	};
}

// Logical Operators
template<typename T> constexpr vec2<T> operator&( const vec2<T> a, const vec2<T> b ) noexcept
{
	return { vf::primitives::bitwise_and( a.x, b.x ), vf::primitives::bitwise_and( a.y, b.y ) };
}

template<typename T> constexpr vec2<T> operator|( const vec2<T> a, const vec2<T> b ) noexcept
{
	return { vf::primitives::bitwise_or( a.x, b.x ), vf::primitives::bitwise_or( a.y, b.y ) };
}

template<typename T> constexpr vec2<T> operator^( const vec2<T> a, const vec2<T> b ) noexcept
{
	return { vf::primitives::bitwise_xor( a.x, b.x ), vf::primitives::bitwise_xor( a.y, b.y ) };
}

template<typename T> constexpr vec2<T> operator&( const vec2<T> a, const T b ) noexcept
{
	return { vf::primitives::bitwise_and( a.x, b ), vf::primitives::bitwise_and( a.y, b ) };
}

template<typename T> constexpr vec2<T> operator|( const vec2<T> a, const T b ) noexcept
{
	return { vf::primitives::bitwise_or( a.x, b ), vf::primitives::bitwise_or( a.y, b ) };
}

template<typename T> constexpr vec2<T> operator^( const vec2<T> a, const T b ) noexcept
{
	return { vf::primitives::bitwise_xor( a.x, b ), vf::primitives::bitwise_xor( a.y, b ) };
}

template<typename T> constexpr vec2<T> operator&( const T a, const vec2<T> b ) noexcept
{
	return { vf::primitives::bitwise_and( a, b.x ), vf::primitives::bitwise_and( a, b.y ) };
}

template<typename T> constexpr vec2<T> operator|( const T a, const vec2<T> b ) noexcept
{
	return { vf::primitives::bitwise_or( a, b.x ), vf::primitives::bitwise_or( a, b.y ) };
}

template<typename T> constexpr vec2<T> operator^( const T a, const vec2<T> b ) noexcept
{
	return { vf::primitives::bitwise_xor( a, b.x ), vf::primitives::bitwise_xor( a, b.y ) };
}

#endif