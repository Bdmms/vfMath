#pragma once
#ifndef VF_MATH_H
#define VF_MATH_H

#include "primitives.hpp"

#include <algorithm>
#include <numbers>

/**
 * @brief Pair of values that form a range.
*/
template <typename T>
struct Bounds
{
	T min;
	T max;
};

/**
 * @brief Math utility
*/
namespace Math
{
	template<typename T> inline constexpr static T ZERO = 0;
	template<typename T> inline constexpr static T ONES;

	template<> inline constexpr static i8  ONES<i8>  = 1;
	template<> inline constexpr static i16 ONES<i16> = 1;
	template<> inline constexpr static i32 ONES<i32> = 1;
	template<> inline constexpr static i64 ONES<i64> = 1LL;
	template<> inline constexpr static u8  ONES<u8>  = 1U;
	template<> inline constexpr static u16 ONES<u16> = 1U;
	template<> inline constexpr static u32 ONES<u32> = 1U;
	template<> inline constexpr static u64 ONES<u64> = 1LLU;
	template<> inline constexpr static f8  ONES<f8>  = 1.0_f8;
	template<> inline constexpr static f16 ONES<f16> = 1.0_f16;
	template<> inline constexpr static f32 ONES<f32> = 1.0f;
	template<> inline constexpr static f64 ONES<f64> = 1.0;

	template<typename T> inline constexpr static T IDENTITY = ONES<T>;
	template<typename T> inline constexpr static T NEGATIVE = -ONES<T>;
	template<typename T> inline constexpr static T MAX = std::numeric_limits<T>::max();
	template<typename T> inline constexpr static T MIN;

	template<> inline constexpr static i8  MIN<i8>  = std::numeric_limits<i8>::min();
	template<> inline constexpr static i16 MIN<i16> = std::numeric_limits<i16>::min();
	template<> inline constexpr static i32 MIN<i32> = std::numeric_limits<i32>::min();
	template<> inline constexpr static i64 MIN<i64> = std::numeric_limits<i64>::min();
	template<> inline constexpr static u8  MIN<u8>  = Math::ZERO<u8>;
	template<> inline constexpr static u16 MIN<u16> = Math::ZERO<u16>;
	template<> inline constexpr static u32 MIN<u32> = Math::ZERO<u32>;
	template<> inline constexpr static u64 MIN<u64> = Math::ZERO<u64>;
	template<> inline constexpr static f32 MIN<f32> = -Math::MAX<f32>;
	template<> inline constexpr static f64 MIN<f64> = -Math::MAX<f64>;

	template<typename T> inline constexpr static T EPSILON = std::numeric_limits<T>::epsilon();
	template<typename T> inline constexpr static T PI;
	template<typename T> inline constexpr static T TWO_PI;
	template<typename T> inline constexpr static T HALF_PI;

	template<> inline constexpr static float PI<f32> = std::numbers::pi_v<f32>;
	template<> inline constexpr static float TWO_PI<f32> = 2.0f * PI<f32>;
	template<> inline constexpr static float HALF_PI<f32> = 0.5f * PI<f32>;

	template<> inline constexpr static float PI<f64> = std::numbers::pi_v<f64>;
	template<> inline constexpr static float TWO_PI<f64> = 2.0f * PI<f64>;
	template<> inline constexpr static float HALF_PI<f64> = 0.5f * PI<f64>;

	constexpr float RAND_MAX_FLOAT = 32767.0f;
	constexpr float RAND_CONVERT_HALFPI = HALF_PI<float> / 32767.0f;
	constexpr float RAND_CONVERT_PI = PI<float> / 32767.0f;
	constexpr float RAND_CONVERT_TAU = TWO_PI<float> / 32767.0f;

	/**
	 * @brief Generates a random value within the range
	 * @param min - minimum value
	 * @param max - maximum value
	 * @return random value
	*/
	template<typename T> [[nodiscard]] static T random( const T& min = ZERO<T>, const T& max = ONES<T> )
	{
		return min + ( (T)rand() / RAND_MAX ) * ( max - min );
	}

	/**
	 * @brief Converts from degrees to radians.
	 * @param degrees - angle in degrees
	 * @return angle in radians
	*/
	[[nodiscard]] constexpr float toRadians( const float degrees )
	{
		return degrees * Math::PI<float> / 180.0f;
	}

	/**
	 * @brief Converts from radians to degrees.
	 * @param radians - angle in radians
	 * @return angle in degrees
	*/
	[[nodiscard]] constexpr float toDegrees( const float radians )
	{
		return radians * 180.0f / Math::PI<float>;
	}

	/**
	 * @brief Equivalent to absolute value
	 * @param val - value
	 * @return absolute value
	*/
	[[nodiscard]] static float length( const float val )
	{
		return fabsf( val );
	}

	/**
	 * @brief Calculates the absolute difference between two values
	 * @param a - first value
	 * @param b - second value
	 * @return absolute difference
	*/
	[[nodiscard]] static float distance( const float a, const float b )
	{
		return fabsf( b - a );
	}

	/**
	 * @brief Returns two different values depending on the input.
	 * @param input - condition input
	 * @param trueVal - value returned on true input
	 * @param falseVal - value returned on false input
	 * @return output value
	*/
	[[nodiscard]] constexpr float condition( const bool input, const float trueVal, const float falseVal )
	{
		return input ? trueVal : falseVal;
	}

	/**
	 * @brief Checks if two overlapping boundaries overlap
	 * @param min0 - minimum of first bounds
	 * @param max0 - maximum of first bounds
	 * @param min1 - minimum of second bounds
	 * @param max1 - maximum of second bounds
	 * @return Whether the two boundaries overlap
	*/
	[[nodiscard]] constexpr bool overlaps( const float min0, const float max0, const float min1, const float max1 )
	{
		return ( min0 >= min1 && min0 <= max1 ) || ( min1 >= min0 && min1 <= max0 );
	}

	/**
	 * @brief Tests if two bounds overlap with each other
	 * @param a - first bound range
	 * @param b - second bound range
	 * @return Whether the boundaries overlap
	*/
	[[nodiscard]] constexpr bool overlaps( const Bounds<float>& a, const Bounds<float>& b )
	{
		return overlaps( a.min, a.max, b.min, b.max );
	}

	/**
	 * @brief Determines the sign of the value in the form of 1.0 or -1.0.
	 * @param value - scalar value
	 * @return sign of the value
	*/
	[[nodiscard]] constexpr float sign( const float value )
	{
		return value >= 0.0f ? 1.0f : -1.0f;
	}

	/**
	 * @brief Determines the sign of the value in the form of 1 or -1.
	 * @param value - integer value
	 * @return sign of the value
	*/
	[[nodiscard]] constexpr int sign( const int value )
	{
		return value >= 0 ? 1 : -1;
	}

	/**
	 * @brief Truncates the value between 0 and 1.
	 * @param value - initial value
	 * @param min - minimum value of clamp (optional)
	 * @param max - maximum value of clamp (optional)
	 * @return clamped value
	*/
	[[nodiscard]] constexpr float clamp( const float value, const float min = 0.0f, const float max = 1.0f )
	{
		return std::clamp( value, min, max );
	}

	/**
	 * @brief Performs a sigmoid-like interpolation of the value.
	 * @param value - scalar value
	 * @return interpolated value
	*/
	[[nodiscard]] constexpr float smoothstep( const float value )
	{
		return value * value * ( 3.0f - 2.0f * value );
	}

	/**
	 * @brief Performs a sigmoid-like interpolation of the value.
	 * @param value - scalar value
	 * @return interpolated value
	*/
	[[nodiscard]] constexpr float smootherstep( const float value )
	{
		return value * value * value * ( 3.0f * value * ( 2.0f * value - 5.0f ) + 10.0f );
	}

	/**
	 * @brief Linear interpolation between two values.
	 * @param a - first value
	 * @param b - second value
	 * @param t - weight
	 * @return weighted value
	*/
	template<typename T> [[nodiscard]] static T lerp( const T& a, const T& b, const float t )
	{
		return ( b - a ) * t + a;
	}

	/**
	 * @brief Returns the positive remainder of the modulo operator.
	 * For example: 
	 *     3 % 4 == 3, 
	 *     6 % 4 == 2, 
	 *    -1 % 4 == 3, 
	 *    -7 % 4 == 1
	 * @param a - first value
	 * @param b - second value
	 * @param t - weight
	 * @return weighted value
	*/
	template<typename T> [[nodiscard]] constexpr T pmod( T a, T b )
	{
		return a >= 0 ? a % b : b + ( a % b );
	}

	/**
	 * @brief Maps a value from one range to another
	 * @param value - initial value
	 * @param fromMin - min initial value
	 * @param fromMax - max initial value
	 * @param toMin - min resulting value
	 * @param toMax - max resulting value
	 * @return resulting value
	*/
	template<typename T> [[nodiscard]] constexpr T mapRange( const T& value, const T& fromMin, const T& fromMax, const T& toMin, const T& toMax )
	{
		return ( value - fromMin ) * ( toMax - toMin ) / ( fromMax - fromMin ) + toMin;
	}

	/**
	 * @brief Maps a value from one range to another while truncating the data
	 * @param value - initial value
	 * @param fromMin - min initial value
	 * @param fromMax - max initial value
	 * @param toMin - min resulting value
	 * @param toMax - max resulting value
	 * @return resulting value
	*/
	template<typename T> [[nodiscard]] constexpr T clampRange( const T& value, const T& fromMin, const T& fromMax, const T& toMin, const T& toMax )
	{
		return Math::clamp( ( value - fromMin ) / ( fromMax - fromMin ) ) * ( toMax - toMin ) + toMin;
	}

	/**
	 * @brief Finds the shortest angle between two absolute angles
	 * @param value - angle
	 * @param relative - angle relative to
	 * @return shortest angle
	*/
	template<typename T> [[nodiscard]] static T shortestAngle( const T& value, const T& relative )
	{
		// 0 = (value - relative) + TWO_PI * i;
		return value + TWO_PI<T> *round( ( relative - value ) / TWO_PI<T> );
	}

	/**
	 * @brief Extends the bounds to include the value.
	 * @param bounds - scalar bounds
	 * @param point - scalar point
	*/
	static void extend( Bounds<float>& bounds, const float value )
	{
		bounds.min = std::min( bounds.min, value );
		bounds.max = std::max( bounds.max, value );
	}
}

#endif