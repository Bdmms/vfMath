#pragma once
#ifndef VF_MATH_H
#define VF_MATH_H

#include <algorithm>
#include <limits>
#include <numbers>

/**
 * @brief Math utility
*/
namespace Math
{
	template<typename T> inline constexpr static T ZERO;
	template<typename T> inline constexpr static T ONES;
	template<typename T> inline constexpr static T IDENTITY;
	template<typename T> inline constexpr static T NEGATIVE;
	template<typename T> inline constexpr static T MIN;
	template<typename T> inline constexpr static T MAX;
	template<typename T> inline constexpr static T EPSILON;
	template<typename T> inline constexpr static T PI;
	template<typename T> inline constexpr static T TWO_PI;
	template<typename T> inline constexpr static T HALF_PI;

	template<> inline constexpr static float ZERO<float> = 0.0f;
	template<> inline constexpr static int ZERO<int> = 0;

	template<> inline constexpr static float ONES<float> = 1.0f;
	template<> inline constexpr static int ONES<int> = 1;

	template<> inline constexpr static float IDENTITY<float> = 1.0f;
	template<> inline constexpr static int IDENTITY<int> = 1;

	template<> inline constexpr static float NEGATIVE<float> = -1.0f;
	template<> inline constexpr static int NEGATIVE<int> = -1;

	template<> inline constexpr static float MIN<float> = -std::numeric_limits<float>::max();
	template<> inline constexpr static int MIN<int> = std::numeric_limits<int>::min();

	template<> inline constexpr static float MAX<float> = std::numeric_limits<float>::max();
	template<> inline constexpr static int MAX<int> = std::numeric_limits<int>::max();

	template<> inline constexpr static float EPSILON<float> = std::numeric_limits<float>::epsilon();
	template<> inline constexpr static int EPSILON<int> = std::numeric_limits<int>::epsilon();

	template<> inline constexpr static float PI<float> = std::numbers::pi_v<float>;
	template<> inline constexpr static float TWO_PI<float> = 2.0f * PI<float>;
	template<> inline constexpr static float HALF_PI<float> = 0.5f * PI<float>;

	/**
	 * @brief Generates a random value within the range
	 * @param min - minimum value
	 * @param max - maximum value
	 * @return random value
	*/
	template<typename T> [[nodiscard]] static T random(const T& min = ZERO<T>, const T& max = ONES<T>)
	{
		return min + ((T)rand() / RAND_MAX) * (max - min);
	}

	/**
	 * @brief Equivalent to absolute value
	 * @param val - value
	 * @return absolute value
	*/
	[[nodiscard]] static float length(const float val)
	{
		return fabsf(val);
	}

	/**
	 * @brief Calculates the absolute difference between two values
	 * @param a - first value
	 * @param b - second value
	 * @return absolute difference
	*/
	[[nodiscard]] static float distance(const float a, const float b)
	{
		return fabsf(b - a);
	}

	/**
	 * @brief Returns two different values depending on the input
	 * @param input - condition input
	 * @param trueVal - value returned on true input
	 * @param falseVal - value returned on false input
	 * @return output value
	*/
	[[nodiscard]] constexpr float condition(const bool input, const float trueVal, const float falseVal)
	{
		return input ? trueVal : falseVal;
	}

	/**
	 * @brief Linear interpolation between two values
	 * @param a - first value
	 * @param b - second value
	 * @param t - weight
	 * @return weighted value
	*/
	template<typename T> [[nodiscard]] static T lerp(const T& a, const T& b, const float t)
	{
		return (b - a) * t + a;
	}

	/**
	 * @brief Determines the sign of the value in the form of 1.0 or -1.0
	 * @param value - scalar value
	 * @return sign of the value
	*/
	[[nodiscard]] constexpr float sign(const float value)
	{
		return value >= 0.0f ? 1.0f : -1.0f;
	}

	/**
	 * @brief Truncates the value between 0 and 1
	 * @param value - initial value
	 * @param min - minimum value of clamp (optional)
	 * @param max - maximum value of clamp (optional)
	 * @return clamped value
	*/
	[[nodiscard]] constexpr float clamp(const float value, const float min = 0.0f, const float max = 1.0f)
	{
		return std::clamp(value, min, max);
	}
}

typedef float (*InterpolationMode)(const float t);

namespace Interpolate
{
	[[nodiscard]] constexpr float linear(const float t)
	{
		return t;
	}

	[[nodiscard]] constexpr float bezier(const float t)
	{
		return 3.0f * (1.0f - t) * t * t + t * t * t;
	}

	[[nodiscard]] constexpr float quadratic(const float t)
	{
		return t * t;
	}
}

#endif