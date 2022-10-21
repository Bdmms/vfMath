#pragma once
#ifndef VF_ADVANCED_MATH_HPP
#define VF_ADVANCED_MATH_HPP

#include "MatrixMath.hpp"

namespace Math
{
	/**
	 * @brief Wrapper function used to handle boolean evaluations
	 * @param value - boolean value
	 * @return evaluated value
	*/
	[[nodiscard]] constexpr static bool evaluate(const bool value)
	{
		return value;
	}

	/**
	 * @brief Checks if two overlapping boundaries overlap
	 * @param min0 - minimum of first bounds
	 * @param max0 - maximum of first bounds
	 * @param min1 - minimum of second bounds
	 * @param max1 - maximum of second bounds
	 * @return Whether the two boundaries overlap
	*/
	template<typename T> [[nodiscard]] constexpr bool overlaps(const T& min0, const T& max0, const T& min1, const T& max1)
	{
		return Math::evaluate((min0 >= min1 && min0 <= max1) || (min1 >= min0 && min1 <= max0));
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
	template<typename T> [[nodiscard]] constexpr T mapRange(const T& value, const T& fromMin, const T& fromMax, const T& toMin, const T& toMax)
	{
		return (value - fromMin) * (toMax - toMin) / (fromMax - fromMin) + toMin;
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
	template<typename T> [[nodiscard]] constexpr T clampRange(const T& value, const T& fromMin, const T& fromMax, const T& toMin, const T& toMax)
	{
		return Math::clamp((value - fromMin) / (fromMax - fromMin)) * (toMax - toMin) + toMin;
	}

	/**
	 * @brief Finds the shortest angle between two absolute angles
	 * @param value - angle
	 * @param relative - angle relative to
	 * @return shortest angle
	*/
	template<typename T> [[nodiscard]] static T shortestAngle(const T& value, const T& relative)
	{
		// 0 = (value - relative) + TWO_PI * i;
		return value + TWO_PI<T> * round((relative - value) / TWO_PI<T>);
	}
};

#endif
