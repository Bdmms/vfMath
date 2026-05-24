#pragma once
#ifndef VF_MATH_CONSTANTS_H
#define VF_MATH_CONSTANTS_H

#include "primitives.hpp"

#include <numbers>

namespace Math
{
	template<typename T> inline constexpr static T ZERO = 0;
	template<typename T> inline constexpr static T ONES;
	template<typename T> inline constexpr static T HALF;

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

	template<> inline constexpr static f8  HALF<f8>  = 0.5_f8;
	template<> inline constexpr static f16 HALF<f16> = 0.5_f16;
	template<> inline constexpr static f32 HALF<f32> = 0.5f;
	template<> inline constexpr static f64 HALF<f64> = 0.5;

	template<typename T> inline constexpr static T IDENTITY = ONES<T>;
	template<typename T> inline constexpr static T NEGATIVE = -ONES<T>;
	template<typename T> inline constexpr static T MAX = std::numeric_limits<T>::max();
	template<typename T> inline constexpr static T MIN;

	template<> inline constexpr static i8  MIN<i8> = std::numeric_limits<i8>::min();
	template<> inline constexpr static i16 MIN<i16> = std::numeric_limits<i16>::min();
	template<> inline constexpr static i32 MIN<i32> = std::numeric_limits<i32>::min();
	template<> inline constexpr static i64 MIN<i64> = std::numeric_limits<i64>::min();
	template<> inline constexpr static u8  MIN<u8> = Math::ZERO<u8>;
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
}

#endif