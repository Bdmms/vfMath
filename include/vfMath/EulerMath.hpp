#pragma once
#ifndef VF_EULER_MATH_HPP
#define VF_EULER_MATH_HPP

#include "VectorMath.hpp"
#include "Euler.hpp"

/**
 * @brief Utilities for Euler Angles
*/
namespace Math
{
	template<> inline constexpr static euler ZERO<euler> = { 0.0f, 0.0f, 0.0f, 0.0f };
	template<> inline constexpr static euler ONES<euler> = { 1.0f, 1.0f, 1.0f, 1.0f };
	template<> inline constexpr static euler IDENTITY<euler> = { 0.0f, 0.0f, 0.0f, 0.0f };
	template<> inline constexpr static euler NEGATIVE<euler> = { -1.0f, -1.0f, -1.0f, -1.0f };
	template<> inline constexpr static euler MIN<euler> = { MIN<float>, MIN<float>, MIN<float>, MIN<float> };
	template<> inline constexpr static euler MAX<euler> = { MAX<float>, MAX<float>, MAX<float>, MAX<float> };
	template<> inline constexpr static euler PI<euler> = { PI<float>, PI<float>, PI<float>, PI<float> };
	template<> inline constexpr static euler TWO_PI<euler> = { TWO_PI<float>, TWO_PI<float>, TWO_PI<float>, TWO_PI<float> };
	template<> inline constexpr static euler HALF_PI<euler> = { HALF_PI<float>, HALF_PI<float>, HALF_PI<float>, HALF_PI<float> };

	/**
	 * @brief Generates rotation between two axes
	 * @tparam T - rotation type
	 * @param from - initial axis (normalized)
	 * @param to - final axis (normalized)
	 * @return rotation between axes
	*/
	template<typename T> [[nodiscard]] static T rotationBetween(const vec3f& from, const vec3f& to)
	{
		return to - from;
	}

	/**
	 * @brief Generates rotation between two axes
	 * @tparam T - rotation type
	 * @param from - initial axis (normalized)
	 * @param to - final axis (normalized)
	 * @param axis - axis to rotate around (normalized)
	 * @return rotation between axes
	*/
	template<typename T> [[nodiscard]] static T rotationBetween(const vec3f& from, const vec3f& to, const vec3f& axis)
	{
		return to - from;
	}

	/**
	 * @brief Generates rotation around an axis
	 * @tparam T - rotation type
	 * @param axis - axis (normalized)
	 * @param angle - radian angle around axis
	 * @return rotation around axis
	*/
	template<typename T> [[nodiscard]] static T rotationAround(const vec3f& axis, const float angle)
	{
		return axis;
	}

	/**
	 * @brief Generates euler rotation around an axis
	 * @param axis - axis (normalized)
	 * @param angle - radian angle around axis
	 * @return euelr angles
	*/
	template<> [[nodiscard]] static euler rotationAround(const vec3f& axis, const float angle)
	{
		vec3f q = axis * sinf(angle * 0.5f);
		q.w = cosf(angle * 0.5f);

		return {
			atan2f(2.0f * (q.w * q.x + q.y * q.z), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z),
			asinf( std::clamp(-2.0f * (q.x * q.z - q.w * q.y), -1.0f, 1.0f) ),
			atan2f(2.0f * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z),
			0.0f
		};
	}

	/**
	 * @brief Calculates the sine of the euler angles
	 * @param a - euler angles
	 * @return sin vector
	*/
	[[nodiscard]] static vec4f sin(const euler& rotation)
	{
		return { _mm_sin_ps(rotation.simd) };
	}

	/**
	 * @brief Calculates the cosine of the euler angles
	 * @param a - euler angles
	 * @return cos vector
	*/
	[[nodiscard]] static vec4f cos(const euler& rotation)
	{
		return { _mm_cos_ps(rotation.simd) };
	}

	/**
	 * @brief Calculates the tan of the euler angles
	 * @param a - euler angles
	 * @return tan vector
	*/
	[[nodiscard]] static vec4f tan(const euler& rotation)
	{
		return { _mm_tan_ps(rotation.simd) };
	}

	/**
	 * @brief Rotates a vector around the origin using euler angles
	 * @param position - position vector
	 * @param rotation - euler angles
	 * @return rotated position
	*/
	[[nodiscard]] static vec3f rotate(const vec3f& position, const euler& rotation)
	{
		vec3f s = sin(rotation);
		vec3f c = cos(rotation);

		return {
			_mm_dot_ps(
				position.simd, _mm_xyzw_ps(c.z * c.y, c.z * s.y * s.x - s.z * c.x, c.z * s.y * c.x + s.z * s.x, 0.0f),
				position.simd, _mm_xyzw_ps(s.z * c.y, s.z * s.y * s.x + c.z * c.x, s.z * s.y * c.x - c.z * s.x, 0.0f),
				position.simd, _mm_xyzw_ps(-s.y, c.y * s.x, c.y * c.x, 0.0f),
				position.simd, SIMD_4f_W
			)
		};
	}

	/**
	 * @brief Generates random euler angles within the range
	 * @param min - minimum angles
	 * @param max - maximum angles
	 * @return random euler angles
	*/
	template<> [[nodiscard]] static euler random<euler>(const euler& min, const euler& max)
	{
		return { _mm_fmadd_ps(_mm_div_ps(_mm_cvtepi32_ps(_mm_set_epi32(rand(), rand(), rand(), rand())), _mm_set1_ps((float)RAND_MAX)), _mm_sub_ps(max.simd, min.simd), min.simd) };
	}
}

#endif