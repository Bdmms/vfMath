#pragma once
#ifndef VF_EULER_MATH_HPP
#define VF_EULER_MATH_HPP

#include "VectorMath.hpp"

typedef vec3f euler;

/**
 * @brief Utilities for Euler Angles
*/
namespace Math
{
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
	 * @brief Rotates a vector around the origin using euler angles
	 * @param position - position vector
	 * @param rotation - euler angles
	 * @return rotated position
	*/
	[[nodiscard]] static vec3f rotate(const vec3f& position, const euler& rotation)
	{
		vec3f s = Math::sin(rotation);
		vec3f c = Math::cos(rotation);

		return {
			_mm_dot_ps(
				position.simd, _mm_xyzw_ps(c.z * c.y, c.z * s.y * s.x - s.z * c.x, c.z * s.y * c.x + s.z * s.x, 0.0f),
				position.simd, _mm_xyzw_ps(s.z * c.y, s.z * s.y * s.x + c.z * c.x, s.z * s.y * c.x - c.z * s.x, 0.0f),
				position.simd, _mm_xyzw_ps(-s.y, c.y * s.x, c.y * c.x, 0.0f),
				position.simd, SIMD_4f_W
			)
		};
	}
}

#endif