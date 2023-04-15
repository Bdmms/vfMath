#pragma once
#ifndef VF_QUAT_MATH_HPP
#define VF_QUAT_MATH_HPP

#include "EulerMath.hpp"

#include "quaternion.hpp"

typedef quat (*QuatInterpolator)(const quat& a, const quat& b, const float weight);

/**
 * @brief Utilities for quaternions
*/
namespace Math
{
	template<> inline constexpr static quat ZERO<quat> = { 0.0f, 0.0f, 0.0f, 0.0f };
	template<> inline constexpr static quat ONES<quat> = { 1.0f, 1.0f, 1.0f, 1.0f };
	template<> inline constexpr static quat IDENTITY<quat> = { 0.0f, 0.0f, 0.0f, 1.0f };

	/**
	 * @brief Converts Euler angles vector to quaternion
	 * @param rotation - euler angles
	 * @return quaternion rotation
	*/
	[[nodiscard]] static quat toQuat(const euler& rotation)
	{
		vec3f s = sin(rotation * 0.5f);
		vec3f c = cos(rotation * 0.5f);

		return {
			s.x * c.y * c.z - c.x * s.y * s.z,
			c.x * s.y * c.z + s.x * c.y * s.z,
			c.x * c.y * s.z - s.x * s.y * c.z,
			c.x * c.y * c.z + s.x * s.y * s.z
		};
	}

	static quat toQuat(const AxisAngle& rotation)
	{
		float angle = Math::length(rotation);
		if (angle <= Math::EPSILON<float>) return Math::IDENTITY<quat>;

		vec3f axis = rotation / angle;
		quat q = { _mm_mul_ps( axis.simd, _mm_set1_ps( sinf( angle * 0.5f ) ) ) };
		q.w = cosf( angle * 0.5f );
		return q;
	}

	/**
	 * @brief Calculates the unit quaternion
	 * @param q - quaternion
	 * @return normalized quaternion
	*/
	[[nodiscard]] static quat normalize(const quat& q)
	{
		return { _mm_div_ps(q.simd, _mm_set1_ps(q.norm())) };
	}

	/**
	 * @brief Calculates the 4D dot product of this vector with another vector
	 * @param a - first vector
	 * @param b - second vector
	 * @return dot product result
	*/
	[[nodiscard]] constexpr float dot(const quat& a, const quat& b)
	{
		return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
	}

	/**
	 * @brief Converts quaternion to Euler angles
	 * @param q - quaternion
	 * @return Euler angles vector
	*/
	[[nodiscard]] static euler toEuler(const quat& q)
	{
		return {
			atan2f( 2.0f * (q.w * q.x + q.y * q.z), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z ),
			asinf( std::clamp(-2.0f * (q.x * q.z - q.w * q.y), -1.0f, 1.0f) ),
			atan2f( 2.0f * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z ),
			0.0f
		};
	}

	/**
	 * @brief Converts quaternion to an axis and angle
	 * @param q - quaternion
	 * @return Axis angle vector
	*/
	[[nodiscard]] static AxisAngle toAxisAngle(const quat& q)
	{
		AxisAngle axis = { q.x, q.y, q.z, 0.0f };
		float length = Math::length(axis);
		if (length <= Math::EPSILON<float>) return Math::ZERO<vec3f>;

		float angle = 2.0f * atan2f( length, q.w );
		return axis * (angle / length);
	}

	/**
	 * @brief Creates a quaternion from the provided axis and angle around axis
	 * @param axis - direction and scaling of quaternion (normalized)
	 * @param angle - angle of rotation around axis
	 * @return quaternion
	*/
	template<> [[nodiscard]] static quat rotationAround(const vec3f& axis, const float angle)
	{
		vec3f r = axis * sinf(angle * 0.5f);
		r.w = cosf(angle * 0.5f);
		return reinterpret_cast<quat&>(r);
	}

	/**
	 * @brief Generates the quaternion that rotates from one axis to another
	 * @param from - initial vector (normalized)
	 * @param to - target vector (normalized)
	 * @return resulting quaternion
	*/
	template<> [[nodiscard]] static quat rotationBetween( const vec3f& from, const vec3f& to )
	{
		float product = Math::dot_3D( from, to );
		if( product > 0.999999f ) return IDENTITY<quat>;
		if( product < -0.999999f ) return { Math::orthogonal( from ).simd };

		__m128 rotation = _mm_cross_ps( from.simd, to.simd );
		rotation.m128_f32[3] = Math::dot_3D( from, to ) + 1.0f;

		return normalize( reinterpret_cast<quat&>( rotation ) );
	}

	/**
	 * @brief Rotates a vector around the origin using a quaternion
	 * @param v - position vector
	 * @param q - quaternion
	 * @return rotated position vector
	*/
	[[nodiscard]] static vec4f rotate(const vec4f& v, const quat& q)
	{
		// v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
		return { _mm_add_ps(v.simd, _mm_mul_ps(_mm_set1_ps(2.0f), _mm_cross_ps(q.simd, _mm_add_ps(_mm_cross_ps(q.simd, v.simd), _mm_mul_ps(v.simd, _mm_set1_ps(q.w)))))) };
	}

	/**
	 * @brief Calculates the angle between two quaternions using the shortest rotation
	 * @param a - first quaternion
	 * @param b - second quaternion
	 * @return angle between the quaternions
	*/
	[[nodiscard]] static float angleBetween(const quat& a, const quat& b)
	{
		return acosf(Math::dot_3D(Math::rotate(Math::axis::X<vec3f>, a), Math::rotate(Math::axis::X<vec3f>, b)));
	}

	/**
	 * @brief Generates a random quaternion within the range
	 * @param min - minimum quat
	 * @param max - maximum quat
	 * @return random quaternion
	*/
	template<> [[nodiscard]] static quat random<quat>(const quat& min, const quat& max)
	{
		quat q = { ((float)rand() / RAND_MAX), ((float)rand() / RAND_MAX), ((float)rand() / RAND_MAX), ((float)rand() / RAND_MAX) };
		return min + q * (max - min);
	}

	/**
	 * @brief Generates a random normalized quaternion
	 * @return random quaternion
	*/
	template<> [[nodiscard]] static quat randomDirection<quat>()
	{
		quat q = { ((float)rand() / RAND_MAX), ((float)rand() / RAND_MAX), ((float)rand() / RAND_MAX), ((float)rand() / RAND_MAX) };
		float norm = q.norm();
		return norm > 0.0f ? q / norm : IDENTITY<quat>;
	}

	/**
	 * @brief Performs spherical interpolation between two quaternions
	 * @param q0 - first quaternion
	 * @param q1 - second quaternion
	 * @param t - weight between quaternions
	 * @return weighted quaternion
	*/
	static quat slerp(const quat& q0, const quat& q1, const float t)
	{
		float cost = dot(q0, q1);
		quat q2 = cost < 0.0f ? -q1 : q1;
		cost = fabsf(cost);

		if (cost > 1.0f - EPSILON<float>) return (q1 - q0) * t + q0;

		float angle = acosf(cost);
		vec3f sin = Math::sin(vec3f{ (1.0f - t) * angle, t * angle, angle });
		return (q0 * sin.x + q2 * sin.y) / sin.z;
	}
}

namespace Interpolate
{
	inline constexpr static QuatInterpolator QUAT_LINEAR = Math::slerp;
}

#endif