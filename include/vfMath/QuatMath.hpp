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
	 * @brief Converts euler angles vector to quaternion
	 * @param rotation - euler angles
	 * @return quaternion rotation
	*/
	[[nodiscard]] static quat toQuat( const euler& rotation )
	{
		vec3f s = sin( rotation * 0.5f );
		vec3f c = cos( rotation * 0.5f );

		return {
			s.x * c.y * c.z - c.x * s.y * s.z,
			c.x * s.y * c.z + s.x * c.y * s.z,
			c.x * c.y * s.z - s.x * s.y * c.z,
			c.x * c.y * c.z + s.x * s.y * s.z
		};
	}

	/**
	 * @brief Converts axis angle rotation vector to quaternion
	 * @param rotation - axis angle rotation
	 * @return quaternion rotation
	*/
	static quat toQuat( const AxisAngle& rotation )
	{
		float angle = Math::length( rotation );
		if( angle <= Math::EPSILON<float> ) return Math::IDENTITY<quat>;

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
	[[nodiscard]] static quat normalize( const quat& q )
	{
		return { _mm_div_ps( q.simd, _mm_set1_ps( q.norm() ) ) };
	}
	
	/**
	 * @brief Calculates the 4D dot product of this vector with another vector
	 * @param a - first vector
	 * @param b - second vector
	 * @return dot product result
	*/
	[[nodiscard]] constexpr float dot( const quat& a, const quat& b )
	{
		return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
	}

	/**
	 * @brief Rotates a vector around the origin using a quaternion
	 * @param v - position vector
	 * @param q - quaternion
	 * @return rotated position vector
	*/
	[[nodiscard]] static vec4f rotate( const vec4f& v, const quat& q )
	{
		// v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
		return { _mm_add_ps( v.simd, _mm_mul_ps( _mm_set1_ps( 2.0f ), _mm_cross_ps( q.simd, _mm_add_ps( _mm_cross_ps( q.simd, v.simd ), _mm_mul_ps( v.simd, _mm_set1_ps( q.w ) ) ) ) ) ) };
	}

	/**
	 * @brief Converts quaternion to Euler angles
	 * @param q - quaternion
	 * @return Euler angles vector
	*/
	[[nodiscard]] static euler toEuler( const quat& q )
	{
		return {
			atan2f( 2.0f * ( q.w * q.x + q.y * q.z ), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z ),
			asinf( std::clamp( -2.0f * ( q.x * q.z - q.w * q.y ), -1.0f, 1.0f ) ),
			atan2f( 2.0f * ( q.x * q.y + q.w * q.z ), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z ),
			0.0f
		};
	}

	/**
	 * @brief Converts quaternion to an axis and angle
	 * @param q - quaternion
	 * @return Axis angle vector
	*/
	[[nodiscard]] static AxisAngle toAxisAngle( const quat& q )
	{
		AxisAngle axis = { q.x, q.y, q.z, 0.0f };
		float length = Math::length( axis );
		if( length <= Math::EPSILON<float> ) return Math::ZERO<vec3f>;

		float angle = 2.0f * atan2f( length, q.w );
		return axis * ( angle / length );
	}

	/**
	 * @brief Creates a quaternion from the provided axis and angle around axis
	 * @param axis - direction and scaling of quaternion (normalized)
	 * @param angle - angle of rotation around axis
	 * @return quaternion
	*/
	template<> [[nodiscard]] static quat rotationAround( const vec3f& axis, const float angle )
	{
		vec3f r = axis * sinf( angle * 0.5f );
		r.w = cosf( angle * 0.5f );
		return reinterpret_cast<quat&>( r );
	}

	/**
	 * @brief Generates the quaternion that rotates from one axis to another.
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
	 * @brief Generates the quaternion that rotates from one space to another.
	 * @param fromPrimary - initial primary vector
	 * @param fromSecondary - initial secondary vector
	 * @param toPrimary - target primary vector
	 * @param toSecondary - target secondary vector
	 * @return resulting quaternion
	*/
	[[nodiscard]] static quat rotationBetween( const vec3f& fromPrimary, const vec3f& fromSecondary, const vec3f& toPrimary, const vec3f& toSecondary )
	{
		// If cannot calculate primary direction, return identity
		if( Math::length2( toPrimary ) < Math::EPSILON<float> ) return Math::IDENTITY<quat>;

		quat r1 = Math::rotationBetween<quat>( fromPrimary, toPrimary );

		vec3f current = Math::rotate( fromSecondary, r1 );
		vec3f target = Math::cross( toPrimary, Math::cross( toSecondary, toPrimary ) );

		// If cannot calculate roll around axis, return arbitrary rotation
		if( Math::length2( target ) < Math::EPSILON<float> ) return r1;

		target = Math::normalize( target );
		float angle = acosf( std::clamp( Math::dot_3D( current, target ), -1.0f, 1.0f ) );
		float sign = Math::sign( Math::dot_3D( Math::cross( current, target ), toPrimary ) );
		return r1 * Math::rotationAround<quat>( fromPrimary, angle * sign );
	}

	/**
	 * @brief Calculates the angle between two quaternions using the shortest rotation
	 * @param a - first quaternion
	 * @param b - second quaternion
	 * @return angle between the quaternions
	*/
	[[nodiscard]] static float angleBetween( const quat& a, const quat& b )
	{
		quat q = b * a.inverse();
		return 2.0f * atan2f( sqrtf( q.x * q.x + q.y * q.y + q.z * q.z ), q.w );
	}

	/**
	 * @brief Generates a random quaternion within the range
	 * @param min - minimum quat
	 * @param max - maximum quat
	 * @return random quaternion
	*/
	template<> [[nodiscard]] static quat random<quat>( const quat& min, const quat& max )
	{
		quat q = { ( rand() / RAND_MAX_FLOAT ), ( rand() / RAND_MAX_FLOAT ), ( rand() / RAND_MAX_FLOAT ), ( rand() / RAND_MAX_FLOAT ) };
		return min + q * ( max - min );
	}

	/**
	 * @brief Generates a random quaternion rotation
	 * @return random quaternion rotation
	*/
	template<> [[nodiscard]] static quat randomRotation<quat>()
	{
		return rotationAround<quat>( randomDirection<vec3f>(), rand() * RAND_CONVERT_TAU );
	}

	/**
	 * @brief Calculates the quaternion rotation needed to rotate the identity matrix to the specified orientation.
	 * @param initial - current direction of axis (normalized)
	 * @param target - expected direction of axis (normalized)
	 * @param trackAxis - direction of tracked axis (normalized)
	 * @return quaternion rotation
	*/
	static quat lookTo( const vec3f& initial, const vec3f& target, const vec3f& trackAxis )
	{
		vec3f projectedInitial = initial - Math::dot_3D( initial, trackAxis ) * trackAxis;
		vec3f projectedTarget = target - Math::dot_3D( target, trackAxis ) * trackAxis;

		float magnitudeInitial = sqrtf( Math::dot_3D( projectedInitial, projectedInitial ) );
		float magnitudeTarget = sqrtf( Math::dot_3D( projectedTarget, projectedTarget ) );

		// Normalize projected vectors
		if( magnitudeInitial <= EPSILON<float> || magnitudeTarget <= EPSILON<float> ) return Math::IDENTITY<quat>;
		
		projectedTarget /= magnitudeTarget;
		projectedInitial /= magnitudeInitial;

		vec3f axis = Math::cross( projectedInitial, projectedTarget );
		float magnitude = sqrtf( Math::dot_3D( axis, axis ) );

		if( magnitude <= EPSILON<float> ) return Math::IDENTITY<quat>;

		float angle = acosf( std::clamp( dot_3D( projectedInitial, projectedTarget ), -1.0f, 1.0f ) );
		return rotationAround<quat>( axis / magnitude, angle );
	}

	/**
	 * @brief Performs spherical interpolation between two quaternions
	 * @param q0 - first quaternion
	 * @param q1 - second quaternion
	 * @param t - weight between quaternions
	 * @return weighted quaternion
	*/
	static quat slerp( const quat& q0, const quat& q1, const float t )
	{
		float cost = dot( q0, q1 );
		quat q2 = cost < 0.0f ? -q1 : q1;
		cost = fabsf( cost );

		if( cost > 1.0f - EPSILON<float> ) return Math::normalize( ( q1 - q0 ) * t + q0 );

		float angle = acosf( cost );
		vec3f sin = Math::sin( vec3f{ ( 1.0f - t ) * angle, t * angle, angle } );
		return ( q0 * sin.x + q2 * sin.y ) / sin.z;
	}
}

#endif