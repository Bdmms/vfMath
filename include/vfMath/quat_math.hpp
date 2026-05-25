#pragma once
#ifndef VF_QUAT_MATH_HPP
#define VF_QUAT_MATH_HPP

#include "euler_math.hpp"
#include "quat.hpp"

/**
 * @brief Utilities for quaternions
*/
namespace Math
{
	template<typename T> inline constexpr static quat<T> ZERO<quat<T>> = quat<T>::set1( Math::ZERO<T> );
	template<typename T> inline constexpr static quat<T> ONES<quat<T>> = quat<T>::set1( Math::ONES<T> );
	template<typename T> inline constexpr static quat<T> IDENTITY<quat<T>> = { Math::ZERO<T>, Math::ZERO<T>, Math::ZERO<T>, Math::ONES<T> };

	/**
	 * @brief Calculates the squared norm of the quaternion
	 * @return squared norm of the quaternion
	*/
	template<typename T>
	[[nodiscard]] constexpr T norm2( const quat<T>& q )
	{
		return vf::simd::dot( q, q );
	}

	/**
	 * @brief Calculates the norm of the quaternion
	 * @return norm of the quaternion
	*/
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] constexpr T norm( const quat<T>& q )
	{
		return sqrt( norm2( q ) );
	}

	/**
	 * @brief Calculates the inverse of the quaternion
	 * @return inverse of the quaternion
	*/
	template<typename T>
	[[nodiscard]] constexpr quat<T> invert( const quat<T>& q )
	{
		quat r{ -1.0f, -1.0f, -1.0f, 1.0f };
		vf::simd::mul( r, r, q );
		return r;
	}

	/**
	 * @brief Converts euler angles vector to quaternion
	 * @param rotation - euler angles
	 * @return quaternion rotation
	*/
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] static quat<T> toQuat( const euler<T>& rotation )
	{
		euler<T> s = sin( rotation * 0.5f );
		euler<T> c = cos( rotation * 0.5f );

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
	template<typename T> requires std::floating_point<T>
	static quat<T> toQuat( const AxisAngle<T>& rotation )
	{
		T angle = Math::length( rotation );
		if( angle <= Math::EPSILON<T> ) return Math::IDENTITY<quat<T>>;

		quat<T> q;
		vf::simd::mul( q, rotation / angle, quat<T>::set1( std::sin( angle * 0.5f ) ) );
		q.w = std::cos( angle * HALF<T> );
		return q;
	}

	/**
	 * @brief Calculates the unit quaternion
	 * @param q - quaternion
	 * @return normalized quaternion
	*/
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] static quat<T> normalize( const quat<T>& q )
	{
		return q / norm( q );
	}
	
	/**
	 * @brief Calculates the 4D dot product of this vector with another vector
	 * @param a - first vector
	 * @param b - second vector
	 * @return dot product result
	*/
	template<typename T>
	[[nodiscard]] constexpr T dot( const quat<T>& a, const quat<T>& b )
	{
		return vf::simd::dot( a, b );
	}

	/**
	 * @brief Rotates a vector around the origin using a quaternion
	 * @param v - position vector
	 * @param q - quaternion
	 * @return rotated position vector
	*/
	template<typename T>
	[[nodiscard]] static vec4<T> rotate( const vec4<T>& v, const quat<T>& q )
	{
		// TODO: Remove reinterpret_cast and make constexpr
		const vec4<T>& qxyz = reinterpret_cast<const vec4<T>&>( q );
		return v + TWO<T> * cross( qxyz, cross( qxyz, v ) + q.w * v );
	}

	/**
	 * @brief Converts quaternion to Euler angles
	 * @param q - quaternion
	 * @return Euler angles vector
	*/
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] static euler<T> toEuler( const quat<T>& q )
	{
		return {
			std::atan2( TWO<T> * ( q.w * q.x + q.y * q.z ), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z ),
			std::asin( clamp( TWO<T> * ( q.w * q.y - q.x * q.z ), NEGATIVE<T>, ONES<T> ) ),
			std::atan2( TWO<T> * ( q.x * q.y + q.w * q.z ), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z )
		};
	}

	/**
	 * @brief Converts quaternion to an axis and angle
	 * @param q - quaternion
	 * @return Axis angle vector
	*/
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] static AxisAngle<T> toAxisAngle( const quat<T>& q )
	{
		AxisAngle<T> axis = { q.x, q.y, q.z, ZERO<T> };
		T length = Math::length( axis );
		if( length <= Math::EPSILON<T> ) return Math::ZERO<AxisAngle<T>>;

		T angle = TWO<T> * std::atan2( length, q.w );
		return axis * ( angle / length );
	}

	/**
	 * @brief Creates a quaternion from the provided axis and angle around axis
	 * @param axis - direction and scaling of quaternion (normalized)
	 * @param angle - angle of rotation around axis
	 * @return quaternion
	*/
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] static quat<T> quatRotationAround( const vec4<T>& axis, const T angle )
	{
		quat<T> r = quat<T>::set1( std::sin( angle * HALF<T> ) );
		vf::simd::mul( r, r, axis );
		r.w = std::cos( angle * HALF<T> );
		return r;
	}

	/**
	 * @brief Generates the quaternion that rotates from one axis to another.
	 * @param from - initial vector (normalized)
	 * @param to - target vector (normalized)
	 * @return resulting quaternion
	*/
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] constexpr quat<T> quatRotationBetween( const vec4<T>& from, const vec4<T>& to )
	{
		T product = dot_3D( from, to );
		if( product > ONES<T> - EPSILON<T> ) return IDENTITY<quat<T>>;
		if( product < NEGATIVE<T> +EPSILON<T> ) return orthogonal( from );

		vec4<T> rotation = cross( from, to );
		rotation.w = dot_3D( from, to ) + ONES<T>;
		return rotation / length( rotation );
	}

	/**
	 * @brief Generates the quaternion that rotates from one space to another.
	 * @param fromPrimary - initial primary vector
	 * @param fromSecondary - initial secondary vector
	 * @param toPrimary - target primary vector
	 * @param toSecondary - target secondary vector
	 * @return resulting quaternion
	*/
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] static quat<T> quatRotationBetween( const vec4<T>& fromPrimary, const vec4<T>& fromSecondary, const vec4<T>& toPrimary, const vec4<T>& toSecondary )
	{
		// If cannot calculate primary direction, return identity
		if( length2( toPrimary ) < EPSILON<T> ) return IDENTITY<quat<T>>;

		quat<T> r1 = quatRotationBetween( fromPrimary, toPrimary );

		vec4<T> current = rotate( fromSecondary, r1 );
		vec4<T> target = cross( toPrimary, cross( toSecondary, toPrimary ) );

		// If cannot calculate roll around axis, return arbitrary rotation
		if( length2( target ) < EPSILON<T> ) return r1;

		target = normalize( target );
		T angle = std::acos( clamp( dot_3D( current, target ), NEGATIVE<T>, ONES<T> ) );
		T angleSign = sign( dot_3D( cross( current, target ), toPrimary ) );
		return r1 * quatRotationAround( fromPrimary, angle * angleSign );
	}

	/**
	 * @brief Calculates the angle between two quaternions using the shortest rotation
	 * @param a - first quaternion
	 * @param b - second quaternion
	 * @return angle between the quaternions
	*/
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] static T angleBetween( const quat<T>& a, const quat<T>& b )
	{
		quat<T> q = b * invert( a );
		return TWO<T> * std::atan2( std::sqrt( q.x * q.x + q.y * q.y + q.z * q.z ), q.w );
	}

	/**
	 * @brief Generates a random quaternion within the range
	 * @param min - minimum quat
	 * @param max - maximum quat
	 * @return random quaternion
	*/
	template<typename T>
	[[nodiscard]] static quat<T> random( const quat<T>& min, const quat<T>& max )
	{
		return { random<T>( min.x, max.x ), random<T>( min.y, max.y ), random<T>( min.z, max.z ), random<T>( min.w, max.w ) };
	}

	/**
	 * @brief Generates a random quaternion rotation
	 * @return random quaternion rotation
	*/
	template<> [[nodiscard]] static quatf32 randomRotation<quatf32>()
	{
		return quatRotationAround( randomDirection<vec4f>(), rand() * RAND_CONVERT_TAU );
	}

	/**
	 * @brief Calculates the quaternion rotation needed to rotate the identity matrix to the specified orientation.
	 * @param initial - current direction of axis (normalized)
	 * @param target - expected direction of axis (normalized)
	 * @param trackAxis - direction of tracked axis (normalized)
	 * @return quaternion rotation
	*/
	template<typename T> requires std::floating_point<T>
	static quat<T> lookTo( const vec4<T>& initial, const vec4<T>& target, const vec4<T>& trackAxis )
	{
		vec4<T> projectedInitial = initial - dot_3D( initial, trackAxis ) * trackAxis;
		vec4<T> projectedTarget = target - dot_3D( target, trackAxis ) * trackAxis;

		T magnitudeInitial = sqrt( dot_3D( projectedInitial, projectedInitial ) );
		T magnitudeTarget = sqrt( dot_3D( projectedTarget, projectedTarget ) );

		// Normalize projected vectors
		if( magnitudeInitial <= EPSILON<T> || magnitudeTarget <= EPSILON<T> ) return IDENTITY<quat<T>>;
		
		projectedTarget /= magnitudeTarget;
		projectedInitial /= magnitudeInitial;

		vec4<T> axis = cross( projectedInitial, projectedTarget );
		T magnitude = sqrt( dot_3D( axis, axis ) );

		if( magnitude <= EPSILON<T> ) return IDENTITY<quat<T>>;

		T angle = std::acos( clamp( dot_3D( projectedInitial, projectedTarget ), NEGATIVE<T>, ONES<T> ) );
		return quatRotationAround( axis / magnitude, angle );
	}

	/**
	 * @brief Performs spherical interpolation between two quaternions
	 * @param q0 - first quaternion
	 * @param q1 - second quaternion
	 * @param t - weight between quaternions
	 * @return weighted quaternion
	*/
	template<typename T> requires std::floating_point<T>
	static quat<T> slerp( const quat<T>& q0, const quat<T>& q1, const T t )
	{
		T cost = dot( q0, q1 );
		quat<T> q2 = cost < ZERO<T> ? -q1 : q1;
		cost = std::fabs( cost );

		if( cost > ONES<T> - EPSILON<T> ) return normalize( ( q1 - q0 ) * t + q0 );

		float angle = std::acos( cost );
		vec4<T> s = sin( vec4<T>{ ( ONES<T> - t ) * angle, t * angle, angle } );
		return ( q0 * s.x + q2 * s.y ) / s.z;
	}
}

#endif