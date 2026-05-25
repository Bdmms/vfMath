#pragma once
#ifndef VF_EULER_MATH_HPP
#define VF_EULER_MATH_HPP

#include "vec_math.hpp"
#include "euler.hpp"

/**
 * @brief Utilities for Euler Angles
*/
namespace Math
{
	template<typename T> 
	inline constexpr static euler<T> ZERO<euler<T>> = { ZERO<T>, ZERO<T>, ZERO<T> };
	template<typename T> 
	inline constexpr static euler<T> ONES<euler<T>> = { ONES<T>, ONES<T>, ONES<T> };
	template<typename T> 
	inline constexpr static euler<T> IDENTITY<euler<T>> = ZERO<euler<T>>;
	template<typename T> 
	inline constexpr static euler<T> NEGATIVE<euler<T>> = { NEGATIVE<T>, NEGATIVE<T>, NEGATIVE<T> };
	template<typename T> 
	inline constexpr static euler<T> MIN<euler<T>> = { MIN<T>, MIN<T>, MIN<T> };
	template<typename T> 
	inline constexpr static euler<T> MAX<euler<T>> = { MAX<T>, MAX<T>, MAX<T> };
	template<typename T> 
	inline constexpr static euler<T> PI<euler<T>> = { PI<T>, PI<T>, PI<T> };
	template<typename T> 
	inline constexpr static euler<T> TWO_PI<euler<T>> = { TWO_PI<T>, TWO_PI<T>, TWO_PI<T> };
	template<typename T> 
	inline constexpr static euler<T> HALF_PI<euler<T>> = { HALF_PI<T>, HALF_PI<T>, HALF_PI<T> };

	/**
	 * @brief Generates euler rotation around an axis
	 * @param axis - axis (normalized)
	 * @param angle - radian angle around axis
	 * @return euelr angles
	*/
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] static euler<T> eulerRotationAround( const vec4<T>& axis, T angle )
	{
		vec4<T> q = axis * std::sin( angle * HALF<T> );
		q.w = std::cos( angle * HALF<T> );

		return {
			std::atan2( TWO<T> * ( q.w * q.x + q.y * q.z ), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z ),
			std::asin( std::clamp( TWO<T> * ( q.w * q.y - q.x * q.z ), NEGATIVE<T>, ONES<T> ) ),
			std::atan2( TWO<T> * ( q.x * q.y + q.w * q.z ), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z )
		};
	}

	/**
	 * @brief Generates rotation between two axes
	 * @tparam T - rotation type
	 * @param from - initial axis (normalized)
	 * @param to - final axis (normalized)
	 * @return rotation between axes
	*/
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] static eulerf32 eulerRotationBetween( const vec4f& from, const vec4f& to )
	{
		float product = dot_3D( from, to );
		if( product > ONES<T> - EPSILON<T> ) return Math::ZERO<euler<T>>;
		if( product < NEGATIVE<T> + EPSILON<T> ) return eulerRotationAround( orthogonal( from ), Math::PI<T> );
		return eulerRotationAround( normalize( cross( from, to ) ), std::acos( product ) );
	}

	/**
	 * @brief Converts from degrees to radians.
	 * @param degrees - angle in degrees
	 * @return angle in radians
	*/
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] constexpr euler<T> toRadians( const euler<T>& degrees )
	{
		return degrees * ( Math::PI<T> / 180.0f ); // TODO: Constant floating-point value
	}

	/**
	 * @brief Converts from radians to degrees.
	 * @param radians - angle in radians
	 * @return angle in degrees
	*/
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] constexpr euler<T> toDegrees( const euler<T>& radians )
	{
		return radians * ( 180.0f / Math::PI<T> ); // TODO: Constant floating-point value
	}

	/**
	 * @brief Calculates the sine of the euler angles
	 * @param a - euler angles
	 * @return sin vector
	*/
	[[nodiscard]] static eulerf32 sin( const eulerf32& rotation )
	{
		eulerf32 r;
		r.simd = _mm_sin_ps( rotation.simd );
		return r;
	}

	/**
	 * @brief Calculates the cosine of the euler angles
	 * @param a - euler angles
	 * @return cos vector
	*/
	[[nodiscard]] static eulerf32 cos( const eulerf32& rotation )
	{
		eulerf32 r;
		r.simd = _mm_cos_ps( rotation.simd );
		return r;
	}

	/**
	 * @brief Calculates the tan of the euler angles
	 * @param a - euler angles
	 * @return tan vector
	*/
	[[nodiscard]] static eulerf32 tan( const eulerf32& rotation )
	{
		eulerf32 r;
		r.simd = _mm_tan_ps( rotation.simd );
		return r;
	}

	/**
	 * @brief Rotates a vector around the origin using euler angles
	 * @param position - position vector
	 * @param rotation - euler angles
	 * @return rotated position
	*/
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] static vec4<T> rotate( const vec4<T>& position, const euler<T>& rotation )
	{
		euler<T> s = sin( rotation );
		euler<T> c = cos( rotation );

		vec4<T> r;
		vf::simd::dot4( r, 
			position, vec4<T>( c.z * c.y, c.z * s.y * s.x - s.z * c.x, c.z * s.y * c.x + s.z * s.x, ZERO<T> ),
			position, vec4<T>( s.z * c.y, s.z * s.y * s.x + c.z * c.x, s.z * s.y * c.x - c.z * s.x, ZERO<T> ),
			position, vec4<T>( -s.y, c.y * s.x, c.y * c.x, ZERO<T> ),
			position, Math::axis::W<vec4<T>>
		);
		return r;
	}

	/**
	 * @brief Generates random euler angles within the range
	 * @param min - minimum angles
	 * @param max - maximum angles
	 * @return random euler angles
	*/
	template<typename T>
	[[nodiscard]] static euler<T> random( const euler<T>& min, const euler<T>& max )
	{
		return { random<T>( min.x, max.x ), random<T>( min.y, max.y ), random<T>( min.z, max.z ) };
	}

	/**
	 * @brief Generates random euler rotation
	 * @return random euler angles
	*/
	template<> [[nodiscard]] static eulerf32 randomRotation<eulerf32>()
	{
		return random( ZERO<eulerf32>, TWO_PI<eulerf32> );
	}
}

#endif