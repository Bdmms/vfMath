#pragma once
#ifndef VF_EULER_MATH_HPP
#define VF_EULER_MATH_HPP

#include "VectorMath.hpp"
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
	template<> 
	[[nodiscard]] static eulerf32 rotationAround( const vec4f& axis, const float angle )
	{
		vec4f q = axis * sinf( angle * 0.5f );
		q.w = cosf( angle * 0.5f );

		return {
			atan2f( 2.0f * ( q.w * q.x + q.y * q.z ), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z ),
			asinf( std::clamp( -2.0f * ( q.x * q.z - q.w * q.y ), -1.0f, 1.0f ) ),
			atan2f( 2.0f * ( q.x * q.y + q.w * q.z ), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z )
		};
	}

	/**
	 * @brief Generates rotation between two axes
	 * @tparam T - rotation type
	 * @param from - initial axis (normalized)
	 * @param to - final axis (normalized)
	 * @return rotation between axes
	*/
	template<> [[nodiscard]] static eulerf32 rotationBetween( const vec4f& from, const vec4f& to )
	{
		float product = dot_3D( from, to );
		if( product > 0.999999f ) return { 0.0f, 0.0f, 0.0f };
		if( product < -0.999999f ) return rotationAround<eulerf32>( orthogonal( from ), Math::PI<float> );
		return rotationAround<eulerf32>( normalize( cross( from, to ) ), acosf( product ) );
	}

	/**
	 * @brief Converts from degrees to radians.
	 * @param degrees - angle in degrees
	 * @return angle in radians
	*/
	template<typename T>
	[[nodiscard]] constexpr euler<T> toRadians( const euler<T>& degrees )
	{
		return degrees * ( Math::PI<T> / 180.0f ); // TODO: Constant floating-point value
	}

	/**
	 * @brief Converts from radians to degrees.
	 * @param radians - angle in radians
	 * @return angle in degrees
	*/
	template<typename T>
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
	[[nodiscard]] static vec4f rotate( const vec4f& position, const eulerf32& rotation )
	{
		eulerf32 s = sin( rotation );
		eulerf32 c = cos( rotation );

		vec4f r;
		vf::simd::dot4( r, 
			position, vec4f( c.z * c.y, c.z * s.y * s.x - s.z * c.x, c.z * s.y * c.x + s.z * s.x, 0.0f ),
			position, vec4f( s.z * c.y, s.z * s.y * s.x + c.z * c.x, s.z * s.y * c.x - c.z * s.x, 0.0f ),
			position, vec4f( -s.y, c.y * s.x, c.y * c.x, 0.0f ),
			position, Math::axis::W<vec4f>
		);
		return r;
	}

	/**
	 * @brief Generates random euler angles within the range
	 * @param min - minimum angles
	 * @param max - maximum angles
	 * @return random euler angles
	*/
	template<> [[nodiscard]] static eulerf32 random<eulerf32>( const eulerf32& min, const eulerf32& max )
	{
		eulerf32 r;
		r.simd = _mm_fmadd_ps( _mm_div_ps( _mm_cvtepi32_ps( _mm_set_epi32( rand(), rand(), rand(), 0 ) ), _mm_set1_ps( RAND_MAX_FLOAT ) ), _mm_sub_ps( max.simd, min.simd ), min.simd );
		return r;
	}

	/**
	 * @brief Generates random euler rotation
	 * @return random euler angles
	*/
	template<> [[nodiscard]] static eulerf32 randomRotation<eulerf32>()
	{
		return random<eulerf32>( ZERO<eulerf32>, TWO_PI<eulerf32> );
	}
}

#endif