#pragma once
#ifndef VF_QUATERNION_HPP
#define VF_QUATERNION_HPP

#include "vec4f.hpp"

static __m128 _mm_hamilton_ps( const __m128& q0, const __m128& q1 )
{
	constexpr static __m128 mskx = { 1.0f,  1.0f, -1.0f, 1.0f };
	constexpr static __m128 msky = { -1.0f,  1.0f,  1.0f, 1.0f };
	constexpr static __m128 mskz = { 1.0f, -1.0f,  1.0f, 1.0f };
	constexpr static __m128 mskw = { -1.0f, -1.0f, -1.0f, 1.0f };

	return _mm_dot_ps(
		_mm_mul_ps( q0, mskx ), _mm_permute_ps( q1, swizzle::WZYX ),
		_mm_mul_ps( q0, msky ), _mm_permute_ps( q1, swizzle::ZWXY ),
		_mm_mul_ps( q0, mskz ), _mm_permute_ps( q1, swizzle::YXWZ ),
		_mm_mul_ps( q0, mskw ), _mm_permute_ps( q1, swizzle::XYZW )
	);

	/*
	return {
		 q0.x * q1.w + q0.y * q1.z - q0.z * q1.y + q0.w * q1.x,
		-q0.x * q1.z + q0.y * q1.w + q0.z * q1.x + q0.w * q1.y,
		 q0.x * q1.y - q0.y * q1.x + q0.z * q1.w + q0.w * q1.z,
		-q0.x * q1.x - q0.y * q1.y - q0.z * q1.z + q0.w * q1.w
	};*/
}

union quat
{
	__m128 simd;
	float v[4];
	struct { float x, y, z, w; };

	// Operators
	quat& operator+=( const quat& b ) noexcept { simd = _mm_add_ps( simd, b.simd ); return *this; }
	quat& operator-=( const quat& b ) noexcept { simd = _mm_sub_ps( simd, b.simd ); return *this; }
	quat& operator*=( const quat& b ) noexcept { simd = _mm_hamilton_ps( simd, b.simd ); return *this; }
	quat& operator*=( const float b ) noexcept { simd = _mm_mul_ps( simd, _mm_set1_ps( b ) ); return *this; }
	quat& operator/=( const float b ) noexcept { simd = _mm_div_ps( simd, _mm_set1_ps( b ) ); return *this; }

	[[nodiscard]] quat operator-() const { return quat{ _mm_mul_ps( simd, _mm_set1_ps( -1.0f ) ) }; }

	/**
	 * @brief Calculates the squared norm of the quaternion
	 * @return squared norm of the quaternion
	*/
	[[nodiscard]] constexpr float norm2() const noexcept
	{
		return x * x + y * y + z * z + w * w;
	}

	/**
	 * @brief Calculates the norm of the quaternion
	 * @return norm of the quaternion
	*/
	[[nodiscard]] float norm() const noexcept
	{
		return sqrtf( x * x + y * y + z * z + w * w );
	}

	/**
	 * @brief Calculates the inverse of the quaternion
	 * @return inverse of the quaternion
	*/
	[[nodiscard]] quat inverse() const noexcept
	{
		return { _mm_mul_ps( simd, _mm_xyzw_ps( -1.0f, -1.0f, -1.0f, 1.0f ) ) };
	}
};

[[nodiscard]] static quat operator+( const quat& a, const quat& b ) noexcept { return { _mm_add_ps( a.simd, b.simd ) }; }
[[nodiscard]] static quat operator-( const quat& a, const quat& b ) noexcept { return { _mm_sub_ps( a.simd, b.simd ) }; }
[[nodiscard]] static quat operator*( const quat& a, const quat& b ) noexcept { return { _mm_hamilton_ps( a.simd, b.simd ) }; }

[[nodiscard]] static quat operator*( const quat& a, const float b ) noexcept { return { _mm_mul_ps( a.simd, _mm_set1_ps( b ) ) }; }
[[nodiscard]] static quat operator/( const quat& a, const float b ) noexcept { return { _mm_div_ps( a.simd, _mm_set1_ps( b ) ) }; }
[[nodiscard]] static quat operator*( const float a, const quat& b ) noexcept { return { _mm_mul_ps( _mm_set1_ps( a ), b.simd ) }; }

#endif