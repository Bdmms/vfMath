#pragma once
#ifndef VF_MAT_2x2_HPP
#define VF_MAT_2x2_HPP

#include "VectorMath.hpp"

/**
 * @brief 2x2 matrix
*/
union mat2x2
{
	using ElementType = float;
	static constexpr uint32_t ElementSize = 4u;

	__m128 simd;
	vec2f col[2];
	struct { vec2f x_axis, y_axis; };
	float m[4];

	constexpr mat2x2() : m() {}

	constexpr mat2x2( const float a, const float b, const float c, const float d )
		: m{ a, b, c, d } {}

	constexpr mat2x2( const vec2f& a, const vec2f& b )
		: col{ a, b } {}

	constexpr mat2x2( const __m128& a )
		: simd{ a } {}

	constexpr float operator[]( const unsigned char i ) const { return m[i]; }
	constexpr float& operator[]( const unsigned char i ) { return m[i]; }

	constexpr mat2x2& operator+=( const mat2x2& b ) noexcept { simd = SIMD_4F_ADD( simd, b.simd ); return *this; }
	constexpr mat2x2& operator-=( const mat2x2& b ) noexcept { simd = SIMD_4F_SUB( simd, b.simd ); return *this; }
	constexpr mat2x2& operator*=( const float b ) noexcept { simd = SIMD_4F_MUL( simd, SIMD_4F_SET1( b ) ); return *this; }
	constexpr mat2x2& operator/=( const float b ) noexcept { simd = SIMD_4F_DIV( simd, SIMD_4F_SET1( b ) ); return *this; }

	mat2x2& operator*=( const mat2x2& b ) noexcept
	{
		__m128 b0 = _mm_permute_ps( simd, swizzle::XYXY );
		__m128 b1 = _mm_permute_ps( b.simd, swizzle::XXZZ );
		__m128 b2 = _mm_permute_ps( simd, swizzle::ZWZW );
		__m128 b3 = _mm_permute_ps( b.simd, swizzle::YYWW );
		simd = _mm_add_ps( _mm_mul_ps( b0, b1 ), _mm_mul_ps( b2, b3 ) );
		return *this;
	}

	// Conversions
	[[nodiscard]] operator mat4x4() const;

	/**
	 * @brief Calculates the determinant of this matrix
	 * @return determinant
	*/
	[[nodiscard]] constexpr float determinant() const
	{
		return m[0] * m[3] - m[1] * m[2];
	}

	/**
	 * @brief Creates a transposed matrix
	 * @return transposed matrix
	*/
	[[nodiscard]] constexpr mat2x2 transpose() const
	{
		return mat2x2{ m[0], m[2], m[1], m[3] };
	}

	/**
	 * @brief Creates an inverse matrix
	 * @return inverse matrix
	*/
	[[nodiscard]] constexpr mat2x2 inverse() const
	{
		return mat2x2{ SIMD_4F_DIV( SIMD_4F_SET( m[3], -m[2], -m[1], m[0] ), SIMD_4F_SET1( determinant() ) ) };
	}
};

[[nodiscard]] constexpr mat2x2 operator+( const mat2x2& a, const mat2x2& b ) noexcept {
	return mat2x2( { SIMD_4F_ADD( a.simd, b.simd ) } );
}

[[nodiscard]] constexpr mat2x2 operator-( const mat2x2& a, const mat2x2& b ) noexcept {
	return mat2x2( { SIMD_4F_SUB( a.simd, b.simd ) } );
}

[[nodiscard]] constexpr mat2x2 operator*( const mat2x2& a, const float b ) noexcept {
	return mat2x2( { SIMD_4F_MUL( a.simd, _mm_set1_ps( b ) ) } );
}

[[nodiscard]] constexpr mat2x2 operator/( const mat2x2& a, const float b ) noexcept {
	return mat2x2( { SIMD_4F_DIV( a.simd, _mm_set1_ps( b ) ) } );
}

[[nodiscard]] constexpr vec2f operator*( const mat2x2& a, const vec2f b ) noexcept {
	return vec2f{ Math::dot( b, { a.m[0], a.m[2] } ), Math::dot( b, { a.m[1], a.m[3] } ) };
}

[[nodiscard]] constexpr mat2x2 operator*( const mat2x2& a, const mat2x2& b ) noexcept
{
	__m128 b0 = SIMD_4F_PERMUTE( a.simd, swizzle::XYXY );
	__m128 b1 = SIMD_4F_PERMUTE( b.simd, swizzle::XXZZ );
	__m128 b2 = SIMD_4F_PERMUTE( a.simd, swizzle::ZWZW );
	__m128 b3 = SIMD_4F_PERMUTE( b.simd, swizzle::YYWW );
	return mat2x2{ SIMD_4F_ADD( SIMD_4F_MUL( b0, b1 ), SIMD_4F_MUL( b2, b3 ) ) };
}

typedef mat2x2 mat2f;

#endif