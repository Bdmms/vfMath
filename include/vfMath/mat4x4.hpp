#pragma once
#ifndef VF_MAT_4x4_HPP
#define VF_MAT_4x4_HPP

#include "VectorMath.hpp"

/**
 * @brief Performs cofactor expansion on rows OR columns.
 * This effectively calculates the determinant of four 3x3 matrices.
 * @param v1 - row or column A
 * @param v2 - row or column B
 * @param v3 - row or column C
 * @return Cofactor expanded row/column
*/
[[nodiscard]] static __m128 cofactorExpansion_SSE2(const __m128& v1, const __m128& v2, const __m128& v3)
{
	// Mask out the last element
	constexpr static __m128 msk = { 1.0f, -1.0f, 1.0f, 0.0f };

	// Row breakdown and parallel dot product
	return _mm_dot_ps(
		_mm_mul_ps(msk, _mm_permute_ps(v1, swizzle::YZW)),
		_mm_sub_ps(_mm_mul_ps(_mm_permute_ps(v2, swizzle::ZYY), _mm_permute_ps(v3, swizzle::WWZ)), _mm_mul_ps(_mm_permute_ps(v3, swizzle::ZYY), _mm_permute_ps(v2, swizzle::WWZ))),
		_mm_mul_ps(msk, _mm_permute_ps(v1, swizzle::XZW)),
		_mm_sub_ps(_mm_mul_ps(_mm_permute_ps(v2, swizzle::ZXX), _mm_permute_ps(v3, swizzle::WWZ)), _mm_mul_ps(_mm_permute_ps(v3, swizzle::ZXX), _mm_permute_ps(v2, swizzle::WWZ))),
		_mm_mul_ps(msk, _mm_permute_ps(v1, swizzle::XYW)),
		_mm_sub_ps(_mm_mul_ps(_mm_permute_ps(v2, swizzle::YXX), _mm_permute_ps(v3, swizzle::WWY)), _mm_mul_ps(_mm_permute_ps(v3, swizzle::YXX), _mm_permute_ps(v2, swizzle::WWY))),
		_mm_mul_ps(msk, v1),
		_mm_sub_ps(_mm_mul_ps(_mm_permute_ps(v2, swizzle::YXX), _mm_permute_ps(v3, swizzle::ZZY)), _mm_mul_ps(_mm_permute_ps(v3, swizzle::YXX), _mm_permute_ps(v2, swizzle::ZZY))));
}

/**
 * @brief Performs cofactor expansion on rows OR columns.
 * This effectively calculates the determinant of four 3x3 matrices.
 * @param v1 - row or column A
 * @param v2 - row or column B
 * @param v3 - row or column C
 * @return Cofactor expanded row/column
*/
[[nodiscard]] static __m256 cofactorExpansion_AVX2(const __m256& v1, const __m256& v2, const __m256& v3)
{
	// Mask out the last element
	constexpr static __m256 msk = { 1.0f, -1.0f, 1.0f, 0.0f, 1.0f, -1.0f, 1.0f, 0.0f };

	// Row breakdown and parallel dot product
	return _mm256_dot_ps(
		_mm256_mul_ps(msk, _mm256_permute_ps(v1, swizzle::YZW)),
		_mm256_sub_ps(_mm256_mul_ps(_mm256_permute_ps(v2, swizzle::ZYY), _mm256_permute_ps(v3, swizzle::WWZ)), _mm256_mul_ps(_mm256_permute_ps(v3, swizzle::ZYY), _mm256_permute_ps(v2, swizzle::WWZ))),
		_mm256_mul_ps(msk, _mm256_permute_ps(v1, swizzle::XZW)),
		_mm256_sub_ps(_mm256_mul_ps(_mm256_permute_ps(v2, swizzle::ZXX), _mm256_permute_ps(v3, swizzle::WWZ)), _mm256_mul_ps(_mm256_permute_ps(v3, swizzle::ZXX), _mm256_permute_ps(v2, swizzle::WWZ))),
		_mm256_mul_ps(msk, _mm256_permute_ps(v1, swizzle::XYW)),
		_mm256_sub_ps(_mm256_mul_ps(_mm256_permute_ps(v2, swizzle::YXX), _mm256_permute_ps(v3, swizzle::WWY)), _mm256_mul_ps(_mm256_permute_ps(v3, swizzle::YXX), _mm256_permute_ps(v2, swizzle::WWY))),
		_mm256_mul_ps(msk, v1),
		_mm256_sub_ps(_mm256_mul_ps(_mm256_permute_ps(v2, swizzle::YXX), _mm256_permute_ps(v3, swizzle::ZZY)), _mm256_mul_ps(_mm256_permute_ps(v3, swizzle::YXX), _mm256_permute_ps(v2, swizzle::ZZY))));
}

/**
 * @brief 4x4 matrix
*/
union mat4x4
{
	vec4f col[4];
	struct { vec4f x_axis, y_axis, z_axis, origin; };
	__m128 simd[4];
	__m256 avx2[2];
#if ENABLE_INSTRUCTIONS_AVX512
	__m512 avx512;
#endif
	float m[16];

	constexpr mat4x4() : m() {}

	constexpr mat4x4( const float a, const float b, const float c, const float d,
		const float e, const float f, const float g, const float h,
		const float i, const float j, const float k, const float l,
		const float m, const float n, const float o, const float p )
		: m{ a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p } {}

	constexpr mat4x4( const vec4f& a, const vec4f& b, const vec4f& c, const vec4f& d )
		: col{ a,b,c,d } {}

	constexpr mat4x4( const __m128& a, const __m128& b, const __m128& c, const __m128& d )
		: simd{ a,b,c,d } {}

	constexpr mat4x4( const __m256& a, const __m256& b )
		: avx2{ a,b } {}

#if ENABLE_INSTRUCTIONS_AVX512
	constexpr mat4x4( const __m512& a ) : avx512( a ) {}
#endif

	constexpr float operator[]( const unsigned char i ) const { return m[i]; }
	constexpr float& operator[]( const unsigned char i ) { return m[i]; }

	mat4x4& operator+=( const mat4x4& b ) noexcept
	{
#if ENABLE_INSTRUCTIONS_AVX512
		avx512 = _mm512_add_ps( avx512, b.avx512 );
#elif ENABLE_INSTRUCTIONS_AVX2
		avx2[0] = _mm256_add_ps( avx2[0], b.avx2[0] );
		avx2[1] = _mm256_add_ps( avx2[1], b.avx2[1] );
#elif ENABLE_INSTRUCTIONS_SSE2
		simd[0] = _mm_add_ps( simd[0], b.simd[0] );
		simd[1] = _mm_add_ps( simd[1], b.simd[1] );
		simd[2] = _mm_add_ps( simd[2], b.simd[2] );
		simd[3] = _mm_add_ps( simd[3], b.simd[3] );
#else
		for( u32 i = 0; i < 16; ++i ) m[i] += b.m[i];
#endif
		return *this;
	}

	mat4x4& operator-=( const mat4x4& b ) noexcept
	{
#if ENABLE_INSTRUCTIONS_AVX512
		avx512 = _mm512_sub_ps( avx512, b.avx512 );
#elif ENABLE_INSTRUCTIONS_AVX2
		avx2[0] = _mm256_sub_ps( avx2[0], b.avx2[0] );
		avx2[1] = _mm256_sub_ps( avx2[1], b.avx2[1] );
#elif ENABLE_INSTRUCTIONS_SSE2
		simd[0] = _mm_sub_ps( simd[0], b.simd[0] );
		simd[1] = _mm_sub_ps( simd[1], b.simd[1] );
		simd[2] = _mm_sub_ps( simd[2], b.simd[2] );
		simd[3] = _mm_sub_ps( simd[3], b.simd[3] );
#else
		for( u32 i = 0; i < 16; ++i ) m[i] -= b.m[i];
#endif
		return *this;
	}

	mat4x4& operator*=( const float b ) noexcept
	{
#if ENABLE_INSTRUCTIONS_AVX512
		avx512 = _mm512_mul_ps( avx512, _mm512_set1_ps( b ) );
#elif ENABLE_INSTRUCTIONS_AVX2
		__m256 scale = _mm256_set1_ps( b );
		avx2[0] = _mm256_mul_ps( avx2[0], scale );
		avx2[1] = _mm256_mul_ps( avx2[1], scale );
#elif ENABLE_INSTRUCTIONS_SSE2
		__m128 scale = _mm_set1_ps( b );
		simd[0] = _mm_mul_ps( simd[0], scale );
		simd[1] = _mm_mul_ps( simd[1], scale );
		simd[2] = _mm_mul_ps( simd[2], scale );
		simd[3] = _mm_mul_ps( simd[3], scale );
#else
		for( u32 i = 0; i < 16; ++i ) m[i] *= b;
#endif
		return *this;
	}

	mat4x4& operator/=( const float b ) noexcept
	{
#if ENABLE_INSTRUCTIONS_AVX512
		avx512 = _mm512_div_ps( avx512, _mm512_set1_ps( b ) );
#elif ENABLE_INSTRUCTIONS_AVX2
		__m256 scale = _mm256_set1_ps( b );
		avx2[0] = _mm256_div_ps( avx2[0], scale );
		avx2[1] = _mm256_div_ps( avx2[1], scale );
#elif ENABLE_INSTRUCTIONS_SSE2
		__m128 scale = _mm_set1_ps( b );
		simd[0] = _mm_div_ps( simd[0], scale );
		simd[1] = _mm_div_ps( simd[1], scale );
		simd[2] = _mm_div_ps( simd[2], scale );
		simd[3] = _mm_div_ps( simd[3], scale );
#else
		for( u32 i = 0; i < 16; ++i ) m[i] /= b;
#endif
		return *this;
	}

	mat4x4& operator*=( const mat4x4& b ) noexcept
	{
#if ENABLE_INSTRUCTIONS_AVX512
		// TODO
#elif ENABLE_INSTRUCTIONS_AVX2
		__m256 r0 = { m[0], m[4], m[8], m[12], m[0], m[4], m[8], m[12] };
		__m256 r1 = { m[1], m[5], m[9], m[13], m[1], m[5], m[9], m[13] };
		__m256 r2 = { m[2], m[6], m[10], m[14], m[2], m[6], m[10], m[14] };
		__m256 r3 = { m[3], m[7], m[11], m[15], m[3], m[7], m[11], m[15] };
		avx2[0] = _mm256_dot_ps( b.avx2[0], r0, b.avx2[0], r1, b.avx2[0], r2, b.avx2[0], r3 );
		avx2[1] = _mm256_dot_ps( b.avx2[1], r0, b.avx2[1], r1, b.avx2[1], r2, b.avx2[1], r3 );
#elif ENABLE_INSTRUCTIONS_SSE2
		__m128 r0 = _mm_xyzw_ps( m[0], m[4], m[8], m[12] );
		__m128 r1 = _mm_xyzw_ps( m[1], m[5], m[9], m[13] );
		__m128 r2 = _mm_xyzw_ps( m[2], m[6], m[10], m[14] );
		__m128 r3 = _mm_xyzw_ps( m[3], m[7], m[11], m[15] );
		simd[0] = _mm_dot_ps( b.simd[0], r0, b.simd[0], r1, b.simd[0], r2, b.simd[0], r3 );
		simd[1] = _mm_dot_ps( b.simd[1], r0, b.simd[1], r1, b.simd[1], r2, b.simd[1], r3 );
		simd[2] = _mm_dot_ps( b.simd[2], r0, b.simd[2], r1, b.simd[2], r2, b.simd[2], r3 );
		simd[3] = _mm_dot_ps( b.simd[3], r0, b.simd[3], r1, b.simd[3], r2, b.simd[3], r3 );
#else
		// TODO
#endif
		return *this;
	}

	// Conversions
	[[nodiscard]] operator mat2x2() const;

	/**
	 * @brief Calculates the determinant of this matrix
	 * @return determinant
	*/
	[[nodiscard]] float determinant() const
	{
		constexpr static __m128 msk = { 1.0f, -1.0f, 1.0f, -1.0f };

		// Dot product between simdumn and its cofactor expansion
		return Math::dot( col[0], { _mm_mul_ps( msk, cofactorExpansion_SSE2( simd[1], simd[2], simd[3] ) ) } );
	}

	/**
	 * @brief Creates a transposed matrix
	 * @return transposed matrix
	*/
	[[nodiscard]] mat4x4 transpose() const
	{
#if ENABLE_INSTRUCTIONS_AVX512
		// TODO
#elif ENABLE_INSTRUCTIONS_AVX2
		__m256i permutation = _mm256_set_epi32( 7, 3, 5, 1, 6, 2, 4, 0 );
		__m256 a0 = _mm256_permutevar8x32_ps( avx2[0], permutation );
		__m256 a1 = _mm256_permutevar8x32_ps( avx2[1], permutation );

		return mat4x4{
			_mm256_shuffle_ps( a0, _mm256_permute_ps( a1, swizzle::ZWXY ), swizzle::XYZW ),
			_mm256_shuffle_ps( _mm256_permute_ps( a0, swizzle::ZWXY ), a1, swizzle::XYZW )
		};
#elif ENABLE_INSTRUCTIONS_SSE2
		return mat4x4( {
			_mm_transpose_ps( simd[0], simd[1], simd[2], simd[3], swizzle::XXXX ),
			_mm_transpose_ps( simd[0], simd[1], simd[2], simd[3], swizzle::YYYY ),
			_mm_transpose_ps( simd[0], simd[1], simd[2], simd[3], swizzle::ZZZZ ),
			_mm_transpose_ps( simd[0], simd[1], simd[2], simd[3], swizzle::WWWW )
			} );
#else
		// TODO
#endif
	}

	/**
	 * @brief Creates an inverse matrix
	 * @return inverse matrix
	*/
	[[nodiscard]] mat4x4 inverse() const
	{
#if ENABLE_INSTRUCTIONS_AVX512
		// TODO
#elif ENABLE_INSTRUCTIONS_AVX2
		constexpr static __m256 msk = { 1.0f, -1.0f, 1.0f, -1.0f, -1.0f, 1.0f, -1.0f, 1.0f };

		// Calculate the cofactor expansion for every column
		__m256 adj0_1 = _mm256_mul_ps( msk, cofactorExpansion_AVX2( _mm256_set_m128( simd[0], simd[1] ), _mm256_set_m128( simd[2], simd[2] ), _mm256_set_m128( simd[3], simd[3] ) ) );
		__m256 adj2_3 = _mm256_mul_ps( msk, cofactorExpansion_AVX2( _mm256_set_m128( simd[0], simd[0] ), _mm256_set_m128( simd[1], simd[1] ), _mm256_set_m128( simd[2], simd[3] ) ) );

		__m256 product = _mm256_mul_ps( avx2[0], adj0_1 );
		__m256 det = _mm256_hadd_ps( product, product );
		det = _mm256_hadd_ps( det, det );

		adj0_1 = _mm256_div_ps( adj0_1, det );
		adj2_3 = _mm256_div_ps( adj2_3, det );

		// Transpose the matrix
		__m256i permutation = _mm256_set_epi32( 7, 3, 5, 1, 6, 2, 4, 0 );
		__m256 a0 = _mm256_permutevar8x32_ps( adj0_1, permutation );
		__m256 a1 = _mm256_permutevar8x32_ps( adj2_3, permutation );

		return mat4x4{
			_mm256_shuffle_ps( a0, _mm256_permute_ps( a1, swizzle::ZWXY ), swizzle::XYZW ),
			_mm256_shuffle_ps( _mm256_permute_ps( a0, swizzle::ZWXY ), a1, swizzle::XYZW )
		};

#elif ENABLE_INSTRUCTIONS_SSE2
		constexpr static __m128 odd = { 1.0f, -1.0f, 1.0f, -1.0f };
		constexpr static __m128 even = { -1.0f, 1.0f, -1.0f, 1.0f };

		// Calculate the cofactor expansion for every column
		__m128 adj0 = _mm_mul_ps( odd, cofactorExpansion_SSE2( simd[1], simd[2], simd[3] ) );
		__m128 adj1 = _mm_mul_ps( even, cofactorExpansion_SSE2( simd[0], simd[2], simd[3] ) );
		__m128 adj2 = _mm_mul_ps( odd, cofactorExpansion_SSE2( simd[0], simd[1], simd[3] ) );
		__m128 adj3 = _mm_mul_ps( even, cofactorExpansion_SSE2( simd[0], simd[1], simd[2] ) );

		// Calculate the determinant from one of the columns
		__m128 product = _mm_mul_ps( simd[0], adj0 );
		__m128 det = _mm_hadd_ps( product, product );
		det = _mm_hadd_ps( det, det );

		// Divide adjugate matrix by determinant
		adj0 = _mm_div_ps( adj0, det );
		adj1 = _mm_div_ps( adj1, det );
		adj2 = _mm_div_ps( adj2, det );
		adj3 = _mm_div_ps( adj3, det );

		// Transpose the matrix
		return mat4x4( {
				_mm_transpose_ps( adj0, adj1, adj2, adj3, swizzle::XXXX ),
				_mm_transpose_ps( adj0, adj1, adj2, adj3, swizzle::YYYY ),
				_mm_transpose_ps( adj0, adj1, adj2, adj3, swizzle::ZZZZ ),
				_mm_transpose_ps( adj0, adj1, adj2, adj3, swizzle::WWWW ),
			} );
#else
		// TODO
#endif
	}
};

[[nodiscard]] static mat4x4 operator+( const mat4x4& a, const mat4x4& b ) noexcept
{
#if ENABLE_INSTRUCTIONS_AVX512
	return mat4x4{ _mm512_add_ps( avx512, b.avx512 ) };
#elif ENABLE_INSTRUCTIONS_AVX2
	return mat4x4{
		_mm256_add_ps( a.avx2[0],b.avx2[0] ),
		_mm256_add_ps( a.avx2[1],b.avx2[1] )
	};
#elif ENABLE_INSTRUCTIONS_SSE2
	return mat4x4( {
		_mm_add_ps( a.simd[0],b.simd[0] ),
		_mm_add_ps( a.simd[1],b.simd[1] ),
		_mm_add_ps( a.simd[2],b.simd[2] ),
		_mm_add_ps( a.simd[3],b.simd[3] ),
		} );
#else
	// TODO
#endif
}

[[nodiscard]] static mat4x4 operator-( const mat4x4& a, const mat4x4& b ) noexcept
{
#if ENABLE_INSTRUCTIONS_AVX512
	return mat4x4{ _mm512_sub_ps( avx512, b.avx512 ) };
#elif ENABLE_INSTRUCTIONS_AVX2
	return mat4x4{
		_mm256_sub_ps( a.avx2[0],b.avx2[0] ),
		_mm256_sub_ps( a.avx2[1],b.avx2[1] )
	};
#elif ENABLE_INSTRUCTIONS_SSE2
	return mat4x4( {
		_mm_sub_ps( a.simd[0],b.simd[0] ),
		_mm_sub_ps( a.simd[1],b.simd[1] ),
		_mm_sub_ps( a.simd[2],b.simd[2] ),
		_mm_sub_ps( a.simd[3],b.simd[3] ),
		} );
#else
	// TODO
#endif
}

[[nodiscard]] static mat4x4 operator*( const mat4x4& a, const float b ) noexcept
{
#if ENABLE_INSTRUCTIONS_AVX512
	return mat4x4{ _mm512_mul_ps( avx512, _mm512_set1_ps( b ) ) };
#elif ENABLE_INSTRUCTIONS_AVX2
	__m256 scale = _mm256_set1_ps( b );
	return mat4x4{
		_mm256_mul_ps( a.avx2[0],scale ),
		_mm256_mul_ps( a.avx2[1],scale )
	};
#elif ENABLE_INSTRUCTIONS_SSE2
	__m128 scale = _mm_set1_ps( b );
	return mat4x4( {
		_mm_mul_ps( a.simd[0],scale ),
		_mm_mul_ps( a.simd[1],scale ),
		_mm_mul_ps( a.simd[2],scale ),
		_mm_mul_ps( a.simd[3],scale ),
		} );
#else
	// TODO
#endif
}

[[nodiscard]] static mat4x4 operator*( const float a, const mat4x4& b ) noexcept
{
#if ENABLE_INSTRUCTIONS_AVX512
	return mat4x4{ _mm512_mul_ps( b.avx512, _mm512_set1_ps( a ) ) };
#elif ENABLE_INSTRUCTIONS_AVX2
	__m256 scale = _mm256_set1_ps( a );
	return mat4x4{
		_mm256_mul_ps( b.avx2[0],scale ),
		_mm256_mul_ps( b.avx2[1],scale )
	};
#elif ENABLE_INSTRUCTIONS_SSE2
	__m128 scale = _mm_set1_ps( a );
	return mat4x4( {
		_mm_mul_ps( b.simd[0],scale ),
		_mm_mul_ps( b.simd[1],scale ),
		_mm_mul_ps( b.simd[2],scale ),
		_mm_mul_ps( b.simd[3],scale ),
		} );
#else
	// TODO
#endif
}

[[nodiscard]] static mat4x4 operator/( const mat4x4& a, const float b ) noexcept
{
#if ENABLE_INSTRUCTIONS_AVX512
	return mat4x4{ _mm512_div_ps( a.avx512, _mm512_set1_ps( b ) ) };
#elif ENABLE_INSTRUCTIONS_AVX2
	__m256 scale = _mm256_set1_ps( b );
	return mat4x4{
		_mm256_div_ps( a.avx2[0],scale ),
		_mm256_div_ps( a.avx2[1],scale )
	};
#elif ENABLE_INSTRUCTIONS_SSE2
	__m128 scale = _mm_set1_ps( b );
	return mat4x4( {
		_mm_div_ps( a.simd[0],scale ),
		_mm_div_ps( a.simd[1],scale ),
		_mm_div_ps( a.simd[2],scale ),
		_mm_div_ps( a.simd[3],scale ),
		} );
#else
	// TODO
#endif
}

[[nodiscard]] static vec4f operator*( const mat4x4& a, const vec4f& b ) noexcept
{
#if ENABLE_INSTRUCTIONS_SSE2
	__m128 t0 = _mm_shuffle_ps( a.x_axis.simd, a.y_axis.simd, 0x44 );
	__m128 t2 = _mm_shuffle_ps( a.x_axis.simd, a.y_axis.simd, 0xEE );
	__m128 t1 = _mm_shuffle_ps( a.z_axis.simd, a.origin.simd, 0x44 );
	__m128 t3 = _mm_shuffle_ps( a.z_axis.simd, a.origin.simd, 0xEE );

	return { _mm_dot_ps(
		_mm_shuffle_ps( t0, t1, 0x88 ), b.simd,
		_mm_shuffle_ps( t0, t1, 0xDD ), b.simd,
		_mm_shuffle_ps( t2, t3, 0x88 ), b.simd,
		_mm_shuffle_ps( t2, t3, 0xDD ), b.simd ) };

	/*return {_mm_dot_ps(b.simd, _mm_xyzw_ps(a.m[0], a.m[4], a.m[8], a.m[12]),
						b.simd, _mm_xyzw_ps(a.m[1], a.m[5], a.m[9], a.m[13]),
						b.simd, _mm_xyzw_ps(a.m[2], a.m[6], a.m[10], a.m[14]),
						b.simd, _mm_xyzw_ps(a.m[3], a.m[7], a.m[11], a.m[15])) };*/
#else
	// TODO
#endif
}

[[nodiscard]] static vec4f operator*( const vec4f& a, const mat4x4& b ) noexcept
{
#if ENABLE_INSTRUCTIONS_SSE2
	return { _mm_dot_ps( a.simd, b.simd[0], a.simd, b.simd[1], a.simd, b.simd[2], a.simd, b.simd[3] ) };
#else
	// TODO
#endif
}

/*
* [default]		= 38579200 ns (avg. 34019200 ns)
* [parallel]	= 19865300 ns (avg. 18801620 ns)
* [unrepeated]	= 15184700 ns (avg. 14801640 ns)
*/
[[nodiscard]] static mat4x4 operator*( const mat4x4& a, const mat4x4& b ) noexcept
{
#if ENABLE_INSTRUCTIONS_AVX512
	// TODO
#elif ENABLE_INSTRUCTIONS_AVX2
	__m256 r0 = { a.m[0], a.m[4], a.m[8], a.m[12], a.m[0], a.m[4], a.m[8], a.m[12] };
	__m256 r1 = { a.m[1], a.m[5], a.m[9], a.m[13], a.m[1], a.m[5], a.m[9], a.m[13] };
	__m256 r2 = { a.m[2], a.m[6], a.m[10], a.m[14], a.m[2], a.m[6], a.m[10], a.m[14] };
	__m256 r3 = { a.m[3], a.m[7], a.m[11], a.m[15], a.m[3], a.m[7], a.m[11], a.m[15] };
	return mat4x4{
		_mm256_dot_ps( b.avx2[0], r0, b.avx2[0], r1, b.avx2[0], r2, b.avx2[0], r3 ),
		_mm256_dot_ps( b.avx2[1], r0, b.avx2[1], r1, b.avx2[1], r2, b.avx2[1], r3 ) };
#elif ENABLE_INSTRUCTIONS_SSE2
	__m128 r0 = _mm_xyzw_ps( a.m[0], a.m[4], a.m[8], a.m[12] );
	__m128 r1 = _mm_xyzw_ps( a.m[1], a.m[5], a.m[9], a.m[13] );
	__m128 r2 = _mm_xyzw_ps( a.m[2], a.m[6], a.m[10], a.m[14] );
	__m128 r3 = _mm_xyzw_ps( a.m[3], a.m[7], a.m[11], a.m[15] );
	return mat4x4{
		_mm_dot_ps( b.simd[0], r0, b.simd[0], r1, b.simd[0], r2, b.simd[0], r3 ),
		_mm_dot_ps( b.simd[1], r0, b.simd[1], r1, b.simd[1], r2, b.simd[1], r3 ),
		_mm_dot_ps( b.simd[2], r0, b.simd[2], r1, b.simd[2], r2, b.simd[2], r3 ),
		_mm_dot_ps( b.simd[3], r0, b.simd[3], r1, b.simd[3], r2, b.simd[3], r3 ) };
#else
	// TODO
#endif
}

#endif