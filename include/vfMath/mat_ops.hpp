#pragma once
#ifndef VF_MATRIX_OPERATIONS_HPP
#define VF_MATRIX_OPERATIONS_HPP

#include "constants.hpp"
#include "vec_ops.hpp"
#include "mat_store.hpp"

namespace vf::simd
{
	namespace compile_time
	{
		template<size_t M, size_t N, typename T>
		constexpr void set1( MatrixStorage<M, N, T>& r, T value )
		{
			for( size_t col = 0LLU; col < N; ++col )
			{
				set1( r.cols[col], value );
			}
		}

		template<size_t M, size_t N, typename T>
		constexpr void setIdentity( MatrixStorage<M, N, T>& r )
		{
			for( size_t col = 0LLU; col < N; ++col )
			{
				for( size_t row = 0LLU; row < M; ++row )
				{
					r.cols[col].elements[row] = row == col ? Math::ONES<T> : Math::ZERO<T>;
				}
			}
		}

		template<size_t M, size_t N, typename T>
		constexpr void transpose( MatrixStorage<N, M, T>& r, const MatrixStorage<M, N, T>& a )
		{
			for( size_t col = 0LLU; col < N; ++col )
			{
				for( size_t row = 0LLU; row < M; ++row )
				{
					r.cols[row].elements[col] = a.cols[col].elements[row];
				}
			}
		}

		template<typename T>
		constexpr T determinant( const MatrixStorage<2, 2, T>& a )
		{
			return a.cols[0].elements[0] * a.cols[1].elements[1] - a.cols[0].elements[1] * a.cols[1].elements[0];
		}

		//constexpr void invert_cofactor_expansion()
		//{

		//}

		//template<size_t N, typename T>
		//constexpr void invert( MatrixStorage<N, N, T>& r, const MatrixStorage<N, N, T>& a )
		//{
		//	// Compute Adjugate Matrix
		//	MatrixStorage<N, N, T> adj;
		//	for( size_t col = 0LLU; col < N; ++col )
		//	{
		//		for( size_t row = 0LLU; row < M; ++row )
		//		{
		//			adj.cols[col].elements[row] = 0;
		//		}
		//	}
		//	
		//	// Compute Determinant
		//	T det = dot( a.cols[0], adj.cols[0] );

		//	// Divide and Transpose
		//	for( size_t col = 0LLU; col < N; ++col )
		//	{
		//		for( size_t row = 0LLU; row < M; ++row )
		//		{
		//			r.cols[row].elements[col] = adj.cols[col].elements[row] / det;
		//		}
		//	}
		//}

		template<typename T>
		constexpr void invert( MatrixStorage<2, 2, T>& r, const MatrixStorage<2, 2, T>& a )
		{
			T det = determinant( a );
			r.cols[0].elements[0] = a.cols[1].elements[1] / det;
			r.cols[0].elements[1] = -a.cols[0].elements[1] / det;
			r.cols[1].elements[0] = -a.cols[1].elements[0] / det;
			r.cols[1].elements[1] = a.cols[0].elements[0] / det;
		}

		template<typename T>
		constexpr void invert( MatrixStorage<4, 4, T>& r, const MatrixStorage<4, 4, T>& a )
		{

		}

		template<size_t N0, size_t N1, size_t N2, typename T>
		constexpr void mul( MatrixStorage<N0, N2, T>& r, const MatrixStorage<N0, N1, T>& a, const MatrixStorage<N1, N2, T>& b )
		{
			MatrixStorage<N1, N0, T> t{};
			transpose( t, a );

			for( size_t col = 0LLU; col < N2; ++col )
			{
				for( size_t row = 0LLU; row < N0; ++row )
				{
					r.cols[col].elements[row] = dot( t.cols[row], b.cols[col] );
				}
			}
		}

		template<size_t M, size_t N, typename T>
		constexpr void mul( VectorStorage<M, T>& r, const MatrixStorage<M, N, T>& a, const VectorStorage<M, T>& b )
		{
			MatrixStorage<N, M, T> t{};
			transpose( t, a );

			for( size_t row = 0LLU; row < M; ++row )
			{
				r.elements[row] = dot<M, T>( t.cols[row], b );
			}
		}

		template<size_t M, size_t N, typename T>
		constexpr void mul( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const MatrixStorage<M, N, T>& b )
		{
			for( size_t col = 0LLU; col < N; ++col )
			{
				r.elements[col] = dot<N, T>( a, b.cols[col] );
			}
		}
	}

	template<size_t M, size_t N, typename T>
	constexpr void set1( MatrixStorage<M, N, T>& r, T value )
	{
		compile_time::set1( r, value );
	}

	template<size_t M, size_t N, typename T>
	constexpr void setIdentity( MatrixStorage<M, N, T>& r )
	{
		compile_time::setIdentity( r );
	}

	template<size_t N, typename T>
	constexpr T determinant( const MatrixStorage<N, N, T>& a )
	{
		return compile_time::determinant( a );
	}

	template<size_t M, size_t N, typename T>
	constexpr void transpose( MatrixStorage<N, M, T>& r, const MatrixStorage<M, N, T>& a )
	{
		compile_time::transpose( r, a );
	}

	template<size_t N, typename T>
	constexpr void invert( MatrixStorage<N, N, T>& r, const MatrixStorage<N, N, T>& a )
	{
		compile_time::invert( r, a );
	}

	template<size_t M, size_t N, typename T>
	constexpr void add( MatrixStorage<N, M, T>& r, const MatrixStorage<M, N, T>& a, const MatrixStorage<M, N, T>& b )
	{
		for( size_t col = 0LLU; col < N; ++col )
		{
			add<M, T>( r.cols[col], a.cols[col], b.cols[col] );
		}
	}

	template<size_t M, size_t N, typename T>
	constexpr void sub( MatrixStorage<N, M, T>& r, const MatrixStorage<M, N, T>& a, const MatrixStorage<M, N, T>& b )
	{
		for( size_t col = 0LLU; col < N; ++col )
		{
			sub<M, T>( r.cols[col], a.cols[col], b.cols[col] );
		}
	}

	template<size_t M, size_t N, typename T>
	constexpr void add( MatrixStorage<N, M, T>& r, const MatrixStorage<M, N, T>& a, T b )
	{
		VectorStorage<M, T> s;
		set1<M, T>( s, b );
		for( size_t col = 0LLU; col < N; ++col )
		{
			add<M, T>( r.cols[col], a.cols[col], s );
		}
	}

	template<size_t M, size_t N, typename T>
	constexpr void sub( MatrixStorage<N, M, T>& r, const MatrixStorage<M, N, T>& a, T b )
	{
		VectorStorage<M, T> s;
		set1<M, T>( s, b );
		for( size_t col = 0LLU; col < N; ++col )
		{
			sub<M, T>( r.cols[col], a.cols[col], s );
		}
	}

	template<size_t M, size_t N, typename T>
	constexpr void mul( MatrixStorage<N, M, T>& r, const MatrixStorage<M, N, T>& a, T b )
	{
		VectorStorage<M, T> s;
		set1<M, T>( s, b );
		for( size_t col = 0LLU; col < N; ++col )
		{
			mul<M, T>( r.cols[col], a.cols[col], s );
		}
	}

	template<size_t M, size_t N, typename T>
	constexpr void div( MatrixStorage<N, M, T>& r, const MatrixStorage<M, N, T>& a, T b )
	{
		VectorStorage<M, T> s;
		set1<M, T>( s, b );
		for( size_t col = 0LLU; col < N; ++col )
		{
			div<M, T>( r.cols[col], a.cols[col], s );
		}
	}

	template<size_t N0, size_t N1, size_t N2, typename T>
	constexpr void mul( MatrixStorage<N0, N2, T>& r, const MatrixStorage<N0, N1, T>& a, const MatrixStorage<N1, N2, T>& b )
	{
		compile_time::mul<N0, N1, N2, T>( r, a, b );
	}

	template<size_t M, size_t N, typename T>
	constexpr void mul( VectorStorage<M, T>& r, const MatrixStorage<M, N, T>& a, const VectorStorage<M, T>& b )
	{
		compile_time::mul<M, N, T>( r, a, b );
	}

	template<size_t M, size_t N, typename T>
	constexpr void mul( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const MatrixStorage<M, N, T>& b )
	{
		compile_time::mul<M, N, T>( r, a, b );
	}

	// SIMD 256-bit floating-point Operations
	typedef MatrixStorage<2, 2, f32> simdMat2f32;
	typedef MatrixStorage<4, 4, f32> simdMat4f32;

#if ENABLE_INSTRUCTIONS_AVX2

	/**
	 * @brief Performs cofactor expansion on rows OR columns.
	 * This effectively calculates the determinant of four 3x3 matrices.
	 * @param v1 - row or column A
	 * @param v2 - row or column B
	 * @param v3 - row or column C
	 * @return Cofactor expanded row/column
	*/
	[[nodiscard]] static __m128 cofactorExpansion_SSE2( const __m128& v1, const __m128& v2, const __m128& v3 )
	{
		// Mask out the last element
		constexpr static __m128 msk = { 1.0f, -1.0f, 1.0f, 0.0f };

		// Row breakdown and parallel dot product
		return _mm_dot_ps(
			_mm_mul_ps( msk, _mm_permute_ps( v1, swizzle::YZW ) ),
			_mm_sub_ps( _mm_mul_ps( _mm_permute_ps( v2, swizzle::ZYY ), _mm_permute_ps( v3, swizzle::WWZ ) ), _mm_mul_ps( _mm_permute_ps( v3, swizzle::ZYY ), _mm_permute_ps( v2, swizzle::WWZ ) ) ),
			_mm_mul_ps( msk, _mm_permute_ps( v1, swizzle::XZW ) ),
			_mm_sub_ps( _mm_mul_ps( _mm_permute_ps( v2, swizzle::ZXX ), _mm_permute_ps( v3, swizzle::WWZ ) ), _mm_mul_ps( _mm_permute_ps( v3, swizzle::ZXX ), _mm_permute_ps( v2, swizzle::WWZ ) ) ),
			_mm_mul_ps( msk, _mm_permute_ps( v1, swizzle::XYW ) ),
			_mm_sub_ps( _mm_mul_ps( _mm_permute_ps( v2, swizzle::YXX ), _mm_permute_ps( v3, swizzle::WWY ) ), _mm_mul_ps( _mm_permute_ps( v3, swizzle::YXX ), _mm_permute_ps( v2, swizzle::WWY ) ) ),
			_mm_mul_ps( msk, v1 ),
			_mm_sub_ps( _mm_mul_ps( _mm_permute_ps( v2, swizzle::YXX ), _mm_permute_ps( v3, swizzle::ZZY ) ), _mm_mul_ps( _mm_permute_ps( v3, swizzle::YXX ), _mm_permute_ps( v2, swizzle::ZZY ) ) ) );
	}

	/**
	 * @brief Performs cofactor expansion on rows OR columns.
	 * This effectively calculates the determinant of four 3x3 matrices.
	 * @param v1 - row or column A
	 * @param v2 - row or column B
	 * @param v3 - row or column C
	 * @return Cofactor expanded row/column
	*/
	[[nodiscard]] static __m256 cofactorExpansion_AVX2( const __m256& v1, const __m256& v2, const __m256& v3 )
	{
		// Mask out the last element
		constexpr static __m256 msk = { 1.0f, -1.0f, 1.0f, 0.0f, 1.0f, -1.0f, 1.0f, 0.0f };

		// Row breakdown and parallel dot product
		return _mm256_dot_ps(
			_mm256_mul_ps( msk, _mm256_permute_ps( v1, swizzle::YZW ) ),
			_mm256_sub_ps( _mm256_mul_ps( _mm256_permute_ps( v2, swizzle::ZYY ), _mm256_permute_ps( v3, swizzle::WWZ ) ), _mm256_mul_ps( _mm256_permute_ps( v3, swizzle::ZYY ), _mm256_permute_ps( v2, swizzle::WWZ ) ) ),
			_mm256_mul_ps( msk, _mm256_permute_ps( v1, swizzle::XZW ) ),
			_mm256_sub_ps( _mm256_mul_ps( _mm256_permute_ps( v2, swizzle::ZXX ), _mm256_permute_ps( v3, swizzle::WWZ ) ), _mm256_mul_ps( _mm256_permute_ps( v3, swizzle::ZXX ), _mm256_permute_ps( v2, swizzle::WWZ ) ) ),
			_mm256_mul_ps( msk, _mm256_permute_ps( v1, swizzle::XYW ) ),
			_mm256_sub_ps( _mm256_mul_ps( _mm256_permute_ps( v2, swizzle::YXX ), _mm256_permute_ps( v3, swizzle::WWY ) ), _mm256_mul_ps( _mm256_permute_ps( v3, swizzle::YXX ), _mm256_permute_ps( v2, swizzle::WWY ) ) ),
			_mm256_mul_ps( msk, v1 ),
			_mm256_sub_ps( _mm256_mul_ps( _mm256_permute_ps( v2, swizzle::YXX ), _mm256_permute_ps( v3, swizzle::ZZY ) ), _mm256_mul_ps( _mm256_permute_ps( v3, swizzle::YXX ), _mm256_permute_ps( v2, swizzle::ZZY ) ) ) );
	}

	template<>
	constexpr f32 determinant( const simdMat4f32& a )
	{
		// TODO: NOT IMPLEMENTED
		if( std::is_constant_evaluated() ) return NAN;

		constexpr static __m128 msk = { 1.0f, -1.0f, 1.0f, -1.0f };

		// Dot product between simdumn and its cofactor expansion
		simd4f32 r;
		r.simd = _mm_mul_ps( msk, cofactorExpansion_SSE2( a.simd[1], a.simd[2], a.simd[3] ) );
		return dot( a.cols[0], r );
	}

	template<>
	constexpr void transpose( simdMat4f32& r, const simdMat4f32& a )
	{
		if( std::is_constant_evaluated() ) return compile_time::transpose( r, a );

		__m256i permutation = _mm256_set_epi32( 7, 3, 5, 1, 6, 2, 4, 0 );
		__m256 a0 = _mm256_permutevar8x32_ps( a.avx2[0], permutation );
		__m256 a1 = _mm256_permutevar8x32_ps( a.avx2[1], permutation );

		r.avx2[0] = _mm256_shuffle_ps( a0, _mm256_permute_ps( a1, swizzle::ZWXY ), swizzle::XYZW );
		r.avx2[1] = _mm256_shuffle_ps( _mm256_permute_ps( a0, swizzle::ZWXY ), a1, swizzle::XYZW );
	}

	template<>
	constexpr void invert( simdMat4f32& r, const simdMat4f32& a )
	{
		// TODO: NOT IMPLEMENTED
		if( std::is_constant_evaluated() ) return compile_time::invert( r, a );

		constexpr static __m256 msk = { 1.0f, -1.0f, 1.0f, -1.0f, -1.0f, 1.0f, -1.0f, 1.0f };

		// Calculate the cofactor expansion for every column
		__m256 adj0_1 = _mm256_mul_ps( msk, cofactorExpansion_AVX2( _mm256_set_m128( a.simd[0], a.simd[1] ), _mm256_set_m128( a.simd[2], a.simd[2] ), _mm256_set_m128( a.simd[3], a.simd[3] ) ) );
		__m256 adj2_3 = _mm256_mul_ps( msk, cofactorExpansion_AVX2( _mm256_set_m128( a.simd[0], a.simd[0] ), _mm256_set_m128( a.simd[1], a.simd[1] ), _mm256_set_m128( a.simd[2], a.simd[3] ) ) );

		__m256 product = _mm256_mul_ps( a.avx2[0], adj0_1 );
		__m256 det = _mm256_hadd_ps( product, product );
		det = _mm256_hadd_ps( det, det );

		adj0_1 = _mm256_div_ps( adj0_1, det );
		adj2_3 = _mm256_div_ps( adj2_3, det );

		// Transpose the matrix
		__m256i permutation = _mm256_set_epi32( 7, 3, 5, 1, 6, 2, 4, 0 );
		__m256 a0 = _mm256_permutevar8x32_ps( adj0_1, permutation );
		__m256 a1 = _mm256_permutevar8x32_ps( adj2_3, permutation );

		r.avx2[0] = _mm256_shuffle_ps( a0, _mm256_permute_ps( a1, swizzle::ZWXY ), swizzle::XYZW );
		r.avx2[1] = _mm256_shuffle_ps( _mm256_permute_ps( a0, swizzle::ZWXY ), a1, swizzle::XYZW );
	}

	template<>
	constexpr void mul( simdMat4f32& r, const simdMat4f32& a, const simdMat4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::mul( r, a, b );
		
		__m256 r0 = { a.m[0], a.m[4], a.m[8], a.m[12], a.m[0], a.m[4], a.m[8], a.m[12] };
		__m256 r1 = { a.m[1], a.m[5], a.m[9], a.m[13], a.m[1], a.m[5], a.m[9], a.m[13] };
		__m256 r2 = { a.m[2], a.m[6], a.m[10], a.m[14], a.m[2], a.m[6], a.m[10], a.m[14] };
		__m256 r3 = { a.m[3], a.m[7], a.m[11], a.m[15], a.m[3], a.m[7], a.m[11], a.m[15] };
		
		r.avx2[0] = _mm256_dot_ps( b.avx2[0], r0, b.avx2[0], r1, b.avx2[0], r2, b.avx2[0], r3 );
		r.avx2[1] = _mm256_dot_ps( b.avx2[1], r0, b.avx2[1], r1, b.avx2[1], r2, b.avx2[1], r3 );
	}

#elif ENABLE_INSTRUCTIONS_SSE2

	template<>
	constexpr void transpose( simdMat4f32& r, const simdMat4f32& a )
	{
		if( std::is_constant_evaluated() ) return compile_time::transpose( r, a );

		r.simd[0] = _mm_transpose_ps( simd[0], simd[1], simd[2], simd[3], swizzle::XXXX );
		r.simd[1] = _mm_transpose_ps( simd[0], simd[1], simd[2], simd[3], swizzle::YYYY );
		r.simd[2] = _mm_transpose_ps( simd[0], simd[1], simd[2], simd[3], swizzle::ZZZZ );
		r.simd[3] = _mm_transpose_ps( simd[0], simd[1], simd[2], simd[3], swizzle::WWWW );
	}

	template<>
	constexpr void invert( simdMat4f32& r, const simdMat4f32& a )
	{
		if( std::is_constant_evaluated() ) return compile_time::invert( r, a );

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
		r.simd[0] = _mm_transpose_ps( adj0, adj1, adj2, adj3, swizzle::YYYY );
		r.simd[1] = _mm_transpose_ps( adj0, adj1, adj2, adj3, swizzle::ZZZZ );
		r.simd[2] = _mm_transpose_ps( adj0, adj1, adj2, adj3, swizzle::WWWW );
		r.simd[3] = _mm_transpose_ps( adj0, adj1, adj2, adj3, swizzle::XXXX );
	}

	template<size_t N0, size_t N1, size_t N2, typename T>
	constexpr void mul( simdMat4f32& r, const simdMat4f32& a, const simdMat4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::mul( r, a, b );

		__m128 r0 = _mm_xyzw_ps( a.m[0], a.m[4], a.m[8], a.m[12] );
		__m128 r1 = _mm_xyzw_ps( a.m[1], a.m[5], a.m[9], a.m[13] );
		__m128 r2 = _mm_xyzw_ps( a.m[2], a.m[6], a.m[10], a.m[14] );
		__m128 r3 = _mm_xyzw_ps( a.m[3], a.m[7], a.m[11], a.m[15] );
		
		r.simd[0] = _mm_dot_ps( b.simd[0], r0, b.simd[0], r1, b.simd[0], r2, b.simd[0], r3 );
		r.simd[1] = _mm_dot_ps( b.simd[1], r0, b.simd[1], r1, b.simd[1], r2, b.simd[1], r3 );
		r.simd[2] = _mm_dot_ps( b.simd[2], r0, b.simd[2], r1, b.simd[2], r2, b.simd[2], r3 );
		r.simd[3] = _mm_dot_ps( b.simd[3], r0, b.simd[3], r1, b.simd[3], r2, b.simd[3], r3 );
	}

#endif

	// SIMD 128-bit floating-point Operations

#if ENABLE_INSTRUCTIONS_SSE2 || ENABLE_INSTRUCTIONS_AVX2

	template<>
	constexpr void mul( simd4f32& r, const simdMat4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::mul( r, a, b );

		__m128 t0 = _mm_shuffle_ps( a.x_axis.simd, a.y_axis.simd, 0x44 );
		__m128 t2 = _mm_shuffle_ps( a.x_axis.simd, a.y_axis.simd, 0xEE );
		__m128 t1 = _mm_shuffle_ps( a.z_axis.simd, a.origin.simd, 0x44 );
		__m128 t3 = _mm_shuffle_ps( a.z_axis.simd, a.origin.simd, 0xEE );

		r.simd = _mm_dot_ps(
			_mm_shuffle_ps( t0, t1, 0x88 ), b.simd,
			_mm_shuffle_ps( t0, t1, 0xDD ), b.simd,
			_mm_shuffle_ps( t2, t3, 0x88 ), b.simd,
			_mm_shuffle_ps( t2, t3, 0xDD ), b.simd );
	}

	template<>
	constexpr void mul( simd4f32& r, const simd4f32& a, const simdMat4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::mul( r, a, b );

		r.simd = _mm_dot_ps( a.simd, b.simd[0], a.simd, b.simd[1], a.simd, b.simd[2], a.simd, b.simd[3] );
	}

	template<>
	constexpr void mul( simdMat2f32& r, const simdMat2f32& a, const simdMat2f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::mul( r, a, b );
		r.simd = _mm_add_ps( 
			_mm_mul_ps( _mm_permute_ps( a.simd, swizzle::XYXY ), _mm_permute_ps( b.simd, swizzle::XXZZ ) ),
			_mm_mul_ps( _mm_permute_ps( a.simd, swizzle::ZWZW ), _mm_permute_ps( b.simd, swizzle::YYWW ) )
		);
	}

#endif

}

#endif