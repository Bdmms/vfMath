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

#elif ENABLE_INSTRUCTIONS_SSE2

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