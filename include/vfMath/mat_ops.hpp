#pragma once
#ifndef VF_MATRIX_OPERATIONS_HPP
#define VF_MATRIX_OPERATIONS_HPP

#include "vec_ops.hpp"
#include "mat_store.hpp"

namespace vf::simd
{
	namespace compile_time
	{
		template<size_t M, size_t N, typename T>
		constexpr MatrixStorage<M, N, T>& set1( MatrixStorage<M, N, T>& r, T value )
		{
			for( size_t col = 0LLU; col < N; ++col )
			{
				set1( r.cols[col], value );
			}
			return r;
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
			// TODO: THIS IS WRONG
			//T det = determinant( a );
			//r.cols[0].elements[0] = a.cols[1].elements[1] / det;
			//r.cols[0].elements[1] = -a.cols[1].elements[0] / det;
			//r.cols[1].elements[0] = -a.cols[0].elements[1] / det;
			//r.cols[1].elements[1] = a.cols[0].elements[0] / det;

			T det = determinant( a );
			r.cols[0].elements[0] = a.cols[1].elements[1] / det;
			r.cols[0].elements[1] = -a.cols[0].elements[1] / det;
			r.cols[1].elements[0] = -a.cols[1].elements[0] / det;
			r.cols[1].elements[1] = a.cols[0].elements[0] / det;
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
		VectorStorage<M, T> s = set1<M, T>( b );
		for( size_t col = 0LLU; col < N; ++col )
		{
			mul<M, T>( r.cols[col], a.cols[col], s );
		}
	}

	template<size_t M, size_t N, typename T>
	constexpr void div( MatrixStorage<N, M, T>& r, const MatrixStorage<M, N, T>& a, T b )
	{
		VectorStorage<M, T> s = set1<M, T>( b );
		for( size_t col = 0LLU; col < N; ++col )
		{
			div<M, T>( r.cols[col], a.cols[col], s );
		}
	}

	template<size_t N0, size_t N1, size_t N2, typename T>
	constexpr void mul( MatrixStorage<N0, N2, T>& r, const MatrixStorage<N0, N1, T>& a, const MatrixStorage<N1, N2, T>& b )
	{
		compile_time::mul( r, a, b );
	}

	template<size_t M, size_t N, typename T>
	constexpr void mul( VectorStorage<M, T>& r, const MatrixStorage<M, N, T>& a, const VectorStorage<M, T>& b )
	{
		compile_time::mul( r, a, b );
	}

	template<size_t M, size_t N, typename T>
	constexpr void mul( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const MatrixStorage<M, N, T>& b )
	{
		compile_time::mul( r, a, b );
	}

	// SIMD 256-bit floating-point Operations

#if ENABLE_INSTRUCTIONS_AVX2

#elif ENABLE_INSTRUCTIONS_SSE2

#endif

	// SIMD 128-bit floating-point Operations

#if ENABLE_INSTRUCTIONS_SSE2 || ENABLE_INSTRUCTIONS_AVX2

	template<>
	constexpr void mul( MatrixStorage<2, 2, f32>& r, const MatrixStorage<2, 2, f32>& a, const MatrixStorage<2, 2, f32>& b )
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