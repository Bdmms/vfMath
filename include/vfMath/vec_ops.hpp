#pragma once
#ifndef VF_VECTOR_OPERATIONS_HPP
#define VF_VECTOR_OPERATIONS_HPP

#include "vec_store.hpp"

#define VF_VEC_N_LOOP( N, X ) for( size_t i = 0LLU; i < N; ++i ) X
#define VF_VEC_N_UNARY_OPERATION( N, r, a, Op ) VF_VEC_N_LOOP( N, r[i] = Op a[i] )
#define VF_VEC_N_UNARY_FUNCTION( N, r, a, Op ) VF_VEC_N_LOOP( N, r[i] = Op( a[i] ) )
#define VF_VEC_N_BINARY_OPERATION( N, r, a, b, Op ) VF_VEC_N_LOOP( N, r[i] = a[i] Op b[i] )
#define VF_VEC_N_BINARY_FUNCTION( N, r, a, b, Op ) VF_VEC_N_LOOP( N, r[i] = Op( a[i], b[i] ) )

//#define VF_VEC_BINARY_OPERATOR_ADAPTER( Name ) template<size_t N, typename T> \
//constexpr VectorStorage<N, T> Name( const VectorStorage<N, T>&a, const VectorStorage<N, T>&b ) \
//	{ \
//		VectorStorage<N, T> r; \
//		Name<N, T>( r, a, b ); \
//		return r; \
//	} \

namespace vf::simd
{
	namespace compile_time
	{
		template<size_t N, typename T>
		constexpr VectorStorage<N, T> set1( T value )
		{
			VectorStorage<N, T> r;
			for( size_t i = 0LLU; i < N; ++i )
			{
				r.elements[i] = value;
			}
			return r;
		}

		template<size_t N, typename T>
		constexpr VectorStorage<N, T> negate( VectorStorage<N, T>& r, const VectorStorage<N, T>& a )
		{
			VF_VEC_N_UNARY_OPERATION( N, r.elements, a.elements, - );
		}

		template<size_t N, typename T>
		constexpr void add( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_BINARY_OPERATION( N, r.elements, a.elements, b.elements, + );
		}

		template<size_t N, typename T>
		constexpr void sub( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_BINARY_OPERATION( N, r.elements, a.elements, b.elements, - );
		}

		template<size_t N, typename T>
		constexpr void mul( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_BINARY_OPERATION( N, r.elements, a.elements, b.elements, * );
		}

		template<size_t N, typename T>
		constexpr void div( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_BINARY_OPERATION( N, r.elements, a.elements, b.elements, / );
		}

		template<size_t N, typename T>
		constexpr void mod( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_BINARY_FUNCTION( N, r.elements, a.elements, b.elements, vf::primitives::modulo );
		}

		template<size_t N, typename T>
		constexpr void bitwise_and( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_BINARY_FUNCTION( N, r.elements, a.elements, b.elements, vf::primitives::bitwise_and );
		}

		template<size_t N, typename T>
		constexpr void bitwise_or( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_BINARY_FUNCTION( N, r.elements, a.elements, b.elements, vf::primitives::bitwise_or );
		}

		template<size_t N, typename T>
		constexpr void bitwise_xor( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_BINARY_FUNCTION( N, r.elements, a.elements, b.elements, vf::primitives::bitwise_xor );
		}

		template<size_t N, typename T>
		constexpr void bitwise_not( VectorStorage<N, T>& r, const VectorStorage<N, T>& a )
		{
			VF_VEC_N_UNARY_FUNCTION( N, r.elements, a.elements, vf::primitives::bitwise_not );
		}

		template<size_t N, typename T>
		constexpr void cmpge( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_LOOP( N, r.elements[i] = a.elements[i] >= b.elements[i] ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T> );
		}

		template<size_t N, typename T>
		constexpr void cmple( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_LOOP( N, r.elements[i] = a.elements[i] <= b.elements[i] ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T> );
		}

		template<size_t N, typename T>
		constexpr void cmpgt( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_LOOP( N, r.elements[i] = a.elements[i] > b.elements[i] ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T> );
		}

		template<size_t N, typename T>
		constexpr void cmplt( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_LOOP( N, r.elements[i] = a.elements[i] < b.elements[i] ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T> );
		}
	}

	template<size_t N, typename T>
	using BinaryVecOp = void (*)( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b );

	// Default Operations

	template<size_t N, typename T>
	constexpr VectorStorage<N, T> set1( T value )
	{
		return compile_time::set1<N, T>( value );
	}

	template<size_t N, typename T>
	constexpr void negate( VectorStorage<N, T>& r, const VectorStorage<N, T>& a )
	{
		compile_time::negate( r, a );
	}

	template<size_t N, typename T>
	constexpr void add( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::add( r, a, b );
	}

	template<size_t N, typename T>
	constexpr void sub( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::sub( r, a, b );
	}

	template<size_t N, typename T>
	constexpr void mul( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::mul( r, a, b );
	}

	template<size_t N, typename T>
	constexpr void div( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::div( r, a, b );
	}

	template<size_t N, typename T>
	constexpr void mod( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::mod( r, a, b );
	}

	template<size_t N, typename T>
	constexpr VectorStorage<N, T> bitwise_not( VectorStorage<N, T>& r, const VectorStorage<N, T>& a )
	{
		compile_time::bitwise_not( r, a );
	}

	template<size_t N, typename T>
	constexpr void bitwise_and( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::bitwise_and( r, a, b );
	}

	template<size_t N, typename T>
	constexpr void bitwise_or( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::bitwise_or( r, a, b );
	}

	template<size_t N, typename T>
	constexpr void bitwise_xor( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::bitwise_xor( r, a, b );
	}

	template<size_t N, typename T>
	constexpr void cmpge( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::cmpge( r, a, b );
	}

	template<size_t N, typename T>
	constexpr void cmple( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::cmple( r, a, b );
	}

	template<size_t N, typename T>
	constexpr void cmpgt( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::cmpgt( r, a, b );
	}

	template<size_t N, typename T>
	constexpr void cmplt( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::cmplt( r, a, b );
	}

	// Optimized Operations

	typedef VectorStorage<4, f32> simd4f32;

	template<>
	constexpr simd4f32 set1( f32 value )
	{
		if( std::is_constant_evaluated() ) return compile_time::set1<4, f32>( value );
		return _mm_set1_ps( value );
	}

	template<>
	constexpr void add( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::add( r, a, b );
		r.simd = _mm_add_ps( a.simd, b.simd );
	}

	template<>
	constexpr void sub( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::sub( r, a, b );
		r.simd = _mm_sub_ps( a.simd, b.simd );
	}

	template<>
	constexpr void mul( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::mul( r, a, b );
		r.simd = _mm_mul_ps( a.simd, b.simd );
	}

	template<>
	constexpr void div( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::div( r, a, b );
		r.simd = _mm_div_ps( a.simd, b.simd );
	}

	template<>
	constexpr void mod( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::mod( r, a, b );
		r.simd = _mm_fmod_ps( a.simd, b.simd );
	}

	template<>
	constexpr void bitwise_and( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::bitwise_and( r, a, b );
		r.simd = _mm_and_ps( a.simd, b.simd );
	}

	template<>
	constexpr void bitwise_or( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::bitwise_or( r, a, b );
		r.simd = _mm_or_ps( a.simd, b.simd );
	}

	template<>
	constexpr void bitwise_xor( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::bitwise_xor( r, a, b );
		r.simd = _mm_xor_ps( a.simd, b.simd );
	}

	template<>
	constexpr void cmpge( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::cmpge( r, a, b );
		r.simd = _mm_cmpge_ps( a.simd, b.simd );
	}

	template<>
	constexpr void cmple( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::cmple( r, a, b );
		r.simd = _mm_cmple_ps( a.simd, b.simd );
	}

	template<>
	constexpr void cmplt( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::cmplt( r, a, b );
		r.simd = _mm_cmplt_ps( a.simd, b.simd );
	}

	template<>
	constexpr void cmpgt( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::cmpgt( r, a, b );
		r.simd = _mm_cmpgt_ps( a.simd, b.simd );
	}
}

#endif