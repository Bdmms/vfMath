#pragma once
#ifndef VF_VECTOR_OPERATIONS_HPP
#define VF_VECTOR_OPERATIONS_HPP

#include "vec_store.hpp"
#include <cmath>

extern "C" int __isa_available;

#define ENABLE_INSTRUCTIONS_SSE2	__isa_available >= __ISA_AVAILABLE_SSE2
#define ENABLE_INSTRUCTIONS_AVX2	__isa_available >= __ISA_AVAILABLE_AVX2
#define ENABLE_INSTRUCTIONS_AVX512	false

// Macros to help eliminate duplicate code
#define VF_VEC_N_LOOP( N, X ) for( size_t i = 0LLU; i < N; ++i ) X
#define VF_VEC_N_UNARY_OPERATION( N, r, a, Op ) VF_VEC_N_LOOP( N, r[i] = Op a[i] )
#define VF_VEC_N_UNARY_FUNCTION( N, r, a, Op ) VF_VEC_N_LOOP( N, r[i] = Op( a[i] ) )
#define VF_VEC_N_BINARY_OPERATION( N, r, a, b, Op ) VF_VEC_N_LOOP( N, r[i] = a[i] Op b[i] )
#define VF_VEC_N_BINARY_FUNCTION( N, r, a, b, Op ) VF_VEC_N_LOOP( N, r[i] = Op( a[i], b[i] ) )

namespace vf::simd
{
	namespace compile_time
	{
		template<size_t N, typename T>
		constexpr VectorStorage<N, T>& set1( VectorStorage<N, T>& r, T value )
		{
			for( size_t i = 0LLU; i < N; ++i )
			{
				r.elements[i] = value;
			}
			return r;
		}

		template<size_t N, typename T>
		constexpr void negate( VectorStorage<N, T>& r, const VectorStorage<N, T>& a )
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

		template<typename T>
		constexpr void hadd( VectorStorage<4, T>& r, const VectorStorage<4, T>& a, const VectorStorage<4, T>& b )
		{
			r.elements[0] = a.elements[0] + a.elements[1];
			r.elements[1] = a.elements[2] + a.elements[3];
			r.elements[2] = b.elements[0] + b.elements[1];
			r.elements[3] = b.elements[2] + b.elements[3];
		}

		template<size_t N, typename T>
		constexpr void fmadd( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b, const VectorStorage<N, T>& c )
		{
			mul( r, a, b );
			add( r, r, c );
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

		template<size_t N, typename T>
		constexpr void min( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_BINARY_FUNCTION( N, r.elements, a.elements, b.elements, std::min );
		}

		template<size_t N, typename T>
		constexpr void max( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_BINARY_FUNCTION( N, r.elements, a.elements, b.elements, std::max );
		}

		template<size_t N, typename T>
		constexpr void floor( VectorStorage<N, T>& r, const VectorStorage<N, T>& a )
		{
			VF_VEC_N_UNARY_FUNCTION( N, r.elements, a.elements, floorf );
		}

		template<size_t N, typename T>
		constexpr void ceil( VectorStorage<N, T>& r, const VectorStorage<N, T>& a )
		{
			VF_VEC_N_UNARY_FUNCTION( N, r.elements, a.elements, ceilf );
		}

		template<size_t N, typename T>
		constexpr void round( VectorStorage<N, T>& r, const VectorStorage<N, T>& a )
		{
			VF_VEC_N_UNARY_FUNCTION( N, r.elements, a.elements, roundf );
		}

		template<typename T>
		constexpr void permute( VectorStorage<4, T>& r, const VectorStorage<4, T>& a, int b )
		{
			r.elements[0] = a.elements[( b & 0x03 )];
			r.elements[1] = a.elements[( b & 0x0C ) >> 2];
			r.elements[2] = a.elements[( b & 0x30 ) >> 4];
			r.elements[3] = a.elements[( b & 0xC0 ) >> 6];
		}

		template<size_t N, typename T>
		[[nodiscard]] constexpr T dot( const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			T r{};
			for( size_t i = 0LLU; i < N; ++i )
			{
				r += a.elements[i] * b.elements[i];
			}
			return r;
		}
	}

	template<size_t N, typename T>
	using BinaryVecOp = void (*)( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b );

	// Default Operations

	template<size_t N, typename T>
	constexpr VectorStorage<N, T>& set1( VectorStorage<N, T>& r, T value )
	{
		return compile_time::set1<N, T>( r, value );
	}

	template<size_t N, typename T>
	[[nodiscard]] constexpr T dot( const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		return compile_time::dot( a, b );
	}

	template<typename T>
	[[nodiscard]] constexpr T dot( const VectorStorage<2, T>& a, const VectorStorage<2, T>& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::dot( a, b );
		return a.x * b.x + a.y * b.y;
	}

	template<typename T>
	[[nodiscard]] constexpr T dot( const VectorStorage<3, T>& a, const VectorStorage<3, T>& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::dot( a, b );
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	template<typename T>
	[[nodiscard]] constexpr T dot( const VectorStorage<4, T>& a, const VectorStorage<4, T>& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::dot( a, b );
		return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
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

	template<typename T>
	constexpr void hadd( VectorStorage<4, T>& r, const VectorStorage<4, T>& a, const VectorStorage<4, T>& b )
	{
		compile_time::hadd( r, a, b );
	}

	template<size_t N, typename T>
	constexpr void fmadd( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b, const VectorStorage<N, T>& c )
	{
		compile_time::fmadd( r, a, b, c );
	}

	template<size_t N, typename T>
	constexpr void bitwise_not( VectorStorage<N, T>& r, const VectorStorage<N, T>& a )
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

	template<size_t N, typename T>
	constexpr void min( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::min( r, a, b );
	}

	template<size_t N, typename T>
	constexpr void max( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::max( r, a, b );
	}

	template<size_t N, typename T>
	constexpr void floor( VectorStorage<N, T>& r, const VectorStorage<N, T>& a )
	{
		compile_time::floor( r, a );
	}

	template<size_t N, typename T>
	constexpr void ceil( VectorStorage<N, T>& r, const VectorStorage<N, T>& a )
	{
		compile_time::ceil( r, a );
	}

	template<size_t N, typename T>
	constexpr void round( VectorStorage<N, T>& r, const VectorStorage<N, T>& a )
	{
		compile_time::round( r, a );
	}

	template<size_t N, typename T>
	static void sqrt( VectorStorage<N, T>& r, const VectorStorage<N, T>& a )
	{
		VF_VEC_N_UNARY_FUNCTION( N, r.elements, a.elements, sqrtf );
	}

	template<typename T>
	constexpr VectorStorage<4, T> permute( const VectorStorage<4, T>& a, int b )
	{
		VectorStorage<4, T> r;
		compile_time::permute( r, a, b );
		return r;
	}

	template<size_t N, typename T>
	constexpr bool any( const VectorStorage<N, T>& a )
	{
		for( size_t i = 0LLU; i < N; ++i )
		{
			if( a.elements[i] ) return true;
		}
		return false;
	}

	template<size_t N, typename T>
	constexpr bool all( const VectorStorage<N, T>& a )
	{
		for( size_t i = 0LLU; i < N; ++i )
		{
			if( !a.elements[i] ) return false;
		}
		return true;
	}

	// SIMD 128-bit floating-point Operations

#if ENABLE_INSTRUCTIONS_SSE2
	typedef VectorStorage<4, f32> simd4f32;

	template<>
	constexpr simd4f32& set1( simd4f32& r, f32 value )
	{
		if( std::is_constant_evaluated() ) return compile_time::set1<4, f32>( r, value );
		r.simd = _mm_set1_ps( value );
		return r;
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
	constexpr void hadd( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::hadd( r, a, b );
		r.simd = _mm_hadd_ps( a.simd, b.simd );
	}

	template<>
	constexpr void fmadd( simd4f32& r, const simd4f32& a, const simd4f32& b, const simd4f32& c )
	{
		if( std::is_constant_evaluated() ) return compile_time::fmadd( r, a, b, c );
		r.simd = _mm_fmadd_ps( a.simd, b.simd, c.simd );
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

	template<>
	constexpr void min( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::min( r, a, b );
		r.simd = _mm_min_ps( a.simd, b.simd );
	}

	template<>
	constexpr void max( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::max( r, a, b );
		r.simd = _mm_max_ps( a.simd, b.simd );
	}

	template<>
	constexpr void floor( simd4f32& r, const simd4f32& a )
	{
		if( std::is_constant_evaluated() ) return compile_time::floor( r, a );
		r.simd = _mm_floor_ps( a.simd );
	}

	template<>
	constexpr void ceil( simd4f32& r, const simd4f32& a )
	{
		if( std::is_constant_evaluated() ) return compile_time::ceil( r, a );
		r.simd = _mm_ceil_ps( a.simd );
	}

	template<>
	constexpr void round( simd4f32& r, const simd4f32& a )
	{
		if( std::is_constant_evaluated() ) return compile_time::round( r, a );
		r.simd = _mm_round_ps( a.simd, 0b00 );
	}

	template<>
	static void sqrt( simd4f32& r, const simd4f32& a )
	{
		r.simd = _mm_sqrt_ps( a.simd );
	}

	template<typename T>
	constexpr simd4f32 permute( const simd4f32& a, int b )
	{
		simd4f32 r;
		if( std::is_constant_evaluated() ) return compile_time::permute( r, a, b );
		r.simd = _mm_permute_ps( a.simd, b );
		return r;
	}
#endif

	// SIMD 128-bit Integer Operations

#if ENABLE_INSTRUCTIONS_SSE2
	typedef VectorStorage<4, i32> simd4i32;

	template<>
	constexpr simd4i32& set1( simd4i32& r, i32 value )
	{
		if( std::is_constant_evaluated() ) return compile_time::set1<4, i32>( r, value );
		r.simd = _mm_set1_epi32( value );
		return r;
	}

	template<>
	constexpr void add( simd4i32& r, const simd4i32& a, const simd4i32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::add( r, a, b );
		r.simd = _mm_add_epi32( a.simd, b.simd );
	}

	template<>
	constexpr void sub( simd4i32& r, const simd4i32& a, const simd4i32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::sub( r, a, b );
		r.simd = _mm_sub_epi32( a.simd, b.simd );
	}

	template<>
	constexpr void mul( simd4i32& r, const simd4i32& a, const simd4i32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::mul( r, a, b );
		r.simd = _mm_mul_epi32( a.simd, b.simd );
	}

	template<>
	constexpr void div( simd4i32& r, const simd4i32& a, const simd4i32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::div( r, a, b );
		r.simd = _mm_div_epi32( a.simd, b.simd );
	}

	template<>
	constexpr void mod( simd4i32& r, const simd4i32& a, const simd4i32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::mod( r, a, b );
		_mm_idivrem_epi32( &r.simd, a.simd, b.simd );
	}

	template<>
	constexpr void hadd( simd4i32& r, const simd4i32& a, const simd4i32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::hadd( r, a, b );
		r.simd = _mm_hadd_epi32( a.simd, b.simd );
	}

	template<>
	constexpr void fmadd( simd4i32& r, const simd4i32& a, const simd4i32& b, const simd4i32& c )
	{
		if( std::is_constant_evaluated() ) return compile_time::fmadd( r, a, b, c );
		r.simd = _mm_add_epi32( _mm_mul_epi32( a.simd, b.simd ), c.simd );
	}

	template<>
	constexpr void bitwise_and( simd4i32& r, const simd4i32& a, const simd4i32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::bitwise_and( r, a, b );
		r.simd = _mm_and_epi32( a.simd, b.simd );
	}

	template<>
	constexpr void bitwise_or( simd4i32& r, const simd4i32& a, const simd4i32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::bitwise_or( r, a, b );
		r.simd = _mm_or_epi32( a.simd, b.simd );
	}

	template<>
	constexpr void bitwise_xor( simd4i32& r, const simd4i32& a, const simd4i32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::bitwise_xor( r, a, b );
		r.simd = _mm_xor_epi32( a.simd, b.simd );
	}

	//template<>
	//constexpr void cmpge( simd4i32& r, const simd4i32& a, const simd4i32& b )
	//{
	//	if( std::is_constant_evaluated() ) return compile_time::cmpge( r, a, b );
	//	r.simd = _mm_cmpge_epi32_mask( a.simd, b.simd );
	//}

	//template<>
	//constexpr void cmple( simd4i32& r, const simd4i32& a, const simd4i32& b )
	//{
	//	if( std::is_constant_evaluated() ) return compile_time::cmple( r, a, b );
	//	r.simd = _mm_cmple_epi32_mask( a.simd, b.simd );
	//}

	//template<>
	//constexpr void cmplt( simd4i32& r, const simd4i32& a, const simd4i32& b )
	//{
	//	if( std::is_constant_evaluated() ) return compile_time::cmplt( r, a, b );
	//	r.simd = _mm_cmplt_epi32_mask( a.simd, b.simd );
	//}

	//template<>
	//constexpr void cmpgt( simd4i32& r, const simd4i32& a, const simd4i32& b )
	//{
	//	if( std::is_constant_evaluated() ) return compile_time::cmpgt( r, a, b );
	//	r.simd = _mm_cmpgt_epi32_mask( a.simd, b.simd );
	//}

	template<>
	constexpr void min( simd4i32& r, const simd4i32& a, const simd4i32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::min( r, a, b );
		r.simd = _mm_min_epi32( a.simd, b.simd );
	}

	template<>
	constexpr void max( simd4i32& r, const simd4i32& a, const simd4i32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::max( r, a, b );
		r.simd = _mm_max_epi32( a.simd, b.simd );
	}
#endif
}

#endif