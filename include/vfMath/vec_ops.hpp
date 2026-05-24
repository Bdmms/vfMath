#pragma once
#ifndef VF_VECTOR_OPERATIONS_HPP
#define VF_VECTOR_OPERATIONS_HPP

#include "Math.hpp"
#include "vec_store.hpp"
#include <cmath>

extern "C" int __isa_available;

#define ENABLE_INSTRUCTIONS_SSE2	__isa_available >= __ISA_AVAILABLE_SSE2
#define ENABLE_INSTRUCTIONS_AVX2	__isa_available >= __ISA_AVAILABLE_AVX2
#define ENABLE_INSTRUCTIONS_AVX512	false

// High-level SSE2 vector instructions
#if ENABLE_INSTRUCTIONS_SSE2

#define _mm_xyzw_epi32(x,y,z,w) _mm_set_epi32(w,z,y,x)

#define _mm_xyzw_ps(x,y,z,w) _mm_set_ps(w,z,y,x)
#define _mm_sum_ps(a,b,c,d) _mm_hadd_ps(_mm_hadd_ps(a,b), _mm_hadd_ps(c,d))
#define _mm_dot_ps(a,b,c,d,e,f,g,h) _mm_sum_ps(_mm_mul_ps(a, b), _mm_mul_ps(c, d), _mm_mul_ps(e, f), _mm_mul_ps(g, h))
#define _mm_mag_ps(a,b,c,d) _mm_sqrt_ps(_mm_dot_ps(a,a,b,b,c,c,d,d))
#define _mm_cross_ps(a,b) _mm_fmsub_ps(_mm_permute_ps(a, swizzle::YZXW), _mm_permute_ps(b, swizzle::ZXYW), _mm_mul_ps(_mm_permute_ps(a, swizzle::ZXYW), _mm_permute_ps(b, swizzle::YZXW)))
#define _mm_transpose_ps(a,b,c,d,msk) _mm_shuffle_ps(_mm_shuffle_ps(a, b, msk), _mm_shuffle_ps(c, d, msk), swizzle::XZXZ)
#define _mm_clamp_ps(a,min,max) _mm_max_ps(_mm_min_ps(a, max), min)

#define _mm_sum2_ps(a,b) _mm_hadd_ps(_mm_hadd_ps(a,b), _mm_set1_ps(0.0f))
#define _mm_dot2_ps(a,b,c,d) _mm_sum2_ps(_mm_mul_ps(a, b), _mm_mul_ps(c, d))
#define _mm_mag2_ps(a,b) _mm_sqrt_ps(_mm_dot2_ps(a,a,b,b))

#define _mm_dot3_ps(a,b,c,d,e,f) _mm_sum_ps(_mm_mul_ps(a, b), _mm_mul_ps(c, d), _mm_mul_ps(e, f), _mm_set1_ps(0.0f))
#define _mm_dot3v_ps(a,b,c,d,e,f,v) _mm_sum_ps(_mm_mul_ps(a, b), _mm_mul_ps(c, d), _mm_mul_ps(e, f), v)
#define _mm_mag3_ps(a,b,c) _mm_sqrt_ps(_mm_dot3_ps(a,a,b,b,c,c))
#define _mm_mag3v_ps(a,b,c,v) _mm_sqrt_ps(_mm_dot3v_ps(a,a,b,b,c,c,v))
#endif

// High-level AVX2 vector instructions
#if ENABLE_INSTRUCTIONS_AVX2

#define _mm256_sum_ps(a,b,c,d) _mm256_hadd_ps(_mm256_hadd_ps(a,b), _mm256_hadd_ps(c,d))
#define _mm256_dot_ps(a,b,c,d,e,f,g,h) _mm256_sum_ps(_mm256_mul_ps(a, b), _mm256_mul_ps(c, d), _mm256_mul_ps(e, f), _mm256_mul_ps(g, h))

#endif

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
			vf::simd::compile_time::mul<N, T>( r, a, b );
			vf::simd::compile_time::add<N, T>( r, r, c );
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
		constexpr void cmpeq( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
		{
			VF_VEC_N_LOOP( N, r.elements[i] = a.elements[i] == b.elements[i] ? vf::primitives::BITWISE_HIGH<T> : vf::primitives::BITWISE_LOW<T> );
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

		template<size_t N, typename DstType, typename SrcType>
		constexpr void convert( VectorStorage<N, DstType>& r, const VectorStorage<N, SrcType>& a )
		{
			for( size_t i = 0LLU; i < N; ++i )
			{
				r.elements[i] = static_cast<DstType>( a.elements[i] );
			}
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

		template<typename T> requires std::is_fundamental_v<T>
		constexpr void dot4( VectorStorage<4, T>& r, const VectorStorage<4, T>& a, const VectorStorage<4, T>& b, const VectorStorage<4, T>& c, const VectorStorage<4, T>& d,
			const VectorStorage<4, T>& e, const VectorStorage<4, T>& f, const VectorStorage<4, T>& g, const VectorStorage<4, T>& h )
		{
			r.elements[0] = dot( a, b );
			r.elements[1] = dot( c, d );
			r.elements[2] = dot( e, f );
			r.elements[3] = dot( g, h );
		}

		template<typename T> requires std::floating_point<T>
		constexpr void length4( VectorStorage<4, T>& r, const VectorStorage<4, T>& a, const VectorStorage<4, T>& b, const VectorStorage<4, T>& c, const VectorStorage<4, T>& d )
		{
			r.elements[0] = Math::sqrt( dot( a, a ) );
			r.elements[1] = Math::sqrt( dot( b, b ) );
			r.elements[2] = Math::sqrt( dot( c, c ) );
			r.elements[3] = Math::sqrt( dot( d, d ) );
		}
	}

	template<size_t N, typename T>
	using BinaryVecOp = void ( * )( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b );

	typedef VectorStorage<4, f32> simd4f32;
	typedef VectorStorage<4, i32> simd4i32;
	typedef VectorStorage<4, u32> simd4u32;

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
	constexpr void cmpeq( VectorStorage<N, T>& r, const VectorStorage<N, T>& a, const VectorStorage<N, T>& b )
	{
		compile_time::cmpeq( r, a, b );
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

	template<size_t N, typename DstType, typename SrcType>
	constexpr void convert( VectorStorage<N, DstType>& r, const VectorStorage<N, SrcType>& a )
	{
		compile_time::convert( r, a );
	}

	template<size_t N, typename T>
	static void sqrt( VectorStorage<N, T>& r, const VectorStorage<N, T>& a )
	{
		VF_VEC_N_UNARY_FUNCTION( N, r.elements, a.elements, sqrtf );
	}

	template<typename T>
	constexpr void permute( VectorStorage<4, T>& r, const VectorStorage<4, T>& a, int b )
	{
		compile_time::permute( r, a, b );
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

	/**
	 * @brief Calculates 4 dot products simultaneously, returned as a vector
	 * @param a - first vector of first pair
	 * @param b - second vector of first pair
	 * @param c - first vector of second pair
	 * @param d - second vector of second pair
	 * @param e - first vector of third pair
	 * @param f - second vector of third pair
	 * @param g - first vector of fourth pair
	 * @param h - second vector of fourth pair
	 * @return 4 dot products stored as a single vector
	*/
	template<typename T> requires std::is_fundamental_v<T>
	constexpr void dot4( VectorStorage<4, T>& r, const VectorStorage<4, T>& a, const VectorStorage<4, T>& b, const VectorStorage<4, T>& c, const VectorStorage<4, T>& d,
		const VectorStorage<4, T>& e, const VectorStorage<4, T>& f, const VectorStorage<4, T>& g, const VectorStorage<4, T>& h )
	{
		compile_time::dot4( r, a, b, c, d, e, f, g, h );
	}

	template<typename T> requires std::floating_point<T>
	constexpr void length4( VectorStorage<4, T>& r, const VectorStorage<4, T>& a, const VectorStorage<4, T>& b, const VectorStorage<4, T>& c, const VectorStorage<4, T>& d )
	{
		compile_time::length4( r, a, b, c, d );
	}

	// SIMD 128-bit floating-point Operations

#if ENABLE_INSTRUCTIONS_SSE2

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
	constexpr void cmpeq( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::cmpeq( r, a, b );
		r.simd = _mm_cmpeq_ps( a.simd, b.simd );
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

	//template<>
	//constexpr void permute( simd4f32& r, const simd4f32& a, int b )
	//{
	//	if( std::is_constant_evaluated() ) return compile_time::permute( r, a, b );
	//	r.simd = _mm_permute_ps( a.simd, b );
	//}

	template<>
	constexpr void dot4( simd4f32& r, const simd4f32& a, const simd4f32& b, const simd4f32& c, const simd4f32& d,
		const simd4f32& e, const simd4f32& f, const simd4f32& g, const simd4f32& h )
	{
		if( std::is_constant_evaluated() ) return compile_time::dot4( r, a, b, c, d, e, f, g, h );
		r.simd = _mm_dot_ps( a.simd, b.simd, c.simd, d.simd, e.simd, f.simd, g.simd, h.simd );
	}

#endif

	// SIMD 128-bit Integer Operations

#if ENABLE_INSTRUCTIONS_SSE2

#define SSE2_NATIVE_BINARY_OPERATION( Type, Name, Func ) template<> constexpr void Name( Type& r, const Type& a, const Type& b ) { \
		if( std::is_constant_evaluated() ) return compile_time::Name( r, a, b ); \
		r.simd = Func( a.simd, b.simd ); }

	template<>
	constexpr simd4i32& set1( simd4i32& r, i32 value )
	{
		if( std::is_constant_evaluated() ) return compile_time::set1<4, i32>( r, value );
		r.simd = _mm_set1_epi32( value );
		return r;
	}

	SSE2_NATIVE_BINARY_OPERATION( simd4i32, add, _mm_add_epi32 );
	SSE2_NATIVE_BINARY_OPERATION( simd4i32, sub, _mm_sub_epi32 );
	SSE2_NATIVE_BINARY_OPERATION( simd4i32, mul, _mm_mul_epi32 );
	SSE2_NATIVE_BINARY_OPERATION( simd4i32, div, _mm_div_epi32 );
	SSE2_NATIVE_BINARY_OPERATION( simd4i32, hadd, _mm_hadd_epi32 );

	template<>
	constexpr void mod( simd4i32& r, const simd4i32& a, const simd4i32& b )
	{
		if( std::is_constant_evaluated() ) return compile_time::mod( r, a, b );
		_mm_idivrem_epi32( &r.simd, a.simd, b.simd );
	}

	template<>
	constexpr void fmadd( simd4i32& r, const simd4i32& a, const simd4i32& b, const simd4i32& c )
	{
		if( std::is_constant_evaluated() ) return compile_time::fmadd( r, a, b, c );
		r.simd = _mm_add_epi32( _mm_mul_epi32( a.simd, b.simd ), c.simd );
	}

	SSE2_NATIVE_BINARY_OPERATION( simd4i32, bitwise_and, _mm_and_epi32 );
	SSE2_NATIVE_BINARY_OPERATION( simd4i32, bitwise_or,  _mm_or_epi32 );
	SSE2_NATIVE_BINARY_OPERATION( simd4i32, bitwise_xor, _mm_xor_epi32 );

	//SSE2_NATIVE_BINARY_OPERATION( simd4i32, cmpge, _mm_cmpge_epi32_mask );
	//SSE2_NATIVE_BINARY_OPERATION( simd4i32, cmple, _mm_cmple_epi32_mask );
	//SSE2_NATIVE_BINARY_OPERATION( simd4i32, cmplt, _mm_cmplt_epi32_mask );
	//SSE2_NATIVE_BINARY_OPERATION( simd4i32, cmpgt, _mm_cmpgt_epi32_mask );

	SSE2_NATIVE_BINARY_OPERATION( simd4i32, min, _mm_min_epi32 );
	SSE2_NATIVE_BINARY_OPERATION( simd4i32, max, _mm_max_epi32 );
#endif

#if ENABLE_INSTRUCTIONS_SSE2

	// SIMD 128-bit Conversion Operations

#define SSE2_NATIVE_CONVERT_OPERATION( TypeA, TypeB, Func ) template<> constexpr void convert( TypeA& r, const TypeB& a ) { \
		if( std::is_constant_evaluated() ) return compile_time::convert( r, a ); \
		r.simd = Func( a.simd ); \
	}

	SSE2_NATIVE_CONVERT_OPERATION( simd4i32, simd4f32, _mm_cvtps_epi32 );
	SSE2_NATIVE_CONVERT_OPERATION( simd4u32, simd4f32, _mm_cvtps_epu32 );
	SSE2_NATIVE_CONVERT_OPERATION( simd4f32, simd4i32, _mm_cvtepi32_ps );
	SSE2_NATIVE_CONVERT_OPERATION( simd4f32, simd4u32, _mm_cvtepu32_ps );

#endif
}

#endif