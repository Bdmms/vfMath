#pragma once
#ifndef VF_VEC_HPP
#define VF_VEC_HPP

#include "swizzle.hpp"
#include "primitives.hpp"
#include "vec_store.hpp"
#include "vec_ops.hpp"

#include <immintrin.h>
#include <memory>
#include <bit>

extern "C" int __isa_available;

#define ENABLE_INSTRUCTIONS_SSE2	__isa_available >= __ISA_AVAILABLE_SSE2
#define ENABLE_INSTRUCTIONS_AVX2	__isa_available >= __ISA_AVAILABLE_AVX2
#define ENABLE_INSTRUCTIONS_AVX512	false

constexpr char BMSK = (char)0xFF;
constexpr float FMSK = std::bit_cast<float>( -1 );

template<size_t N, typename T> 
struct vec : public VectorStorage<N, T>
{
	using Storage = VectorStorage<N, T>;

	// Default Constuctor
	//template<typename ... TArgs>
	//constexpr vec( TArgs&& ... args ) : VectorStorage<N, T>{ std::forward<TArgs>( args )... } {}

	// Default Constuctor
	constexpr vec() : VectorStorage<N, T>{} {}

	// Initializing List
	constexpr vec( const std::initializer_list<T>& list ) : VectorStorage<N, T>{} 
	{
		std::copy( list.begin(), list.end(), Storage::elements.data() );
	}

	// Copy Constructor
	constexpr vec( const vec<N, T>& copy ) : VectorStorage<N, T>( copy ) {}

	// Cast Constructor
	//template<typename DstType>
	//constexpr vec( const vec<N, DstType>& cast_copy ) : VectorStorage<N, T>{}
	//{
	//	for( size_t i = 0LLU; i < N; ++i )
	//	{
	//		Storage::elements[i] = static_cast<DstType>( cast_copy.elements[i] );
	//	}
	//}

	// Container Operations
	constexpr T* data() { return Storage::elements.data(); }
	constexpr const T* data() const { return Storage::elements.data(); }
	constexpr size_t size() { return Storage::elements.size(); }

	// Array Operators
	constexpr T operator[]( size_t i ) const { return Storage::elements[i]; }
	constexpr T& operator[]( size_t i ) { return Storage::elements[i]; }

	// Unary Operators
	[[nodiscard]] constexpr vec operator-() const 
	{ 
		vec<N, T> r;
		vf::simd::negate<N, T>( r, *this );
		return r;
	}
	[[nodiscard]] constexpr vec operator~() const 
	{
		vec<N, T> r;
		vf::simd::bitwise_not<N, T>( r, *this );
		return r;
	}

	// Arithmetic Operators
	constexpr vec& operator+=( const vec& rhs ) noexcept { vf::simd::add<N,T>( *this, *this, rhs ); return *this; }
	constexpr vec& operator-=( const vec& rhs ) noexcept { vf::simd::sub<N, T>( *this, *this, rhs ); return *this; }
	constexpr vec& operator*=( const vec& rhs ) noexcept { vf::simd::mul<N, T>( *this, *this, rhs ); return *this; }
	constexpr vec& operator/=( const vec& rhs ) noexcept { vf::simd::div<N, T>( *this, *this, rhs ); return *this; }
	constexpr vec& operator%=( const vec& rhs ) noexcept { vf::simd::mod<N, T>( *this, *this, rhs ); return *this; }

	constexpr vec& operator+=( T rhs ) noexcept { vf::simd::add<N, T>( *this, *this, vf::simd::set1<N, T>( rhs ) ); return *this; }
	constexpr vec& operator-=( T rhs ) noexcept { vf::simd::sub<N, T>( *this, *this, vf::simd::set1<N, T>( rhs ) ); return *this; }
	constexpr vec& operator*=( T rhs ) noexcept { vf::simd::mul<N, T>( *this, *this, vf::simd::set1<N, T>( rhs ) ); return *this; }
	constexpr vec& operator/=( T rhs ) noexcept { vf::simd::div<N, T>( *this, *this, vf::simd::set1<N, T>( rhs ) ); return *this; }
	constexpr vec& operator%=( T rhs ) noexcept { vf::simd::mod<N, T>( *this, *this, vf::simd::set1<N, T>( rhs ) ); return *this; }

	// Logical Operators
	constexpr vec& operator&=( const vec& rhs ) noexcept { vf::simd::bitwise_and<N, T>( *this, *this, rhs ); return *this; }
	constexpr vec& operator|=( const vec& rhs ) noexcept { vf::simd::bitwise_or<N, T>(  *this, *this, rhs ); return *this; }
	constexpr vec& operator^=( const vec& rhs ) noexcept { vf::simd::bitwise_xor<N, T>( *this, *this, rhs ); return *this; }

	constexpr vec& operator&=( T rhs ) noexcept { vf::simd::bitwise_and<N, T>( *this, *this, vf::simd::set1<N, T>( rhs ) ); return *this; }
	constexpr vec& operator|=( T rhs ) noexcept { vf::simd::bitwise_or<N, T>(  *this, *this, vf::simd::set1<N, T>( rhs ) ); return *this; }
	constexpr vec& operator^=( T rhs ) noexcept { vf::simd::bitwise_xor<N, T>( *this, *this, vf::simd::set1<N, T>( rhs ) ); return *this; }

	// Conversions
	template<typename DstType>
	[[nodiscard]] constexpr explicit operator vec<N, DstType>() const
	{
		vec<N, DstType> r;
		for( size_t i = 0LLU; i < N; ++i )
		{
			r.elements[i] = static_cast<DstType>( Storage::elements[i] );
		}
		return r;
	}
};

#define VF_SCALAR T
#define VF_VECTOR vec<N, T>

#define VF_VECTOR_OPERATOR_HEAD( Op ) template<size_t N, typename T> [[nodiscard]] constexpr VF_VECTOR operator##Op
#define VF_VECTOR_OPERATOR_VECTOR_VECTOR( Op, Func ) VF_VECTOR_OPERATOR_HEAD( Op )( const VF_VECTOR& a, const VF_VECTOR& b ) noexcept \
	 { VF_VECTOR r; Func<N,T>( r, a, b ); return r; }
#define VF_VECTOR_OPERATOR_VECTOR_SCALAR( Op, Func ) VF_VECTOR_OPERATOR_HEAD( Op )( const VF_VECTOR& a, const VF_SCALAR& b ) noexcept \
	 { VF_VECTOR r; Func<N,T>( r, a, vf::simd::set1<N,T>( b ) ); return r; }
#define VF_VECTOR_OPERATOR_SCALAR_VECTOR( Op, Func ) VF_VECTOR_OPERATOR_HEAD( Op )( const VF_SCALAR& a, const VF_VECTOR& b ) noexcept \
	 { VF_VECTOR r; Func<N,T>( r, vf::simd::set1<N,T>( a ), b ); return r; }

// Non-Member Arithmetic Operators

VF_VECTOR_OPERATOR_VECTOR_VECTOR( +, vf::simd::add );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( -, vf::simd::sub );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( *, vf::simd::mul );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( /, vf::simd::div );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( %, vf::simd::mod );

VF_VECTOR_OPERATOR_VECTOR_SCALAR( +, vf::simd::add );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( -, vf::simd::sub );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( *, vf::simd::mul );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( /, vf::simd::div );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( %, vf::simd::mod );

VF_VECTOR_OPERATOR_SCALAR_VECTOR( +, vf::simd::add );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( -, vf::simd::sub );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( *, vf::simd::mul );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( /, vf::simd::div );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( %, vf::simd::mod );

// Non-Member Logical Operators

VF_VECTOR_OPERATOR_VECTOR_VECTOR( &, vf::simd::bitwise_and );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( |, vf::simd::bitwise_or );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( ^, vf::simd::bitwise_xor );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( &&, vf::simd::bitwise_and );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( || , vf::simd::bitwise_or );

VF_VECTOR_OPERATOR_VECTOR_SCALAR( &, vf::simd::bitwise_and );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( |, vf::simd::bitwise_or );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( ^, vf::simd::bitwise_xor );

VF_VECTOR_OPERATOR_SCALAR_VECTOR( &, vf::simd::bitwise_and );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( |, vf::simd::bitwise_or );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( ^, vf::simd::bitwise_xor );

// Non-Member Comparison Operators
#define VF_OPERATOR_LE <=
#define VF_OPERATOR_GE >=
#define VF_OPERATOR_LT <
#define VF_OPERATOR_GT >

VF_VECTOR_OPERATOR_VECTOR_VECTOR( VF_OPERATOR_LE, vf::simd::cmple );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( VF_OPERATOR_GE, vf::simd::cmpge );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( VF_OPERATOR_LT, vf::simd::cmplt );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( VF_OPERATOR_GT, vf::simd::cmpgt );

VF_VECTOR_OPERATOR_VECTOR_SCALAR( VF_OPERATOR_LE, vf::simd::cmple );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( VF_OPERATOR_GE, vf::simd::cmpge );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( VF_OPERATOR_LT, vf::simd::cmplt );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( VF_OPERATOR_GT, vf::simd::cmpgt );

VF_VECTOR_OPERATOR_SCALAR_VECTOR( VF_OPERATOR_LE, vf::simd::cmple );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( VF_OPERATOR_GE, vf::simd::cmpge );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( VF_OPERATOR_LT, vf::simd::cmplt );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( VF_OPERATOR_GT, vf::simd::cmpgt );

// Explcit Vector Typedefs

template <typename T>
using vec2 = vec<2, T>;
template <typename T>
using vec4 = vec<4, T>;

typedef vec2<f32> vec2f;
typedef vec2<i32> vec2i;
typedef vec2<u32> vec2u;

typedef vec2<f32> vec2f32;
typedef vec2<f64> vec2f64;

typedef vec2<i8>  vec2i8;
typedef vec2<i16> vec2i16;
typedef vec2<i32> vec2i32;
typedef vec2<i64> vec2i64;

typedef vec2<u8>  vec2u8;
typedef vec2<u16> vec2u16;
typedef vec2<u32> vec2u32;
typedef vec2<u64> vec2u64;

typedef vec4<f32> vec4f;

typedef vec4<f32> vec4f32;
typedef vec4<f64> vec4f64;

//struct vec4f;
union vec4i;

union mat2x2;
union mat4x4;

namespace vf::native
{
	constexpr __m128 simd_add_4f( const __m128& a, const __m128& b )
	{
		return { 
			a.m128_f32[0] + b.m128_f32[0], 
			a.m128_f32[1] + b.m128_f32[1], 
			a.m128_f32[2] + b.m128_f32[2], 
			a.m128_f32[3] + b.m128_f32[3]
		};
	}

	constexpr __m128 simd_sub_4f( const __m128& a, const __m128& b )
	{
		return {
			a.m128_f32[0] - b.m128_f32[0],
			a.m128_f32[1] - b.m128_f32[1],
			a.m128_f32[2] - b.m128_f32[2],
			a.m128_f32[3] - b.m128_f32[3]
		};
	}

	constexpr __m128 simd_mul_4f( const __m128& a, const __m128& b )
	{
		return {
			a.m128_f32[0] * b.m128_f32[0],
			a.m128_f32[1] * b.m128_f32[1],
			a.m128_f32[2] * b.m128_f32[2],
			a.m128_f32[3] * b.m128_f32[3]
		};
	}

	constexpr __m128 simd_div_4f( const __m128& a, const __m128& b )
	{
		return {
			a.m128_f32[0] / b.m128_f32[0],
			a.m128_f32[1] / b.m128_f32[1],
			a.m128_f32[2] / b.m128_f32[2],
			a.m128_f32[3] / b.m128_f32[3]
		};
	}

	constexpr __m128 simd_and_4f( const __m128& a, const __m128& b )
	{
		__m128 result;
		result.m128_i32[0] = a.m128_i32[0] & b.m128_i32[0];
		result.m128_i32[1] = a.m128_i32[1] & b.m128_i32[1];
		result.m128_i32[2] = a.m128_i32[2] & b.m128_i32[2];
		result.m128_i32[3] = a.m128_i32[3] & b.m128_i32[3];
		return result;
	}

	constexpr __m128 simd_or_4f( const __m128& a, const __m128& b )
	{
		__m128 result;
		result.m128_i32[0] = a.m128_i32[0] | b.m128_i32[0];
		result.m128_i32[1] = a.m128_i32[1] | b.m128_i32[1];
		result.m128_i32[2] = a.m128_i32[2] | b.m128_i32[2];
		result.m128_i32[3] = a.m128_i32[3] | b.m128_i32[3];
		return result;
	}

	constexpr __m128 simd_xor_4f( const __m128& a, const __m128& b )
	{
		__m128 result;
		result.m128_i32[0] = a.m128_i32[0] ^ b.m128_i32[0];
		result.m128_i32[1] = a.m128_i32[1] ^ b.m128_i32[1];
		result.m128_i32[2] = a.m128_i32[2] ^ b.m128_i32[2];
		result.m128_i32[3] = a.m128_i32[3] ^ b.m128_i32[3];
		return result;
	}

	constexpr __m128 simd_hadd_4f( const __m128& a, const __m128& b )
	{
		return {
			a.m128_f32[0] + a.m128_f32[1],
			a.m128_f32[2] + a.m128_f32[3],
			b.m128_f32[0] + b.m128_f32[1],
			b.m128_f32[2] + b.m128_f32[3]
		};
	}
	
	constexpr __m128 simd_cross_4f( const __m128& a, const __m128& b )
	{
		return { 
			a.m128_f32[1] * b.m128_f32[2] - a.m128_f32[2] * b.m128_f32[1],
			a.m128_f32[2] * b.m128_f32[0] - a.m128_f32[0] * b.m128_f32[2],
			a.m128_f32[0] * b.m128_f32[1] - a.m128_f32[1] * b.m128_f32[0],
			0.0f
		};
	}

	constexpr __m128 simd_permute_4f( const __m128& a, int b )
	{
		return {
			a.m128_f32[(b & 0x03)     ],
			a.m128_f32[(b & 0x0C) >> 2],
			a.m128_f32[(b & 0x30) >> 4],
			a.m128_f32[(b & 0xC0) >> 6]
		};
	}

	constexpr __m128 simd_min_4f( const __m128& a, const __m128& b )
	{
		return {
			std::min( a.m128_f32[0], b.m128_f32[0] ),
			std::min( a.m128_f32[1], b.m128_f32[1] ),
			std::min( a.m128_f32[2], b.m128_f32[2] ),
			std::min( a.m128_f32[3], b.m128_f32[3] )
		};
	}

	constexpr __m128 simd_max_4f( const __m128& a, const __m128& b )
	{
		return {
			std::max( a.m128_f32[0], b.m128_f32[0] ),
			std::max( a.m128_f32[1], b.m128_f32[1] ),
			std::max( a.m128_f32[2], b.m128_f32[2] ),
			std::max( a.m128_f32[3], b.m128_f32[3] )
		};
	}

	constexpr __m128 simd_cmpgt_4f( const __m128& a, const __m128& b )
	{
		__m128 result;
		result.m128_i32[0] = a.m128_f32[0] > b.m128_f32[0] ? -1 : 0;
		result.m128_i32[1] = a.m128_f32[1] > b.m128_f32[1] ? -1 : 0;
		result.m128_i32[2] = a.m128_f32[2] > b.m128_f32[2] ? -1 : 0;
		result.m128_i32[3] = a.m128_f32[3] > b.m128_f32[3] ? -1 : 0;
		return result;
	}

	constexpr __m128 simd_cmpge_4f( const __m128& a, const __m128& b )
	{
		__m128 result;
		result.m128_i32[0] = a.m128_f32[0] >= b.m128_f32[0] ? -1 : 0;
		result.m128_i32[1] = a.m128_f32[1] >= b.m128_f32[1] ? -1 : 0;
		result.m128_i32[2] = a.m128_f32[2] >= b.m128_f32[2] ? -1 : 0;
		result.m128_i32[3] = a.m128_f32[3] >= b.m128_f32[3] ? -1 : 0;
		return result;
	}

	constexpr __m128 simd_cmplt_4f( const __m128& a, const __m128& b )
	{
		__m128 result;
		result.m128_i32[0] = a.m128_f32[0] < b.m128_f32[0] ? -1 : 0;
		result.m128_i32[1] = a.m128_f32[1] < b.m128_f32[1] ? -1 : 0;
		result.m128_i32[2] = a.m128_f32[2] < b.m128_f32[2] ? -1 : 0;
		result.m128_i32[3] = a.m128_f32[3] < b.m128_f32[3] ? -1 : 0;
		return result;
	}

	constexpr __m128 simd_cmple_4f( const __m128& a, const __m128& b )
	{
		__m128 result;
		result.m128_i32[0] = a.m128_f32[0] <= b.m128_f32[0] ? -1 : 0;
		result.m128_i32[1] = a.m128_f32[1] <= b.m128_f32[1] ? -1 : 0;
		result.m128_i32[2] = a.m128_f32[2] <= b.m128_f32[2] ? -1 : 0;
		result.m128_i32[3] = a.m128_f32[3] <= b.m128_f32[3] ? -1 : 0;
		return result;
	}

	constexpr __m128i simd_min_4i( const __m128i& a, const __m128i& b )
	{
		__m128i result;
		result.m128i_i32[0] = std::min( a.m128i_i32[0], b.m128i_i32[0] );
		result.m128i_i32[1] = std::min( a.m128i_i32[1], b.m128i_i32[1] );
		result.m128i_i32[2] = std::min( a.m128i_i32[2], b.m128i_i32[2] );
		result.m128i_i32[3] = std::min( a.m128i_i32[3], b.m128i_i32[3] );
		return result;
	}

	constexpr __m128i simd_max_4i( const __m128i& a, const __m128i& b )
	{
		__m128i result;
		result.m128i_i32[0] = std::max( a.m128i_i32[0], b.m128i_i32[0] );
		result.m128i_i32[1] = std::max( a.m128i_i32[1], b.m128i_i32[1] );
		result.m128i_i32[2] = std::max( a.m128i_i32[2], b.m128i_i32[2] );
		result.m128i_i32[3] = std::max( a.m128i_i32[3], b.m128i_i32[3] );
		return result;
	}
}

// High-level SSE2 vector instructions
#if ENABLE_INSTRUCTIONS_SSE2
constexpr __m128	SIMD_4f_ZERO = { 0.0f, 0.0f, 0.0f, 0.0f };
constexpr __m128i	SIMD_4i_ZERO = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
constexpr __m128	SIMD_4f_ONES = { 1.0f, 1.0f, 1.0f, 1.0f };
constexpr __m128i	SIMD_4i_ONES = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0 };
constexpr __m128	SIMD_4f_NEG = { -1.0f, -1.0f, -1.0f, -1.0f };
constexpr __m128i	SIMD_4i_NEG = { BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK };
constexpr __m128	SIMD_4f_W = { 0.0f, 0.0f, 0.0f, 1.0f };
constexpr __m128i	SIMD_4i_W = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 };
constexpr __m128	SIMD_4f_MASK = { FMSK, FMSK, FMSK, FMSK };
constexpr __m128i   SIMD_4i_MASK = SIMD_4i_NEG;

#define _mm_xyzw_epi32(x,y,z,w) _mm_set_epi32(w,z,y,x)

#define _mm_xyzw_ps(x,y,z,w) _mm_set_ps(w,z,y,x)
#define _mm_sum_ps(a,b,c,d) _mm_hadd_ps(_mm_hadd_ps(a,b), _mm_hadd_ps(c,d))
#define _mm_dot_ps(a,b,c,d,e,f,g,h) _mm_sum_ps(_mm_mul_ps(a, b), _mm_mul_ps(c, d), _mm_mul_ps(e, f), _mm_mul_ps(g, h))
#define _mm_mag_ps(a,b,c,d) _mm_sqrt_ps(_mm_dot_ps(a,a,b,b,c,c,d,d))
#define _mm_cross_ps(a,b) _mm_fmsub_ps(_mm_permute_ps(a, swizzle::YZXW), _mm_permute_ps(b, swizzle::ZXYW), _mm_mul_ps(_mm_permute_ps(a, swizzle::ZXYW), _mm_permute_ps(b, swizzle::YZXW)))
#define _mm_transpose_ps(a,b,c,d,msk) _mm_shuffle_ps(_mm_shuffle_ps(a, b, msk), _mm_shuffle_ps(c, d, msk), swizzle::XZXZ)
#define _mm_clamp_ps(a,min,max) _mm_max_ps(_mm_min_ps(a, max), min)

#define _mm_sum2_ps(a,b) _mm_hadd_ps(_mm_hadd_ps(a,b), SIMD_4f_ZERO)
#define _mm_dot2_ps(a,b,c,d) _mm_sum2_ps(_mm_mul_ps(a, b), _mm_mul_ps(c, d))
#define _mm_mag2_ps(a,b) _mm_sqrt_ps(_mm_dot2_ps(a,a,b,b))

#define _mm_dot3_ps(a,b,c,d,e,f) _mm_sum_ps(_mm_mul_ps(a, b), _mm_mul_ps(c, d), _mm_mul_ps(e, f), SIMD_4f_ZERO)
#define _mm_dot3v_ps(a,b,c,d,e,f,v) _mm_sum_ps(_mm_mul_ps(a, b), _mm_mul_ps(c, d), _mm_mul_ps(e, f), v)
#define _mm_mag3_ps(a,b,c) _mm_sqrt_ps(_mm_dot3_ps(a,a,b,b,c,c))
#define _mm_mag3v_ps(a,b,c,v) _mm_sqrt_ps(_mm_dot3v_ps(a,a,b,b,c,c,v))
#endif

// High-level AVX2 vector instructions
#if ENABLE_INSTRUCTIONS_AVX2
constexpr __m256	SIMD_8f_ZERO = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
constexpr __m256i	SIMD_8i_ZERO = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
constexpr __m256	SIMD_8f_ONES = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
constexpr __m256i	SIMD_8i_ONES = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0 };
constexpr __m256	SIMD_8f_NEG = { -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f };
constexpr __m256i	SIMD_8i_NEG = { BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK };
constexpr __m256	SIMD_8f_W = { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
constexpr __m256i	SIMD_8i_W = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0 };

#define _mm256_sum_ps(a,b,c,d) _mm256_hadd_ps(_mm256_hadd_ps(a,b), _mm256_hadd_ps(c,d))
#define _mm256_dot_ps(a,b,c,d,e,f,g,h) _mm256_sum_ps(_mm256_mul_ps(a, b), _mm256_mul_ps(c, d), _mm256_mul_ps(e, f), _mm256_mul_ps(g, h))
#define _mm256_mag_ps(a,b,c,d) _mm256_sqrt_ps(_mm256_dot_ps(a,a,b,b,c,c,d,d))
#define _mm256_transpose_ps(a,b,msk) _mm256_shuffle_ps(_mm256_shuffle_ps(a, b, msk), _mm256_shuffle_ps(c, d, msk), swizzle::XZXZ)
#define _mm256_clamp_ps(a,min,max) _mm256_min_ps(_mm256_max_ps(a, max), min)

#define _mm256_dot3_ps(a,b,c,d,e,f,g,h) _mm256_sum_ps(_mm256_mul_ps(a, b), _mm256_mul_ps(c, d), _mm256_mul_ps(e, f), SIMD_8f_ZERO)
#endif

// Macros for selecting native function
#define SIMD_4F_SET( x, y, z, w ) std::is_constant_evaluated() ? __m128{ x, y, z, w } : _mm_xyzw_ps( x, y, z, w )
#define SIMD_4F_SET1( x ) std::is_constant_evaluated() ? __m128{ x, x, x, x } : _mm_set1_ps( x )
#define SIMD_4F_ADD( x, y ) std::is_constant_evaluated() ? vf::native::simd_add_4f( x, y ) : _mm_add_ps( x, y )
#define SIMD_4F_SUB( x, y ) std::is_constant_evaluated() ? vf::native::simd_sub_4f( x, y ) : _mm_sub_ps( x, y )
#define SIMD_4F_MUL( x, y ) std::is_constant_evaluated() ? vf::native::simd_mul_4f( x, y ) : _mm_mul_ps( x, y )
#define SIMD_4F_DIV( x, y ) std::is_constant_evaluated() ? vf::native::simd_div_4f( x, y ) : _mm_div_ps( x, y )
#define SIMD_4F_AND( x, y ) std::is_constant_evaluated() ? vf::native::simd_and_4f( x, y ) : _mm_and_ps( x, y )
#define SIMD_4F_OR( x, y ) std::is_constant_evaluated() ? vf::native::simd_or_4f( x, y ) : _mm_or_ps( x, y )
#define SIMD_4F_XOR( x, y ) std::is_constant_evaluated() ? vf::native::simd_xor_4f( x, y ) : _mm_xor_ps( x, y )
#define SIMD_4F_HADD( x, y ) std::is_constant_evaluated() ? vf::native::simd_hadd_4f( x, y ) : _mm_hadd_ps( x, y )
#define SIMD_4F_FMADD( a, b, c ) std::is_constant_evaluated() ? vf::native::simd_add_4f( vf::native::simd_mul_4f( a, b ), c ) : _mm_fmadd_ps( a, b, c )
#define SIMD_4F_CROSS( x, y ) std::is_constant_evaluated() ? vf::native::simd_cross_4f( x, y ) : _mm_cross_ps( x, y )
#define SIMD_4F_MIN( x, y ) std::is_constant_evaluated() ? vf::native::simd_min_4f( x, y ) : _mm_min_ps( x, y )
#define SIMD_4F_MAX( x, y ) std::is_constant_evaluated() ? vf::native::simd_max_4f( x, y ) : _mm_max_ps( x, y )
#define SIMD_4F_CLAMP( x, min, max ) SIMD_4F_MAX( SIMD_4F_MIN( x, max ), min )
#define SIMD_4F_SUM4( a, b, c, d ) SIMD_4F_HADD( SIMD_4F_HADD( a, b ), SIMD_4F_HADD( c, d ) )
#define SIMD_4F_SUM2( a, b ) SIMD_4F_HADD( SIMD_4F_HADD( a, b ), SIMD_4f_ZERO )
#define SIMD_4F_DOT2( a, b, c, d ) SIMD_4F_SUM2( SIMD_4F_MUL( a, b ), SIMD_4F_MUL( c, d ) )
#define SIMD_4F_DOT3( a, b, c, d, e, f ) SIMD_4F_SUM4( SIMD_4F_MUL( a, b ), SIMD_4F_MUL( c, d ), SIMD_4F_MUL( e, f ), SIMD_4f_ZERO )
#define SIMD_4F_DOT4( a, b, c, d, e, f, g, h ) SIMD_4F_SUM4( SIMD_4F_MUL( a, b ), SIMD_4F_MUL( c, d ), SIMD_4F_MUL( e, f ), SIMD_4F_MUL( g, h ) )
#define SIMD_4F_PERMUTE( a, b ) std::is_constant_evaluated() ? vf::native::simd_permute_4f( a, b ) : _mm_permute_ps( a, b )
#define SIMD_4F_CMP_GT( x, y ) std::is_constant_evaluated() ? vf::native::simd_cmpgt_4f( x, y ) : _mm_cmpgt_ps( x, y )
#define SIMD_4F_CMP_GE( x, y ) std::is_constant_evaluated() ? vf::native::simd_cmpge_4f( x, y ) : _mm_cmpge_ps( x, y )
#define SIMD_4F_CMP_LT( x, y ) std::is_constant_evaluated() ? vf::native::simd_cmplt_4f( x, y ) : _mm_cmplt_ps( x, y )
#define SIMD_4F_CMP_LE( x, y ) std::is_constant_evaluated() ? vf::native::simd_cmple_4f( x, y ) : _mm_cmple_ps( x, y )

#define SIMD_4I_MIN( x, y ) std::is_constant_evaluated() ? vf::native::simd_min_4i( x, y ) : _mm_min_epi32( x, y )
#define SIMD_4I_MAX( x, y ) std::is_constant_evaluated() ? vf::native::simd_max_4i( x, y ) : _mm_max_epi32( x, y )
#define SIMD_4I_CLAMP( x, min, max ) SIMD_4I_MAX( SIMD_4I_MIN( x, max ), min )

#endif