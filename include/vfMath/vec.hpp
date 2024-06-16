#pragma once
#ifndef VF_VEC_HPP
#define VF_VEC_HPP

#include "swizzle.hpp"
#include <immintrin.h>
#include <memory>
#include <bit>

union vec2f;
union vec4f;
union mat2x2;
union mat4x4;

union vec2i;
union vec4i;

extern "C" int __isa_available;

#define ENABLE_INSTRUCTIONS_SSE2	__isa_available >= __ISA_AVAILABLE_SSE2
#define ENABLE_INSTRUCTIONS_AVX2	__isa_available >= __ISA_AVAILABLE_AVX2
#define ENABLE_INSTRUCTIONS_AVX512	false

constexpr char BMSK = (char)0xFF;
constexpr float FMSK = std::bit_cast<float>( -1 );

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