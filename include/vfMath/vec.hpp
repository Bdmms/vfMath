#pragma once
#ifndef VF_VEC_HPP
#define VF_VEC_HPP

#include "swizzle.hpp"
#include <immintrin.h>
#include <memory>

union vec2f;
union vec4f;
union mat2x2;
union mat4x4;

union vec2i;
union vec4i;

extern "C" int __isa_available;

#define ENABLE_INSTRUCTIONS_SSE2	true
#define ENABLE_INSTRUCTIONS_AVX2	true
#define ENABLE_INSTRUCTIONS_AVX512	false

//constexpr bool ENABLE_X86 = __isa_available == __ISA_AVAILABLE_SSE2

constexpr char BMSK = (char)0xFF;

// High-level SSE2 vector instructions
#if ENABLE_INSTRUCTIONS_SSE2
constexpr __m128	SIMD_4f_ZERO = { 0.0f, 0.0f, 0.0f, 0.0f };
constexpr __m128i	SIMD_4i_ZERO = { 0, 0, 0, 0 };
constexpr __m128	SIMD_4f_ONES = { 1.0f, 1.0f, 1.0f, 1.0f };
constexpr __m128i	SIMD_4i_ONES = { 1, 1, 1, 1 };
constexpr __m128	SIMD_4f_NEG = { -1.0f, -1.0f, -1.0f, -1.0f };
constexpr __m128i	SIMD_4i_NEG = { -1, -1, -1, -1 };
constexpr __m128	SIMD_4f_W = { 0.0f, 0.0f, 0.0f, 1.0f };
constexpr __m128i	SIMD_4i_W = { 0, 0, 0, 1 };
constexpr __m128i	SIMD_4i_BITMASK = { BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK, BMSK };
// TODO: Compensate for __m128i using bytes!!!
#endif

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

// High-level AVX2 vector instructions
#if ENABLE_INSTRUCTIONS_AVX2
constexpr __m256	SIMD_8f_ZERO = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
constexpr __m256i	SIMD_8i_ZERO = { 0, 0, 0, 0, 0, 0, 0, 0 };
constexpr __m256	SIMD_8f_ONES = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
constexpr __m256i	SIMD_8i_ONES = { 1, 1, 1, 1, 1, 1, 1, 1 };
constexpr __m256	SIMD_8f_NEG = { -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f };
constexpr __m256i	SIMD_8i_NEG = { -1, -1, -1, -1, -1, -1, -1, -1 };
constexpr __m256	SIMD_8f_W = { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
constexpr __m256i	SIMD_8i_W = { 0, 0, 0, 1, 0, 0, 0, 1 };
#endif

#define _mm256_sum_ps(a,b,c,d) _mm256_hadd_ps(_mm256_hadd_ps(a,b), _mm256_hadd_ps(c,d))
#define _mm256_dot_ps(a,b,c,d,e,f,g,h) _mm256_sum_ps(_mm256_mul_ps(a, b), _mm256_mul_ps(c, d), _mm256_mul_ps(e, f), _mm256_mul_ps(g, h))
#define _mm256_mag_ps(a,b,c,d) _mm256_sqrt_ps(_mm256_dot_ps(a,a,b,b,c,c,d,d))
#define _mm256_transpose_ps(a,b,msk) _mm256_shuffle_ps(_mm256_shuffle_ps(a, b, msk), _mm256_shuffle_ps(c, d, msk), swizzle::XZXZ)
#define _mm256_clamp_ps(a,min,max) _mm256_min_ps(_mm256_max_ps(a, max), min)

#define _mm256_dot3_ps(a,b,c,d,e,f,g,h) _mm256_sum_ps(_mm256_mul_ps(a, b), _mm256_mul_ps(c, d), _mm256_mul_ps(e, f), SIMD_8f_ZERO)

#endif