#pragma once
#ifndef VF_MAT_3x3_HPP
#define VF_MAT_3x3_HPP

#include "VectorMath.hpp"

union mat3x3
{
	float m[9];
	struct { float m0, m1, m2, m3, m4, m5, m6, m7, m8;  };
	struct { __m128 simd[2]; float rm; };
	struct { __m256 avx2; float rx; };

	constexpr mat3x3() : m() {}

	constexpr mat3x3(const float a, const float b, const float c,
		const float d, const float e, const float f,
		const float g, const float h, const float i)
		: m{ a, b, c, d, e, f, g, h, i } {}

	constexpr mat3x3(const vec3f& a, const vec3f& b, const vec3f& c)
		: m{ a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z } {}

	constexpr mat3x3(const __m128& a, const __m128& b, const float m8)
		: simd{ a, b }, rm(m8) {}

	constexpr mat3x3(const __m256& a, const float m8)
		: avx2{ a }, rx(m8) {}

	constexpr float operator[](const unsigned char i) const { return m[i]; }
	constexpr float& operator[](const unsigned char i) { return m[i]; }

	mat3x3& operator+=(const mat3x3& b) noexcept
	{
#if ENABLE_INSTRUCTIONS_AVX2
		avx2 = _mm256_add_ps(avx2, b.avx2);
#elif ENABLE_INSTRUCTIONS_SSE2
		simd[0] = _mm_add_ps(simd[0], b.simd[0]);
		simd[1] = _mm_add_ps(simd[1], b.simd[1]);
#else
		for (u32 i = 0; i < 8; ++i) m[i] += b.m[i];
#endif
		rm += b.rm;
		return *this;
	}

	mat3x3& operator-=(const mat3x3& b) noexcept
	{
#if ENABLE_INSTRUCTIONS_AVX2
		avx2 = _mm256_sub_ps(avx2, b.avx2);
#elif ENABLE_INSTRUCTIONS_SSE2
		simd[0] = _mm_sub_ps(simd[0], b.simd[0]);
		simd[1] = _mm_sub_ps(simd[1], b.simd[1]);
#else
		for (u32 i = 0; i < 8; ++i) m[i] -= b.m[i];
#endif
		rm -= b.rm;
		return *this;
	}

	mat3x3& operator*=(const float b) noexcept
	{
#if ENABLE_INSTRUCTIONS_AVX2
		avx2 = _mm256_mul_ps(avx2, _mm256_set1_ps(b));
#elif ENABLE_INSTRUCTIONS_SSE2
		__m128 scale = _mm_set1_ps(b);
		simd[0] = _mm_mul_ps(simd[0], scale);
		simd[1] = _mm_mul_ps(simd[1], scale);
#else
		for (u32 i = 0; i < 8; ++i) m[i] *= b;
#endif
		rm *= b;
		return *this;
	}

	mat3x3& operator/=(const float b) noexcept
	{
#if ENABLE_INSTRUCTIONS_AVX2
		avx2 = _mm256_div_ps(avx2, _mm256_set1_ps(b));
#elif ENABLE_INSTRUCTIONS_SSE2
		__m128 scale = _mm_set1_ps(b);
		simd[0] = _mm_div_ps(simd[0], scale);
		simd[1] = _mm_div_ps(simd[1], scale);
#else
		for (u32 i = 0; i < 8; ++i) m[i] /= b;
#endif
		rm /= b;
		return *this;
	}

	constexpr mat3x3& operator*=(const mat3x3& b) noexcept
	{
		m0 = m0 * b.m0 + m3 * b.m1 + m6 * b.m2;
		m1 = m1 * b.m0 + m4 * b.m1 + m7 * b.m2;
		m2 = m2 * b.m0 + m5 * b.m1 + m8 * b.m2;

		m3 = m0 * b.m3 + m3 * b.m4 + m6 * b.m5;
		m4 = m1 * b.m3 + m4 * b.m4 + m7 * b.m5;
		m5 = m2 * b.m3 + m5 * b.m4 + m8 * b.m5;

		m6 = m0 * b.m6 + m3 * b.m7 + m6 * b.m8;
		m7 = m1 * b.m6 + m4 * b.m7 + m7 * b.m8;
		m8 = m2 * b.m6 + m5 * b.m7 + m8 * b.m8;
		return *this;
	}

	[[nodiscard]] mat3x3 transpose() const
	{
		return { m0, m3, m6, m1, m4, m7, m2, m5, m8 };
	}
};

#endif