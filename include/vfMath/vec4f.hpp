#pragma once
#ifndef VF_VEC_4D_FLOAT_HPP
#define VF_VEC_4D_FLOAT_HPP

#include "vec2f.hpp"

#define VECTOR_FORWARD(x) { x }
//#define VECTOR_FORWARD(x) reinterpret_cast<vec4f&&>(std::forward<__m128>(x))

/**
 * @brief 4D floating-point vector
*/
union vec4f
{
	using ElementType = float;

	__m128 simd;
	float v[4];
	struct { float x, y, z, w; };
	struct { float r, g, b, a; };

	// Operators
	constexpr float operator[](const unsigned char i) const { return v[i]; }
	constexpr float& operator[](const unsigned char i) { return v[i]; }
	vec4f& operator+=(const vec4f& b) noexcept { simd = _mm_add_ps(simd, b.simd); return *this; }
	vec4f& operator-=(const vec4f& b) noexcept { simd = _mm_sub_ps(simd, b.simd); return *this; }
	vec4f& operator*=(const vec4f& b) noexcept { simd = _mm_mul_ps(simd, b.simd); return *this; }
	vec4f& operator/=(const vec4f& b) noexcept { simd = _mm_div_ps(simd, b.simd); return *this; }
	vec4f& operator%=(const vec4f& b) noexcept { simd = _mm_fmod_ps(simd, b.simd); return *this; }
	vec4f& operator+=(const float b) noexcept { simd = _mm_add_ps(simd, _mm_set1_ps(b)); return *this; }
	vec4f& operator-=(const float b) noexcept { simd = _mm_sub_ps(simd, _mm_set1_ps(b)); return *this; }
	vec4f& operator*=(const float b) noexcept { simd = _mm_mul_ps(simd, _mm_set1_ps(b)); return *this; }
	vec4f& operator/=(const float b) noexcept { simd = _mm_div_ps(simd, _mm_set1_ps(b)); return *this; }
	vec4f& operator%=(const float b) noexcept { simd = _mm_fmod_ps(simd, _mm_set1_ps(b)); return *this; }

	[[nodiscard]] vec4f operator-() const { return VECTOR_FORWARD(_mm_mul_ps(simd, _mm_set1_ps(-1.0f))); }
	[[nodiscard]] vec4f operator~() const { return VECTOR_FORWARD(_mm_xor_ps(simd, reinterpret_cast<const __m128&>(SIMD_4i_BITMASK))); }

	// Conversions
	[[nodiscard]] operator vec2f&() noexcept { return reinterpret_cast<vec2f&>(*this); }
	[[nodiscard]] operator const vec2f& () const noexcept { return reinterpret_cast<const vec2f&>(*this); }
	[[nodiscard]] operator vec4i() const;
};

// Arithmetic
[[nodiscard]] static vec4f operator+(const vec4f& a, const vec4f& b) noexcept { return VECTOR_FORWARD( _mm_add_ps(a.simd, b.simd)); }
[[nodiscard]] static vec4f operator-(const vec4f& a, const vec4f& b) noexcept { return VECTOR_FORWARD( _mm_sub_ps(a.simd, b.simd)); }
[[nodiscard]] static vec4f operator*(const vec4f& a, const vec4f& b) noexcept { return VECTOR_FORWARD( _mm_mul_ps(a.simd, b.simd)); }
[[nodiscard]] static vec4f operator/(const vec4f& a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_div_ps(a.simd, b.simd)); }
[[nodiscard]] static vec4f operator%(const vec4f& a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_fmod_ps(a.simd, b.simd)); }

[[nodiscard]] static vec4f operator+(const vec4f& a, const float b) noexcept { return VECTOR_FORWARD(_mm_add_ps(a.simd, _mm_set1_ps(b))); }
[[nodiscard]] static vec4f operator-(const vec4f& a, const float b) noexcept { return VECTOR_FORWARD(_mm_sub_ps(a.simd, _mm_set1_ps(b))); }
[[nodiscard]] static vec4f operator*(const vec4f& a, const float b) noexcept { return VECTOR_FORWARD(_mm_mul_ps(a.simd, _mm_set1_ps(b))); }
[[nodiscard]] static vec4f operator/(const vec4f& a, const float b) noexcept { return VECTOR_FORWARD(_mm_div_ps(a.simd, _mm_set1_ps(b))); }
[[nodiscard]] static vec4f operator%(const vec4f& a, const float b) noexcept { return VECTOR_FORWARD(_mm_fmod_ps(a.simd, _mm_set1_ps(b))); }

[[nodiscard]] static vec4f operator+(const float a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_add_ps(_mm_set1_ps(a), b.simd)); }
[[nodiscard]] static vec4f operator-(const float a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_sub_ps(_mm_set1_ps(a), b.simd)); }
[[nodiscard]] static vec4f operator*(const float a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_mul_ps(_mm_set1_ps(a), b.simd)); }
[[nodiscard]] static vec4f operator/(const float a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_div_ps(_mm_set1_ps(a), b.simd)); }
[[nodiscard]] static vec4f operator%(const float a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_fmod_ps(_mm_set1_ps(a), b.simd)); }

// Comparators
[[nodiscard]] static vec4f operator>=(const vec4f& a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_cmpge_ps(a.simd, b.simd)); }
[[nodiscard]] static vec4f operator<=(const vec4f& a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_cmple_ps(a.simd, b.simd)); }
[[nodiscard]] static vec4f operator>(const vec4f& a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_cmpgt_ps(a.simd, b.simd)); }
[[nodiscard]] static vec4f operator<(const vec4f& a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_cmplt_ps(a.simd, b.simd)); }
[[nodiscard]] static vec4f operator&&(const vec4f& a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_and_ps(a.simd, b.simd)); }
[[nodiscard]] static vec4f operator||(const vec4f& a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_or_ps(a.simd, b.simd)); }

[[nodiscard]] static vec4f operator>=(const vec4f& a, const float b) noexcept { return VECTOR_FORWARD(_mm_cmpge_ps(a.simd, _mm_set1_ps(b))); }
[[nodiscard]] static vec4f operator<=(const vec4f& a, const float b) noexcept { return VECTOR_FORWARD(_mm_cmple_ps(a.simd, _mm_set1_ps(b))); }
[[nodiscard]] static vec4f operator>(const vec4f& a, const float b) noexcept { return VECTOR_FORWARD(_mm_cmpgt_ps(a.simd, _mm_set1_ps(b))); }
[[nodiscard]] static vec4f operator<(const vec4f& a, const float b) noexcept { return VECTOR_FORWARD(_mm_cmplt_ps(a.simd, _mm_set1_ps(b))); }

[[nodiscard]] static vec4f operator>=(const float a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_cmpge_ps(_mm_set1_ps(a), b.simd)); }
[[nodiscard]] static vec4f operator<=(const float a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_cmple_ps(_mm_set1_ps(a), b.simd)); }
[[nodiscard]] static vec4f operator>(const float a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_cmpgt_ps(_mm_set1_ps(a), b.simd)); }
[[nodiscard]] static vec4f operator<(const float a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_cmplt_ps(_mm_set1_ps(a), b.simd)); }

// Logical
[[nodiscard]] static vec4f operator&(const vec4f& a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_and_ps(a.simd, b.simd)); }
[[nodiscard]] static vec4f operator|(const vec4f& a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_or_ps(a.simd, b.simd)); }
[[nodiscard]] static vec4f operator^(const vec4f& a, const vec4f& b) noexcept { return VECTOR_FORWARD(_mm_xor_ps(a.simd, b.simd)); }

#endif