#pragma once
#ifndef VF_EULER_HPP
#define VF_EULER_HPP

#include "vec.hpp"

#define VECTOR_FORWARD(x) { x }

union euler
{
	using ElementType = float;

	__m128 simd;
	float v[4];
	struct { float rx, ry, rz, r0; };
	struct { float yaw, pitch, roll; };

	// Operators
	constexpr float operator[](const unsigned char i) const { return v[i]; }
	constexpr float& operator[](const unsigned char i) { return v[i]; }
	euler& operator+=(const euler& b) noexcept { simd = _mm_add_ps(simd, b.simd); return *this; }
	euler& operator-=(const euler& b) noexcept { simd = _mm_sub_ps(simd, b.simd); return *this; }
	euler& operator*=(const euler& b) noexcept { simd = _mm_mul_ps(simd, b.simd); return *this; }
	euler& operator/=(const euler& b) noexcept { simd = _mm_div_ps(simd, b.simd); return *this; }
	euler& operator%=(const euler& b) noexcept { simd = _mm_fmod_ps(simd, b.simd); return *this; }
	euler& operator+=(const float b) noexcept { simd = _mm_add_ps(simd, _mm_set1_ps(b)); return *this; }
	euler& operator-=(const float b) noexcept { simd = _mm_sub_ps(simd, _mm_set1_ps(b)); return *this; }
	euler& operator*=(const float b) noexcept { simd = _mm_mul_ps(simd, _mm_set1_ps(b)); return *this; }
	euler& operator/=(const float b) noexcept { simd = _mm_div_ps(simd, _mm_set1_ps(b)); return *this; }
	euler& operator%=(const float b) noexcept { simd = _mm_fmod_ps(simd, _mm_set1_ps(b)); return *this; }

	[[nodiscard]] euler operator-() const { return VECTOR_FORWARD(_mm_mul_ps(simd, _mm_set1_ps(-1.0f))); }
};

// Euler Arithmetic
[[nodiscard]] static euler operator+(const euler& a, const euler& b) noexcept { return VECTOR_FORWARD(_mm_add_ps(a.simd, b.simd)); }
[[nodiscard]] static euler operator-(const euler& a, const euler& b) noexcept { return VECTOR_FORWARD(_mm_sub_ps(a.simd, b.simd)); }
[[nodiscard]] static euler operator*(const euler& a, const euler& b) noexcept { return VECTOR_FORWARD(_mm_mul_ps(a.simd, b.simd)); }
[[nodiscard]] static euler operator/(const euler& a, const euler& b) noexcept { return VECTOR_FORWARD(_mm_div_ps(a.simd, b.simd)); }
[[nodiscard]] static euler operator%(const euler& a, const euler& b) noexcept { return VECTOR_FORWARD(_mm_fmod_ps(a.simd, b.simd)); }

[[nodiscard]] static euler operator+(const euler& a, const float b) noexcept { return VECTOR_FORWARD(_mm_add_ps(a.simd, _mm_set1_ps(b))); }
[[nodiscard]] static euler operator-(const euler& a, const float b) noexcept { return VECTOR_FORWARD(_mm_sub_ps(a.simd, _mm_set1_ps(b))); }
[[nodiscard]] static euler operator*(const euler& a, const float b) noexcept { return VECTOR_FORWARD(_mm_mul_ps(a.simd, _mm_set1_ps(b))); }
[[nodiscard]] static euler operator/(const euler& a, const float b) noexcept { return VECTOR_FORWARD(_mm_div_ps(a.simd, _mm_set1_ps(b))); }
[[nodiscard]] static euler operator%(const euler& a, const float b) noexcept { return VECTOR_FORWARD(_mm_fmod_ps(a.simd, _mm_set1_ps(b))); }

[[nodiscard]] static euler operator+(const float a, const euler& b) noexcept { return VECTOR_FORWARD(_mm_add_ps(_mm_set1_ps(a), b.simd)); }
[[nodiscard]] static euler operator-(const float a, const euler& b) noexcept { return VECTOR_FORWARD(_mm_sub_ps(_mm_set1_ps(a), b.simd)); }
[[nodiscard]] static euler operator*(const float a, const euler& b) noexcept { return VECTOR_FORWARD(_mm_mul_ps(_mm_set1_ps(a), b.simd)); }
[[nodiscard]] static euler operator/(const float a, const euler& b) noexcept { return VECTOR_FORWARD(_mm_div_ps(_mm_set1_ps(a), b.simd)); }
[[nodiscard]] static euler operator%(const float a, const euler& b) noexcept { return VECTOR_FORWARD(_mm_fmod_ps(_mm_set1_ps(a), b.simd)); }

#endif