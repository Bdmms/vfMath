#pragma once
#ifndef VF_VEC_3D_FLOAT_HPP
#define VF_VEC_3D_FLOAT_HPP

#include "vec2f.hpp"

#define SIMD_3d(x) _mm_load_ps(x.v)
#define p_SIMD_3d(v) _mm_xyzw_ps(v->x, v->y, v->z, 0.0f)

/**
 * @brief 3D floating-point vector
*/
union vec3f_t
{
	using ElementType = float;

	float v[3];
	struct { float x, y, z; };
	struct { float r, g, b; };

	constexpr vec3f_t(const float x, const float y, const float z) : x(x), y(y), z(z) {}
	constexpr vec3f_t(const __m128& simd) : x(simd.m128_f32[0]), y(simd.m128_f32[1]), z(simd.m128_f32[2]) {}

	// Operators
	constexpr float operator[](const unsigned char i) const { return v[i]; }
	constexpr float& operator[](const unsigned char i) { return v[i]; }

	/*
	vec3f& operator+=(const vec3f& b) noexcept { simd = _mm_add_ps(p_SIMD_3d(this), SIMD_3d(b)); return *this; }
	vec3f& operator-=(const vec3f& b) noexcept { simd = _mm_sub_ps(p_SIMD_3d(this), SIMD_3d(b)); return *this; }
	vec3f& operator*=(const vec3f& b) noexcept { simd = _mm_mul_ps(p_SIMD_3d(this), SIMD_3d(b)); return *this; }
	vec3f& operator/=(const vec3f& b) noexcept { simd = _mm_div_ps(p_SIMD_3d(this), SIMD_3d(b)); return *this; }
	vec3f& operator%=(const vec3f& b) noexcept { simd = _mm_fmod_ps(p_SIMD_3d(this), SIMD_3d(b)); return *this; }
	vec3f& operator*=(const float b) noexcept { simd = _mm_mul_ps(p_SIMD_3d(this), _mm_set1_ps(b)); return *this; }
	vec3f& operator/=(const float b) noexcept { simd = _mm_div_ps(p_SIMD_3d(this), _mm_set1_ps(b)); return *this; }*/
};

[[nodiscard]] static vec3f_t operator+(const vec3f_t& a, const vec3f_t& b) noexcept 
{ 
	__m128 simd = _mm_sub_ps(SIMD_3d(a), SIMD_3d(b));
	return reinterpret_cast<vec3f_t&>(simd);
}

[[nodiscard]] static vec3f_t operator-(const vec3f_t& a, const vec3f_t& b) noexcept { return { _mm_sub_ps(SIMD_3d(a), SIMD_3d(b)) }; }
[[nodiscard]] static vec3f_t operator*(const vec3f_t& a, const vec3f_t& b) noexcept { return { _mm_mul_ps(SIMD_3d(a), SIMD_3d(b)) }; }
[[nodiscard]] static vec3f_t operator/(const vec3f_t& a, const vec3f_t& b) noexcept { return { _mm_div_ps(SIMD_3d(a), SIMD_3d(b)) }; }

#endif