#pragma once
#ifndef VF_VEC_2D_INT_HPP
#define VF_VEC_2D_INT_HPP

#include "vec.hpp"

/**
 * @brief 2D signed-integer vector
*/
union vec2i
{
	using ElementType = int;

	int v[2];
	__m64 simd;
	struct { int x, y; };
	struct { unsigned int u_x, u_y; };

	// Array Operators
	constexpr int operator[](const unsigned char i) const { return v[i]; }
	constexpr int& operator[](const unsigned char i) { return v[i]; }

	// Arithmetic Operators
	constexpr vec2i& operator+=(const vec2i& b) noexcept { x += b.x; y += b.y; return *this; }
	constexpr vec2i& operator-=(const vec2i& b) noexcept { x -= b.x; y -= b.y; return *this; }
	constexpr vec2i& operator*=(const vec2i& b) noexcept { x *= b.x; y *= b.y; return *this; }
	constexpr vec2i& operator/=(const vec2i& b) noexcept { x /= b.x; y /= b.y; return *this; }
	constexpr vec2i& operator*=(const int b) noexcept { x *= b; y *= b; return *this; }
	constexpr vec2i& operator/=(const int b) noexcept { x /= b; y /= b; return *this; }

	[[nodiscard]] constexpr vec2i operator-() { return vec2i{ -x, -y }; }

	// Logical Operators
	constexpr vec2i& operator&=(const vec2i& b) noexcept { x &= b.x; y &= b.y; return *this; }
	constexpr vec2i& operator|=(const vec2i& b) noexcept { x |= b.x; y |= b.y; return *this; }
	constexpr vec2i& operator^=(const vec2i& b) noexcept { x ^= b.x; y ^= b.y; return *this; }

	// Conversions
	[[nodiscard]] operator vec4i() const;
	[[nodiscard]] operator vec2f() const;
};

// Global Arithmetic Operators
[[nodiscard]] constexpr vec2i operator+(const vec2i& a, const vec2i& b) noexcept { return { a.x + b.x, a.y + b.y }; }
[[nodiscard]] constexpr vec2i operator-(const vec2i& a, const vec2i& b) noexcept { return { a.x - b.x, a.y - b.y }; }
[[nodiscard]] constexpr vec2i operator*(const vec2i& a, const vec2i& b) noexcept { return { a.x * b.x, a.y * b.y }; }
[[nodiscard]] constexpr vec2i operator/(const vec2i& a, const vec2i& b) noexcept { return { a.x / b.x, a.y / b.y }; }
[[nodiscard]] constexpr vec2i operator*(const vec2i& a, const int b) noexcept { return { a.x * b, a.y * b }; }
[[nodiscard]] constexpr vec2i operator/(const vec2i& a, const int b) noexcept { return { a.x / b, a.y / b }; }
[[nodiscard]] constexpr vec2i operator*(const int a, const vec2i& b) noexcept { return { a * b.x, a * b.y }; }

// Global Logical Operators
[[nodiscard]] constexpr vec2i operator&(const vec2i& a, const vec2i& b) noexcept { return { a.x & b.x, a.y & b.y }; }
[[nodiscard]] constexpr vec2i operator|(const vec2i& a, const vec2i& b) noexcept { return { a.x | b.x, a.y | b.y }; }
[[nodiscard]] constexpr vec2i operator^(const vec2i& a, const vec2i& b) noexcept { return { a.x ^ b.x, a.y ^ b.y }; }

#ifdef WB_DEBUG_CONSOLE_FLAG
static std::ostream& operator<<(std::ostream& os, const vec2i& vector)
{
	os << vector.x << ", " << vector.y << ", " << "\n";
	return os;
}
#endif

#endif