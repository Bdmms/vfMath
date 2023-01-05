#pragma once
#ifndef VF_MATH_IOSTREAM_HPP
#define VF_MATH_IOSTREAM_HPP

#include "vec4f.hpp"
#include "vec4i.hpp"

#include "quaternion.hpp"

#include "mat2x2.hpp"
#include "mat4x4.hpp"

#include <iostream>
#include <format>

static std::ostream& operator<<(std::ostream& os, const vec2i& vector)
{
	return os << std::format("{}, {}", vector.x, vector.y);
}

static std::ostream& operator<<(std::ostream& os, const vec2f& vector)
{
	return os << std::format("{}, {}", vector.x, vector.y);
}

static std::ostream& operator<<(std::ostream& os, const euler& euler)
{
	return os << std::format("{}, {}, {}", euler.rx, euler.ry, euler.rz);
}

static std::ostream& operator<<(std::ostream& os, const vec4f& vector)
{
	return os << std::format("{}, {}, {}, {}", vector.x, vector.y, vector.z, vector.w);
}

static std::ostream& operator<<(std::ostream& os, const vec4i& vector)
{
	return os << std::format("{}, {}, {}, {}", vector.x, vector.y, vector.z, vector.w);
}

static std::ostream& operator<<(std::ostream& os, const quat& q)
{
	return os << std::format("{}, {}, {}, {}", q.x, q.y, q.z, q.w);
}

static std::ostream& operator<<(std::ostream& os, const mat2x2& matrix)
{
	os << matrix[0] << ", " << matrix[2] << "\n";
	os << matrix[1] << ", " << matrix[3] << "\n";
	return os;
}

static std::ostream& operator<<(std::ostream& os, const mat4x4& matrix)
{
	os << matrix[0] << ", " << matrix[4] << ", " << matrix[8] << ", " << matrix[12] << "\n";
	os << matrix[1] << ", " << matrix[5] << ", " << matrix[9] << ", " << matrix[13] << "\n";
	os << matrix[2] << ", " << matrix[6] << ", " << matrix[10] << ", " << matrix[14] << "\n";
	os << matrix[3] << ", " << matrix[7] << ", " << matrix[11] << ", " << matrix[15] << "\n";
	return os;
}

#endif