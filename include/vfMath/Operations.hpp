#pragma once
#ifndef VF_MATH_OPERATIONS_H
#define VF_MATH_OPERATIONS_H

#include "vec4f.hpp"

enum class ScalarOperation : uint8_t
{
	// Assignments
	KEEP,
	SET,

	// Common Arithemetic
	ADD,
	SUB,
	MUL,
	DIV,
	MOD,
};

enum class VectorOperation : uint8_t
{
	// Assignments
	KEEP = uint8_t( ScalarOperation::KEEP ),
	SET = uint8_t( ScalarOperation::SET ),

	// Common Arithemetic
	ADD = uint8_t( ScalarOperation::ADD ),
	SUB = uint8_t( ScalarOperation::SUB ),
	MUL = uint8_t( ScalarOperation::MUL ),
	DIV = uint8_t( ScalarOperation::DIV ),
	MOD = uint8_t( ScalarOperation::MOD ),

	// Vector Exclusive
	PROJECT,
	REFLECT,
	ORTHOGONAL,

	// Special
	SET_PARALLEL,
};

namespace Math
{
	[[nodiscard]] float execute( ScalarOperation op, float source, const float operand );
	[[nodiscard]] vec4f execute( VectorOperation op, const vec4f& source, const vec4f& operand );
}

#endif