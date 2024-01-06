#pragma once
#ifndef VF_MATH_OPERATIONS_H
#define VF_MATH_OPERATIONS_H

#include "format.hpp"
#include "MatrixMath.hpp"

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
	[[nodiscard]] static float execute( ScalarOperation op, float source, const float operand )
	{
		using enum ScalarOperation;
		switch( op )
		{
		default:
		case KEEP:	return source;
		case SET:	return operand;

		case ADD:	return source + operand;
		case SUB:	return source - operand;
		case MUL:	return source * operand;
		case DIV:   return source / operand;
		case MOD:	return fmodf( source, operand );
		}
	}

	[[nodiscard]] static vec4f execute( VectorOperation op, const vec4f& source, const vec4f& operand )
	{
		using enum VectorOperation;
		switch( op )
		{
		default:
		case KEEP:	return source;
		case SET:	return operand;

		case ADD:	return source + operand;
		case SUB:	return source - operand;
		case MUL:	return source * operand;
		case DIV:   return source / operand;
		case MOD:   return source % operand;

		case PROJECT:	 return Math::project( source, operand );
		case REFLECT:	 return Math::reflect( source, Math::normalize( operand ) );
		case ORTHOGONAL: return source - Math::project( source, operand );

		case SET_PARALLEL:	return ( source - Math::project( source, operand ) ) + operand;
		}
	}
}

namespace vf
{
#define TO_STRING_CASE( x ) case x: return #x##s

	constexpr std::string toString( ScalarOperation op )
	{
		using namespace std::literals::string_literals;
		using enum ScalarOperation;
		switch( op )
		{
			default:	return "UNKNOWN"s;
			TO_STRING_CASE( KEEP );
			TO_STRING_CASE( SET );
			TO_STRING_CASE( ADD );
			TO_STRING_CASE( SUB );
			TO_STRING_CASE( MUL );
			TO_STRING_CASE( DIV );
			TO_STRING_CASE( MOD );
		}
	}

	constexpr std::string toString( VectorOperation op )
	{
		using namespace std::literals::string_literals;
		using enum VectorOperation;
		switch( op )
		{
			default:	return "UNKNOWN"s;
			TO_STRING_CASE( KEEP );
			TO_STRING_CASE( SET );
			TO_STRING_CASE( ADD );
			TO_STRING_CASE( SUB );
			TO_STRING_CASE( MUL );
			TO_STRING_CASE( DIV );
			TO_STRING_CASE( MOD );

			TO_STRING_CASE( PROJECT );
			TO_STRING_CASE( REFLECT );
			TO_STRING_CASE( ORTHOGONAL );
			TO_STRING_CASE( SET_PARALLEL );
		}
	}
}

#endif