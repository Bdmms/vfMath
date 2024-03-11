#include "../include/vfMath/Operations.hpp"
#include "../include/vfMath/MatrixMath.hpp"

float Math::execute( ScalarOperation op, float source, const float operand )
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

vec4f Math::execute( VectorOperation op, const vec4f& source, const vec4f& operand )
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