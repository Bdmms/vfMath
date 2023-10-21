#pragma once
#ifndef VF_TRANSFORM_2D_HPP
#define VF_TRANSFORM_2D_HPP

#include "MatrixMath.hpp"

struct Transform2D
{
	mat2f rotScale;
	vec2f position;

	constexpr Transform2D() :
		rotScale( Math::IDENTITY<mat2f> ),
		position( Math::ZERO<vec2f> )
	{

	}

	constexpr Transform2D( const mat2f& rotScale, vec2f translation ) :
		rotScale{ rotScale },
		position( translation )
	{

	}

	constexpr Transform2D( vec2f xAxis, vec2f yAxis, vec2f position ) :
		rotScale{ xAxis, yAxis },
		position( position )
	{

	}

	constexpr Transform2D( vec2f position, vec2f scale ) :
		rotScale( Math::create::scale( scale ) ),
		position( position )
	{

	}

	Transform2D( vec2f position, float rotation, vec2f scale ) :
		rotScale( Math::create::transform( rotation, scale ) ),
		position( position )
	{

	}

	Transform2D inverse() const
	{
		mat2f inv = rotScale.inverse();
		return { inv, inv * -position };
	}
};

[[nodiscard]] static vec2f operator*( const Transform2D& a, const vec2f b ) noexcept
{
	return a.rotScale * b + a.position;
}

[[nodiscard]] static Transform2D operator*( const Transform2D& a, const Transform2D& b ) noexcept
{
	return { a.rotScale * b.rotScale, a.rotScale * b.position + a.position };
}

#endif