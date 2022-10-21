#pragma once
#ifndef VF_TRANSFORM_HPP
#define VF_TRANSFORM_HPP

#include "MatrixMath.hpp"

struct Transform
{
	virtual vec4f operator*(const vec4f& vector) = 0;
	virtual vec4f operator^(const vec4f& vector) = 0;
};

class LocQuatTransform : public Transform
{
	quat rotation = Math::IDENTITY<quat>;
	vec3f translation = Math::ZERO<vec3f>;

public:
	virtual vec4f operator*(const vec4f& vector) override
	{
		return Math::rotate( vector, rotation ) + translation * vector.w;
	}

	virtual vec4f operator^(const vec4f& vector) override
	{
		return Math::rotate( vector - translation * vector.w, rotation.inverse() );
	}
};

class QuatTransform : public Transform
{
	quat rotation = Math::IDENTITY<quat>;
	vec3f translation = Math::ZERO<vec3f>;
	vec3f scale = Math::ONES<vec3f>;

public:
	virtual vec4f operator*(const vec4f& vector) override
	{
		return Math::rotate( vector * scale, rotation ) + translation * vector.w;
	}

	virtual vec4f operator^(const vec4f& vector) override
	{
		return Math::rotate( vector - translation * vector.w, rotation.inverse() ) / scale;
	}
};

#endif