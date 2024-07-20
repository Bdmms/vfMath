#pragma once
#ifndef VF_TRANSFORM_3D_HPP
#define VF_TRANSFORM_3D_HPP

#include "MatrixMath.hpp"

/**
 * @brief Pair of transform matrices that define transformations into and out of a space.
*/
struct TransformSpace
{
	mat4f transform;
	mat4f inverse;

	TransformSpace( const mat4f& transform ) : transform( transform ), inverse( transform.inverse() ) {}

	constexpr TransformSpace() : transform( Math::IDENTITY<mat4f> ), inverse( Math::IDENTITY<mat4f> ) {}
	constexpr TransformSpace( const mat4f& transform, const mat4f& inverse ) : transform( transform ), inverse( inverse ) {}
	constexpr TransformSpace( const TransformSpace& copy ) : transform( copy.transform ), inverse( copy.inverse ) {}
};

[[nodiscard]] static TransformSpace operator*( const TransformSpace& a, const TransformSpace& b ) noexcept
{
	return { a.transform * b.transform, b.inverse * a.inverse };
}

/**
 * @brief Stores a decomposed 3D affine transformation.
*/
struct Transform3D
{
	vec3f translation;
	quat  rotation;
	vec3f scale;

	constexpr Transform3D( const vec3f& translation = Math::ZERO<vec4f>, const quat& rotation = Math::IDENTITY<quat>, const vec3f& scale = Math::ONES<vec3f> ) :
		translation( translation ),
		rotation( rotation ),
		scale( scale )
	{

	}

	Transform3D( const vec3f& translation, const euler& rotation, const vec3f& scale = Math::ONES<vec3f> ) :
		translation( translation ),
		rotation( Math::toQuat( rotation ) ),
		scale( scale )
	{

	}

	Transform3D( const vec3f& translation, const AxisAngle& rotation, const vec3f& scale = Math::ONES<vec3f> ) :
		translation( translation ),
		rotation( Math::toQuat( rotation ) ),
		scale( scale )
	{

	}

	[[nodiscard]] Transform3D inverse() const
	{
		quat invRotation = rotation.inverse();
		vec3f invScale = 1.0f / scale;

		return { Math::rotate( -translation, invRotation ) * invScale, invRotation, invScale };
	}
};

[[nodiscard]] static vec4f operator*( const Transform3D& a, const vec4f& b ) noexcept
{
	return Math::rotate( b * a.scale, a.rotation ) + a.translation;
}

[[nodiscard]] static Transform3D operator*( const Transform3D& a, const Transform3D& b ) noexcept
{
	return 
	{
		a.translation + ( a * b.translation ),
		a.rotation * b.rotation,
		a.scale * b.scale,
	};
}

namespace Math
{
	template<> inline constexpr static TransformSpace IDENTITY<TransformSpace> = { IDENTITY<mat4f>, IDENTITY<mat4f> };
	template<> inline constexpr static Transform3D    IDENTITY<Transform3D>;

	/**
	 * @brief Translates the regular and inverse transforms of the space.
	 * @param space - transform space
	 * @param translation - translation vector
	*/
	static void translate( TransformSpace& space, const vec3f& translation )
	{
		space.transform.origin += translation;
		space.inverse.origin -= space.inverse * translation;
	}
}

#endif
