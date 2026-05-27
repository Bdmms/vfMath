#pragma once
#ifndef VF_GEOMETRY_HPP
#define VF_GEOMETRY_HPP

#include "transform3D.hpp"

namespace vf::geometry
{
	using namespace Math;

	// Custom epsilon value for single-prevision floating point values
	constexpr float P_EPSILON = 2E-5f;
	constexpr float N_EPSILON = -P_EPSILON;
	constexpr float ONE_EPSILON = 1.0f + P_EPSILON;

	namespace process
	{
		/**
		 * @brief Returns the index of the vector's component with the largest value.
		 * @param vector - 3D vector
		 * @return component index
		*/
		constexpr static uint8_t maxIndex( const vec4f& vector )
		{
			if( vector.x >= vector.y )	return vector.z >= vector.x ? 2u : 0u;
			else						return vector.z >= vector.y ? 2u : 1u;
		}

		/**
		 * @brief Returns the index of the vector's component with the smallest value.
		 * @param vector - 3D vector
		 * @return component index
		*/
		constexpr static uint8_t minIndex( const vec4f& vector )
		{
			if( vector.x <= vector.y )	return vector.z <= vector.x ? 2u : 0u;
			else						return vector.z <= vector.y ? 2u : 1u;
		}

		static void AABB_TRIANGLE( const mat4f& t, vec4f& min, vec4f& max )
		{
			min.simd = _mm_min_ps( t.origin.simd, _mm_min_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), _mm_add_ps( t.origin.simd, t.y_axis.simd ) ) );
			max.simd = _mm_max_ps( t.origin.simd, _mm_max_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), _mm_add_ps( t.origin.simd, t.y_axis.simd ) ) );
		}

		static void AABB_BOX_FULL( const mat4f& t, vec4f& min, vec4f& max )
		{
			__m128 point = _mm_add_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min.simd = point;
			max.simd = point;

			point = _mm_add_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min.simd = _mm_min_ps( min.simd, point );
			max.simd = _mm_max_ps( max.simd, point );

			point = _mm_add_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min.simd = _mm_min_ps( min.simd, point );
			max.simd = _mm_max_ps( max.simd, point );

			point = _mm_add_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min.simd = _mm_min_ps( min.simd, point );
			max.simd = _mm_max_ps( max.simd, point );

			point = _mm_sub_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min.simd = _mm_min_ps( min.simd, point );
			max.simd = _mm_max_ps( max.simd, point );

			point = _mm_sub_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min.simd = _mm_min_ps( min.simd, point );
			max.simd = _mm_max_ps( max.simd, point );

			point = _mm_sub_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min.simd = _mm_min_ps( min.simd, point );
			max.simd = _mm_max_ps( max.simd, point );

			point = _mm_sub_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min.simd = _mm_min_ps( min.simd, point );
			max.simd = _mm_max_ps( max.simd, point );
		}

		static void AABB_BOX_HALF( const mat4f& t, __m128& min )
		{
			__m128 point__ = _mm_add_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min = point__;

			point__ = _mm_add_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min = _mm_min_ps( min, point__ );

			point__ = _mm_add_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min = _mm_min_ps( min, point__ );

			point__ = _mm_sub_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min = _mm_min_ps( min, point__ );

			point__ = _mm_sub_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min = _mm_min_ps( min, point__ );

			point__ = _mm_sub_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min = _mm_min_ps( min, point__ );

			point__ = _mm_sub_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
			min = _mm_min_ps( min, point__ );
		}

		static vec4f MIRRORED_AABB_SEPARATION_TEST( const mat4f& t, const vec4f& max )
		{
			// Create a sign vector to select the corner of the box
			vec4f quadrant = Math::sign( t.origin );

			vec4f bounds( Math::MAX<float>, Math::MAX<float>, Math::MAX<float>, Math::MIN<float> );
			vf::simd::min( bounds, bounds, quadrant * ( t.origin + t.x_axis + t.y_axis + t.z_axis ) );
			vf::simd::min( bounds, bounds, quadrant * ( t.origin + t.x_axis + t.y_axis - t.z_axis ) );
			vf::simd::min( bounds, bounds, quadrant * ( t.origin + t.x_axis - t.y_axis + t.z_axis ) );
			vf::simd::min( bounds, bounds, quadrant * ( t.origin + t.x_axis - t.y_axis - t.z_axis ) );
			vf::simd::min( bounds, bounds, quadrant * ( t.origin - t.x_axis + t.y_axis + t.z_axis ) );
			vf::simd::min( bounds, bounds, quadrant * ( t.origin - t.x_axis + t.y_axis - t.z_axis ) );
			vf::simd::min( bounds, bounds, quadrant * ( t.origin - t.x_axis - t.y_axis + t.z_axis ) );
			vf::simd::min( bounds, bounds, quadrant * ( t.origin - t.x_axis - t.y_axis - t.z_axis ) );

			return bounds <= max;
		}

		constexpr static vec4f PROJECTED_AABB_SEPARATION_TEST( const mat4f& t, const vec4f& min, const vec4f& max )
		{
			vec4f point = t.origin + t.x_axis + t.y_axis + t.z_axis;
			vec4f scale = vec4f::set1( point.w );
			vf::simd::div( point, point, scale );
			vec4f minBounds = point;
			vec4f maxBounds = point;

			point = t.origin + t.x_axis + t.y_axis - t.z_axis;
			vf::simd::set1( scale, point.w );
			vf::simd::div( point, point, scale );
			vf::simd::min( minBounds, minBounds, point );
			vf::simd::max( maxBounds, maxBounds, point );

			point = t.origin + t.x_axis - t.y_axis + t.z_axis;
			vf::simd::set1( scale, point.w );
			vf::simd::div( point, point, scale );
			vf::simd::min( minBounds, minBounds, point );
			vf::simd::max( maxBounds, maxBounds, point );

			point = t.origin + t.x_axis - t.y_axis - t.z_axis;
			vf::simd::set1( scale, point.w );
			vf::simd::div( point, point, scale );
			vf::simd::min( minBounds, minBounds, point );
			vf::simd::max( maxBounds, maxBounds, point );

			point = t.origin - t.x_axis + t.y_axis + t.z_axis;
			vf::simd::set1( scale, point.w );
			vf::simd::div( point, point, scale );
			vf::simd::min( minBounds, minBounds, point );
			vf::simd::max( maxBounds, maxBounds, point );

			point = t.origin - t.x_axis + t.y_axis - t.z_axis;
			vf::simd::set1( scale, point.w );
			vf::simd::div( point, point, scale );
			vf::simd::min( minBounds, minBounds, point );
			vf::simd::max( maxBounds, maxBounds, point );

			point = t.origin - t.x_axis - t.y_axis + t.z_axis;
			vf::simd::set1( scale, point.w );
			vf::simd::div( point, point, scale );
			vf::simd::min( minBounds, minBounds, point );
			vf::simd::max( maxBounds, maxBounds, point );

			point = t.origin - t.x_axis - t.y_axis - t.z_axis;
			vf::simd::set1( scale, point.w );
			vf::simd::div( point, point, scale );
			vf::simd::min( minBounds, minBounds, point );
			vf::simd::max( maxBounds, maxBounds, point );

			return Math::overlaps( min, max, minBounds, maxBounds );
		}
	}

	namespace aabb
	{
		/**
		 * @brief Tests if the finite ray intersects with the axis-aligned bounding box.
		 * @param aabb - axis-aligned bounding box
		 * @param rayOrigin - origin point of the ray
		 * @param rayVector - displacement vector to the end of the ray
		 * @return Whether the ray intersects the unit cube
		*/
		constexpr static bool rayTest( const vec4f& min, const vec4f& max, const vec4f& rayOrigin, const vec4f& rayVector )
		{
			vec4f intersect0 = ( min - rayOrigin ) / rayVector;
			vec4f intersect1 = ( max - rayOrigin ) / rayVector;
			vec4f isZero = rayVector == Math::ZERO<vec4f>;

			vec4f tmin = condition<4, f32>( isZero, Math::ZERO<vec4f>, Math::min<4, f32>( intersect0, intersect1 ) );
			vec4f tmax = condition<4, f32>( isZero, Math::ONES<vec4f>, Math::max<4, f32>( intersect0, intersect1 ) );

			float minBounds = std::max( std::max( tmin.x, tmin.y ), std::max( tmin.z, 0.0f ) );
			float maxBounds = std::min( std::min( tmax.x, tmax.y ), std::min( tmax.z, 1.0f ) );
			return minBounds <= maxBounds;
		}

		/**
		 * @brief Calculates the entry and exit distances where the ray intersects with the axis-aligned bounding box.
		 * @param aabb - axis-aligned bounding box
		 * @param rayOrigin - origin point of the ray
		 * @param rayVector - displacement vector to the end of the ray
		 * @return minimum and maximum values along the ray vector
		*/
		constexpr static Bounds<float> rayBounds( const vec4f& min, const vec4f& max, const vec4f& rayOrigin, const vec4f& rayVector )
		{
			vec4f intersect0 = ( min - rayOrigin ) / rayVector;
			vec4f intersect1 = ( max - rayOrigin ) / rayVector;

			vec4f tmin = Math::condition( rayVector == Math::ZERO<vec4f>, Math::ZERO<vec4f>, Math::min( intersect0, intersect1 ) );
			vec4f tmax = Math::condition( rayVector == Math::ZERO<vec4f>, Math::ONES<vec4f>, Math::max( intersect0, intersect1 ) );

			return
			{
				std::clamp( std::max( std::max( tmin.x, tmin.y ), tmin.z ), 0.0f, 1.0f ),
				std::clamp( std::min( std::min( tmax.x, tmax.y ), tmax.z ), 0.0f, 1.0f )
			};
		}
	}

	namespace box
	{
		/**
		 * @brief Calculates the Axis Aligned Bounding Box (AABB) that fits the arbitrary box
		 * @param transform - transform of the box in the axis aligned space
		 * @return bounding vector pair that represents the AABB
		*/
		static Bounds<vec4f> calculateAABB( const mat4f& transform )
		{
			Bounds<vec4f> bounds;
			process::AABB_BOX_FULL( transform, bounds.min, bounds.max );
			return bounds;
		}

		/**
		 * @brief Tests if the unit cube contains the point.
		 * @param point - point vector
		 * @return Whether the point is inside the unit cube
		*/
		constexpr static bool pointTest( const vec4f& point )
		{
			vec4f isInside = point >= Math::NEGATIVE<vec4f> && point <= Math::ONES<vec4f>;
			return isInside.x && isInside.y && isInside.z;
		}

		/**
		 * @brief Tests if the finite ray intersects with the unit cube.
		 * @param rayOrigin - origin point of the ray
		 * @param rayVector - displacement vector to the end of the ray
		 * @return Whether the ray intersects the unit cube
		*/
		constexpr static bool rayTest( const vec4f& rayOrigin, const vec4f& rayVector )
		{
			return geometry::aabb::rayTest( Math::NEGATIVE<vec4f>, Math::ONES<vec4f>, rayOrigin, rayVector );
		}

		/**
		 * @brief Tests if the triangle intersects with the unit cube.
		 * @param triangle - transform space of triangle
		 * @return Whether the triangle intersects the unit cube
		*/
		static bool triangleTest( const TransformSpace& triangle )
		{
			// Test triangle overlapping box
			const mat4x4& tri = triangle.transform;
			vec4f min, max;

			process::AABB_TRIANGLE( tri, min, max );

			vec4f minBounds( -ONE_EPSILON, -ONE_EPSILON, -ONE_EPSILON, 0.0f );
			vec4f maxBounds( ONE_EPSILON, ONE_EPSILON, ONE_EPSILON, 0.0f );
			vec4f overlaps = Math::overlaps( min, max, minBounds, maxBounds );
			if( !( overlaps.x && overlaps.y && overlaps.z ) ) return false;

			// Test box overlapping triangle
			process::AABB_BOX_FULL( triangle.inverse, min, max );

			return max.x >= N_EPSILON && max.y >= N_EPSILON
				&& min.x + min.y <= ONE_EPSILON && min.z <= P_EPSILON && max.z >= N_EPSILON;
		}

		/**
		 * @brief Tests if the arbitrary box intersects with the unit cube.
		 * @param box - transform space of box
		 * @return Whether the box intersects the unit cube
		*/
		static bool boxTest( const TransformSpace& box )
		{
			// Test regular overlap
			vec4f overlaps = process::MIRRORED_AABB_SEPARATION_TEST( box.transform, Math::ONES<vec4f> );
			if( !( overlaps.x && overlaps.y && overlaps.z ) ) return false;

			overlaps = process::MIRRORED_AABB_SEPARATION_TEST( box.inverse, Math::ONES<vec4f> );
			return overlaps.x && overlaps.y && overlaps.z;
		}

		/**
		 * @brief Tests if the arbitrary box intersects with the unit cube.
		 * This differs from boxTest() by dividing projected points by their w coordinate.
		 * @param box - transform space of box
		 * @return Whether the box intersects the unit cube
		*/
		static bool projectedBoxTest( const TransformSpace& box )
		{
			// Test regular overlap
			vec4f overlaps = process::PROJECTED_AABB_SEPARATION_TEST( box.transform, Math::NEGATIVE<vec4f>, Math::ONES<vec4f> );
			if( !( overlaps.x && overlaps.y && overlaps.z ) ) return false;

			overlaps = process::PROJECTED_AABB_SEPARATION_TEST( box.inverse, Math::NEGATIVE<vec4f>, Math::ONES<vec4f> );
			return overlaps.x && overlaps.y && overlaps.z;
		}

		/**
		 * @brief Calculates the displacement of an arbitrary sphere intersecting with the unit cube.
		 * @param displacement - write destination of displacement vector
		 * @param sphere - transform space of sphere
		 * @return Whether the sphere intersects the unit cube
		*/
		constexpr static bool sphereCollision( vec4f& displacement, const TransformSpace& sphere )
		{
			const mat4x4& boxSpace = sphere.inverse;
			const vec4f& origin = sphere.transform.origin;

			vec4f clamped = Math::clamp( origin, Math::NEGATIVE<vec4f>, Math::ONES<vec4f> );
			vec4f quadrant = Math::sign( origin );

			vec4f rx = boxSpace.x_axis * clamped.x;
			vec4f ry = boxSpace.y_axis * clamped.y;
			vec4f rz = boxSpace.z_axis * clamped.z;

			vec4f vectorBoxSpace[3] = {
				( boxSpace.origin - Math::axis::W<vec4f> ) + ( boxSpace.x_axis * quadrant.x ) + ry + rz,
				( boxSpace.origin - Math::axis::W<vec4f> ) + ( boxSpace.y_axis * quadrant.y ) + rz + rx,
				( boxSpace.origin - Math::axis::W<vec4f> ) + ( boxSpace.z_axis * quadrant.z ) + rx + ry
			};
			vec4f lengthBoxSpace;
			vf::simd::length4( lengthBoxSpace, vectorBoxSpace[0], vectorBoxSpace[1], vectorBoxSpace[2], Math::ZERO<vec4f> );

			uint8_t idx = process::minIndex( lengthBoxSpace );
			displacement = sphere.transform * vectorBoxSpace[idx];
			float distance2 = Math::length2( displacement );
			float radius2 = Math::length2( sphere.transform * ( vectorBoxSpace[idx] / lengthBoxSpace[idx] ) );

			return distance2 <= radius2;
		}
	}

	namespace sphere
	{
		/**
		 * @brief Tests if the arbitrary sphere intersects with the unit sphere.
		 * @param sphere - transform space of sphere
		 * @return Whether the sphere intersects the unit sphere
		*/
		constexpr static bool sphereTest( const TransformSpace& sphere )
		{
			vec4f displacement = sphere.transform.origin;
			displacement.w = 0.0f;

			float distance = Math::length( displacement );
			if( distance <= Math::EPSILON<float> ) return true;

			float radius = Math::length( sphere.transform * Math::normalize( sphere.inverse * displacement ) );

			// Test if the distance is less than the radius of both spheres
			return distance <= radius + 1.0f;
		}

		/**
		 * @brief Tests if the arbitrary box intersects with the unit sphere.
		 * @param box - transform space of box
		 * @return Whether the box intersects the unit sphere
		*/
		constexpr static bool boxTest( const TransformSpace& box )
		{
			vec4f closestPoint = Math::clamp( box.inverse.origin, Math::NEGATIVE<vec4f>, Math::ONES<vec4f> );
			vec4f displacement = box.transform * closestPoint;
			float distance2 = displacement.x * displacement.x + displacement.y * displacement.y + displacement.z * displacement.z;
			return distance2 <= 1.0f;
		}

		/**
		 * @brief Calculates the displacement of an arbitrary box intersecting with the unit sphere.
		 * @param displacement - write destination of displacement vector
		 * @param sphere - transform space of box
		 * @return Whether the box intersects the unit sphere
		*/
		constexpr static bool boxCollision( vec4f& displacement, const TransformSpace& box )
		{
			const mat4x4& sphereSpace = box.inverse;

			vec4f boxPoint = Math::clamp( sphereSpace.origin, { -1.0f, -1.0f, -1.0f, 0.0f }, { 1.0f, 1.0f, 1.0f, 0.0f } );
			vec4f spherePoint = box.transform * boxPoint;
			float distance2 = Math::dot_3D( spherePoint, spherePoint );

			if( distance2 > Math::EPSILON<float> )
			{
				// Calculate displacement from "revised" point
				displacement = spherePoint - ( spherePoint / sqrtf( distance2 ) );
			}
			else
			{
				// Arbitrarily displace along the x-axis
				displacement = Math::axis::X<vec4f>;
			}

			return distance2 <= 1.0f;
		}

		/**
		 * @brief Calculates the displacement of an arbitrary sphere intersecting with the unit sphere.
		 * @param displacement - write destination of displacement vector
		 * @param sphere - transform space of sphere
		 * @return Whether the sphere intersects the unit sphere
		*/
		static bool sphereCollision( vec4f& displacement, const TransformSpace& sphere )
		{
			vec4f vector = sphere.transform.origin;
			vector.w = 0.0f;

			float distance = Math::length( vector );
			if( distance <= Math::EPSILON<float> )
			{
				displacement = Math::axis::X<vec4f>;
				return true;
			}

			float radius = Math::length( sphere.transform * Math::normalize( sphere.inverse * vector ) );
			float signedDistance = ( ( 1.0f + radius ) - distance );

			displacement = vector * ( signedDistance / distance );
			return distance < 1.0f + radius;
		}
	}

	namespace triangle
	{
		/**
		 * @brief Calculates the Axis Aligned Bounding Box (AABB) that fits the triangle
		 * @param transform - transform of the triangle in the axis aligned space
		 * @return bounding vector pair that represents the AABB
		*/
		static Bounds<vec4f> calculateAABB( const mat4f& transform )
		{
			Bounds<vec4f> bounds;
			process::AABB_TRIANGLE( transform, bounds.min, bounds.max );
			return bounds;
		}

		/**
		 * @brief Calculates the intersecting distance of the ray against the triangle.
		 * @param triangle - transform of triangle
		 * @param rayOrigin - origin point of the ray
		 * @param rayVector - displacement vector to the end of the ray
		 * @return Whether the triangle intersects the unit cube
		*/
		constexpr static float rayDistance( const mat4f& triangle, const vec4f& rayOrigin, const vec4f& rayVector )
		{
			// Calculate the line-plane intersection
			vec4f displaced = triangle.origin - rayOrigin;
			vec4f normal = Math::cross( triangle.x_axis, triangle.y_axis );
			vec4f product;
			vf::simd::dot4( product,
				normal, displaced,
				Math::cross( rayVector, triangle.y_axis ), displaced,
				Math::cross( triangle.x_axis, rayVector ), displaced,
				normal, rayVector
			);

			// Check that the plane intersection hits the region of the triangle
			vec4f intersection = product / product.w;
			return intersection.y >= 0.0f && intersection.z >= 0.0f && intersection.y + intersection.z <= 1.0f ? intersection.x : INFINITY;
		}
	}

	constexpr vec2f intersection2D( vec2f originA, vec2f lineA, vec2f originB, vec2f lineB )
	{
		// p + rt
		// q + su
		float dx = originB.x - originA.x;
		float dy = originB.y - originA.y;
		float dp = lineA.x * lineB.y - lineA.y * lineB.x;

		// t = ( q - p ) x s / ( r x s )
		float t = ( dx * lineB.y - dy * lineB.x ) / dp;
		// u = ( q - p ) x r / ( r x s )
		float u = ( dx * lineA.y - dy * lineA.x ) / dp;
		return { t, u };
	}

	constexpr vec2f intersection3D( const vec4f& originA, const vec4f& lineA, const vec4f& originB, const vec4f& lineB )
	{
		vec4f normal = normalize( cross( lineA, lineB ) );
		vec4f tangent = normalize( lineA );
		vec4f bitangent = cross( normal, tangent );
		vec4f displacement = originB - originA;

		vec4f p;
		vf::simd::dot4( p, displacement, tangent, displacement, bitangent, lineB, tangent, lineB, bitangent );
		if( p.w <= Math::EPSILON<float> ) return { 0.0f, 0.0f };

		return { p.x - p.y * p.z / p.w, -p.y / p.w };
	}
}

#endif