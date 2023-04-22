#pragma once
#ifndef VF_GEOMETRY_HPP
#define VF_GEOMETRY_HPP

#include "MatrixMath.hpp"

// W-coordinate of vector is the sphere's radius
typedef vec4f Sphere;

// Matrix represents transformation of unit cube
typedef mat4x4 Box;
// Matrix represents transformation of unit sphere
typedef mat4x4 Ellipsoid;
// Each column of the matrix represents a point on the triangle
typedef mat4x4 Triangle;
// Each column of the matrix represents a point on the quad
typedef mat4x4 Quad;
// First two columns are the axes along the plane, third column is normal, fourth column is origin
typedef mat4x4 Plane;

typedef bool (*MatrixIntersector)(const mat4x4& a, const mat4x4& b);
typedef bool (*VectorIntersector)(const vec4f& a, const vec4f& b);

struct Line
{
	vec4f origin;
	vec3f vector;
};

/*
struct Capsule
{
	union
	{
		Line line;
		struct { vec3f p0, p1; };
	};
	float radius;
};*/

namespace Geometry
{
	/**
	 * @brief Calculates the distance from a point to a line defined by an origin and direction
	 * @param point - target point
	 * @param origin - point on the line
	 * @param direction - direction of line (normalized)
	 * @return distance between point and line
	*/
	static float distanceToLine(const vec4f& point, const vec4f& origin, const vec3f& direction)
	{
		vec3f displacement = point - origin;
		float c2 = Math::dot_3D(displacement, displacement);
		float b  = Math::dot_3D(displacement, direction);
		return sqrtf(c2 - b * b);
	}

	/**
	 * @brief Calculates the distance from a point to a plane defined by an origin and normal
	 * @param point - target point
	 * @param origin - point that lies on the plane
	 * @param normal - normal of the plane (normalized)
	 * @return distance of the point to the plane
	*/
	static float distanceToPlane(const vec4f& point, const vec4f& origin, const vec3f& normal)
	{
		return Math::dot_3D(point - origin, normal);
	}

	/**
	 * @brief Calculates the closest point between lines.
	 * @param p0 - first point
	 * @param l0 - first vector (normalized)
	 * @param p1 - second point
	 * @param l1 - second vector (normalized)
	 * @return point on first line closest to second line
	*/
	/*static vec4f closestPointBetweenLines(const vec4f& p0, const vec4f& l0, const vec4f& p1, const vec4f& l1)
	{
		vec3f displacement = p1 - p0;
		vec3f normal = Math::normalize(Math::cross(l0, l1));
		vec3f rejection = displacement - (l1 * Math::dot(displacement, l1)) - (normal * Math::dot(displacement, normal));
		float length = Math::length(rejection);
		float t = length / Math::dot(l0, rejection / length);
		return p0 - l0 * t;
	}*/

	/**
	 * @brief Calculates a point on the line that has the shortest distance to the target
	 * @param origin - line origin
	 * @param direction - line direction (normalized)
	 * @param target - target point
	 * @return point on line closest to target
	*/
	static vec4f closestPointOnLine(const vec4f& origin, const vec3f& direction, const vec4f& target)
	{
		return origin + direction * Math::dot_3D(target - origin, direction);
	}

	/**
	 * @brief Calculates the intersecting UVT coordinates of the line and plane
	 * @param lineOrigin - origin of line segment
	 * @param line - vector of line segment
	 * @param planeOrigin - origin of plane
	 * @param primary - primary axis of plane
	 * @param secondary - secondary axis of plane
	 * @return UVT coordinates ( u, v, t, 1.0 )
	*/
	static vec4f intersectionCoords(const vec4f& lineOrigin, const vec3f& line, const mat4x4& face)
	{
		vec3f normal = Math::cross(face.x_axis, face.y_axis);
		vec3f displace = lineOrigin - face.origin;
		vec4f intersection = Math::parallel::dot(Math::cross(line, face.y_axis), displace, Math::cross(face.x_axis, line), displace, normal, displace, -line, normal);
		return intersection / intersection.w;
	}

	/**
	 * @brief Clamps the UV coordinates to the boundary of a triangle
	 * @param uv - UV coordinates stored in a 2D vector
	 * @return clamped UV coordinates
	*/
	static vec2f clampTriangleUV(const vec2f& uv)
	{
		vec2f bounded = Math::max(uv, Math::ZERO<vec2f>);
		return bounded.x + bounded.y > 1.0f ? bounded / (bounded.x + bounded.y) : bounded;
	}

	/**
	 * @brief Clamps the UV coordinates to the boundary of a triangle
	 * @param uv - UV coordinates stored in a 4D vector
	 * @return clamped UV coordinates
	*/
	static vec4f clampTriangleUV(const vec4f& uv)
	{
		vec4f bounded = Math::max(uv, Math::ZERO<vec4f>);
		return bounded.x + bounded.y > 1.0f ? bounded / (bounded.x + bounded.y) : bounded;
	}

	/**
	 * @brief Returns the squared radius of the ellipsoid along the direction out from the origin
	 * @param e - ellipsoid
	 * @param direction - direction vector from origin (normalized)
	 * @return squared radius of the ellipsoid in that direction
	*/
	static float getSquaredRadius(const Ellipsoid& e, const vec3f& direction)
	{
		vec3f projection = Math::parallel::dot(direction, e.x_axis, direction, e.y_axis, direction, e.z_axis)
			/ Math::parallel::dot(e.x_axis, e.x_axis, e.y_axis, e.y_axis, e.z_axis, e.z_axis);

		return 1.0f / (projection.x * projection.x + projection.y * projection.y + projection.z * projection.z);
	}

	/**
	 * @brief Returns the radius of the ellipsoid along the direction out from the origin
	 * @param e - ellipsoid
	 * @param direction - direction vector from origin (normalized)
	 * @return radius of the ellipsoid in that direction
	*/
	static float getRadius(const Ellipsoid& e, const vec3f& direction)
	{
		return sqrtf(getSquaredRadius(e, direction));
	}

	/**
	 * @brief Tests for the bounds of each point along each axis of the box and updates the min and max vectors
	 * @param box - box that is tested
	 * @param point - point that is being tested
	 * @param min - minimum boundary of the current test
	 * @param max - maximum boundary of the current test 
	*/
	static void testPoint(const Box& box, const vec3f& point, vec3f& min, vec3f& max)
	{
		__m128 relative = _mm_sub_ps(point.simd, box.simd[3]);
		__m128 product = _mm_dot3_ps(relative, box.simd[0], relative, box.simd[1], relative, box.simd[2]);
		min.simd = _mm_min_ps(product, min.simd);
		max.simd = _mm_max_ps(product, max.simd);
	}

	/**
	 * @brief Checks if the point is contained within the box
	 * @param box - box of the intersection
	 * @param point - point of the intersection
	 * @return Whether the point is inside the box
	*/
	static bool intersect_Box_Point(const Box& box, const vec3f& point)
	{
		vec3f vector = point - box.origin;
		vec3f proj = Math::parallel::dot(vector, box.x_axis, vector, box.y_axis, vector, box.z_axis);
		vec3f len2 = Math::parallel::dot(box.x_axis, box.x_axis, box.y_axis, box.y_axis, box.z_axis, box.z_axis);
		return Math::evaluate(proj >= -len2 && proj <= len2);
	}

	/**
	 * @brief Checks if two boxes intersect
	 * @param b0 - First box
	 * @param b1 - second box
	 * @return Whether the intersection occurs
	*/
	static bool intersect_Box(const Box& b0, const Box& b1)
	{
		// Test axis alignment on first box
		{
			vec3f min0 = Math::MAX<vec4f>;
			vec3f max0 = Math::MIN<vec4f>;
			testPoint(b0, b1.col[3] + b1.col[0] + b1.col[1] + b1.col[2], min0, max0);
			testPoint(b0, b1.col[3] - b1.col[0] + b1.col[1] + b1.col[2], min0, max0);
			testPoint(b0, b1.col[3] + b1.col[0] - b1.col[1] + b1.col[2], min0, max0);
			testPoint(b0, b1.col[3] - b1.col[0] - b1.col[1] + b1.col[2], min0, max0);
			testPoint(b0, b1.col[3] + b1.col[0] + b1.col[1] - b1.col[2], min0, max0);
			testPoint(b0, b1.col[3] - b1.col[0] + b1.col[1] - b1.col[2], min0, max0);
			testPoint(b0, b1.col[3] + b1.col[0] - b1.col[1] - b1.col[2], min0, max0);
			testPoint(b0, b1.col[3] - b1.col[0] - b1.col[1] - b1.col[2], min0, max0);

			vec3f len2b0 = Math::parallel::dot(b0.x_axis, b0.x_axis, b0.y_axis, b0.y_axis, b0.z_axis, b0.z_axis);
			if (!Math::overlaps(min0, max0, -len2b0, len2b0)) return false;
		}

		// Test axis alginment on second box
		{
			vec3f min1 = Math::MAX<vec4f>;
			vec3f max1 = Math::MIN<vec4f>;
			testPoint(b1, b0.col[3] + b0.col[0] + b0.col[1] + b0.col[2], min1, max1);
			testPoint(b1, b0.col[3] - b0.col[0] + b0.col[1] + b0.col[2], min1, max1);
			testPoint(b1, b0.col[3] + b0.col[0] - b0.col[1] + b0.col[2], min1, max1);
			testPoint(b1, b0.col[3] - b0.col[0] - b0.col[1] + b0.col[2], min1, max1);
			testPoint(b1, b0.col[3] + b0.col[0] + b0.col[1] - b0.col[2], min1, max1);
			testPoint(b1, b0.col[3] - b0.col[0] + b0.col[1] - b0.col[2], min1, max1);
			testPoint(b1, b0.col[3] + b0.col[0] - b0.col[1] - b0.col[2], min1, max1);
			testPoint(b1, b0.col[3] - b0.col[0] - b0.col[1] - b0.col[2], min1, max1);

			vec3f len2b1 = Math::parallel::dot(b1.x_axis, b1.x_axis, b1.y_axis, b1.y_axis, b1.z_axis, b1.z_axis);
			return Math::overlaps(min1, max1, -len2b1, len2b1);
		}
	}

	/**
	 * @brief Checks whether the box intersects the ellipsoid
	 * @param b0 - box of the intersection
	 * @param e - ellipsoid of the intersection
	 * @return Whether the intersection occurs
	*/
	static bool intersect_Box_Ellipsoid(const Box& b0, const Ellipsoid& e)
	{
		vec3f displacement = e.origin - b0.origin;
		vec3f direction = Math::normalize(displacement);
		float radius = getRadius(e, direction);
		return intersect_Box_Point(b0, direction * radius + e.origin) 
			|| Math::length(displacement) <= radius;
	}

	/**
	 * @brief Checks if a box and a triangle intersects
	 * @param b0 - box of the intersection
	 * @param p0
	 * @param p1
	 * @param p2
	 * @return Whether the intersection occurs
	*/
	static bool intersect_Box_Triangle(const Box& b0, const vec4f& p0, const vec4f& p1, const vec4f& p2)
	{
		vec3f min = Math::MAX<vec4f>;
		vec3f max = Math::MIN<vec4f>;
		
		testPoint(b0, p0, min, max);
		testPoint(b0, p1, min, max);
		testPoint(b0, p2, min, max);

		vec3f len2 = Math::parallel::dot(b0.x_axis, b0.x_axis, b0.y_axis, b0.y_axis, b0.z_axis, b0.z_axis);
		return Math::overlaps(min, max, -len2, len2);
	}

	/**
	 * @brief Checks whether two ellipsoids intersect
	 * @param e0 - first ellipsoid
	 * @param e1 - second ellipsoid
	 * @return Whether the intersection occurs
	*/
	static bool intersect_Ellipsoid(const Ellipsoid& e0, const Ellipsoid& e1)
	{
		vec3f vector = e1.col[3] - e0.col[3];
		float dist2 = Math::length2(vector);
		vec3f direction = vector / sqrtf(dist2);
		return dist2 < getSquaredRadius(e0, direction) + getSquaredRadius(e1, direction);
	}

	/**
	 * @brief Checks whether the ellipsoid contains a point
	 * @param e - ellipsoid of the intersection
	 * @param point - point of the intersection
	 * @return Whether the intersection occurs
	*/
	static bool intersect_Ellipsoid_Point(const Ellipsoid& e, const vec3f& point)
	{
		vec3f vector = point - e.col[3];
		float dist2 = Math::length2(vector);
		vec3f direction = vector / sqrtf(dist2);
		return dist2 < getSquaredRadius(e, direction);
	}

	/**
	 * @brief Checks whether the ellipsoid contains a point
	 * @param e - ellipsoid of the intersection
	 * @param point - point of the intersection
	 * @return Modified point moved outside of the geometry
	*/
	static vec3f collision_Ellipsoid_Point(const Ellipsoid& e, const vec4f& point)
	{
		vec3f vector = point - e.origin;
		float dist2 = Math::length2(vector);
		vec3f direction = vector / Math::sqrt( { _mm_set1_ps( dist2 ) } );
		float radius2 = getSquaredRadius(e, direction);
		return dist2 >= radius2 ? point : e.origin + direction * Math::sqrt( { _mm_set1_ps( radius2 ) } );
	}

	static bool intersect_Ellipsoid_Line(const Ellipsoid& e, const vec3f& point)
	{

	}

	/**
	 * @brief Checks whether the ellipsoid intersects the plane
	 * @param e - ellipsoid of the intersection
	 * @param plane - plane of the intersection
	 * @return Whether the intersection occurs
	*/
	static bool intersect_Ellipsoid_Plane(const Ellipsoid& e, const Plane& plane)
	{
		return distanceToPlane(e.col[3], plane.col[3], plane.col[2]) < getRadius(e, -plane.col[2]);
	}

	/**
	 * @brief Checks whether the ellipsoid intersects the triangle
	 * @param e - ellipsoid of the intersection
	 * @param triangle - triangle of the intersection
	 * @return Whether the intersection occurs
	*/
	static bool intersect_Ellipsoid_Triangle(const Ellipsoid& e, const Triangle& triangle)
	{
		return false;
	}

	/**
	 * @brief Checks whether two spheres intersect
	 * @param s0 - first sphere
	 * @param s1 - second sphere
	 * @return Whether the intersection occurs
	*/
	static bool intersect_Sphere(const Sphere& s0, const Sphere& s1)
	{
		vec3f diff = s1 - s0;
		return diff.x * diff.x + diff.y * diff.y + diff .z * diff.z < s0.w * s0.w + s1.w * s1.w;
	}

	/**
	 * @brief Checks whether the sphere intersects the plane
	 * @param sphere - sphere of the intersection
	 * @param plane - plane of the intersection
	 * @return Whether the intersection occurs
	*/
	static bool intersect_Sphere_Plane(const Sphere& sphere, const Plane& plane)
	{
		return distanceToPlane(sphere, plane.col[3], plane.col[2]) < sphere.w;
	}

	static bool intersect_Point_Triangle(const vec4f& point, const vec4f* tri, const float distance2 = 0.0f)
	{
		vec3f normal = Math::cross( tri[1] - tri[0], tri[2] - tri[0] );
		float area2 = Math::length2(normal);
		float shortDist2 = Math::dot_3D( point - tri[0], normal ) / area2;
		if ( shortDist2 > distance2 ) return false;

		vec4f closest = point - normal * sqrtf( shortDist2 / area2 );

		vec3f tangentLine = closest - tri[0];
		float u = Math::dot( Math::cross( normal, tri[2] - tri[0] ), tangentLine ) / area2;
		float v = Math::dot( Math::cross( normal, tri[2] - tri[0] ), tangentLine ) / area2;
		return u >= 0.0f && v >= 0.0f && u + v <= 1.0f;
		
		/*
		// Area solution
		vec3f v0 = tri[0] - closest;
		vec3f v1 = tri[1] - closest;
		vec3f v2 = tri[2] - closest;
		vec3f t0 = Math::cross(v0, v1);
		vec3f t1 = Math::cross(v1, v2);
		vec3f t2 = Math::cross(v2, v0);
		vec3f subArea2 = Math::parallel::dot(t0, t0, t1, t1, t2, t2);

		return subArea2.x + subArea2.y + subArea2.z <= area2;*/
	}

	static bool intersect_Segment_Triangle_V2(const vec4f* line, const Plane& tri, const float distance2 = 0.0f)
	{
		float distanceA = Math::dot_3D(line[0] - tri.origin, tri.z_axis);
		float distanceB = Math::dot_3D(line[1] - tri.origin, tri.z_axis);

		vec4f pointA = line[0] - tri.z_axis * distanceA;
		vec4f pointB = line[1] - tri.z_axis * distanceB;


	}

	/**
	 * @brief Determines if a line segment intersects a triangle within a distance threshold
	 * @param origin - line segment origin
	 * @param segment - segment vector
	 * @param tri - triangle defined by a transform matrix
	 * @param distance2 - squared distance threshold 
	 * @return Whether there is an intersection
	*/
	static bool intersect_Segment_Triangle(const vec4f& origin, const vec4f& segment, const Plane& tri, const float distance2 = 0.0f)
	{
		vec4f displacement = origin - tri.origin;
		vec4f solution = Math::parallel::dot(
			tri.z_axis, displacement,
			Math::cross( segment, tri.y_axis ), displacement,
			Math::cross( tri.x_axis, segment ), displacement,
			segment, tri.z_axis );

		solution /= solution.w;
		float maxClamp = 1.0f / ( solution.y + solution.z );
		solution = Math::clamp( solution, Math::ZERO<vec4f>, { 1.0f, maxClamp, maxClamp, 1.0f } );
		return Math::distance2( origin + segment * solution.x, tri.origin + tri.x_axis * solution.y + tri.y_axis * solution.z ) < distance2;
	}

	/**
	 * @brief Calculates where two planes intersect
	 * @param p0 - first plane of intersection
	 * @param p1 - second plane of intersection
	 * @param intersection - Line that intersection is written to
	 * @return Whether the intersection occured
	*/
	static bool intersection(const Plane& p0, const Plane& p1, Line& intersection)
	{
		vec3f line = Math::cross(p0.z_axis, p1.z_axis);
		float length = Math::length2(line);
		if (length < 1E-9f) return false;

		// Find arbitrary point in intersection
		intersection.origin = p0.origin + p0.x_axis * Math::dot(p1.z_axis, p0.origin - p1.origin) / -Math::dot(p0.x_axis, p1.z_axis);
		intersection.vector = line / sqrtf(length);
		return true;
	}

	/**
	 * @brief Calculates where the line and plane intersect
	 * @param line - line of the intersection
	 * @param plane - plane of the intersection
	 * @param intersection - vector that intersecting point is written to
	 * @return Whether the intersection occured
	*/
	static bool intersection_Line_Plane(const Line& line, const Plane& plane, vec3f& intersection)
	{
		float product = -Math::dot(line.vector, plane.col[2]);
		if (product == 0.0f) return false;

		intersection = line.origin + line.vector * Math::dot(plane.col[2], line.origin - plane.col[3]) / product;
		return true;
	}

	constexpr float clampedMin(const float value, const float min)
	{
		if (value < 0.0f || value > 1.0f) return min;
		return value < min ? value : min;
	}

	static float link_Line_Triangle(const Line& line, const vec4f& p0, const vec4f& p1, const vec4f& p2, const float min)
	{
		vec3f displace = line.origin - p0;
		vec3f normal = Math::cross(p1 - p0, p2 - p0);
		vec4f intersection = Math::parallel::dot(normal, displace, Math::cross(line.vector, p2 - p0), displace, Math::cross(p1 - p0, line.vector), displace, -line.vector, normal);
		intersection /= intersection.w;

		return intersection.x >= 0.0f && intersection.x <= 1.0f 
			&& intersection.y >= 0.0f && intersection.z >= 0.0f
			&& intersection.y + intersection.z <= 1.0f && intersection.x < min ? intersection.x : min;
	}

	static float link_Line_BoxFace(const Line& line, const vec3f& origin, const vec3f& axis0, const vec3f& axis1, const float min)
	{
		vec3f displace = line.origin - origin;
		vec3f normal = Math::cross(axis0, axis1);
		vec4f intersection = Math::parallel::dot(normal, displace, Math::cross(line.vector, axis1), displace, Math::cross(axis0, line.vector), displace, -line.vector, normal);
		intersection /= intersection.w;

		return intersection.x >= 0.0f && intersection.x <= 1.0f 
			&& intersection.y >= -1.0f && intersection.y <= 1.0f
			&& intersection.z >= -1.0f && intersection.z <= 1.0f && intersection.x < min ? intersection.x : min;
	}

	/**
	 * @brief Calculates where the box and point intersect
	 * @param box - box of the intersection
	 * @param line - line of the intersection
	 * @param intersection - vector that intersecting point is written to
	 * @return Whether the intersection occured
	*/
	static bool intersect_Box_Line(const Box& box, const Line& line, vec3f& intersection)
	{
		float t = Math::MAX<float>;

		t = link_Line_BoxFace(line, box.col[3] + box.col[0], box.col[1], box.col[2], t);
		t = link_Line_BoxFace(line, box.col[3] - box.col[0], box.col[2], box.col[1], t);
		t = link_Line_BoxFace(line, box.col[3] + box.col[1], box.col[2], box.col[0], t);
		t = link_Line_BoxFace(line, box.col[3] - box.col[1], box.col[0], box.col[2], t);
		t = link_Line_BoxFace(line, box.col[3] + box.col[2], box.col[0], box.col[1], t);
		t = link_Line_BoxFace(line, box.col[3] - box.col[2], box.col[1], box.col[0], t);

		if (t > 1.0f) return false;

		intersection = line.origin + line.vector * t;
		return true;
	}

	/*
	static bool intersects(const Capsule& capsule, const Plane& plane)
	{
		vec3f tuv = Math::clamp(intersection(capsule.line, plane));
		vec3f p0 = (capsule.p1 - capsule.p0) * tuv.x;
		vec3f p1 = plane.origin + plane.axis0 * tuv.y + plane.axis1 * tuv.z;
		return Math::distance(p0, p1) < capsule.radius;
	}
*/

	/**
	 * @brief Checks if the point is contained within the box
	 * @param box - box of the intersection
	 * @param point - point of the intersection
	 * @return Modified point moved outside of the geometry
	*/
	/*static vec3f collision_Box_Point(const Box& box, const vec3f& point)
	{
		vec3f vector = point - box.origin;
		vec3f proj = Math::parallel::dot(vector, box.x_axis, vector, box.y_axis, vector, box.z_axis);
		vec3f len2 = Math::parallel::dot(box.x_axis, box.x_axis, box.y_axis, box.y_axis, box.z_axis, box.z_axis);

		if (!Math::evaluate(proj >= -len2 && proj <= len2)) return point;

		vec3f magnitude = Math::abs(proj / len2);

		if (magnitude.x >= magnitude.y && magnitude.x >= magnitude.z)
		{
			return proj.x > 0.0f ?
				point + box.x_axis * (1.0f - magnitude.x) :
				point - box.x_axis * (1.0f - magnitude.x);
		}
		else if (magnitude.y >= magnitude.x && magnitude.y >= magnitude.z)
		{
			return proj.y > 0.0f ?
				point + box.y_axis * (1.0f - magnitude.y) :
				point - box.y_axis * (1.0f - magnitude.y);
		}
		else
		{
			return proj.z > 0.0f ?
				point + box.z_axis * (1.0f - magnitude.z) :
				point - box.z_axis * (1.0f - magnitude.z);
		}
	}*/
}

namespace IK
{
	/**
	 * @brief Calculates a point on a line that is the specified distance away from the target in the positive direction
	 * @param origin - line origin
	 * @param direction - line direction (normalized)
	 * @param target - target point
	 * @param distance - target distance
	 * @return point on line that is the specified distance from the target
	*/
	static vec4f ik_solve_line(const vec4f& origin, const vec3f& direction, const vec4f& target, const float distance)
	{
		// Form right-angle triangle between line origin, target, and point
		vec4f point = Geometry::closestPointOnLine(origin, direction, target);
		return point + direction * sqrtf(distance * distance - Math::distance2(target, point));
	}

	/**
	 * @brief Calculates the new end point of a bone defined by two points along a plane
	 * @param origin - plane origin
	 * @param normal - plane normal (normalized)
	 * @param start - bone starting point
	 * @param end - bone ending point
	 * @return new end point along plane
	*/
	static vec4f ik_snap_to_plane(const vec4f& origin, const vec3f& normal, const vec4f& start, const vec4f& end)
	{
		vec3f displacement = end - start;
		vec4f lineOrigin = origin + Math::orthogonal(start - origin, normal);
		vec3f lineDirection = Math::normalize( Math::orthogonal( displacement, normal ) );

		return ik_solve_line( lineOrigin, lineDirection, start, Math::length(displacement) );
	}

	/**
	 * @brief Calculates the midpoint displacement that fits the length constraints
	 * @param displacement - displacement vector
	 * @param trackAxis - normal of plane the solution must be on (normalized)
	 * @param lenA - length from start to midpoint
	 * @param lenB - length from midpoint to end
	 * @return Calculated midpoint displacement
	*/
	static vec3f ik_solve_midpoint(vec3f displacement, const vec3f& trackAxis, const float lenA, const float lenB)
	{
		// Calculate the normal vector based on track axis;
		float lenC = Math::length(displacement);
		displacement /= lenC;
		vec3f normal = Math::cross(displacement, trackAxis);

		// Calculate the midpoint
		float offset = (lenA * lenA + lenC * lenC - lenB * lenB) / (2.0f * lenC);
		float height2 = lenA * lenA - offset * offset;

		// Return half vector if distance is greather than combined length
		if (height2 < 0.0f) return displacement * ( (lenC * lenA) / (lenA + lenB) );
		return displacement * offset + normal * sqrtf(height2);
	}

	/**
	 * @brief Calculates the transforms of a pair of beams connected between two points
	 * @param jointA - joint of the starting beam
	 * @param jointB - joint between the beams
	 * @param target - position of the end of the two beams
	 * @param pole - direction that beams should point towards
	 * @param lenA - length of the first beam
	 * @param lenB - length of the second beam
	*/
	static void ik_solve_pair(mat4x4& jointA, mat4x4& jointB, const vec4f& target, const vec3f& pole, const float lenA, const float lenB)
	{
		vec3f vector = target - jointA.origin;
		vec3f left = Math::normalize(Math::cross(pole, vector));
		vec4f midpoint = jointA.origin + ik_solve_midpoint(vector, left, lenA, lenB);

		// Generate the rotation matrix required to track the point from this bone
		jointA = Math::lookToY(Math::normalize(midpoint - jointA.origin), left, jointA.origin);
		jointB = Math::lookToY(Math::normalize(target - midpoint), left, midpoint);
	}
}

#endif