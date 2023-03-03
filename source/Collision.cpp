#include "../include/vfMath/Collision.hpp"

constexpr uint8_t maxIndex(const vec3f& vector)
{
	if (vector.x >= vector.y)	return vector.z >= vector.x ? 2u : 0u;
	else						return vector.z >= vector.y ? 2u : 1u;
}

constexpr uint8_t minIndex(const vec3f& vector)
{
	if (vector.x <= vector.y)	return vector.z <= vector.x ? 2u : 0u;
	else						return vector.z <= vector.y ? 2u : 1u;
}

bool raycast_Sphere(vec3f& entering, vec3f& exiting, const vec3f& origin, const vec3f& line)
{
	vec3f displacement = origin - Math::axis::W<vec4f>;
	vec3f projection = Math::project(displacement, line);
	vec3f perpendicular = displacement - projection;
	float dist2ToLine = Math::length2(perpendicular);

	if (dist2ToLine <= 1.0f) return false;

	float halfDistance = sqrtf(1.0f - dist2ToLine);
	entering = perpendicular - projection * halfDistance;
	exiting = perpendicular + projection * halfDistance;
	return true;
}

vec3f dcast_Box(const vec3f& displacement)
{
	vec3f abs = Math::abs( displacement );
	return displacement / std::max( std::max( abs.x, abs.y ), abs.z );
}

#define dcast_Sphere(x) Math::normalize(x)

vec3f dtest_Box(const vec4f& xAxis, const vec4f& yAxis, const vec4f& zAxis, const vec4f& origin)
{
	vec3f bounds = Math::MAX<vec3f>;
	bounds = Math::min(bounds, origin + xAxis + yAxis + zAxis);
	bounds = Math::min(bounds, origin - xAxis + yAxis + zAxis);
	bounds = Math::min(bounds, origin + xAxis - yAxis + zAxis);
	bounds = Math::min(bounds, origin - xAxis - yAxis + zAxis);
	bounds = Math::min(bounds, origin + xAxis + yAxis - zAxis);
	bounds = Math::min(bounds, origin - xAxis + yAxis - zAxis);
	bounds = Math::min(bounds, origin + xAxis - yAxis - zAxis);
	bounds = Math::min(bounds, origin - xAxis - yAxis - zAxis);
	return bounds;
}

// ---------------------------
// Collision Test Functions
// ---------------------------

static void test_Unimplemented( CollisionData& collision, const InstantCollider& a, const InstantCollider& b)
{
	collision.signedDistance = Math::MAX<float>;
}

static void test_Box_Box( CollisionData& collision, const InstantCollider& boxA, const InstantCollider& boxB )
{
	mat4x4 rBA = boxA.inverse * boxB.transform;
	mat4x4 rAB = boxB.inverse * boxA.transform;

	vec3f qBA = Math::sign(rBA.origin);
	vec3f qAB = Math::sign(rAB.origin);

	vec3f boundsBA = dtest_Box(rBA.x_axis * qBA, rBA.y_axis * qBA, rBA.z_axis * qBA, rBA.origin * qBA);
	vec3f boundsAB = dtest_Box(rAB.x_axis * qAB, rAB.y_axis * qAB, rAB.z_axis * qAB, rAB.origin * qAB);

	uint8_t idxA = maxIndex(boundsAB);
	uint8_t idxB = maxIndex(boundsBA);

	float lengthAB = Math::length(boxB.transform.col[idxA]);
	float lengthBA = Math::length(boxA.transform.col[idxB]);

	collision.normal = boxB.transform.col[idxA] * (qAB[idxA] / lengthAB);
	collision.recoveryDirection = collision.normal;
	collision.signedDistance = std::max( lengthAB * (boundsAB[idxA] - 1.0f), lengthBA * (boundsBA[idxB] - 1.0f) );
}

static void test_Box_Sphere( CollisionData& collision, const InstantCollider& boxA, const InstantCollider& sphereB )
{
	mat4x4 rAB = sphereB.inverse * boxA.transform;
	vec4f pBA = boxA.inverse * sphereB.transform.origin;

	vec3f clamped = Math::clamp(pBA, Math::NEGATIVE<vec3f>, Math::ONES<vec3f>);
	vec3f qBA = Math::sign(pBA);

	vec3f rx = rAB.x_axis * clamped.x;
	vec3f ry = rAB.y_axis * clamped.y;
	vec3f rz = rAB.z_axis * clamped.z;

	vec3f vectorAB[3] = {
		(rAB.origin - Math::axis::W<vec4f>) + (rAB.x_axis * qBA.x) + ry + rz,
		(rAB.origin - Math::axis::W<vec4f>) + (rAB.y_axis * qBA.y) + rz + rx,
		(rAB.origin - Math::axis::W<vec4f>) + (rAB.z_axis * qBA.z) + rx + ry
	};
	vec3f lengthAB = Math::parallel::length(vectorAB[0], vectorAB[1], vectorAB[2]);

	uint8_t idx = minIndex(lengthAB);
	vec3f displaced = sphereB.transform * vectorAB[idx];
	float distance = Math::length(displaced);

	collision.recoveryDirection = distance > Math::EPSILON<float> ? displaced / distance : Math::axis::X<vec3f>;
	collision.signedDistance = distance - Math::length(sphereB.transform * (vectorAB[idx] / lengthAB[idx]));
	collision.normal = collision.recoveryDirection;
}

static void test_Sphere_Box(CollisionData& collision, const InstantCollider& sphereA, const InstantCollider& boxB)
{
	mat4x4 rBA = sphereA.inverse * boxB.transform;
	vec4f pAB = boxB.inverse * sphereA.transform.origin;

	vec3f clamped = Math::clamp(pAB, Math::NEGATIVE<vec3f>, Math::ONES<vec3f>);
	vec3f qAB = Math::sign(pAB);

	vec3f rx = rBA.x_axis * clamped.x;
	vec3f ry = rBA.y_axis * clamped.y;
	vec3f rz = rBA.z_axis * clamped.z;

	vec3f vectorBA[3] = {
		(rBA.origin - Math::axis::W<vec4f>) + (rBA.x_axis * qAB.x) + ry + rz,
		(rBA.origin - Math::axis::W<vec4f>) + (rBA.y_axis * qAB.y) + rz + rx,
		(rBA.origin - Math::axis::W<vec4f>) + (rBA.z_axis * qAB.z) + rx + ry
	};
	vec3f lengthBA = Math::parallel::length( vectorBA[0], vectorBA[1], vectorBA[2] );

	uint8_t idx = minIndex(lengthBA);
	vec3f displaced = sphereA.transform * vectorBA[idx];
	float distance = Math::length( displaced );

	collision.recoveryDirection = distance > Math::EPSILON<float> ? displaced / -distance : Math::axis::X<vec3f>;
	collision.signedDistance = distance - Math::length( sphereA.transform * ( vectorBA[idx] / lengthBA[idx] ) );
	collision.normal = Math::normalize( qAB[idx] * boxB.transform.col[idx] );
}

static void test_Sphere_Sphere( CollisionData& collision, const InstantCollider& sphereA, const InstantCollider& sphereB)
{
	vec3f displacement = sphereA.transform.origin - sphereB.transform.origin;
	float distance2 = Math::length2(displacement);
	
	vec3f castedBA = sphereA.transform * dcast_Sphere(sphereA.inverse * Math::axis::X<vec3f>);
	vec3f castedAB = sphereB.transform * dcast_Sphere(sphereB.inverse * Math::axis::X<vec3f>);
	float distance = sqrtf(distance2);
	collision.signedDistance = distance - (Math::length(castedBA) + Math::length(castedAB));
	collision.normal = distance2 > Math::EPSILON<float> ? displacement / distance : Math::axis::X<vec3f>;
	collision.recoveryDirection = collision.normal;
}

static void test_Box_Triangle( CollisionData& collision, const InstantCollider& sphere, const InstantCollider& face )
{
	// TODO
	collision.signedDistance = Math::MAX<float>;
}

/**
 * @brief Performs the collision calculations between a sphere and a triangle
 * Assumptions:
 *     - Z-axis of triangle must be normalized
 * @param collision - collision data
 * @param sphere - spherical collider
 * @param face - triangle collider
*/
static void test_Sphere_Triangle( CollisionData& collision, const InstantCollider& sphere, const InstantCollider& face )
{
	const mat4x4& xform = sphere.transform;
	const vec3f& origin = face.transform.origin;

	// Get UV coordinates of closest point on triangle
	vec4f clamped = Geometry::clampTriangleUV( face.inverse * xform.origin );

	// Calculate distance vs radius
	collision.position = origin + face.transform.x_axis * clamped.x + face.transform.y_axis * clamped.y;
	collision.normal = face.transform.z_axis;
	vec3f displaced = xform.origin - collision.position;
	float distance = Math::length( displaced );

	collision.recoveryDirection = distance > Math::EPSILON<float> ? displaced / distance : collision.normal;
	collision.signedDistance = distance - Math::length( xform * Math::normalize( sphere.inverse * collision.recoveryDirection ) );
}

static void test_Triangle_Sphere( CollisionData& collision, const InstantCollider& face, const InstantCollider& sphere)
{
	//test_Sphere_Triangle( collision, sphere, face );
	//collision.recoveryDirection *= -1.0f;
	//collision.normal *= -1.0f;
	// TODO
	collision.signedDistance = Math::MAX<float>;
}

static void test_Triangle_Triangle( CollisionData& collision, const InstantCollider& faceA, const InstantCollider& faceB)
{
	// TODO
	collision.signedDistance = Math::MAX<float>;
}

const CollisionTest collisionMatrix[3][3]
{
	{ test_Box_Box,			test_Box_Sphere,		test_Box_Triangle },
	{ test_Sphere_Box,		test_Sphere_Sphere,		test_Sphere_Triangle },
	{ test_Unimplemented,	test_Triangle_Sphere,	test_Triangle_Triangle }
};

void Collision::getCollisionData(CollisionData& collision, const Collider& a, const Collider& b)
{
	return collisionMatrix[(unsigned char)a.type][(unsigned char)b.type](collision, a.volume, b.volume);
}

CollisionTest Collision::getCollisionTest(const ColliderType a, const ColliderType b)
{
	return collisionMatrix[(unsigned char)a][(unsigned char)b];
}

// ---------------------------
// Intersection Test Functions
// ---------------------------

bool intersect_Unimplemented(const InstantCollider& a, const InstantCollider& b)
{
	return false;
}

static bool intersect_Sphere_Triangle(const InstantCollider& sphere, const InstantCollider& face)
{
	const mat4x4& xform = sphere.transform;
	const vec3f& origin = face.transform.origin;

	// Get UV coordinates of closest point on triangle
	vec4f clamped = Geometry::clampTriangleUV(face.inverse * xform.origin);

	// Calculate distance vs radius
	vec3f position = origin + face.transform.x_axis * clamped.x + face.transform.y_axis * clamped.y;
	vec3f displaced = xform.origin - position;
	float distance = Math::length( displaced );

	return distance <= Math::EPSILON<float> || distance <= Math::length( xform * Math::normalize( sphere.inverse * displaced ) );
}

const IntersectTest intersectionMatrix[3][3]
{
	{ intersect_Unimplemented,	intersect_Unimplemented,	intersect_Unimplemented },
	{ intersect_Unimplemented,	intersect_Unimplemented,	intersect_Sphere_Triangle },
	{ intersect_Unimplemented,	intersect_Unimplemented,	intersect_Unimplemented }
};

bool Collision::isIntersecting(const Collider& a, const Collider& b)
{
	return intersectionMatrix[(unsigned char)a.type][(unsigned char)b.type](a.volume, b.volume);
}

IntersectTest Collision::getIntersectTest(const ColliderType a, const ColliderType b)
{
	return intersectionMatrix[(unsigned char)a][(unsigned char)b];
}


// ---------------------------
// Collision Mesh Primitives
// ---------------------------

CollisionFace createFace(const vec4f& p0, const vec4f& p1, const vec4f& p2)
{
	vec3f xAxis = p1 - p0;
	vec3f yAxis = p2 - p0;
	mat4x4 transform{
		xAxis,
		yAxis,
		Math::normalize(Math::cross(xAxis, yAxis)),
		{ p0.x, p0.y, p0.z, 1.0f }
	};
	return { transform, transform.inverse() };
}

CollisionFace* setQuad(CollisionFace* faces, const vec4f& p0, const vec4f& p1, const vec4f& p2, const vec4f& p3, const bool reverse)
{
	if (reverse)
	{
		*(faces++) = createFace(p0, p3, p1);
		*(faces++) = createFace(p2, p1, p3);
	}
	else
	{
		*(faces++) = createFace(p0, p1, p3);
		*(faces++) = createFace(p2, p3, p1);
	}
	return faces;
}

MeshCollider Collision::createCubeMesh()
{
	MeshCollider collider(12);
	CollisionFace* ptr = collider.faces;

	ptr = setQuad(ptr, { -1.0f, -1.0f, -1.0f }, { -1.0f,  1.0f, -1.0f }, {  1.0f,  1.0f, -1.0f }, {  1.0f, -1.0f, -1.0f }, false);
	ptr = setQuad(ptr, { -1.0f, -1.0f,  1.0f }, { -1.0f,  1.0f,  1.0f }, {  1.0f,  1.0f,  1.0f }, {  1.0f, -1.0f,  1.0f }, true);
	ptr = setQuad(ptr, { -1.0f, -1.0f, -1.0f }, { -1.0f, -1.0f,  1.0f }, {  1.0f, -1.0f,  1.0f }, {  1.0f, -1.0f, -1.0f }, true);
	ptr = setQuad(ptr, { -1.0f,  1.0f, -1.0f }, { -1.0f,  1.0f,  1.0f }, {  1.0f,  1.0f,  1.0f }, {  1.0f,  1.0f, -1.0f }, false);
	ptr = setQuad(ptr, { -1.0f, -1.0f, -1.0f }, { -1.0f,  1.0f, -1.0f }, { -1.0f,  1.0f,  1.0f }, { -1.0f, -1.0f,  1.0f }, true);
	ptr = setQuad(ptr, {  1.0f, -1.0f, -1.0f }, {  1.0f,  1.0f, -1.0f }, {  1.0f,  1.0f,  1.0f }, {  1.0f, -1.0f,  1.0f }, false);

	return collider;
}