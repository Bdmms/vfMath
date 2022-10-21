#include "../include/vfMath/Collision.hpp"

// ---------------------------
// Collision Test Functions
// ---------------------------

static void test_Unimplemented( CollisionData& collision, const InstantCollider& a, const InstantCollider& b)
{
	//collision = { 0.0f, 0.0f, 0.0f, -1.0f };
}

static void test_Box_Box( CollisionData& collision, const InstantCollider& boxA, const InstantCollider& boxB )
{
	// TODO
	//collision = { 0.0f, 0.0f, 0.0f, Geometry::intersect_Box( boxA.transform, boxB.transform ) ? 1.0f : -1.0f };
}

static void test_Sphere_Sphere( CollisionData& collision, const InstantCollider& sphereA, const InstantCollider& sphereB)
{
	// TODO
	//collision = { 0.0f, 0.0f, 0.0f, Geometry::intersect_Ellipsoid( sphereA.transform, sphereB.transform ) ? 1.0f : -1.0f };
}

static void test_Box_Triangle( CollisionData& collision, const InstantCollider& sphere, const InstantCollider& face )
{
	//collision = { 0.0f, 0.0f, 0.0f, 1.0f };
}

/**
 * @brief Evaluates if an ellipsoid intersects a triangle, and calculates the recovery vector if an intersection occurs
 * @param recovery - vector used to eliminate collision when applied to ellipsoid
 * @param sphere - ellipsoid transform
 * @param face - triangle face
 * @return Whether the intersection occured
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
	float distance = Math::length(displaced);

	if (distance != 0.0f)
	{
		collision.recoveryDirection = displaced / distance;
		collision.signedDistance = distance - Math::length( xform * Math::normalize( sphere.inverse * collision.recoveryDirection ) );
	}
	else
	{
		collision.recoveryDirection = collision.normal;
		collision.signedDistance = 0.0f;
	}
}

static void test_Triangle_Sphere( CollisionData& collision, const InstantCollider& face, const InstantCollider& ellipsoid )
{
	return test_Sphere_Triangle( collision, ellipsoid, face );
}

const CollisionTest collisionMatrix[4][4]
{
	{ test_Box_Box,			test_Unimplemented,		test_Unimplemented,		test_Unimplemented },
	{ test_Unimplemented,	test_Sphere_Sphere,		test_Unimplemented,		test_Sphere_Triangle },
	{ test_Unimplemented,	test_Unimplemented,		test_Unimplemented,		test_Unimplemented },
	{ test_Unimplemented,	test_Triangle_Sphere,	test_Unimplemented,		test_Unimplemented }
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