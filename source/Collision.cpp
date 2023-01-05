#include "../include/vfMath/Collision.hpp"

vec3f dcast_Box(const vec3f& displacement)
{
	vec3f abs = Math::abs( displacement );
	return displacement / std::max( std::max( abs.x, abs.y ), abs.z );
}

/*
vec3f dcast_Cylinder(const InstantCollider& cylinder, const vec3f& displacement)
{
	vec3f relative = Math::normalize( cylinder.inverse * displacement );

	float vertical = relative.y;
	relative.y = 0.0f;

	if ( relative.y > 0.70710677f )
	{
		relative.y = 1.0f;
	}
	else
	{
		float len = sqrtf( relative.x * relative.x + relative.z * relative.z );
		relative.x /= len;
		relative.z /= len;
	}

	return cylinder.transform * relative;
}*/

static void testPoint(const InstantCollider& box, const vec3f point, vec3f& min, vec3f& max)
{
	vec3f relative = box.inverse * point;
	min.simd = _mm_min_ps(relative.simd, min.simd);
	max.simd = _mm_max_ps(relative.simd, max.simd);
}

// ---------------------------
// Collision Test Functions
// ---------------------------

static void test_Unimplemented( CollisionData& collision, const InstantCollider& a, const InstantCollider& b)
{
	//collision = { 0.0f, 0.0f, 0.0f, -1.0f };
}

static void test_Box_Box( CollisionData& collision, const InstantCollider& boxA, const InstantCollider& boxB )
{
	// Box B relative to box A
	{
		mat4x4 rB1 = boxA.inverse * boxB.transform;
		vec3f sign = Math::sign( rB1.origin );

		vec3f bounds = Math::MIN<vec4f>;
		bounds = Math::min(bounds, (rB1.origin + rB1.x_axis + rB1.y_axis + rB1.z_axis) * sign);
		bounds = Math::min(bounds, (rB1.origin - rB1.x_axis + rB1.y_axis + rB1.z_axis) * sign);
		bounds = Math::min(bounds, (rB1.origin + rB1.x_axis - rB1.y_axis + rB1.z_axis) * sign);
		bounds = Math::min(bounds, (rB1.origin - rB1.x_axis - rB1.y_axis + rB1.z_axis) * sign);
		bounds = Math::min(bounds, (rB1.origin + rB1.x_axis + rB1.y_axis - rB1.z_axis) * sign);
		bounds = Math::min(bounds, (rB1.origin - rB1.x_axis + rB1.y_axis - rB1.z_axis) * sign);
		bounds = Math::min(bounds, (rB1.origin + rB1.x_axis - rB1.y_axis - rB1.z_axis) * sign);
		bounds = Math::min(bounds, (rB1.origin - rB1.x_axis - rB1.y_axis - rB1.z_axis) * sign);
		
		// Check if the boxes intersect
		if ( Math::evaluate(bounds > Math::ONES<vec3f> ) ) return;
	}

	// Box A relative to box B
	{
		mat4x4 rB2 = boxB.inverse * boxA.transform;
		vec3f sign = Math::sign(rB2.origin);

		vec3f bounds = Math::MIN<vec4f>;
		bounds = Math::min(bounds, (rB2.origin + rB2.x_axis + rB2.y_axis + rB2.z_axis) * sign);
		bounds = Math::min(bounds, (rB2.origin - rB2.x_axis + rB2.y_axis + rB2.z_axis) * sign);
		bounds = Math::min(bounds, (rB2.origin + rB2.x_axis - rB2.y_axis + rB2.z_axis) * sign);
		bounds = Math::min(bounds, (rB2.origin - rB2.x_axis - rB2.y_axis + rB2.z_axis) * sign);
		bounds = Math::min(bounds, (rB2.origin + rB2.x_axis + rB2.y_axis - rB2.z_axis) * sign);
		bounds = Math::min(bounds, (rB2.origin - rB2.x_axis + rB2.y_axis - rB2.z_axis) * sign);
		bounds = Math::min(bounds, (rB2.origin + rB2.x_axis - rB2.y_axis - rB2.z_axis) * sign);
		bounds = Math::min(bounds, (rB2.origin - rB2.x_axis - rB2.y_axis - rB2.z_axis) * sign);

		// Check if the boxes intersect
		if ( Math::evaluate(bounds > Math::ONES<vec3f> ) ) return;

		// Calculate collision displacement
		vec3f displacement;
		if (bounds.x >= bounds.y)
		{
			if (bounds.x >= bounds.z)
			{
				displacement = boxA.transform.x_axis * sign.x;
				collision.signedDistance = 1.0f - bounds.x;
			}
			else
			{
				displacement = boxA.transform.z_axis * sign.z;
				collision.signedDistance = 1.0f - bounds.z;
			}
		}
		else
		{
			if (bounds.x >= bounds.z)
			{
				displacement = boxA.transform.y_axis * sign.y;
				collision.signedDistance = 1.0f - bounds.y;
			}
			else
			{
				displacement = boxA.transform.z_axis * sign.z;
				collision.signedDistance = 1.0f - bounds.z;
			}
		}

		float distance = Math::length( displacement );
		collision.normal = displacement / distance;
		collision.recoveryDirection = collision.normal;
		collision.signedDistance *= distance;
	}
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

static void test_Triangle_Sphere( CollisionData& collision, const InstantCollider& face, const InstantCollider& sphere)
{
	test_Sphere_Triangle( collision, sphere, face );
	collision.recoveryDirection *= -1.0f;
	collision.normal *= -1.0f;
}

static void test_Triangle_Triangle( CollisionData& collision, const InstantCollider& faceA, const InstantCollider& faceB)
{
	
}

const CollisionTest collisionMatrix[4][4]
{
	{ test_Box_Box,			test_Unimplemented,		test_Unimplemented,		test_Unimplemented },
	{ test_Unimplemented,	test_Sphere_Sphere,		test_Unimplemented,		test_Sphere_Triangle },
	{ test_Unimplemented,	test_Unimplemented,		test_Unimplemented,		test_Unimplemented },
	{ test_Unimplemented,	test_Triangle_Sphere,	test_Unimplemented,		test_Triangle_Triangle }
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