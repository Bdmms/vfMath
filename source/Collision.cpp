#include "../include/vfMath/Collision.hpp"

/**
 * @brief Clamps the UV coordinates to the boundary of a triangle
 * @param uv - UV coordinates stored in a 2D vector
 * @return clamped UV coordinates
*/
static vec2f clampTriangleUV( const vec2f& uv )
{
	vec2f bounded = Math::max( uv, Math::ZERO<vec2f> );
	return bounded.x + bounded.y > 1.0f ? bounded / ( bounded.x + bounded.y ) : bounded;
}

/**
 * @brief Clamps the UV coordinates to the boundary of a triangle
 * @param uv - UV coordinates stored in a 4D vector
 * @return clamped UV coordinates
*/
static vec4f clampTriangleUV( const vec4f& uv )
{
	vec4f bounded = Math::max( uv, Math::ZERO<vec4f> );
	return bounded.x + bounded.y > 1.0f ? bounded / ( bounded.x + bounded.y ) : bounded;
}

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

static void test_Unimplemented( CollisionData& collision, const TransformSpace& a, const TransformSpace& b)
{
	collision.signedDistance = Math::MAX<float>;
}

static void test_Box_Box( CollisionData& collision, const TransformSpace& boxA, const TransformSpace& boxB )
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

static void test_Box_Sphere( CollisionData& collision, const TransformSpace& boxA, const TransformSpace& sphereB )
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

static void test_Sphere_Box(CollisionData& collision, const TransformSpace& sphereA, const TransformSpace& boxB)
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

static void test_Sphere_Sphere( CollisionData& collision, const TransformSpace& sphereA, const TransformSpace& sphereB)
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

static void test_Box_Triangle( CollisionData& collision, const TransformSpace& sphere, const TransformSpace& face )
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
static void test_Sphere_Triangle( CollisionData& collision, const TransformSpace& sphere, const TransformSpace& face )
{
	const mat4x4& xform = sphere.transform;
	const vec3f& origin = face.transform.origin;

	// Get UV coordinates of closest point on triangle
	vec4f clamped = clampTriangleUV( face.inverse * xform.origin );

	// Calculate distance vs radius
	collision.position = origin + face.transform.x_axis * clamped.x + face.transform.y_axis * clamped.y;
	collision.normal = face.transform.z_axis;
	vec3f displaced = xform.origin - collision.position;
	float distance = Math::length( displaced );

	collision.recoveryDirection = distance > Math::EPSILON<float> ? displaced / distance : collision.normal;
	collision.signedDistance = distance - Math::length( xform * Math::normalize( sphere.inverse * collision.recoveryDirection ) );
}

static void test_Triangle_Sphere( CollisionData& collision, const TransformSpace& face, const TransformSpace& sphere)
{
	//test_Sphere_Triangle( collision, sphere, face );
	//collision.recoveryDirection *= -1.0f;
	//collision.normal *= -1.0f;
	// TODO
	collision.signedDistance = Math::MAX<float>;
}

static void test_Triangle_Triangle( CollisionData& collision, const TransformSpace& faceA, const TransformSpace& faceB)
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

bool intersect_Unimplemented(const TransformSpace& a, const TransformSpace& b)
{
	return false;
}

static bool intersect_Sphere_Triangle(const TransformSpace& sphere, const TransformSpace& face)
{
	const mat4x4& xform = sphere.transform;
	const vec3f& origin = face.transform.origin;

	// Get UV coordinates of closest point on triangle
	vec4f clamped = clampTriangleUV(face.inverse * xform.origin);

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


// ---------------------------
// Other Functions
// ---------------------------

#define _mm_transform_ps( point, r0, r1, r2, r3 ) _mm_dot_ps( point, r0, point, r1, point, r2, point, r3 )

constexpr __m128 SIMD_X_AXIS = { 1.0f, 0.0f, 0.0f, 0.0f };
constexpr __m128 SIMD_Y_AXIS = { 0.0f, 1.0f, 0.0f, 0.0f };
constexpr __m128 SIMD_Z_AXIS = { 0.0f, 0.0f, 1.0f, 0.0f };

constexpr __m128 SIMD_BOX_P0 = { 1.0f, 1.0f, 1.0f, 1.0f };
constexpr __m128 SIMD_BOX_P1 = { 1.0f, 1.0f, -1.0f, 1.0f };
constexpr __m128 SIMD_BOX_P2 = { 1.0f, -1.0f, 1.0f, 1.0f };
constexpr __m128 SIMD_BOX_P3 = { 1.0f, -1.0f, -1.0f, 1.0f };
constexpr __m128 SIMD_BOX_P4 = { -1.0f, 1.0f, 1.0f, 1.0f };
constexpr __m128 SIMD_BOX_P5 = { -1.0f, 1.0f, -1.0f, 1.0f };
constexpr __m128 SIMD_BOX_P6 = { -1.0f, -1.0f, 1.0f, 1.0f };
constexpr __m128 SIMD_BOX_P7 = { -1.0f, -1.0f, -1.0f, 1.0f };

__m128 COMPLETE_AXIS_SEPARATION_TEST( const mat4f& t, const __m128& min, const __m128& max )
{
	__m128 _Tmp0 = _mm_shuffle_ps( t.x_axis.simd, t.y_axis.simd, 0x44 );
	__m128 _Tmp2 = _mm_shuffle_ps( t.x_axis.simd, t.y_axis.simd, 0xEE );
	__m128 _Tmp1 = _mm_shuffle_ps( t.z_axis.simd, t.origin.simd, 0x44 );
	__m128 _Tmp3 = _mm_shuffle_ps( t.z_axis.simd, t.origin.simd, 0xEE );
	__m128 r0 = _mm_shuffle_ps( _Tmp0, _Tmp1, 0x88 );
	__m128 r1 = _mm_shuffle_ps( _Tmp0, _Tmp1, 0xDD );
	__m128 r2 = _mm_shuffle_ps( _Tmp2, _Tmp3, 0x88 );
	__m128 r3 = _mm_shuffle_ps( _Tmp2, _Tmp3, 0xDD );

	__m128 minBounds = _mm_xyzw_ps( Math::MAX<float>, Math::MAX<float>, Math::MAX<float>, Math::MIN<float> );
	__m128 maxBounds = _mm_xyzw_ps( Math::MIN<float>, Math::MIN<float>, Math::MIN<float>, Math::MAX<float> );
	
	__m128 point = _mm_transform_ps( SIMD_BOX_P0, r0, r1, r2, r3 );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_transform_ps( SIMD_BOX_P1, r0, r1, r2, r3 );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_transform_ps( SIMD_BOX_P2, r0, r1, r2, r3 );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_transform_ps( SIMD_BOX_P3, r0, r1, r2, r3 );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_transform_ps( SIMD_BOX_P4, r0, r1, r2, r3 );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_transform_ps( SIMD_BOX_P5, r0, r1, r2, r3 );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_transform_ps( SIMD_BOX_P6, r0, r1, r2, r3 );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_transform_ps( SIMD_BOX_P7, r0, r1, r2, r3 );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	return _mm_or_ps( _mm_and_ps( _mm_cmpge_ps( minBounds, min ), _mm_cmple_ps( minBounds, max ) ), _mm_and_ps( _mm_cmpge_ps( min, minBounds ), _mm_cmple_ps( min, maxBounds ) ) );
}

__m128 AFFINE_AABB_SEPARATION_TEST( const mat4f& t, const __m128& min, const __m128& max )
{
	__m128 minBounds = _mm_xyzw_ps( Math::MAX<float>, Math::MAX<float>, Math::MAX<float>, Math::MIN<float> );
	__m128 maxBounds = _mm_xyzw_ps( Math::MIN<float>, Math::MIN<float>, Math::MIN<float>, Math::MAX<float> );
	__m128 point = _mm_add_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_add_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_add_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_add_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_sub_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_sub_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_sub_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_sub_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	return _mm_or_ps( _mm_and_ps( _mm_cmpge_ps( minBounds, min ), _mm_cmple_ps( minBounds, max ) ), _mm_and_ps( _mm_cmpge_ps( min, minBounds ), _mm_cmple_ps( min, maxBounds ) ) );
}

__m128 MIRRORED_AABB_SEPARATION_TEST( const mat4f& t, const __m128& max )
{
	// Create a sign vector to select the corner of the box
	__m128 comparison = _mm_cmpge_ps( t.origin.simd, SIMD_4f_ZERO );
	__m128 quadrant = _mm_or_ps( _mm_and_ps( comparison, SIMD_4f_ONES ), _mm_and_ps( _mm_xor_ps( comparison, reinterpret_cast<const __m128&>( SIMD_4i_NEG ) ), SIMD_4f_NEG ) );

	// Calculate the minimum bounds along each axis
	__m128 bounds = _mm_xyzw_ps( Math::MAX<float>, Math::MAX<float>, Math::MAX<float>, Math::MIN<float> );
	bounds = _mm_min_ps( bounds, _mm_mul_ps( quadrant, _mm_add_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd ) ) );
	bounds = _mm_min_ps( bounds, _mm_mul_ps( quadrant, _mm_add_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd ) ) );
	bounds = _mm_min_ps( bounds, _mm_mul_ps( quadrant, _mm_add_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd ) ) );
	bounds = _mm_min_ps( bounds, _mm_mul_ps( quadrant, _mm_add_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd ) ) );
	bounds = _mm_min_ps( bounds, _mm_mul_ps( quadrant, _mm_sub_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd ) ) );
	bounds = _mm_min_ps( bounds, _mm_mul_ps( quadrant, _mm_sub_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd ) ) );
	bounds = _mm_min_ps( bounds, _mm_mul_ps( quadrant, _mm_sub_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd ) ) );
	bounds = _mm_min_ps( bounds, _mm_mul_ps( quadrant, _mm_sub_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd ) ) );

	return _mm_cmple_ps( bounds, max );
}

vec3f linePlane( const vec4f& planeOrigin, const vec3f& planeNormal, const vec4f& lineOrigin, const vec3f& lineDirection )
{
	return
	{
		Math::dot( planeNormal, planeOrigin - lineOrigin ) / Math::dot( lineDirection, planeNormal ),
		Math::dot( planeNormal, planeOrigin - lineOrigin ) / Math::dot( lineDirection, planeNormal ),
		Math::dot( planeNormal, planeOrigin - lineOrigin ) / Math::dot( lineDirection, planeNormal )
	};
}

float Math::Triangle::rayDistance( const mat4f& triangle, const vec4f& lineOrigin, const vec3f& lineVector )
{
	__m128 displaced = _mm_sub_ps( triangle.origin.simd, lineOrigin.simd );
	__m128 normal = _mm_cross_ps( triangle.x_axis.simd, triangle.y_axis.simd );

	__m128 product = _mm_dot_ps( normal, displaced,
		_mm_cross_ps( lineVector.simd, triangle.y_axis.simd ), displaced,
		_mm_cross_ps( triangle.x_axis.simd, lineVector.simd ), displaced,
		normal, lineVector.simd );

	__m128 intersectData = _mm_div_ps( product, _mm_set1_ps( product.m128_f32[3] ) );
	vec3f& intersection = reinterpret_cast<vec3f&>( intersectData );
	return intersection.y >= 0.0f && intersection.z >= 0.0f && intersection.y + intersection.z <= 1.0f ? intersection.x : INFINITY;
}

vec3f Math::Triangle::rayIntersect( const mat4f& triangle, const vec4f& lineOrigin, const vec3f& lineVector )
{
	__m128 displaced = _mm_sub_ps( triangle.origin.simd, lineOrigin.simd );
	__m128 normal = _mm_cross_ps( triangle.x_axis.simd, triangle.y_axis.simd );

	__m128 product = _mm_dot_ps( normal, displaced, 
		_mm_cross_ps( lineVector.simd, triangle.y_axis.simd ), displaced, 
		_mm_cross_ps( triangle.x_axis.simd, lineVector.simd ), displaced, 
		normal, lineVector.simd );

	return { _mm_div_ps( product, _mm_set1_ps( product.m128_f32[3] ) ) };
}

vec4f Math::Box::rayCast( const vec4f& lineOrigin, const vec3f& lineVector )
{
	// Create a sign vector to select the corner of the box
	__m128 comparison = _mm_cmpge_ps( lineOrigin.simd, SIMD_4f_ZERO );
	__m128 quadrant = _mm_or_ps( _mm_and_ps( comparison, SIMD_4f_ONES ), _mm_and_ps( _mm_xor_ps( comparison, reinterpret_cast<const __m128&>( SIMD_4i_NEG ) ), SIMD_4f_NEG ) );

	// Calculate the vectors in the mirrored quadrant
	__m128 direction = _mm_mul_ps( quadrant, lineVector.simd );
	__m128 displaced = _mm_sub_ps( quadrant, _mm_mul_ps( quadrant, lineOrigin.simd ) );

	// Calculate UV vectors
	__m128 xProduct = _mm_cross_ps( direction, Math::axis::X<vec3f>.simd );
	__m128 yProduct = _mm_cross_ps( direction, Math::axis::Y<vec3f>.simd );
	__m128 zProduct = _mm_cross_ps( direction, Math::axis::Z<vec3f>.simd );

	// Calculate the TUV values for 3 line-plane intersections simultaneously
	__m128 denominator =         _mm_dot3_ps( SIMD_X_AXIS, direction, SIMD_Y_AXIS, direction, SIMD_Z_AXIS, direction );
	__m128 tValues = _mm_div_ps( _mm_dot3_ps( SIMD_X_AXIS, displaced, SIMD_Y_AXIS, displaced, SIMD_Z_AXIS, displaced ), denominator );
	__m128 uValues = _mm_div_ps( _mm_dot3_ps( yProduct,	   displaced, xProduct,    displaced, yProduct,    displaced ), denominator );
	__m128 vValues = _mm_div_ps( _mm_dot3_ps( zProduct,    displaced, zProduct,    displaced, zProduct,    displaced ), denominator );

	// Check if each intersection is within the bounds
	__m128 isValid = _mm_and_ps( _mm_cmpge_ps( tValues, SIMD_4f_ZERO ), _mm_and_ps(
		_mm_and_ps( _mm_cmpge_ps( uValues, SIMD_4f_NEG ), _mm_cmple_ps( uValues, SIMD_4f_ONES ) ), 
		_mm_and_ps( _mm_cmpge_ps( vValues, SIMD_4f_NEG ), _mm_cmple_ps( vValues, SIMD_4f_ONES ) ) ) );

	// Calculate the minimum t value
	float t = INFINITY;
	if( isValid.m128_u32[0] && tValues.m128_f32[0] < t ) t = tValues.m128_f32[0];
	if( isValid.m128_u32[1] && tValues.m128_f32[1] < t ) t = tValues.m128_f32[1];
	if( isValid.m128_u32[2] && tValues.m128_f32[2] < t ) t = tValues.m128_f32[2];

	// Calculate the intersecting point
	return { _mm_add_ps( lineOrigin.simd, _mm_mul_ps( lineVector.simd, _mm_set1_ps( t ) ) ) };
}

bool Math::Box::pointTest( const vec4f& point )
{
	__m128 isInside = _mm_and_ps( _mm_cmpge_ps( point.simd, SIMD_4f_NEG ), _mm_cmple_ps( point.simd, SIMD_4f_ONES ) );
	return isInside.m128_u32[0] && isInside.m128_u32[1] && isInside.m128_u32[2];
}

bool Math::Box::rayTest( const vec4f& lineOrigin, const vec3f& lineVector )
{
	if( pointTest( lineOrigin ) || pointTest( lineOrigin + lineVector ) ) return true;

	// Create a sign vector to select the corner of the box
	__m128 comparison = _mm_cmpge_ps( lineOrigin.simd, SIMD_4f_ZERO );
	__m128 quadrant = _mm_or_ps( _mm_and_ps( comparison, SIMD_4f_ONES ), _mm_and_ps( _mm_xor_ps( comparison, reinterpret_cast<const __m128&>( SIMD_4i_NEG ) ), SIMD_4f_NEG ) );

	// Calculate the vectors in the mirrored quadrant
	__m128 direction = _mm_mul_ps( quadrant, lineVector.simd );
	__m128 displaced = _mm_sub_ps( quadrant, _mm_mul_ps( quadrant, lineOrigin.simd ) );

	// Calculate UV vectors
	__m128 xProduct = _mm_cross_ps( direction, Math::axis::X<vec3f>.simd );
	__m128 yProduct = _mm_cross_ps( direction, Math::axis::Y<vec3f>.simd );
	__m128 zProduct = _mm_cross_ps( direction, Math::axis::Z<vec3f>.simd );

	// Calculate the TUV values for 3 line-plane intersections simultaneously
	__m128 denominator = _mm_dot3_ps( SIMD_X_AXIS, direction, SIMD_Y_AXIS, direction, SIMD_Z_AXIS, direction );
	__m128 xIntersect = _mm_div_ps( _mm_dot3_ps( SIMD_X_AXIS, displaced, yProduct, displaced, zProduct, displaced ), denominator );
	__m128 yIntersect = _mm_div_ps( _mm_dot3_ps( SIMD_Y_AXIS, displaced, xProduct, displaced, zProduct, displaced ), denominator );
	__m128 zIntersect = _mm_div_ps( _mm_dot3_ps( SIMD_Z_AXIS, displaced, xProduct, displaced, yProduct, displaced ), denominator );

	// Check if the intersection is within the correct bounds
	__m128 minBounds = _mm_xyzw_ps( 0.0f, -1.0f, -1.0f, 0.0f );
	__m128 isValid = _mm_and_ps( 
		_mm_and_ps( _mm_cmpge_ps( xIntersect, minBounds ), _mm_cmple_ps( xIntersect, SIMD_4f_ONES ) ), _mm_and_ps(
		_mm_and_ps( _mm_cmpge_ps( yIntersect, minBounds ), _mm_cmple_ps( yIntersect, SIMD_4f_ONES ) ),
		_mm_and_ps( _mm_cmpge_ps( zIntersect, minBounds ), _mm_cmple_ps( zIntersect, SIMD_4f_ONES ) ) ) );

	return isValid.m128_u32[0] || isValid.m128_u32[1] || isValid.m128_u32[2];
}

bool Math::Box::triangleTest( const TransformSpace& triangle )
{
	// Test triangle intersecting box
	{
		const mat4x4& tri = triangle.transform;

		// Create a sign vector to select the corner of the box
		__m128 comparison = _mm_cmpge_ps( tri.origin.simd, SIMD_4f_ZERO );
		__m128 quadrant = _mm_or_ps( _mm_and_ps( comparison, SIMD_4f_ONES ), _mm_and_ps( _mm_xor_ps( comparison, reinterpret_cast<const __m128&>( SIMD_4i_NEG ) ), SIMD_4f_NEG ) );

		__m128 p0 = _mm_mul_ps( tri.origin.simd, quadrant );
		__m128 p1 = _mm_mul_ps( tri.origin.simd, quadrant );
		__m128 p2 = _mm_mul_ps( tri.origin.simd, quadrant );

		__m128 minBounds = { Math::MAX<float>, Math::MAX<float>, Math::MAX<float>, 0.0f };
		minBounds = _mm_min_ps( minBounds, p0 );
		minBounds = _mm_min_ps( minBounds, p1 );
		minBounds = _mm_min_ps( minBounds, p2 );
		__m128 overlaps = _mm_cmple_ps( minBounds, SIMD_4f_ONES );

		if( overlaps.m128_f32[0] && overlaps.m128_f32[1] && overlaps.m128_f32[2] ) return true;
	}

	// Test box intersecting triangle
	__m128 overlaps = AFFINE_AABB_SEPARATION_TEST( triangle.inverse, _mm_xyzw_ps( 0.0f, 0.0f, 0.0f, 0.0f ), _mm_xyzw_ps( 1.0f, 1.0f, 0.0f, 0.0f ) );
	return overlaps.m128_u32[0] && overlaps.m128_u32[1] && overlaps.m128_u32[2];
}

bool Math::Box::boxTest( const TransformSpace& box )
{
	// Test regular overlap
	__m128 overlaps = MIRRORED_AABB_SEPARATION_TEST( box.transform, SIMD_4f_ONES );
	if( overlaps.m128_u32[0] && overlaps.m128_u32[1] && overlaps.m128_u32[2] ) return true;

	overlaps = MIRRORED_AABB_SEPARATION_TEST( box.inverse, SIMD_4f_ONES );
	return overlaps.m128_u32[0] && overlaps.m128_u32[1] && overlaps.m128_u32[2];
}

vec3f Math::Triangle::sphereDisplace( const TransformSpace& sphere, const TransformSpace& face )
{
	vec3f uvs = face.inverse * sphere.transform.origin;
	vec4f bounded = Math::max( uvs, Math::ZERO<vec4f> );
	vec4f clamped = bounded.x + bounded.y > 1.0f ? bounded / ( bounded.x + bounded.y ) : bounded;

	if( clamped.z < 0.0f ) return Math::ZERO<vec3f>;

	mat4x4 relFace = sphere.inverse * face.transform;
	vec3f compNormal = Math::axis::W<vec4f> - ( relFace.origin + relFace.x_axis * uvs.x + relFace.y_axis * uvs.y );
	vec3f compCombined = Math::axis::W<vec4f> - ( relFace.origin + relFace.x_axis * clamped.x + relFace.y_axis * clamped.y );
	vec3f compSurface = compCombined - compNormal;

	if( Math::length2( compSurface ) > 1.0f ) return Math::ZERO<vec3f>;

	float x = sqrtf( 1.0f - Math::length2( compSurface ) ) - Math::length( compNormal );
	return sphere.transform * ( Math::normalize( relFace.z_axis ) * x );
}

/*vec3f sphereDisplace2(const mat4x4 sphere, const TransformSpace& face)
{
	vec3f uvs = face.inverse * sphere.origin;
	vec4f bounded = Math::max( uvs, Math::ZERO<vec4f> );
	vec4f clamped = bounded.x + bounded.y > 1.0f ? bounded / ( bounded.x + bounded.y ) : bounded;

	if( clamped.z < 0.0f ) return Math::ZERO<vec3f>;

	vec3f displacement = sphere.origin - face.transform.origin;

}*/


// ---------------------------
// Uniform Collision Field
// ---------------------------

void calculateTriangleBounds( Bounds<vec3f>& bounds, const mat4x4& t )
{
	bounds.min = t.origin;
	bounds.max = t.origin;

	__m128 point = _mm_add_ps( t.origin.simd, t.x_axis.simd );
	bounds.min.simd = _mm_min_ps( bounds.min.simd, point );
	bounds.min.simd = _mm_max_ps( bounds.min.simd, point );

	point = _mm_add_ps( t.origin.simd, t.y_axis.simd );
	bounds.min.simd = _mm_min_ps( bounds.min.simd, point );
	bounds.min.simd = _mm_max_ps( bounds.min.simd, point );
}

Bounds<vec3f> Math::Box::calculateAABB( const mat4x4& t )
{
	Bounds<vec3f> bounds;

	__m128 point = _mm_add_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	bounds.min.simd = point;
	bounds.max.simd = point;

	point = _mm_add_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	bounds.min.simd = _mm_min_ps( bounds.min.simd, point );
	bounds.max.simd = _mm_max_ps( bounds.max.simd, point );

	point = _mm_add_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	bounds.min.simd = _mm_min_ps( bounds.min.simd, point );
	bounds.max.simd = _mm_max_ps( bounds.max.simd, point );

	point = _mm_add_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	bounds.min.simd = _mm_min_ps( bounds.min.simd, point );
	bounds.max.simd = _mm_max_ps( bounds.max.simd, point );

	point = _mm_sub_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	bounds.min.simd = _mm_min_ps( bounds.min.simd, point );
	bounds.max.simd = _mm_max_ps( bounds.max.simd, point );

	point = _mm_sub_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	bounds.min.simd = _mm_min_ps( bounds.min.simd, point );
	bounds.max.simd = _mm_max_ps( bounds.max.simd, point );

	point = _mm_sub_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	bounds.min.simd = _mm_min_ps( bounds.min.simd, point );
	bounds.max.simd = _mm_max_ps( bounds.max.simd, point );

	point = _mm_sub_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	bounds.min.simd = _mm_min_ps( bounds.min.simd, point );
	bounds.max.simd = _mm_max_ps( bounds.max.simd, point );

	return bounds;
}

mat4x4 calculateBoundingTransform( const MeshCollider& rawCollision )
{
	vec3f min = Math::MAX<vec3f>;
	vec3f max = Math::MIN<vec3f>;

	for( const CollisionFace& face : rawCollision )
	{
		const vec4f& p0 = face.transform.origin;
		const vec4f p1 = p0 + face.transform.x_axis;
		const vec4f p2 = p0 + face.transform.y_axis;

		min = Math::min( min, p0 );
		min = Math::min( min, p1 );
		min = Math::min( min, p2 );
		max = Math::max( max, p0 );
		max = Math::max( max, p1 );
		max = Math::max( max, p2 );
	}

	vec3f midpoint = ( max + min ) / 2.0f;
	vec3f scale = ( max - min ) / 2.0f;
	return { { scale.x, 0.0f, 0.0f }, { 0.0f, scale.y, 0.0f }, { 0.0f, 0.0f, scale.z }, midpoint };
}

mat4x4 calculateBoundingTransform( const std::vector<MeshCollider>& collision )
{
	vec3f min = Math::MAX<vec3f>;
	vec3f max = Math::MIN<vec3f>;

	for( const MeshCollider& collider : collision )
	{
		for( const CollisionFace& face : collider )
		{
			const vec4f& p0 = face.transform.origin;
			const vec4f p1 = p0 + face.transform.x_axis;
			const vec4f p2 = p0 + face.transform.y_axis;

			min = Math::min( min, p0 );
			min = Math::min( min, p1 );
			min = Math::min( min, p2 );
			max = Math::max( max, p0 );
			max = Math::max( max, p1 );
			max = Math::max( max, p2 );
		}
	}

	vec3f midpoint = ( max + min ) / 2.0f;
	vec3f scale = ( max - min ) / 2.0f;
	return { { scale.x, 0.0f, 0.0f }, { 0.0f, scale.y, 0.0f }, { 0.0f, 0.0f, scale.z }, midpoint };
}

void UniformCollisionField::addTriangle( const TransformSpace& triangle )
{
	Bounds<vec3f> triangleBounds;
	TransformSpace relative = { unitSpace.inverse * triangle.transform, triangle.inverse * unitSpace.transform };
	TransformSpace unit = relative;
	
	calculateTriangleBounds( triangleBounds, relative.transform );
	vec3f originIdx = relative.transform.origin;
	vec3f originInv = relative.inverse.origin;

	vec3i minIdx = Math::clamp( vec3i( Math::floor( triangleBounds.min ) ), Math::ZERO<vec3i>, dimensions );
	vec3i maxIdx = Math::clamp( vec3i( Math::floor( triangleBounds.max ) ) + Math::ONES<vec4i>, Math::ZERO<vec3i>, dimensions );

	// Populate the units with any intersecting face 
	for( vec3i idx = minIdx; idx.z < maxIdx.z; ++idx.z )
	{
		size_t iz = idx.u_z * dimensions.u_y;
		for( idx.y = minIdx.y; idx.y < maxIdx.y; ++idx.y )
		{
			size_t iy = ( iz + idx.u_y ) * dimensions.u_x;
			for( idx.x = minIdx.x; idx.x < maxIdx.x; ++idx.x )
			{
				vec3f fIdx = vec3f( idx );
				unit.transform.origin = originIdx - fIdx;
				unit.inverse = unit.transform.inverse();

				if( Math::Box::triangleTest( unit ) )
				{
					units[iy + idx.u_x].faces.emplace_back( relative );
				}
			}
		}
	}
}

const UniformCollisionField& CollisionMap::add( const MeshCollider& meshCollision, const vec3i& dimensions )
{
	mat4x4 bounds = calculateBoundingTransform( meshCollision );
	UniformCollisionField& field = fields.emplace_back( bounds, dimensions );

	for( const CollisionFace& face : meshCollision )
	{
		field.addTriangle( face );
	}

	return field;
}

const UniformCollisionField& CollisionMap::add( const std::vector<MeshCollider>& meshCollision, const vec3i& dimensions )
{
	mat4x4 bounds = calculateBoundingTransform( meshCollision );
	UniformCollisionField& field = fields.emplace_back( bounds, dimensions );

	for( const MeshCollider& collider : meshCollision )
	{
		for( const CollisionFace& face : meshCollision )
		{
			field.addTriangle( face );
		}
	}

	return field;
}

vec4i CollisionMap::getPointID( const vec4f& point ) const
{
	for( size_t i = 0; i < fields.size(); ++i )
	{
		const UniformCollisionField& field = fields[i];
		vec4i index = vec3i( Math::floor( field.unitSpace.inverse * point ) );
		vec3i cmp = index >= Math::ZERO<vec3i> && index < field.dimensions;

		if( cmp.x && cmp.y && cmp.z )
		{
			index.w = static_cast<uint32_t>( i );
			return index;
		}
	}

	return Math::NEGATIVE<vec4f>;
}

void CollisionUnit::collision( TransformSpace& relativeSphere ) const
{
	for( const CollisionFace& face : faces )
	{
		Math::translate( relativeSphere, Math::Triangle::sphereDisplace( relativeSphere, face ) );
	}
}

void UniformCollisionField::collision( TransformSpace& sphere ) const
{
	TransformSpace relative = { unitSpace.inverse * sphere.transform, sphere.inverse * unitSpace.transform };
	Bounds<vec3f> aabb = Math::Box::calculateAABB( relative.transform );
	vec4f startOrigin = relative.transform.origin;

	vec3i minIdx = Math::clamp( vec3i( Math::floor( aabb.min ) ), Math::ZERO<vec3i>, dimensions );
	vec3i maxIdx = Math::clamp( vec3i( Math::floor( aabb.max ) ) + Math::ONES<vec4i>, Math::ZERO<vec3i>, dimensions );

	for( vec3i idx = minIdx; idx.z < maxIdx.z; ++idx.z )
	{
		size_t iz = idx.u_z * dimensions.u_y;
		for( idx.y = minIdx.y; idx.y < maxIdx.y; ++idx.y )
		{
			size_t iy = ( iz + idx.u_y ) * dimensions.u_x;
			for( idx.x = minIdx.x; idx.x < maxIdx.x; ++idx.x )
			{
				units[iy + idx.u_x].collision( relative );
			}
		}
	}

	Math::translate( sphere, unitSpace.transform * ( relative.transform.origin - startOrigin ) );
}

void CollisionMap::collision( TransformSpace& sphere ) const
{
	Bounds<vec3f> aabb = Math::Box::calculateAABB( sphere.transform );

	for( const UniformCollisionField& field : fields )
	{
		if( Math::overlaps( aabb.min, aabb.max, field.aabb.min, field.aabb.max ) )
		{
			field.collision( sphere );
		}
	}
}