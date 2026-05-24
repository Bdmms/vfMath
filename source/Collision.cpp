#include "../include/vfMath/Collision.hpp"

#if defined(VF_DEBUG_TRIANGLE_LATTICE) | defined(VF_RAYCAST_DEBUG)
#include <iostream>
#endif

typedef bool ( *IntersectTest ) ( const Collider& a, const Collider& b );

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

/**
 * @brief Returns the index of the vector's component with the largest value.
 * @param vector - 3D vector
 * @return component index
*/
constexpr uint8_t maxIndex( const vec4f& vector )
{
	if( vector.x >= vector.y )	return vector.z >= vector.x ? 2u : 0u;
	else						return vector.z >= vector.y ? 2u : 1u;
}

/**
 * @brief Returns the index of the vector's component with the smallest value.
 * @param vector - 3D vector
 * @return component index
*/
constexpr uint8_t minIndex( const vec4f& vector )
{
	if( vector.x <= vector.y )	return vector.z <= vector.x ? 2u : 0u;
	else						return vector.z <= vector.y ? 2u : 1u;
}

// ---------------------------
// Collision Mesh Primitives
// ---------------------------

CollisionFace createFace(const vec4f& p0, const vec4f& p1, const vec4f& p2)
{
	vec4f xAxis = p1 - p0;
	vec4f yAxis = p2 - p0;
	mat4x4 transform{
		xAxis,
		yAxis,
		Math::normalize(Math::cross(xAxis, yAxis)),
		vec4f{ p0.x, p0.y, p0.z, 1.0f }
	};
	return { transform, Math::invert( transform ) };
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

std::vector<CollisionFace> Collision::createCubeMesh()
{
	std::vector<CollisionFace> collider(12);
	CollisionFace* ptr = collider.data();

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

// Epsilon used when compiling triangles in lattice
constexpr float P_EPSILON = 2E-5f;
constexpr float N_EPSILON = -P_EPSILON;
constexpr float ONE_EPSILON = 1.0f + P_EPSILON;

__m128 COMPLETE_AXIS_SEPARATION_TEST( const mat4f& t, const vec4f& min, const vec4f& max )
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

	return _mm_or_ps( _mm_and_ps( _mm_cmpge_ps( minBounds, min.simd ), _mm_cmple_ps( minBounds, max.simd ) ), 
		_mm_and_ps( _mm_cmpge_ps( min.simd, minBounds ), _mm_cmple_ps( min.simd, maxBounds ) ) );
}

__m128 AFFINE_AABB_SEPARATION_TEST( const mat4f& t, const vec4f& min, const vec4f& max )
{
	__m128 point = _mm_add_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	__m128 minBounds = point;
	__m128 maxBounds = point;

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

	return _mm_or_ps( _mm_and_ps( _mm_cmpge_ps( minBounds, min.simd ), _mm_cmple_ps( minBounds, max.simd ) ), 
		_mm_and_ps( _mm_cmpge_ps( min.simd, minBounds ), _mm_cmple_ps( min.simd, maxBounds ) ) );
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

constexpr vec4f PROJECTED_AABB_SEPARATION_TEST( const mat4f& t, const vec4f& min, const vec4f& max )
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

bool Math::AABB::rayTest( const vec4f& min, const vec4f& max, const vec4f& point, const vec4f& ray )
{
	vec4f intersect0 = ( min - point ) / ray;
	vec4f intersect1 = ( max - point ) / ray;

	vec4f tmin = Math::condition( ray == Math::ZERO<vec4f>, Math::ZERO<vec4f>, Math::min( intersect0, intersect1 ) );
	vec4f tmax = Math::condition( ray == Math::ZERO<vec4f>, Math::ONES<vec4f>, Math::max( intersect0, intersect1 ) );

	float minBounds = std::max( std::max( tmin.x, tmin.y ), std::max( tmin.z, 0.0f ) );
	float maxBounds = std::min( std::min( tmax.x, tmax.y ), std::min( tmax.z, 1.0f ) );
	return minBounds <= maxBounds;
}

Bounds<float> Math::AABB::rayBounds( const vec4f& min, const vec4f& max, const vec4f& point, const vec4f& ray )
{
	vec4f intersect0 = ( min - point ) / ray;
	vec4f intersect1 = ( max - point ) / ray;

	vec4f tmin = Math::condition( ray == Math::ZERO<vec4f>, Math::ZERO<vec4f>, Math::min( intersect0, intersect1 ) );
	vec4f tmax = Math::condition( ray == Math::ZERO<vec4f>, Math::ONES<vec4f>, Math::max( intersect0, intersect1 ) );

	return
	{
		std::clamp( std::max( std::max( tmin.x, tmin.y ), tmin.z ), 0.0f, 1.0f ),
		std::clamp( std::min( std::min( tmax.x, tmax.y ), tmax.z ), 0.0f, 1.0f )
	};
}

float Math::Triangle::rayDistance( const mat4f& triangle, const vec4f& rayOrigin, const vec4f& rayVector )
{
	// Calculate the line-plane intersection
	vec4f displaced = triangle.origin - rayOrigin;
	vec4f normal = Math::cross( triangle.x_axis, triangle.y_axis );
	vec4f product = Math::parallel::dot(
		normal, displaced,
		Math::cross( rayVector, triangle.y_axis ), displaced,
		Math::cross( triangle.x_axis, rayVector ), displaced,
		normal, rayVector
	);

	// Check that the plane intersection hits the region of the triangle
	vec4f intersection = product / product.w;
	return intersection.y >= 0.0f && intersection.z >= 0.0f && intersection.y + intersection.z <= 1.0f ? intersection.x : INFINITY;
}

bool Math::Box::pointTest( const vec4f& point )
{
	vec4f isInside = point >= Math::NEGATIVE<vec4f> && point <= Math::ONES<vec4f>;
	return isInside.x && isInside.y && isInside.z;
}

bool Math::Box::rayTest( const vec4f& origin, const vec4f& ray )
{
	return Math::AABB::rayTest( Math::NEGATIVE<vec4f>, Math::ONES<vec4f>, origin, ray );
}

bool Math::Box::triangleTest( const TransformSpace& triangle )
{
	// Test triangle overlapping box
	const mat4x4& tri = triangle.transform;
	vec4f min, max;

	AABB_TRIANGLE( tri, min, max );

	vec4f minBounds( -ONE_EPSILON, -ONE_EPSILON, -ONE_EPSILON, 0.0f );
	vec4f maxBounds(  ONE_EPSILON,  ONE_EPSILON,  ONE_EPSILON, 0.0f );
	vec4f overlaps = Math::overlaps( min, max, minBounds, maxBounds );
	if( !( overlaps.x && overlaps.y && overlaps.z ) ) return false;
	
	// Test box overlapping triangle
	AABB_BOX_FULL( triangle.inverse, min, max );

	return max.x >= N_EPSILON && max.y >= N_EPSILON
		&& min.x + min.y <= ONE_EPSILON && min.z <= P_EPSILON && max.z >= N_EPSILON;
}

bool Math::Box::boxTest( const TransformSpace& box )
{
	// Test regular overlap
	vec4f overlaps = MIRRORED_AABB_SEPARATION_TEST( box.transform, Math::ONES<vec4f> );
	if( !( overlaps.x && overlaps.y && overlaps.z ) ) return false;

	overlaps = MIRRORED_AABB_SEPARATION_TEST( box.inverse, Math::ONES<vec4f> );
	return overlaps.x && overlaps.y && overlaps.z;
}

bool Math::Box::projectedBoxTest( const TransformSpace& box )
{
	// Test regular overlap
	vec4f overlaps = PROJECTED_AABB_SEPARATION_TEST( box.transform, Math::NEGATIVE<vec4f>, Math::ONES<vec4f> );
	if( !( overlaps.x && overlaps.y && overlaps.z ) ) return false;

	overlaps = PROJECTED_AABB_SEPARATION_TEST( box.inverse, Math::NEGATIVE<vec4f>, Math::ONES<vec4f> );
	return overlaps.x && overlaps.y && overlaps.z;
}

bool Math::Sphere::sphereTest( const TransformSpace& sphere )
{
	vec4f displacement = sphere.transform.origin;
	displacement.w = 0.0f;

	float distance = Math::length( displacement );
	if( distance <= Math::EPSILON<float> ) return true;

	float radius = Math::length( sphere.transform * Math::normalize( sphere.inverse * displacement ) );

	// Test if the distance is less than the radius of both spheres
	return distance <= radius + 1.0f;
}

bool Math::Sphere::boxTest( const TransformSpace& box )
{
	vec4f closestPoint = Math::clamp( box.inverse.origin, Math::NEGATIVE<vec4f>, Math::ONES<vec4f> );
	vec4f displacement = box.transform * closestPoint;
	float distance2 = displacement.x * displacement.x + displacement.y * displacement.y + displacement.z * displacement.z;
	return distance2 <= 1.0f;
}

bool sphereFaceCollision( vec4f& displacement, const TransformSpace& sphere, const TransformSpace& face )
{
	vec4f uvs = face.inverse * sphere.transform.origin;
	if( uvs.x + uvs.y > 1.0f || uvs.x < 0.0f || uvs.y < 0.0f ) return false;

	vec4f bounded = Math::max( uvs, Math::ZERO<vec4f> );
	vec4f clamped = bounded.x + bounded.y > 1.0f ? bounded / ( bounded.x + bounded.y ) : bounded;
	mat4x4 relFace = sphere.inverse * face.transform;

	vec4f compNormal = relFace.origin + relFace.x_axis * uvs.x + relFace.y_axis * uvs.y;
	vec4f compCombined = relFace.origin + relFace.x_axis * clamped.x + relFace.y_axis * clamped.y;
	vec4f compSurface = compCombined - compNormal;
	float surfaceDistance2 = Math::dot_3D( compSurface, compSurface );
	float normalDistance2 = Math::dot_3D( compNormal, compNormal );
	
	if( surfaceDistance2 > 1.0f ) return false;

	float x = sqrtf( 1.0f - surfaceDistance2 ) - sqrtf( normalDistance2 );
	displacement = sphere.transform * ( Math::normalize( relFace.z_axis ) * x );

	return Math::dot_3D( compCombined, compCombined ) <= 1.0f;
}

bool Math::Box::sphereCollision( vec4f& displacement, const TransformSpace& sphere )
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
	vec4f lengthBoxSpace = Math::parallel::length( vectorBoxSpace[0], vectorBoxSpace[1], vectorBoxSpace[2] );

	uint8_t idx = minIndex( lengthBoxSpace );
	displacement = sphere.transform * vectorBoxSpace[idx];
	float distance2 = Math::length2( displacement );
	float radius2 = Math::length2( sphere.transform * ( vectorBoxSpace[idx] / lengthBoxSpace[idx] ) );

	return distance2 <= radius2;
}

bool Math::Sphere::boxCollision( vec4f& displacement, const TransformSpace& box )
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

bool Math::Sphere::sphereCollision( vec4f& displacement, const TransformSpace& sphere )
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

// --------------------------
// --- Intersection Table ---
// --------------------------

bool intersect_AABB( const Collider& a, const Collider& b )
{
	return Math::overlaps3D( a.aabb, b.aabb );
}

bool intersect_Box_Box( const Collider& a, const Collider& b )
{
	return Math::overlaps3D( a.aabb, b.aabb ) &&
		Math::Box::boxTest( { a.current.inverse * b.current.transform, b.current.inverse * a.current.transform } );
}

bool intersect_Box_Sphere( const Collider& a, const Collider& b )
{
	return Math::overlaps3D( a.aabb, b.aabb ) &&
		Math::Sphere::boxTest( { b.current.inverse * a.current.transform, a.current.inverse * b.current.transform } );
}

bool intersect_Sphere_Box( const Collider& a, const Collider& b )
{
	return Math::overlaps3D( a.aabb, b.aabb ) &&
		Math::Sphere::boxTest( { a.current.inverse * b.current.transform, b.current.inverse * a.current.transform } );
}

bool intersect_Sphere_Sphere( const Collider& a, const Collider& b )
{
	return Math::overlaps3D( a.aabb, b.aabb ) &&
		Math::Sphere::sphereTest( { a.current.inverse * b.current.transform, b.current.inverse * a.current.transform } );
}

const IntersectTest intersectionMatrix[4][4]
{
	{ intersect_AABB, intersect_AABB,		intersect_AABB,			 intersect_AABB },
	{ intersect_AABB, intersect_Box_Box,	intersect_Box_Sphere,	 intersect_AABB },
	{ intersect_AABB, intersect_Sphere_Box, intersect_Sphere_Sphere, intersect_AABB },
	{ intersect_AABB, intersect_AABB,		intersect_AABB,			 intersect_AABB },
};

// -------------------------
// --- General Collision ---
// -------------------------

Bounds<vec4f> Math::Box::calculateAABB( const mat4x4& t )
{
	Bounds<vec4f> bounds;
	AABB_BOX_FULL( t, bounds.min, bounds.max );
	return bounds;
}

Bounds<vec4f> Math::Triangle::calculateAABB( const mat4x4& t )
{
	Bounds<vec4f> bounds;
	AABB_TRIANGLE( t, bounds.min, bounds.max );
	return bounds;
}

/**
 * This function converts a 2D array into a 1D array of faces.
*/
void packTriangles( CollisionLatticeData& lattice, const std::vector<std::vector<CollisionFace>>& unitFaces )
{
	for( size_t i = 0LLU; i < lattice.units.size(); ++i )
	{
		const std::vector<CollisionFace>& unitList = unitFaces[i];
		CollisionLatticeData::Unit& unit = lattice.units[i];

		// Store the point in the list used by the unit
		unit.idx = lattice.faces.size();
		for( const CollisionFace& face : unitList )
		{
			lattice.faces.emplace_back( face );
		}
		unit.size = lattice.faces.size() - unit.idx;
	}
}

/**
 * This function determines which units a triangle intersects with.
*/
void addTriangle( std::vector<std::vector<CollisionFace>>& unitFaces, const TransformSpace& triangle, const CollisionLatticeData& lattice )
{
	const vec4i& dimensions = lattice.getDimensions();
	const TransformSpace relative = { lattice.defaultBounds.inverse * triangle.transform, triangle.inverse * lattice.defaultBounds.transform };
	const TransformSpace unitXform = { lattice.unitSpace.inverse * relative.transform, relative.inverse * lattice.unitSpace.transform };
	TransformSpace unit = { Math::create::scale( vec4f{ 0.5f, 0.5f, 0.5f } ), Math::create::scale( vec4f{ 2.0f, 2.0f, 2.0f } ) };
	TransformSpace unitTriangle;
	
	Bounds<vec4f> triangleBounds;
	AABB_TRIANGLE( unitXform.transform, triangleBounds.min, triangleBounds.max );

	vec4i minIdx = Math::clamp( static_cast<vec4i>( Math::floor( triangleBounds.min - P_EPSILON ) ), Math::ZERO<vec4i>, dimensions - Math::ONES<vec4i> );
	vec4i maxIdx = Math::clamp( static_cast<vec4i>( Math::ceil(  triangleBounds.max + P_EPSILON ) ), Math::ONES<vec4i>, dimensions );
	minIdx.w = 0;
	maxIdx.w = 0;

	// Populate the units with any intersecting face 
	for( vec4i idx = minIdx; idx.z < maxIdx.z; ++idx.z )
	{
		size_t iz = idx.z * dimensions.y;
		for( idx.y = minIdx.y; idx.y < maxIdx.y; ++idx.y )
		{
			size_t iy = ( iz + idx.y ) * dimensions.x;
			for( idx.x = minIdx.x; idx.x < maxIdx.x; ++idx.x )
			{
				unit.transform.origin = static_cast<vec4f>( idx ) + vec4f{ 0.5f, 0.5f, 0.5f, 1.0f };
				unit.inverse.origin = unit.transform.origin * vec4f{ -2.0f, -2.0f, -2.0f, 1.0f };
				unit.inverse = Math::invert( unit.transform );

				unitTriangle.transform = unit.inverse * unitXform.transform;
				unitTriangle.inverse = unitXform.inverse * unit.transform;

				if( Math::Box::triangleTest( unitTriangle ) )
				{
					unitFaces[iy + idx.x].emplace_back( unitXform );
				}

#ifdef VF_DEBUG_TRIANGLE_LATTICE
				else
				{
					vec4f v0 = unitXform.transform.origin;
					vec4f v1 = unitXform.transform.origin + unitXform.transform.x_axis;
					vec4f v2 = unitXform.transform.origin + unitXform.transform.y_axis;

					vec4i p0 = vec4i( Math::floor( v0 ) );
					vec4i p1 = vec4i( Math::floor( v1 ) );
					vec4i p2 = vec4i( Math::floor( v2 ) );

					vec4i a = idx ^ p0;
					vec4i b = idx ^ p1;
					vec4i c = idx ^ p2;
					bool expected = !( a.x | a.y | a.z ) || !( b.x | b.y | b.z ) || !( c.x | c.y | c.z );

					if( expected )
					{
						std::cout << "--- FAILED ---\n";
						std::cout << idx.x << ", " << idx.y << ", " << idx.z << "\n";

						std::cout << p0.x << ", " << p0.y << ", " << p0.z << "\n";
						std::cout << p1.x << ", " << p1.y << ", " << p1.z << "\n";
						std::cout << p2.x << ", " << p2.y << ", " << p2.z << "\n";

						std::cout << v0.x << ", " << v0.y << ", " << v0.z << "\n";
						std::cout << v1.x << ", " << v1.y << ", " << v1.z << "\n";
						std::cout << v2.x << ", " << v2.y << ", " << v2.z << "\n";

						v0 = unitTriangle.transform.origin;
						v1 = unitTriangle.transform.origin + unitTriangle.transform.x_axis;
						v2 = unitTriangle.transform.origin + unitTriangle.transform.y_axis;

						std::cout << v0.x << ", " << v0.y << ", " << v0.z << "\n";
						std::cout << v1.x << ", " << v1.y << ", " << v1.z << "\n";
						std::cout << v2.x << ", " << v2.y << ", " << v2.z << "\n";

						Math::Box::triangleTest( unitTriangle );
					}
				}
#endif
			}
		}
	}
}

void Collision::convertToLattice( CollisionLatticeData& lattice, const std::vector<CollisionFace>& mesh, const vec4i& dimensions )
{
	lattice.resize( dimensions );
	Bounds<vec4f> aabb = { Math::MAX<vec4f>, Math::MIN<vec4f> };
	Math::extend( aabb, mesh );

	lattice.defaultBounds.transform = Math::createBoundingTransform( aabb );
	lattice.defaultBounds.inverse = Math::createBoundingInverseTransform( aabb );

	std::vector<std::vector<CollisionFace>> unitFaces( lattice.getUnitCount() );
	for( const CollisionFace& face : mesh )
	{
		addTriangle( unitFaces, face, lattice );
	}
	packTriangles( lattice, unitFaces );
}

void Collision::convertToLattice( CollisionLatticeData& lattice, const std::vector<std::vector<CollisionFace>>& mesh, const vec4i& dimensions )
{
	lattice.resize( dimensions );
	Bounds<vec4f> aabb = { Math::MAX<vec4f>, Math::MIN<vec4f> };

	for( const std::vector<CollisionFace>& group : mesh )
	{
		Math::extend( aabb, group );
	}

	lattice.defaultBounds.transform = Math::createBoundingTransform( aabb );
	lattice.defaultBounds.inverse = Math::createBoundingInverseTransform( aabb );

	std::vector<std::vector<CollisionFace>> unitFaces( lattice.getUnitCount() );
	for( const std::vector<CollisionFace>& group : mesh )
	{
		for( const CollisionFace& face : group )
		{
			addTriangle( unitFaces, face, lattice );
		}
	}
	packTriangles( lattice, unitFaces );
}


CollisionMesh::CollisionMesh( const std::vector<CollisionFace>& source ) : 
	ColliderSpace(),
	faces()
{
	setFaces( source );
}

void CollisionMesh::setFaces( const std::vector<CollisionFace>& source )
{
	aabb = { Math::MAX<vec4f>, Math::MIN<vec4f> };
	Math::extend( aabb, source );

	current.transform = Math::createBoundingTransform( aabb );
	current.inverse = Math::createBoundingInverseTransform( aabb );
	previous = current;

	faces.resize( source.size() );
	for( size_t i = 0; i < source.size(); ++i )
	{
		faces[i].transform = current.inverse * source[i].transform;
		faces[i].inverse = Math::invert( faces[i].transform );
	}
}

void CollisionMesh::collision( TransformSpace& sphere ) const
{
	TransformSpace relative = { current.inverse * sphere.transform, sphere.inverse * current.transform };
	vec4f startOrigin = relative.transform.origin;
	vec4f displacement;

	for( const CollisionFace& face : faces )
	{
		if( sphereFaceCollision( displacement, relative, face ) )
		{
			Math::translate( relative, displacement );
		}
	}

	Math::translate( sphere, current.transform * ( relative.transform.origin - startOrigin ) );
}

void collision_CollisionLatticeUnit_sphere( TransformSpace& relativeSphere, const CollisionFace* ptr, const CollisionFace* end )
{
	vec4f displacement;
	while( ptr < end )
	{
		// If sphere intersects face, translate sphere by face displacement
		if( sphereFaceCollision( displacement, relativeSphere, *ptr ) )
		{
			Math::translate( relativeSphere, displacement );
		}
		++ptr;
	}
}

void CollisionLattice::collision( TransformSpace& sphere ) const
{
	const TransformSpace& unitSpace = data.getUnitSpace();
	const vec4i& dimensions = data.getDimensions();
	const CollisionLatticeData::Unit* units = data.getUnits();
	const CollisionFace* faces = data.getFaces();

	// Convert the sphere into unit space
	TransformSpace relative = { current.inverse * sphere.transform, sphere.inverse * current.transform };
	TransformSpace unitXform = { unitSpace.inverse * relative.transform, relative.inverse * unitSpace.transform };
	vec4f startOrigin = unitXform.transform.origin;

	// Compute the bounds of the sphere in unit space
	Bounds<vec4f> aabb = Math::Box::calculateAABB( unitXform.transform );
	vec4i minIdx = Math::clamp( static_cast<vec4i>( Math::floor( aabb.min ) ), Math::ZERO<vec4i>, dimensions );
	vec4i maxIdx = Math::clamp( static_cast<vec4i>( Math::ceil(  aabb.max ) ), Math::ZERO<vec4i>, dimensions );

	// Iterate over the units the sphere overlaps with
	for( vec4i idx = minIdx; idx.z < maxIdx.z; ++idx.z )
	{
		size_t iz = idx.z * dimensions.y;
		for( idx.y = minIdx.y; idx.y < maxIdx.y; ++idx.y )
		{
			size_t iy = ( iz + idx.y ) * dimensions.x;
			for( idx.x = minIdx.x; idx.x < maxIdx.x; ++idx.x )
			{
				// Process the collision within the unit
				const CollisionLatticeData::Unit& unit = units[iy + idx.x];
				const CollisionFace* face = faces + unit.idx;
				collision_CollisionLatticeUnit_sphere( unitXform, face, face + unit.size );
			}
		}
	}

	// Propagate the translation of the collision back into world space
	Math::translate( sphere, current.transform * ( unitSpace.transform * ( unitXform.transform.origin - startOrigin ) ) );
}

bool CollisionMesh::rayCast( RaySensor& ray ) const
{
	// Convert finite ray into mesh space
	vec4f origin = current.inverse * ray.origin;
	vec4f line = current.inverse * ( ray.direction * ray.distance );

	if( !Math::Box::rayTest( origin, line ) ) return false;

	// We need to make sure the distance/normal is calculated relative to the mesh space
	float relDistance = 1.0f;
	vec4f relNormal;
	bool hit = false;

	for( const CollisionFace& face : faces )
	{
		// Rays only intersect with front-facing faces
		if( Math::dot_3D( face.transform.z_axis, line ) > 0.0f ) continue;

		// Check that the distance is postive and less than the current distance
		float rayDistance = Math::Triangle::rayDistance( face.transform, origin, line );
		if( rayDistance >= 0.0f && rayDistance <= relDistance )
		{
			relDistance = rayDistance;
			relNormal = face.transform.z_axis;
			hit = true;
		}
	}

	if( hit )
	{
		// Convert the distance/normal back into world space
		ray.normal = Math::normalize( current.transform * relNormal );
		ray.distance = Math::length( current.transform * ( line * relDistance ) );
		ray.space = &current;
		ray.hit = true;
		return true;
	}

	return false;
}

bool raycast_CollisionLatticeUnit_sphere( const vec4f& point, const vec4f& ray, float& distance, vec4f& normal, const CollisionFace* ptr, const CollisionFace* end )
{
	bool hit = false;

	while( ptr < end )
	{
		const CollisionFace& face = *ptr;
		++ptr;

		// Rays only intersect with front-facing faces
		if( Math::dot_3D( face.transform.z_axis, ray ) > 0.0f ) continue;

		// Check that the distance is postive and less than the current distance
		float rayDistance = Math::Triangle::rayDistance( face.transform, point, ray );
		if( rayDistance >= 0.0f && rayDistance <= distance )
		{
			distance = rayDistance;
			normal = face.transform.z_axis;
			hit = true;
		}
	}

	return hit;
}

bool CollisionLattice::rayCast( RaySensor& ray ) const
{
	const TransformSpace& unitSpace = data.getUnitSpace();
	const vec4i& dimensions = data.getDimensions();
	const CollisionLatticeData::Unit* units = data.getUnits();
	const CollisionFace* faces = data.getFaces();

	// Convert finite ray into unit space
	vec4f origin = unitSpace.inverse * ( current.inverse * ray.origin );
	vec4f line = unitSpace.inverse * ( current.inverse * ( ray.direction * ray.distance ) );

	// Clip the ray within along the boundaries of the lattice, exit if the ray is outside the bounds
	Bounds<float> rayClip = Math::AABB::rayBounds( Math::ZERO<vec4f>, static_cast<vec4f>( dimensions ), origin, line );
	if( rayClip.min > rayClip.max ) return false;

	// Epsilon fixes a rounding issue when retrieving the unit index
	float traveled = rayClip.min + Math::EPSILON<float>;
	float maxDistance = rayClip.max - Math::EPSILON<float>;

	// Define relative parameters for use when casting rays within the units
	vec4f relNormal;
	float relDistance = rayClip.max;

	// This stores whether the line is moving in the direction of -1 or 1 across each axis
	vec4i idxSign = static_cast<vec4i>( Math::sign( line ) );
	vec4i idxUnit = ( idxSign + Math::ONES<vec4i> ) / 2;
	vec4i idx = static_cast<vec4i>( Math::floor( origin + line * traveled ) );

	// Check that the starting idx is within bounds (Fail-safe)
	bool xInBounds = idx.x >= 0 && idx.x < dimensions.x;
	bool yInBounds = idx.y >= 0 && idx.y < dimensions.y;
	bool zInBounds = idx.z >= 0 && idx.z < dimensions.z;
	if( !( xInBounds && yInBounds && zInBounds ) )
	{
		return false;
	}

#ifdef VF_RAYCAST_DEBUG
	vec4i end = vec4i( Math::floor( origin + line * maxDistance ) );
	std::cout << "Bounds:\n";
	std::cout << idx.x << ", " << idx.y << ", " << idx.z << "\n";
	std::cout << end.x << ", " << end.y << ", " << end.z << "\n";
	std::cout << line.x << ", " << line.y << ", " << line.z << "\n";
	std::cout << "Path:\n";
#endif

	// Keep going until it reaches the end of the line
	while( traveled <= maxDistance )
	{
#ifdef VF_RAYCAST_SAFE
		// Check that the current idx is within bounds
		vec4i inBounds = ( idx >= Math::ZERO<vec4i> ) & ( idx < dimensions );

		if( inBounds.x && inBounds.y && inBounds.z )
		{
			// Execute collision test
			size_t i = idx.x + ( ( idx.y + ( idx.z * dimensions.y ) ) * dimensions.x );
			if( units[i].rayCast( origin, line, relDistance, relNormal ) )
			{
				// Convert the distance/normal back into world space
				ray.normal = Math::normalize( current.transform * ( unitSpace.transform * relNormal ) );
				ray.distance = Math::length( current.transform * ( unitSpace.transform * ( line * relDistance ) ) );
				ray.space = &current;
				ray.hit = true;
				return true;
			}
		}
#else
		// Assume that the index is in bounds after clipping the ray
		size_t i = idx.x + ( ( idx.y + ( idx.z * dimensions.y ) ) * dimensions.x );

		const CollisionLatticeData::Unit& unit = units[i];
		const CollisionFace* ptr = faces + unit.idx;
		if( raycast_CollisionLatticeUnit_sphere( origin, line, relDistance, relNormal, ptr, ptr + unit.size ) )
		{
			// Convert the distance/normal back into world space
			ray.normal = Math::normalize( current.transform * ( unitSpace.transform * relNormal ) );
			ray.distance = Math::length( current.transform * ( unitSpace.transform * ( line * relDistance ) ) );
			ray.space = &current;
			ray.hit = true;
			return true;
		}
#endif

		// Find the closest unit to increment to
		vec4f point = origin + line * traveled;
		vec4f difference = ( static_cast<vec4f>( idx + idxUnit ) - point ) / line;

#ifdef VF_RAYCAST_DEBUG
		std::cout << "distance = " << traveled << "\n";
		std::cout << idx.x << ", " << idx.y << ", " << idx.z << "\n";
		std::cout << point.x << ", " << point.y << ", " << point.z << "\n";
		std::cout << difference.x << ", " << difference.y << ", " << difference.z << "\n";
#endif

		if( difference.x < difference.y && difference.x < difference.z )
		{
			traveled += difference.x;
			idx.x += idxSign.x;
		}
		else if( difference.y < difference.x && difference.y < difference.z )
		{
			traveled += difference.y;
			idx.y += idxSign.y;
		}
		else
		{
			traveled += difference.z;
			idx.z += idxSign.z;
		}
	}

	return false;
}

bool CollisionLayer::testCollision( const Collider& a, const Collider& b )
{
	return intersectionMatrix[(uint8_t)a.type][(uint8_t)b.type]( a, b );
}

void CollisionLayer::testCollision()
{
	collisionLock.lock();

	size_t l1 = objects.size();
	if( l1 >= 2LLU )
	{
		size_t l0 = l1 - 1LLU;

		// Process the collisions
		for( size_t i = 0; i < l0; ++i )
		{
			CollisionBinding& a = objects[i];
			Collider& colliderA = *a.object.collider;
			const IntersectTest* colTest = intersectionMatrix[(uint8_t)colliderA.type];

			// Skip collision if collider A is disabled
			if( colliderA.flags & Collider::DISABLED ) continue;

			for( size_t j = i + 1LLU; j < l1; ++j )
			{
				CollisionBinding& b = objects[j];
				Collider& colliderB = *b.object.collider;
				const uint8_t typeB = (uint8_t)colliderB.type;
				
				// Skip collision if collider B is disabled, or if colliders A and B are both passive
				if( ( colliderB.flags & Collider::DISABLED ) ||
					( colliderA.flags & colliderB.flags & Collider::PASSIVE ) ) continue;

				if( colTest[typeB]( colliderA, colliderB ) )
				{
					colliderA.flags |= Collider::COLLIDED;
					colliderB.flags |= Collider::COLLIDED;
					a.handler( a.object, b.object );
					b.handler( b.object, a.object );
				}
			}
		}
	}

	// Process the sensors after the object may have moved
	for( RaySensor* sensor : sensors )
	{
		for( const CollisionBinding& pair : objects )
		{
			pair.object.collider->rayCast( *sensor );
		}
	}

	collisionLock.unlock();
}


void CollisionLayer::bind( Collider& collider, CollisionCallback handler, void* source, uint64_t type )
{
	collisionLock.lock();
	objects.emplace_back( handler, CollisionObject( &collider, type, source ) );
	collisionLock.unlock();
}

void CollisionLayer::unbind( const Collider& collider, CollisionCallback handler )
{
	collisionLock.lock();
	std::erase_if( objects, [ptr = &collider, handler = handler]( CollisionBinding& value ) 
	{ 
		return value.object.collider == ptr && value.handler == handler; 
	} );
	collisionLock.unlock();
}

void CollisionLayer::unbind( const Collider& collider )
{
	collisionLock.lock();
	std::erase_if( objects, [ptr = &collider]( CollisionBinding& value ) { return value.object.collider == ptr; } );
	collisionLock.unlock();
}

void CollisionLayer::addSensor( RaySensor& sensor )
{
	collisionLock.lock();
	sensors.emplace_back( &sensor );
	collisionLock.unlock();
}

void CollisionLayer::removeSensor( RaySensor& sensor )
{
	collisionLock.lock();
	std::erase_if( sensors, [ptr = &sensor]( RaySensor* value ) { return value == ptr; } );
	collisionLock.unlock();
}