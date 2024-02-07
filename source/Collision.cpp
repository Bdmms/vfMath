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
constexpr uint8_t maxIndex( const vec3f& vector )
{
	if( vector.x >= vector.y )	return vector.z >= vector.x ? 2u : 0u;
	else						return vector.z >= vector.y ? 2u : 1u;
}

/**
 * @brief Returns the index of the vector's component with the smallest value.
 * @param vector - 3D vector
 * @return component index
*/
constexpr uint8_t minIndex( const vec3f& vector )
{
	if( vector.x <= vector.y )	return vector.z <= vector.x ? 2u : 0u;
	else						return vector.z <= vector.y ? 2u : 1u;
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

#define _mm_not_ps( v ) _mm_xor_ps( v, reinterpret_cast<const __m128&>( SIMD_4i_NEG ) )
#define _mm_transform_ps( point, r0, r1, r2, r3 ) _mm_dot_ps( point, r0, point, r1, point, r2, point, r3 )
#define _mm_condition_ps( cmp, a, b ) _mm_or_ps( _mm_and_ps( cmp, a ), _mm_and_ps( _mm_not_ps( cmp ), b ) )
#define _mm_overlaps_ps( a, b, x, y ) _mm_or_ps( _mm_and_ps( _mm_cmpge_ps( a, x ), _mm_cmple_ps( a, y ) ), _mm_and_ps( _mm_cmpge_ps( x, a ), _mm_cmple_ps( x, b ) ) );

#define AABB_TRIANGLE( t, min, max )\
			min = _mm_min_ps( t.origin.simd, _mm_min_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), _mm_add_ps( t.origin.simd, t.y_axis.simd ) ) );\
			max = _mm_max_ps( t.origin.simd, _mm_max_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), _mm_add_ps( t.origin.simd, t.y_axis.simd ) ) )

#define AABB_BOX_FULL( t, min, max ) \
			__m128 point__ = _mm_add_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = point__;\
			max = point__;\
			\
			point__ = _mm_add_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = _mm_min_ps( min, point__ );\
			max = _mm_max_ps( max, point__ );\
			\
			point__ = _mm_add_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = _mm_min_ps( min, point__ );\
			max = _mm_max_ps( max, point__ );\
			\
			point__ = _mm_add_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = _mm_min_ps( min, point__ );\
			max = _mm_max_ps( max, point__ );\
			\
			point__ = _mm_sub_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = _mm_min_ps( min, point__ );\
			max = _mm_max_ps( max, point__ );\
			\
			point__ = _mm_sub_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = _mm_min_ps( min, point__ );\
			max = _mm_max_ps( max, point__ );\
			\
			point__ = _mm_sub_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = _mm_min_ps( min, point__ );\
			max = _mm_max_ps( max, point__ );\
			\
			point__ = _mm_sub_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = _mm_min_ps( min, point__ );\
			max = _mm_max_ps( max, point__ )

#define AABB_BOX_HALF( t, min ) \
			__m128 point__ = _mm_add_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = point__;\
			\
			point__ = _mm_add_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = _mm_min_ps( min, point__ );\
			\
			point__ = _mm_add_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = _mm_min_ps( min, point__ );\
			\
			point__ = _mm_sub_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = _mm_min_ps( min, point__ );\
			\
			point__ = _mm_sub_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = _mm_min_ps( min, point__ );\
			\
			point__ = _mm_sub_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = _mm_min_ps( min, point__ );\
			\
			point__ = _mm_sub_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );\
			min = _mm_min_ps( min, point__ );\



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

__m128 PROJECTED_AABB_SEPARATION_TEST( const mat4f& t, const __m128& min, const __m128& max )
{
	__m128 point = _mm_add_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	point = _mm_div_ps( point, _mm_set1_ps( point.m128_f32[3] ) );
	__m128 minBounds = point;
	__m128 maxBounds = point;

	point = _mm_add_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	point = _mm_div_ps( point, _mm_set1_ps( point.m128_f32[3] ) );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_add_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	point = _mm_div_ps( point, _mm_set1_ps( point.m128_f32[3] ) );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_add_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	point = _mm_div_ps( point, _mm_set1_ps( point.m128_f32[3] ) );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_sub_ps( _mm_add_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	point = _mm_div_ps( point, _mm_set1_ps( point.m128_f32[3] ) );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_sub_ps( _mm_add_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	point = _mm_div_ps( point, _mm_set1_ps( point.m128_f32[3] ) );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_sub_ps( _mm_sub_ps( _mm_add_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	point = _mm_div_ps( point, _mm_set1_ps( point.m128_f32[3] ) );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	point = _mm_sub_ps( _mm_sub_ps( _mm_sub_ps( t.origin.simd, t.x_axis.simd ), t.y_axis.simd ), t.z_axis.simd );
	point = _mm_div_ps( point, _mm_set1_ps( point.m128_f32[3] ) );
	minBounds = _mm_min_ps( minBounds, point );
	maxBounds = _mm_max_ps( maxBounds, point );

	return _mm_or_ps( _mm_and_ps( _mm_cmpge_ps( minBounds, min ), _mm_cmple_ps( minBounds, max ) ), _mm_and_ps( _mm_cmpge_ps( min, minBounds ), _mm_cmple_ps( min, maxBounds ) ) );
}

bool Math::AABB::rayTest( const vec3f& min, const vec3f& max, const vec4f& point, const vec3f& ray )
{
	__m128 intersect0 = _mm_div_ps( _mm_sub_ps( min.simd, point.simd ), ray.simd );
	__m128 intersect1 = _mm_div_ps( _mm_sub_ps( max.simd, point.simd ), ray.simd );

	__m128 tmin = _mm_condition_ps( _mm_cmpeq_ps( ray.simd, SIMD_4f_ZERO ), SIMD_4f_ZERO, _mm_min_ps( intersect0, intersect1 ) );
	__m128 tmax = _mm_condition_ps( _mm_cmpeq_ps( ray.simd, SIMD_4f_ZERO ), SIMD_4f_ONES, _mm_max_ps( intersect0, intersect1 ) );
	
	float minBounds = std::max( std::max( tmin.m128_f32[0], tmin.m128_f32[1] ), std::max( tmin.m128_f32[2], 0.0f ) );
	float maxBounds = std::min( std::min( tmax.m128_f32[0], tmax.m128_f32[1] ), std::min( tmax.m128_f32[2], 1.0f ) );

	return minBounds <= maxBounds;
}

Bounds<float> Math::AABB::rayBounds( const vec3f& min, const vec3f& max, const vec4f& point, const vec3f& ray )
{
	__m128 intersect0 = _mm_div_ps( _mm_sub_ps( min.simd, point.simd ), ray.simd );
	__m128 intersect1 = _mm_div_ps( _mm_sub_ps( max.simd, point.simd ), ray.simd );

	__m128 tmin = _mm_condition_ps( _mm_cmpeq_ps( ray.simd, SIMD_4f_ZERO ), SIMD_4f_ZERO, _mm_min_ps( intersect0, intersect1 ) );
	__m128 tmax = _mm_condition_ps( _mm_cmpeq_ps( ray.simd, SIMD_4f_ZERO ), SIMD_4f_ONES, _mm_max_ps( intersect0, intersect1 ) );

	return
	{
		std::clamp( std::max( std::max( tmin.m128_f32[0], tmin.m128_f32[1] ), tmin.m128_f32[2] ), 0.0f, 1.0f ),
		std::clamp( std::min( std::min( tmax.m128_f32[0], tmax.m128_f32[1] ), tmax.m128_f32[2] ), 0.0f, 1.0f )
	};
}

float Math::Triangle::rayDistance( const mat4f& triangle, const vec4f& rayOrigin, const vec3f& rayVector )
{
	// Calculate the line-plane intersection
	__m128 displaced = _mm_sub_ps( triangle.origin.simd, rayOrigin.simd );
	__m128 normal = _mm_cross_ps( triangle.x_axis.simd, triangle.y_axis.simd );

	__m128 product = _mm_dot_ps( normal, displaced,
		_mm_cross_ps( rayVector.simd, triangle.y_axis.simd ), displaced,
		_mm_cross_ps( triangle.x_axis.simd, rayVector.simd ), displaced,
		normal, rayVector.simd );

	// Check that the plane intersection hits the region of the triangle
	__m128 intersectData = _mm_div_ps( product, _mm_set1_ps( product.m128_f32[3] ) );
	vec3f& intersection = reinterpret_cast<vec3f&>( intersectData );
	return intersection.y >= 0.0f && intersection.z >= 0.0f && intersection.y + intersection.z <= 1.0f ? intersection.x : INFINITY;
}

bool Math::Box::pointTest( const vec4f& point )
{
	__m128 isInside = _mm_and_ps( _mm_cmpge_ps( point.simd, SIMD_4f_NEG ), _mm_cmple_ps( point.simd, SIMD_4f_ONES ) );
	return isInside.m128_u32[0] && isInside.m128_u32[1] && isInside.m128_u32[2];
}

bool Math::Box::rayTest( const vec4f& origin, const vec3f& ray )
{
	return Math::AABB::rayTest( Math::NEGATIVE<vec4f>, Math::ONES<vec4f>, origin, ray );

	// OLD Implementation (TODO: Delete after proving new implementation)
	/*if( pointTest(rayOrigin) || pointTest(rayOrigin + rayVector) ) return true;

	// Create a sign vector to select the corner of the box
	__m128 comparison = _mm_cmpge_ps( rayOrigin.simd, SIMD_4f_ZERO );
	__m128 quadrant = _mm_or_ps( _mm_and_ps( comparison, SIMD_4f_ONES ), _mm_and_ps( _mm_xor_ps( comparison, reinterpret_cast<const __m128&>( SIMD_4i_NEG ) ), SIMD_4f_NEG ) );

	// Calculate the vectors in the mirrored quadrant
	__m128 vector = _mm_mul_ps( quadrant, rayVector.simd );								// l1 - l0
	__m128 origin = _mm_sub_ps( SIMD_4f_W, _mm_mul_ps( quadrant, rayOrigin.simd ) );	// p0 - l0

	// Calculate UV vectors
	__m128 xProduct = _mm_cross_ps( vector, Math::axis::X<vec3f>.simd );
	__m128 yProduct = _mm_cross_ps( vector, Math::axis::Y<vec3f>.simd );
	__m128 zProduct = _mm_cross_ps( vector, Math::axis::Z<vec3f>.simd );

	// Calculate the TUV values for 3 line-plane intersections simultaneously
	__m128 denominator = _mm_dot3_ps( SIMD_X_AXIS, vector, SIMD_Y_AXIS, vector, SIMD_Z_AXIS, vector );
	__m128 t = _mm_div_ps( _mm_dot3_ps( SIMD_X_AXIS, origin, SIMD_Y_AXIS, origin, SIMD_Z_AXIS, origin ), denominator );
	__m128 u = _mm_div_ps( _mm_dot3_ps( yProduct, origin, xProduct, origin, xProduct, origin ), denominator );
	__m128 v = _mm_div_ps( _mm_dot3_ps( zProduct, origin, zProduct, origin, yProduct, origin ), denominator );

	// Check if the intersection is within the correct bounds
	__m128 isValid = _mm_and_ps( 
		_mm_and_ps( _mm_cmpge_ps( t, SIMD_4f_ZERO ), _mm_cmple_ps( t, SIMD_4f_ONES ) ), _mm_and_ps(
		_mm_and_ps( _mm_cmpge_ps( u, SIMD_4f_NEG ), _mm_cmple_ps( u, SIMD_4f_ONES ) ),
		_mm_and_ps( _mm_cmpge_ps( v, SIMD_4f_NEG ), _mm_cmple_ps( v, SIMD_4f_ONES ) ) ) );

	// Check if either of the 3 faces intersected the ray
	return isValid.m128_u32[0] || isValid.m128_u32[1] || isValid.m128_u32[2];*/
}

bool Math::Box::triangleTest( const TransformSpace& triangle )
{
	// Test triangle overlapping box
	const mat4x4& tri = triangle.transform;
	__m128 min, max;

	AABB_TRIANGLE( tri, min, max );

	__m128 minBounds = _mm_xyzw_ps( -ONE_EPSILON, -ONE_EPSILON, -ONE_EPSILON, 0.0f );
	__m128 maxBounds = _mm_xyzw_ps(  ONE_EPSILON,  ONE_EPSILON,  ONE_EPSILON, 0.0f );
	__m128 overlaps = _mm_overlaps_ps( min, max, minBounds, maxBounds );
	if( !( overlaps.m128_f32[0] && overlaps.m128_f32[1] && overlaps.m128_f32[2] ) ) return false;
	
	// Test box overlapping triangle
	AABB_BOX_FULL( triangle.inverse, min, max );

	return max.m128_f32[0] >= N_EPSILON && max.m128_f32[1] >= N_EPSILON
		&& min.m128_f32[0] + min.m128_f32[1] <= ONE_EPSILON
		&& min.m128_f32[2] <= P_EPSILON && max.m128_f32[2] >= N_EPSILON;
}

bool Math::Box::boxTest( const TransformSpace& box )
{
	// Test regular overlap
	__m128 overlaps = MIRRORED_AABB_SEPARATION_TEST( box.transform, SIMD_4f_ONES );
	if( !( overlaps.m128_f32[0] && overlaps.m128_f32[1] && overlaps.m128_f32[2] ) ) return false;

	overlaps = MIRRORED_AABB_SEPARATION_TEST( box.inverse, SIMD_4f_ONES );
	return overlaps.m128_u32[0] && overlaps.m128_u32[1] && overlaps.m128_u32[2];
}

bool Math::Box::projectedBoxTest( const TransformSpace& box )
{
	// Test regular overlap
	__m128 overlaps = PROJECTED_AABB_SEPARATION_TEST( box.transform, SIMD_4f_NEG, SIMD_4f_ONES );
	if( !( overlaps.m128_f32[0] && overlaps.m128_f32[1] && overlaps.m128_f32[2] ) ) return false;

	overlaps = PROJECTED_AABB_SEPARATION_TEST( box.inverse, SIMD_4f_NEG, SIMD_4f_ONES );
	return overlaps.m128_u32[0] && overlaps.m128_u32[1] && overlaps.m128_u32[2];
}

bool Math::Sphere::sphereTest( const TransformSpace& sphere )
{
	vec3f displacement = sphere.transform.origin;
	displacement.w = 0.0f;

	float distance = Math::length( displacement );
	if( distance <= Math::EPSILON<float> ) return true;

	float radius = Math::length( sphere.transform * Math::normalize( sphere.inverse * displacement ) );

	// Test if the distance is less than the radius of both spheres
	return distance <= radius + 1.0f;
}

bool Math::Sphere::boxTest( const TransformSpace& box )
{
	vec3f closestPoint = Math::clamp( box.inverse.origin, Math::NEGATIVE<vec3f>, Math::ONES<vec3f> );
	vec4f displacement = box.transform * closestPoint;
	float distance2 = displacement.x * displacement.x + displacement.y * displacement.y + displacement.z * displacement.z;
	return distance2 <= 1.0f;
}

bool sphereFaceCollision( vec3f& displacement, const TransformSpace& sphere, const TransformSpace& face )
{
	vec3f uvs = face.inverse * sphere.transform.origin;
	if( uvs.x + uvs.y > 1.0f || uvs.x < 0.0f || uvs.y < 0.0f ) return false;

	vec4f bounded = Math::max( uvs, Math::ZERO<vec4f> );
	vec4f clamped = bounded.x + bounded.y > 1.0f ? bounded / ( bounded.x + bounded.y ) : bounded;
	mat4x4 relFace = sphere.inverse * face.transform;

	vec3f compNormal = relFace.origin + relFace.x_axis * uvs.x + relFace.y_axis * uvs.y;
	vec3f compCombined = relFace.origin + relFace.x_axis * clamped.x + relFace.y_axis * clamped.y;
	vec3f compSurface = compCombined - compNormal;
	float surfaceDistance2 = Math::dot_3D( compSurface, compSurface );
	float normalDistance2 = Math::dot_3D( compNormal, compNormal );
	
	if( surfaceDistance2 > 1.0f ) return false;

	float x = sqrtf( 1.0f - surfaceDistance2 ) - sqrtf( normalDistance2 );
	displacement = sphere.transform * ( Math::normalize( relFace.z_axis ) * x );

	return Math::dot_3D( compCombined, compCombined ) <= 1.0f;
}

bool Math::Box::sphereCollision( vec3f& displacement, const TransformSpace& sphere )
{
	const mat4x4& boxSpace = sphere.inverse;
	const vec4f& origin = sphere.transform.origin;

	vec3f clamped = Math::clamp( origin, Math::NEGATIVE<vec3f>, Math::ONES<vec3f> );
	vec3f quadrant = Math::sign( origin );

	vec3f rx = boxSpace.x_axis * clamped.x;
	vec3f ry = boxSpace.y_axis * clamped.y;
	vec3f rz = boxSpace.z_axis * clamped.z;

	vec3f vectorBoxSpace[3] = {
		( boxSpace.origin - Math::axis::W<vec4f> ) + ( boxSpace.x_axis * quadrant.x ) + ry + rz,
		( boxSpace.origin - Math::axis::W<vec4f> ) + ( boxSpace.y_axis * quadrant.y ) + rz + rx,
		( boxSpace.origin - Math::axis::W<vec4f> ) + ( boxSpace.z_axis * quadrant.z ) + rx + ry
	};
	vec3f lengthBoxSpace = Math::parallel::length( vectorBoxSpace[0], vectorBoxSpace[1], vectorBoxSpace[2] );

	uint8_t idx = minIndex( lengthBoxSpace );
	displacement = sphere.transform * vectorBoxSpace[idx];
	float distance2 = Math::length2( displacement );
	float radius2 = Math::length2( sphere.transform * ( vectorBoxSpace[idx] / lengthBoxSpace[idx] ) );

	return distance2 <= radius2;
}

bool Math::Sphere::boxCollision( vec3f& displacement, const TransformSpace& box )
{
	const mat4x4& sphereSpace = box.inverse;

	vec3f boxPoint = Math::clamp( sphereSpace.origin, { -1.0f, -1.0f, -1.0f, 0.0f }, { 1.0f, 1.0f, 1.0f, 0.0f } );
	vec3f spherePoint = box.transform * boxPoint;
	float distance2 = Math::dot_3D( spherePoint, spherePoint );
	
	if( distance2 > Math::EPSILON<float> )
	{
		// Calculate displacement from "revised" point
		displacement = spherePoint - ( spherePoint / sqrtf( distance2 ) );
	}
	else
	{
		// Arbitrarily displace along the x-axis
		displacement = Math::axis::X<vec3f>;
	}

	return distance2 <= 1.0f;
}

bool Math::Sphere::sphereCollision( vec3f& displacement, const TransformSpace& sphere )
{
	vec3f vector = sphere.transform.origin;
	vector.w = 0.0f;

	float distance = Math::length( vector );
	if( distance <= Math::EPSILON<float> )
	{
		displacement = Math::axis::X<vec3f>;
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

const IntersectTest intersectionMatrix[3][3]
{
	{ intersect_AABB, intersect_AABB,		intersect_AABB },
	{ intersect_AABB, intersect_Box_Box,	intersect_Box_Sphere },
	{ intersect_AABB, intersect_Sphere_Box, intersect_Sphere_Sphere }
};

// -------------------------
// --- General Collision ---
// -------------------------

Bounds<vec3f> Math::Box::calculateAABB( const mat4x4& t )
{
	Bounds<vec3f> bounds;
	AABB_BOX_FULL( t, bounds.min.simd, bounds.max.simd );
	return bounds;
}

Bounds<vec3f> Math::Triangle::calculateAABB( const mat4x4& t )
{
	Bounds<vec3f> bounds;
	AABB_TRIANGLE( t, bounds.min.simd, bounds.max.simd );
	return bounds;
}

void CollisionLattice::addTriangle( const TransformSpace& triangle )
{
	const TransformSpace relative = { unitSpace.inverse * triangle.transform, triangle.inverse * unitSpace.transform };
	TransformSpace unit = { Math::create::scale( { 0.5f, 0.5f, 0.5f } ), Math::create::scale( { 2.0f, 2.0f, 2.0f } ) };
	TransformSpace unitTriangle;
	
	Bounds<vec3f> triangleBounds;
	AABB_TRIANGLE( relative.transform, triangleBounds.min.simd, triangleBounds.max.simd );

	vec3i minIdx = Math::clamp( vec3i( Math::floor( triangleBounds.min - P_EPSILON ) ), Math::ZERO<vec3i>, dimensions - Math::ONES<vec3i> );
	vec3i maxIdx = Math::clamp( vec3i( Math::ceil(  triangleBounds.max + P_EPSILON ) ), Math::ONES<vec3i>, dimensions );
	minIdx.w = 0;
	maxIdx.w = 0;

	// Populate the units with any intersecting face 
	for( vec3i idx = minIdx; idx.z < maxIdx.z; ++idx.z )
	{
		size_t iz = idx.u_z * dimensions.u_y;
		for( idx.y = minIdx.y; idx.y < maxIdx.y; ++idx.y )
		{
			size_t iy = ( iz + idx.u_y ) * dimensions.u_x;
			for( idx.x = minIdx.x; idx.x < maxIdx.x; ++idx.x )
			{
				unit.transform.origin = vec4f( idx ) + vec4f{ 0.5f, 0.5f, 0.5f, 1.0f };
				unit.inverse.origin = unit.transform.origin * vec4f{ -2.0f, -2.0f, -2.0f, 1.0f };
				unit.inverse = unit.transform.inverse();

				unitTriangle.transform = unit.inverse * relative.transform;
				unitTriangle.inverse = relative.inverse * unit.transform;

				if( Math::Box::triangleTest( unitTriangle ) )
				{
					units[iy + idx.u_x].faces.emplace_back( relative );
				}

#ifdef VF_DEBUG_TRIANGLE_LATTICE
				else
				{
					vec3f v0 = relative.transform.origin;
					vec3f v1 = relative.transform.origin + relative.transform.x_axis;
					vec3f v2 = relative.transform.origin + relative.transform.y_axis;

					vec3i p0 = vec3i( Math::floor( v0 ) );
					vec3i p1 = vec3i( Math::floor( v1 ) );
					vec3i p2 = vec3i( Math::floor( v2 ) );

					vec3i a = idx ^ p0;
					vec3i b = idx ^ p1;
					vec3i c = idx ^ p2;
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

CollisionMesh::CollisionMesh( const std::vector<CollisionFace>& source ) : 
	ColliderSpace(),
	faces()
{
	setFaces( source );
}

void CollisionMesh::setFaces( const std::vector<CollisionFace>& source )
{
	aabb = { Math::MAX<vec3f>, Math::MIN<vec3f> };
	Math::extend( aabb, source );

	current.transform = Math::createBoundingTransform( aabb );
	current.inverse = Math::createBoundingInverseTransform( aabb );
	previous = current;

	faces.resize( source.size() );
	for( size_t i = 0; i < source.size(); ++i )
	{
		faces[i].transform = current.inverse * source[i].transform;
		faces[i].inverse = faces[i].transform.inverse();
	}
}

CollisionLattice::CollisionLattice( const Bounds<vec3f>& aabb, const vec3i& dimensions ) :
	ColliderSpace( aabb ),
	dimensions( dimensions ),
	length( size_t( dimensions.x ) * size_t( dimensions.y ) * size_t( dimensions.z ) ),
	units( new CollisionUnit[length] )
{
	vec3f unitScale = 2.0f / vec3f( dimensions );
	mat4x4 unitConvert = { { unitScale.x, 0.0f, 0.0f }, { 0.0f, unitScale.y, 0.0f }, { 0.0f, 0.0f, unitScale.z }, vec4f{ -1.0f, -1.0f, -1.0f, 1.0f } };
	
	unitSpace.transform = current.transform * unitConvert;
	unitSpace.inverse = unitSpace.transform.inverse();
}

CollisionLattice::CollisionLattice( const std::vector<CollisionFace>& meshCollision, const vec3i& dimensions ) :
	ColliderSpace(),
	dimensions( dimensions ),
	length( size_t( dimensions.x ) * size_t( dimensions.y ) * size_t( dimensions.z ) ),
	units( new CollisionUnit[length] )
{
	vec3f unitScale = 2.0f / vec3f( dimensions );
	mat4x4 unitConvert = { { unitScale.x, 0.0f, 0.0f }, { 0.0f, unitScale.y, 0.0f }, { 0.0f, 0.0f, unitScale.z }, vec4f{ -1.0f, -1.0f, -1.0f, 1.0f } };
	
	Math::extend( aabb, meshCollision );

	current.transform = Math::createBoundingTransform( aabb );
	current.inverse = Math::createBoundingInverseTransform( aabb );
	unitSpace.transform = current.transform * unitConvert;
	unitSpace.inverse = unitSpace.transform.inverse();

	for( const CollisionFace& face : meshCollision )
	{
		addTriangle( face );
	}
}

CollisionLattice::CollisionLattice( const std::vector<std::vector<CollisionFace>>& meshCollision, const vec3i& dimensions ) :
	ColliderSpace(),
	dimensions( dimensions ),
	length( size_t( dimensions.x )* size_t( dimensions.y )* size_t( dimensions.z ) ),
	units( new CollisionUnit[length] )
{
	vec3f unitScale = 2.0f / vec3f( dimensions );
	mat4x4 unitConvert = { { unitScale.x, 0.0f, 0.0f }, { 0.0f, unitScale.y, 0.0f }, { 0.0f, 0.0f, unitScale.z }, vec4f{ -1.0f, -1.0f, -1.0f, 1.0f } };
	
	for( const std::vector<CollisionFace>& collider : meshCollision )
	{
		Math::extend( aabb, collider );
	}

	current.transform = Math::createBoundingTransform( aabb );
	current.inverse = Math::createBoundingInverseTransform( aabb );
	unitSpace.transform = current.transform * unitConvert;
	unitSpace.inverse = unitSpace.transform.inverse();

	for( const std::vector<CollisionFace>& collider : meshCollision )
	{
		for( const CollisionFace& face : collider )
		{
			addTriangle( face );
		}
	}
}

void CollisionUnit::collision( TransformSpace& relativeSphere ) const
{
	vec3f displacement;
	for( const CollisionFace& face : faces )
	{
		// If sphere intersects face, translate sphere by face displacement
		if( sphereFaceCollision( displacement, relativeSphere, face ) )
		{
			Math::translate( relativeSphere, displacement );
		}
	}
}

void CollisionMesh::collision( TransformSpace& sphere ) const
{
	TransformSpace relative = { current.inverse * sphere.transform, sphere.inverse * current.transform };
	vec4f startOrigin = relative.transform.origin;
	vec3f displacement;

	for( const CollisionFace& face : faces )
	{
		if( sphereFaceCollision( displacement, relative, face ) )
		{
			Math::translate( relative, displacement );
		}
	}

	Math::translate( sphere, current.transform * ( relative.transform.origin - startOrigin ) );
}

void CollisionLattice::collision( TransformSpace& sphere ) const
{
	// Convert the sphere into unit space
	TransformSpace relative = { unitSpace.inverse * sphere.transform, sphere.inverse * unitSpace.transform };
	vec4f startOrigin = relative.transform.origin;

	// Compute the bounds of the sphere in unit space
	Bounds<vec3f> aabb = Math::Box::calculateAABB( relative.transform );
	vec3i minIdx = Math::clamp( vec3i( Math::floor( aabb.min ) ), Math::ZERO<vec3i>, dimensions );
	vec3i maxIdx = Math::clamp( vec3i( Math::ceil(  aabb.max ) ), Math::ZERO<vec3i>, dimensions );

	// Iterate over the units the sphere overlaps with
	for( vec3i idx = minIdx; idx.z < maxIdx.z; ++idx.z )
	{
		size_t iz = idx.u_z * dimensions.u_y;
		for( idx.y = minIdx.y; idx.y < maxIdx.y; ++idx.y )
		{
			size_t iy = ( iz + idx.u_y ) * dimensions.u_x;
			for( idx.x = minIdx.x; idx.x < maxIdx.x; ++idx.x )
			{
				// Process the collision within the unit
				units[iy + idx.u_x].collision( relative );
			}
		}
	}

	// Propagate the translation of the collision back into world space
	Math::translate( sphere, unitSpace.transform * ( relative.transform.origin - startOrigin ) );
}

bool CollisionMesh::rayCast( RaySensor& ray ) const
{
	// Convert finite ray into mesh space
	vec4f origin = current.inverse * ray.origin;
	vec3f line = current.inverse * ( ray.direction * ray.distance );

	if( !Math::Box::rayTest( origin, line ) ) return false;

	// We need to make sure the distance/normal is calculated relative to the mesh space
	float relDistance = 1.0f;
	vec3f relNormal;
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

bool CollisionUnit::rayCast( const vec4f& point, const vec3f& ray, float& distance, vec3f& normal ) const
{
	bool hit = false;

	for( const CollisionFace& face : faces )
	{
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
	// Convert finite ray into unit space
	vec4f origin = unitSpace.inverse * ray.origin;
	vec3f line = unitSpace.inverse * ( ray.direction * ray.distance );

	// Clip the ray within along the boundaries of the lattice, exit if the ray is outside the bounds
	Bounds<float> rayClip = Math::AABB::rayBounds( Math::ZERO<vec3f>, vec3f( dimensions ), origin, line );
	if( rayClip.min > rayClip.max ) return false;

	// Epsilon fixes a rounding issue when retrieving the unit index
	float traveled = rayClip.min + Math::EPSILON<float>;
	float maxDistance = rayClip.max - Math::EPSILON<float>;

	// Define relative parameters for use when casting rays within the units
	vec3f relNormal;
	float relDistance = rayClip.max;

	// This stores whether the line is moving in the direction of -1 or 1 across each axis
	vec3i idxSign = vec3i( Math::sign( line ) );
	vec3i idxUnit = ( idxSign + Math::ONES<vec3i> ) / 2;
	vec3i idx = vec3i( Math::floor( origin + line * traveled ) );

	// Check that the starting idx is within bounds (Fail-safe)
	bool xInBounds = idx.x >= 0 && idx.x < dimensions.x;
	bool yInBounds = idx.y >= 0 && idx.y < dimensions.y;
	bool zInBounds = idx.z >= 0 && idx.z < dimensions.z;
	if( !( xInBounds && yInBounds && zInBounds ) )
	{
		return false;
	}

#ifdef VF_RAYCAST_DEBUG
	vec3i end = vec3i( Math::floor( origin + line * maxDistance ) );
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
		vec3i inBounds = ( idx >= Math::ZERO<vec3i> ) & ( idx < dimensions );

		if( inBounds.x && inBounds.y && inBounds.z )
		{
			// Execute collision test
			size_t i = idx.x + ( ( idx.y + ( idx.z * dimensions.y ) ) * dimensions.x );
			if( units[i].rayCast( origin, line, relDistance, relNormal ) )
			{
				// Convert the distance/normal back into world space
				ray.normal = Math::normalize( unitSpace.transform * relNormal );
				ray.distance = Math::length( unitSpace.transform * ( line * relDistance ) );
				ray.space = &current;
				ray.hit = true;
				return true;
			}
		}
#else
		// Assume that the index is in bounds after clipping the ray
		size_t i = idx.x + ( ( idx.y + ( idx.z * dimensions.y ) ) * dimensions.x );
		if( units[i].rayCast( origin, line, relDistance, relNormal ) )
		{
			// Convert the distance/normal back into world space
			ray.normal = Math::normalize( unitSpace.transform * relNormal );
			ray.distance = Math::length( unitSpace.transform * ( line * relDistance ) );
			ray.space = &current;
			ray.hit = true;
			return true;
		}
#endif

		// Find the closest unit to increment to
		vec3f point = origin + line * traveled;
		vec3f difference = ( vec3f( idx + idxUnit ) - point ) / line;

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
			if( colliderA.flags & Collider::DISABLED ) continue;

			for( size_t j = i + 1LLU; j < l1; ++j )
			{
				CollisionBinding& b = objects[j];
				Collider& colliderB = *b.object.collider;
				const uint8_t typeB = (uint8_t)colliderB.type;
				if( colliderB.flags & Collider::DISABLED ) continue;

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