#pragma once
#ifndef VF_CURVES_HPP
#define VF_CURVES_HPP

#include "VectorMath.hpp"
#include "mat4x4.hpp"
#include <vector>

/**
 * @brief Parameters that specify the type of curve implemented.
*/
struct CurveType
{
	mat4f basis;
	float factor;

	/**
	 * @brief Interpolates the result of the curve between all of the knot controls based on the sample weight vector.
	 * @param weight - sample weight vector
	 * @param r0 - 1st knot point sample
	 * @param r1 - 2nd knot point sample
	 * @param r2 - 3rd knot point sample
	 * @param r3 - 4th knot point sample
	*/
	[[nodiscard]] vec4f weightSample( const vec4f& weight, const vec4f& r0, const vec4f& r1, const vec4f& r2, const vec4f& r3 ) const
	{
		// Transpose knot matrix
		__m128 c0 = { r0.x, r1.x, r2.x, r3.x };
		__m128 c1 = { r0.y, r1.y, r2.y, r3.y };
		__m128 c2 = { r0.z, r1.z, r2.z, r3.z };
		__m128 c3 = { r0.w, r1.w, r2.w, r3.w };

		// Multiply knot matrix with basis matrix
		c0 = _mm_dot_ps( basis.simd[0], c0, basis.simd[1], c0, basis.simd[2], c0, basis.simd[3], c0 );
		c1 = _mm_dot_ps( basis.simd[0], c1, basis.simd[1], c1, basis.simd[2], c1, basis.simd[3], c1 );
		c2 = _mm_dot_ps( basis.simd[0], c2, basis.simd[1], c2, basis.simd[2], c2, basis.simd[3], c2 );
		c3 = _mm_dot_ps( basis.simd[0], c3, basis.simd[1], c3, basis.simd[2], c3, basis.simd[3], c3 );

		// Multiply weight vector with matrix
		return { _mm_mul_ps( _mm_dot_ps( weight.simd, c0, weight.simd, c1, weight.simd, c2, weight.simd, c3 ), _mm_set1_ps( factor ) ) };
	}

	/**
	 * @brief Creates a curve with a specified level of "smoothness".
	 * @param s - scale of the cardinal curve
	*/
	constexpr static CurveType createCardinalSplineType( float s )
	{
		return { { 0.0f, 1.0f, 0.0f, 0.0f, -s, 0.0f, s, 0.0f, 2.0f * s, s - 3.0f, 3.0f - 2.0f * s, -s, -s, 2.0f - s, s - 2.0f, s }, 1.0f };
	}
};

/**
 * @brief Container of discrete points used to form a continuous curve. 
*/
struct KnotCurve : public std::vector<vec4f>
{
	constexpr static float CURVE_SAMPLE_MIN = 0.0f;
	constexpr static float CURVE_SAMPLE_MAX = 1.0f - Math::EPSILON<float>;

	constexpr static CurveType POLYLINE    = CurveType::createCardinalSplineType( 0.0f );
	constexpr static CurveType HERMITE     = { { 1.0f, 0.0f, 0.0f, 0.0f,    0.0f, 1.0f, 0.0f, 0.0f,  -3.0f, -2.0f, 3.0f,  0.0f,    2.0f, 1.0f, -2.0f, 1.0f }, 1.0f };
	constexpr static CurveType BEZIER      = { { 1.0f, 0.0f, 0.0f, 0.0f,   -3.0f, 3.0f, 0.0f, 0.0f,   3.0f, -6.0f, 3.0f,  0.0f,   -1.0f, 3.0f, -3.0f, 1.0f }, 1.0f };
	constexpr static CurveType CATMULL_ROM = { { 0.0f, 2.0f, 0.0f, 0.0f,   -1.0f, 0.0f, 1.0f, 0.0f,   2.0f, -5.0f, 4.0f, -1.0f,   -1.0f, 3.0f, -3.0f, 1.0f }, 0.5f };
	constexpr static CurveType BSPLINE     = { { 1.0f, 4.0f, 1.0f, 0.0f,   -3.0f, 0.0f, 3.0f, 0.0f,   3.0f, -6.0f, 3.0f,  0.0f,   -1.0f, 3.0f, -3.0f, 1.0f }, 1.0f / 6.0f };

	CurveType type;

	/**
	 * @brief Creates a curve of the specified type.
	 * @param type - curve type
	*/
	constexpr KnotCurve( const CurveType& type ) :
		std::vector<vec4f>(),
		type( type )
	{

	}

	/**
	 * @brief Samples the value of the curve at a scalar point.
	 * @param t - sample scalar between 0.0 and 1.0
	 * @return sampled vector at scalar point
	*/
	vec4f sampleAt( float t ) const
	{
		if( size() < 4 ) return Math::axis::W<vec4f>;

		// Convert absolute t weight into relative t weight
		float absolute = std::clamp( t, CURVE_SAMPLE_MIN, CURVE_SAMPLE_MAX ) * ( size() - 3LLU );
		float weight = absolute - floorf( absolute );

		const vec4f* ptr = data() + static_cast<uint32_t>( absolute );
		return type.weightSample( { 1.0f, weight, weight * weight, weight * weight * weight }, ptr[0], ptr[1], ptr[2], ptr[3] );
	}

	/**
	 * @brief Samples the derivative value of the curve at a scalar point.
	 * @param t - sample scalar between 0.0 and 1.0
	 * @return derivative vector at scalar point
	*/
	vec4f derivativeAt( float t ) const
	{
		if( size() < 4 ) return Math::axis::W<vec4f>;

		// Convert absolute t weight into relative t weight
		float absolute = std::clamp( t, CURVE_SAMPLE_MIN, CURVE_SAMPLE_MAX ) * ( size() - 3LLU );
		float weight = absolute - floorf( absolute );

		const vec4f* ptr = data() + static_cast<uint32_t>( absolute );
		return type.weightSample( { 0.0f, 1.0f, 2.0f * weight, 3.0f * weight * weight }, ptr[0], ptr[1], ptr[2], ptr[3] );
	}

	/**
	 * @brief Samples the integrated value of the curve at a scalar point.
	 * @param t - sample scalar between 0.0 and 1.0
	 * @return integrated vector at scalar point
	*/
	vec4f integrateAt( float t ) const
	{
		if( size() < 4 ) return Math::axis::W<vec4f>;

		// Convert absolute t weight into relative t weight
		float absolute = std::clamp( t, CURVE_SAMPLE_MIN, CURVE_SAMPLE_MAX ) * (size() - 3LLU );
		float weight = absolute - floorf( absolute );
		float weight2 = weight * weight;

		const vec4f* ptr = data() + static_cast<uint32_t>( absolute );
		return type.weightSample( { weight, weight2 / 2.0f, weight2 * weight / 3.0f, weight2 * weight2 / 4.0f }, ptr[0], ptr[1], ptr[2], ptr[3] );
	}

	/**
	 * @brief Calculates the length between two sample points on the line.
	 * @param t0 - first sample scalar between 0.0 and 1.0
	 * @param t1 - second sample scalar between 0.0 and 1.0
	 * @return length of curve between two points
	*/
	float lengthBetween( float t0, float t1 ) const
	{
		return Math::length( integrateAt( t1 ) - integrateAt( t0 ) );
	}
};

struct PathPoint
{
	vec4f position;
	vec3f normal;
};

/**
 * @brief Container of discrete points used to form a continuous curve.
*/
struct PathCurve : public std::vector<PathPoint>
{
	constexpr static float CURVE_SAMPLE_MIN = 0.0f;
	constexpr static float CURVE_SAMPLE_MAX = 1.0f - Math::EPSILON<float>;

	CurveType type;

	/**
	 * @brief Creates a curve of the specified type.
	 * @param type - curve type
	*/
	constexpr PathCurve( const CurveType& type ) :
		std::vector<PathPoint>(),
		type( type )
	{

	}

	/**
	 * @brief Samples the value of the curve at a scalar point.
	 * @param t - sample scalar between 0.0 and 1.0
	 * @return sampled vector at scalar point
	*/
	PathPoint sampleAt( float t ) const
	{
		if( size() < 4 ) return { Math::axis::W<vec4f>, Math::axis::Y<vec3f> };

		// Convert absolute t weight into relative t weight
		float absolute = std::clamp( t, CURVE_SAMPLE_MIN, CURVE_SAMPLE_MAX ) * ( size() - 3LLU );
		float weight = absolute - floorf( absolute );

		const PathPoint* ptr = data() + static_cast<uint32_t>( absolute );
		const vec4f weightSample = { 1.0f, weight, weight * weight, weight * weight * weight };

		return 
		{
			type.weightSample( weightSample, ptr[0].position, ptr[1].position, ptr[2].position, ptr[3].position ),
			type.weightSample( weightSample, ptr[0].normal, ptr[1].normal, ptr[2].normal, ptr[3].normal )
		};
	}

	/**
	 * @brief Samples the derivative value of the curve at a scalar point.
	 * @param t - sample scalar between 0.0 and 1.0
	 * @return derivative vector at scalar point
	*/
	vec4f derivativeAt( float t ) const
	{
		if( size() < 4 ) return Math::axis::W<vec4f>;

		// Convert absolute t weight into relative t weight
		float absolute = std::clamp( t, CURVE_SAMPLE_MIN, CURVE_SAMPLE_MAX ) * ( size() - 3LLU );
		float weight = absolute - floorf( absolute );

		const PathPoint* ptr = data() + static_cast<uint32_t>( absolute );
		return type.weightSample( { 0.0f, 1.0f, 2.0f * weight, 3.0f * weight * weight }, ptr[0].position, ptr[1].position, ptr[2].position, ptr[3].position );
	}

	/**
	 * @brief Samples the integrated value of the curve at a scalar point.
	 * @param t - sample scalar between 0.0 and 1.0
	 * @return integrated vector at scalar point
	*/
	vec4f integrateAt( float t ) const
	{
		if( size() < 4 ) return Math::axis::W<vec4f>;

		// Convert absolute t weight into relative t weight
		float absolute = std::clamp( t, CURVE_SAMPLE_MIN, CURVE_SAMPLE_MAX ) * ( size() - 3LLU );
		float weight = absolute - floorf( absolute );
		float weight2 = weight * weight;

		const PathPoint* ptr = data() + static_cast<uint32_t>( absolute );
		return type.weightSample( { weight, weight2 / 2.0f, weight2 * weight / 3.0f, weight2 * weight2 / 4.0f }, ptr[0].position, ptr[1].position, ptr[2].position, ptr[3].position );
	}

	/**
	 * @brief Calculates the length between two sample points on the line.
	 * @param t0 - first sample scalar between 0.0 and 1.0
	 * @param t1 - second sample scalar between 0.0 and 1.0
	 * @return length of curve between two points
	*/
	float lengthBetween( float t0, float t1 ) const
	{
		return Math::length( integrateAt( t1 ) - integrateAt( t0 ) );
	}
};


template <typename T>
class Linear
{
	T current;
	T target;
	T speed;

public:
	constexpr Linear( const T& initValue, const T& target, const T& speed )
		: current( initValue ), target( target ), speed( speed )
	{

	}

	constexpr void setTarget( const T& newTarget )
	{
		target = newTarget;
	}

	constexpr T getValue() const
	{
		return current;
	}

	T getValue( const float elapsedTime )
	{
		T displacement = speed * elapsedTime;
		current += Math::clamp( target - current, -displacement, displacement );
		return current;
	}
};

template <typename T>
struct Smoothed
{
	T current;
	T target;
	float speed;

	constexpr Smoothed( const T& initValue, const T& target, const T& speed )
		: current( initValue ), target( target ), speed( speed )
	{

	}

	constexpr void setTarget( const T& newTarget )
	{
		target = newTarget;
	}

	constexpr T getValue() const
	{
		return current;
	}

	T getValue( const float elapsedTime )
	{
		return current = Math::lerp( current, target, elapsedTime * speed );
	}
};

struct LinearDirection
{
	vec3f current;
	vec3f target;
	float angularSpeed;

public:
	constexpr LinearDirection( const vec3f& initValue, const vec3f& target, const float speed )
		: current( initValue ), target( target ), angularSpeed( speed )
	{

	}

	constexpr void setDirection( const vec3f& direction )
	{
		current = direction;
	}

	constexpr void setTarget( const vec3f& direction )
	{
		target = direction;
	}

	constexpr const vec3f& getDirection() const
	{
		return current;
	}

	const vec3f& getDirection( const float elapsedTime )
	{
		float product = Math::dot_3D( current, target );
		vec3f normal;

		if( product > 1.0f - Math::EPSILON<float> )
		{
			return current;
		}
		else if( product < Math::EPSILON<float> -1.0f )
		{
			normal = Math::orthogonal( current );
		}
		else
		{
			normal = Math::normalize( Math::cross( current, target ) );
		}

		return current = Math::rotate( current, normal, std::min( acosf( product ), angularSpeed ) );
	}
};

struct SmoothDirection
{
	vec3f current;
	vec3f target;
	float speed;

	constexpr SmoothDirection( const vec3f& initValue, const vec3f& target, const float speed )
		: current( initValue ), target( target ), speed( speed )
	{

	}

	constexpr void setDirection( const vec3f& direction )
	{
		current = direction;
	}

	constexpr void setTarget( const vec3f& newTarget )
	{
		target = newTarget;
	}

	constexpr const vec3f& getDirection() const
	{
		return current;
	}

	const vec3f& getDirection( const float elapsedTime )
	{
		return current = Math::slerp( current, target, elapsedTime * speed );
	}
};

#endif