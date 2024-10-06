#pragma once
#ifndef VF_CURVES_HPP
#define VF_CURVES_HPP

#include "VectorMath.hpp"
#include "mat4x4.hpp"
#include <vector>

/**
 * TODO: Make matrix scaling constexpr and use a typedef instead
 * @brief Parameters that specify the type of curve implemented.
*/
struct SplineType
{
	mat4f basis;

	constexpr SplineType( const mat4f& matrix, float factor ) :
		basis( matrix )
	{
		for( float& value : basis.m )
		{
			value *= factor;
		}
	}

	/**
	 * @brief Interpolates the result of the curve between all of the knot controls based on the sample weight vector.
	 * @param weight - sample weight vector
	 * @param knot - knot point samples
	*/
	[[nodiscard]] vec4f weightSample( const vec4f& weight, const mat4f& knot ) const
	{
		return knot * ( basis * weight );
	}
};

/**
 * @brief Container of discrete points used to form a continuous curve. 
*/
struct Spline : public std::vector<vec4f>
{
	constexpr static uint64_t KNOT_SIZE = 4LLU;

	constexpr static SplineType POLYLINE    = { { 0.0f, 1.0f, 0.0f, 0.0f,    0.0f, -1.0f, 1.0f, 0.0f,  0.0f,  0.0f, 0.0f,  0.0f,    0.0f, 0.0f,  0.0f, 0.0f }, 1.0f };
	constexpr static SplineType HERMITE     = { { 1.0f, 0.0f, 0.0f, 0.0f,    0.0f, 1.0f, 0.0f, 0.0f,  -3.0f, -2.0f, 3.0f,  0.0f,    2.0f, 1.0f, -2.0f, 1.0f }, 1.0f };
	constexpr static SplineType BEZIER      = { { 1.0f, 0.0f, 0.0f, 0.0f,   -3.0f, 3.0f, 0.0f, 0.0f,   3.0f, -6.0f, 3.0f,  0.0f,   -1.0f, 3.0f, -3.0f, 1.0f }, 1.0f };
	constexpr static SplineType CATMULL_ROM = { { 0.0f, 2.0f, 0.0f, 0.0f,   -1.0f, 0.0f, 1.0f, 0.0f,   2.0f, -5.0f, 4.0f, -1.0f,   -1.0f, 3.0f, -3.0f, 1.0f }, 0.5f };
	constexpr static SplineType BSPLINE     = { { 1.0f, 4.0f, 1.0f, 0.0f,   -3.0f, 0.0f, 3.0f, 0.0f,   3.0f, -6.0f, 3.0f,  0.0f,   -1.0f, 3.0f, -3.0f, 1.0f }, 1.0f / 6.0f };

	SplineType type;

	/**
	 * @brief Creates a curve of the specified type.
	 * @param type - curve type
	*/
	constexpr Spline( const SplineType& type = CATMULL_ROM ) :
		std::vector<vec4f>(),
		type( type )
	{

	}

	/**
	 * @brief Creates a curve with initial points.
	 * @param list - initial points
	*/
	constexpr Spline( const std::initializer_list<vec4f>& list ) :
		std::vector<vec4f>( list ),
		type( CATMULL_ROM )
	{

	}

	constexpr uint64_t getKnotCount() const
	{
		return size() - ( KNOT_SIZE - 1LLU );
	}

	/**
	 * @brief Samples the value of the curve at a scalar point.
	 * @param t - sample scalar between 0.0 and 1.0
	 * @return sampled vector at scalar point
	*/
	vec4f sampleAt( float t ) const
	{
		if( size() < KNOT_SIZE ) return Math::ZERO<vec4f>;
		float knotCount = static_cast<float>( getKnotCount() );

		// Convert absolute t weight into relative t weight
		float absolute = t * knotCount;
		float base = std::clamp( floorf( absolute ), 0.0f, knotCount - 1.0f );
		float weight = absolute - base;

		// Get knot matrix pointer
		const vec4f* ptr = data() + static_cast<uint64_t>( base );
		return type.weightSample( { 1.0f, weight, weight * weight, weight * weight * weight }, *reinterpret_cast<const mat4f*>( ptr ) );
	}

	/**
	 * @brief Samples the derivative value of the curve at a scalar point.
	 * @param t - sample scalar between 0.0 and 1.0
	 * @return derivative vector at scalar point
	*/
	vec4f derivativeAt( float t ) const
	{
		if( size() < KNOT_SIZE ) return Math::ZERO<vec4f>;
		float knotCount = static_cast<float>( getKnotCount() );

		// Convert absolute t weight into relative t weight
		float absolute = t * knotCount;
		float base = std::clamp( floorf( absolute ), 0.0f, knotCount - 1.0f );
		float weight = absolute - base;

		const vec4f* ptr = data() + static_cast<uint64_t>( base );
		return type.weightSample( { 0.0f, 1.0f, 2.0f * weight, 3.0f * weight * weight }, *reinterpret_cast<const mat4f*>( ptr ) );
	}

	/**
	 * @brief Samples the integrated value of the curve at a scalar point.
	 * @param t - sample scalar between 0.0 and 1.0
	 * @return integrated vector at scalar point
	*/
	vec4f integrateAt( float t ) const
	{
		if( size() < KNOT_SIZE ) return Math::ZERO<vec4f>;
		float knotCount = static_cast<float>( getKnotCount() );

		// Convert absolute t weight into relative t weight
		float absolute = t * knotCount;
		float base = std::clamp( floorf( absolute ), 0.0f, knotCount - 1.0f );
		float weight = absolute - base;
		float weight2 = weight * weight;

		const vec4f* ptr = data() + static_cast<uint64_t>( base );
		return type.weightSample( { weight, weight2 / 2.0f, weight2 * weight / 3.0f, weight2 * weight2 / 4.0f }, *reinterpret_cast<const mat4f*>( ptr ) );
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

	/**
	 * @brief Finds the value of t that is closest to the vector within the local range.
	 * @param point - arbitrary point
	 * @param min - local range minimum
	 * @param max - local range maximum
	 * @param e - epsilon match threshold
	*/
	float localMinimum( const vec4f& point, float min, float max, float e = 1e-3f ) const
	{
		float m = min;
		float n = max;
		float k = ( n + m ) / 2.0f;

		while( ( n - m ) > e )
		{
			float d0 = Math::distance2( point, sampleAt( k - e ) );
			float d1 = Math::distance2( point, sampleAt( k + e ) );

			if( d0 < d1 ) n = k;
			else		  m = k;

			k = ( n + m ) / 2.0f;
		}

		return k;
	}

	/**
	 * @brief Finds the value of t that is closest to the vector along the curve.
	 * @param point - arbitrary point
	 * @param e - epsilon match threshold
	*/
	float getClosest( const vec4f& point, float e = 1e-3f ) const
	{
		if( size() < KNOT_SIZE ) return NAN;
		size_t maxIndex = size() - 1LLU;
		float maxIndexF = static_cast<float>( maxIndex );

		// Find pair of control points closest to point
		float t0 = 0.0f;
		float t1 = 1.0f / maxIndexF;
		float min = t0;
		float max = t1;
		float d0 = Math::distance2( point, sampleAt( t0 ) );
		float d1 = Math::distance2( point, sampleAt( t1 ) );
		float minDistance2 = d0 + d1;

		for( uint64_t i = 1LLU; i < maxIndex; ++i )
		{
			t0 = t1;
			d0 = d1;
			t1 = static_cast<float>( i + 1LLU ) / maxIndexF;
			d1 = Math::distance2( point, sampleAt( t1 ) );
			float sumDistance2 = d0 + d1;

			if( sumDistance2 < minDistance2 )
			{
				minDistance2 = sumDistance2;
				min = t0;
				max = t1;
			}
		}

		// Calculate local minimum between points
		return localMinimum( point, min, max, e );
	}

	/**
	 * @brief Creates a curve basis matrix with a specified level of "smoothness".
	 * @param s - scale of the cardinal curve
	 * @return basis matrix spline type
	*/
	constexpr static SplineType createCardinalBasis( float s )
	{
		return { { 0.0f, 1.0f, 0.0f, 0.0f, -s, 0.0f, s, 0.0f, 2.0f * s, s - 3.0f, 3.0f - 2.0f * s, -s, -s, 2.0f - s, s - 2.0f, s }, 1.0f };
	}
};

/**
 * TODO: This struct might not need to exist
 * @brief Pair of spline's that define a path.
*/
struct PathSpline
{
	Spline position;
	Spline orientation;

	constexpr PathSpline( const SplineType& type ) :
		position( type ),
		orientation( type )
	{

	}

	constexpr void setType( const SplineType& type )
	{
		position.type = type;
		orientation.type = type;
	}

	constexpr void addPoint( const vec4f& point, const vec3f& normal )
	{
		position.emplace_back( point );
		orientation.emplace_back( normal );
	}

	/**
	 * @brief Finds the value of t that is closest to the point along the curve.
	 * @param point - arbitrary point
	 * @param e - epsilon match threshold
	*/
	float getClosest( const vec4f& point, float e = 1e-3f )
	{
		return position.getClosest( point, e );
	}
};

#endif