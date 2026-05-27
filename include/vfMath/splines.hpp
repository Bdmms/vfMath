#pragma once
#ifndef VF_CURVES_HPP
#define VF_CURVES_HPP

#include "vec_math.hpp"
#include "matrix.hpp"
#include <vector>

namespace vf::math::splines
{
	using namespace Math;

	/**
	 * @brief Parameters that specify the type of curve implemented.
	*/
	template<typename T> requires std::floating_point<T>
	struct SplineType
	{
		mat4<T> basis;

		constexpr SplineType() = default;

		constexpr SplineType( const mat4<T>& matrix, T factor ) :
			basis( matrix )
		{
			for( vec4<T>& col : basis.cols )
			{
				col *= factor;
			}
		}

		/**
		 * @brief Interpolates the result of the curve between all of the knot controls based on the sample weight vector.
		 * @param weight - sample weight vector
		 * @param knot - knot point samples
		*/
		[[nodiscard]] constexpr vec4<T> weightSample( const vec4<T>& weight, const mat4<T>& knot ) const
		{
			return knot * ( basis * weight );
		}
	};

	template<typename T> constexpr static SplineType<T> POLYLINE;
	template<typename T> constexpr static SplineType<T> HERMITE;
	template<typename T> constexpr static SplineType<T> BEZIER;
	template<typename T> constexpr static SplineType<T> CATMULL_ROM;
	template<typename T> constexpr static SplineType<T> BSPLINE;

	template<> constexpr static SplineType<f32> POLYLINE<f32> = { {
			vec4f32{  0.0f,  1.0f,  0.0f,  0.0f },
			vec4f32{  0.0f, -1.0f,  1.0f,  0.0f },
			vec4f32{  0.0f,  0.0f,  0.0f,  0.0f },
			vec4f32{  0.0f,  0.0f,  0.0f,  0.0f } }, 1.0f };
	template<> constexpr static SplineType<f32> HERMITE<f32> = { {
			vec4f32{  1.0f,  0.0f,  0.0f, 0.0f },
			vec4f32{  0.0f,  1.0f,  0.0f, 0.0f },
			vec4f32{ -3.0f, -2.0f,  3.0f, 0.0f },
			vec4f32{  2.0f,  1.0f, -2.0f, 1.0f } }, 1.0f };
	template<> constexpr static SplineType<f32> BEZIER<f32> = { {
			vec4f32{  1.0f,  0.0f,  0.0f, 0.0f },
			vec4f32{ -3.0f,  3.0f,  0.0f, 0.0f },
			vec4f32{  3.0f, -6.0f,  3.0f, 0.0f },
			vec4f32{ -1.0f,  3.0f, -3.0f, 1.0f } }, 1.0f };
	template<> constexpr static SplineType<f32> CATMULL_ROM<f32> = { {
			vec4f32{  0.0f,  2.0f,  0.0f,  0.0f },
			vec4f32{ -1.0f,  0.0f,  1.0f,  0.0f },
			vec4f32{  2.0f, -5.0f,  4.0f, -1.0f },
			vec4f32{ -1.0f,  3.0f, -3.0f,  1.0f } }, 0.5f };
	template<> constexpr static SplineType<f32> BSPLINE<f32> = { {
			vec4f32{  1.0f,  4.0f,  1.0f, 0.0f },
			vec4f32{ -3.0f,  0.0f,  3.0f, 0.0f },
			vec4f32{  3.0f, -6.0f,  3.0f, 0.0f },
			vec4f32{ -1.0f,  3.0f, -3.0f, 1.0f } }, 1.0f / 6.0f };

	template<> constexpr static SplineType<f64> POLYLINE<f64> = { {
			vec4f64{  0.0,  1.0,  0.0,  0.0 },
			vec4f64{  0.0, -1.0,  1.0,  0.0 },
			vec4f64{  0.0,  0.0,  0.0,  0.0 },
			vec4f64{  0.0,  0.0,  0.0,  0.0 } }, 1.0 };
	template<> constexpr static SplineType<f64> HERMITE<f64> = { {
			vec4f64{  1.0,  0.0,  0.0, 0.0 },
			vec4f64{  0.0,  1.0,  0.0, 0.0 },
			vec4f64{ -3.0, -2.0,  3.0, 0.0 },
			vec4f64{  2.0,  1.0, -2.0, 1.0 } }, 1.0 };
	template<> constexpr static SplineType<f64> BEZIER<f64> = { {
			vec4f64{  1.0,  0.0,  0.0, 0.0 },
			vec4f64{ -3.0,  3.0,  0.0, 0.0 },
			vec4f64{  3.0, -6.0,  3.0, 0.0 },
			vec4f64{ -1.0,  3.0, -3.0, 1.0 } }, 1.0 };
	template<> constexpr static SplineType<f64> CATMULL_ROM<f64> = { {
			vec4f64{  0.0,  2.0,  0.0,  0.0 },
			vec4f64{ -1.0,  0.0,  1.0,  0.0 },
			vec4f64{  2.0, -5.0,  4.0, -1.0 },
			vec4f64{ -1.0,  3.0, -3.0,  1.0 } }, 0.5 };
	template<> constexpr static SplineType<f64> BSPLINE<f64> = { {
			vec4f64{  1.0,  4.0,  1.0, 0.0 },
			vec4f64{ -3.0,  0.0,  3.0, 0.0 },
			vec4f64{  3.0, -6.0,  3.0, 0.0 },
			vec4f64{ -1.0,  3.0, -3.0, 1.0 } }, 1.0 / 6.0 };

	/**
	 * @brief Creates a curve basis matrix with a specified level of "smoothness".
	 * @param s - scale of the cardinal curve
	 * @return basis matrix spline type
	*/
	template<typename T>
	constexpr static SplineType<T> createCardinalBasis( T s )
	{
		return { {
			vec4<T>{ 0.0f, 1.0f, 0.0f, 0.0f },
			vec4<T>{ -s, 0.0f, s, 0.0f },
			vec4<T>{ 2.0f * s, s - 3.0f, 3.0f - 2.0f * s, -s },
			vec4<T>{ -s, 2.0f - s, s - 2.0f, s } },
			1.0f
		};
	}

	/**
	 * @brief Container of discrete points used to form a continuous curve.
	*/
	template<typename T> requires std::floating_point<T>
	struct Spline : public std::vector<vec4<T>>
	{
		using Parent = std::vector<vec4<T>>;

		constexpr static size_t KNOT_SIZE = 4LLU;

		SplineType<T> type;

		/**
		 * @brief Creates a curve of the specified type.
		 * @param type - curve type
		*/
		constexpr Spline( const SplineType<T>& type = CATMULL_ROM<T> ) :
			Parent(),
			type( type )
		{

		}

		/**
		 * @brief Creates a curve with initial points.
		 * @param list - initial points
		*/
		constexpr Spline( const std::initializer_list<vec4<T>>& list ) :
			Parent( list ),
			type( CATMULL_ROM<T> )
		{

		}

		constexpr uint64_t getKnotCount() const
		{
			return Parent::size() - ( KNOT_SIZE - 1LLU );
		}

		/**
		 * @brief Samples the value of the curve at a scalar point.
		 * @param t - sample scalar between 0.0 and 1.0
		 * @return sampled vector at scalar point
		*/
		vec4<T> sampleAt( T t ) const
		{
			if( Parent::size() < KNOT_SIZE ) return ZERO<vec4<T>>;
			T knotCount = static_cast<T>( getKnotCount() );

			// Convert absolute t weight into relative t weight
			T absolute = t * knotCount;
			T base = clamp( std::floor( absolute ), ZERO<T>, knotCount - ONES<T> );
			float weight = absolute - base;

			// Get knot matrix pointer
			const vec4<T>* ptr = Parent::data() + static_cast<uint64_t>( base );
			return type.weightSample( { ONES<T>, weight, weight * weight, weight * weight * weight }, *reinterpret_cast<const mat4<T>*>( ptr ) );
		}

		/**
		 * @brief Samples the derivative value of the curve at a scalar point.
		 * @param t - sample scalar between 0.0 and 1.0
		 * @return derivative vector at scalar point
		*/
		vec4<T> derivativeAt( T t ) const
		{
			if( Parent::size() < KNOT_SIZE ) return ZERO<vec4<T>>;
			T knotCount = static_cast<T>( getKnotCount() );

			// Convert absolute t weight into relative t weight
			T absolute = t * knotCount;
			T base = clamp( std::floor( absolute ), ZERO<T>, knotCount - ONES<T> );
			T weight = absolute - base;

			const vec4<T>* ptr = Parent::data() + static_cast<uint64_t>( base );
			return type.weightSample( { ZERO<T>, ONES<T>, TWO<T> * weight, 3.0f * weight * weight }, *reinterpret_cast<const mat4<T>*>( ptr ) );
		}

		/**
		 * @brief Samples the integrated value of the curve at a scalar point.
		 * @param t - sample scalar between 0.0 and 1.0
		 * @return integrated vector at scalar point
		*/
		vec4<T> integrateAt( T t ) const
		{
			if( Parent::size() < KNOT_SIZE ) return ZERO<vec4<T>>;
			T knotCount = static_cast<T>( getKnotCount() );

			// Convert absolute t weight into relative t weight
			T absolute = t * knotCount;
			T base = clamp( std::floor( absolute ), ZERO<T>, knotCount - ONES<T> );
			T weight = absolute - base;
			T weight2 = weight * weight;

			const vec4<T>* ptr = Parent::data() + static_cast<uint64_t>( base );
			return type.weightSample( { weight, weight2 / TWO<T>, weight2 * weight / 3.0f, weight2 * weight2 / 4.0f }, *reinterpret_cast<const mat4<T>*>( ptr ) );
		}

		/**
		 * @brief Calculates the length between two sample points on the line.
		 * @param t0 - first sample scalar between 0.0 and 1.0
		 * @param t1 - second sample scalar between 0.0 and 1.0
		 * @return length of curve between two points
		*/
		template<typename T>
		T lengthBetween( T t0, T t1 ) const
		{
			return length( integrateAt( t1 ) - integrateAt( t0 ) );
		}

		/**
		 * @brief Finds the value of t that is closest to the vector within the local range.
		 * @param point - arbitrary point
		 * @param min - local range minimum
		 * @param max - local range maximum
		 * @param e - epsilon match threshold
		*/
		template<typename T>
		T localMinimum( const vec4<T>& point, T min, T max, T e = 1e-3f ) const
		{
			T m = min;
			T n = max;
			T k = ( n + m ) / TWO<T>;

			while( ( n - m ) > e )
			{
				T d0 = distance2( point, sampleAt( k - e ) );
				T d1 = distance2( point, sampleAt( k + e ) );

				if( d0 < d1 ) n = k;
				else		  m = k;

				k = ( n + m ) / TWO<T>;
			}

			return k;
		}

		/**
		 * @brief Finds the value of t that is closest to the vector along the curve.
		 * @param point - arbitrary point
		 * @param e - epsilon match threshold
		*/
		template<typename T>
		T getClosest( const vec4<T>& point, T e = 1e-3f ) const
		{
			if( Parent::size() < KNOT_SIZE ) return NAN;
			size_t maxIndex = Parent::size() - 1LLU;
			T maxIndexF = static_cast<T>( maxIndex );

			// Find pair of control points closest to point
			T t0 = ZERO<T>;
			T t1 = ONES<T> / maxIndexF;
			T min = t0;
			T max = t1;
			T d0 = distance2( point, sampleAt( t0 ) );
			T d1 = distance2( point, sampleAt( t1 ) );
			T minDistance2 = d0 + d1;

			for( uint64_t i = 1LLU; i < maxIndex; ++i )
			{
				t0 = t1;
				d0 = d1;
				t1 = static_cast<T>( i + 1LLU ) / maxIndexF;
				d1 = distance2( point, sampleAt( t1 ) );
				T sumDistance2 = d0 + d1;

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
	};

	/**
	 * TODO: This struct might not need to exist
	 * @brief Pair of spline's that define a path.
	*/
	template<typename T>
	struct PathSpline
	{
		Spline<T> position;
		Spline<T> orientation;

		constexpr PathSpline( const SplineType<T>& type ) :
			position( type ),
			orientation( type )
		{

		}

		constexpr void setType( const SplineType<T>& type )
		{
			position.type = type;
			orientation.type = type;
		}

		constexpr void addPoint( const vec4<T>& point, const vec4<T>& normal )
		{
			position.emplace_back( point );
			orientation.emplace_back( normal );
		}

		/**
		 * @brief Finds the value of t that is closest to the point along the curve.
		 * @param point - arbitrary point
		 * @param e - epsilon match threshold
		*/
		T getClosest( const vec4<T>& point, T e = 1e-3f )
		{
			return position.getClosest( point, e );
		}
	};
}

#endif