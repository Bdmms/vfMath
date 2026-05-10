#pragma once
#ifndef VF_VECTOR_MATH_HPP
#define VF_VECTOR_MATH_HPP

#include "Math.hpp"
#include "vec.hpp"

typedef vec3f AxisAngle;

// TODO:
// template<typename V>
// concept IsVector = std::is_same<V, vec4f>::value || std::is_same<V, vec2f>::value;

/**
 * @brief Utilities for 4D vectors
*/
namespace Math
{
	template<typename ElementType>
	inline constexpr static vec2<ElementType> ZERO<vec2<ElementType>> = { Math::ZERO<ElementType>, Math::ZERO<ElementType> };
	template<typename ElementType>
	inline constexpr static vec4<ElementType> ZERO<vec4<ElementType>> = { Math::ZERO<ElementType>, Math::ZERO<ElementType>, Math::ZERO<ElementType>, Math::ZERO<ElementType> };

	template<typename ElementType>
	inline constexpr static vec2<ElementType> ONES<vec2<ElementType>> = { Math::ONES<ElementType>, Math::ONES<ElementType> };
	template<typename ElementType>
	inline constexpr static vec4<ElementType> ONES<vec4<ElementType>> = { Math::ONES<ElementType>, Math::ONES<ElementType>, Math::ONES<ElementType>, Math::ONES<ElementType> };

	template<typename ElementType>
	inline constexpr static vec2<ElementType> IDENTITY<vec2<ElementType>> = ZERO<vec2<ElementType>>;
	template<typename ElementType>
	inline constexpr static vec4<ElementType> IDENTITY<vec4<ElementType>> = ZERO<vec4<ElementType>>;

	template<typename ElementType>
	inline constexpr static vec2<ElementType> NEGATIVE<vec2<ElementType>> = { Math::NEGATIVE<ElementType>, Math::NEGATIVE<ElementType> };
	template<typename ElementType>
	inline constexpr static vec4<ElementType> NEGATIVE<vec4<ElementType>> = { Math::NEGATIVE<ElementType>, Math::NEGATIVE<ElementType>, Math::NEGATIVE<ElementType>, Math::NEGATIVE<ElementType> };

	template<typename ElementType>
	inline constexpr static vec2<ElementType> MIN<vec2<ElementType>> = { Math::MIN<ElementType>, Math::MIN<ElementType> };
	template<typename ElementType>
	inline constexpr static vec4<ElementType> MIN<vec4<ElementType>> = { Math::MIN<ElementType>, Math::MIN<ElementType>, Math::MIN<ElementType>, Math::MIN<ElementType> };

	template<typename ElementType>
	inline constexpr static vec2<ElementType> MAX<vec2<ElementType>> = { Math::MAX<ElementType>, Math::MAX<ElementType> };
	template<typename ElementType>
	inline constexpr static vec4<ElementType> MAX<vec4<ElementType>> = { Math::MAX<ElementType>, Math::MAX<ElementType>, Math::MAX<ElementType>, Math::MAX<ElementType> };

	template<typename ElementType>
	inline constexpr static vec2<ElementType> PI<vec2<ElementType>> = { Math::PI<ElementType>, Math::PI<ElementType> };
	template<typename ElementType>
	inline constexpr static vec4<ElementType> PI<vec4<ElementType>> = { Math::PI<ElementType>, Math::PI<ElementType>, Math::PI<ElementType>, Math::PI<ElementType> };

	template<typename ElementType>
	inline constexpr static vec2<ElementType> TWO_PI<vec2<ElementType>> = { Math::TWO_PI<ElementType>, Math::TWO_PI<ElementType> };
	template<typename ElementType>
	inline constexpr static vec4<ElementType> TWO_PI<vec4<ElementType>> = { Math::TWO_PI<ElementType>, Math::TWO_PI<ElementType>, Math::TWO_PI<ElementType>, Math::TWO_PI<ElementType> };

	template<typename ElementType>
	inline constexpr static vec2<ElementType> HALF_PI<vec2<ElementType>> = { Math::HALF_PI<ElementType>, Math::HALF_PI<ElementType> };
	template<typename ElementType>
	inline constexpr static vec4<ElementType> HALF_PI<vec4<ElementType>> = { Math::HALF_PI<ElementType>, Math::HALF_PI<ElementType>, Math::HALF_PI<ElementType>, Math::HALF_PI<ElementType> };

	// Unit vectors for each axis
	namespace axis
	{
		template<typename T> inline constexpr static T X;
		template<typename T> inline constexpr static T Y;
		template<typename T> inline constexpr static T Z;
		template<typename T> inline constexpr static T W;

		template<typename ElementType>
		inline constexpr static vec2<ElementType> X<vec2<ElementType>> = { Math::ONES<ElementType>, Math::ZERO<ElementType> };
		template<typename ElementType>
		inline constexpr static vec4<ElementType> X<vec4<ElementType>> = { Math::ONES<ElementType>, Math::ZERO<ElementType>, Math::ZERO<ElementType>, Math::ZERO<ElementType> };

		template<typename ElementType>
		inline constexpr static vec2<ElementType> Y<vec2<ElementType>> = { Math::ZERO<ElementType>, Math::ONES<ElementType> };
		template<typename ElementType>
		inline constexpr static vec4<ElementType> Y<vec4<ElementType>> = { Math::ZERO<ElementType>, Math::ONES<ElementType>, Math::ZERO<ElementType>, Math::ZERO<ElementType> };

		template<typename ElementType>
		inline constexpr static vec4<ElementType> Z<vec4<ElementType>> = { Math::ZERO<ElementType>, Math::ZERO<ElementType>, Math::ONES<ElementType>, Math::ZERO<ElementType> };

		template<typename ElementType>
		inline constexpr static vec4<ElementType> W<vec4<ElementType>> = { Math::ZERO<ElementType>, Math::ZERO<ElementType>, Math::ZERO<ElementType>, Math::ONES<ElementType> };
	};

	/**
	 * @brief Returns a composite value resulting on the lane-wise condition of the input vector.
	 * @param input - condition input vector
	 * @param trueVal - vector components returned on true input
	 * @param falseVal - vector components returned on false input
	 * @return output vector
	*/
	template<size_t N, typename T>
	[[nodiscard]] constexpr vec<N, T> condition( const vec<N, T>& input, const vec<N, T>& trueVal, const vec<N, T>& falseVal )
	{
		return ( input & trueVal ) | ( ( ~input ) & falseVal );
	}

	/**
	 * @brief Checks if two overlapping boundaries overlap
	 * @param min0 - minimum of first bounds
	 * @param max0 - maximum of first bounds
	 * @param min1 - minimum of second bounds
	 * @param max1 - maximum of second bounds
	 * @return Boolean vector for whether the two boundaries overlap 
	*/
	template<size_t N, typename T>
	[[nodiscard]] constexpr vec<N, T> overlaps( const vec<N, T>& min0, const vec<N, T>& max0, const vec<N, T>& min1, const vec<N, T>& max1 )
	{
		return ( min0 >= min1 && min0 <= max1 ) || ( min1 >= min0 && min1 <= max0 );
	}

	/**
	 * @brief Tests if two bounds overlap with each other across each axis
	 * @param a - first bound range
	 * @param b - second bound range
	 * @return Whether the boundaries overlap
	*/
	constexpr bool overlaps3D( const Bounds<vec4f>& a, const Bounds<vec4f>& b )
	{
		vec4f result = overlaps( a.min, a.max, b.min, b.max );
		return result.x && result.y && result.z;
	}

	/**
	 * @brief Tests if two bounds overlap with each other across each axis
	 * @param a - first bound range
	 * @param b - second bound range
	 * @return Whether the boundaries overlap
	*/
	constexpr bool overlaps3D( const Bounds<vec4i>& a, const Bounds<vec4i>& b )
	{
		vec4i result = overlaps( a.min, a.max, b.min, b.max );
		return result.x && result.y && result.z;
	}

	/**
	 * @brief Calculates the 2D dot product of this vector with another vector
	 * @param a - first vector
	 * @param b - second vector
	 * @return dot product result
	*/
	template<typename T>
	[[nodiscard]] constexpr T dot( const vec2<T>& a, const vec2<T>& b )
	{
		return a.x * b.x + a.y * b.y;
	}

	/**
	 * @brief Calculates the 4D dot product of this vector with another vector
	 * @param a - first vector
	 * @param b - second vector
	 * @return dot product result
	*/
	template<typename T>
	[[nodiscard]] constexpr T dot( const vec4<T>& a, const vec4<T>& b )
	{
		return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
	}

	/**
	 * @brief Calculates the 3D dot product of this vector with another vector
	 * @param a - first vector
	 * @param b - second vector
	 * @return dot product result
	*/
	template<typename T>
	[[nodiscard]] constexpr T dot_3D( const vec4<T>& a, const vec4<T>& b )
	{
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	/**
	 * @brief Calculates the cross product between two 2D vectors
	 * @param a - first vector
	 * @param b - second vector
	 * @return cross product of the two vectors
	*/
	template<typename T>
	[[nodiscard]] constexpr vec4<T> cross( const vec2<T>& a, const vec2<T>& b )
	{
		return { 
			Math::ZERO<T>, 
			Math::ZERO<T>, 
			a.x * b.y - a.y * b.x, 
			Math::ZERO<T> 
		};
	}

	/**
	 * @brief Calculates the cross product between two 3D vectors
	 * @param a - first vector
	 * @param b - second vector
	 * @return cross product of the two vectors
	*/
	template<typename T>
	[[nodiscard]] constexpr vec4<T> cross( const vec4<T>& a, const vec4<T>& b )
	{
		return {
			a.y * b.z - a.z * b.y,
			a.z * b.x - a.x * b.z,
			a.x * b.y - a.y * b.x,
			Math::ZERO<T>
		};
	}

	/**
	 * @brief Calculates the squared length of the vector
	 * @return squared length of the vector
	*/
	template<typename T>
	[[nodiscard]] constexpr T length2( const vec2<T>& v )
	{
		return v.x * v.x + v.y * v.y;
	}

	/**
	 * @brief Calculates the squared length of the vector
	 * @return squared length of the vector
	*/
	template<typename T>
	[[nodiscard]] constexpr T length2( const vec4<T>& v )
	{
		return v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w;
	}

	/**
	 * @brief Calculates the length of the vector
	 * @return length of the vector
	*/
	template<typename T>
	[[nodiscard]] static T length( const vec2<T>& v ) noexcept
	{
		return sqrt( v.x * v.x + v.y * v.y );
	}

	/**
	 * @brief Calculates the length of the vector
	 * @return length of the vector
	*/
	template<typename T>
	[[nodiscard]] static T length( const vec4<T>& v ) noexcept
	{
		return sqrt( v.x * v.x + v.y * v.y + v.z * v.z + v.w * v.w );
	}

	/**
	 * @brief Calculates the unit vector. Returns zero vector if length is zero.
	 * @param v - vector
	 * @return normalized vector
	*/
	template<size_t N, typename T>
	[[nodiscard]] static vec<N, T> normalize( const vec<N, T>& v )
	{
		T vLength = length( v );
		if( vLength == Math::ZERO<T> ) return Math::ZERO<vec<N, T>>;
		return v / vLength;
	}

	/**
	 * @brief Calculates the squared distance between vectors
	 * @param a - first vector
	 * @param b - second vector
	 * @return squared distance
	*/
	template<size_t N, typename T>
	[[nodiscard]] constexpr T distance2( const vec<N, T>& a, const vec<N, T>& b )
	{
		return length2( a - b );
	}

	/**
	 * @brief Calculates the distance between vectors
	 * @param a - first vector
	 * @param b - second vector
	 * @return distance
	*/
	template<size_t N, typename T>
	[[nodiscard]] static T distance( const vec<N, T>& a, const vec<N, T>& b )
	{
		return sqrt( distance2( a, b ) );
	}

	/**
	 * @brief Projects one vector onto another
	 * @param a - first vector
	 * @param b - second vector
	 * @return projected vector
	*/
	template<typename T> [[nodiscard]] constexpr T project( const T& a, const T& b )
	{
		return b * ( dot( a, b ) / dot( b, b ) );
	}

	/**
	 * @brief Reflects a vector across another
	 * @param a - ray vector
	 * @param b - normal vector (normalized)
	 * @return reflected vector
	*/
	template<typename T> [[nodiscard]] constexpr T reflect( const T& a, const T& b )
	{
		return a - b * dot( a, b ) * 2.0f; // TODO: Constant floating point value
	}

	/**
	 * @brief Truncates the vector by a minimum
	 * @param vector - vector
	 * @param min - minimum vector
	 * @return minimum vector
	*/
	template<size_t N, typename T>
	[[nodiscard]] constexpr vec<N, T> min( const vec<N, T>& vector, const vec<N, T>& min )
	{
		vec<N, T> r;
		vf::simd::min( r, vector, min );
		return r;
	}

	/**
	 * @brief Truncates the vector by a maximum
	 * @param vector - vector
	 * @param max - maximum vector
	 * @return maximum vector
	*/
	template<size_t N, typename T>
	[[nodiscard]] constexpr vec<N, T> max( const vec<N, T>& vector, const vec<N, T>& max )
	{
		vec<N, T> r;
		vf::simd::max( r, vector, max );
		return r;
	}

	/**
	 * @brief Truncates all elements the vector between minimum and maximum
	 * @param value - initial vector
	 * @param min - minimum vector of clamp (optional)
	 * @param max - maximum vector of clamp (optional)
	 * @return clamped vector
	*/
	template<size_t N, typename T>
	[[nodiscard]] constexpr vec<N, T> clamp( vec<N, T> vector, const vec<N, T>& min = ZERO<vec<N, T>>, const vec<N, T>& max = ONES<vec<N, T>> )
	{
		vf::simd::max( vector, vector, min );
		vf::simd::min( vector, vector, max );
		return vector;
	}

	/**
	 * @brief Rounds the vector down
	 * @param a - vector
	 * @return rounded vector
	*/
	template<size_t N, typename T>
	[[nodiscard]] static vec<N, T> floor( vec<N, T> v )
	{
		vf::simd::floor( v, v );
		return v;
	}

	/**
	 * @brief Rounds the vector up
	 * @param a - vector
	 * @return rounded vector
	*/
	template<size_t N, typename T>
	[[nodiscard]] static vec<N, T> ceil( vec<N, T> v )
	{
		vf::simd::ceil( v, v );
		return v;
	}
	
	/**
	 * @brief Rounds the vector to the closest integer
	 * @param a - vector
	 * @return rounded vector
	*/
	template<size_t N, typename T>
	[[nodiscard]] static vec<N, T> round( vec<N, T> v )
	{
		vf::simd::round( v, v );
		return v;
	}

	/**
	 * @brief Calculates the square root of a vector
	 * @param v - vector
	 * @return square root vector
	*/
	template<size_t N, typename T>
	[[nodiscard]] static vec<N, T> sqrt( vec<N, T> v )
	{
		vf::simd::sqrt( v, v );
		return v;
	}

	/**
	 * @brief Calculates the absolute of a vector
	 * @param a - vector
	 * @return absolute vector
	*/
	template<size_t N, typename T>
	[[nodiscard]] constexpr vec<N, T> abs( const vec<N, T>& a )
	{
		return condition( a >= ZERO<vec<N, T>>, a, -a );
	}

	/**
	 * @brief Determines the sign of the vector's components in the form of 1.0 or -1.0
	 * @param vector - vector components
	 * @return sign of the vector components
	*/
	template<size_t N, typename T>
	[[nodiscard]] constexpr vec<N, T> sign( const vec<N, T>& vector )
	{
		return Math::condition( vector >= Math::ZERO<vec<N, T>>, Math::ONES<vec<N, T>>, Math::NEGATIVE<vec<N, T>> );
	}

	/**
	 * @brief Calculates the sine of the vector
	 * @param a - vector
	 * @return sin vector
	*/
	[[nodiscard]] static vec2f sin( const vec2f v )
	{
		return { sinf( v.x ), sinf( v.y ) };
	}

	/**
	 * @brief Calculates the sine of the vector
	 * @param a - vector
	 * @return sin vector
	*/
	[[nodiscard]] static vec4f sin( const vec4f& v )
	{
		vec4f r;
		r.simd = _mm_sin_ps( v.simd );
		return r;
	}

	/**
	 * @brief Calculates the cosine of the vector
	 * @param a - vector
	 * @return cos vector
	*/
	[[nodiscard]] static vec2f cos( const vec2f v )
	{
		return { cosf( v.x ), cosf( v.y ) };
	}

	/**
	 * @brief Calculates the cosine of the vector
	 * @param a - vector
	 * @return cos vector
	*/
	[[nodiscard]] static vec4f cos( const vec4f& v )
	{
		vec4f r;
		r.simd = _mm_cos_ps( v.simd );
		return r;
	}

	/**
	 * @brief Calculates the tan of the vector
	 * @param a - vector
	 * @return tan vector
	*/
	[[nodiscard]] static vec2f tan( const vec2f v )
	{
		return { tanf( v.x ), tanf( v.y ) };
	}

	/**
	 * @brief Calculates the tan of the vector
	 * @param a - vector
	 * @return tan vector
	*/
	[[nodiscard]] static vec4f tan( const vec4f& v )
	{
		vec4f r;
		r.simd = _mm_tan_ps( v.simd );
		return r;
	}

	/**
	 * @brief Calculates a vector to a power of another vector
	 * @param a - base vector
	 * @param b - exponent vector
	 * @return resulting vector
	*/
	[[nodiscard]] static vec2f pow( const vec2f a, const vec2f b )
	{
		return { powf( a.x, b.x ), powf( a.y, b.y ) };
	}

	/**
	 * @brief Calculates a vector to a power of another vector
	 * @param a - base vector
	 * @param b - exponent vector
	 * @return resulting vector
	*/
	[[nodiscard]] static vec4f pow( const vec4f& a, const vec4f& b )
	{
		vec4f r;
		r.simd = _mm_pow_ps( a.simd, b.simd );
		return r;
	}

	/**
	 * @brief Generates an orthogonal direction vector
	 * @param vector - non-zero vector (normalized)
	 * @return direction orthogonal to vector
	*/
	template<typename T>
	[[nodiscard]] constexpr vec2<T> orthogonal( const vec2<T>& vector )
	{
		return { vector.y, -vector.x };
	}

	/**
	 * @brief Generates an arbitrary orthogonal direction vector
	 * @param vector - non-zero vector (normalized)
	 * @return direction orthogonal to vector
	*/
	template<typename T>
	[[nodiscard]] static vec4<T> orthogonal( const vec4<T>& vector )
	{
		// TODO: This is a 3D vector operation, not a 4D vector
		return fabsf( dot_3D( Math::axis::X<vec4<T>>, vector ) ) > 0.9f ?
			Math::normalize( Math::cross( vector, Math::axis::Z<vec4<T>> ) ) :
			Math::normalize( Math::cross( vector, Math::axis::X<vec4<T>> ) );
	}

	/**
	 * @brief Calculates the component of the vector that is orthogonal to the direction
	 * @param vector - non-zero vector
	 * @param direction - direction vector (normalized)
	 * @return vector orthogonal to direction
	*/
	template<typename T>
	[[nodiscard]] constexpr vec4<T> orthogonal( const vec4<T>& vector, const vec4<T>& direction )
	{
		// TODO: This is a 3D vector operation, not a 4D vector
		return vector - direction * dot_3D( vector, direction );
	}

	/**
	 * @brief Linear interpolation between two vectors
	 * @param a - first vector
	 * @param b - second vector
	 * @param t - weight
	 * @return weighted vector
	*/
	template<size_t N, typename T> 
	[[nodiscard]] constexpr vec<N, T> lerp( const vec<N, T>& a, const vec<N, T>& b, const float t )
	{
		vec<N, T> r;
		vf::simd::sub( r, b, a );
		vf::simd::fmadd( r, r, vf::simd::set1<N, T>( t ), a );
		return r;
	}

	/**
	 * @brief Rotates a vector around an axis by an angle
	 * @param position - position vector
	 * @param axis - axis direction (normalized)
	 * @param angle - angle around axis
	 * @return rotated position vector
	*/
	[[nodiscard]] static vec4f rotate( const vec4f& position, const vec3f& axis, const float angle )
	{
		// Based on quaternion implementation, but without using a quaternion
		vec4f r;
		__m128 qxyz = _mm_mul_ps( axis.simd, _mm_set1_ps( sinf( angle * 0.5f ) ) );
		r.simd = _mm_add_ps( position.simd, _mm_mul_ps( _mm_set1_ps( 2.0f ),
			_mm_cross_ps( qxyz, _mm_add_ps( _mm_cross_ps( qxyz, position.simd ), _mm_mul_ps( position.simd, _mm_set1_ps( cosf( angle * 0.5f ) ) ) ) ) ) );
		return r;
	}

	/**
	 * @brief Rotates a vector around an axis by an angle
	 * @param position - position vector
	 * @param rotation - axis angle vector
	 * @return rotated position
	*/
	[[nodiscard]] static vec3f rotate( const vec3f& position, const AxisAngle& rotation )
	{
		float angle = Math::length( rotation );
		if( angle <= Math::EPSILON<float> ) return position;

		vec3f axis = rotation / angle;
		return rotate( position, axis, angle );
	}

	/**
	 * @brief Generates rotation around an axis
	 * @tparam T - rotation type
	 * @param axis - axis (normalized)
	 * @param angle - radian angle around axis
	 * @return rotation around axis
	*/
	template<typename T> [[nodiscard]] static T rotationAround( const vec3f& axis, const float angle )
	{
		return axis * angle;
	}

	/**
	 * @brief Generates rotation between two axes
	 * @tparam T - rotation type
	 * @param from - initial axis (normalized)
	 * @param to - final axis (normalized)
	 * @return rotation between axes
	*/
	template<typename T> [[nodiscard]] static T rotationBetween( const vec3f& from, const vec3f& to )
	{
		float product = dot_3D( from, to );
		if( product > 0.999999f ) return { 0.0f, 0.0f, 0.0f, 0.0f };
		if( product < -0.999999f ) return orthogonal( from ) * Math::PI<float>;
		return normalize( cross( from, to ) ) * acosf( product );
	}

	/**
	 * @brief Spherical interpolation between two direction vectors
	 * @param a - first direction (normalized)
	 * @param b - second direction (normalized)
	 * @param t - weight
	 * @return weighted direction (normalized)
	*/
	[[nodiscard]] static vec3f slerp( const vec3f& a, const vec3f& b, const float t )
	{
		float product = Math::dot_3D( a, b );
		if( product > 1.0f - Math::EPSILON<float> ) return a;

		vec3f normal = product < Math::EPSILON<float> -1.0f ? Math::orthogonal( a ) : Math::normalize( Math::cross( a, b ) );
		return Math::rotate( a, normal, acosf( product ) * t );
	}

	/**
	 * @brief Utilities for performing 4 independent calculations in parallel
	*/
	namespace parallel
	{
		/**
		 * @brief Calculates 4 vector sums simultaneously, returned as a vector
		 * @param a - first vector
		 * @param b - second vector
		 * @param c - third vector
		 * @param d - fourth vector
		 * @return sum of the 4 vectors stored as a single vector
		*/
		[[nodiscard]] constexpr vec4f hsum( const vec4f& a, const vec4f& b, const vec4f& c, const vec4f& d = ZERO<vec4f> )
		{
			vec4f r;
			r.simd = SIMD_4F_SUM4( a.simd, b.simd, c.simd, d.simd );
			return r;
		}

		/**
		 * @brief Calculates 2 vector sums simultaneously, returned as a vector
		 * @param a - first vector
		 * @param b - second vector
		 * @return sum of the 2 vectors stored as a single vector
		*/
		[[nodiscard]] constexpr vec4f hsum( const vec4f& a, const vec4f& b )
		{
			vec4f r;
			r.simd = SIMD_4F_SUM2( a.simd, b.simd );
			return r;
		}

		/**
		 * @brief Calculates 4 dot products simultaneously, returned as a vector
		 * @param a - first vector of first pair
		 * @param b - second vector of first pair
		 * @param c - first vector of second pair
		 * @param d - second vector of second pair
		 * @param e - first vector of third pair
		 * @param f - second vector of third pair
		 * @param g - first vector of fourth pair
		 * @param h - second vector of fourth pair
		 * @return 4 dot products stored as a single vector
		*/
		[[nodiscard]] inline static vec4f dot( const vec4f& a, const vec4f& b, const vec4f& c, const vec4f& d,
			const vec4f& e, const vec4f& f, const vec4f& g, const vec4f& h )
		{
			vec4f r;
			r.simd = SIMD_4F_DOT4( a.simd, b.simd, c.simd, d.simd, e.simd, f.simd, g.simd, h.simd );
			return r;
		}

		/**
		 * @brief Calculates 3 dot products simultaneously, returned as a vector
		 * @param a - first vector of first pair
		 * @param b - second vector of first pair
		 * @param c - first vector of second pair
		 * @param d - second vector of second pair
		 * @param e - first vector of third pair
		 * @param f - second vector of third pair
		 * @return 3 dot products stored as a single vector
		*/
		[[nodiscard]] inline static vec4f dot( const vec4f& a, const vec4f& b, const vec4f& c, const vec4f& d, const vec4f& e, const vec4f& f )
		{
			vec4f r;
			r.simd = SIMD_4F_DOT3( a.simd, b.simd, c.simd, d.simd, e.simd, f.simd );
			return r;
		}

		/**
		 * @brief Calculates 2 dot products simultaneously, returned as a vector
		 * @param a - first vector of first pair
		 * @param b - second vector of first pair
		 * @param c - first vector of second pair
		 * @param d - second vector of second pair
		 * @return 2 dot products stored as a single vector
		*/
		[[nodiscard]] inline static vec4f dot( const vec4f& a, const vec4f& b, const vec4f& c, const vec4f& d )
		{
			vec4f r;
			r.simd = SIMD_4F_DOT2( a.simd, b.simd, c.simd, d.simd );
			return r;
		}

		/**
		 * @brief Calculates 4 vector lengths simultaneously, returned as a vector
		 * @param a - first vector
		 * @param b - second vector
		 * @param c - third vector
		 * @param d - fourth vector
		 * @return 4 vector lengths stored as a single vector
		*/
		[[nodiscard]] static vec4f length( const vec4f& a, const vec4f& b, const vec4f& c, const vec4f& d )
		{
			vec4f r;
			r.simd = _mm_mag_ps( a.simd, b.simd, c.simd, d.simd );
			return r;
		}

		/**
		 * @brief Calculates 3 vector lengths simultaneously, returned as a vector
		 * @param a - first vector
		 * @param b - second vector
		 * @param c - third vector
		 * @return 3 vector lengths stored as a single vector
		*/
		[[nodiscard]] static vec4f length( const vec4f& a, const vec4f& b, const vec4f& c )
		{
			vec4f r;
			r.simd = _mm_mag3_ps( a.simd, b.simd, c.simd );
			return r;
		}

		/**
		 * @brief Calculates 2 vector lengths simultaneously, returned as a vector
		 * @param a - first vector
		 * @param b - second vector
		 * @return 2 vector lengths stored as a single vector
		*/
		[[nodiscard]] static vec4f length( const vec4f& a, const vec4f& b )
		{
			vec4f r;
			r.simd = _mm_mag2_ps( a.simd, b.simd );
			return r;
		}
	};

	/**
	 * @brief Generates a random 2D vector within the range
	 * @param min - minimum vector
	 * @param max - maximum vector
	 * @return random vector
	*/
	template<> [[nodiscard]] static vec2f random<vec2f>( const vec2f& min, const vec2f& max )
	{
		return min + vec2f{ ( (float)rand() / RAND_MAX ), ( (float)rand() / RAND_MAX ) } * ( max - min );
	}

	/**
	 * @brief Generates a random 4D vector within the range
	 * @param min - minimum vector
	 * @param max - maximum vector
	 * @return random vector
	*/
	template<> [[nodiscard]] static vec4f random<vec4f>( const vec4f& min, const vec4f& max )
	{
		vec4f r;
		r.simd = _mm_fmadd_ps( _mm_div_ps( _mm_cvtepi32_ps( _mm_set_epi32( rand(), rand(), rand(), rand() ) ), _mm_set1_ps( RAND_MAX_FLOAT ) ), _mm_sub_ps( max.simd, min.simd ), min.simd );
		return r;
	}

	/**
	 * @brief Generates a random direction vector
	 * @tparam T - type of the direction
	 * @return random direction of specified type
	*/
	template <typename T> static T randomDirection()
	{
		return rand() / RAND_MAX_FLOAT;
	}

	/**
	 * @brief Generates a random 3D rotation
	 * @tparam T - type of the rotation
	 * @return random rotation of specified type
	*/
	template <typename T> static T randomRotation()
	{
		return rand() / RAND_MAX_FLOAT;
	}

	/**
	 * @brief Generates a random 2D directional vector
	 * @return random 2D directional vector
	*/
	template<> [[nodiscard]] static vec2f randomDirection<vec2f>()
	{
		float theta = rand() * RAND_CONVERT_TAU;
		return { cosf( theta ), sinf( theta ) };
	}

	/**
	 * @brief Generates a random 3D directional vector
	 * @return random 3D directional vector
	*/
	template<> [[nodiscard]] static vec3f randomDirection<vec3f>()
	{
		float alpha = rand() * RAND_CONVERT_TAU;
		float beta = rand() * RAND_CONVERT_PI - RAND_CONVERT_HALFPI;
		float cosBeta = cosf( beta );
		return { cosf( alpha ) * cosBeta, sinf( alpha ) * cosBeta, sinf( beta ), 0.0f };
	}

	/**
	 * @brief Generates a random axis angle rotation
	 * @return random axis angle rotation
	*/
	template<> [[nodiscard]] static AxisAngle randomRotation<AxisAngle>()
	{
		return randomDirection<vec3f>() * random<float>( 0.0f, Math::TWO_PI<float> );
	}

	/**
	 * @brief Extends the bounds to include the point
	 * @param bounds - vector bounds
	 * @param point - vector point
	*/
	constexpr void extend( Bounds<vec2f>& bounds, const vec2f& point )
	{
		bounds.min = Math::min( bounds.min, point );
		bounds.max = Math::max( bounds.max, point );
	}

	/**
	 * @brief Extends the bounds to include the point
	 * @param bounds - vector bounds
	 * @param point - vector point
	*/
	constexpr void extend( Bounds<vec2i>& bounds, const vec2i& point )
	{
		bounds.min = Math::min( bounds.min, point );
		bounds.max = Math::max( bounds.max, point );
	}

	/**
	 * @brief Extends the bounds to include the point
	 * @param bounds - vector bounds
	 * @param point - vector point
	*/
	constexpr void extend( Bounds<vec4f>& bounds, const vec4f& point )
	{
		bounds.min = Math::min( bounds.min, point );
		bounds.max = Math::max( bounds.max, point );
	}

	/**
	 * @brief Extends the bounds to include the point
	 * @param bounds - vector bounds
	 * @param point - vector point
	*/
	constexpr void extend( Bounds<vec4i>& bounds, const vec4i& point )
	{
		bounds.min = Math::min( bounds.min, point );
		bounds.max = Math::max( bounds.max, point );
	}
};

#endif
