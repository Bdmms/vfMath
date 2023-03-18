#pragma once
#ifndef VF_CURVES_HPP
#define VF_CURVES_HPP

#include "VectorMath.hpp"
#include <array>

constexpr float CURVE_SAMPLE_MIN = 0.0f;
constexpr float CURVE_SAMPLE_MAX = 1.0f - Math::EPSILON<float>;

constexpr mat4x4 BASIS_BEZIER			= { 1.0f, 0.0f, 0.0f, 0.0f, -3.0f, 3.0f, 0.0f, 0.0f, 3.0f, -6.0f, 3.0f,  0.0f, -1.0f, 3.0f, -3.0f, 1.0f };
//constexpr mat4x4 BASIS_CATMULL_ROM	= { 0.0f, 2.0f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f, 2.0f, -5.0f, 4.0f, -1.0f, -1.0f, 3.0f, -3.0f, 1.0f };
constexpr mat4x4 BASIS_CATMULL_ROM		= { 0.0f, 0.5f, 0.0f, 0.0f, -0.5f, 0.0f, 0.5f, 0.0f, 1.0f, -2.5f, 2.0f, -0.5f, -0.5f, 1.5f, -1.5f, 0.5f };
//constexpr mat4x4 BASIS_BSPLINE		= { 1.0f, 4.0f, 1.0f, 0.0f, -3.0f, 0.0f, 3.0f, 0.0f, 3.0f, -6.0f, 3.0f,  0.0f, -1.0f, 3.0f, -3.0f, 1.0f };
constexpr mat4x4 BASIS_BSPLINE			= { 1.0f, 4.0f, 1.0f, 0.0f, -3.0f, 0.0f, 3.0f, 0.0f, 3.0f, -6.0f, 3.0f,  0.0f, -1.0f, 3.0f, -3.0f, 1.0f };

namespace Curves
{
	template <unsigned int size>
	constexpr std::array<float, size> generateBezierCoefficients()
	{
		if constexpr (size == 1) return std::array<float, size>({ 1.0f });
		else
		{
			//     1
			//    1 1
			//   1 2 1
			//  1 3 3 1
			// 1 4 6 4 1
			std::array<float, size - 1> parent = generateBezierCoefficients<size - 1>();
			std::array<float, size> coefficents;

			coefficents[0] = 1.0f;
			for (unsigned int i = 1; i < size - 1; ++i)
			{
				coefficents[i] = parent[i - 1] + parent[i];
			}
			coefficents[size - 1] = 1.0f;
			return coefficents;
		}
	}
}

template <typename T, unsigned int degree>
struct BezierCurve
{
	constexpr static std::array<float, degree> coefficents = Curves::generateBezierCoefficients<degree>();
	T points[degree];

	/**
	 * @brief Samples the curve at the specified input
	 * @param t - curve input
	 * @return point on the curve
	*/
	T sampleAt(const float t) const
	{
		float rt = 1.0f - t;
		T point = points[0] * powf( rt, (float)(degree - 1) );

		float front = 1.0f;
		for (unsigned int i = 1; i < degree; ++i)
		{
			front *= t;
			point += points[i] * ( coefficents[i] * front * powf( rt, (float)( degree - 1 - i ) ) );
		}

		return point;
	}
};

template <const mat4x4 Basis>
struct KnotCurve : public std::vector<vec4f>
{
	/**
	 * @brief Samples the curve at a scalar point
	 * @param t - sample scalar between 0.0 and 1.0
	 * @return sampled vector at scalar point
	*/
	vec4f sampleAt(const float t) const
	{
		if ( size() <= 4 ) return Math::IDENTITY<vec4f>;

		// Convert absolute t weight into relative t weight
		float absolute = Math::clamp( t, CURVE_SAMPLE_MIN, CURVE_SAMPLE_MAX ) * ( size() - 4LLU );
		float weight = absolute - floorf( absolute );

		// Create sample vector
		__m128 sample = { 1.0f, weight, weight * weight, weight * weight * weight };

		// Get knot matrix
		const vec4f* ptr = data() + static_cast<uint32_t>( absolute );
		const vec4f& r0 = ptr[0];
		const vec4f& r1 = ptr[1];
		const vec4f& r2 = ptr[2];
		const vec4f& r3 = ptr[3];

		__m128 c0 = { r0.x, r1.x, r2.x, r3.x };
		__m128 c1 = { r0.y, r1.y, r2.y, r3.y };
		__m128 c2 = { r0.z, r1.z, r2.z, r3.z };
		__m128 c3 = { r0.w, r1.w, r2.w, r3.w };

		// Multiple knot matrix with basis matrix
		c0 = _mm_dot_ps( Basis.simd[0], c0, Basis.simd[1], c0, Basis.simd[2], c0, Basis.simd[3], c0 );
		c1 = _mm_dot_ps( Basis.simd[0], c1, Basis.simd[1], c1, Basis.simd[2], c1, Basis.simd[3], c1 );
		c2 = _mm_dot_ps( Basis.simd[0], c2, Basis.simd[1], c2, Basis.simd[2], c2, Basis.simd[3], c2 );
		c3 = _mm_dot_ps( Basis.simd[0], c3, Basis.simd[1], c3, Basis.simd[2], c3, Basis.simd[3], c3 );

		// Multiple sample vector with matrix
		return { _mm_dot_ps( sample, c0, sample, c1, sample, c2, sample, c3 ) };
	}
};

typedef KnotCurve<BASIS_BEZIER>			BezierKnotCurve;
typedef KnotCurve<BASIS_CATMULL_ROM>	CatmullRomCurve;
typedef KnotCurve<BASIS_BSPLINE>		BSpline;

template <typename T>
class Linear
{
	T current;
	T target;
	T speed;

public:
	constexpr Linear(const T& initValue, const T& target, const T& speed)
		: current(initValue), target(target), speed(speed)
	{

	}

	constexpr void setTarget(const T& newTarget)
	{
		target = newTarget;
	}

	constexpr T getValue() const
	{ 
		return current;
	}

	T getValue(const float elapsedTime)
	{
		T displacement = speed * elapsedTime;
		current += Math::clamp(target - current, -displacement, displacement);
		return current;
	}
};

template <typename T>
struct Smoothed
{
	T current;
	T target;
	float speed;

	constexpr Smoothed(const T& initValue, const T& target, const T& speed)
		: current(initValue), target(target), speed(speed)
	{

	}

	constexpr void setTarget(const T& newTarget)
	{
		target = newTarget;
	}

	constexpr T getValue() const
	{
		return current;
	}

	T getValue(const float elapsedTime)
	{
		return current = Math::lerp( current, target, elapsedTime * speed);
	}
};

struct LinearDirection
{
	vec3f current;
	vec3f target;
	float angularSpeed;

public:
	constexpr LinearDirection(const vec3f& initValue, const vec3f& target, const float speed)
		: current(initValue), target(target), angularSpeed(speed)
	{

	}

	constexpr void setDirection(const vec3f& direction)
	{
		current = direction;
	}

	constexpr void setTarget(const vec3f& direction)
	{
		target = direction;
	}

	constexpr const vec3f& getDirection() const
	{
		return current;
	}

	const vec3f& getDirection(const float elapsedTime)
	{
		float product = Math::dot_3D( current, target );
		vec3f normal;

		if( product > 1.0f - Math::EPSILON<float> )
		{
			return current;
		}
		else if ( product < Math::EPSILON<float> - 1.0f )
		{
			normal = Math::orthogonal(current);
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

	constexpr SmoothDirection(const vec3f& initValue, const vec3f& target, const float speed)
		: current(initValue), target(target), speed(speed)
	{

	}

	constexpr void setDirection(const vec3f& direction)
	{
		current = direction;
	}

	constexpr void setTarget(const vec3f& newTarget)
	{
		target = newTarget;
	}

	constexpr const vec3f& getDirection() const
	{
		return current;
	}

	const vec3f& getDirection(const float elapsedTime)
	{
		return current = Math::slerp( current, target, elapsedTime * speed );
	}
};

#endif