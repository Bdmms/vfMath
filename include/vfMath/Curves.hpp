#pragma once
#ifndef VF_CURVES_HPP
#define VF_CURVES_HPP

#include "VectorMath.hpp"
#include <array>

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
\
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

	constexpr vec3f getDirection() const
	{
		return current;
	}

	vec3f getDirection(const float elapsedTime)
	{
		return current = Math::slerp(current, target, elapsedTime * speed);
	}
};

#endif