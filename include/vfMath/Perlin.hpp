#pragma once
#ifndef VF_PERLIN_HPP
#define VF_PERLIN_HPP

#include "vec4f.hpp"

namespace Perlin
{
	/**
	 * @brief Generates a deterministic random value between 0.0 and 1.0 at the specified point.
	 * @param value - 1D position
	 * @return scalar noise value
	*/
	float perlin1D( float value );

	/**
	 * @brief Generates a deterministic random value between 0.0 and 1.0 at the specified point.
	 * @param point - 2D position
	 * @return scalar noise value
	*/
	float perlin2D( vec2f point );

	/**
	 * @brief Generates a deterministic random value between 0.0 and 1.0 at the specified point.
	 * @param point - 3D position
	 * @return scalar noise value
	*/
	float perlin3D( const vec3f& point );
}

#endif