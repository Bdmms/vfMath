#pragma once
#ifndef VF_PERLIN_HPP
#define VF_PERLIN_HPP

#include "vec2f.hpp"

namespace Perlin
{
	/**
	 * @brief Generates a deterministic random value between 0.0 and 1.0 at the specified point.
	 * @param point - 2D position
	 * @return scalar noise value
	*/
	float perlin2D( const vec2f point );
}

#endif