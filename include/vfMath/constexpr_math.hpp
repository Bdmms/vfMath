#pragma once
#ifndef VF_CONSTEXPR_MATH_HPP
#define VF_CONSTEXPR_MATH_HPP

#include "constants.hpp"

#include <concepts>
#include <type_traits>

namespace vf::math::compile_time
{
	template<typename T> requires std::floating_point<T>
	[[nodiscard]] constexpr T sqrt( T x ) noexcept
	{
		if( x < 0 ) return NAN; // Handle domain error

		T curr = x, prev = Math::ZERO<T>;
		while( curr != prev )
		{
			prev = curr;
			curr = Math::HALF<T> * ( curr + x / curr );
		}
		return curr;
	}
}

#endif