#pragma once
#ifndef VF_MATRIX_STORAGE_HPP
#define VF_MATRIX_STORAGE_HPP

#include "vec.hpp"

template<size_t M, size_t N, typename T>
struct MatrixStorage
{
	std::array<vec<M, T>, N> cols;
};

// 2x2 Matrices

template<>
struct MatrixStorage<2, 2, f32>
{
	union
	{
		std::array<vec2f32, 2> cols;
		struct
		{
			vec2f32 x_axis;
			vec2f32 y_axis;
		};
		__m128 simd;
	};
};

#endif