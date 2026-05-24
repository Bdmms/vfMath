#pragma once
#ifndef VF_VECTOR_STORAGE_HPP
#define VF_VECTOR_STORAGE_HPP

#include "primitives.hpp"
#include <immintrin.h>
#include <array>

template<size_t N, typename T>
struct VectorStorage
{
	std::array<T, N> elements;
};

// 2D - Vectors

template<typename T>
struct VectorStorage<2LLU, T>
{
	union
	{
		std::array<T, 2LLU> elements;
		struct { T x, y; };
	};
};

// 3D - Vectors

template<typename T>
struct VectorStorage<3LLU, T>
{
	union
	{
		std::array<T, 3LLU> elements;
		struct { T x, y, z; };
	};
};

template<>
struct VectorStorage<3LLU, i32>
{
	union
	{
		std::array<i32, 3LLU> elements;
		struct { i32 x, y, z; };
		__m128i simd;
	};
};

template<>
struct VectorStorage<3LLU, f32>
{
	union
	{
		std::array<f32, 3LLU> elements;
		struct { f32 x, y, z; };
		__m128 simd;
	};
};

// 4D - Vectors

template<typename T>
struct VectorStorage<4LLU, T>
{
	union
	{
		std::array<T, 4LLU> elements;
		struct { T x, y, z, w; };
	};
};

template<>
struct VectorStorage<4LLU, i32>
{
	union
	{
		std::array<i32, 4LLU> elements;
		struct { i32 x, y, z, w; };
		__m128i simd;
	};
};

template<>
struct VectorStorage<4LLU, u32>
{
	union
	{
		std::array<u32, 4LLU> elements;
		struct { u32 x, y, z, w; };
		__m128i simd;
	};
};

template<>
struct VectorStorage<4LLU, f32>
{
	union
	{
		std::array<f32, 4LLU> elements;
		struct { f32 x, y, z, w; };
		__m128 simd;
	};
};

#endif