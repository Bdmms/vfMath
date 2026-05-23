#pragma once
#ifndef VF_MATRIX_HPP
#define VF_MATRIX_HPP

#include "mat_ops.hpp"

template<size_t M, size_t N, typename T>
struct mat : public MatrixStorage<M, N, T>
{
	using Storage = MatrixStorage<M, N, T>;

	// Default Constuctor
	constexpr mat() : Storage{} {}

	// Initializing List
	template<typename ... Args, typename = std::enable_if_t<( sizeof...( Args ) == M * N || sizeof...( Args ) == N ) && 
		( ( std::is_constructible_v<T, Args&&> || std::is_constructible_v<VectorStorage<M, T>, Args&&> ) && ... )>>
	constexpr mat( Args&& ... args ) : Storage{ std::forward<Args>( args )... } {}

	// Copy Constructor
	constexpr mat( const mat<M, N, T>& copy ) = default;
	constexpr mat& operator=( const mat& copy ) = default;

	// Container Operations
	constexpr T* data() { return Storage::elements.data(); }
	constexpr const T* data() const { return Storage::elements.data(); }
	constexpr size_t size() { return Storage::elements.size(); }

	// Array Operators
	constexpr const T& operator[]( size_t r, size_t c ) const { return Storage::cols[c].elements[r]; }
	constexpr T& operator[]( size_t r, size_t c ) { return Storage::cols[c].elements[r]; }

	// Arithmetic Operators
	constexpr mat& operator+=( const mat& rhs ) noexcept { vf::simd::add<M, N, T>( *this, *this, rhs ); return *this; }
	constexpr mat& operator-=( const mat& rhs ) noexcept { vf::simd::sub<M, N, T>( *this, *this, rhs ); return *this; }

	//constexpr mat& operator+=( T rhs ) noexcept { vf::simd::add<N, T>( *this, *this, rhs ); return *this; }
	//constexpr mat& operator-=( T rhs ) noexcept { vf::simd::sub<N, T>( *this, *this, rhs ); return *this; }
	constexpr mat& operator*=( T rhs ) noexcept { vf::simd::mul<M, N, T>( *this, *this, rhs ); return *this; }
	constexpr mat& operator/=( T rhs ) noexcept { vf::simd::div<M, N, T>( *this, *this, rhs ); return *this; }
};

template<size_t M, size_t N, size_t N2, typename T>
[[nodiscard]] constexpr mat<M, N, T> operator+( const mat<M, N, T>& a, const mat<M, N, T>& b ) noexcept
{
	mat<M, N, T> result;
	vf::simd::add( result, a, b );
	return result;
}

template<size_t M, size_t N, size_t N2, typename T>
[[nodiscard]] constexpr mat<M, N, T> operator-( const mat<M, N, T>& a, const mat<M, N, T>& b ) noexcept
{
	mat<M, N, T> result;
	vf::simd::sub( result, a, b );
	return result;
}

template<size_t M, size_t N, typename T>
[[nodiscard]] constexpr mat<M, N, T> operator*( const mat<M, N, T>& a, T b ) noexcept
{
	mat<M, N, T> result;
	vf::simd::mul( result, a, b );
	return result;
}

template<size_t M, size_t N, typename T>
[[nodiscard]] constexpr mat<M, N, T> operator/( const mat<M, N, T>& a, T b ) noexcept
{
	mat<M, N, T> result;
	vf::simd::div( result, a, b );
	return result;
}


template<size_t N0, size_t N1, size_t N2, typename T>
[[nodiscard]] constexpr mat<N0, N2, T> operator*( const mat<N0, N1, T>& a, const mat<N1, N2, T>& b ) noexcept
{
	mat<N0, N2, T> result;
	vf::simd::mul( result, a, b );
	return result;
}

template<size_t M, size_t N, typename T>
[[nodiscard]] constexpr vec<M, T> operator*( const mat<M, N, T>& a, const vec<M, T>& b ) noexcept
{
	vec<M, T> result;
	vf::simd::mul( result, a, b );
	return result;
}

template<size_t M, size_t N, typename T>
[[nodiscard]] constexpr vec<N, T> operator*( const vec<N, T>& a, const mat<M, N, T>& b ) noexcept
{
	vec<N, T> result;
	vf::simd::mul( result, a, b );
	return result;
}


template <typename T>
using mat2 = mat<2, 2, T>;
template <typename T>
using mat3 = mat<3, 3, T>;
template <typename T>
using mat4 = mat<4, 4, T>;

typedef mat2<f32> mat2f;
typedef mat2<i32> mat2i;
typedef mat2<u32> mat2u;

typedef mat2<f32> mat2f32;
typedef mat2<i32> mat2i32;
typedef mat2<u32> mat2u32;

//typedef mat4<f32> mat4f;
typedef mat4<i32> mat4i;
typedef mat4<u32> mat4u;

typedef mat4<f32> mat4f32;
typedef mat4<i32> mat4i32;
typedef mat4<u32> mat4u32;

// TODO:
typedef mat2f mat2x2;

#endif