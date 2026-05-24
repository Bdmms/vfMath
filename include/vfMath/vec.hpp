#pragma once
#ifndef VF_VECTOR_HPP
#define VF_VECTOR_HPP

#include "swizzle.hpp"
#include "primitives.hpp"
#include "vec_store.hpp"
#include "vec_ops.hpp"

#include <immintrin.h>
#include <memory>
#include <bit>
#include <type_traits>
#include <utility>

/**
 * @brief Generalized vector type with built-in vector operations.
*/
template<size_t N, typename T> 
struct vec : public VectorStorage<N, T>
{
	using Storage = VectorStorage<N, T>;

	// Default Constuctor
	constexpr vec() : Storage{} {}

	// Initializing List
	template<typename ... Args, typename = std::enable_if_t<( 1LLU < sizeof...( Args ) ) && ( sizeof...( Args ) <= N )>> // ( std::is_constructible_v<T, Args&&> && ... )
	constexpr vec( Args&& ... args ) : Storage{ std::forward<Args>( args )... } {}

	// Copy Constructor
	constexpr vec( const vec<N, T>& copy ) = default;
	constexpr vec& operator=( const vec& copy ) = default;

	/**
	 * Creates a vector with the value set across each element.
	 */
	constexpr static vec set1( T value )
	{
		vec r;
		vf::simd::set1<N, T>( r, value );
		return r;
	}

	// Container Operations
	constexpr T* data() { return Storage::elements.data(); }
	constexpr const T* data() const { return Storage::elements.data(); }
	constexpr auto begin() { return Storage::elements.begin(); }
	constexpr const auto begin() const { return Storage::elements.begin(); }
	constexpr auto end() { return Storage::elements.end(); }
	constexpr const auto end() const { return Storage::elements.end(); }
	constexpr size_t size() { return Storage::elements.size(); }

	// Array Operators
	constexpr const T& operator[]( size_t i ) const { return Storage::elements[i]; }
	constexpr T& operator[]( size_t i ) { return Storage::elements[i]; }

	// Unary Operators
	[[nodiscard]] constexpr vec operator-() const 
	{ 
		vec<N, T> r;
		vf::simd::negate<N, T>( r, *this );
		return r;
	}
	[[nodiscard]] constexpr vec operator~() const 
	{
		vec<N, T> r;
		vf::simd::bitwise_not<N, T>( r, *this );
		return r;
	}

	// Arithmetic Operators
	constexpr vec& operator+=( const vec& rhs ) noexcept { vf::simd::add<N,T>( *this, *this, rhs ); return *this; }
	constexpr vec& operator-=( const vec& rhs ) noexcept { vf::simd::sub<N, T>( *this, *this, rhs ); return *this; }
	constexpr vec& operator*=( const vec& rhs ) noexcept { vf::simd::mul<N, T>( *this, *this, rhs ); return *this; }
	constexpr vec& operator/=( const vec& rhs ) noexcept { vf::simd::div<N, T>( *this, *this, rhs ); return *this; }
	constexpr vec& operator%=( const vec& rhs ) noexcept { vf::simd::mod<N, T>( *this, *this, rhs ); return *this; }

	constexpr vec& operator+=( T rhs ) noexcept { vf::simd::add<N, T>( *this, *this, set1( rhs ) ); return *this; }
	constexpr vec& operator-=( T rhs ) noexcept { vf::simd::sub<N, T>( *this, *this, set1( rhs ) ); return *this; }
	constexpr vec& operator*=( T rhs ) noexcept { vf::simd::mul<N, T>( *this, *this, set1( rhs ) ); return *this; }
	constexpr vec& operator/=( T rhs ) noexcept { vf::simd::div<N, T>( *this, *this, set1( rhs ) ); return *this; }
	constexpr vec& operator%=( T rhs ) noexcept { vf::simd::mod<N, T>( *this, *this, set1( rhs ) ); return *this; }

	// Logical Operators
	constexpr vec& operator&=( const vec& rhs ) noexcept { vf::simd::bitwise_and<N, T>( *this, *this, rhs ); return *this; }
	constexpr vec& operator|=( const vec& rhs ) noexcept { vf::simd::bitwise_or<N, T>(  *this, *this, rhs ); return *this; }
	constexpr vec& operator^=( const vec& rhs ) noexcept { vf::simd::bitwise_xor<N, T>( *this, *this, rhs ); return *this; }

	constexpr vec& operator&=( T rhs ) noexcept { vf::simd::bitwise_and<N, T>( *this, *this, set1( rhs ) ); return *this; }
	constexpr vec& operator|=( T rhs ) noexcept { vf::simd::bitwise_or<N, T>(  *this, *this, set1( rhs ) ); return *this; }
	constexpr vec& operator^=( T rhs ) noexcept { vf::simd::bitwise_xor<N, T>( *this, *this, set1( rhs ) ); return *this; }

	// Conversions
	template<typename DstType>
	[[nodiscard]] constexpr explicit operator vec<N, DstType>() const
	{
		vec<N, DstType> r;
		for( size_t i = 0LLU; i < N; ++i )
		{
			r.elements[i] = static_cast<DstType>( Storage::elements[i] );
		}
		return r;
	}
};

#define VF_SCALAR T
#define VF_VECTOR vec<N, T>

#define VF_VECTOR_OPERATOR_HEAD( Op ) template<size_t N, typename T> [[nodiscard]] constexpr VF_VECTOR operator##Op
#define VF_VECTOR_OPERATOR_VECTOR_VECTOR( Op, Func ) VF_VECTOR_OPERATOR_HEAD( Op )( const VF_VECTOR& a, const VF_VECTOR& b ) noexcept { VF_VECTOR r; Func<N,T>( r, a, b ); return r; }
#define VF_VECTOR_OPERATOR_VECTOR_SCALAR( Op, Func ) VF_VECTOR_OPERATOR_HEAD( Op )( const VF_VECTOR& a, const VF_SCALAR& b ) noexcept { VF_VECTOR r; Func<N,T>( r, a, vec<N, T>::set1( b ) ); return r; }
#define VF_VECTOR_OPERATOR_SCALAR_VECTOR( Op, Func ) VF_VECTOR_OPERATOR_HEAD( Op )( const VF_SCALAR& a, const VF_VECTOR& b ) noexcept { VF_VECTOR r; Func<N,T>( r, vec<N, T>::set1( a ), b ); return r; }

// Non-Member Arithmetic Operators

VF_VECTOR_OPERATOR_VECTOR_VECTOR( +, vf::simd::add );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( -, vf::simd::sub );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( *, vf::simd::mul );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( /, vf::simd::div );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( %, vf::simd::mod );

VF_VECTOR_OPERATOR_VECTOR_SCALAR( +, vf::simd::add );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( -, vf::simd::sub );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( *, vf::simd::mul );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( /, vf::simd::div );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( %, vf::simd::mod );

VF_VECTOR_OPERATOR_SCALAR_VECTOR( +, vf::simd::add );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( -, vf::simd::sub );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( *, vf::simd::mul );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( /, vf::simd::div );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( %, vf::simd::mod );

// Non-Member Logical Operators

VF_VECTOR_OPERATOR_VECTOR_VECTOR( &, vf::simd::bitwise_and );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( |, vf::simd::bitwise_or );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( ^, vf::simd::bitwise_xor );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( &&, vf::simd::bitwise_and );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( || , vf::simd::bitwise_or );

VF_VECTOR_OPERATOR_VECTOR_SCALAR( &, vf::simd::bitwise_and );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( |, vf::simd::bitwise_or );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( ^, vf::simd::bitwise_xor );

VF_VECTOR_OPERATOR_SCALAR_VECTOR( &, vf::simd::bitwise_and );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( |, vf::simd::bitwise_or );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( ^, vf::simd::bitwise_xor );

// Non-Member Comparison Operators
#define VF_OPERATOR_LE <=
#define VF_OPERATOR_GE >=
#define VF_OPERATOR_LT <
#define VF_OPERATOR_GT >
#define VF_OPERATOR_EQ ==

VF_VECTOR_OPERATOR_VECTOR_VECTOR( VF_OPERATOR_LE, vf::simd::cmple );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( VF_OPERATOR_GE, vf::simd::cmpge );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( VF_OPERATOR_LT, vf::simd::cmplt );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( VF_OPERATOR_GT, vf::simd::cmpgt );
VF_VECTOR_OPERATOR_VECTOR_VECTOR( VF_OPERATOR_EQ, vf::simd::cmpeq );

VF_VECTOR_OPERATOR_VECTOR_SCALAR( VF_OPERATOR_LE, vf::simd::cmple );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( VF_OPERATOR_GE, vf::simd::cmpge );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( VF_OPERATOR_LT, vf::simd::cmplt );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( VF_OPERATOR_GT, vf::simd::cmpgt );
VF_VECTOR_OPERATOR_VECTOR_SCALAR( VF_OPERATOR_EQ, vf::simd::cmpeq );

VF_VECTOR_OPERATOR_SCALAR_VECTOR( VF_OPERATOR_LE, vf::simd::cmple );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( VF_OPERATOR_GE, vf::simd::cmpge );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( VF_OPERATOR_LT, vf::simd::cmplt );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( VF_OPERATOR_GT, vf::simd::cmpgt );
VF_VECTOR_OPERATOR_SCALAR_VECTOR( VF_OPERATOR_EQ, vf::simd::cmpeq );

// Explcit Vector Typedefs

template <typename T>
using vec2 = vec<2, T>;
template <typename T>
using vec3 = vec<3, T>;
template <typename T>
using vec4 = vec<4, T>;

typedef vec2<f32> vec2f;
typedef vec2<i32> vec2i;
typedef vec2<u32> vec2u;

typedef vec2<f32> vec2f32;
typedef vec2<f64> vec2f64;
typedef vec2<i8>  vec2i8;
typedef vec2<i16> vec2i16;
typedef vec2<i32> vec2i32;
typedef vec2<i64> vec2i64;
typedef vec2<u8>  vec2u8;
typedef vec2<u16> vec2u16;
typedef vec2<u32> vec2u32;
typedef vec2<u64> vec2u64;

typedef vec4<f32> vec4f;
typedef vec4<i32> vec4i;

typedef vec4<f32> vec4f32;
typedef vec4<f64> vec4f64;
typedef vec4<i8>  vec4i8;
typedef vec4<i16> vec4i16;
typedef vec4<i32> vec4i32;
typedef vec4<i64> vec4i64;

#endif