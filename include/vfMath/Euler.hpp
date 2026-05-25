#pragma once
#ifndef VF_EULER_HPP
#define VF_EULER_HPP

#include "vec.hpp"

#define VECTOR_FORWARD(x) { x }

template<typename T>
struct euler : public VectorStorage<3, T>
{
	using Storage = VectorStorage<3, T>;

	// Default Constuctor
	constexpr euler() : Storage{} {}

	// Initializing List
	constexpr euler( T x, T y, T z ) : Storage{ x, y, z } {}

	// Copy Constructor
	constexpr euler( const euler& copy ) : Storage( copy ) {}

	/**
	 * Creates an euler vector with the value set across each element.
	 */
	constexpr static euler set1( T value )
	{
		euler r;
		vf::simd::set1<3, T>( r, value );
		return r;
	}

	// Container Operations
	constexpr T* data() { return Storage::elements.data(); }
	constexpr const T* data() const { return Storage::elements.data(); }
	constexpr size_t size() { return Storage::elements.size(); }

	// Array Operators
	constexpr const T& operator[]( size_t i ) const { return Storage::elements[i]; }
	constexpr T& operator[]( size_t i ) { return Storage::elements[i]; }

	// Unary Operators
	[[nodiscard]] constexpr euler operator-() const
	{
		euler r;
		vf::simd::negate<3, T>( r, *this );
		return r;
	}

	// Arithmetic Operators
	constexpr euler& operator+=( const euler& rhs ) noexcept { vf::simd::add<3, T>( *this, *this, rhs ); return *this; }
	constexpr euler& operator-=( const euler& rhs ) noexcept { vf::simd::sub<3, T>( *this, *this, rhs ); return *this; }
	constexpr euler& operator*=( const euler& rhs ) noexcept { vf::simd::mul<3, T>( *this, *this, rhs ); return *this; }
	constexpr euler& operator/=( const euler& rhs ) noexcept { vf::simd::div<3, T>( *this, *this, rhs ); return *this; }
	constexpr euler& operator%=( const euler& rhs ) noexcept { vf::simd::mod<3, T>( *this, *this, rhs ); return *this; }

	constexpr euler& operator+=( T rhs ) noexcept { vf::simd::add<3, T>( *this, *this, set1( rhs ) ); return *this; }
	constexpr euler& operator-=( T rhs ) noexcept { vf::simd::sub<3, T>( *this, *this, set1( rhs ) ); return *this; }
	constexpr euler& operator*=( T rhs ) noexcept { vf::simd::mul<3, T>( *this, *this, set1( rhs ) ); return *this; }
	constexpr euler& operator/=( T rhs ) noexcept { vf::simd::div<3, T>( *this, *this, set1( rhs ) ); return *this; }
	constexpr euler& operator%=( T rhs ) noexcept { vf::simd::mod<3, T>( *this, *this, set1( rhs ) ); return *this; }
};

#define VF_EULER euler<T>

#define VF_EULER_OPERATOR_HEAD( Op ) template<typename T> [[nodiscard]] constexpr VF_EULER operator##Op
#define VF_EULER_OPERATOR_EULER_EULER( Op, Func )  VF_EULER_OPERATOR_HEAD( Op )( const VF_EULER& a, const VF_EULER& b )  noexcept { VF_EULER r; Func<3,T>( r, a, b ); return r; }
#define VF_EULER_OPERATOR_EULER_SCALAR( Op, Func ) VF_EULER_OPERATOR_HEAD( Op )( const VF_EULER& a, const VF_SCALAR& b ) noexcept { VF_EULER r; Func<3,T>( r, a, euler<T>::set1( b ) ); return r; }
#define VF_EULER_OPERATOR_SCALAR_EULER( Op, Func ) VF_EULER_OPERATOR_HEAD( Op )( const VF_SCALAR& a, const VF_EULER& b ) noexcept { VF_EULER r; Func<3,T>( r, euler<T>::set1( a ), b ); return r; }

// Non-Member Arithmetic Operators

VF_EULER_OPERATOR_EULER_EULER( +, vf::simd::add );
VF_EULER_OPERATOR_EULER_EULER( -, vf::simd::sub );
VF_EULER_OPERATOR_EULER_EULER( *, vf::simd::mul );
VF_EULER_OPERATOR_EULER_EULER( /, vf::simd::div );
VF_EULER_OPERATOR_EULER_EULER( %, vf::simd::mod );

VF_EULER_OPERATOR_EULER_SCALAR( +, vf::simd::add );
VF_EULER_OPERATOR_EULER_SCALAR( -, vf::simd::sub );
VF_EULER_OPERATOR_EULER_SCALAR( *, vf::simd::mul );
VF_EULER_OPERATOR_EULER_SCALAR( /, vf::simd::div );
VF_EULER_OPERATOR_EULER_SCALAR( %, vf::simd::mod );

VF_EULER_OPERATOR_SCALAR_EULER( +, vf::simd::add );
VF_EULER_OPERATOR_SCALAR_EULER( -, vf::simd::sub );
VF_EULER_OPERATOR_SCALAR_EULER( *, vf::simd::mul );
VF_EULER_OPERATOR_SCALAR_EULER( /, vf::simd::div );
VF_EULER_OPERATOR_SCALAR_EULER( %, vf::simd::mod );

typedef euler<f32> eulerf32;
typedef euler<f64> eulerf64;

typedef eulerf32 eulerf;

#endif