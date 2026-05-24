#pragma once
#ifndef VF_QUATERNION_HPP
#define VF_QUATERNION_HPP

#include "vec.hpp"

namespace vf::simd
{
	namespace compile_time
	{
		template<typename T>
		constexpr void hamilton_product( VectorStorage<4, T>& r, const VectorStorage<4, T>& a, const VectorStorage<4, T>& b )
		{
			r.x =  a.x * b.w + a.y * b.z - a.z * b.y + a.w * b.x;
			r.y = -a.x * b.z + a.y * b.w + a.z * b.x + a.w * b.y;
			r.z =  a.x * b.y - a.y * b.x + a.z * b.w + a.w * b.z;
			r.w = -a.x * b.x - a.y * b.y - a.z * b.z + a.w * b.w;
		}
	}

	template<typename T>
	constexpr void hamilton_product( VectorStorage<4, T>& r, const VectorStorage<4, T>& a, const VectorStorage<4, T>& b )
	{
		compile_time::hamilton_product( r, a, b );
	}

	template<>
	constexpr void hamilton_product( simd4f32& r, const simd4f32& a, const simd4f32& b )
	{
		if( std::is_constant_evaluated() ) return hamilton_product( r, a, b );

		constexpr static __m128 mskx = { 1.0f,  1.0f, -1.0f, 1.0f };
		constexpr static __m128 msky = { -1.0f,  1.0f,  1.0f, 1.0f };
		constexpr static __m128 mskz = { 1.0f, -1.0f,  1.0f, 1.0f };
		constexpr static __m128 mskw = { -1.0f, -1.0f, -1.0f, 1.0f };

		r.simd = _mm_dot_ps(
			_mm_mul_ps( a.simd, mskx ), _mm_permute_ps( b.simd, swizzle::WZYX ),
			_mm_mul_ps( a.simd, msky ), _mm_permute_ps( b.simd, swizzle::ZWXY ),
			_mm_mul_ps( a.simd, mskz ), _mm_permute_ps( b.simd, swizzle::YXWZ ),
			_mm_mul_ps( a.simd, mskw ), _mm_permute_ps( b.simd, swizzle::XYZW )
		);
	}
}

template<typename T>
struct quat : public VectorStorage<4, T>
{
	using Storage = VectorStorage<4, T>;

	// Default Constuctor
	constexpr quat() : Storage{} {}

	// Initializing List
	constexpr quat( T x, T y, T z, T w ) : Storage{ x, y, z, w } {}

	// TODO: Temp Constructor
	constexpr quat( const __m128& copy ) : Storage() { Storage::simd = copy; }

	// Copy Constructor
	constexpr quat( const quat& copy ) : Storage( copy ) {}

	/**
	 * Creates an euler vector with the value set across each element.
	 */
	constexpr static quat set1( T value )
	{
		quat r;
		vf::simd::set1<4, T>( r, value );
		return r;
	}

	// Container Operations
	constexpr T* data() { return Storage::elements.data(); }
	constexpr const T* data() const { return Storage::elements.data(); }
	constexpr size_t size() { return Storage::elements.size(); }

	// Array Operators
	constexpr const T& operator[]( size_t i ) const { return Storage::elements[i]; }
	constexpr T& operator[]( size_t i ) { return Storage::elements[i]; }

	// Arithmetic Operators
	constexpr quat& operator+=( const quat& rhs ) noexcept { vf::simd::add<4, T>( *this, *this, rhs ); return *this; }
	constexpr quat& operator-=( const quat& rhs ) noexcept { vf::simd::sub<4, T>( *this, *this, rhs ); return *this; }
	constexpr quat& operator*=( const quat& rhs ) noexcept { vf::simd::hamilton_product<T>( *this, *this, rhs ); return *this; }

	constexpr quat& operator+=( T rhs ) noexcept { vf::simd::add<4, T>( *this, *this, set1( rhs ) ); return *this; }
	constexpr quat& operator-=( T rhs ) noexcept { vf::simd::sub<4, T>( *this, *this, set1( rhs ) ); return *this; }
	constexpr quat& operator*=( T rhs ) noexcept { vf::simd::mul<4, T>( *this, *this, set1( rhs ) ); return *this; }
	constexpr quat& operator/=( T rhs ) noexcept { vf::simd::div<4, T>( *this, *this, set1( rhs ) ); return *this; }

	// Unary Operators
	[[nodiscard]] constexpr quat operator-() const
	{
		quat r;
		vf::simd::negate<4, T>( r, *this );
		return r;
	}

	/**
	 * @brief Calculates the squared norm of the quaternion
	 * @return squared norm of the quaternion
	*/
	[[nodiscard]] constexpr T norm2() const noexcept
	{
		return vf::simd::dot( *this, *this );
	}

	/**
	 * @brief Calculates the norm of the quaternion
	 * @return norm of the quaternion
	*/
	[[nodiscard]] T norm() const noexcept
	{
		return sqrt( norm2() );
	}

	/**
	 * @brief Calculates the inverse of the quaternion
	 * @return inverse of the quaternion
	*/
	[[nodiscard]] constexpr quat inverse() const noexcept
	{
		quat r{ -1.0f, -1.0f, -1.0f, 1.0f };
		vf::simd::mul( r, r, *this );
		return r;
	}
};

#define VF_QUAT quat<T>

#define VF_QUAT_OPERATOR_HEAD( Op ) template<typename T> [[nodiscard]] constexpr VF_QUAT operator##Op
#define VF_QUAT_OPERATOR_QUAT_QUAT( Op, Func )   VF_QUAT_OPERATOR_HEAD( Op )( const VF_QUAT& a, const VF_QUAT& b )  noexcept { VF_QUAT r; Func( r, a, b ); return r; }
#define VF_QUAT_OPERATOR_QUAT_SCALAR( Op, Func ) VF_QUAT_OPERATOR_HEAD( Op )( const VF_QUAT& a, const VF_SCALAR& b ) noexcept { VF_QUAT r; Func( r, a, quat<T>::set1( b ) ); return r; }
#define VF_QUAT_OPERATOR_SCALAR_QUAT( Op, Func ) VF_QUAT_OPERATOR_HEAD( Op )( const VF_SCALAR& a, const VF_QUAT& b ) noexcept { VF_QUAT r; Func( r, quat<T>::set1( a ), b ); return r; }

// Non-Member Arithmetic Operators

VF_QUAT_OPERATOR_QUAT_QUAT( +, vf::simd::add );
VF_QUAT_OPERATOR_QUAT_QUAT( -, vf::simd::sub );
VF_QUAT_OPERATOR_QUAT_QUAT( *, vf::simd::hamilton_product );

VF_QUAT_OPERATOR_QUAT_SCALAR( +, vf::simd::add );
VF_QUAT_OPERATOR_QUAT_SCALAR( -, vf::simd::sub );
VF_QUAT_OPERATOR_QUAT_SCALAR( *, vf::simd::mul );
VF_QUAT_OPERATOR_QUAT_SCALAR( / , vf::simd::div );

VF_QUAT_OPERATOR_SCALAR_QUAT( +, vf::simd::add );
VF_QUAT_OPERATOR_SCALAR_QUAT( -, vf::simd::sub );
VF_QUAT_OPERATOR_SCALAR_QUAT( *, vf::simd::mul );

typedef quat<f32> quatf32;

#endif