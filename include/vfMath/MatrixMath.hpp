#pragma once
#ifndef VF_MATRIX_MATH_HPP
#define VF_MATRIX_MATH_HPP

#include "QuatMath.hpp"
#include "mat4x4.hpp"
#include "mat2x2.hpp"

/**
 * @brief Utilities for matrices
*/
namespace Math
{
	template<> inline constexpr static mat2x2 ZERO<mat2x2> = { 0.f, 0.f, 0.f, 0.f };
	template<> inline constexpr static mat4x4 ZERO<mat4x4> = { 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f };

	template<> inline constexpr static mat2x2 ONES<mat2x2> = { 1.f, 1.f, 1.f, 1.f };
	template<> inline constexpr static mat4x4 ONES<mat4x4> = { 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f, 1.f };

	template<> inline constexpr static mat2x2 IDENTITY<mat2x2> = { 1.f, 0.f, 0.f, 1.f };
	template<> inline constexpr static mat4x4 IDENTITY<mat4x4> = { 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f };

	/**
	 * @brief Truncates a 4x4 matrix into a 3x3 matrix
	 * @param matrix - source matrix
	*/
	constexpr mat4x4 cast_mat3x3( const mat4x4& matrix )
	{
		return { matrix.x_axis, matrix.y_axis, matrix.z_axis, ZERO<vec4f> };
	}

	/**
	 * @brief Truncates a 4x4 matrix into a 3x3 matrix
	 * @param dst - destination matrix
	 * @param src - source matrix
	*/
	constexpr void copy_mat3x3( mat4x4& dst, const mat4x4& src )
	{
		std::copy( src.m, src.m + 12, dst.m );
	}

	/* TODO: Test if it works
	static void inverse3x3(mat3x3& dst, const mat3x3& mat)
	{
		dst.simd[0] = _mm_sub_ps(_mm_mul_ps(_mm_permute_ps(mat.simd[1], swizzle::YXX), _mm_permute_ps(mat.simd[2], swizzle::ZZY)),
			_mm_mul_ps(_mm_permute_ps(mat.simd[1], swizzle::ZZY), _mm_permute_ps(mat.simd[2], swizzle::YXX)));
		dst.simd[1] = _mm_sub_ps(_mm_mul_ps(_mm_permute_ps(mat.simd[0], swizzle::YXX), _mm_permute_ps(mat.simd[2], swizzle::ZZY)),
			_mm_mul_ps(_mm_permute_ps(mat.simd[0], swizzle::ZZY), _mm_permute_ps(mat.simd[2], swizzle::YXX)));
		dst.simd[2] = _mm_sub_ps(_mm_mul_ps(_mm_permute_ps(mat.simd[0], swizzle::YXX), _mm_permute_ps(mat.simd[1], swizzle::ZZY)),
			_mm_mul_ps(_mm_permute_ps(mat.simd[0], swizzle::ZZY), _mm_permute_ps(mat.simd[1], swizzle::YXX)));

		constexpr static __m128 m0 = { 1.0f, -1.0f, 1.0f, 0.0f };
		constexpr static __m128 m1 = { -1.0f, 1.0f, -1.0f, 0.0f };
		__m128 det = _mm_set1_ps(Math::dot(mat.col[0], dst.col[0]));
		dst.simd[0] = _mm_div_ps(_mm_mul_ps(dst.simd[0], m0), det);
		dst.simd[1] = _mm_div_ps(_mm_mul_ps(dst.simd[1], m1), det);
		dst.simd[2] = _mm_div_ps(_mm_mul_ps(dst.simd[2], m0), det);
	}*/

	/**
	 * @brief Creates a copy of the matrix translated
	 * @param matrix - source transform
	 * @param translation - displacement vector
	 * @return translated transform
	*/
	static mat4x4 translate( const mat4x4& matrix, const vec3f& translation )
	{
		mat4x4 copy = matrix;
		copy.origin += translation;
		return copy;
	}

	/**
	 * @brief Creates a copy of the matrix scaled around its origin
	 * @param matrix - source transform
	 * @param scale - scale vector
	 * @return scaled transform
	*/
	static mat2x2 scale( const mat2x2& matrix, const vec2f scale )
	{
		mat2x2 copy = matrix;
		copy.x_axis *= scale.x;
		copy.y_axis *= scale.y;
		return copy;
	}

	/**
	 * @brief Creates a copy of the matrix scaled around its origin
	 * @param matrix - source transform
	 * @param scale - scale vector
	 * @return scaled transform
	*/
	static mat4x4 scale( const mat4x4& matrix, const vec3f& scale )
	{
		mat4x4 copy = matrix;
		copy.x_axis *= scale.x;
		copy.y_axis *= scale.y;
		copy.z_axis *= scale.z;
		return copy;
	}

	/**
	 * @brief Sets the translation of the matrix (Overwrites the last column)
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	*/
	constexpr void setTranslation( mat4x4& matrix, const vec3f& translation )
	{
		matrix.m[12] = translation.x;
		matrix.m[13] = translation.y;
		matrix.m[14] = translation.z;
	}

	/**
	 * @brief Sets the rotation of the matrix
	 * @param matrix - 2D transform matrix
	 * @param rotation - rotation angle
	*/
	static void setRotation( mat2x2& matrix, float rotation )
	{
		matrix.m[0] = cosf( rotation );
		matrix.m[1] = sinf( rotation );
		matrix.m[2] = -matrix.m[1];
		matrix.m[3] = matrix.m[0];
	}

	/**
	 * @brief Sets the rotation of the matrix (Overwrites the 3x3 matrix)
	 * @param matrix - transform matrix
	 * @param rotation - euler rotation vector
	*/
	static void setRotation( mat4x4& matrix, const euler& rotation )
	{
		vec3f s = Math::sin( rotation );
		vec3f c = Math::cos( rotation );
		matrix.m[0] = c.z * c.y;
		matrix.m[1] = s.z * c.y;
		matrix.m[2] = -s.y;

		matrix.m[4] = c.z * s.y * s.x - s.z * c.x;
		matrix.m[5] = s.z * s.y * s.x + c.z * c.x;
		matrix.m[6] = c.y * s.x;

		matrix.m[8] = c.z * s.y * c.x + s.z * s.x;
		matrix.m[9] = s.z * s.y * c.x - c.z * s.x;
		matrix.m[10] = c.y * c.x;
	}

	/**
	 * @brief Sets the rotation of the matrix (Overwrites the 3x3 matrix)
	 * @param matrix - transform matrix
	 * @param rotation - quaternion vector
	*/
	static void setRotation( mat4x4& matrix, const quat& q )
	{
		matrix.m[0] = 1.0f - 2.0f * ( q.y * q.y + q.z * q.z );
		matrix.m[1] = 2.0f * ( q.x * q.y + q.w * q.z );
		matrix.m[2] = 2.0f * ( q.x * q.z - q.w * q.y );

		matrix.m[4] = 2.0f * ( q.x * q.y - q.w * q.z );
		matrix.m[5] = 1.0f - 2.0f * ( q.x * q.x + q.z * q.z );
		matrix.m[6] = 2.0f * ( q.y * q.z + q.w * q.x );

		matrix.m[8] = 2.0f * ( q.x * q.z + q.w * q.y );
		matrix.m[9] = 2.0f * ( q.y * q.z - q.w * q.x );
		matrix.m[10] = 1.0f - 2.0f * ( q.x * q.x + q.y * q.y );
	}

	/**
	 * @brief Sets the inverse rotation of the matrix (Overwrites the 3x3 matrix)
	 * @param matrix - transform matrix
	 * @param rotation - euler rotation vector
	*/
	static void setRotationInverse( mat4x4& matrix, const euler& rotation )
	{
		vec3f s = Math::sin( rotation );
		vec3f c = Math::cos( rotation );
		matrix.m[0] = c.z * c.y;
		matrix.m[4] = s.z * c.y;
		matrix.m[8] = -s.y;

		matrix.m[1] = c.z * s.y * s.x - s.z * c.x;
		matrix.m[5] = s.z * s.y * s.x + c.z * c.x;
		matrix.m[9] = c.y * s.x;

		matrix.m[2] = c.z * s.y * c.x + s.z * s.x;
		matrix.m[6] = s.z * s.y * c.x - c.z * s.x;
		matrix.m[10] = c.y * c.x;
	}

	/**
	 * @brief Sets the inverse rotation of the matrix (Overwrites the 3x3 matrix)
	 * @param matrix - transform matrix
	 * @param rotation - quaternion vector
	*/
	constexpr void setRotationInverse( mat4x4& matrix, const quat& q )
	{
		matrix.m[0] = 1.0f - 2.0f * ( q.y * q.y + q.z * q.z );
		matrix.m[4] = 2.0f * ( q.x * q.y + q.w * q.z );
		matrix.m[8] = 2.0f * ( q.x * q.z - q.w * q.y );

		matrix.m[1] = 2.0f * ( q.x * q.y - q.w * q.z );
		matrix.m[5] = 1.0f - 2.0f * ( q.x * q.x + q.z * q.z );
		matrix.m[9] = 2.0f * ( q.y * q.z + q.w * q.x );

		matrix.m[2] = 2.0f * ( q.x * q.z + q.w * q.y );
		matrix.m[6] = 2.0f * ( q.y * q.z - q.w * q.x );
		matrix.m[10] = 1.0f - 2.0f * ( q.x * q.x + q.y * q.y );
	}

	/*
	 * @brief Sets the 2D transform of the 2x2 matrix
	 * @param rotation - rotation angle
	 * @param scale - scaling vector
	*/
	static void setTransform( mat2x2& matrix, const float rotation, const vec2f scale )
	{
		setRotation( matrix, rotation );
		matrix.m[0] *= scale.x;
		matrix.m[1] *= scale.x;
		matrix.m[2] *= scale.y;
		matrix.m[3] *= scale.y;
	}

	/**
	 * @brief Sets the transform of the matrix
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param rotation - rotation vector
	 * @param scale - scaling vector
	*/
	static void setTransform( mat4x4& matrix, const vec3f translation, const euler& rotation, const vec3f& scale )
	{
		setRotation( matrix, rotation );
		matrix.col[0] *= scale.x;
		matrix.col[1] *= scale.y;
		matrix.col[2] *= scale.z;
		setTranslation( matrix, translation );
	}

	/**
	 * @brief Sets the transform of the matrix
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param q - quaternion rotation
	 * @param scale - scaling vector
	*/
	static void setTransform( mat4x4& matrix, const vec3f translation, const quat& q, const vec3f& scale )
	{
		setRotation( matrix, q );
		matrix.col[0] *= scale.x;
		matrix.col[1] *= scale.y;
		matrix.col[2] *= scale.z;
		setTranslation( matrix, translation );
	}

	/**
	 * @brief Sets the inverse transform of the matrix
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param rotation - rotation vector
	 * @param scale - scaling vector
	*/
	static void setTransformInverse( mat4x4& matrix, const vec3f translation, const euler& rotation, const vec3f& scale )
	{
		setRotationInverse( matrix, rotation );
		matrix.col[0] /= scale;
		matrix.col[1] /= scale;
		matrix.col[2] /= scale;
		matrix.m[12] = -dot( matrix.col[0], translation );
		matrix.m[13] = -dot( matrix.col[1], translation );
		matrix.m[14] = -dot( matrix.col[2], translation );
	}

	/**
	 * @brief Sets the inverse transform of the matrix
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param q - quaternion rotation
	 * @param scale - scaling vector
	*/
	static void setTransformInverse( mat4x4& matrix, const vec3f& translation, const quat& q, const vec3f& scale )
	{
		setRotationInverse( matrix, q );
		matrix.col[0] /= scale;
		matrix.col[1] /= scale;
		matrix.col[2] /= scale;
		matrix.m[12] = -dot( matrix.col[0], translation );
		matrix.m[13] = -dot( matrix.col[1], translation );
		matrix.m[14] = -dot( matrix.col[2], translation );
	}

	/**
	 * @brief Converts a rotation matrix into a quaternion.
	 * @param rotation - normalized rotation matrix
	 * @return quaternion rotation
	*/
	static quat toQuat( const mat4x4& rotation )
	{
		// TODO: This may flip signs
		float a = rotation[0] - rotation[5] - rotation[10];
		float b = rotation[5] - rotation[0] - rotation[10];
		float c = rotation[10] - rotation[0] - rotation[5];
		float d = rotation[0] + rotation[5] + rotation[10];
		quat q;
		float t;
		float fac;

		if( c > d )
		{
			t = sqrtf( c + 1.0f ) * 0.5f;
			fac = 0.25f / t;
			q = { ( rotation[8] + rotation[2] ) * fac, ( rotation[6] + rotation[9] ) * fac, t, ( rotation[1] - rotation[4] ) * fac };
		}
		else if( b > d )
		{
			t = sqrtf( b + 1.0f ) * 0.5f;
			fac = 0.25f / t;
			q = { ( rotation[1] + rotation[4] ) * fac, t, ( rotation[6] + rotation[9] ) * fac, ( rotation[8] - rotation[2] ) * fac };
		}
		else if( a > d )
		{
			t = sqrtf( a + 1.0f ) * 0.5f;
			fac = 0.25f / t;
			q = { t, ( rotation[1] + rotation[4] ) * fac, ( rotation[8] + rotation[2] ) * fac, ( rotation[6] - rotation[9] ) * fac };
		}
		else
		{
			t = sqrtf( d + 1.0f ) * 0.5f;
			fac = 0.25f / t;
			q = { ( rotation[6] - rotation[9] ) * fac, ( rotation[8] - rotation[2] ) * fac, ( rotation[1] - rotation[4] ) * fac, t };
		}

		return q;
	}

	/**
	 * @brief Decomposes an affine transform into its components
	 * @param matrix - affine transform matrix
	 * @param position - translation component
	 * @param rotation - rotation component
	 * @param scale - scale component
	*/
	static void decompose( const mat4x4& matrix, vec4f& position, quat& rotation, vec3f& scale )
	{
		scale = Math::parallel::length( matrix.x_axis, matrix.y_axis, matrix.z_axis );
		rotation = toQuat( { matrix.x_axis / scale.x, matrix.y_axis / scale.y, matrix.z_axis / scale.z, Math::axis::W<vec4f> } );
		position = matrix.origin;
	}

	/**
	 * @brief Generates a view matrix
	 * @param mat - view matrix
	 * @param eye - view position vector
	 * @param center - view target vector
	 * @param up - view up vector
	*/
	static void lookAt( mat4x4& mat, const vec4f& eye, const vec4f& center, const vec3f& up )
	{
		vec3f f = Math::normalize( eye - center );
		vec3f s = Math::normalize( Math::cross( up, f ) );
		vec3f u = Math::cross( f, s );
		mat.simd[0] = _mm_transpose_ps( s.simd, u.simd, f.simd, SIMD_4f_ZERO, swizzle::XXXX );
		mat.simd[1] = _mm_transpose_ps( s.simd, u.simd, f.simd, SIMD_4f_ZERO, swizzle::YYYY );
		mat.simd[2] = _mm_transpose_ps( s.simd, u.simd, f.simd, SIMD_4f_ZERO, swizzle::ZZZZ );
		mat.simd[3] = _mm_mul_ps( _mm_dot3v_ps( s.simd, eye.simd, u.simd, eye.simd, f.simd, eye.simd, SIMD_4f_W ), _mm_xyzw_ps( -1.0f, -1.0f, -1.0f, 1.0f ) );
	};

	/**
	 * @brief Generates a rotation matrix that rotates the x-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @param trackAxis - axis to rotate around (non-zero)
	 * @param position - translation component (optional)
	 * @return rotation matrix
	*/
	[[nodiscard]] static mat4x4 lookToX( const vec3f& forward, const vec3f& trackAxis, const vec4f& position = Math::axis::W<vec4f> )
	{
		vec4f z_axis = Math::normalize( Math::cross( forward, trackAxis ) );
		vec4f y_axis = Math::cross( z_axis, forward );
		return mat4x4{ forward, y_axis, z_axis, position };
	}

	/**
	 * @brief Generates a rotation matrix that rotates the x-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @return rotation matrix
	*/
	[[nodiscard]] static mat4x4 lookToX( const vec3f& forward )
	{
		return lookToX( forward, Math::cross( Math::axis::X<vec4f>, forward ) );
	}

	/**
	 * @brief Generates a rotation matrix that rotates the y-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @param trackAxis - axis to rotate around
	 * @param position - translation component (optional)
	 * @return rotation matrix
	*/
	[[nodiscard]] static mat4x4 lookToY( const vec3f& forward, const vec3f& trackAxis, const vec4f& position = Math::axis::W<vec4f> )
	{
		vec4f z_axis = Math::normalize( Math::cross( trackAxis, forward ) );
		vec4f x_axis = Math::cross( forward, z_axis );
		return mat4x4{ x_axis, forward, z_axis, position };
	}

	/**
	 * @brief Generates a rotation matrix that rotates the y-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @return rotation matrix
	*/
	[[nodiscard]] static mat4x4 lookToY( const vec3f& forward )
	{
		return lookToY( forward, Math::cross( Math::axis::Y<vec4f>, forward ) );
	}

	/**
	 * @brief Generates a rotation matrix that rotates the z-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @param trackAxis - axis to rotate around
	 * @param position - translation component (optional)
	 * @return rotation matrix
	*/
	[[nodiscard]] static mat4x4 lookToZ( const vec3f& forward, const vec3f& trackAxis, const vec4f& position = Math::axis::W<vec4f> )
	{
		vec4f x_axis = Math::normalize( Math::cross( trackAxis, forward ) );
		vec4f y_axis = Math::cross( forward, x_axis );
		return mat4x4{ x_axis, y_axis, forward, position };
	}

	/**
	 * @brief Generates a rotation matrix that rotates the z-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @return rotation matrix
	*/
	[[nodiscard]] static mat4x4 lookToZ( const vec3f& forward )
	{
		return lookToX( forward, Math::cross( Math::axis::Z<vec4f>, forward ) );
	}

	/**
	 * @brief Generates a rotation matrix that rotates around an axis
	 * @param axis - axis of the rotation (normalized)
	 * @param angle - angle of the rotation
	 * @return rotation matrix
	*/
	template<> [[nodiscard]] static mat4x4 rotationAround( const vec3f& axis, const float angle )
	{
		float sin = sinf( angle );
		float cos = cosf( angle );
		vec3f tpose = ( 1.0f - cos ) * axis;

		return {
			_mm_fmadd_ps( axis.simd, _mm_set1_ps( tpose.x ), _mm_xyzw_ps( cos, sin * axis.z, -sin * axis.y, 0.0f ) ),
			_mm_fmadd_ps( axis.simd, _mm_set1_ps( tpose.y ), _mm_xyzw_ps( -sin * axis.z, cos, sin * axis.x, 0.0f ) ),
			_mm_fmadd_ps( axis.simd, _mm_set1_ps( tpose.z ), _mm_xyzw_ps( sin * axis.y, -sin * axis.x, cos, 0.0f ) ),
			SIMD_4f_W
		};
	}

	/**
	 * @brief Generates a rotation matrix that rotates between two vectors
	 * @param from - initial vector (normalized)
	 * @param to - final vector (normalized)
	 * @return rotation matrix
	*/
	template<> [[nodiscard]] static mat4x4 rotationBetween( const vec3f& from, const vec3f& to )
	{
		float product = dot_3D( from, to );
		if( product > 0.999999f ) return Math::IDENTITY<mat4x4>;
		if( product < -0.999999f ) return rotationAround<mat4x4>( orthogonal( from ), PI<float> );

		vec3f axis = Math::normalize( Math::cross( from, to ) );
		float cos = Math::dot( from, to );
		float sin = sqrtf( 1.0f - cos * cos );
		vec3f tpose = ( 1.0f - cos ) * axis;

		return {
			_mm_fmadd_ps( axis.simd, _mm_set1_ps( tpose.x ), _mm_xyzw_ps( cos, sin * axis.z, -sin * axis.y, 0.0f ) ),
			_mm_fmadd_ps( axis.simd, _mm_set1_ps( tpose.y ), _mm_xyzw_ps( -sin * axis.z, cos, sin * axis.x, 0.0f ) ),
			_mm_fmadd_ps( axis.simd, _mm_set1_ps( tpose.z ), _mm_xyzw_ps( sin * axis.y, -sin * axis.x, cos, 0.0f ) ),
			SIMD_4f_W
		};
	}

	/**
	 * @brief Generates an orthographic matrix
	 * @param mat - orthographic matrix
	 * @param left - left bounds
	 * @param right - right bounds
	 * @param bottom - bottom bounds
	 * @param top - top bounds
	 * @param zNear - boundary of closest z
	 * @param zFar - boundary of farthest z
	*/
	static void setOrthographic( mat4x4& mat, const float left, const float right, const float bottom, const float top, const float zNear, const float zFar )
	{
#ifdef CLAMP_PERSPECTIVE_ZERO_ONE
		mat.m[0] = 2.0f / ( right - left );
		mat.m[5] = -2.0f / ( top - bottom );
		mat.m[10] = 1.0f / ( zNear - zFar );
		mat.m[12] = -( right + left ) / ( right - left );
		mat.m[13] = -( top + bottom ) / ( top - bottom );
		mat.m[14] = -zNear / ( zFar - zNear );
#else
		mat.m[0] = 2.0f / ( right - left );
		mat.m[5] = 2.0f / ( top - bottom );
		mat.m[10] = 2.0f / ( zNear - zFar );
		mat.m[12] = -( right + left ) / ( right - left );
		mat.m[13] = -( top + bottom ) / ( top - bottom );
		mat.m[14] = -( zFar + zNear ) / ( zFar - zNear );
#endif
	}

	/**
	 * @brief Generates perspective matrix
	 * @param mat - projection matrix
	 * @param fovy - field of view
	 * @param aspect - aspect ratio
	 * @param zNear - near plane distance
	 * @param zFar - far plane distance
	*/
	static void setPerspective( mat4x4& mat, float fovy, float aspect, float zNear, float zFar )
	{
#ifdef CLAMP_PERSPECTIVE_ZERO_ONE
		float tanHalfFovy = tanf( fovy / 2.0f );
		mat.m[0] = 1.0f / ( aspect * tanHalfFovy );
		mat.m[5] = -1.0f / tanHalfFovy;
		mat.m[10] = zFar / ( zNear - zFar );
		mat.m[11] = -1.0f;
		mat.m[14] = -( zFar * zNear ) / ( zFar - zNear );
		mat.m[15] = 0.0f;
#else
		float tanHalfFovy = tanf( fovy / 2.0f );
		mat.m[0] = 1.0f / ( aspect * tanHalfFovy );
		mat.m[5] = 1.0f / tanHalfFovy;
		mat.m[10] = -( zFar + zNear ) / ( zFar - zNear );
		mat.m[11] = -1.0f;
		mat.m[14] = -( 2.0f * zFar * zNear ) / ( zFar - zNear );
		mat.m[15] = 0.0f;
#endif
	}

	/**
	 * @brief Linear interpolation between two matrices
	 * @param a - first matrix
	 * @param b - second matrix
	 * @param t - weight
	 * @return weighted matrix
	*/
	template<> [[nodiscard]] static mat4x4 lerp( const mat4x4& a, const mat4x4& b, const float t )
	{
#if ENABLE_INSTRUCTIONS_AVX512
		return { _mm512_fmadd_ps( _mm512_sub_ps( b.avx512, a.avx512 ), _mm512_set1_ps( t ), a.avx512 ) };
#elif ENABLE_INSTRUCTIONS_AVX2
		__m256 weight = _mm256_set1_ps( t );
		return {
			_mm256_fmadd_ps( _mm256_sub_ps( b.avx2[0], a.avx2[0] ), weight, a.avx2[0] ),
			_mm256_fmadd_ps( _mm256_sub_ps( b.avx2[1], a.avx2[1] ), weight, a.avx2[1] )
		};
#elif ENABLE_INSTRUCTIONS_SSE2
		__m128 weight = _mm_set1_ps( t );
		return {
			_mm_fmadd_ps( _mm_sub_ps( b.simd[0], a.simd[0] ), weight, a.simd[0] ),
			_mm_fmadd_ps( _mm_sub_ps( b.simd[1], a.simd[1] ), weight, a.simd[1] ),
			_mm_fmadd_ps( _mm_sub_ps( b.simd[2], a.simd[2] ), weight, a.simd[2] ),
			_mm_fmadd_ps( _mm_sub_ps( b.simd[3], a.simd[3] ), weight, a.simd[3] )
		};
#else
		return ( b - a ) * t + a;
#endif
	}

	/**
	 * @brief Normalizes the first 3 columns of the matrix. For affine transformations,
	 * this operation removes scaling from the transform.
	 * @param transform - matrix transform
	 * @return normalized affine transformation
	*/
	static mat4x4 normalize( const mat4x4& transform )
	{
		vec3f len = Math::parallel::length( transform.x_axis, transform.y_axis, transform.z_axis );
		return { transform.x_axis / len.x, transform.y_axis / len.y, transform.z_axis / len.z, transform.origin };
	}

	/**
	 * @brief Utility for creating matrices
	*/
	namespace create
	{
		constexpr mat4x4 translation( const vec3f& trans )
		{
			mat4x4 m = Math::IDENTITY<mat4x4>;
			setTranslation( m, trans );
			return m;
		}

		static mat4x4 rotation( const euler& rot )
		{
			mat4x4 m = Math::IDENTITY<mat4x4>;
			setRotation( m, rot );
			return m;
		}

		static mat4x4 rotation( const quat& q )
		{
			mat4x4 m = Math::IDENTITY<mat4x4>;
			setRotation( m, q );
			return m;
		}

		constexpr mat4x4 scale( const vec3f& sc )
		{
			return mat4x4{ sc.x, 0.0f, 0.0f, 0.0f, 0.0f, sc.y, 0.0f, 0.0f, 0.0f, 0.0f, sc.z, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
		}

		static mat4x4 transform( const vec3f translation, const euler& rotation, const vec3f& scale )
		{
			mat4x4 m = Math::IDENTITY<mat4x4>;
			setTransform( m, translation, rotation, scale );
			return m;
		}

		static mat4x4 transform( const vec3f translation, const quat& rotation, const vec3f& scale )
		{
			mat4x4 m = Math::IDENTITY<mat4x4>;
			setTransform( m, translation, rotation, scale );
			return m;
		}

		static mat4x4 view( const vec3f& position, const vec3f& view, const vec3f& up )
		{
			mat4x4 m = Math::IDENTITY<mat4x4>;
			lookAt( m, position, view, up );
			return m;
		}

		static mat4x4 perspective( float fovy, float aspect, float zNear, float zFar )
		{
			mat4x4 m = Math::ZERO<mat4x4>;
			setPerspective( m, fovy, aspect, zNear, zFar );
			return m;
		}
	};

	/**
	 * @brief Generates a random matrix within the range
	 * @param min - minimum matrix
	 * @param max - maximum matrix
	 * @return random matrix
	*/
	template<> [[nodiscard]] static mat4x4 random( const mat4x4& min, const mat4x4& max )
	{
		return {
			Math::random<vec4f>( min.col[0], max.col[0] ),
			Math::random<vec4f>( min.col[1], max.col[1] ),
			Math::random<vec4f>( min.col[2], max.col[2] ),
			Math::random<vec4f>( min.col[3], max.col[3] )
		};
	}

	/**
	 * @brief Generates a random rotation matrix
	 * @return random rotation matrix
	*/
	template<> [[nodiscard]] static mat4x4 randomRotation()
	{
		return rotationAround<mat4x4>( randomDirection<vec3f>(), random<float>( 0.0f, TWO_PI<float> ) );
	}

	/**
	 * @brief Generates a random transformation matrix
	 * @param minTrans - minimum translation bounds
	 * @param maxTrans - maximum translation bounds
	 * @param minScale - minimum scaling bounds 
	 * @param maxScale - maximum scaling bounds 
	 * @return random transform matrix
	*/
	static mat4x4 randomTransform( const vec3f& minTrans = ZERO<vec4f>, const vec3f& maxTrans = ZERO<vec4f>, const vec3f& minScale = ONES<vec4f>, const vec3f& maxScale = ONES<vec4f> )
	{
		vec3f translation = Math::random( minTrans, maxTrans );
		euler rotation = Math::random( ZERO<euler>, TWO_PI<euler> );
		vec3f scale = Math::random( minScale, maxScale );
		return create::transform( translation, rotation, scale );
	}

	/**
	 * @brief Generates a random transformation matrix, includes shearing
	 * @param minTrans - minimum translation bounds
	 * @param maxTrans - maximum translation bounds
	 * @param minScale - minimum scaling bounds 
	 * @param maxScale - maximum scaling bounds 
	 * @return random transform matrix
	*/
	static mat4x4 randomTransformAxes( const vec3f& minTrans = ZERO<vec4f>, const vec3f& maxTrans = ZERO<vec4f>, const vec3f& minScale = ONES<vec4f>, const vec3f& maxScale = ONES<vec4f> )
	{
		vec4f origin = Math::random( minTrans, maxTrans );
		return {
			Math::randomDirection<vec3f>() * Math::random( minScale.x, maxScale.x ),
			Math::randomDirection<vec3f>() * Math::random( minScale.y, maxScale.y ),
			Math::randomDirection<vec3f>() * Math::random( minScale.z, maxScale.z ),
			{ origin.x, origin.y, origin.z, 1.0f }
		};
	}
};

#endif
