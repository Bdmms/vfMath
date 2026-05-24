#pragma once
#ifndef VF_MATRIX_MATH_HPP
#define VF_MATRIX_MATH_HPP

#include "QuatMath.hpp"
#include "mat.hpp"

/**
 * @brief Utilities for matrices
*/
namespace Math
{
	template<size_t M, size_t N, typename T> 
	inline constexpr static mat<M, N, T> ZERO<mat<M, N, T>> = mat<M, N, T>::set1( Math::ZERO<T> );

	template<size_t M, size_t N, typename T>
	inline constexpr static mat<M, N, T> ONES<mat<M, N, T>> = mat<M, N, T>::set1( Math::ONES<T> );

	template<size_t M, size_t N, typename T>
	inline constexpr static mat<M, N, T> IDENTITY<mat<M, N, T>> = mat<M, N, T>::identity();

	/**
	 * @brief Calculates the determinant of this matrix
	 * @return determinant
	*/
	template<size_t N, typename T>
	constexpr T determinant( const mat<N, N, T>& matrix )
	{
		return vf::simd::determinant( matrix );
	}

	/**
	 * @brief Creates a transposed matrix
	 * @return transposed matrix
	*/
	template<size_t M, size_t N, typename T>
	constexpr mat<N, M, T> transpose( const mat<M, N, T>& matrix )
	{
		mat<N, M, T> result;
		vf::simd::transpose( result, matrix );
		return result;
	}

	/**
	 * @brief Creates an inverse matrix
	 * @return inverse matrix
	*/
	template<size_t N, typename T>
	constexpr mat<N, N, T> invert( const mat<N, N, T>& matrix )
	{
		mat<N, N, T> result;
		vf::simd::invert( result, matrix );
		return result;
	}

	template<size_t M, size_t N, typename T>
	constexpr mat<2, 2, T> toMat2x2( const mat<M, N, T>& matrix )
	{
		mat<2, 2, T> r;
		r.cols[0] = { matrix[0,0], matrix[1,0] };
		r.cols[1] = { matrix[0,1], matrix[1,1] };
		return r;
	}

	/**
	 * DEPRECATED
	 * @brief Truncates a 4x4 matrix into a 3x3 matrix
	 * @param dst - destination matrix
	 * @param src - source matrix
	*/
	constexpr void copy_mat3f( mat4f& dst, const mat4f& src )
	{
		for( size_t c = 0LLU; c < 3LLU; ++c )
		{
			// TODO: Requires copying last row
			for( size_t r = 0LLU; r < 4LLU; ++r )
			{
				dst[r, c] = src[r, c];
			}
		}
	}

	/**
	 * @brief Sets the translation of the matrix (Overwrites the last column)
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	*/
	constexpr void setTranslation( mat4f& matrix, const vec3f& translation )
	{
		matrix[0,3] = translation.x;
		matrix[1,3] = translation.y;
		matrix[2,3] = translation.z;
	}

	/**
	 * @brief Sets the rotation of the matrix
	 * @param matrix - 2D transform matrix
	 * @param rotation - rotation angle [rad]
	*/
	static void setRotation( mat2f& matrix, float rotation )
	{
		float s = sinf( rotation );
		float c = cosf( rotation );
		matrix[0,0] = c;
		matrix[1,0] = s;
		matrix[0,1] = -s;
		matrix[1,1] = c;
	}

	/**
	 * @brief Sets the rotation of the matrix (Overwrites the 3x3 matrix)
	 * @param matrix - transform matrix
	 * @param rotation - euler rotation vector [rad]
	*/
	static void setRotation( mat4f& matrix, const eulerf32& rotation )
	{
		eulerf32 s = Math::sin( rotation );
		eulerf32 c = Math::cos( rotation );
		matrix[0,0] = c.z * c.y;
		matrix[1,0] = s.z * c.y;
		matrix[2,0] = -s.y;

		matrix[0,1] = c.z * s.y * s.x - s.z * c.x;
		matrix[1,1] = s.z * s.y * s.x + c.z * c.x;
		matrix[2,1] = c.y * s.x;

		matrix[0,2] = c.z * s.y * c.x + s.z * s.x;
		matrix[1,2] = s.z * s.y * c.x - c.z * s.x;
		matrix[2,2] = c.y * c.x;
	}

	/**
	 * @brief Sets the rotation of the matrix (Overwrites the 3x3 matrix)
	 * @param matrix - transform matrix
	 * @param rotation - quaternion vector
	*/
	constexpr void setRotation( mat4f& matrix, const quat& q )
	{
		matrix[0,0] = 1.0f - 2.0f * ( q.y * q.y + q.z * q.z );
		matrix[1,0] = 2.0f * ( q.x * q.y + q.w * q.z );
		matrix[2,0] = 2.0f * ( q.x * q.z - q.w * q.y );

		matrix[0,1] = 2.0f * ( q.x * q.y - q.w * q.z );
		matrix[1,1] = 1.0f - 2.0f * ( q.x * q.x + q.z * q.z );
		matrix[2,1] = 2.0f * ( q.y * q.z + q.w * q.x );

		matrix[0,2] = 2.0f * ( q.x * q.z + q.w * q.y );
		matrix[1,2] = 2.0f * ( q.y * q.z - q.w * q.x );
		matrix[2,2] = 1.0f - 2.0f * ( q.x * q.x + q.y * q.y );
	}

	/**
	 * @brief Sets the rotation of the matrix (Overwrites the 3x3 matrix)
	 * @param matrix - transform matrix
	 * @param rotation - axis-angle rotation
	*/
	static void setRotation( mat4f& matrix, const AxisAngle& rotation )
	{
		float angle = Math::length( rotation );
		if( angle < Math::EPSILON<float> )
		{
			copy_mat3f( matrix, Math::IDENTITY<mat4f> );
		}
		else
		{
			vec3f axis = rotation / angle;
			float sin = sinf( angle );
			float cos = cosf( angle );
			vec3f tpose = ( 1.0f - cos ) * axis;

			vf::simd::fmadd( matrix.cols[0], axis, vec4f32::set1( tpose.x ), vec4f( cos, sin * axis.z, -sin * axis.y, 0.0f ) );
			vf::simd::fmadd( matrix.cols[1], axis, vec4f32::set1( tpose.y ), vec4f( -sin * axis.z, cos, sin * axis.x, 0.0f ) );
			vf::simd::fmadd( matrix.cols[2], axis, vec4f32::set1( tpose.z ), vec4f( sin * axis.y, -sin * axis.x, cos, 0.0f ) );
		}
	}

	/**
	 * @brief Sets the inverse rotation of the matrix
	 * @param matrix - 2D transform matrix
	 * @param rotation - rotation angle [rad]
	*/
	static void setRotationInverse( mat2f& matrix, float rotation )
	{
		float s = sinf( rotation );
		float c = cosf( rotation );
		matrix[0, 0] = c;
		matrix[1, 0] = -s;
		matrix[0, 1] = s;
		matrix[1, 1] = c;
	}

	/**
	 * @brief Sets the inverse rotation of the matrix (Overwrites the 3x3 matrix)
	 * @param matrix - transform matrix
	 * @param rotation - euler rotation vector [rad]
	*/
	static void setRotationInverse( mat4f& matrix, const eulerf32& rotation )
	{
		eulerf32 s = Math::sin( rotation );
		eulerf32 c = Math::cos( rotation );
		matrix[0,0] = c.z * c.y;
		matrix[0,1] = s.z * c.y;
		matrix[0,2] = -s.y;

		matrix[1,0] = c.z * s.y * s.x - s.z * c.x;
		matrix[1,1] = s.z * s.y * s.x + c.z * c.x;
		matrix[1,2] = c.y * s.x;

		matrix[2,0] = c.z * s.y * c.x + s.z * s.x;
		matrix[2,1] = s.z * s.y * c.x - c.z * s.x;
		matrix[2,2] = c.y * c.x;
	}

	/**
	 * @brief Sets the inverse rotation of the matrix (Overwrites the 3x3 matrix)
	 * @param matrix - transform matrix
	 * @param rotation - quaternion vector
	*/
	constexpr void setRotationInverse( mat4f& matrix, const quat& q )
	{
		matrix[0,0] = 1.0f - 2.0f * ( q.y * q.y + q.z * q.z );
		matrix[0,1] = 2.0f * ( q.x * q.y + q.w * q.z );
		matrix[0,2] = 2.0f * ( q.x * q.z - q.w * q.y );

		matrix[1,0] = 2.0f * ( q.x * q.y - q.w * q.z );
		matrix[1,1] = 1.0f - 2.0f * ( q.x * q.x + q.z * q.z );
		matrix[1,2] = 2.0f * ( q.y * q.z + q.w * q.x );

		matrix[2,0] = 2.0f * ( q.x * q.z + q.w * q.y );
		matrix[2,1] = 2.0f * ( q.y * q.z - q.w * q.x );
		matrix[2,2] = 1.0f - 2.0f * ( q.x * q.x + q.y * q.y );
	}

	/*
	 * @brief Sets the 2D transform of the 2x2 matrix
	 * @param rotation - rotation angle [rad]
	 * @param scale - scaling vector
	*/
	static void setTransform( mat2f& matrix, const float rotation, const vec2f scale )
	{
		setRotation( matrix, rotation );
		matrix[0,0] *= scale.x;
		matrix[1,0] *= scale.x;
		matrix[0,1] *= scale.y;
		matrix[1,1] *= scale.y;
	}

	/**
	 * @brief Sets the transform of the matrix
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param rotation - euler rotation vector [rad]
	 * @param scale - scaling vector
	*/
	static void setTransform( mat4f& matrix, const vec3f translation, const eulerf32& rotation, const vec3f& scale )
	{
		setRotation( matrix, rotation );
		matrix[0] *= scale.x;
		matrix[1] *= scale.y;
		matrix[2] *= scale.z;
		setTranslation( matrix, translation );
	}

	/**
	 * @brief Sets the transform of the matrix
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param q - quaternion rotation
	 * @param scale - scaling vector
	*/
	constexpr void setTransform( mat4f& matrix, const vec3f translation, const quat& q, const vec3f& scale )
	{
		setRotation( matrix, q );
		matrix[0] *= scale.x;
		matrix[1] *= scale.y;
		matrix[2] *= scale.z;
		setTranslation( matrix, translation );
	}

	/**
	 * @brief Sets the transform of the matrix
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param q - quaternion rotation
	*/
	constexpr void setTransform( mat4f& matrix, const vec3f translation, const quat& q )
	{
		setRotation( matrix, q );
		setTranslation( matrix, translation );
	}

	/*
	 * @brief Sets the inverse 2D transform of the 2x2 matrix
	 * @param rotation - rotation angle [rad]
	 * @param scale - scaling vector
	*/
	static void setTransformInverse( mat2f& matrix, const float rotation, const vec2f scale )
	{
		setRotationInverse( matrix, rotation );
		matrix[0, 0] /= scale.x;
		matrix[1, 0] /= scale.y;
		matrix[0, 1] /= scale.x;
		matrix[1, 1] /= scale.y;
	}

	/**
	 * @brief Sets the inverse transform of the matrix
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param rotation - euler rotation vector [rad]
	 * @param scale - scaling vector
	*/
	static void setTransformInverse( mat4f& matrix, const vec3f translation, const eulerf32& rotation, const vec3f& scale )
	{
		setRotationInverse( matrix, rotation );
		vec3f copiedScale = { scale.x, scale.y, scale.z, 1.0f };
		matrix.x_axis /= copiedScale;
		matrix.y_axis /= copiedScale;
		matrix.z_axis /= copiedScale;
		matrix[0,3] = -( matrix.x_axis.x * translation.x + matrix.y_axis.x * translation.y + matrix.z_axis.x * translation.z );
		matrix[1,3] = -( matrix.x_axis.y * translation.x + matrix.y_axis.y * translation.y + matrix.z_axis.y * translation.z );
		matrix[2,3] = -( matrix.x_axis.z * translation.x + matrix.y_axis.z * translation.y + matrix.z_axis.z * translation.z );
	}

	/**
	 * @brief Sets the inverse transform of the matrix
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param q - quaternion rotation
	 * @param scale - scaling vector
	*/
	constexpr void setTransformInverse( mat4f& matrix, const vec3f& translation, const quat& q, const vec3f& scale )
	{
		setRotationInverse( matrix, q );
		vec3f copiedScale = { scale.x, scale.y, scale.z, 1.0f };
		matrix.x_axis /= copiedScale;
		matrix.y_axis /= copiedScale;
		matrix.z_axis /= copiedScale;
		matrix[0,3] = -( matrix.x_axis.x * translation.x + matrix.y_axis.x * translation.y + matrix.z_axis.x * translation.z );
		matrix[1,3] = -( matrix.x_axis.y * translation.x + matrix.y_axis.y * translation.y + matrix.z_axis.y * translation.z );
		matrix[2,3] = -( matrix.x_axis.z * translation.x + matrix.y_axis.z * translation.y + matrix.z_axis.z * translation.z );
	}

	/**
	 * @brief Sets the inverse transform of the matrix
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param q - quaternion rotation
	*/
	constexpr void setTransformInverse( mat4f& matrix, const vec3f& translation, const quat& q )
	{
		setRotationInverse( matrix, q );
		matrix[0,3] = -( matrix.x_axis.x * translation.x + matrix.y_axis.x * translation.y + matrix.z_axis.x * translation.z );
		matrix[1,3] = -( matrix.x_axis.y * translation.x + matrix.y_axis.y * translation.y + matrix.z_axis.y * translation.z );
		matrix[2,3] = -( matrix.x_axis.z * translation.x + matrix.y_axis.z * translation.y + matrix.z_axis.z * translation.z );
	}

	/**
	 * @brief Sets the transform to match the boundary.
	 * @param matrix - transform matrix
	 * @param bounds - vector bounds
	*/
	constexpr void setTransform( mat4f& matrix, Bounds<vec4f> bounds )
	{
		vec4f difference = ( bounds.max - bounds.min ) * 0.5f;

		matrix[0,0] = difference.x;
		matrix[1,0] = 0.0f;
		matrix[2,0] = 0.0f;

		matrix[0,1] = 0.0f;
		matrix[1,1] = difference.y;
		matrix[2,1] = 0.0f;

		matrix[0,2] = 0.0f;
		matrix[1,2] = 0.0f;
		matrix[2,2] = difference.z;

		matrix[0,3] = difference.x + bounds.min.x;
		matrix[1,3] = difference.y + bounds.min.y;
		matrix[2,3] = difference.z + bounds.min.z;
	}

	/**
	 * @brief Converts a rotation matrix into a quaternion.
	 * @param rotation - normalized rotation matrix
	 * @return quaternion rotation
	*/
	static quat toQuat( const mat4f& rotation )
	{
		// TODO: This may flip signs
		float a = rotation[0,0] - rotation[1,1] - rotation[2,2];
		float b = rotation[1,1] - rotation[0,0] - rotation[2,2];
		float c = rotation[2,2] - rotation[0,0] - rotation[1,1];
		float d = rotation[0,0] + rotation[1,1] + rotation[2,2];
		quat q;
		float t;
		float fac;

		if( c > d )
		{
			t = sqrtf( c + 1.0f ) * 0.5f;
			fac = 0.25f / t;
			q = { ( rotation[0,2] + rotation[2,0] ) * fac, ( rotation[2,1] + rotation[1,2] ) * fac, t, ( rotation[1,0] - rotation[0,1] ) * fac };
		}
		else if( b > d )
		{
			t = sqrtf( b + 1.0f ) * 0.5f;
			fac = 0.25f / t;
			q = { ( rotation[1,0] + rotation[0,1] ) * fac, t, ( rotation[2,1] + rotation[1,2] ) * fac, ( rotation[0,2] - rotation[2,0] ) * fac };
		}
		else if( a > d )
		{
			t = sqrtf( a + 1.0f ) * 0.5f;
			fac = 0.25f / t;
			q = { t, ( rotation[1,0] + rotation[0,1] ) * fac, ( rotation[0,2] + rotation[2,0] ) * fac, ( rotation[2,1] - rotation[1,2] ) * fac };
		}
		else
		{
			t = sqrtf( d + 1.0f ) * 0.5f;
			fac = 0.25f / t;
			q = { ( rotation[2,1] - rotation[1,2] ) * fac, ( rotation[0,2] - rotation[2,0] ) * fac, ( rotation[1,0] - rotation[0,1] ) * fac, t };
		}

		return q;
	}

	/**
	 * @brief Converts rotation matrix to an axis and angle.
	 * Note: matrix must be a valid rotation matrix
	 * @param rotation - rotation matrix
	 * @return Axis angle vector
	*/
	[[nodiscard]] static AxisAngle toAxisAngle( const mat4f& rotation )
	{
		vec3f diagonal = 0.5f * vec3f{ rotation.x_axis.x, rotation.y_axis.y, rotation.z_axis.z };
		float angle = acosf( diagonal.x + diagonal.y + diagonal.z - 0.5f );

		return ( vec3f{ rotation.y_axis.z, rotation.z_axis.x, rotation.x_axis.y } - vec3f{ rotation.z_axis.y, rotation.x_axis.z, rotation.y_axis.x } ) * ( angle / ( 2.0f * sinf( angle ) ) );
	}

	/**
	 * @brief Decomposes an affine transform into its components
	 * @param matrix - affine transform matrix
	 * @param position - translation component
	 * @param rotation - rotation component
	 * @param scale - scale component
	*/
	static void decompose( const mat4f& matrix, vec4f& position, quat& rotation, vec3f& scale )
	{
		scale = Math::parallel::length( matrix.x_axis, matrix.y_axis, matrix.z_axis );
		rotation = toQuat( mat4f{ matrix.x_axis / scale.x, matrix.y_axis / scale.y, matrix.z_axis / scale.z, Math::axis::W<vec4f> } );
		position = matrix.origin;
	}

	/**
	 * @brief Generates a view matrix
	 * @param mat - view matrix
	 * @param eye - view position vector
	 * @param center - view target vector
	 * @param up - view up vector
	*/
	static void lookAt( mat4f& mat, const vec4f& eye, const vec4f& center, const vec3f& up )
	{
		// TODO: Re-write without intrinsics !!!
		vec3f f = Math::normalize( eye - center );
		vec3f s = Math::normalize( Math::cross( up, f ) );
		vec3f u = Math::cross( f, s );

		constexpr static __m128 SIMD_4f_ZERO = { 0.0f, 0.0f, 0.0f, 0.0f };
		mat.simd[0] = _mm_transpose_ps( s.simd, u.simd, f.simd, SIMD_4f_ZERO, swizzle::XXXX );
		mat.simd[1] = _mm_transpose_ps( s.simd, u.simd, f.simd, SIMD_4f_ZERO, swizzle::YYYY );
		mat.simd[2] = _mm_transpose_ps( s.simd, u.simd, f.simd, SIMD_4f_ZERO, swizzle::ZZZZ );
		mat.simd[3] = _mm_mul_ps( _mm_dot3v_ps( s.simd, eye.simd, u.simd, eye.simd, f.simd, eye.simd, _mm_xyzw_ps( 0.0f, 0.0f, 0.0f, 1.0f ) ), 
			_mm_xyzw_ps( -1.0f, -1.0f, -1.0f, 1.0f ) );
	};

	/**
	 * @brief Generates a rotation matrix that rotates the x-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @param trackAxis - axis to rotate around (non-zero)
	 * @param position - translation component (optional)
	 * @return rotation matrix
	*/
	[[nodiscard]] static mat4f lookToX( const vec3f& forward, const vec3f& trackAxis, const vec4f& position = Math::axis::W<vec4f> )
	{
		vec4f z_axis = Math::normalize( Math::cross( forward, trackAxis ) );
		vec4f y_axis = Math::cross( z_axis, forward );
		return mat4f{ forward, y_axis, z_axis, position };
	}

	/**
	 * @brief Generates a rotation matrix that rotates the x-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @return rotation matrix
	*/
	[[nodiscard]] static mat4f lookToX( const vec3f& forward )
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
	[[nodiscard]] static mat4f lookToY( const vec3f& forward, const vec3f& trackAxis, const vec4f& position = Math::axis::W<vec4f> )
	{
		vec4f z_axis = Math::normalize( Math::cross( trackAxis, forward ) );
		vec4f x_axis = Math::cross( forward, z_axis );
		return mat4f{ x_axis, forward, z_axis, position };
	}

	/**
	 * @brief Generates a rotation matrix that rotates the y-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @return rotation matrix
	*/
	[[nodiscard]] static mat4f lookToY( const vec3f& forward )
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
	[[nodiscard]] static mat4f lookToZ( const vec3f& forward, const vec3f& trackAxis, const vec4f& position = Math::axis::W<vec4f> )
	{
		vec4f x_axis = Math::normalize( Math::cross( trackAxis, forward ) );
		vec4f y_axis = Math::cross( forward, x_axis );
		return mat4f{ x_axis, y_axis, forward, position };
	}

	/**
	 * @brief Generates a rotation matrix that rotates the z-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @return rotation matrix
	*/
	[[nodiscard]] static mat4f lookToZ( const vec3f& forward )
	{
		return lookToX( forward, Math::cross( Math::axis::Z<vec4f>, forward ) );
	}

	/**
	 * @brief Generates a rotation matrix that rotates around an axis
	 * @param axis - axis of the rotation (normalized)
	 * @param angle - angle of the rotation [rad]
	 * @return rotation matrix
	*/
	template<> [[nodiscard]] static mat4f rotationAround( const vec3f& axis, const float angle )
	{
		float sin = sinf( angle );
		float cos = cosf( angle );
		vec3f tpose = ( 1.0f - cos ) * axis;

		mat4f r;
		vf::simd::fmadd( r.x_axis, axis, vec4f::set1( tpose.x ), vec4f( cos, sin * axis.z, -sin * axis.y, 0.0f ) );
		vf::simd::fmadd( r.y_axis, axis, vec4f::set1( tpose.y ), vec4f( -sin * axis.z, cos, sin * axis.x, 0.0f ) );
		vf::simd::fmadd( r.z_axis, axis, vec4f::set1( tpose.z ), vec4f( sin * axis.y, -sin * axis.x, cos, 0.0f ) );
		r.origin = axis::W<vec4f>;
		return r;
	}

	/**
	 * @brief Generates a rotation matrix that rotates between two vectors
	 * @param from - initial vector (normalized)
	 * @param to - final vector (normalized)
	 * @return rotation matrix
	*/
	template<> [[nodiscard]] static mat4f rotationBetween( const vec3f& from, const vec3f& to )
	{
		float product = dot_3D( from, to );
		if( product > 0.999999f ) return Math::IDENTITY<mat4f>;
		if( product < -0.999999f ) return rotationAround<mat4f>( orthogonal( from ), PI<float> );

		vec3f axis = Math::normalize( Math::cross( from, to ) );
		float cos = Math::dot( from, to );
		float sin = sqrtf( 1.0f - cos * cos );
		vec3f tpose = ( 1.0f - cos ) * axis;

		mat4f r;
		vf::simd::fmadd( r.x_axis, axis, vec4f::set1( tpose.x ), vec4f( cos, sin * axis.z, -sin * axis.y, 0.0f ) );
		vf::simd::fmadd( r.y_axis, axis, vec4f::set1( tpose.y ), vec4f( -sin * axis.z, cos, sin * axis.x, 0.0f ) );
		vf::simd::fmadd( r.z_axis, axis, vec4f::set1( tpose.z ), vec4f( sin * axis.y, -sin * axis.x, cos, 0.0f ) );
		r.origin = axis::W<vec4f>;
		return r;
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
	constexpr void setOrthographic( mat4f& mat, const float left, const float right, const float bottom, const float top, const float zNear, const float zFar )
	{
#ifdef CLAMP_PERSPECTIVE_ZERO_ONE
		mat[0,0] = 2.0f / ( right - left );
		mat[1,1] = -2.0f / ( top - bottom );
		mat[2,2] = 1.0f / ( zNear - zFar );
		mat[0,3] = -( right + left ) / ( right - left );
		mat[1,3] = -( top + bottom ) / ( top - bottom );
		mat[2,3] = -zNear / ( zFar - zNear );
#else
		mat[0,0] = 2.0f / ( right - left );
		mat[1,1] = 2.0f / ( top - bottom );
		mat[2,2] = 2.0f / ( zNear - zFar );
		mat[0,3] = -( right + left ) / ( right - left );
		mat[1,3] = -( top + bottom ) / ( top - bottom );
		mat[2,3] = -( zFar + zNear ) / ( zFar - zNear );
#endif
	}

	/**
	 * @brief Generates perspective matrix
	 * @param mat - projection matrix
	 * @param fovy - field of view [rad]
	 * @param aspect - aspect ratio
	 * @param zNear - near plane distance
	 * @param zFar - far plane distance
	*/
	static void setPerspective( mat4f& mat, float fovy, float aspect, float zNear, float zFar )
	{
#ifdef CLAMP_PERSPECTIVE_ZERO_ONE
		float tanHalfFovy = tanf( fovy / 2.0f );
		mat[0,0] = 1.0f / ( aspect * tanHalfFovy );
		mat[1,1] = -1.0f / tanHalfFovy;
		mat[2,2] = zFar / ( zNear - zFar );
		mat[3,2] = -1.0f;
		mat[2,3] = -( zFar * zNear ) / ( zFar - zNear );
		mat[3,3] = 0.0f;
#else
		float tanHalfFovy = tanf( fovy / 2.0f );
		mat[0,0] = 1.0f / ( aspect * tanHalfFovy );
		mat[1,1] = 1.0f / tanHalfFovy;
		mat[2,2] = -( zFar + zNear ) / ( zFar - zNear );
		mat[3,2] = -1.0f;
		mat[2,3] = -( 2.0f * zFar * zNear ) / ( zFar - zNear );
		mat[3,3] = 0.0f;
#endif
	}

	// TODO: Re-write without intrinsics !!!
	static void slerp( mat4f& result, const mat4f& a, const mat4f& b, const float t )
	{
		// Calculate rotation between a and b
		__m128 c0 = _mm_dot3_ps( a.x_axis.simd, b.x_axis.simd, a.y_axis.simd, b.x_axis.simd, a.z_axis.simd, b.x_axis.simd );
		__m128 c1 = _mm_dot3_ps( a.x_axis.simd, b.y_axis.simd, a.y_axis.simd, b.y_axis.simd, a.z_axis.simd, b.y_axis.simd );
		__m128 c2 = _mm_dot3_ps( a.x_axis.simd, b.z_axis.simd, a.y_axis.simd, b.z_axis.simd, a.z_axis.simd, b.z_axis.simd );

		// Calculate the axis of rotation
		__m128 diagonal = _mm_mul_ps( _mm_set1_ps( 0.5f ), _mm_xyzw_ps( c0.m128_f32[0], c1.m128_f32[1], c2.m128_f32[2], 0.0f ) );
		float angle = acosf( diagonal.m128_f32[0] + diagonal.m128_f32[1] + diagonal.m128_f32[2] - 0.5f );
		__m128 axis = _mm_div_ps( _mm_sub_ps( _mm_xyzw_ps( c1.m128_f32[2], c2.m128_f32[0], c0.m128_f32[1], 0.0f ), _mm_xyzw_ps( c2.m128_f32[1], c0.m128_f32[2], c1.m128_f32[0], 0.0f ) ), _mm_set1_ps( 2.0f * sinf( angle ) ) );

		float c = cosf( angle * t );
		__m128 vc = _mm_mul_ps( axis, _mm_set1_ps( 1.0f - c ) );
		__m128 vs = _mm_mul_ps( axis, _mm_set1_ps( sinf( angle * t ) ) );

		// Create new weighted rotation
		c0 = _mm_fmadd_ps( axis, _mm_set1_ps( vc.m128_f32[0] ), _mm_xyzw_ps( c, vs.m128_f32[2], -vs.m128_f32[1], 0.0f ) );
		c1 = _mm_fmadd_ps( axis, _mm_set1_ps( vc.m128_f32[1] ), _mm_xyzw_ps( -vs.m128_f32[2], c, vs.m128_f32[0], 0.0f ) );
		c2 = _mm_fmadd_ps( axis, _mm_set1_ps( vc.m128_f32[2] ), _mm_xyzw_ps( vs.m128_f32[1], -vs.m128_f32[0], c, 0.0f ) );

		// Apply weighted rotation to a
		__m128 r0a = _mm_xyzw_ps( a.x_axis.x, a.y_axis.x, a.z_axis.x, 0.0f );
		__m128 r1a = _mm_xyzw_ps( a.x_axis.y, a.y_axis.y, a.z_axis.y, 0.0f );
		__m128 r2a = _mm_xyzw_ps( a.x_axis.z, a.y_axis.z, a.z_axis.z, 0.0f );

		result.x_axis.simd = _mm_dot3_ps( r0a, c0, r1a, c0, r2a, c0 );
		result.y_axis.simd = _mm_dot3_ps( r0a, c1, r1a, c1, r2a, c1 );
		result.z_axis.simd = _mm_dot3_ps( r0a, c2, r1a, c2, r2a, c2 );
	}

	/**
	 * @brief Creates a copy of the matrix translated
	 * @param matrix - source transform
	 * @param translation - displacement vector
	 * @return translated transform
	*/
	constexpr mat4f translate( const mat4f& matrix, const vec3f& translation )
	{
		mat4f copy = matrix;
		copy.origin += translation;
		return copy;
	}

	/**
	 * @brief Creates a copy of the matrix scaled around its origin
	 * @param matrix - source transform
	 * @param scale - scale vector
	 * @return scaled transform
	*/
	template<size_t N, typename T>
	constexpr mat<N, N, T> scale( const mat<N, N, T>& matrix, const vec<N, T> scale )
	{
		mat<N, N, T> copy = matrix;
		for( size_t i = 0LLU; i < N; ++i )
		{
			copy[i] *= scale[i];
		}
		return copy;
	}

	/**
	 * @brief Utility for creating matrices
	*/
	namespace create
	{
		/**
		 * @brief Creates a 3D translation matrix.
		 * @param trans - translation vector
		 * @return translation matrix
		*/
		constexpr mat4f translation( const vec3f& trans )
		{
			mat4f m = Math::IDENTITY<mat4f>;
			setTranslation( m, trans );
			return m;
		}

		/**
		 * @brief Creates a 2D rotation matrix.
		 * @param rot - rotation angle [rad]
		 * @return rotation matrix
		*/
		static mat2f rotation( const float rot )
		{
			mat2f m;
			setRotation( m, rot );
			return m;
		}

		/**
		 * @brief Creates a 3D rotation matrix.
		 * @param rot - rotation euler angles [rad]
		 * @return rotation matrix
		*/
		static mat4f rotation( const eulerf32& rot )
		{
			mat4f m = Math::IDENTITY<mat4f>;
			setRotation( m, rot );
			return m;
		}

		/**
		 * @brief Creates a 3D rotation matrix.
		 * @param rot - rotation quaternion
		 * @return rotation matrix
		*/
		constexpr mat4f rotation( const quat& q )
		{
			mat4f m = Math::IDENTITY<mat4f>;
			setRotation( m, q );
			return m;
		}

		/**
		 * @brief Creates a 2D scale matrix.
		 * @param sc - scale vector
		 * @return scale matrix
		*/
		constexpr mat2f scale( const vec2f sc )
		{
			return { vec2f( sc.x, 0.0f ), vec2f( 0.0f, sc.y ) };
		}

		/**
		 * @brief Creates a 3D scale matrix.
		 * @param sc - scale vector
		 * @return scale matrix
		*/
		constexpr mat4f scale( const vec3f& sc )
		{
			return mat4f{ 
				vec4f{ sc.x, 0.0f, 0.0f, 0.0f }, 
				vec4f{ 0.0f, sc.y, 0.0f, 0.0f }, 
				vec4f{ 0.0f, 0.0f, sc.z, 0.0f }, 
				vec4f{ 0.0f, 0.0f, 0.0f, 1.0f } };
		}

		/**
		 * @brief Creates a 2D transform matrix.
		 * @param rotation - rotation angle [rad]
		 * @param scale - scale vector
		 * @return transform matrix
		*/
		static mat2f transform( const float rotation, const vec2f scale )
		{
			mat2f m;
			setTransform( m, rotation, scale );
			return m;
		}

		/**
		 * @brief Creates a 3D transform matrix.
		 * @param translation - translation vector
		 * @param rot - rotation euler angles [rad]
		 * @param scale - scale vector
		 * @return transform matrix
		*/
		static mat4f transform( const vec3f translation, const eulerf32& rotation, const vec3f& scale )
		{
			mat4f m = Math::IDENTITY<mat4f>;
			setTransform( m, translation, rotation, scale );
			return m;
		}

		/**
		 * @brief Creates a 3D transform matrix.
		 * @param translation - translation vector
		 * @param rot - rotation quaternion
		 * @param scale - scale vector
		 * @return transform matrix
		*/
		constexpr mat4f transform( const vec3f translation, const quat& rotation, const vec3f& scale )
		{
			mat4f m = Math::IDENTITY<mat4f>;
			setTransform( m, translation, rotation, scale );
			return m;
		}

		/**
		 * @brief Creates a view matrix.
		 * @param position - position of camera
		 * @param center - position of target
		 * @param up - camera orientation axis
		 * @return view matrix
		*/
		static mat4f view( const vec3f& position, const vec3f& center, const vec3f& up )
		{
			mat4f m = Math::IDENTITY<mat4f>;
			lookAt( m, position, center, up );
			return m;
		}

		/**
		 * @brief Creates a camera perspective matrix.
		 * @param fovy - field of view [rad]
		 * @param aspect - aspect ratio
		 * @param zNear - near plane depth
		 * @param zFar - far plane depth
		 * @return perspective matrix
		*/
		static mat4f perspective( float fovy, float aspect, float zNear, float zFar )
		{
			mat4f m = Math::ZERO<mat4f>;
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
	template<> [[nodiscard]] static mat4f random( const mat4f& min, const mat4f& max )
	{
		return {
			Math::random<vec4f>( min.cols[0], max.cols[0] ),
			Math::random<vec4f>( min.cols[1], max.cols[1] ),
			Math::random<vec4f>( min.cols[2], max.cols[2] ),
			Math::random<vec4f>( min.cols[3], max.cols[3] )
		};
	}

	/**
	 * @brief Generates a random rotation matrix
	 * @return random rotation matrix
	*/
	template<> [[nodiscard]] static mat4f randomRotation()
	{
		return rotationAround<mat4f>( randomDirection<vec3f>(), rand() * RAND_CONVERT_TAU );
	}

	/**
	 * @brief Generates a random transformation matrix
	 * @param minTrans - minimum translation bounds
	 * @param maxTrans - maximum translation bounds
	 * @param minScale - minimum scaling bounds 
	 * @param maxScale - maximum scaling bounds 
	 * @return random transform matrix
	*/
	static mat4f randomTransform( const vec3f& minTrans = ZERO<vec4f>, const vec3f& maxTrans = ZERO<vec4f>, const vec3f& minScale = ONES<vec4f>, const vec3f& maxScale = ONES<vec4f> )
	{
		vec3f translation = Math::random( minTrans, maxTrans );
		eulerf rotation = Math::random( ZERO<eulerf>, TWO_PI<eulerf> );
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
	static mat4f randomTransformAxes( const vec3f& minTrans = ZERO<vec4f>, const vec3f& maxTrans = ZERO<vec4f>, const vec3f& minScale = ONES<vec4f>, const vec3f& maxScale = ONES<vec4f> )
	{
		vec4f origin = Math::random( minTrans, maxTrans );
		return {
			Math::randomDirection<vec3f>() * Math::random( minScale.x, maxScale.x ),
			Math::randomDirection<vec3f>() * Math::random( minScale.y, maxScale.y ),
			Math::randomDirection<vec3f>() * Math::random( minScale.z, maxScale.z ),
			vec4f{ origin.x, origin.y, origin.z, 1.0f }
		};
	}
};

#endif
