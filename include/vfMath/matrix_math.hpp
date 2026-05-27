#pragma once
#ifndef VF_MATRIX_MATH_HPP
#define VF_MATRIX_MATH_HPP

#include "quat_math.hpp"
#include "matrix.hpp"

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
	template<size_t N, typename T> requires std::floating_point<T>
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
	template<typename T>
	constexpr void setTranslation( mat4<T>& matrix, const vec4<T>& translation )
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
	template<typename T> requires std::floating_point<T>
	static void setRotation( mat2<T>& matrix, T rotation )
	{
		T s = std::sin( rotation );
		T c = std::cos( rotation );
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
	template<typename T> requires std::floating_point<T>
	static void setRotation( mat4<T>& matrix, const euler<T>& rotation )
	{
		euler<T> s = sin( rotation );
		euler<T> c = cos( rotation );
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
	template<typename T> requires std::floating_point<T>
	constexpr void setRotation( mat4<T>& matrix, const quat<T>& q )
	{
		matrix[0,0] = ONES<T> - TWO<T> * ( q.y * q.y + q.z * q.z );
		matrix[1,0] = TWO<T> * ( q.x * q.y + q.w * q.z );
		matrix[2,0] = TWO<T> * ( q.x * q.z - q.w * q.y );

		matrix[0,1] = TWO<T> * ( q.x * q.y - q.w * q.z );
		matrix[1,1] = ONES<T> - TWO<T> * ( q.x * q.x + q.z * q.z );
		matrix[2,1] = TWO<T> * ( q.y * q.z + q.w * q.x );

		matrix[0,2] = TWO<T> * ( q.x * q.z + q.w * q.y );
		matrix[1,2] = TWO<T> * ( q.y * q.z - q.w * q.x );
		matrix[2,2] = ONES<T> - TWO<T> * ( q.x * q.x + q.y * q.y );
	}

	/**
	 * @brief Sets the rotation of the matrix (Overwrites the 3x3 matrix)
	 * @param matrix - transform matrix
	 * @param rotation - axis-angle rotation
	*/
	template<typename T> requires std::floating_point<T>
	static void setRotation( mat4<T>& matrix, const AxisAngle<T>& rotation )
	{
		T angle = length( rotation );
		if( angle < EPSILON<T> )
		{
			copy_mat3f( matrix, Math::IDENTITY<mat4<T>> );
		}
		else
		{
			vec4<T> axis = rotation / angle;
			T sin = std::sin( angle );
			T cos = std::cos( angle );
			vec4<T> tpose = ( ONES<T> - cos ) * axis;

			vf::simd::fmadd( matrix.cols[0], axis, vec4<T>::set1( tpose.x ), vec4<T>( cos, sin * axis.z, -sin * axis.y, ZERO<T> ) );
			vf::simd::fmadd( matrix.cols[1], axis, vec4<T>::set1( tpose.y ), vec4<T>( -sin * axis.z, cos, sin * axis.x, ZERO<T> ) );
			vf::simd::fmadd( matrix.cols[2], axis, vec4<T>::set1( tpose.z ), vec4<T>( sin * axis.y, -sin * axis.x, cos, ZERO<T> ) );
		}
	}

	/**
	 * @brief Sets the inverse rotation of the matrix
	 * @param matrix - 2D transform matrix
	 * @param rotation - rotation angle [rad]
	*/
	template<typename T> requires std::floating_point<T>
	static void setRotationInverse( mat2<T>& matrix, T rotation )
	{
		T s = std::sin( rotation );
		T c = std::cos( rotation );
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
	template<typename T> requires std::floating_point<T>
	static void setRotationInverse( mat4<T>& matrix, const euler<T>& rotation )
	{
		euler<T> s = sin( rotation );
		euler<T> c = cos( rotation );
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
	template<typename T> requires std::floating_point<T>
	constexpr void setRotationInverse( mat4<T>& matrix, const quat<T>& q )
	{
		matrix[0,0] = ONES<T> - TWO<T> * ( q.y * q.y + q.z * q.z );
		matrix[0,1] = TWO<T> * ( q.x * q.y + q.w * q.z );
		matrix[0,2] = TWO<T> * ( q.x * q.z - q.w * q.y );

		matrix[1,0] = TWO<T> * ( q.x * q.y - q.w * q.z );
		matrix[1,1] = ONES<T> - TWO<T> * ( q.x * q.x + q.z * q.z );
		matrix[1,2] = TWO<T> * ( q.y * q.z + q.w * q.x );

		matrix[2,0] = TWO<T> * ( q.x * q.z + q.w * q.y );
		matrix[2,1] = TWO<T> * ( q.y * q.z - q.w * q.x );
		matrix[2,2] = ONES<T> - TWO<T> * ( q.x * q.x + q.y * q.y );
	}

	/*
	 * @brief Sets the 2D transform of the 2x2 matrix
	 * @param rotation - rotation angle [rad]
	 * @param scale - scaling vector
	*/
	template<typename T> requires std::floating_point<T>
	static void setTransform( mat2<T>& matrix, T rotation, const vec2<T>& scale )
	{
		setRotation( matrix, rotation );
		matrix[0] *= scale.x;
		matrix[1] *= scale.y;
	}

	/**
	 * @brief Sets the transform of the matrix
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param rotation - euler rotation vector [rad]
	 * @param scale - scaling vector
	*/
	template<typename T> requires std::floating_point<T>
	static void setTransform( mat4<T>& matrix, const vec4<T>& translation, const euler<T>& rotation, const vec4<T>& scale )
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
	template<typename T> requires std::floating_point<T>
	constexpr void setTransform( mat4<T>& matrix, const vec4<T>& translation, const quat<T>& q, const vec4<T>& scale )
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
	template<typename T> requires std::floating_point<T>
	constexpr void setTransform( mat4f& matrix, const vec4<T>& translation, const quat<T>& q )
	{
		setRotation( matrix, q );
		setTranslation( matrix, translation );
	}

	/*
	 * @brief Sets the inverse 2D transform of the 2x2 matrix
	 * @param rotation - rotation angle [rad]
	 * @param scale - scaling vector
	*/
	template<typename T> requires std::floating_point<T>
	static void setTransformInverse( mat2f& matrix, T rotation, const vec2<T>& scale )
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
	template<typename T> requires std::floating_point<T>
	static void setTransformInverse( mat4<T>& matrix, const vec4<T>& translation, const euler<T>& rotation, const vec4<T>& scale )
	{
		setRotationInverse( matrix, rotation );
		vec4<T> copiedScale = { scale.x, scale.y, scale.z, ONES<T> };
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
	template<typename T> requires std::floating_point<T>
	constexpr void setTransformInverse( mat4<T>& matrix, const vec4<T>& translation, const quat<T>& q, const vec4<T>& scale )
	{
		setRotationInverse( matrix, q );
		vec4<T> copiedScale = { scale.x, scale.y, scale.z, ONES<T> };
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
	template<typename T> requires std::floating_point<T>
	constexpr void setTransformInverse( mat4<T>& matrix, const vec4<T>& translation, const quat<T>& q )
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
	template<typename T>
	constexpr void setTransform( mat4<T>& matrix, Bounds<vec4<T>> bounds )
	{
		vec4<T> difference = ( bounds.max - bounds.min ) * HALF<T>;

		matrix[0,0] = difference.x;
		matrix[1,0] = ZERO<T>;
		matrix[2,0] = ZERO<T>;

		matrix[0,1] = ZERO<T>;
		matrix[1,1] = difference.y;
		matrix[2,1] = ZERO<T>;

		matrix[0,2] = ZERO<T>;
		matrix[1,2] = ZERO<T>;
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
	template<typename T>
	constexpr quat<T> toQuat( const mat4<T>& rotation )
	{
		// TODO: This may flip signs
		T a = rotation[0,0] - rotation[1,1] - rotation[2,2];
		T b = rotation[1,1] - rotation[0,0] - rotation[2,2];
		T c = rotation[2,2] - rotation[0,0] - rotation[1,1];
		T d = rotation[0,0] + rotation[1,1] + rotation[2,2];
		quat<T> q;
		T t, fac;

		if( c > d )
		{
			t = sqrt( c + ONES<T> ) * HALF<T>;
			fac = ( HALF<T> * HALF<T> ) / t;
			q = { ( rotation[0,2] + rotation[2,0] ) * fac, ( rotation[2,1] + rotation[1,2] ) * fac, t, ( rotation[1,0] - rotation[0,1] ) * fac };
		}
		else if( b > d )
		{
			t = sqrt( b + ONES<T> ) * HALF<T>;
			fac = ( HALF<T> * HALF<T> ) / t;
			q = { ( rotation[1,0] + rotation[0,1] ) * fac, t, ( rotation[2,1] + rotation[1,2] ) * fac, ( rotation[0,2] - rotation[2,0] ) * fac };
		}
		else if( a > d )
		{
			t = sqrt( a + ONES<T> ) * HALF<T>;
			fac = ( HALF<T> * HALF<T> ) / t;
			q = { t, ( rotation[1,0] + rotation[0,1] ) * fac, ( rotation[0,2] + rotation[2,0] ) * fac, ( rotation[2,1] - rotation[1,2] ) * fac };
		}
		else
		{
			t = sqrt( d + ONES<T> ) * HALF<T>;
			fac = ( HALF<T> * HALF<T> ) / t;
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
	template<typename T>
	[[nodiscard]] static AxisAngle<T> toAxisAngle( const mat4<T>& rotation )
	{
		vec4<T> diagonal = HALF<T> * vec4<T>{ rotation.x_axis.x, rotation.y_axis.y, rotation.z_axis.z };
		T angle = std::acos( diagonal.x + diagonal.y + diagonal.z - HALF<T> );

		return ( vec4<T>{ rotation.y_axis.z, rotation.z_axis.x, rotation.x_axis.y } - vec4<T>{ rotation.z_axis.y, rotation.x_axis.z, rotation.y_axis.x } ) * ( angle / ( TWO<T> * std::sin( angle ) ) );
	}

	/**
	 * @brief Decomposes an affine transform into its components
	 * @param matrix - affine transform matrix
	 * @param position - translation component
	 * @param rotation - rotation component
	 * @param scale - scale component
	*/
	template<typename T>
	static void decompose( const mat4<T>& matrix, vec4<T>& position, quat<T>& rotation, vec4<T>& scale )
	{
		vf::simd::length4( scale, matrix.x_axis, matrix.y_axis, matrix.z_axis, Math::ZERO<vec4<T>> );
		rotation = toQuat( mat4<T>{ matrix.x_axis / scale.x, matrix.y_axis / scale.y, matrix.z_axis / scale.z, Math::axis::W<vec4<T>> } );
		position = matrix.origin;
	}

	/**
	 * @brief Generates a view matrix
	 * @param mat - view matrix
	 * @param eye - view position vector
	 * @param center - view target vector
	 * @param up - view up vector
	*/
	template<typename T>
	static void lookAt( mat4<T>& mat, const vec4<T>& eye, const vec4<T>& center, const vec4<T>& up )
	{
		vec4<T> f = normalize( eye - center );
		vec4<T> s = normalize( cross( up, f ) );
		vec4<T> u = cross( f, s );

		mat.x_axis = { s.x, u.x, f.x, ZERO<T> };
		mat.y_axis = { s.y, u.y, f.y, ZERO<T> };
		mat.z_axis = { s.z, u.z, f.z, ZERO<T> };
		
		vf::simd::dot4( mat.origin, s, eye, u, eye, f, eye, axis::W<vec4<T>>, axis::W<vec4<T>> );
		vf::simd::mul( mat.origin, mat.origin, { NEGATIVE<T>, NEGATIVE<T>, NEGATIVE<T>, ONES<T> } );
	};

	/**
	 * @brief Generates a rotation matrix that rotates the x-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @param trackAxis - axis to rotate around (non-zero)
	 * @param position - translation component (optional)
	 * @return rotation matrix
	*/
	template<typename T>
	[[nodiscard]] static mat4<T> lookToX( const vec4<T>& forward, const vec4<T>& trackAxis, const vec4<T>& position = axis::W<vec4<T>> )
	{
		vec4<T> z_axis = normalize( cross( forward, trackAxis ) );
		vec4<T> y_axis = cross( z_axis, forward );
		return mat4<T>{ forward, y_axis, z_axis, position };
	}

	/**
	 * @brief Generates a rotation matrix that rotates the x-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @return rotation matrix
	*/
	template<typename T>
	[[nodiscard]] static mat4<T> lookToX( const vec4<T>& forward )
	{
		return lookToX( forward, cross( axis::X<vec4<T>>, forward ) );
	}

	/**
	 * @brief Generates a rotation matrix that rotates the y-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @param trackAxis - axis to rotate around
	 * @param position - translation component (optional)
	 * @return rotation matrix
	*/
	template<typename T>
	[[nodiscard]] static mat4<T> lookToY( const vec4<T>& forward, const vec4<T>& trackAxis, const vec4<T>& position = axis::W<vec4<T>> )
	{
		vec4<T> z_axis = normalize( Math::cross( trackAxis, forward ) );
		vec4<T> x_axis = cross( forward, z_axis );
		return mat4<T>{ x_axis, forward, z_axis, position };
	}

	/**
	 * @brief Generates a rotation matrix that rotates the y-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @return rotation matrix
	*/
	template<typename T>
	[[nodiscard]] static mat4<T> lookToY( const vec4<T>& forward )
	{
		return lookToY( forward, cross( axis::Y<vec4<T>>, forward ) );
	}

	/**
	 * @brief Generates a rotation matrix that rotates the z-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @param trackAxis - axis to rotate around
	 * @param position - translation component (optional)
	 * @return rotation matrix
	*/
	template<typename T>
	[[nodiscard]] static mat4<T> lookToZ( const vec4<T>& forward, const vec4<T>& trackAxis, const vec4<T>& position = axis::W<vec4<T>> )
	{
		vec4<T> x_axis = normalize( cross( trackAxis, forward ) );
		vec4<T> y_axis = cross( forward, x_axis );
		return mat4<T>{ x_axis, y_axis, forward, position };
	}

	/**
	 * @brief Generates a rotation matrix that rotates the z-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @return rotation matrix
	*/
	template<typename T>
	[[nodiscard]] static mat4<T> lookToZ( const vec4<T>& forward )
	{
		return lookToX( forward, cross( axis::Z<vec4<T>>, forward ) );
	}

	/**
	 * @brief Generates a rotation matrix that rotates around an axis
	 * @param axis - axis of the rotation (normalized)
	 * @param angle - angle of the rotation [rad]
	 * @return rotation matrix
	*/
	template<typename T>
	[[nodiscard]] static mat4<T> matrixRotationAround( const vec4<T>& axis, T angle )
	{
		T sin = std::sin( angle );
		T cos = std::cos( angle );
		vec4<T> tpose = ( ONES<T> - cos ) * axis;

		mat4<T> r;
		vf::simd::fmadd( r.x_axis, axis, vec4<T>::set1( tpose.x ), vec4<T>( cos, sin * axis.z, -sin * axis.y, ZERO<T> ) );
		vf::simd::fmadd( r.y_axis, axis, vec4<T>::set1( tpose.y ), vec4<T>( -sin * axis.z, cos, sin * axis.x, ZERO<T> ) );
		vf::simd::fmadd( r.z_axis, axis, vec4<T>::set1( tpose.z ), vec4<T>( sin * axis.y, -sin * axis.x, cos, ZERO<T> ) );
		r.origin = axis::W<vec4<T>>;
		return r;
	}

	/**
	 * @brief Generates a rotation matrix that rotates between two vectors
	 * @param from - initial vector (normalized)
	 * @param to - final vector (normalized)
	 * @return rotation matrix
	*/
	template<typename T>
	[[nodiscard]] static mat4<T> matrixRotationBetween( const vec4<T>& from, const vec4f& to )
	{
		T product = dot_3D( from, to );
		if( product > ONES<T> - EPSILON<T> ) return IDENTITY<mat4<T>>;
		if( product < NEGATIVE<T> + EPSILON<T> ) return matrixRotationAround( orthogonal( from ), PI<T> );

		vec4<T> axis = normalize( cross( from, to ) );
		T cos = dot( from, to );
		T sin = sqrt( ONES<T> - cos * cos );
		vec4<T> tpose = ( ONES<T> - cos ) * axis;

		mat4<T> r;
		vf::simd::fmadd( r.x_axis, axis, vec4<T>::set1( tpose.x ), vec4<T>( cos, sin * axis.z, -sin * axis.y, ZERO<T> ) );
		vf::simd::fmadd( r.y_axis, axis, vec4<T>::set1( tpose.y ), vec4<T>( -sin * axis.z, cos, sin * axis.x, ZERO<T> ) );
		vf::simd::fmadd( r.z_axis, axis, vec4<T>::set1( tpose.z ), vec4<T>( sin * axis.y, -sin * axis.x, cos, ZERO<T> ) );
		r.origin = axis::W<vec4<T>>;
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
	template<typename T>
	constexpr void setOrthographic( mat4<T>& mat, T left, T right, T bottom, T top, T zNear, T zFar )
	{
#ifdef CLAMP_PERSPECTIVE_ZERO_ONE
		mat[0,0] = TWO<T> / ( right - left );
		mat[1,1] = TWO<T> / ( bottom - top );
		mat[2,2] = ONES<T> / ( zNear - zFar );
		mat[0,3] = -( right + left ) / ( right - left );
		mat[1,3] = -( top + bottom ) / ( top - bottom );
		mat[2,3] = -zNear / ( zFar - zNear );
#else
		mat[0,0] = TWO<T> / ( right - left );
		mat[1,1] = TWO<T> / ( top - bottom );
		mat[2,2] = TWO<T> / ( zNear - zFar );
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
	template<typename T>
	static void setPerspective( mat4<T>& mat, T fovy, T aspect, T zNear, T zFar )
	{
#ifdef CLAMP_PERSPECTIVE_ZERO_ONE
		float tanHalfFovy = tanf( fovy / TWO<T> );
		mat[0,0] = ONES<T> / ( aspect * tanHalfFovy );
		mat[1,1] = NEGATIVE<T> / tanHalfFovy;
		mat[2,2] = zFar / ( zNear - zFar );
		mat[3,2] = NEGATIVE<T>;
		mat[2,3] = -( zFar * zNear ) / ( zFar - zNear );
		mat[3,3] = ZERO<T>;
#else
		float tanHalfFovy = tanf( fovy / TWO<T> );
		mat[0,0] = ONES<T> / ( aspect * tanHalfFovy );
		mat[1,1] = ONES<T> / tanHalfFovy;
		mat[2,2] = -( zFar + zNear ) / ( zFar - zNear );
		mat[3,2] = NEGATIVE<T>;
		mat[2,3] = -( TWO<T> * zFar * zNear ) / ( zFar - zNear );
		mat[3,3] = ZERO<T>;
#endif
	}

	/**
	 * @brief Creates a copy of the matrix translated
	 * @param matrix - source transform
	 * @param translation - displacement vector
	 * @return translated transform
	*/
	template<typename T>
	constexpr mat4<T> translate( const mat4<T>& matrix, const mat4<T>& translation )
	{
		mat4<T> copy = matrix;
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
		template<typename T>
		constexpr mat4<T> translation( const vec4<T>& trans )
		{
			mat4<T> m = Math::IDENTITY<mat4<T>>;
			setTranslation( m, trans );
			return m;
		}

		/**
		 * @brief Creates a 2D rotation matrix.
		 * @param rot - rotation angle [rad]
		 * @return rotation matrix
		*/
		template<typename T> requires std::floating_point<T>
		static mat2<T> rotation( T rot )
		{
			mat2<T> m;
			setRotation( m, rot );
			return m;
		}

		/**
		 * @brief Creates a 3D rotation matrix.
		 * @param rot - rotation euler angles [rad]
		 * @return rotation matrix
		*/
		template<typename T> requires std::floating_point<T>
		static mat4<T> rotation( const euler<T>& rot )
		{
			mat4<T> m = Math::IDENTITY<mat4<T>>;
			setRotation( m, rot );
			return m;
		}

		/**
		 * @brief Creates a 3D rotation matrix.
		 * @param rot - rotation quaternion
		 * @return rotation matrix
		*/
		template<typename T> requires std::floating_point<T>
		constexpr mat4<T> rotation( const quat<T>& q )
		{
			mat4<T> m = Math::IDENTITY<mat4<T>>;
			setRotation( m, q );
			return m;
		}

		/**
		 * @brief Creates a scale matrix.
		 * @param sc - scale vector
		 * @return scale matrix
		*/
		template<size_t N, typename T>
		constexpr mat<N, N, T> scale( const vec<N, T>& sc )
		{
			mat<N, N, T> m = Math::ZERO<mat<N, N, T>>;
			for( size_t i = 0LLU; i < N; ++i )
			{
				m[i, i] = sc[i];
			}
			return m;
		}

		/**
		 * @brief Creates a 2D transform matrix.
		 * @param rotation - rotation angle [rad]
		 * @param scale - scale vector
		 * @return transform matrix
		*/
		template<typename T> requires std::floating_point<T>
		static mat2<T> transform( const T rotation, const vec2<T> scale )
		{
			mat2<T> m;
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
		template<typename T> requires std::floating_point<T>
		static mat4<T> transform( const vec4<T> translation, const euler<T>& rotation, const vec4<T>& scale )
		{
			mat4<T> m = Math::IDENTITY<mat4<T>>;
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
		template<typename T> requires std::floating_point<T>
		constexpr mat4<T> transform( const vec4<T> translation, const quat<T>& rotation, const vec4<T>& scale )
		{
			mat4<T> m = Math::IDENTITY<mat4<T>>;
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
		static mat4f view( const vec4f& position, const vec4f& center, const vec4f& up )
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
	[[nodiscard]] static mat4f random( const mat4f& min, const mat4f& max )
	{
		return {
			random( min.cols[0], max.cols[0] ),
			random( min.cols[1], max.cols[1] ),
			random( min.cols[2], max.cols[2] ),
			random( min.cols[3], max.cols[3] )
		};
	}

	/**
	 * @brief Generates a random rotation matrix
	 * @return random rotation matrix
	*/
	template<> [[nodiscard]] static mat4f randomRotation()
	{
		return matrixRotationAround( randomDirection<vec4f>(), rand() * RAND_CONVERT_TAU );
	}

	/**
	 * @brief Generates a random transformation matrix
	 * @param minTrans - minimum translation bounds
	 * @param maxTrans - maximum translation bounds
	 * @param minScale - minimum scaling bounds 
	 * @param maxScale - maximum scaling bounds 
	 * @return random transform matrix
	*/
	static mat4f randomTransform( const vec4f& minTrans = ZERO<vec4f>, const vec4f& maxTrans = ZERO<vec4f>, const vec4f& minScale = ONES<vec4f>, const vec4f& maxScale = ONES<vec4f> )
	{
		vec4f translation = Math::random( minTrans, maxTrans );
		eulerf rotation = Math::random( ZERO<eulerf>, TWO_PI<eulerf> );
		vec4f scale = Math::random( minScale, maxScale );
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
	static mat4f randomTransformAxes( const vec4f& minTrans = ZERO<vec4f>, const vec4f& maxTrans = ZERO<vec4f>, const vec4f& minScale = ONES<vec4f>, const vec4f& maxScale = ONES<vec4f> )
	{
		vec4f origin = Math::random( minTrans, maxTrans );
		return {
			Math::randomDirection<vec4f>() * Math::random( minScale.x, maxScale.x ),
			Math::randomDirection<vec4f>() * Math::random( minScale.y, maxScale.y ),
			Math::randomDirection<vec4f>() * Math::random( minScale.z, maxScale.z ),
			vec4f{ origin.x, origin.y, origin.z, 1.0f }
		};
	}
};

#endif
