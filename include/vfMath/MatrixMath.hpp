#pragma once
#ifndef VF_MATRIX_MATH_HPP
#define VF_MATRIX_MATH_HPP

#include "QuatMath.hpp"
#include "mat4x4.hpp"
#include "mat3x3.hpp"
#include "mat2x2.hpp"

typedef mat4x4 (*MatrixInterpolator)(const mat4x4& a, const mat4x4& b, const float weight);

/**
 * @brief Utilities for matrices
*/
namespace Math
{
	template<> inline constexpr static mat2x2 ZERO<mat2x2> = { 0, 0, 0, 0 };
	template<> inline constexpr static mat4x4 ZERO<mat4x4> = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	template<> inline constexpr static mat2x2 ONES<mat2x2> = { 1, 1, 1, 1 };
	template<> inline constexpr static mat4x4 ONES<mat4x4> = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };

	template<> inline constexpr static mat2x2 IDENTITY<mat2x2> = { 1, 0, 1, 0 };
	template<> inline constexpr static mat4x4 IDENTITY<mat4x4> = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

	/**
	 * @brief Truncates a 4x4 matrix into a 3x3 matrix
	 * @param matrix - source matrix
	*/
	constexpr mat4x4 cast_mat3x3(const mat4x4& matrix)
	{
		return { matrix.x_axis, matrix.y_axis, matrix.z_axis, ZERO<vec4f> };
	}

	/**
	 * @brief Truncates a 4x4 matrix into a 3x3 matrix
	 * @param dst - destination matrix
	 * @param src - source matrix
	*/
	constexpr void copyMat3x3(mat4x4& dst, const mat4x4& src)
	{
		std::copy(src.m, src.m + 12, dst.m);
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
	 * @brief Converts rotation matrix into a quaternion
	 * @param rotation - rotation matrix
	 * @return quat rotation
	*/
	/*static quat rotationFrom(const mat4x4& matrix)
	{
		float hTheta = acosf( ( ( matrix.x_axis.x + matrix.y_axis.y + matrix.z_axis.z ) - 1.0f ) / 2.0f ) / 2.0f;
		vec3f eigen = Math::ZERO<vec3f> * sinf(hTheta);
		return { eigen.x, eigen.y, eigen.z, cosf(hTheta) };
	}*/

	/**
	 * @brief Creates a copy of the matrix translated
	 * @param matrix - source transform
	 * @param translation - displacement vector
	 * @return translated transform
	*/
	static mat4x4 translate(const mat4x4& matrix, const vec3f& translation)
	{
		mat4x4 copy = matrix;
		copy.origin += translation;
		return copy;
	}

	/**
	 * @brief Creates a copy of the matrix scaled aroudn its origin
	 * @param matrix - source transform
	 * @param scale - scale vector
	 * @return scaled transform
	*/
	static mat4x4 scale(const mat4x4& matrix, const vec3f& scale)
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
	constexpr void setTranslation(mat4x4& matrix, const vec3f& translation)
	{
		matrix.m[12] = translation.x;
		matrix.m[13] = translation.y;
		matrix.m[14] = translation.z;
	}

	/**
	 * @brief Sets the rotation of the matrix (Overwrites the 3x3 matrix)
	 * @param matrix - transform matrix
	 * @param rotation - euler rotation vector
	*/
	static void setRotation(mat4x4& matrix, const euler& rotation)
	{
		vec3f s = Math::sin(rotation);
		vec3f c = Math::cos(rotation);
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
	static void setRotation(mat4x4& matrix, const quat& q)
	{
		//using namespace swizzle;
		/*matrix.simd[0] = _mm_fmadd_ps(_mm_xyzw_ps(-2.0f, 2.0f, 2.0f, 0.0f),
			_mm_add_ps(_mm_mul_ps(_mm_permute_ps(q.simd, YXX), _mm_permute_ps(q.simd, YYX)),
				_mm_mul_ps(_mm_xyzw_ps(q.z, q.w, -q.w, 0.0f), _mm_permute_ps(q.simd, ZZY))),
			_mm_xyzw_ps(1.0f, 0.0f, 0.0f, 0.0f) );*/

		matrix.m[0]  = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
		matrix.m[1]  =        2.0f * (q.x * q.y + q.w * q.z);
		matrix.m[2]  =        2.0f * (q.x * q.z - q.w * q.y);

		matrix.m[4]  =        2.0f * (q.x * q.y - q.w * q.z);
		matrix.m[5]  = 1.0f - 2.0f * (q.x * q.x + q.z * q.z);
		matrix.m[6]  =        2.0f * (q.y * q.z + q.w * q.x);

		matrix.m[8]  =        2.0f * (q.x * q.z + q.w * q.y);
		matrix.m[9]  =        2.0f * (q.y * q.z - q.w * q.x);
		matrix.m[10] = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
	}

	/**
	 * @brief Sets the inverse rotation of the matrix (Overwrites the 3x3 matrix)
	 * @param matrix - transform matrix
	 * @param rotation - euler rotation vector
	*/
	static void setRotationInverse(mat4x4& matrix, const euler& rotation)
	{
		vec3f s = Math::sin(rotation);
		vec3f c = Math::cos(rotation);
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
	constexpr void setRotationInverse(mat4x4& matrix, const quat& q)
	{
		matrix.m[0] = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
		matrix.m[4] = 2.0f * (q.x * q.y + q.w * q.z);
		matrix.m[8] = 2.0f * (q.x * q.z - q.w * q.y);

		matrix.m[1] = 2.0f * (q.x * q.y - q.w * q.z);
		matrix.m[5] = 1.0f - 2.0f * (q.x * q.x + q.z * q.z);
		matrix.m[9] = 2.0f * (q.y * q.z + q.w * q.x);

		matrix.m[2] = 2.0f * (q.x * q.z + q.w * q.y);
		matrix.m[6] = 2.0f * (q.y * q.z - q.w * q.x);
		matrix.m[10] = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
	}

	/**
	 * @brief Sets the transform of the matrix
	 * [default]	= 139245000 ns (avg. 122487180 ns)
	 * [parallel]	= 240875400 ns (avg. 223455020 ns)
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param rotation - rotation vector
	 * @param scale - scaling vector
	*/
	static void setTransform(mat4x4& matrix, const vec3f translation, const euler& rotation, const vec3f& scale)
	{
		setRotation(matrix, rotation);
		matrix.col[0] *= scale.x;
		matrix.col[1] *= scale.y;
		matrix.col[2] *= scale.z;
		setTranslation(matrix, translation);
	}

	/**
	 * @brief Sets the transform of the matrix
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param q - quaternion rotation
	 * @param scale - scaling vector
	*/
	static void setTransform(mat4x4& matrix, const vec3f translation, const quat& q, const vec3f& scale)
	{
		setRotation(matrix, q);
		matrix.col[0] *= scale.x;
		matrix.col[1] *= scale.y;
		matrix.col[2] *= scale.z;
		setTranslation(matrix, translation);
	}

	/**
	 * @brief Sets the inverse transform of the matrix
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param rotation - rotation vector
	 * @param scale - scaling vector
	*/
	static void setTransformInverse(mat4x4& matrix, const vec3f translation, const euler& rotation, const vec3f& scale)
	{
		setRotationInverse(matrix, rotation);
		matrix.col[0] /= scale;
		matrix.col[1] /= scale;
		matrix.col[2] /= scale;
		matrix.m[12] = -dot(matrix.col[0], translation);
		matrix.m[13] = -dot(matrix.col[1], translation);
		matrix.m[14] = -dot(matrix.col[2], translation);
	}

	/**
	 * @brief Sets the inverse transform of the matrix
	 * @param matrix - transform matrix
	 * @param translation - translation vector
	 * @param q - quaternion rotation
	 * @param scale - scaling vector
	*/
	static void setTransformInverse(mat4x4& matrix, const vec3f& translation, const quat& q, const vec3f& scale)
	{
		setRotationInverse(matrix, q);
		matrix.col[0] /= scale;
		matrix.col[1] /= scale;
		matrix.col[2] /= scale;
		matrix.m[12] = -dot(matrix.col[0], translation);
		matrix.m[13] = -dot(matrix.col[1], translation);
		matrix.m[14] = -dot(matrix.col[2], translation);
	}

	/*
	static void decompose(const mat4x4& matrix, vec4f& position, quat& rotation, vec3f& scale)
	{
		position = matrix.origin;
		scale = Math::parallel::length(matrix.x_axis, matrix.y_axis, matrix.z_axis);

		vec3f xAxis = matrix.x_axis / scale.x;
		vec3f yAxis = matrix.y_axis / scale.y;
		vec3f zAxis = matrix.z_axis / scale.z;

		// TODO:
	}*/

	/**
	 * @brief Generates a view matrix
	 * @param mat - view matrix
	 * @param eye - view position vector
	 * @param center - view target vector
	 * @param up - view up vector
	*/
	static void lookAt(mat4x4& mat, const vec4f& eye, const vec4f& center, const vec3f& up)
	{
		vec3f f = Math::normalize(eye - center);
		vec3f s = Math::normalize(Math::cross(up, f));
		vec3f u = Math::cross(f, s);
		mat.simd[0] = _mm_transpose_ps(s.simd, u.simd, f.simd, SIMD_4f_ZERO, swizzle::XXXX);
		mat.simd[1] = _mm_transpose_ps(s.simd, u.simd, f.simd, SIMD_4f_ZERO, swizzle::YYYY);
		mat.simd[2] = _mm_transpose_ps(s.simd, u.simd, f.simd, SIMD_4f_ZERO, swizzle::ZZZZ);
		mat.simd[3] = _mm_mul_ps(_mm_dot3v_ps(s.simd, eye.simd, u.simd, eye.simd, f.simd, eye.simd, SIMD_4f_W), _mm_xyzw_ps(-1.0f, -1.0f, -1.0f, 1.0f));
	}

	/**
	 * @brief Generates a rotation matrix that rotates the x-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @param trackAxis - axis to rotate around
	 * @param position - translation component (optional)
	 * @return rotation matrix
	*/
	[[nodiscard]] static mat4x4 lookToX(const vec3f& forward, const vec3f& trackAxis, const vec4f& position = Math::axis::W<vec4f>)
	{
		vec4f z_axis = Math::normalize(Math::cross(forward, trackAxis));
		vec4f y_axis = Math::cross(z_axis, forward);
		return mat4x4{ forward, y_axis, z_axis, position };
	}

	/**
	 * @brief Generates a rotation matrix that rotates the y-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @param trackAxis - axis to rotate around
	 * @param position - translation component (optional)
	 * @return rotation matrix
	*/
	[[nodiscard]] static mat4x4 lookToY(const vec3f& forward, const vec3f& trackAxis, const vec4f& position = Math::axis::W<vec4f>)
	{
		vec4f z_axis = Math::normalize(Math::cross(trackAxis, forward));
		vec4f x_axis = Math::cross(forward, z_axis);
		return mat4x4{ x_axis, forward, z_axis, position };
	}

	/**
	 * @brief Generates a rotation matrix that rotates the z-axis to point in a direction
	 * @param forward - forward orientation (normalized)
	 * @param trackAxis - axis to rotate around
	 * @param position - translation component (optional)
	 * @return rotation matrix
	*/
	[[nodiscard]] static mat4x4 lookToZ(const vec3f& forward, const vec3f& trackAxis, const vec4f& position = Math::axis::W<vec4f>)
	{
		vec4f x_axis = Math::normalize(Math::cross(trackAxis, forward));
		vec4f y_axis = Math::cross(forward, x_axis);
		return mat4x4{ x_axis, y_axis, forward, position };
	}

	/**
	 * @brief Generates a rotation matrix to point an axis in the specified direction
	 * @param direction - direction to rotate axis towards
	 * @param axisType - the axis to rotate
	 * @param position - translation component (optional)
	 * @return rotation matrix
	*/
	[[nodiscard]] static mat4x4 lookTo(const vec3f& direction, const Axis axisType, const vec4f& position = Math::axis::W<vec4f>)
	{
		switch (axisType)
		{
		case Axis::X: return lookToX(direction, Math::cross(Math::axis::X<vec4f>, direction), position);
		case Axis::Y: return lookToY(direction, Math::cross(Math::axis::Y<vec4f>, direction), position);
		case Axis::Z: return lookToZ(direction, Math::cross(Math::axis::Z<vec4f>, direction), position);
		default:	  return Math::IDENTITY<mat4x4>;
		}
	}

	/**
	 * @brief Generates a rotation matrix that rotates around an axis
	 * @param axis - axis of the rotation (normalized)
	 * @param angle - angle of the rotation
	 * @return rotation matrix
	*/
	template<> [[nodiscard]] static mat4x4 rotationAround(const vec3f& axis, const float angle)
	{
		float sin = sinf(angle);
		float cos = cosf(angle);
		vec3f tpose = (1.0f - cos) * axis;

		return {
			_mm_fmadd_ps(axis.simd, _mm_set1_ps(tpose.x), _mm_xyzw_ps(cos, sin * axis.z, -sin * axis.y, 0.0f)),
			_mm_fmadd_ps(axis.simd, _mm_set1_ps(tpose.y), _mm_xyzw_ps(-sin * axis.z, cos, sin * axis.x, 0.0f)),
			_mm_fmadd_ps(axis.simd, _mm_set1_ps(tpose.z), _mm_xyzw_ps(sin * axis.y, -sin * axis.x, cos, 0.0f)),
			SIMD_4f_W
		};
	}

	/**
	 * @brief Generates a rotation matrix that rotates between two vectors
	 * @param from - initial vector (normalized)
	 * @param to - final vector (normalized)
	 * @param axis - axis to rotate around (normalized)
	 * @return rotation matrix
	 * 
	 * TODO: Check if this functions works in general cases
	*/
	template<> [[nodiscard]] static mat4x4 rotationBetween(const vec3f& from, const vec3f& to, const vec3f& axis)
	{
		float cos = Math::dot(from, to);
		float sin = sqrtf(1.0f - cos * cos);
		vec3f tpose = (1.0f - cos) * axis;

		return {
			_mm_fmadd_ps(axis.simd, _mm_set1_ps(tpose.x), _mm_xyzw_ps(cos, sin * axis.z, -sin * axis.y, 0.0f)),
			_mm_fmadd_ps(axis.simd, _mm_set1_ps(tpose.y), _mm_xyzw_ps(-sin * axis.z, cos, sin * axis.x, 0.0f)),
			_mm_fmadd_ps(axis.simd, _mm_set1_ps(tpose.z), _mm_xyzw_ps(sin * axis.y, -sin * axis.x, cos, 0.0f)),
			SIMD_4f_W
		};
	}

	/**
	 * @brief Generates a rotation matrix that rotates between two vectors
	 * @param from - initial vector (normalized)
	 * @param to - final vector (normalized)
	 * @return rotation matrix
	*/
	template<> [[nodiscard]] static mat4x4 rotationBetween(const vec3f& from, const vec3f& to)
	{
		return rotationBetween<mat4x4>(from, to, Math::normalize(Math::cross(from, to)));
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
	static void setOrthographic(mat4x4& mat,
		const float left, const float right, const float bottom,
		const float top, const float zNear, const float zFar)
	{
		mat.m[0] = 2.0f / (right - left);
		mat.m[5] = 2.0f / (top - bottom);
		mat.m[10] = 2.0f / (zNear - zFar);
		mat.m[12] = -(right + left) / (right - left);
		mat.m[13] = -(top + bottom) / (top - bottom);
		mat.m[14] = -(zFar + zNear) / (zFar - zNear);
	}

	/**
	 * @brief Generates perspective matrix
	 * @param mat - projection matrix
	 * @param fovy - field of view
	 * @param aspect - aspect ratio
	 * @param zNear - near plane distance
	 * @param zFar - far plane distance
	*/
	static void setPerspective(mat4x4& mat, float fovy, float aspect, float zNear, float zFar)
	{
		float tanHalfFovy = tanf(fovy / 2.0f);
		mat.m[0] = 1.0f / (aspect * tanHalfFovy);
		mat.m[5] = 1.0f / tanHalfFovy;
		mat.m[10] = -(zFar + zNear) / (zFar - zNear);
		mat.m[11] = -1.0f;
		mat.m[14] = -(2.0f * zFar * zNear) / (zFar - zNear);
		mat.m[15] = 0.0f;
	}

	/**
	 * @brief Linear interpolation between two matrices
	 * @param a - first matrix
	 * @param b - second matrix
	 * @param t - weight
	 * @return weighted matrix
	*/
	template<> [[nodiscard]] static mat4x4 lerp(const mat4x4& a, const mat4x4& b, const float t)
	{
#if ENABLE_INSTRUCTIONS_AVX512
		return { _mm512_fmadd_ps(_mm512_sub_ps(b.avx512, a.avx512), _mm512_set1_ps(t), a.avx512) };
#elif ENABLE_INSTRUCTIONS_AVX2
		__m256 weight = _mm256_set1_ps(t);
		return {
			_mm256_fmadd_ps(_mm256_sub_ps(b.avx2[0], a.avx2[0]), weight, a.avx2[0]),
			_mm256_fmadd_ps(_mm256_sub_ps(b.avx2[1], a.avx2[1]), weight, a.avx2[1])
		};
#elif ENABLE_INSTRUCTIONS_SSE2
		__m128 weight = _mm_set1_ps(t);
		return {
			_mm_fmadd_ps(_mm_sub_ps(b.simd[0], a.simd[0]), weight, a.simd[0]),
			_mm_fmadd_ps(_mm_sub_ps(b.simd[1], a.simd[1]), weight, a.simd[1]),
			_mm_fmadd_ps(_mm_sub_ps(b.simd[2], a.simd[2]), weight, a.simd[2]),
			_mm_fmadd_ps(_mm_sub_ps(b.simd[3], a.simd[3]), weight, a.simd[3])
		};
#else
		return (b - a) * t + a;
#endif
	}

	/*
	[[nodiscard]] static mat4x4 slerp(const mat4x4& a, const mat4x4& b, const float t)
	{
		vec3f scaleA = Math::parallel::length(a.x_axis, a.y_axis, a.z_axis);
		vec3f scaleB = Math::parallel::length(b.x_axis, b.y_axis, b.z_axis);

		vec3f axisA = Math::normalize(a.x_axis + a.y_axis + a.z_axis);
		vec3f axisB = Math::normalize(b.x_axis + b.y_axis + b.z_axis);

		float hThetaA = acosf(((a.x_axis.x + a.y_axis.y + a.z_axis.z) - 1.0f) / 2.0f) / 2.0f;
		float hThetaB = acosf(((b.x_axis.x + b.y_axis.y + b.z_axis.z) - 1.0f) / 2.0f) / 2.0f;
		float hTheta = Math::lerp( hThetaA, hThetaB, t );

		vec3f trans = Math::lerp(a.origin, b.origin, t);
		vec3f scale = Math::lerp(scaleA, scaleB, t);
		vec3f eigen = Math::slerp(axisA, axisB, t) * sinf(hTheta);
		quat rotation = { eigen.x, eigen.y, eigen.z, cosf(hTheta) };

		mat4x4 xform = { Math::ZERO<vec3f>, Math::ZERO<vec3f>, Math::ZERO<vec3f>, trans };
		setRotation(xform, rotation);
		xform.x_axis *= scale.x;
		xform.y_axis *= scale.y;
		xform.z_axis *= scale.z;
		return xform;
	}*/

	/**
	 * @brief Utility for creating matrices
	*/
	namespace create
	{
		constexpr mat4x4 translation(const vec3f& trans)
		{
			mat4x4 m = Math::IDENTITY<mat4x4>;
			setTranslation(m, trans);
			return m;
		}

		static mat4x4 rotation(const euler& rot)
		{
			mat4x4 m = Math::IDENTITY<mat4x4>;
			setRotation(m, rot);
			return m;
		}

		static mat4x4 rotation(const quat& q)
		{
			mat4x4 m = Math::IDENTITY<mat4x4>;
			setRotation(m, q);
			return m;
		}

		constexpr mat4x4 scale(const vec3f& sc)
		{
			return mat4x4{ sc.x, 0.0f, 0.0f, 0.0f, 0.0f, sc.y, 0.0f, 0.0f, 0.0f, 0.0f, sc.z, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
		}

		static mat4x4 transform(const vec3f translation, const euler& rotation, const vec3f& scale)
		{
			mat4x4 m = Math::IDENTITY<mat4x4>;
			setTransform(m, translation, rotation, scale);
			return m;
		}

		static mat4x4 transform(const vec3f translation, const quat& rotation, const vec3f& scale)
		{
			mat4x4 m = Math::IDENTITY<mat4x4>;
			setTransform(m, translation, rotation, scale);
			return m;
		}

		static mat4x4 view(const vec3f& position, const vec3f& view, const vec3f& up)
		{
			mat4x4 m = Math::IDENTITY<mat4x4>;
			lookAt(m, position, view, up);
			return m;
		}

		static mat4x4 perspective(float fovy, float aspect, float zNear, float zFar)
		{
			mat4x4 m = Math::ZERO<mat4x4>;
			setPerspective(m, fovy, aspect, zNear, zFar);
			return m;
		}
	};

	/**
	 * @brief Generates a random matrix within the range
	 * @param min - minimum matrix
	 * @param max - maximum matrix
	 * @return random matrix
	*/
	template<> [[nodiscard]] static mat4x4 random(const mat4x4& min, const mat4x4& max)
	{
		return {
			Math::random<vec4f>(min.col[0], max.col[0]),
			Math::random<vec4f>(min.col[1], max.col[1]),
			Math::random<vec4f>(min.col[2], max.col[2]),
			Math::random<vec4f>(min.col[3], max.col[3])
		};
	}

	/**
	 * @brief Generates a random transformation matrix
	 * @param minTrans - minimum translation bounds
	 * @param maxTrans - maximum translation bounds
	 * @param minScale - minimum scaling bounds 
	 * @param maxScale - maximum scaling bounds 
	 * @return random transform matrix
	*/
	static mat4x4 randomTransform(const vec3f& minTrans = ZERO<vec4f>, const vec3f& maxTrans = ZERO<vec4f>, const vec3f& minScale = ONES<vec4f>, const vec3f& maxScale = ONES<vec4f>)
	{
		vec3f translation = Math::random(minTrans, maxTrans);
		vec3f rotation = Math::random(ZERO<vec4f>, TWO_PI<vec4f>);
		vec3f scale = Math::random(minScale, maxScale);
		return create::transform(translation, rotation, scale);
	}

	/**
	 * @brief Generates a random transformation matrix, includes shearing
	 * @param minTrans - minimum translation bounds
	 * @param maxTrans - maximum translation bounds
	 * @param minScale - minimum scaling bounds 
	 * @param maxScale - maximum scaling bounds 
	 * @return random transform matrix
	*/
	static mat4x4 randomTransformAxes(const vec3f& minTrans = ZERO<vec4f>, const vec3f& maxTrans = ZERO<vec4f>, const vec3f& minScale = ONES<vec4f>, const vec3f& maxScale = ONES<vec4f>)
	{
		vec4f origin = Math::random(minTrans, maxTrans);
		return {
			Math::randomDirection<vec3f>() * Math::random(minScale.x, maxScale.x),
			Math::randomDirection<vec3f>() * Math::random(minScale.y, maxScale.y),
			Math::randomDirection<vec3f>() * Math::random(minScale.z, maxScale.z),
			{ origin.x, origin.y, origin.z, 1.0f }
		};
	}
};

namespace Interpolate
{
	inline constexpr static MatrixInterpolator MATRIX_LINEAR = Math::lerp<mat4x4>;

	[[nodiscard]] inline static mat4x4 MATRIX_BEZIER(const mat4x4& a, const mat4x4& b, const float t)
	{
		return Math::lerp<mat4x4>(a, b, 3.0f * (1.0f - t) * t * t + t * t * t);
	}

	[[nodiscard]] inline static mat4x4 MATRIX_QUADRATIC(const mat4x4& a, const mat4x4& b, const float t)
	{
		return Math::lerp<mat4x4>(a, b, t * t);
	}

	[[nodiscard]] inline static mat4x4 TRANSFORM_LINEAR(const mat4x4& a, const mat4x4& b, const float t)
	{
		vec3f scaleA = Math::parallel::length(a.x_axis, a.y_axis, a.z_axis);
		vec3f scaleB = Math::parallel::length(b.x_axis, b.y_axis, b.z_axis);

		vec3f scale = Math::lerp(scaleA, scaleB, t);
		mat4x4 rawXform = Math::lerp(a, b, t);

		rawXform.x_axis = Math::normalize(rawXform.x_axis) * scale.x;
		rawXform.y_axis = Math::normalize(rawXform.y_axis) * scale.y;
		rawXform.z_axis = Math::normalize(rawXform.z_axis) * scale.z;

		return rawXform;
	}

	[[nodiscard]] inline static mat4x4 TRANSFORM_BEZIER(const mat4x4& a, const mat4x4& b, const float t)
	{
		return TRANSFORM_LINEAR(a, b, 3.0f * (1.0f - t) * t * t + t * t * t);
	}
}

#endif
