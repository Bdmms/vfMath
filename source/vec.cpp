#include "../include/vfMath/vec4f.hpp"
#include "../include/vfMath/vec4i.hpp"

//vec4f::operator vec4i() const
//{
//	return { _mm_cvtps_epi32( simd ) };
//}

//vec4i vf::cast_to_vec4i( vec4f vector )
//{
//	vec4i r;
//	r.simd = _mm_cvtps_epi32( vector.simd );
//	return r;
//}

//vec4i::operator vec4f() const
//{
//	vec4f r;
//	r.simd = _mm_cvtepi32_ps( simd );
//	return r;
//}

#include "../include/vfMath/mat2x2.hpp"
#include "../include/vfMath/mat4x4.hpp"

mat4x4::operator mat2x2() const
{
	return { m[0], m[1], m[4], m[5] };
}

mat2x2::operator mat4x4() const
{
	return { m[0], m[1], 0.0f, 0.0f, m[2], m[3], 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f };
}