#pragma once
#ifndef VF_MATH_FORMAT_HPP
#define VF_MATH_FORMAT_HPP

#include "vec4f.hpp"
#include "vec4i.hpp"
#include "mat2x2.hpp"
#include "mat4x4.hpp"
#include "euler.hpp"
#include "quaternion.hpp"
#include <iostream>
#include <format>
#include <string>

namespace vf
{
	static std::string toString( int value )
	{
		return std::to_string( value );
	}

	static std::string toString( float value )
	{
		return std::format( "{:.3f}", value );
	}

	static std::string toString( const vec2i& vector )
	{
		return std::format( "{}, {}", vector.x, vector.y );
	}

	static std::string toString( const vec2f& vector )
	{
		return std::format( "{:.3f}, {:.3f}", vector.x, vector.y );
	}

	static std::string toString( const vec4i& vector )
	{
		return std::format( "{}, {}, {}, {}", vector.x, vector.y, vector.z, vector.w );
	}

	static std::string toString( const vec4f& vector )
	{
		return std::format( "{:.3f}, {:.3f}, {:.3f}, {:.3f}", vector.x, vector.y, vector.z, vector.w );
	}

	static std::string toString( const euler& euler )
	{
		return std::format( "{:.3f}, {:.3f}, {:.3f}", euler.rx, euler.ry, euler.rz );
	}

	static std::string toString( const quat& q )
	{
		return std::format( "{:.3f}, {:.3f}, {:.3f}, {:.3f}", q.x, q.y, q.z, q.w );
	}

	static std::string toString( const mat2f& matrix )
	{
		return std::format( "{:.3f}, {:.3f}\n{:.3f}, {:.3f}", matrix[0], matrix[2], matrix[1], matrix[3] );
	}

	static std::string toString( const mat4f& matrix )
	{
		return std::format( "\n{:.3f}, {:.3f}, {:.3f}, {:.3f}\n{:.3f}, {:.3f}, {:.3f}, {:.3f}\n{:.3f}, {:.3f}, {:.3f}, {:.3f}\n{:.3f}, {:.3f}, {:.3f}, {:.3f}", 
			matrix[0], matrix[4], matrix[8], matrix[12],
			matrix[1], matrix[5], matrix[9], matrix[13],
			matrix[2], matrix[6], matrix[10], matrix[14],
			matrix[3], matrix[7], matrix[11], matrix[15] );
	}
}

static std::ostream& operator<<( std::ostream& os, const vec2i& vector )
{
	return os << vf::toString( vector );
}

static std::ostream& operator<<( std::ostream& os, const vec2f& vector )
{
	return os << vf::toString( vector );
}

static std::ostream& operator<<( std::ostream& os, const vec4f& vector )
{
	return os << vf::toString( vector );
}

static std::ostream& operator<<( std::ostream& os, const vec4i& vector )
{
	return os << vf::toString( vector );
}

static std::ostream& operator<<( std::ostream& os, const euler& euler )
{
	return os << vf::toString( euler );
}

static std::ostream& operator<<( std::ostream& os, const quat& q )
{
	return os << vf::toString( q );
}

static std::ostream& operator<<( std::ostream& os, const mat2f& matrix )
{
	return os << vf::toString( matrix );
}

static std::ostream& operator<<( std::ostream& os, const mat4f& matrix )
{
	return os << vf::toString( matrix );
}

#endif