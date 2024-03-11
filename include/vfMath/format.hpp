#pragma once
#ifndef VF_MATH_FORMAT_HPP
#define VF_MATH_FORMAT_HPP

#include "vec4f.hpp"
#include "vec4i.hpp"
#include "mat2x2.hpp"
#include "mat4x4.hpp"
#include "euler.hpp"
#include "quaternion.hpp"
#include "Operations.hpp"

#include <iostream>
#include <format>
#include <string>

namespace vf
{
	#define TO_STRING_CASE( x ) case x: return #x##s

	constexpr std::string toString( ScalarOperation op )
	{
		using namespace std::literals::string_literals;
		using enum ScalarOperation;
		switch( op )
		{
			default:	return "UNKNOWN"s;
				TO_STRING_CASE( KEEP );
				TO_STRING_CASE( SET );
				TO_STRING_CASE( ADD );
				TO_STRING_CASE( SUB );
				TO_STRING_CASE( MUL );
				TO_STRING_CASE( DIV );
				TO_STRING_CASE( MOD );
		}
	}

	constexpr std::string toString( VectorOperation op )
	{
		using namespace std::literals::string_literals;
		using enum VectorOperation;
		switch( op )
		{
			default:	return "UNKNOWN"s;
				TO_STRING_CASE( KEEP );
				TO_STRING_CASE( SET );
				TO_STRING_CASE( ADD );
				TO_STRING_CASE( SUB );
				TO_STRING_CASE( MUL );
				TO_STRING_CASE( DIV );
				TO_STRING_CASE( MOD );

				TO_STRING_CASE( PROJECT );
				TO_STRING_CASE( REFLECT );
				TO_STRING_CASE( ORTHOGONAL );
				TO_STRING_CASE( SET_PARALLEL );
		}
	}
}

template <class CharT>
struct std::formatter<vec2f, CharT>
{
	constexpr auto parse( std::format_parse_context& ctx )
	{
		return ctx.begin();
	}

	template <typename Context>
	constexpr auto format( const vec2f& v, Context& ctx ) const
	{
		return std::format_to( ctx.out(), "[ {:.3f}, {:.3f} ]", v.x, v.y );
	}
};

template <class CharT>
struct std::formatter<vec4f, CharT>
{
	constexpr auto parse( std::format_parse_context& ctx )
	{
		return ctx.begin();
	}

	template <typename Context>
	constexpr auto format( const vec4f& v, Context& ctx ) const
	{
		return std::format_to( ctx.out(), "[ {:.3f}, {:.3f}, {:.3f}, {:.3f} ]", v.x, v.y, v.z, v.w );
	}
};

template <class CharT>
struct std::formatter<vec2i, CharT>
{
	constexpr auto parse( std::format_parse_context& ctx )
	{
		return ctx.begin();
	}

	template <typename Context>
	constexpr auto format( const vec2i& v, Context& ctx ) const
	{
		return std::format_to( ctx.out(), "[ {}, {} ]", v.x, v.y );
	}
};

template <class CharT>
struct std::formatter<vec4i, CharT>
{
	constexpr auto parse( std::format_parse_context& ctx )
	{
		return ctx.begin();
	}

	template <typename Context>
	constexpr auto format( const vec4i& v, Context& ctx ) const
	{
		return std::format_to( ctx.out(), "[ {}, {}, {}, {} ]", v.x, v.y, v.z, v.w );
	}
};

template <class CharT>
struct std::formatter<euler, CharT>
{
	constexpr auto parse( std::format_parse_context& ctx )
	{
		return ctx.begin();
	}

	template <typename Context>
	constexpr auto format( const euler& e, Context& ctx ) const
	{
		return std::format_to( ctx.out(), "[ {:.3f}, {:.3f}, {:.3f} ]", e.rx, e.ry, e.rz );
	}
};

template <class CharT>
struct std::formatter<quat, CharT>
{
	constexpr auto parse( std::format_parse_context& ctx )
	{
		return ctx.begin();
	}

	template <typename Context>
	constexpr auto format( const quat& q, Context& ctx ) const
	{
		return std::format_to( ctx.out(), "[ {:.3f}, {:.3f}, {:.3f}, {:.3f} ]", q.x, q.y, q.z, q.w );
	}
};

template <class CharT>
struct std::formatter<mat2f, CharT>
{
	constexpr auto parse( std::format_parse_context& ctx )
	{
		return ctx.begin();
	}

	template <typename Context>
	constexpr auto format( const mat2f& matrix, Context& ctx ) const
	{
		return std::format_to( ctx.out(), "[ {:.3f}, {:.3f}\n{:.3f}, {:.3f} ]", matrix[0], matrix[2], matrix[1], matrix[3] );
	}
};

template <class CharT>
struct std::formatter<mat4f, CharT>
{
	constexpr auto parse( std::format_parse_context& ctx )
	{
		return ctx.begin();
	}

	template <typename Context>
	constexpr auto format( const mat4f& matrix, Context& ctx ) const
	{
		return std::format_to( ctx.out(), "[ {:.3f}, {:.3f}, {:.3f}, {:.3f}\n{:.3f}, {:.3f}, {:.3f}, {:.3f}\n{:.3f}, {:.3f}, {:.3f}, {:.3f}\n{:.3f}, {:.3f}, {:.3f}, {:.3f} ]", 
			matrix[0], matrix[4], matrix[8], matrix[12],
			matrix[1], matrix[5], matrix[9], matrix[13],
			matrix[2], matrix[6], matrix[10], matrix[14],
			matrix[3], matrix[7], matrix[11], matrix[15] );
	}
};

template <class CharT>
struct std::formatter<ScalarOperation, CharT>
{
	constexpr auto parse( std::format_parse_context& ctx )
	{
		return ctx.begin();
	}

	template <typename Context>
	constexpr auto format( const ScalarOperation& op, Context& ctx ) const
	{
		return std::format_to( ctx.out(), "{}", vf::toString( op ) );
	}
};

template <class CharT>
struct std::formatter<VectorOperation, CharT>
{
	constexpr auto parse( std::format_parse_context& ctx )
	{
		return ctx.begin();
	}

	template <typename Context>
	constexpr auto format( const VectorOperation& op, Context& ctx ) const
	{
		return std::format_to( ctx.out(), "{}", vf::toString( op ) );
	}
};

static std::ostream& operator<<( std::ostream& os, const vec2i& vector )
{
	return os << std::format( "{}", vector );
}

static std::ostream& operator<<( std::ostream& os, const vec2f& vector )
{
	return os << std::format( "{}", vector );
}

static std::ostream& operator<<( std::ostream& os, const vec4f& vector )
{
	return os << std::format( "{}", vector );
}

static std::ostream& operator<<( std::ostream& os, const vec4i& vector )
{
	return os << std::format( "{}", vector );
}

static std::ostream& operator<<( std::ostream& os, const euler& euler )
{
	return os << std::format( "{}", euler );
}

static std::ostream& operator<<( std::ostream& os, const quat& q )
{
	return os << std::format( "{}", q );
}

static std::ostream& operator<<( std::ostream& os, const mat2f& matrix )
{
	return os << std::format( "{}", matrix );
}

static std::ostream& operator<<( std::ostream& os, const mat4f& matrix )
{
	return os << std::format( "{}", matrix );
}

#endif