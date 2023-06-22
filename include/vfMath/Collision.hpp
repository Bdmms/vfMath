#pragma once
#ifndef WB_COLLISION_HPP
#define WB_COLLISION_HPP

#include "MatrixMath.hpp"
#include <vector>
#include <list>

/**
 * @brief Enum of supported collider types
*/
enum class ColliderType : unsigned char
{
	Cube = 0,
	Sphere = 1
};

/**
 * @brief Pair of transform matrices that define transformations into and out of a space.
*/
struct TransformSpace
{
	mat4x4 transform;
	mat4x4 inverse;
};

/**
 * @brief Pair of values that form a range.
*/
template <typename T>
struct Bounds
{
	T min;
	T max;
};

/**
 * @brief Source object of the collision
*/
struct CollisionObject
{
	uint64_t type;
	void* data;
};

typedef TransformSpace CollisionFace;
typedef void ( *CollisionCallback )( const CollisionObject& a, const CollisionObject& b );

/**
 * @brief Collider containing references to volume and data stored in the collision layer
*/
struct Collider
{
	TransformSpace bounds;
	Bounds<vec3f> aabb;
	ColliderType type;
};

/**
 * @brief Binding of collider and its handler 
*/
struct CollisionBinding
{
	const Collider& collider;
	CollisionCallback handler;
	CollisionObject object;
};

namespace Math
{
	/**
	 * @brief Translates the regular and inverse transforms of the space.
	 * @param space - transform space
	 * @param translation - translation vector
	*/
	static void translate( TransformSpace& space, const vec3f& translation )
	{
		space.transform.origin += translation;
		space.inverse.origin -= space.inverse * translation;
	}

	/**
	 * @brief Tests if two bounds overlap with each other across each axis
	 * @param a - first bound range
	 * @param b - second bound range
	 * @return Whether the boundaries overlap
	*/
	template <typename T>
	static bool overlaps( const Bounds<T>& a, const Bounds<T>& b )
	{
		return overlaps( a.min, a.max, b.min, b.max );
	}

	/**
	 * @brief Extends the bounds, such that the face is completley inside the bounds.
	 * @param bounds - bounds range
	 * @param face - triangle transform
	*/
	static void extend( Bounds<vec3f>& bounds, const CollisionFace& face )
	{
		const vec4f& p0 = face.transform.origin;
		const vec4f p1 = p0 + face.transform.x_axis;
		const vec4f p2 = p0 + face.transform.y_axis;

		bounds.min = Math::min( bounds.min, p0 );
		bounds.min = Math::min( bounds.min, p1 );
		bounds.min = Math::min( bounds.min, p2 );
		bounds.max = Math::max( bounds.max, p0 );
		bounds.max = Math::max( bounds.max, p1 );
		bounds.max = Math::max( bounds.max, p2 );
	}

	/**
	 * @brief Extends the bounds, such that all faces are completley inside the bounds.
	 * @param bounds - bounds range
	 * @param faces - iterable set of faces
	*/
	template <typename IterableFaces>
	static void extend( Bounds<vec3f>& bounds, const IterableFaces& faces )
	{
		for( const CollisionFace& face : faces )
		{
			extend( bounds, face );
		}
	}

	/*
	 * @brief Creates a transform matrix that represents the 3-dimensional bounds.
	 * @param bounds - axis-aligned bound range
	 * @return transform matrix
	*/
	static mat4x4 createBoundingTransform( const Bounds<vec3f>& bounds )
	{
		vec3f scale = ( bounds.max - bounds.min ) * 0.5f;
		vec3f translation = ( bounds.max + bounds.min ) * 0.5f;
		return { { scale.x, 0.0f, 0.0f }, { 0.0f, scale.y, 0.0f }, { 0.0f, 0.0f, scale.z }, translation };
	}

	/*
	 * @brief Creates an inverse transform matrix that represents the 3-dimensional bounds.
	 * @param bounds - axis-aligned bound range
	 * @return inverse transform matrix
	*/
	static mat4x4 createBoundingInverseTransform( const Bounds<vec3f>& bounds )
	{
		vec3f scale = 2.0f / ( bounds.max - bounds.min );
		vec3f translation = ( bounds.max + bounds.min ) * ( -0.5f * scale );
		translation.w = 1.0f;
		return { { scale.x, 0.0f, 0.0f }, { 0.0f, scale.y, 0.0f }, { 0.0f, 0.0f, scale.z }, translation };
	}

	namespace Box
	{
		/**
		 * @brief Calculates the Axis Aligned Bounding Box (AABB) that fits the arbitrary box
		 * @param transform - transform of the box in the axis aligned space
		 * @return bounding vector pair that represents the AABB
		*/
		Bounds<vec3f> calculateAABB( const mat4f& transform );

		/**
		 * @brief Tests if the unit cube contains the point.
		 * @param point - point vector
		 * @return Whether the point is inside the unit cube
		*/
		bool pointTest( const vec4f& point );

		/**
		 * @brief Tests if the finite ray intersects with the unit cube.
		 * @param rayOrigin - origin point of the ray
		 * @param rayVector - displacement vector to the end of the ray
		 * @return Whether the ray intersects the unit cube
		*/
		bool rayTest( const vec4f& rayOrigin, const vec3f& rayVector );

		/**
		 * @brief Tests if the triangle intersects with the unit cube.
		 * @param triangle - transform space of triangle
		 * @return Whether the triangle intersects the unit cube
		*/
		bool triangleTest( const TransformSpace& triangle );

		/**
		 * @brief Tests if the arbitrary box intersects with the unit cube.
		 * @param box - transform space of box
		 * @return Whether the box intersects the unit cube
		*/
		bool boxTest( const TransformSpace& box );
	}

	namespace Sphere
	{
		/**
		 * @brief Tests if the arbitrary sphere intersects with the unit sphere.
		 * @param sphere - transform space of sphere
		 * @return Whether the sphere intersects the unit sphere
		*/
		bool sphereTest( const TransformSpace& sphere );

		/**
		 * @brief Tests if the arbitrary box intersects with the unit sphere.
		 * @param box - transform space of box
		 * @return Whether the box intersects the unit sphere
		*/
		bool boxTest( const TransformSpace& box );
	}

	namespace Triangle
	{
		/**
		 * @brief Calculates the Axis Aligned Bounding Box (AABB) that fits the triangle
		 * @param transform - transform of the triangle in the axis aligned space
		 * @return bounding vector pair that represents the AABB
		*/
		Bounds<vec3f> calculateAABB( const mat4f& transform );

		/**
		 * @brief Calculates the intersecting distance of the ray against the triangle.
		 * @param triangle - transform of triangle
		 * @param rayOrigin - origin point of the ray
		 * @param rayVector - displacement vector to the end of the ray
		 * @return Whether the triangle intersects the unit cube
		*/
		float rayDistance( const mat4f& triangle, const vec4f& rayOrigin, const vec3f& rayVector );
	}
}

/**
 * @brief Generalizes handling of collision between colliders of different types
*/
namespace Collision
{
	std::vector<CollisionFace> createCubeMesh();
}

struct CollisionUnit
{
	std::vector<CollisionFace> faces;

	void collision( TransformSpace& relative ) const;
	bool rayCast( const vec4f& point, const vec3f& ray, float& distance, vec3f& normal ) const;
};

struct CollisionLattice
{
	TransformSpace bounds;
	TransformSpace unitSpace;
	Bounds<vec3f> aabb;
	vec3i dimensions;
	size_t length;
	CollisionUnit* units;

	CollisionLattice( const Bounds<vec3f>& bounds, const vec3i& dimensions );

	// Copy Constructor
	constexpr CollisionLattice( const CollisionLattice& copy ) :
		bounds( copy.bounds ),
		unitSpace( copy.unitSpace ),
		dimensions( copy.dimensions ),
		aabb( copy.aabb ),
		length( copy.length ),
		units( new CollisionUnit[length] )
	{
		std::copy_n( copy.units, copy.length, units );
	}

	// Move Constructor
	constexpr CollisionLattice( CollisionLattice&& copy ) noexcept :
		bounds( std::move( copy.bounds ) ),
		unitSpace( std::move( copy.unitSpace ) ),
		dimensions( std::move( copy.dimensions ) ),
		aabb( std::move( copy.aabb ) ),
		length( copy.length ),
		units( std::exchange( copy.units, nullptr ) )
	{

	}

	constexpr ~CollisionLattice()
	{
		if( units ) delete[] units;
	}

	void addTriangle( const TransformSpace& triangle );

	void collision( TransformSpace& sphere ) const;
	bool rayCast( const vec4f& point, const vec3f& ray, float& distance, vec3f& normal ) const;

	constexpr const CollisionUnit& getUnit( const vec4i& idx ) const
	{
		size_t iz = size_t( idx.u_z ) * size_t( dimensions.u_y );
		size_t iy = size_t( iz + idx.u_y ) * size_t( dimensions.u_x );
		return units[size_t( iy + idx.u_x )];
	}

	constexpr CollisionUnit* data() { return units; }
	constexpr CollisionUnit* begin() { return units; }
	constexpr CollisionUnit* end() { return units + length; }

	constexpr const CollisionUnit* data() const { return units; }
	constexpr const CollisionUnit* begin() const { return units; }
	constexpr const CollisionUnit* end() const { return units + length; }
	constexpr size_t size() const { return length; }

	constexpr size_t getMaxTriangleCount() const
	{
		size_t maxCount = 0u;
		for( size_t i = 0; i < length; ++i )
		{
			maxCount = std::max( maxCount, units[i].faces.size() );
		}
		return maxCount;
	}

	constexpr size_t getTotalTriangleCount() const
	{
		size_t totalCount = 0u;
		for( size_t i = 0; i < length; ++i )
		{
			totalCount += units[i].faces.size();
		}
		return totalCount;
	}

	constexpr size_t getAvgTriangleCount() const
	{
		return getTotalTriangleCount() / length;
	}
};

struct CollisionMesh
{
	TransformSpace bounds;
	Bounds<vec3f> aabb;
	std::vector<CollisionFace> faces;

	CollisionMesh( const std::vector<CollisionFace>& source );

	CollisionMesh( const mat4f& transform ) :
		bounds{ transform, transform.inverse() },
		aabb( Math::Box::calculateAABB( transform ) )
	{

	}

	CollisionMesh( const mat4f& transform, const CollisionFace* faceArr, size_t numFaces ) :
		bounds{ transform, transform.inverse() },
		aabb( Math::Box::calculateAABB( transform ) )
	{
		faces.reserve( numFaces );
		for( size_t i = 0LLU; i < numFaces; ++i )
		{
			faces.emplace_back( bounds.inverse * faceArr[i].transform, faceArr[i].inverse * bounds.transform );
		}
	}

	void collision( TransformSpace& relative ) const;
	bool rayCast( const vec4f& point, const vec3f& ray, float& distance, vec3f& normal ) const;
};

struct CollisionMap
{
	std::vector<CollisionLattice> fields;
	std::vector<CollisionMesh> meshes;

	CollisionMesh& addMesh( const std::vector<CollisionFace>& meshCollision );
	CollisionLattice& addLattice( const std::vector<CollisionFace>& meshCollision, const vec3i& dimensions );
	CollisionLattice& addLattice( const std::vector<std::vector<CollisionFace>>& meshCollision, const vec3i& dimensions );
	
	constexpr const CollisionUnit& getUnit( const vec4i& id ) const
	{
		return fields[id.w].getUnit( id );
	}

	vec4i getPointID( const vec4f& point ) const;

	void update();

	void collision( TransformSpace& sphere ) const;
	const TransformSpace* rayCast( const vec4f& point, const vec3f& ray, float& distance, vec3f& normal ) const;
};

struct CollisionLayer
{
	std::vector<CollisionBinding> objects;

	constexpr void bind( Collider& collider, CollisionCallback handler, void* source, uint64_t type )
	{
		objects.emplace_back( collider, handler, CollisionObject( type, source ) );
	}

	void testCollision();
};

#endif