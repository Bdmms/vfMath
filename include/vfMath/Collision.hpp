#pragma once
#ifndef WB_COLLISION_HPP
#define WB_COLLISION_HPP

#include "Transform3D.hpp"
#include <vector>
#include <list>
#include <mutex>

struct RaySensor;
struct Collider;
struct CollisionObject;

typedef void ( *CollisionCallback )( CollisionObject& a, CollisionObject& b );

/**
 * @brief Enum of supported collider types
*/
enum class ColliderType : uint8_t
{
	AABB = 0,		// Axis-aligned bounding box, which ignores transform
	Cube = 1,		// Unit cube with transformation + AABB
	Sphere = 2,		// Unit sphere with transformation + AABB
	Mesh = 3		// Treated as AABB, contains additional colliders that must be checked in handler
};

typedef TransformSpace CollisionFace;

/**
 * @brief Extension to math utilites for collision checking
*/
namespace Math
{
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
		vec3f translation = scale + bounds.min;
		translation.w = 1.0f;
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

	namespace AABB
	{
		/**
		 * @brief Tests if the finite ray intersects with the axis-aligned bounding box.
		 * @param aabb - axis-aligned bounding box
		 * @param rayOrigin - origin point of the ray
		 * @param rayVector - displacement vector to the end of the ray
		 * @return Whether the ray intersects the unit cube
		*/
		bool rayTest( const vec3f& min, const vec3f& max, const vec4f& rayOrigin, const vec3f& rayVector );

		/**
		 * @brief Calculates the entry and exit distances where the ray intersects with the axis-aligned bounding box.
		 * @param aabb - axis-aligned bounding box
		 * @param rayOrigin - origin point of the ray
		 * @param rayVector - displacement vector to the end of the ray
		 * @return minimum and maximum values along the ray vector
		*/
		Bounds<float> rayBounds( const vec3f& min, const vec3f& max, const vec4f& rayOrigin, const vec3f& rayVector );
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

		/**
		 * @brief Tests if the arbitrary box intersects with the unit cube.
		 * This differs from boxTest() by dividing projected points by their w coordinate.
		 * @param box - transform space of box
		 * @return Whether the box intersects the unit cube
		*/
		bool projectedBoxTest( const TransformSpace& box );

		/**
		 * @brief Calculates the displacement of an arbitrary sphere intersecting with the unit cube.
		 * @param displacement - write destination of displacement vector
		 * @param sphere - transform space of sphere
		 * @return Whether the sphere intersects the unit cube
		*/
		bool sphereCollision( vec3f& displacement, const TransformSpace& sphere );
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

		/**
		 * @brief Calculates the displacement of an arbitrary box intersecting with the unit sphere.
		 * @param displacement - write destination of displacement vector
		 * @param sphere - transform space of box
		 * @return Whether the box intersects the unit sphere
		*/
		bool boxCollision( vec3f& displacement, const TransformSpace& box );

		/**
		 * @brief Calculates the displacement of an arbitrary sphere intersecting with the unit sphere.
		 * @param displacement - write destination of displacement vector
		 * @param sphere - transform space of sphere
		 * @return Whether the sphere intersects the unit sphere
		*/
		bool sphereCollision( vec3f& displacement, const TransformSpace& sphere );
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

	static vec2f intersection2D( vec2f originA, vec2f lineA, vec2f originB, vec2f lineB )
	{
		// p + rt
		// q + su
		float dx = originB.x - originA.x;
		float dy = originB.y - originA.y;
		float dp = lineA.x * lineB.y - lineA.y * lineB.x;

		// t = ( q - p ) x s / ( r x s )
		float t = ( dx * lineB.y - dy * lineB.x ) / dp;
		// u = ( q - p ) x r / ( r x s )
		float u = ( dx * lineA.y - dy * lineA.x ) / dp;
		return { t, u };
	}

	static vec2f intersection3D( const vec4f& originA, const vec4f& lineA, const vec4f& originB, const vec4f& lineB )
	{
		vec3f normal = normalize( cross( lineA, lineB ) );
		vec3f tangent = normalize( lineA );
		vec3f bitangent = cross( normal, tangent );
		vec3f displacement = originB - originA;

		vec4f p = Math::parallel::dot( displacement, tangent, displacement, bitangent, lineB, tangent, lineB, bitangent );
		if( p.w <= Math::EPSILON<float> ) return { 0.0f, 0.0f };

		return { p.x - p.y * p.z / p.w, -p.y / p.w };
	}
}

/**
 * @brief Sensor used to retrieve distance along a ray
*/
struct RaySensor
{
	vec4f origin;
	vec3f direction;
	vec3f normal;
	const TransformSpace* space;
	float distance;
	bool hit;
};

/**
 * @brief Collider containing references to volume and data stored in the collision layer
*/
struct Collider
{
	enum ColliderFlags : uint32_t
	{
		ENABLED     = 0x00u,    // Default state for a collider

		// Input flags
		DISABLED    = 0x01u,    // Indicates the collider will not trigger collisions
		PASSIVE     = 0x02u,	// Collisions between passive colliders are ignored
		SWEEPING    = 0x04u,    // Enables checks for sweep collision between frames (TODO: not implemented)
		STATIC	    = 0x08u,    // Indicates that the collider cannot be transformed on collision

		// Output flags
		COLLIDED    = 0x10u,	// Indicates that at least one collision occurred with this collider
		TRANSLATED  = 0x20u,    // Indicates that the collider has translated during the collision test
		ROTATED     = 0x40u,    // Indicates that the collider has rotated during the collision test
		SELECTED    = 0x80u	    // Indicates that the collider is selected
	};

	TransformSpace current;
	TransformSpace previous;
	Bounds<vec3f> aabb;
	ColliderType type;
	uint32_t flags;

	constexpr Collider( ColliderType type = ColliderType::AABB ) :
		current( Math::IDENTITY<TransformSpace> ),
		aabb{ Math::MAX<vec3f>, Math::MIN<vec3f> },
		type( type ),
		flags( ENABLED )
	{

	}

	constexpr Collider( const TransformSpace& bounds, const Bounds<vec3f> aabb, ColliderType type ) :
		current( bounds ),
		aabb( aabb ),
		type( type ),
		flags( ENABLED )
	{

	}

	/**
	 * @brief Clears previous transformations recorded by the collider.
	*/
	constexpr void clearPrevious()
	{
		previous = current;
	}

	/**
	 * @brief Clears previous transforms, sets the current transform, and re-calculates the AABB.
	 * @param transform - transform matrix
	*/
	void setTransform( const mat4x4& transform )
	{
		current.transform = transform;
		current.inverse = transform.inverse();
		clearPrevious();
		aabb = Math::Box::calculateAABB( transform );
	}

	/**
	 * @brief Clears previous transforms, sets the current transform, and re-calculates the AABB.
	 * @param transform - transform space
	*/
	void setTransform( const TransformSpace& transformSpace )
	{
		current = transformSpace;
		clearPrevious();
		aabb = Math::Box::calculateAABB( transformSpace.transform );
	}

	/**
	 * @brief Replaces the collider's current transform and re-calculates the AABB.
	 * @param transform - transform matrix
	*/
	void overrideTransform( const mat4x4& transform )
	{
		current.transform = transform;
		current.inverse = transform.inverse();
		aabb = Math::Box::calculateAABB( transform );
	}

	/**
	 * @brief Adds the transform to the collider's queue and re-calculates the AABB.
	 * Previous transform is stored.
	 * @param transform - transform matrix
	*/
	void pushTransform( const mat4x4& transform )
	{
		previous = current;
		current.transform = transform;
		current.inverse = transform.inverse();
		aabb = Math::Box::calculateAABB( transform );
	}

	/**
	 * @brief Adds the transform to the collider's queue and re-calculates the AABB.
	 * Previous transform is stored.
	 * @param space - transform space
	*/
	void pushTransform( const TransformSpace& space )
	{
		previous = current;
		current = space;
		aabb = Math::Box::calculateAABB( current.transform );
	}

	/**
	 * @brief Tests for a ray collision on the collider.
	 * @param ray - ray parameters
	 * @return Whether the ray hit the collider
	*/
	virtual bool rayCast( RaySensor& ray ) const
	{
		// TODO: Add support for general colliders
		return false;
	}
};

/**
 * @brief Source object of the collision
*/
struct CollisionObject
{
	Collider* collider;
	uint64_t type;
	void* data;
};

/**
 * @brief Binding of collider and its handler
*/
struct CollisionBinding
{
	CollisionCallback handler;
	CollisionObject object;
};

struct ColliderSpace : public Collider
{
	constexpr ColliderSpace() :
		Collider{ { Math::IDENTITY<mat4f>, Math::IDENTITY<mat4f> }, { Math::MAX<vec3f>, Math::MIN<vec3f> }, ColliderType::Mesh }
	{

	}

	ColliderSpace( const mat4x4& transform ) : 
		Collider{ { transform, transform.inverse() }, Math::Box::calculateAABB( transform ), ColliderType::Mesh }
	{

	}

	ColliderSpace( const Bounds<vec3f>& aabb ) :
		Collider{ { Math::createBoundingTransform( aabb ), Math::createBoundingInverseTransform( aabb ) }, aabb, ColliderType::Mesh }
	{

	}

	virtual void collision( TransformSpace& sphere ) const = 0;
};

/**
 * @brief Defines a set of collision faces that can be tested against.
 * There are no optimizations performed, this object represents a raw unfiltered list of triangles.
*/
struct CollisionMesh : public ColliderSpace
{
	std::vector<CollisionFace> faces;

	CollisionMesh() : ColliderSpace()
	{

	}

	CollisionMesh( const std::vector<CollisionFace>& source );
	CollisionMesh( const mat4f& transform ) : ColliderSpace( transform )
	{

	}

	// TODO: Improve this interface
	void setFaces( const std::vector<CollisionFace>& source );

	virtual void collision( TransformSpace& relative ) const override;
	virtual bool rayCast( RaySensor& ray ) const override;
};

/**
 * @brief Defines a bounding box divided into multiple equally sized regions (units)
 * that optimizes the performance of checking for collisions against triangles.
*/
struct CollisionLattice : public ColliderSpace
{
	friend void packTriangles( CollisionLattice& lattice, const std::vector<std::vector<CollisionFace>>& unitFaces );
	friend void addTriangle( std::vector<std::vector<CollisionFace>>& unitFaces, const TransformSpace& triangle, const CollisionLattice& lattice );

	struct Unit
	{
		size_t idx;
		size_t size;
	};

private:
	TransformSpace unitSpace;
	vec3i dimensions;
	std::vector<CollisionFace> faces;
	std::vector<Unit> units;

	/**
	 * @brief Retrieves the unit that corresponds with the integer vector.
	 * TODO: This function does not check for index out of bounds, and it returns a unit that isn't helpful in other applications
	*/
	constexpr const Unit& getUnit( const vec4i& idx ) const
	{
		size_t iz = size_t( idx.u_z ) * size_t( dimensions.u_y );
		size_t iy = size_t( iz + idx.u_y ) * size_t( dimensions.u_x );
		return units[size_t( iy + idx.u_x )];
	}

public:
	// Default Constructor
	constexpr CollisionLattice() :
		ColliderSpace(),
		unitSpace( { Math::IDENTITY<mat4f>, Math::IDENTITY<mat4f> } ),
		dimensions( Math::ZERO<vec4i> )
	{

	}

	// Copy Constructor
	constexpr CollisionLattice( const CollisionLattice& copy ) : 
		ColliderSpace( copy ),
		unitSpace( copy.unitSpace ),
		dimensions( copy.dimensions ),
		faces( copy.faces ),
		units( copy.units )
	{

	}

	// Move Constructor
	constexpr CollisionLattice( CollisionLattice&& copy ) noexcept :
		ColliderSpace( copy ),
		unitSpace( std::move( copy.unitSpace ) ),
		dimensions( std::move( copy.dimensions ) ),
		faces( std::move( copy.faces ) ),
		units( std::move( copy.units ) )
	{

	}

	/**
	 * @brief Clears all existing data and resizes the lattice to alter the division of the bounding box.
	*/
	void resize( const vec3i newDimensions, size_t initialFaceCount = 0LLU )
	{
		dimensions = newDimensions;
		size_t unitCount = static_cast<size_t>( dimensions.x ) * static_cast<size_t>( dimensions.y ) * static_cast<size_t>( dimensions.z );

		// Re-compute unit space
		vec3f unitScale = 2.0f / static_cast<vec3f>( dimensions );
		mat4x4 unitConvert = { { unitScale.x, 0.0f, 0.0f }, { 0.0f, unitScale.y, 0.0f }, { 0.0f, 0.0f, unitScale.z }, vec4f{ -1.0f, -1.0f, -1.0f, 1.0f } };
		unitSpace.transform = unitConvert;
		unitSpace.inverse = unitConvert.inverse();

		units.clear();
		faces.clear();
		units.resize( unitCount );
		faces.resize( initialFaceCount );
	}

	/**
	 * @brief Clears all existing data and sets the dimensions to 0.
	*/
	constexpr void clear()
	{
		unitSpace = Math::IDENTITY<TransformSpace>;
		dimensions = Math::ZERO<vec3i>;
		units.clear();
		faces.clear();
	}

	/**
	 * @brief Generates the transform that defines the unit space.
	 * This transform will convert objects into a coordinate system where the integral part
	 * of a position directly corresponds to the region/unit the point intersects with.
	 * @return composite transform
	*/
	const TransformSpace getUnitSpace() const
	{
		return current * unitSpace;
	}

	/**
	 * @return number of units the box is split across each relative axis
	*/
	const vec3i getDimensions() const
	{
		return dimensions;
	}

	// Accessors to unit data
	Unit* getUnits() { return units.data(); }
	const Unit* getUnits() const { return units.data(); }
	size_t getUnitCount() const { return units.size(); }

	// Accessors to face data
	CollisionFace* getFaces() { return faces.data(); }
	const CollisionFace* getFaces() const { return faces.data(); }
	size_t getFaceCount() const { return faces.size(); }

	virtual void collision( TransformSpace& sphere ) const override;
	virtual bool rayCast( RaySensor& ray ) const override;
};

struct CollisionLayer
{
	std::vector<CollisionBinding> objects;
	std::vector<RaySensor*> sensors;
	std::mutex collisionLock;

	void testCollision();

	void bind( Collider& collider, CollisionCallback handler, void* source, uint64_t type );
	void unbind( const Collider& collider, CollisionCallback handler );
	void unbind( const Collider& collider );

	void addSensor( RaySensor& sensor );
	void removeSensor( RaySensor& sensor );

	static bool testCollision( const Collider& a, const Collider& b );
};

/**
 * @brief Utility dedicated to collision checking
*/
namespace Collision
{
	constexpr CollisionCallback DEFAULT_HANDLE = []( CollisionObject& a, CollisionObject& b ) {};

	std::vector<CollisionFace> createCubeMesh();

	void convertToLattice( CollisionLattice& lattice, const std::vector<CollisionFace>& mesh, const vec3i& dimensions );
	void convertToLattice( CollisionLattice& lattice, const std::vector<std::vector<CollisionFace>>& mesh, const vec3i& dimensions );
}

#endif