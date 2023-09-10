#pragma once
#ifndef WB_COLLISION_HPP
#define WB_COLLISION_HPP

#include "MatrixMath.hpp"
#include <vector>
#include <list>

struct RaySensor;
struct Collider;
struct CollisionObject;

typedef void ( *CollisionCallback )( CollisionObject& a, CollisionObject& b );

/**
 * @brief Enum of supported collider types
*/
enum class ColliderType : unsigned char
{
	AABB = 0,
	Cube = 1,
	Sphere = 2
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

typedef TransformSpace CollisionFace;

/**
 * @brief Extension to math utilites for collision checking
*/
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
	TransformSpace bounds;
	Bounds<vec3f> aabb;
	ColliderType type;

	constexpr Collider() :
		bounds{ Math::IDENTITY<mat4f>, Math::IDENTITY<mat4f> },
		aabb{ Math::MAX<vec3f>, Math::MIN<vec3f> },
		type( ColliderType::AABB )
	{

	}

	constexpr Collider( const TransformSpace& bounds, const Bounds<vec3f> aabb, ColliderType type ) :
		bounds( bounds ),
		aabb( aabb ),
		type( type )
	{

	}

	void setTransform( const mat4x4& transform )
	{
		bounds.transform = transform;
		bounds.inverse = transform.inverse();
		aabb = Math::Box::calculateAABB( transform );
	}

	virtual bool rayCast( RaySensor& ray ) const
	{
		return false;
	}
};

/**
 * @brief Source object of the collision
*/
struct CollisionObject
{
	const Collider& collider;
	const uint64_t type;
	void* data;
};

/**
 * @brief Utility dedicated to collision checking
*/
namespace Collision
{
	constexpr CollisionCallback DEFAULT_HANDLE = []( CollisionObject& a, CollisionObject& b ) {};

	std::vector<CollisionFace> createCubeMesh();
}

/**
 * @brief Binding of collider and its handler
*/
struct CollisionBinding
{
	CollisionCallback handler;
	CollisionObject object;
};

struct CollisionUnit
{
	std::vector<CollisionFace> faces;

	void collision( TransformSpace& relative ) const;
	bool rayCast( const vec4f& point, const vec3f& ray, float& distance, vec3f& normal ) const;
};

struct ColliderSpace : public Collider
{
	constexpr ColliderSpace() :
		Collider{ { Math::IDENTITY<mat4f>, Math::IDENTITY<mat4f> }, { Math::MAX<vec3f>, Math::MIN<vec3f> }, ColliderType::AABB }
	{

	}

	ColliderSpace( const mat4x4& transform ) : 
		Collider{ { transform, transform.inverse() }, Math::Box::calculateAABB( transform ), ColliderType::AABB }
	{

	}

	ColliderSpace( const Bounds<vec3f>& aabb ) :
		Collider{ { Math::createBoundingTransform( aabb ), Math::createBoundingInverseTransform( aabb ) }, aabb, ColliderType::AABB }
	{

	}

	virtual void collision( TransformSpace& sphere ) const = 0;
};

struct CollisionLattice : public ColliderSpace
{
	TransformSpace unitSpace;
	vec3i dimensions;
	size_t length;
	CollisionUnit* units;

	CollisionLattice( const Bounds<vec3f>& bounds, const vec3i& dimensions );
	CollisionLattice( const std::vector<CollisionFace>& meshCollision, const vec3i& dimensions );
	CollisionLattice( const std::vector<std::vector<CollisionFace>>& meshCollision, const vec3i& dimensions );

	// Copy Constructor
	constexpr CollisionLattice( const CollisionLattice& copy ) : 
		ColliderSpace( copy ),
		unitSpace( copy.unitSpace ),
		dimensions( copy.dimensions ),
		length( copy.length ),
		units( new CollisionUnit[length] )
	{
		std::copy_n( copy.units, copy.length, units );
	}

	// Move Constructor
	constexpr CollisionLattice( CollisionLattice&& copy ) noexcept :
		ColliderSpace( copy ),
		unitSpace( std::move( copy.unitSpace ) ),
		dimensions( std::move( copy.dimensions ) ),
		length( copy.length ),
		units( std::exchange( copy.units, nullptr ) )
	{

	}

	constexpr ~CollisionLattice()
	{
		if( units ) delete[] units;
	}

	void addTriangle( const TransformSpace& triangle );

	virtual void collision( TransformSpace& sphere ) const override;
	virtual bool rayCast( RaySensor& ray ) const override;

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

struct CollisionMesh : public ColliderSpace
{
	std::vector<CollisionFace> faces;

	CollisionMesh( const std::vector<CollisionFace>& source );

	CollisionMesh( const mat4f& transform ) : ColliderSpace( transform )
	{

	}

	virtual void collision( TransformSpace& relative ) const override;
	virtual bool rayCast( RaySensor& ray ) const override;
};

struct CollisionLayer
{
	std::vector<CollisionBinding> objects;
	std::vector<RaySensor*> sensors;

	constexpr void bind( const Collider& collider, CollisionCallback handler, void* source, uint64_t type )
	{
		objects.emplace_back( handler, CollisionObject( collider, type, source ) );
	}

	constexpr void unbind( const Collider& collider )
	{
		// TODO:
		// std::erase_if( objects, [ptr = &collider]( CollisionBinding& value ) { return &value.collider == ptr; } );
	}

	constexpr void addSensor( RaySensor& sensor )
	{
		sensors.emplace_back( &sensor );
	}

	constexpr void removeSensor( RaySensor& sensor )
	{
		std::erase_if( sensors, [ptr = &sensor]( RaySensor* value ) { return value == ptr; });
	}

	void testCollision();
};

#endif