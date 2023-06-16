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
	Sphere = 1,
	Triangle = 2
};

struct TransformSpace
{
	mat4x4 transform;
	mat4x4 inverse;
};

template <typename T>
struct Bounds
{
	T min;
	T max;
};

typedef TransformSpace CollisionFace;

namespace Math
{
	static void translate( TransformSpace& space, const vec3f& translation )
	{
		space.transform.origin += translation;
		space.inverse.origin -= space.inverse * translation;
	}

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

	template <typename IterableFaces>
	static void extend( Bounds<vec3f>& bounds, const IterableFaces& faces )
	{
		for( const CollisionFace& face : faces )
		{
			extend( bounds, face );
		}
	}

	static mat4x4 createBoundingTransform( const Bounds<vec3f>& bounds )
	{
		vec3f scale = ( bounds.max - bounds.min ) * 0.5f;
		vec3f translation = ( bounds.max + bounds.min ) * 0.5f;
		return { { scale.x, 0.0f, 0.0f }, { 0.0f, scale.y, 0.0f }, { 0.0f, 0.0f, scale.z }, translation };
	}

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

	namespace Triangle
	{
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
 * @brief Structure that stores information about the collision
*/
struct CollisionData
{
	const TransformSpace* parent = nullptr;
	vec4f position	 = Math::ZERO<vec4f>;
	vec3f normal	 = Math::ZERO<vec3f>;
	vec3f recoveryDirection	= Math::ZERO<vec3f>;
	vec3f displacement = Math::ZERO<vec3f>;
	float signedDistance = Math::MAX<float>;
};

typedef void (*CollisionTest) (CollisionData& collision, const TransformSpace& a, const TransformSpace& b);
typedef bool (*IntersectTest) (const TransformSpace& a, const TransformSpace& b);

/**
 * @brief Interface for object handling collision
*/
struct CollisionHandler
{
	/**
	 * @brief Triggered when collision occurs
	 * @param collision - data relevant to the collision
	 * @param collider - opposite collider
	*/
	virtual void onCollision(const CollisionData& collision, TransformSpace& collider) { }
};

inline static CollisionHandler DEFAULT_COLLISION_HANDLER;

/**
 * @brief Collider containing references to volume and data stored in the collision layer
*/
struct Collider
{
	TransformSpace& volume;
	CollisionHandler& handler;
	ColliderType type;
	uint8_t collisionFlag;

	Collider(const ColliderType type, TransformSpace& volume, CollisionHandler& handler = DEFAULT_COLLISION_HANDLER)
		: volume(volume), handler(handler), type(type), collisionFlag(0) {}
};

/**
 * @brief Generalizes handling of collision between colliders of different types
*/
namespace Collision
{
	void getCollisionData(CollisionData& collision, const Collider& a, const Collider& b);
	CollisionTest getCollisionTest(const ColliderType a, const ColliderType b);

	bool isIntersecting(const Collider& a, const Collider& b);
	IntersectTest getIntersectTest(const ColliderType a, const ColliderType b);

	std::vector<CollisionFace> createCubeMesh();
}

/**
 * @brief Interface for defining a collision layer
*/
struct CollisionLayer
{
	virtual void executeCollisionTest() = 0;
	virtual const Collider* data() const = 0;
	virtual size_t size() const = 0;
};

/**
 * @brief A controlled list of colliders that handles internal collisions
*/
class MultiTypeCollisionLayer : public CollisionLayer
{
	std::vector<TransformSpace> volume;
	std::vector<Collider> colliders;

public:
	/**
	 * @brief 
	 * @param initCapacity 
	*/
	constexpr MultiTypeCollisionLayer() { }

	/**
	 * @brief 
	 * @param initCapacity
	*/
	constexpr MultiTypeCollisionLayer(const size_t initCapacity)
	{
		volume.reserve(initCapacity);
		colliders.reserve(initCapacity);
	}

	/**
	 * @brief Adds the collider to the layer
	 * @param type
	 * @param transform
	*/
	Collider& addCollider(const ColliderType type, const mat4x4& transform = Math::IDENTITY<mat4x4>, CollisionHandler& handler = DEFAULT_COLLISION_HANDLER)
	{
		TransformSpace& objVolume = volume.emplace_back( transform, transform.inverse() );
		return colliders.emplace_back( type, objVolume, handler );
	}

	/**
	 * @brief Performs the test for collisions on this layer
	*/
	virtual void executeCollisionTest() override
	{
		size_t size = colliders.size();
		size_t rsize = size - 1;
		CollisionData collision;

		for (size_t i = 0; i < rsize; ++i)
		{
			Collider& colliderA = colliders[i];

			for (size_t j = i + 1; j < size; ++j)
			{
				Collider& colliderB = colliders[j];
				Collision::getCollisionData( collision, colliderA, colliderB );

				// Check if the colliders intersect
				if ( collision.signedDistance <= 0.0f )
				{
					// Activate the handlers on each collider
					colliderA.handler.onCollision(collision, colliderB.volume);
					colliderB.handler.onCollision(collision, colliderA.volume);
				}
			}
		}
	}

	virtual size_t size() const override
	{
		return colliders.size();
	}

	virtual const Collider* data() const override
	{
		return colliders.data();
	}

	constexpr std::vector<Collider>::const_iterator begin() const
	{
		return colliders.begin();
	}

	constexpr std::vector<Collider>::const_iterator end() const
	{
		return colliders.end();
	}
};

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

#endif