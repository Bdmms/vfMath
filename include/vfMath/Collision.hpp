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

namespace Math
{
	static void translate( TransformSpace& space, const vec3f& translation )
	{
		space.transform.origin += translation;
		space.inverse.origin -= space.inverse * translation;
	}

	namespace Box
	{
		Bounds<vec3f> calculateAABB( const mat4f& transform );

		bool pointTest( const vec4f& point );
		bool rayTest( const vec4f& point, const vec3f& direction );
		bool triangleTest( const TransformSpace& triangle );
		bool boxTest( const TransformSpace& box );

		vec4f rayCast( const vec4f& point, const vec3f& direction );
	}

	namespace Triangle
	{
		vec3f rayIntersect( const mat4f& triangle, const vec4f& point, const vec3f& direction );
		float rayDistance( const mat4f& triangle, const vec4f& point, const vec3f& direction );

		vec3f sphereDisplace( const TransformSpace& sphere, const TransformSpace& triangle );
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

typedef TransformSpace CollisionFace;

struct MeshCollider : public TransformSpace
{
	CollisionFace* faces;
	size_t numFaces;
	float surfaceThickness = 0.1f;

	constexpr MeshCollider(const size_t numFaces)
		: TransformSpace({ Math::IDENTITY<mat4x4>, Math::IDENTITY<mat4x4> }), faces(new CollisionFace[numFaces]), numFaces(numFaces)
	{

	}

	constexpr MeshCollider( const MeshCollider& copy )
		: TransformSpace( copy ), faces( new CollisionFace[copy.numFaces] ), numFaces( copy.numFaces )
	{
		std::copy(copy.faces, copy.faces + numFaces, faces);
	}

	constexpr MeshCollider( MeshCollider&& copy ) noexcept
		: TransformSpace( copy ), faces( std::exchange( copy.faces, nullptr ) ), numFaces( std::exchange( copy.numFaces, 0 ) )
	{

	}

	~MeshCollider()
	{
		if( faces != nullptr )
			delete[] faces;
	}

	constexpr CollisionFace* begin() { return faces; }
	constexpr CollisionFace* end() { return faces + numFaces; }
	constexpr const CollisionFace* begin() const { return faces; }
	constexpr const CollisionFace* end() const { return faces + numFaces; }
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

	MeshCollider createCubeMesh();
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
 * @brief CollisionLayer dedicated for handling collision against mesh colliders
*/
class MeshCollisionLayer : public CollisionLayer
{
	std::vector<MeshCollider*> meshData;
	std::vector<TransformSpace> volume;
	std::vector<Collider> colliders;

public:
	MeshCollisionLayer(const size_t initCapacity)
	{
		volume.reserve(initCapacity);
		colliders.reserve(initCapacity);
	}

	/**
	 * @brief Adds the collider to the layer
	*/
	Collider& addCollider(const ColliderType type, CollisionHandler& handler = DEFAULT_COLLISION_HANDLER, const mat4x4& transform = Math::IDENTITY<mat4x4>)
	{
		TransformSpace& objVolume = volume.emplace_back(transform, transform.inverse());
		return colliders.emplace_back(type, objVolume, handler);
	}

	/**
	 * @brief Adds the mesh collider to the layer
	*/
	void addCollider(MeshCollider& collider)
	{
		meshData.emplace_back(&collider);
	}

	/**
	 * @brief Performs the test for collisions on this layer
	*/
	virtual void executeCollisionTest() override
	{
		size_t size = colliders.size();
		TransformSpace relative;
		CollisionData collision;

		for ( Collider& collider : colliders )
		{
			CollisionTest test = Collision::getCollisionTest( collider.type, ColliderType::Triangle );
			
			// Check each face on the mesh
			for( MeshCollider* mesh : meshData )
			{
				MeshCollider& meshCollider = *mesh;
				collision.parent = mesh;
				relative.transform = meshCollider.inverse * collider.volume.transform;
				relative.inverse = meshCollider.transform * collider.volume.inverse;

				// TODO:
				//relative.inverse = Math::IDENTITY<mat4x4>;

				for ( CollisionFace& face : *mesh )
				{
					// Test collision relative to face
					test( collision, relative, face );

					// Check if the colliders intersect
					if ( collision.signedDistance <= meshCollider.surfaceThickness )
					{
						collider.handler.onCollision( collision, face );
					}
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
};

/*template<ColliderType TypeID>
class MonoTypeCollisionLayer : public CollisionLayer
{
	std::vector<LinearCollider> volume;
	std::vector<Collider> colliders;
	CollisionTest collisionTest;

public:
	MonoTypeCollisionLayer(const size_t initCapacity)
	{
		volume.reserve(initCapacity);
		colliders.reserve(initCapacity);
		collisionTest = Collision::getCollisionTest(TypeID, TypeID);
	}

	Collider& addCollider(const mat4x4& transform = Math::IDENTITY<mat4x4>)
	{
		LinearCollider& objVolume = volume.emplace_back(transform);
		return colliders.emplace_back(TypeID, objVolume);
	}

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
				collisionTest( collision, colliderA.volume, colliderB.volume );

				// Check if the colliders intersect
				if ( collision.signedDistance <= 0.0f )
				{
					// Activate the handlers on each collider
					colliderA.handler.onCollision( collision, colliderB.volume );
					colliderB.handler.onCollision( collision, colliderA.volume );
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

	constexpr LinearCollider* getVolumeBuffer()
	{
		return volume.data();
	}
};*/

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
};

struct CollisionLattice
{
	TransformSpace bounds;
	TransformSpace unitSpace;
	Bounds<vec3f> aabb;
	vec3i dimensions;
	size_t length;
	CollisionUnit* units;

	CollisionLattice( const mat4f& transform, const vec3i& dimensions ) :
		bounds{ transform, transform.inverse() },
		dimensions( dimensions ),
		aabb( Math::Box::calculateAABB( transform ) ),
		length( size_t( dimensions.x ) * size_t( dimensions.y ) * size_t( dimensions.z ) ),
		units( new CollisionUnit[length] )
	{
		vec3f unitScale = 2.0f / vec3f( dimensions );
		mat4x4 unitConvert = { { unitScale.x, 0.0f, 0.0f }, { 0.0f, unitScale.y, 0.0f }, { 0.0f, 0.0f, unitScale.z }, vec4f{ -1.0f, -1.0f, -1.0f, 1.0f } };
		unitSpace.transform = bounds.transform * unitConvert;
		unitSpace.inverse = unitSpace.transform.inverse();

		unitScale.w = 1.0f;
	}

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

	constexpr size_t getAvgTriangleCount() const
	{
		size_t totalCount = 0u;
		for( size_t i = 0; i < length; ++i )
		{
			totalCount += units[i].faces.size();
		}
		return totalCount / length;
	}
};

struct CollisionMap
{
	std::vector<CollisionLattice> fields;

	const CollisionLattice& add( const MeshCollider& meshCollision, const vec3i& dimensions );
	const CollisionLattice& add( const std::vector<MeshCollider>& meshCollision, const vec3i& dimensions );

	constexpr const CollisionUnit& getUnit( const vec4i& id ) const
	{
		return fields[id.w].getUnit( id );
	}

	vec4i getPointID( const vec4f& point ) const;

	void collision( TransformSpace& sphere ) const;
};

#endif