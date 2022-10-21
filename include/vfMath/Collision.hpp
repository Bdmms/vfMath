#pragma once
#ifndef WB_COLLISION_HPP
#define WB_COLLISION_HPP

#include "Geometry.hpp"
#include <vector>

/**
 * @brief Enum of supported collider types
*/
enum class ColliderType : unsigned char
{
	Cube = 0,
	Sphere = 1,
	Cylinder = 2,
	Triangle = 3
};

/**
 * @brief Defines basic volume for instantaneous collisions
*/
struct InstantCollider
{
	mat4x4 transform;
	mat4x4 inverse;
};

/**
 * @brief Defines volume for testing collision
*/
struct LinearCollider : public InstantCollider
{
private:
	mat4x4 lastTransform;
	mat4x4 lastInverse;

public:
	/**
	 * @brief Creates the collider volume
	 * @param transform - reference to collider's transform
	*/
	LinearCollider(const mat4x4& transform = Math::IDENTITY<mat4x4>) : InstantCollider{ transform, transform.inverse() }, lastTransform(transform), lastInverse(inverse) {}

	void setTransform(const mat4x4& xform)
	{
		lastTransform = transform;
		lastInverse = inverse;
		transform = xform;
		inverse = xform.inverse();
	}

	void setTransform(const mat4x4& xform, const mat4x4& invXform)
	{
		lastTransform = transform;
		lastInverse = inverse;
		transform = xform;
		inverse = invXform;
	}

	constexpr const mat4x4& getTransform() const { return transform; }
	constexpr const mat4x4& getInverse() const { return inverse; }
};

/**
 * @brief Structure that stores information about the collision
*/
struct CollisionData
{
	const InstantCollider* parent;
	vec4f position	 = Math::ZERO<vec4f>;
	vec3f normal	 = Math::ZERO<vec3f>;
	vec3f recoveryDirection	 = Math::ZERO<vec3f>;
	float signedDistance = 0.0f;
};

typedef void (*CollisionTest)(CollisionData& collision, const InstantCollider& a, const InstantCollider& b);
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
	virtual void onCollision(const CollisionData& collision, InstantCollider& collider) { }
};

inline static CollisionHandler DEFAULT_COLLISION_HANDLER;

/**
 * @brief Collider containing references to volume and data stored in the collision layer
*/
struct Collider
{
	InstantCollider& volume;
	CollisionHandler& handler;
	ColliderType type;

	Collider(const ColliderType type, InstantCollider& volume, CollisionHandler& handler = DEFAULT_COLLISION_HANDLER)
		: volume(volume), handler(handler), type(type) {}
};

typedef InstantCollider CollisionFace;

struct MeshCollider : public InstantCollider
{
	CollisionFace* faces;
	size_t numFaces;
	float surfaceThickness = 0.1f;

	constexpr MeshCollider(const size_t numFaces)
		: InstantCollider({ Math::IDENTITY<mat4x4>, Math::IDENTITY<mat4x4> }), faces(new CollisionFace[numFaces]), numFaces(numFaces)
	{

	}

	constexpr MeshCollider(const MeshCollider& copy)
		: InstantCollider(copy), faces(new CollisionFace[copy.numFaces]), numFaces(copy.numFaces)
	{
		std::copy(copy.faces, copy.faces + numFaces, faces);
	}

	constexpr MeshCollider(MeshCollider&& copy) noexcept
		: InstantCollider(std::move(copy)), faces(std::exchange(copy.faces, nullptr)), numFaces(std::exchange(copy.numFaces, 0))
	{

	}

	~MeshCollider()
	{
		if( faces != nullptr )
			delete[] faces;
	}

	constexpr CollisionFace* begin() { return faces; }
	constexpr CollisionFace* end() { return faces + numFaces; }
};

/**
 * @brief Generalizes handling of collision between colliders of different types
*/
namespace Collision
{
	void getCollisionData(CollisionData& collision, const Collider& a, const Collider& b);
	CollisionTest getCollisionTest(const ColliderType a, const ColliderType b);

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
	std::vector<InstantCollider> volume;
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
		InstantCollider& objVolume = volume.emplace_back(transform, transform.inverse());
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
		InstantCollider relative;
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

template<ColliderType TypeID>
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

	/**
	 * @brief Adds the collider to the layer
	 * @param transform
	*/
	Collider& addCollider(const mat4x4& transform = Math::IDENTITY<mat4x4>)
	{
		LinearCollider& objVolume = volume.emplace_back(transform);
		return colliders.emplace_back(TypeID, objVolume);
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
};

/**
 * @brief A controlled list of colliders that handles internal collisions
*/
class MultiTypeCollisionLayer : public CollisionLayer
{
	std::vector<LinearCollider> volume;
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
	Collider& addCollider(const ColliderType type, const mat4x4& transform = Math::IDENTITY<mat4x4>)
	{
		LinearCollider& objVolume = volume.emplace_back(transform);
		return colliders.emplace_back( type, objVolume );
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


#endif