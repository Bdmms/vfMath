#include "../include/vfMath/Collision.hpp"

#if defined(VF_DEBUG_TRIANGLE_LATTICE) | defined(VF_RAYCAST_DEBUG)
#include <iostream>
#endif

typedef bool ( *IntersectTest ) ( const Collider& a, const Collider& b );

/**
 * @brief Clamps the UV coordinates to the boundary of a triangle
 * @param uv - UV coordinates stored in a 2D vector
 * @return clamped UV coordinates
*/
static vec2f clampTriangleUV( const vec2f& uv )
{
	vec2f bounded = Math::max( uv, Math::ZERO<vec2f> );
	return bounded.x + bounded.y > 1.0f ? bounded / ( bounded.x + bounded.y ) : bounded;
}

/**
 * @brief Clamps the UV coordinates to the boundary of a triangle
 * @param uv - UV coordinates stored in a 4D vector
 * @return clamped UV coordinates
*/
static vec4f clampTriangleUV( const vec4f& uv )
{
	vec4f bounded = Math::max( uv, Math::ZERO<vec4f> );
	return bounded.x + bounded.y > 1.0f ? bounded / ( bounded.x + bounded.y ) : bounded;
}

// ---------------------------
// Collision Mesh Primitives
// ---------------------------

CollisionFace createFace(const vec4f& p0, const vec4f& p1, const vec4f& p2)
{
	vec4f xAxis = p1 - p0;
	vec4f yAxis = p2 - p0;
	mat4x4 transform{
		xAxis,
		yAxis,
		Math::normalize(Math::cross(xAxis, yAxis)),
		vec4f{ p0.x, p0.y, p0.z, 1.0f }
	};
	return { transform, Math::invert( transform ) };
}

CollisionFace* setQuad(CollisionFace* faces, const vec4f& p0, const vec4f& p1, const vec4f& p2, const vec4f& p3, const bool reverse)
{
	if (reverse)
	{
		*(faces++) = createFace(p0, p3, p1);
		*(faces++) = createFace(p2, p1, p3);
	}
	else
	{
		*(faces++) = createFace(p0, p1, p3);
		*(faces++) = createFace(p2, p3, p1);
	}
	return faces;
}

std::vector<CollisionFace> Collision::createCubeMesh()
{
	std::vector<CollisionFace> collider(12);
	CollisionFace* ptr = collider.data();

	ptr = setQuad(ptr, { -1.0f, -1.0f, -1.0f }, { -1.0f,  1.0f, -1.0f }, {  1.0f,  1.0f, -1.0f }, {  1.0f, -1.0f, -1.0f }, false);
	ptr = setQuad(ptr, { -1.0f, -1.0f,  1.0f }, { -1.0f,  1.0f,  1.0f }, {  1.0f,  1.0f,  1.0f }, {  1.0f, -1.0f,  1.0f }, true);
	ptr = setQuad(ptr, { -1.0f, -1.0f, -1.0f }, { -1.0f, -1.0f,  1.0f }, {  1.0f, -1.0f,  1.0f }, {  1.0f, -1.0f, -1.0f }, true);
	ptr = setQuad(ptr, { -1.0f,  1.0f, -1.0f }, { -1.0f,  1.0f,  1.0f }, {  1.0f,  1.0f,  1.0f }, {  1.0f,  1.0f, -1.0f }, false);
	ptr = setQuad(ptr, { -1.0f, -1.0f, -1.0f }, { -1.0f,  1.0f, -1.0f }, { -1.0f,  1.0f,  1.0f }, { -1.0f, -1.0f,  1.0f }, true);
	ptr = setQuad(ptr, {  1.0f, -1.0f, -1.0f }, {  1.0f,  1.0f, -1.0f }, {  1.0f,  1.0f,  1.0f }, {  1.0f, -1.0f,  1.0f }, false);

	return collider;
}

// ---------------------------
// Other Functions
// ---------------------------

static bool sphereFaceCollision( vec4f& displacement, const TransformSpace& sphere, const TransformSpace& face )
{
	vec4f uvs = face.inverse * sphere.transform.origin;
	if( uvs.x + uvs.y > 1.0f || uvs.x < 0.0f || uvs.y < 0.0f ) return false;

	vec4f bounded = Math::max( uvs, Math::ZERO<vec4f> );
	vec4f clamped = bounded.x + bounded.y > 1.0f ? bounded / ( bounded.x + bounded.y ) : bounded;
	mat4x4 relFace = sphere.inverse * face.transform;

	vec4f compNormal = relFace.origin + relFace.x_axis * uvs.x + relFace.y_axis * uvs.y;
	vec4f compCombined = relFace.origin + relFace.x_axis * clamped.x + relFace.y_axis * clamped.y;
	vec4f compSurface = compCombined - compNormal;
	float surfaceDistance2 = Math::dot_3D( compSurface, compSurface );
	float normalDistance2 = Math::dot_3D( compNormal, compNormal );
	
	if( surfaceDistance2 > 1.0f ) return false;

	float x = sqrtf( 1.0f - surfaceDistance2 ) - sqrtf( normalDistance2 );
	displacement = sphere.transform * ( Math::normalize( relFace.z_axis ) * x );

	return Math::dot_3D( compCombined, compCombined ) <= 1.0f;
}

// --------------------------
// --- Intersection Table ---
// --------------------------

bool intersect_AABB( const Collider& a, const Collider& b )
{
	return Math::overlaps3D( a.aabb, b.aabb );
}

bool intersect_Box_Box( const Collider& a, const Collider& b )
{
	return Math::overlaps3D( a.aabb, b.aabb ) &&
		vf::geometry::box::boxTest( { a.current.inverse * b.current.transform, b.current.inverse * a.current.transform } );
}

bool intersect_Box_Sphere( const Collider& a, const Collider& b )
{
	return Math::overlaps3D( a.aabb, b.aabb ) &&
		vf::geometry::sphere::boxTest( { b.current.inverse * a.current.transform, a.current.inverse * b.current.transform } );
}

bool intersect_Sphere_Box( const Collider& a, const Collider& b )
{
	return Math::overlaps3D( a.aabb, b.aabb ) &&
		vf::geometry::sphere::boxTest( { a.current.inverse * b.current.transform, b.current.inverse * a.current.transform } );
}

bool intersect_Sphere_Sphere( const Collider& a, const Collider& b )
{
	return Math::overlaps3D( a.aabb, b.aabb ) &&
		vf::geometry::sphere::sphereTest( { a.current.inverse * b.current.transform, b.current.inverse * a.current.transform } );
}

const IntersectTest intersectionMatrix[4][4]
{
	{ intersect_AABB, intersect_AABB,		intersect_AABB,			 intersect_AABB },
	{ intersect_AABB, intersect_Box_Box,	intersect_Box_Sphere,	 intersect_AABB },
	{ intersect_AABB, intersect_Sphere_Box, intersect_Sphere_Sphere, intersect_AABB },
	{ intersect_AABB, intersect_AABB,		intersect_AABB,			 intersect_AABB },
};

// -------------------------
// --- General Collision ---
// -------------------------

/**
 * This function converts a 2D array into a 1D array of faces.
*/
void packTriangles( CollisionLatticeData& lattice, const std::vector<std::vector<CollisionFace>>& unitFaces )
{
	for( size_t i = 0LLU; i < lattice.units.size(); ++i )
	{
		const std::vector<CollisionFace>& unitList = unitFaces[i];
		CollisionLatticeData::Unit& unit = lattice.units[i];

		// Store the point in the list used by the unit
		unit.idx = lattice.faces.size();
		for( const CollisionFace& face : unitList )
		{
			lattice.faces.emplace_back( face );
		}
		unit.size = lattice.faces.size() - unit.idx;
	}
}

/**
 * This function determines which units a triangle intersects with.
*/
void addTriangle( std::vector<std::vector<CollisionFace>>& unitFaces, const TransformSpace& triangle, const CollisionLatticeData& lattice )
{
	const vec4i& dimensions = lattice.getDimensions();
	const TransformSpace relative = { lattice.defaultBounds.inverse * triangle.transform, triangle.inverse * lattice.defaultBounds.transform };
	const TransformSpace unitXform = { lattice.unitSpace.inverse * relative.transform, relative.inverse * lattice.unitSpace.transform };
	TransformSpace unit = { Math::create::scale( vec4f{ 0.5f, 0.5f, 0.5f } ), Math::create::scale( vec4f{ 2.0f, 2.0f, 2.0f } ) };
	TransformSpace unitTriangle;
	
	Bounds<vec4f> triangleBounds;
	vf::geometry::process::AABB_TRIANGLE( unitXform.transform, triangleBounds.min, triangleBounds.max );

	vec4i minIdx = Math::clamp( static_cast<vec4i>( Math::floor( triangleBounds.min - vf::geometry::P_EPSILON ) ), Math::ZERO<vec4i>, dimensions - Math::ONES<vec4i> );
	vec4i maxIdx = Math::clamp( static_cast<vec4i>( Math::ceil(  triangleBounds.max + vf::geometry::P_EPSILON ) ), Math::ONES<vec4i>, dimensions );
	minIdx.w = 0;
	maxIdx.w = 0;

	// Populate the units with any intersecting face 
	for( vec4i idx = minIdx; idx.z < maxIdx.z; ++idx.z )
	{
		size_t iz = idx.z * dimensions.y;
		for( idx.y = minIdx.y; idx.y < maxIdx.y; ++idx.y )
		{
			size_t iy = ( iz + idx.y ) * dimensions.x;
			for( idx.x = minIdx.x; idx.x < maxIdx.x; ++idx.x )
			{
				unit.transform.origin = static_cast<vec4f>( idx ) + vec4f{ 0.5f, 0.5f, 0.5f, 1.0f };
				unit.inverse.origin = unit.transform.origin * vec4f{ -2.0f, -2.0f, -2.0f, 1.0f };
				unit.inverse = Math::invert( unit.transform );

				unitTriangle.transform = unit.inverse * unitXform.transform;
				unitTriangle.inverse = unitXform.inverse * unit.transform;

				if( vf::geometry::box::triangleTest( unitTriangle ) )
				{
					unitFaces[iy + idx.x].emplace_back( unitXform );
				}

#ifdef VF_DEBUG_TRIANGLE_LATTICE
				else
				{
					vec4f v0 = unitXform.transform.origin;
					vec4f v1 = unitXform.transform.origin + unitXform.transform.x_axis;
					vec4f v2 = unitXform.transform.origin + unitXform.transform.y_axis;

					vec4i p0 = vec4i( Math::floor( v0 ) );
					vec4i p1 = vec4i( Math::floor( v1 ) );
					vec4i p2 = vec4i( Math::floor( v2 ) );

					vec4i a = idx ^ p0;
					vec4i b = idx ^ p1;
					vec4i c = idx ^ p2;
					bool expected = !( a.x | a.y | a.z ) || !( b.x | b.y | b.z ) || !( c.x | c.y | c.z );

					if( expected )
					{
						std::cout << "--- FAILED ---\n";
						std::cout << idx.x << ", " << idx.y << ", " << idx.z << "\n";

						std::cout << p0.x << ", " << p0.y << ", " << p0.z << "\n";
						std::cout << p1.x << ", " << p1.y << ", " << p1.z << "\n";
						std::cout << p2.x << ", " << p2.y << ", " << p2.z << "\n";

						std::cout << v0.x << ", " << v0.y << ", " << v0.z << "\n";
						std::cout << v1.x << ", " << v1.y << ", " << v1.z << "\n";
						std::cout << v2.x << ", " << v2.y << ", " << v2.z << "\n";

						v0 = unitTriangle.transform.origin;
						v1 = unitTriangle.transform.origin + unitTriangle.transform.x_axis;
						v2 = unitTriangle.transform.origin + unitTriangle.transform.y_axis;

						std::cout << v0.x << ", " << v0.y << ", " << v0.z << "\n";
						std::cout << v1.x << ", " << v1.y << ", " << v1.z << "\n";
						std::cout << v2.x << ", " << v2.y << ", " << v2.z << "\n";

						Math::Box::triangleTest( unitTriangle );
					}
				}
#endif
			}
		}
	}
}

void Collision::convertToLattice( CollisionLatticeData& lattice, const std::vector<CollisionFace>& mesh, const vec4i& dimensions )
{
	lattice.resize( dimensions );
	Bounds<vec4f> aabb = { Math::MAX<vec4f>, Math::MIN<vec4f> };
	Math::extendFaces( aabb, mesh );

	lattice.defaultBounds.transform = Math::createBoundingTransform( aabb );
	lattice.defaultBounds.inverse = Math::createBoundingInverseTransform( aabb );

	std::vector<std::vector<CollisionFace>> unitFaces( lattice.getUnitCount() );
	for( const CollisionFace& face : mesh )
	{
		addTriangle( unitFaces, face, lattice );
	}
	packTriangles( lattice, unitFaces );
}

void Collision::convertToLattice( CollisionLatticeData& lattice, const std::vector<std::vector<CollisionFace>>& mesh, const vec4i& dimensions )
{
	lattice.resize( dimensions );
	Bounds<vec4f> aabb = { Math::MAX<vec4f>, Math::MIN<vec4f> };

	for( const std::vector<CollisionFace>& group : mesh )
	{
		Math::extendFaces( aabb, group );
	}

	lattice.defaultBounds.transform = Math::createBoundingTransform( aabb );
	lattice.defaultBounds.inverse = Math::createBoundingInverseTransform( aabb );

	std::vector<std::vector<CollisionFace>> unitFaces( lattice.getUnitCount() );
	for( const std::vector<CollisionFace>& group : mesh )
	{
		for( const CollisionFace& face : group )
		{
			addTriangle( unitFaces, face, lattice );
		}
	}
	packTriangles( lattice, unitFaces );
}


CollisionMesh::CollisionMesh( const std::vector<CollisionFace>& source ) : 
	ColliderSpace(),
	faces()
{
	setFaces( source );
}

void CollisionMesh::setFaces( const std::vector<CollisionFace>& source )
{
	aabb = { Math::MAX<vec4f>, Math::MIN<vec4f> };
	Math::extendFaces( aabb, source );

	current.transform = Math::createBoundingTransform( aabb );
	current.inverse = Math::createBoundingInverseTransform( aabb );
	previous = current;

	faces.resize( source.size() );
	for( size_t i = 0; i < source.size(); ++i )
	{
		faces[i].transform = current.inverse * source[i].transform;
		faces[i].inverse = Math::invert( faces[i].transform );
	}
}

void CollisionMesh::collision( TransformSpace& sphere ) const
{
	TransformSpace relative = { current.inverse * sphere.transform, sphere.inverse * current.transform };
	vec4f startOrigin = relative.transform.origin;
	vec4f displacement;

	for( const CollisionFace& face : faces )
	{
		if( sphereFaceCollision( displacement, relative, face ) )
		{
			Math::translate( relative, displacement );
		}
	}

	Math::translate( sphere, current.transform * ( relative.transform.origin - startOrigin ) );
}

void collision_CollisionLatticeUnit_sphere( TransformSpace& relativeSphere, const CollisionFace* ptr, const CollisionFace* end )
{
	vec4f displacement;
	while( ptr < end )
	{
		// If sphere intersects face, translate sphere by face displacement
		if( sphereFaceCollision( displacement, relativeSphere, *ptr ) )
		{
			Math::translate( relativeSphere, displacement );
		}
		++ptr;
	}
}

void CollisionLattice::collision( TransformSpace& sphere ) const
{
	const TransformSpace& unitSpace = data.getUnitSpace();
	const vec4i& dimensions = data.getDimensions();
	const CollisionLatticeData::Unit* units = data.getUnits();
	const CollisionFace* faces = data.getFaces();

	// Convert the sphere into unit space
	TransformSpace relative = { current.inverse * sphere.transform, sphere.inverse * current.transform };
	TransformSpace unitXform = { unitSpace.inverse * relative.transform, relative.inverse * unitSpace.transform };
	vec4f startOrigin = unitXform.transform.origin;

	// Compute the bounds of the sphere in unit space
	Bounds<vec4f> aabb = vf::geometry::box::calculateAABB( unitXform.transform );
	vec4i minIdx = Math::clamp( static_cast<vec4i>( Math::floor( aabb.min ) ), Math::ZERO<vec4i>, dimensions );
	vec4i maxIdx = Math::clamp( static_cast<vec4i>( Math::ceil(  aabb.max ) ), Math::ZERO<vec4i>, dimensions );

	// Iterate over the units the sphere overlaps with
	for( vec4i idx = minIdx; idx.z < maxIdx.z; ++idx.z )
	{
		size_t iz = idx.z * dimensions.y;
		for( idx.y = minIdx.y; idx.y < maxIdx.y; ++idx.y )
		{
			size_t iy = ( iz + idx.y ) * dimensions.x;
			for( idx.x = minIdx.x; idx.x < maxIdx.x; ++idx.x )
			{
				// Process the collision within the unit
				const CollisionLatticeData::Unit& unit = units[iy + idx.x];
				const CollisionFace* face = faces + unit.idx;
				collision_CollisionLatticeUnit_sphere( unitXform, face, face + unit.size );
			}
		}
	}

	// Propagate the translation of the collision back into world space
	Math::translate( sphere, current.transform * ( unitSpace.transform * ( unitXform.transform.origin - startOrigin ) ) );
}

bool CollisionMesh::rayCast( RaySensor& ray ) const
{
	// Convert finite ray into mesh space
	vec4f origin = current.inverse * ray.origin;
	vec4f line = current.inverse * ( ray.direction * ray.distance );

	if( !vf::geometry::box::rayTest( origin, line ) ) return false;

	// We need to make sure the distance/normal is calculated relative to the mesh space
	float relDistance = 1.0f;
	vec4f relNormal;
	bool hit = false;

	for( const CollisionFace& face : faces )
	{
		// Rays only intersect with front-facing faces
		if( Math::dot_3D( face.transform.z_axis, line ) > 0.0f ) continue;

		// Check that the distance is postive and less than the current distance
		float rayDistance = vf::geometry::triangle::rayDistance( face.transform, origin, line );
		if( rayDistance >= 0.0f && rayDistance <= relDistance )
		{
			relDistance = rayDistance;
			relNormal = face.transform.z_axis;
			hit = true;
		}
	}

	if( hit )
	{
		// Convert the distance/normal back into world space
		ray.normal = Math::normalize( current.transform * relNormal );
		ray.distance = Math::length( current.transform * ( line * relDistance ) );
		ray.space = &current;
		ray.hit = true;
		return true;
	}

	return false;
}

bool raycast_CollisionLatticeUnit_sphere( const vec4f& point, const vec4f& ray, float& distance, vec4f& normal, const CollisionFace* ptr, const CollisionFace* end )
{
	bool hit = false;

	while( ptr < end )
	{
		const CollisionFace& face = *ptr;
		++ptr;

		// Rays only intersect with front-facing faces
		if( Math::dot_3D( face.transform.z_axis, ray ) > 0.0f ) continue;

		// Check that the distance is postive and less than the current distance
		float rayDistance = vf::geometry::triangle::rayDistance( face.transform, point, ray );
		if( rayDistance >= 0.0f && rayDistance <= distance )
		{
			distance = rayDistance;
			normal = face.transform.z_axis;
			hit = true;
		}
	}

	return hit;
}

bool CollisionLattice::rayCast( RaySensor& ray ) const
{
	const TransformSpace& unitSpace = data.getUnitSpace();
	const vec4i& dimensions = data.getDimensions();
	const CollisionLatticeData::Unit* units = data.getUnits();
	const CollisionFace* faces = data.getFaces();

	// Convert finite ray into unit space
	vec4f origin = unitSpace.inverse * ( current.inverse * ray.origin );
	vec4f line = unitSpace.inverse * ( current.inverse * ( ray.direction * ray.distance ) );

	// Clip the ray within along the boundaries of the lattice, exit if the ray is outside the bounds
	Bounds<float> rayClip = vf::geometry::aabb::rayBounds( Math::ZERO<vec4f>, static_cast<vec4f>( dimensions ), origin, line );
	if( rayClip.min > rayClip.max ) return false;

	// Epsilon fixes a rounding issue when retrieving the unit index
	float traveled = rayClip.min + Math::EPSILON<float>;
	float maxDistance = rayClip.max - Math::EPSILON<float>;

	// Define relative parameters for use when casting rays within the units
	vec4f relNormal;
	float relDistance = rayClip.max;

	// This stores whether the line is moving in the direction of -1 or 1 across each axis
	vec4i idxSign = static_cast<vec4i>( Math::sign( line ) );
	vec4i idxUnit = ( idxSign + Math::ONES<vec4i> ) / 2;
	vec4i idx = static_cast<vec4i>( Math::floor( origin + line * traveled ) );

	// Check that the starting idx is within bounds (Fail-safe)
	bool xInBounds = idx.x >= 0 && idx.x < dimensions.x;
	bool yInBounds = idx.y >= 0 && idx.y < dimensions.y;
	bool zInBounds = idx.z >= 0 && idx.z < dimensions.z;
	if( !( xInBounds && yInBounds && zInBounds ) )
	{
		return false;
	}

#ifdef VF_RAYCAST_DEBUG
	vec4i end = vec4i( Math::floor( origin + line * maxDistance ) );
	std::cout << "Bounds:\n";
	std::cout << idx.x << ", " << idx.y << ", " << idx.z << "\n";
	std::cout << end.x << ", " << end.y << ", " << end.z << "\n";
	std::cout << line.x << ", " << line.y << ", " << line.z << "\n";
	std::cout << "Path:\n";
#endif

	// Keep going until it reaches the end of the line
	while( traveled <= maxDistance )
	{
#ifdef VF_RAYCAST_SAFE
		// Check that the current idx is within bounds
		vec4i inBounds = ( idx >= Math::ZERO<vec4i> ) & ( idx < dimensions );

		if( inBounds.x && inBounds.y && inBounds.z )
		{
			// Execute collision test
			size_t i = idx.x + ( ( idx.y + ( idx.z * dimensions.y ) ) * dimensions.x );
			if( units[i].rayCast( origin, line, relDistance, relNormal ) )
			{
				// Convert the distance/normal back into world space
				ray.normal = Math::normalize( current.transform * ( unitSpace.transform * relNormal ) );
				ray.distance = Math::length( current.transform * ( unitSpace.transform * ( line * relDistance ) ) );
				ray.space = &current;
				ray.hit = true;
				return true;
			}
		}
#else
		// Assume that the index is in bounds after clipping the ray
		size_t i = idx.x + ( ( idx.y + ( idx.z * dimensions.y ) ) * dimensions.x );

		const CollisionLatticeData::Unit& unit = units[i];
		const CollisionFace* ptr = faces + unit.idx;
		if( raycast_CollisionLatticeUnit_sphere( origin, line, relDistance, relNormal, ptr, ptr + unit.size ) )
		{
			// Convert the distance/normal back into world space
			ray.normal = Math::normalize( current.transform * ( unitSpace.transform * relNormal ) );
			ray.distance = Math::length( current.transform * ( unitSpace.transform * ( line * relDistance ) ) );
			ray.space = &current;
			ray.hit = true;
			return true;
		}
#endif

		// Find the closest unit to increment to
		vec4f point = origin + line * traveled;
		vec4f difference = ( static_cast<vec4f>( idx + idxUnit ) - point ) / line;

#ifdef VF_RAYCAST_DEBUG
		std::cout << "distance = " << traveled << "\n";
		std::cout << idx.x << ", " << idx.y << ", " << idx.z << "\n";
		std::cout << point.x << ", " << point.y << ", " << point.z << "\n";
		std::cout << difference.x << ", " << difference.y << ", " << difference.z << "\n";
#endif

		if( difference.x < difference.y && difference.x < difference.z )
		{
			traveled += difference.x;
			idx.x += idxSign.x;
		}
		else if( difference.y < difference.x && difference.y < difference.z )
		{
			traveled += difference.y;
			idx.y += idxSign.y;
		}
		else
		{
			traveled += difference.z;
			idx.z += idxSign.z;
		}
	}

	return false;
}

bool CollisionLayer::testCollision( const Collider& a, const Collider& b )
{
	return intersectionMatrix[(uint8_t)a.type][(uint8_t)b.type]( a, b );
}

void CollisionLayer::testCollision()
{
	collisionLock.lock();

	size_t l1 = objects.size();
	if( l1 >= 2LLU )
	{
		size_t l0 = l1 - 1LLU;

		// Process the collisions
		for( size_t i = 0; i < l0; ++i )
		{
			CollisionBinding& a = objects[i];
			Collider& colliderA = *a.object.collider;
			const IntersectTest* colTest = intersectionMatrix[(uint8_t)colliderA.type];

			// Skip collision if collider A is disabled
			if( colliderA.flags & Collider::DISABLED ) continue;

			for( size_t j = i + 1LLU; j < l1; ++j )
			{
				CollisionBinding& b = objects[j];
				Collider& colliderB = *b.object.collider;
				const uint8_t typeB = (uint8_t)colliderB.type;
				
				// Skip collision if collider B is disabled, or if colliders A and B are both passive
				if( ( colliderB.flags & Collider::DISABLED ) ||
					( colliderA.flags & colliderB.flags & Collider::PASSIVE ) ) continue;

				if( colTest[typeB]( colliderA, colliderB ) )
				{
					colliderA.flags |= Collider::COLLIDED;
					colliderB.flags |= Collider::COLLIDED;
					a.handler( a.object, b.object );
					b.handler( b.object, a.object );
				}
			}
		}
	}

	// Process the sensors after the object may have moved
	for( RaySensor* sensor : sensors )
	{
		for( const CollisionBinding& pair : objects )
		{
			pair.object.collider->rayCast( *sensor );
		}
	}

	collisionLock.unlock();
}


void CollisionLayer::bind( Collider& collider, CollisionCallback handler, void* source, uint64_t type )
{
	collisionLock.lock();
	objects.emplace_back( handler, CollisionObject( &collider, type, source ) );
	collisionLock.unlock();
}

void CollisionLayer::unbind( const Collider& collider, CollisionCallback handler )
{
	collisionLock.lock();
	std::erase_if( objects, [ptr = &collider, handler = handler]( CollisionBinding& value ) 
	{ 
		return value.object.collider == ptr && value.handler == handler; 
	} );
	collisionLock.unlock();
}

void CollisionLayer::unbind( const Collider& collider )
{
	collisionLock.lock();
	std::erase_if( objects, [ptr = &collider]( CollisionBinding& value ) { return value.object.collider == ptr; } );
	collisionLock.unlock();
}

void CollisionLayer::addSensor( RaySensor& sensor )
{
	collisionLock.lock();
	sensors.emplace_back( &sensor );
	collisionLock.unlock();
}

void CollisionLayer::removeSensor( RaySensor& sensor )
{
	collisionLock.lock();
	std::erase_if( sensors, [ptr = &sensor]( RaySensor* value ) { return value == ptr; } );
	collisionLock.unlock();
}