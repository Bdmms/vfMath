#pragma once
#ifndef VF_PHYSICS_HPP
#define VF_PHYSICS_HPP

#include "Collision.hpp"

// Scalar
typedef float Mass_t;			// kg

// Linear
typedef vec4f Point_t;			// x metres
typedef vec3f Displacement_t;	// x metres
typedef vec3f Velocity_t;		// x metres / s
typedef vec3f Acceleration_t;	// x metres / s^2
typedef vec3f Momentum_t;		// x kg * x metres / s
typedef vec3f Force_t;			// x kg * units / s^2

// Angular
typedef quat Rotation_t;					// x radians
typedef AxisAngle AngularVelocity_t;		// x radians / s
typedef AxisAngle AngularAcceleration_t;	// x radians / s^2
typedef AxisAngle AngularMomentum_t;		// x kg * radians / s
typedef AxisAngle Moment_t;					// x kg * radians / s^2

typedef InstantCollider TransformSpace;

constexpr Acceleration_t GRAVITATIONAL_ACCELERATION = { 0.0f, -9.81f, 0.0f };
constexpr TransformSpace GLOBAL_SPACE = { Math::IDENTITY<mat4x4>, Math::IDENTITY<mat4x4> };

namespace Physics
{
	static Moment_t getTorque(Displacement_t displacement, Force_t force)
	{
		return Math::cross(displacement, force);
	}
}

/*struct Joint
{
	Point_t point = Math::ZERO<Point_t>;
	Rotation_t rotation = Math::IDENTITY<Rotation_t>;
};*/

struct RigidBody : public CollisionHandler
{
	// Transform space of RigidBody
	TransformSpace jointSpace;
	TransformSpace xformSpace;
	// Children of RigidBody
	std::vector<RigidBody> children;
	// Transform relative to parent
	mat4x4 rJoint;
	vec4f rCentroid;

	// Linear (Relative to joint)
	//Point_t rPosition;
	//Velocity_t rVelocity = Math::ZERO<Velocity_t>;
	Force_t rForce = Math::ZERO<Force_t>;

	// Angular (Relative to joint)
	Rotation_t rOrientation = Math::IDENTITY<Rotation_t>;
	AngularVelocity_t rAngularVelocity = Math::IDENTITY<AngularVelocity_t>;
	Moment_t rMoment = Math::IDENTITY<Moment_t>;

	// Space that members are relative to
	const TransformSpace& parent;

	// Scalars
	Mass_t mass = 1.0f;

	RigidBody(const Point_t& joint = Math::ZERO<Point_t>, const Point_t& position = Math::ZERO<Point_t>)
		: parent(GLOBAL_SPACE), rJoint(Math::create::translation(joint)), rCentroid(position)
	{
		updateTransforms();
	}

	RigidBody(const TransformSpace& parent, const Point_t& joint, const Point_t& position) :
		parent(parent), rJoint(Math::create::translation(joint)), rCentroid(position)
	{
		updateTransforms();
	}

	RigidBody& emplace_back(const Point_t& joint, const Point_t& position)
	{
		return children.emplace_back(xformSpace, joint, position);
	}

	constexpr size_t size() const
	{
		size_t count = 1;

		for (const RigidBody& child : children)
		{
			count += child.size();
		}

		return count;
	}

	void copyTo(mat4x4* transforms, const mat4x4& offset, size_t& index) const
	{
		transforms[index] = jointSpace.transform * offset;
		++index;
		transforms[index] = jointSpace.transform * Math::create::translation(rCentroid) * offset;
		++index;

		for (const RigidBody& child : children)
		{
			child.copyTo(transforms, offset, index);
		}
	}

	void copyTo(mat4x4* transforms, const mat4x4& offset = Math::IDENTITY<mat4x4>) const
	{
		size_t index = 0;
		copyTo(transforms, offset, index);
	}

	void updateTransforms()
	{
		jointSpace.transform = parent.transform * rJoint;
		jointSpace.inverse = jointSpace.transform.inverse();

		xformSpace.transform = jointSpace.transform * Math::create::translation(rCentroid);
		xformSpace.inverse = xformSpace.transform.inverse();
	}

	void updateNetForce(const Acceleration_t& gravity)
	{
		float gravityIntensity = Math::length(gravity);
		vec3f aGravityDirection = gravity / gravityIntensity;
		vec3f rGravityDirection = Math::normalize(jointSpace.inverse * aGravityDirection);

		// Initialize net force
		rForce = rGravityDirection * gravityIntensity * mass;
		rMoment = Physics::getTorque( rCentroid, rForce ) - (0.75f * Math::length(rAngularVelocity) * rAngularVelocity);

		// Transform relative to joint
		//mat4x4 rTransform = Math::IDENTITY<mat4x4>;
		//Math::setRotation(rTransform, rOrientation);
		//Math::setTranslation(rTransform, rPosition);
		
		for (RigidBody& child : children)
		{
			child.updateNetForce(gravity);

			//Point_t rChildJoint = rTransform * child.rJoint;			// Child joint relative to joint
			//Point_t rChildForce = rTransform * child.rForce;			// Child force at joint relative to joint
			//Displacement_t rChildDisplacement = child.rJoint. - rJoint;	// Displacement relative to joint

			//rForce += Math::project(rChildForce, rChildDisplacement);
			//rMoment += Physics::getTorque(rChildDisplacement, rChildForce);
		}
	}

	void updateSimulation(const float deltaTime)
	{
		rAngularVelocity += rMoment * (deltaTime / mass);
		rOrientation *= Math::toQuat(rAngularVelocity * deltaTime);

		Math::setRotation(rJoint, rOrientation);
		updateTransforms();

		for (RigidBody& child : children)
		{
			child.updateSimulation(deltaTime);
		}
	}

	void update(const Acceleration_t& gravity, const float deltaTime)
	{
		updateNetForce(gravity);
		updateSimulation(deltaTime);
	}

	// Converts to matrix
	/*mat4x4 getTransform()
	{
		return Math::create::transform(position, orientation, scale);
	}*/

	virtual void onCollision(const CollisionData& collision, InstantCollider& collider) override
	{
		//ransform.origin -= collision.recoveryDirection * collision.signedDistance;
	}
};

/*struct RigidBodyConstraints
{
	vec4f minPosition = Math::MIN<vec3f>;
	vec4f maxPosition = Math::MAX<vec3f>;
	euler minRotation = Math::MIN<euler>;
	euler maxRotation = Math::MAX<euler>;
};

struct RigidBodyJoint
{
	// Regular Kinematics
	vec3f position;
	vec3f velocity = Math::ZERO<vec3f>;

	// Angular Kinematics
	quat orientation = Math::IDENTITY<quat>;
	vec3f angularVelocity = Math::ZERO<vec3f>;

	// Output
	vec3f absolutePosition = Math::ZERO<vec3f>;
	quat absoluteOrientation = Math::IDENTITY<quat>;

	const vec3f centerOfMass;
	const RigidBodyJoint* parent = nullptr;
	const float mass;
	const float damping;

	constexpr RigidBodyJoint(const vec3f& position, const vec3f& center, const float mass = 0.25f, const float damping = 0.1f)
		: position(position), centerOfMass(center), mass(mass), damping(damping) 
	{
		absoluteOrientation = orientation;
		absolutePosition = position;
	}

	RigidBodyJoint(const RigidBodyJoint& parent, const vec3f& position, const vec3f& center, const float mass = 0.25f, const float damping = 0.1f)
		: position(position), centerOfMass(center), mass(mass), damping(damping), parent(&parent) 
	{
		absoluteOrientation = parent.absoluteOrientation * orientation;
		absolutePosition = Math::rotate(position, parent.absoluteOrientation) + parent.absolutePosition;
	}
};

struct RigidBodyObject : public CollisionHandler
{
	mat4x4 offset;
	mat4x4 parent;
	mat4x4 absolute;
	RigidBodyJoint& joint;
	bool isColliding = false;
	bool isIntersecting = false;

	RigidBodyObject(RigidBodyJoint& joint, const mat4x4& offset) : joint(joint), offset(offset) 
	{ 
		parent = Math::create::transform(joint.absolutePosition, joint.absoluteOrientation, Math::ONES<vec3f>);
		absolute = parent * offset;
	}

	virtual void onCollision(const CollisionData& collision, InstantCollider& collider) override
	{
		isColliding = true;
	}

	void update()
	{
		Math::setRotation(parent, joint.absoluteOrientation);
		Math::setTranslation(parent, joint.absolutePosition);
		absolute = parent * offset;

		if (isColliding && !isIntersecting)
		{
			isIntersecting = true;
			joint.velocity *= -1.0f;
			joint.angularVelocity *= -1.0f;
			//std::cout << "Handling: Start\n";
		}
		else if(!isColliding)
		{
			isIntersecting = false;
			//std::cout << "Handling: Stop\n";
		}

		isColliding = false;
	}
};*/

namespace Physics
{
	/*static void applyRotation(quat& rotation, const vec3f& angularDisplacement)
	{
		const quat& vQuat = reinterpret_cast<const quat&>(angularDisplacement);
		rotation += 0.5f * rotation * vQuat;
		rotation /= rotation.norm();
	}

	static void testCollision(RigidBodyJoint& node, LinearCollider& collider)
	{

	}

	static void simulate(RigidBodyJoint& node, const float time)
	{
		quat inverse = node.parent == nullptr ? Math::IDENTITY<quat> : node.parent->absoluteOrientation.inverse();

		vec4f radius = Math::rotate(node.centerOfMass, node.orientation);
		Force_t netForce = Math::rotate(GRAVITATIONAL_ACCELERATION, inverse) * node.mass;

		float inertia = node.mass * Math::length2(radius);
		vec3f torque = Math::cross(radius, netForce);
		vec3f damping = node.damping * Math::length(node.angularVelocity) * node.angularVelocity;

		node.angularVelocity += (torque - damping) * (time / inertia);

		applyRotation(node.orientation, node.angularVelocity * time);

		// Calculate the absolute transform of the rigidbody
		if (node.parent != nullptr)
		{
			node.absoluteOrientation = node.parent->absoluteOrientation * node.orientation;
			node.absolutePosition = Math::rotate(node.position, node.parent->absoluteOrientation) + node.parent->absolutePosition;
		}
		else
		{
			node.absoluteOrientation = node.orientation;
			node.absolutePosition = node.position;
		}
	}*/
}

#endif