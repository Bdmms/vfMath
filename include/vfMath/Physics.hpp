#pragma once
#ifndef VF_PHYSICS_HPP
#define VF_PHYSICS_HPP

#include "Collision.hpp"

typedef vec4f Point_t;			// x metres
typedef vec3f Displacement_t;	// x metres
typedef vec3f Velocity_t;		// x metres / s
typedef vec3f Acceleration_t;	// x metres / s^2
typedef vec3f Force_t;			// x kg * units / s^2

//typedef mat4x4 Rotation_t;			// x radians
//typedef mat4x4 AngularVelocity_t;		// x radians / s
//typedef mat4x4 AngularAccel_t;		// x radians / s^2
//typedef mat4x4 Torque_t;				// x kg * radians / s^2

constexpr Acceleration_t GRAVITATIONAL_ACCELERATION = { 0.0f, -9.81f, 0.0f };

struct RigidBodyConstraints
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
};

namespace Physics
{
	static void applyRotation(quat& rotation, const vec3f& angularDisplacement)
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
	}
}

#endif