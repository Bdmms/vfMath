#pragma once
#ifndef VF_PHYSICS_HPP
#define VF_PHYSICS_HPP

#include "MatrixMath.hpp"

struct LinearState
{
	vec4f position = Math::ZERO<vec4f>;
	vec3f momentum = Math::ZERO<vec3f>;

	LinearState operator+=( const LinearState& b )
	{
		position += b.position;
		momentum += b.momentum;
		return *this;
	}
};

[[nodiscard]] static LinearState operator*( const LinearState& state, float h )
{
	return { state.position * h, state.momentum * h };
}

[[nodiscard]] static LinearState operator*( float h, const LinearState& state )
{
	return { state.position * h, state.momentum * h };
}

[[nodiscard]] static LinearState operator+( const LinearState& a, const LinearState& b )
{
	return { a.position + b.position, a.momentum + b.momentum };
}

struct AngularState
{
	quat orientation = Math::IDENTITY<quat>;
	AxisAngle momentum = Math::IDENTITY<AxisAngle>;

	AngularState operator+=( const AngularState& b )
	{
		orientation += b.orientation;
		momentum += b.momentum;
		return *this;
	}
};

[[nodiscard]] static AngularState operator*( const AngularState& state, float h )
{
	return { state.orientation * h, state.momentum * h };
}

[[nodiscard]] static AngularState operator*( float h, const AngularState& state )
{
	return { state.orientation * h, state.momentum * h };
}

[[nodiscard]] static AngularState operator+( const AngularState& a, const AngularState& b )
{
	return { a.orientation + b.orientation, a.momentum + b.momentum };
}

struct MotionState
{
	LinearState linear;
	AngularState angular;

	MotionState operator+=( const MotionState& b )
	{
		linear += b.linear;
		angular += b.angular;
		return *this;
	}
};

[[nodiscard]] static MotionState operator*( const MotionState& state, float h )
{
	return { state.linear * h, state.angular * h };
}

[[nodiscard]] static MotionState operator*( float h, const MotionState& state )
{
	return { state.linear * h, state.angular * h };
}

[[nodiscard]] static MotionState operator+( const MotionState& a, const MotionState& b )
{
	return { a.linear + b.linear, a.angular + b.angular };
}

struct LinearMotion
{
	vec3f velocity;
	vec3f netForce;
};

struct AngularMotion
{
	vec3f velocity;
	vec3f netTorque;
};

struct Motion
{
	LinearMotion linear;
	AngularMotion angular;
};

namespace Physics
{
	constexpr LinearState DEFAULT_LINEAR_STATE;
	constexpr AngularState DEFAULT_ANGULAR_STATE;
	constexpr MotionState DEFAULT_STATE;

	/**
	 * Gets the linear velocity from the current state.
	 * @param state - current linear motion state
	 * @param mass - mass associated with state
	 * @return linear velocity
	*/
	inline static vec3f getLinearVelocity( const LinearState& state, float mass )
	{
		return state.momentum / mass;
	}

	/**
	 * Gets the angular velocity from the current state.
	 * @param state - current angular motion state
	 * @param invInertiaTensor - transform to convert rotation to inertial space
	 * @return angular velocity
	*/
	inline static AxisAngle getAngularVelocity( const AngularState& state, const mat4f& invInertiaTensor )
	{
		mat4f rotation = Math::create::rotation( state.orientation );
		mat4f I_inv = rotation * invInertiaTensor * rotation.transpose();

		return I_inv * state.momentum;
	}

	/**
	 * Calculates the delta linear motion state.
	 * @param current - current linear motion state
	 * @param motion - linear motion applied to state
	 * @return delta linear state
	*/
	constexpr LinearState getDelta( const LinearState& current, const LinearMotion& motion )
	{
		return { motion.velocity, motion.netForce };
	}

	/**
	 * Calculates the delta angular motion state.
	 * @param current - current angular motion state
	 * @param motion - angular motion applied to state
	 * @return delta angular state
	*/
	inline static AngularState getDelta( const AngularState& current, const AngularMotion& motion )
	{
		quat dOrientation = { motion.velocity.x, motion.velocity.y, motion.velocity.z, 0.0f };
		return { 0.5f * ( dOrientation * current.orientation ), motion.netTorque };
	}

	/**
	 * Calculates the delta motion state.
	 * @param current - current motion state
	 * @param motion - motion applied to state
	 * @return delta motion state
	*/
	inline static MotionState getDelta( const MotionState& current, const Motion& motion )
	{
		return { getDelta( current.linear, motion.linear ), getDelta( current.angular, motion.angular ) };
	}

	/**
	 * Simulates the motion applied to the current state of a period of time using RK4.
	 * This only simulates motion for one object independent of any other object.
	 * @tparam MotionType - type of motion to simulate
	 * @tparam Lambda - lambda type supplying motion
	 * @param current - current motion state to update
	 * @param deltaTime - period of time
	 * @param motionSupplier - function to supply motion based on current state
	*/
	template<typename MotionType, typename Lambda>
	void simulateSingularMotion( MotionType& current, float deltaTime, Lambda motionSupplier )
	{
		MotionType rk0 = getDelta( current, motionSupplier( current ) );

		MotionType state = current + rk0 * ( deltaTime * 0.5f );
		MotionType rk1 = getDelta( state, motionSupplier( state ) );

		state = current + rk1 * ( deltaTime * 0.5f );
		MotionType rk2 = getDelta( state, motionSupplier( state ) );

		state = current + rk2 * deltaTime * 0.5f;
		MotionType rk3 = getDelta( state, motionSupplier( state ) );

		current += ( rk0 + 2.0f * rk1 + 2.0f * rk2 + rk3 ) * ( deltaTime / 6.0f );
	}
}

#endif