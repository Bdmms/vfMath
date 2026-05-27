#pragma once
#ifndef VF_PHYSICS_HPP
#define VF_PHYSICS_HPP

#include "matrix_math.hpp"

namespace vf::physics
{
	using namespace Math;

	template<typename T>
	struct LinearState
	{
		vec4<T> position = axis::W<vec4<T>>;
		vec4<T> momentum = ZERO<vec4<T>>;

		constexpr LinearState<T> operator+=( const LinearState<T>& b )
		{
			position += b.position;
			momentum += b.momentum;
			return *this;
		}
	};

	template<typename T>
	[[nodiscard]] constexpr static LinearState<T> operator*( const LinearState<T>& state, T h )
	{
		return { state.position * h, state.momentum * h };
	}

	template<typename T>
	[[nodiscard]] constexpr static LinearState<T> operator*( T h, const LinearState<T>& state )
	{
		return { state.position * h, state.momentum * h };
	}

	template<typename T>
	[[nodiscard]] constexpr static LinearState<T> operator+( const LinearState<T>& a, const LinearState<T>& b )
	{
		return { a.position + b.position, a.momentum + b.momentum };
	}

	template<typename T>
	struct AngularState
	{
		quat<T> orientation = IDENTITY<quat<T>>;
		AxisAngle<T> momentum = IDENTITY<AxisAngle<T>>;

		constexpr AngularState<T> operator+=( const AngularState<T>& b )
		{
			orientation += b.orientation;
			momentum += b.momentum;
			return *this;
		}
	};

	template<typename T>
	[[nodiscard]] constexpr static AngularState<T> operator*( const AngularState<T>& state, T h )
	{
		return { state.orientation * h, state.momentum * h };
	}

	template<typename T>
	[[nodiscard]] constexpr static AngularState<T> operator*( T h, const AngularState<T>& state )
	{
		return { state.orientation * h, state.momentum * h };
	}

	template<typename T>
	[[nodiscard]] constexpr static AngularState<T> operator+( const AngularState<T>& a, const AngularState<T>& b )
	{
		return { a.orientation + b.orientation, a.momentum + b.momentum };
	}

	template<typename T>
	struct MotionState
	{
		LinearState<T> linear;
		AngularState<T> angular;

		constexpr MotionState<T> operator+=( const MotionState<T>& b )
		{
			linear += b.linear;
			angular += b.angular;
			return *this;
		}
	};

	template<typename T>
	[[nodiscard]] constexpr static MotionState<T> operator*( const MotionState<T>& state, T h )
	{
		return { state.linear * h, state.angular * h };
	}

	template<typename T>
	[[nodiscard]] constexpr static MotionState<T> operator*( T h, const MotionState<T>& state )
	{
		return { state.linear * h, state.angular * h };
	}

	template<typename T>
	[[nodiscard]] constexpr static MotionState<T> operator+( const MotionState<T>& a, const MotionState<T>& b )
	{
		return { a.linear + b.linear, a.angular + b.angular };
	}

	template<typename T>
	struct LinearMotion
	{
		vec4<T> velocity;
		vec4<T> netForce;
	};

	template<typename T>
	struct AngularMotion
	{
		vec4<T> velocity;
		vec4<T> netTorque;
	};

	template<typename T>
	struct Motion
	{
		LinearMotion<T> linear;
		AngularMotion<T> angular;
	};

	template<typename T> constexpr LinearState<T> DEFAULT_LINEAR_STATE;
	template<typename T> constexpr AngularState<T> DEFAULT_ANGULAR_STATE;
	template<typename T> constexpr MotionState<T> DEFAULT_STATE;

	/**
		* Gets the linear velocity from the current state.
		* @param state - current linear motion state
		* @param mass - mass associated with state
		* @return linear velocity
	*/
	template<typename T>
	constexpr static vec4<T> getLinearVelocity( const LinearState<T>& state, T mass )
	{
		return state.momentum / mass;
	}

	/**
		* Gets the angular velocity from the current state.
		* @param state - current angular motion state
		* @param invInertiaTensor - transform to convert rotation to inertial space
		* @return angular velocity
	*/
	template<typename T>
	constexpr static AxisAngle<T> getAngularVelocity( const AngularState<T>& state, const mat4<T>& invInertiaTensor )
	{
		mat4<T> rotation = create::rotation( state.orientation );
		mat4<T> I_inv = rotation * invInertiaTensor * transpose( rotation );

		return I_inv * state.momentum;
	}

	/**
		* Calculates the delta linear motion state.
		* @param current - current linear motion state
		* @param motion - linear motion applied to state
		* @return delta linear state
	*/
	template<typename T>
	constexpr static LinearState<T> getDelta( const LinearState<T>& current, const LinearMotion<T>& motion )
	{
		return { motion.velocity, motion.netForce };
	}

	/**
		* Calculates the delta angular motion state.
		* @param current - current angular motion state
		* @param motion - angular motion applied to state
		* @return delta angular state
	*/
	template<typename T>
	constexpr static AngularState<T> getDelta( const AngularState<T>& current, const AngularMotion<T>& motion )
	{
		quat<T> dOrientation = { motion.velocity.x, motion.velocity.y, motion.velocity.z, ZERO<T> };
		return { HALF<T> * ( dOrientation * current.orientation ), motion.netTorque };
	}

	/**
		* Calculates the delta motion state.
		* @param current - current motion state
		* @param motion - motion applied to state
		* @return delta motion state
	*/
	template<typename T>
	constexpr static MotionState<T> getDelta( const MotionState<T>& current, const Motion<T>& motion )
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
	template<typename MotionType, typename T, typename Lambda>
	void simulateSingularMotion( MotionType& current, T deltaTime, Lambda motionSupplier )
	{
		MotionType rk0 = getDelta( current, motionSupplier( current ) );

		MotionType state = current + rk0 * ( deltaTime * HALF<T> );
		MotionType rk1 = getDelta( state, motionSupplier( state ) );

		state = current + rk1 * ( deltaTime * HALF<T> );
		MotionType rk2 = getDelta( state, motionSupplier( state ) );

		state = current + rk2 * deltaTime * HALF<T>;
		MotionType rk3 = getDelta( state, motionSupplier( state ) );

		current += ( rk0 + TWO<T> * rk1 + TWO<T> * rk2 + rk3 ) * ( deltaTime / 6.0f );
	}
}

#endif