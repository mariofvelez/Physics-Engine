#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "geometry/Shape.h"

namespace fiz
{
	struct BodyDef
	{
		BodyType type;

		glm::vec3 pos;
		glm::vec3 vel;

		glm::quat orientation;
		glm::vec3 angular_vel;

		float linear_damping;
		float angular_damping;

		float density;

		float friction;
		float restitution;

		bool rotation_locked;

		Shape* shape;

		BodyDef() : type(BodyType::DYNAMIC), pos(0.0f), vel(0.0f), orientation(0.0f, 0.0f, 0.0f, 0.0f), angular_vel(0.0f), linear_damping(1.0f), angular_damping(1.0f), density(1.0f), friction(0.2f), restitution(0.2f), rotation_locked(false), shape(nullptr)
		{

		}
	};
}