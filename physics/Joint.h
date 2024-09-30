#pragma once

#include "Body.h"
#include "geometry/Collision.h"

namespace fiz
{
	class Joint
	{
	public:
		virtual void applyForces() = 0;
	};

	class AnchoredSpringJoint : public Joint
	{
	public:
		DynamicBody* body;
		glm::vec3 local;
		glm::vec3 anchor;
		float spring_constant;
		float length;

		AnchoredSpringJoint(float spring_constant, float length) : spring_constant(spring_constant), length(length), local(0.0f), anchor(0.0f)
		{
			
		}

		void applyForces()
		{
			glm::vec3 world = body->getWorldPos(local);
			glm::vec3 dir = anchor - world; // from anchor to local
			float dir_len = glm::length(dir);
			dir = glm::normalize(dir);
			float diff = dir_len - length;

			glm::vec3 force = dir * (diff * spring_constant);
			body->applyForceLocal(force, local);
		}
	};

	class SpringJoint : public Joint
	{
	public:
		DynamicBody* a;
		DynamicBody* b;
		glm::vec3 local_a;
		glm::vec3 local_b;

		float spring_constant;
		float length;

		SpringJoint(float spring_constant, float length) : spring_constant(spring_constant), length(length), local_a(0.0f), local_b(0.0f)
		{
			
		}

		void applyForces()
		{
			glm::vec3 world_a = a->getWorldPos(local_a);
			glm::vec3 world_b = b->getWorldPos(local_b);
			glm::vec3 dir = world_b - world_a; // from a to b
			float dir_len = glm::length(dir);
			dir = glm::normalize(dir);
			float diff = dir_len - length;

			glm::vec3 force = dir * (diff * spring_constant);
			a->applyForceLocal(force, local_a);
			b->applyForceLocal(-force, local_b);
		}
	};

	class AnchoredBallJoint : public Joint
	{
	public:
		DynamicBody* body;
		glm::vec3 local;
		glm::vec3 anchor;

		AnchoredBallJoint() : body(nullptr), local(0.0f), anchor(0.0f)
		{

		}

		void applyForces()
		{
			// reset pos
			glm::vec3 dir = body->getWorldPos(local) - anchor;
			body->pos -= dir;

			// create contact
			ContactInfo contact;
			contact.collided = true;
			contact.body_a = nullptr;
			contact.body_b = body;
			contact.poc = anchor;
			contact.normal = -glm::normalize(body->getVelocityWorld(anchor));
			if (contact.normal.x != contact.normal.x)
				contact.normal = glm::vec3(0.0f, 0.0f, 1.0f);
			contact.depth = 0.0f;
			contact.restitution = 0.0f;
			contact.friction = 1.0f;
			contact.solveContactStatic();
		}
	};

	class BallJoint : public Joint
	{
	public:
		DynamicBody* a;
		DynamicBody* b;
		glm::vec3 local_a;
		glm::vec3 local_b;

		BallJoint() : a(nullptr), b(nullptr), local_a(0.0f), local_b(0.0f)
		{

		}

		void applyForces()
		{
			// reset pos
			glm::vec3 world_a = a->getWorldPos(local_a);
			glm::vec3 world_b = b->getWorldPos(local_b);
			glm::vec3 dir = world_b - world_a;
			a->pos += dir * 0.5f;
			b->pos -= dir * 0.5f;

			ContactInfo contact;
			contact.collided = true;
			contact.body_a = a;
			contact.body_b = b;
			contact.poc = (world_a + world_b) * 0.5f;
			contact.normal = -glm::normalize(b->getVelocityWorld(contact.poc) - a->getVelocityWorld(contact.poc));
			if (contact.normal.x != contact.normal.x)
				contact.normal = glm::vec3(0.0f, 0.0f, 1.0f);
			contact.depth = 0.0f;
			contact.restitution = 0.0f;
			contact.friction = 1.0f;
			contact.solveContactDynamic();
		}
	};

	class AnchoredRevoluteJoint : public Joint
	{
	public:
		DynamicBody* body;
		glm::vec3 local;
		glm::vec3 local_axis;

		glm::vec3 anchor;
		glm::vec3 anchor_axis;

		AnchoredRevoluteJoint() : body(nullptr), local(0.0f), local_axis(0.0f, 0.0f, 1.0f), anchor(0.0f), anchor_axis(0.0f, 0.0f, 1.0f)
		{

		}

		void applyForces()
		{
			// position constraint
			glm::vec3 dir = body->getWorldPos(local) - anchor;
			body->pos -= dir;
		}
	};

	class RevoluteJoint : public Joint
	{
	public:
		DynamicBody* a;
		DynamicBody* b;
		glm::vec3 local_a;
		glm::vec3 local_b;
		glm::vec3 local_axis_a;
		glm::vec3 local_axis_b;

		RevoluteJoint() : a(nullptr), b(nullptr), local_a(0.0f), local_b(0.0f), local_axis_a(0.0f, 0.0f, 1.0f), local_axis_b(0.0f, 0.0f, 1.0f)
		{

		}

		void applyForces()
		{
			// solve position constraint
			glm::vec3 world_a = a->getWorldPos(local_a);
			glm::vec3 world_b = b->getWorldPos(local_b);
			glm::vec3 dir = world_b - world_a;
			a->pos += dir * 0.5f;
			b->pos -= dir * 0.5f;

			// solve angle constraint
			glm::vec3 world_axis_a = glm::normalize(a->getWorldVec(local_axis_a));
			glm::vec3 world_axis_b = glm::normalize(b->getWorldVec(local_axis_b));
			
			glm::quat rot;
			float cos = glm::dot(world_axis_a, world_axis_b);
			glm::vec3 rot_axis;
			if (cos < -0.9999f) // almost opposite direction
			{
				rot_axis = glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), world_axis_a);
				if (glm::dot(rot_axis, rot_axis) < 0.01f)
					rot_axis = glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), world_axis_a);

				rot_axis = glm::normalize(rot_axis);
				rot = glm::angleAxis(glm::pi<float>(), rot_axis);
			}
			else
			{
				rot_axis = glm::cross(world_axis_a, world_axis_b);

				float s = glm::sqrt((1 + cos) * 2.0f);
				float inv_s = 1.0f / s;

				rot = glm::quat(s * 0.5f, rot_axis.x * inv_s, rot_axis.y * inv_s, rot_axis.z * inv_s);
			}
			
			glm::quat rot_a = glm::mix(glm::quat(0.0f, 0.0f, 0.0f, 0.0f), rot, 0.5f);
			glm::quat rot_b = glm::quat(rot_a.w, -rot_a.x, -rot_a.y, -rot_a.z);

			a->orientation = glm::normalize(rot * a->orientation);
			//a->updateOrientationMat();
			a->updateInverseInertiaWorld();
			/*b->orientation = glm::normalize(rot_b * b->orientation);

			a->updateOrientationMat();
			a->updateInverseInertiaWorld();
			b->updateOrientationMat();
			b->updateInverseInertiaWorld();*/

			// solve velocity constraint
			a->angular_vel = world_axis_a * glm::dot(a->angular_vel, world_axis_a);
			b->angular_vel = world_axis_b * glm::dot(b->angular_vel, world_axis_b);
			a->setAwake();
			b->setAwake();
		}
	};
}