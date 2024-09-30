#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include "geometry/Shape.h"

#include <vector>

namespace fiz
{
	enum BodyType
	{
		STATIC,
		DYNAMIC
	};

	class Body
	{
	public:
		BodyType type;

		glm::vec3 pos;

		glm::quat orientation;
		glm::mat3 orientation_mat;
		glm::mat3 orientation_mat_inv;

		float friction;
		float restitution;

		AABB aabb;
		std::vector<Shape*> shapes;

		Body() : Body(glm::vec3(0.0f))
		{

		}
		Body(glm::vec3 pos) : pos(pos), orientation(0.0f, 0.0f, 0.0f, 0.0f), orientation_mat(1.0f), orientation_mat_inv(1.0f), friction(0.2f), restitution(0.2f)
		{

		}

		void addShape(Shape* shape)
		{
			shapes.push_back(shape);
		}

		void updateOrientationMat()
		{
			orientation_mat = glm::mat3_cast(orientation);
			orientation_mat_inv = glm::transpose(orientation_mat);
		}

		void updateAABB()
		{
			shapes[0]->setAABB(&aabb, pos, orientation_mat);
			for (unsigned int i = 1; i < shapes.size(); ++i)
			{
				AABB aabb1(glm::vec3(0.0f), glm::vec3(0.0f));
				shapes[i]->setAABB(&aabb1, pos, orientation_mat);
				aabb.combine(aabb1);
			}
		}

		glm::vec3 getWorldPos(glm::vec3 pos)
		{
			return orientation_mat * pos + this->pos;
		}

		glm::vec3 getLocalPos(glm::vec3 pos)
		{
			return orientation_mat_inv * (pos - this->pos);
		}

		glm::vec3 getWorldVec(glm::vec3 vec)
		{
			return orientation_mat * vec;
		}

		glm::vec3 getLocalVec(glm::vec3 vec)
		{
			return orientation_mat_inv * vec;
		}
	};

	class StaticBody : public Body
	{
	public:
		StaticBody() : StaticBody(glm::vec3(0.0f))
		{
			type = STATIC;
		}
		StaticBody(glm::vec3 pos) : Body(pos)
		{
			type = STATIC;
		}
	};

	class DynamicBody : public Body
	{
	public:
		glm::vec3 vel;

		glm::vec3 angular_vel;

		float linear_damping;
		float angular_damping;

		float mass;
		glm::vec3 centroid;

		glm::mat3 inertia;
		glm::mat3 inertia_inv;
		glm::mat3 inertia_inv_world;

		float density;

		glm::vec3 forces; // world coordinates
		glm::vec3 torques; // world coordinates

		unsigned int still_frames;

		bool rotation_locked;
		bool is_awake;

		DynamicBody() : DynamicBody(glm::vec3(0.0f, 0.0f, 0.0f))
		{
			type = DYNAMIC;
		}
		DynamicBody(glm::vec3 pos) : Body(pos), vel(0.0f), angular_vel(0.0f), linear_damping(1.0f), angular_damping(1.0f), mass(0.0f), centroid(0.0f), inertia(1.0f), inertia_inv(1.0f), inertia_inv_world(1.0f), density(1.0f), forces(0.0f), torques(0.0f), still_frames(0), rotation_locked(false), is_awake(true)
		{
			type = DYNAMIC;
		}

		void setDensity(float density)
		{
			this->density = density;
		}

		void updateMassProperties()
		{
			// update mass and centroid
			mass = 0.0f;
			centroid = glm::vec3(0.0f);
			for (unsigned int i = 0; i < shapes.size(); ++i)
			{
				shapes[i]->computeMassProperties();
				float shape_mass = shapes[i]->volume * density;
				mass += shape_mass;
				centroid += shape_mass * shapes[i]->centroid;
			}
			centroid /= mass;

			// update inertia
			float Ixx = 0, Iyy = 0, Izz = 0;
			float Ixy = 0, Ixz = 0, Iyz = 0;
			for (unsigned int i = 0; i < shapes.size(); ++i)
			{
				glm::vec3 cg = shapes[i]->centroid - centroid; // center of gravity relative to body
				float shape_mass = shapes[i]->volume * density;
				glm::vec3 loc_inertia = shapes[i]->local_inertia;
				glm::vec3 loc_products = shapes[i]->local_products;

				Ixx += loc_inertia.x;// +shape_mass * (cg.y * cg.y + cg.z * cg.z);
				Iyy += loc_inertia.y;// +shape_mass * (cg.x * cg.x + cg.z * cg.z);
				Izz += loc_inertia.z;// +shape_mass * (cg.x * cg.x + cg.y * cg.y);
				Ixy += loc_products.x;// shape_mass* cg.x* cg.y;
				Ixz += loc_products.y;// shape_mass* cg.x* cg.z;
				Iyz += loc_products.z;// shape_mass* cg.y* cg.z;
			}

			inertia[0][0] = Ixx;
			inertia[0][1] = -Ixy;
			inertia[0][2] = -Ixz;
			inertia[1][0] = -Ixy;
			inertia[1][1] = Iyy;
			inertia[1][2] = -Iyz;
			inertia[2][0] = -Ixz;
			inertia[2][1] = -Iyz;
			inertia[2][2] = Izz;

			inertia_inv = glm::inverse(inertia);
		}

		void updateInverseInertiaWorld()
		{
			// definitely needs optimization
			inertia_inv_world = glm::transpose(orientation_mat_inv) * inertia_inv * orientation_mat_inv;
		}

		void updateSleep(float dt)
		{
			float motion = glm::max(glm::dot(vel, vel), glm::dot(angular_vel, angular_vel));
			if (motion < 0.005f)
				still_frames++;
			else
				still_frames = 0;

			if (still_frames >= 80)
				is_awake = false;
		}

		void setAwake()
		{
			still_frames = 0;
			is_awake = true;
		}

		void update(float dt)
		{
			if (!is_awake)
				return;

			updateOrientationMat();
			updateInverseInertiaWorld();

			// update forces
			glm::vec3 acceleration = forces / mass;
			vel += acceleration * dt;
			vel *= linear_damping;
			pos += vel * dt;

			// update torques
			if (!rotation_locked)
			{
				glm::vec3 angular_acceleration = inertia_inv_world * torques;
				angular_vel += angular_acceleration * dt;
				angular_vel *= angular_damping;
				glm::vec3 d_avel = angular_vel * dt;
				float angle = glm::sin(0.5f * glm::length(d_avel));
				if (angle != 0.0f)
				{
					d_avel = glm::normalize(d_avel);
					float sint = glm::sin(angle);
					glm::quat rot = glm::quat(glm::cos(angle), d_avel.x * sint, d_avel.y * sint, d_avel.z * sint);
					orientation = rot * orientation;
					//orientation = orientation + 0.5f * rot * dt;
					orientation = glm::normalize(orientation);
				}

				updateOrientationMat();
				updateInverseInertiaWorld();
			}

			updateAABB();

			resetForces();
		}

		glm::vec3 getVelocityWorld(glm::vec3 pos)
		{
			return vel + glm::cross(angular_vel, pos - this->pos);
		}

		/**
		Applies a force at the centroid
		Force is in world coordinates
		*/
		void applyForce(glm::vec3 force)
		{
			forces += force;
		}

		/**
		Applies a force at a given position
		Force is in world coordinates
		Pos is in local coordinates
		*/
		void applyForceLocal(glm::vec3 force, glm::vec3 pos)
		{
			forces += force;
			if (!rotation_locked)
			{
				glm::vec3 r = getWorldPos(pos) - this->pos;
				torques += glm::cross(r, force);
			}
		}

		/**
		Applies a force at a given position
		Force is in world coordinates
		Pos is in world coordinates
		*/
		void applyForceWorld(glm::vec3 force, glm::vec3 pos)
		{
			forces += force;
			if (!rotation_locked)
			{
				torques += glm::cross(pos - this->pos, force);
			}
		}

		void applyTorque(glm::vec3 torque)
		{
			if (!rotation_locked)
				torques += torque;
		}

		void resetForces()
		{
			forces = glm::vec3(0.0f, 0.0f, 0.0f);
			torques = glm::vec3(0.0f, 0.0f, 0.0f);
		}

		void applyImpulse(glm::vec3 impulse, glm::vec3 pos)
		{
			glm::vec3 d_vel = impulse * (1.0f / mass);
			vel += d_vel;

			if (!rotation_locked)
			{
				glm::vec3 impulsive_torque = glm::cross(pos - this->pos, impulse);
				glm::vec3 d_angular_vel = inertia_inv_world * impulsive_torque;

				angular_vel += d_angular_vel;
			}
		}

	};
}