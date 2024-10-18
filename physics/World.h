#pragma once
#include <vector>
#include <stdlib.h>
#include <algorithm>

#include "Body.h"
#include "BodyDef.h"
#include "Joint.h"
#include "geometry/Shape.h"
#include "geometry/Collision.h"

#include "acceleration/BVH.h"

namespace fiz
{
	class World
	{
	public:
		unsigned int iters;

		std::vector<Shape*> shapes;

		std::vector<DynamicBody> dynamic_bodies;
		std::vector<StaticBody> static_bodies;
		BVH<StaticBody> static_bvh;

		std::vector<Joint*> joints;

		std::vector<ContactInfo> contacts;
		std::vector<ContactManifold> contact_manifolds;

		glm::vec3 gravity;

		void (*static_dynamic_collision_listener)(ContactInfo*);
		void (*dynamic_dynamic_collision_listener)(ContactInfo*);

		World() : gravity(0.0f, 0.0f, -9.8f), iters(4), static_bvh(&static_bodies), static_dynamic_collision_listener(nullptr), dynamic_dynamic_collision_listener(nullptr)
		{
			shapes.reserve(10);
			dynamic_bodies.reserve(600);
			static_bodies.reserve(100);
			joints.reserve(40);
		}
		~World() {}

		void buildBVH()
		{
			static_bvh.createBVH();
		}

		Body* createBody(BodyDef& bd)
		{
			if (bd.type == BodyType::DYNAMIC)
			{
				dynamic_bodies.emplace_back(bd.pos);
				DynamicBody* body = &dynamic_bodies[dynamic_bodies.size() - 1];
				//shapes.push_back(bd.shape);

				body->vel = bd.vel;
				body->orientation = bd.orientation;
				body->angular_vel = bd.angular_vel;
				body->density = bd.density;
				body->restitution = bd.restitution;
				body->friction = bd.friction;
				body->linear_damping = bd.linear_damping;
				body->angular_damping = bd.angular_damping;
				body->rotation_locked = bd.rotation_locked;
				body->addShape(bd.shape);

				body->updateMassProperties();
				body->updateOrientationMat();
				body->updateAABB();
				body->updateInverseInertiaWorld();

				return body;
			}
			else if (bd.type == BodyType::STATIC)
			{
				static_bodies.emplace_back(bd.pos);
				StaticBody* body = &static_bodies[static_bodies.size() - 1];
				//shapes.push_back(bd.shape);

				body->orientation = bd.orientation;
				body->restitution = bd.restitution;
				body->friction = bd.friction;
				body->is_sensor = bd.is_sensor;
				body->addShape(bd.shape);

				body->updateOrientationMat();
				body->updateAABB();

				return body;
			}
			return nullptr;
		}

		void addJoint(Joint* joint)
		{
			joints.push_back(joint);
		}

		void step(float delta_t)
		{
			float dt = delta_t / (float)iters;
			for (unsigned int x = 0; x < iters; ++x)
			{
				// apply joint forces
				for (unsigned int i = 0; i < joints.size(); ++i)
				{
					joints[i]->applyForces();
				}

				// update bodies
				for (unsigned int i = 0; i < dynamic_bodies.size(); ++i)
				{
					if (dynamic_bodies[i].is_awake)
						dynamic_bodies[i].applyForce(gravity * dynamic_bodies[i].mass);

					dynamic_bodies[i].update(dt);
				}

				// dynamic vs dynamic collision detection
				for (unsigned int i = 0; i < dynamic_bodies.size(); ++i)
				{
					if (dynamic_bodies[i].is_awake)
					{
						ContactInfo contact = checkCollisionGround(&dynamic_bodies[i]);
						if (contact.collided)
							contact.solveContactStatic();
					}

					for (unsigned int j = 0; j < i; ++j)
					{
						if (!dynamic_bodies[i].is_awake && !dynamic_bodies[j].is_awake)
							continue;

						if (!dynamic_bodies[i].aabb.intersects(dynamic_bodies[j].aabb))
							continue;

						// check for collision between bodies
						if (dynamic_bodies[i].shapes[0]->shape_type == ShapeType::SPHERE_TYPE &&
							dynamic_bodies[j].shapes[0]->shape_type == ShapeType::SPHERE_TYPE)
						{
							ContactInfo contact = checkCollisionSphereSphere(&dynamic_bodies[i], &dynamic_bodies[j]);
							if (contact.collided)
							{
								if (dynamic_dynamic_collision_listener != nullptr)
									dynamic_dynamic_collision_listener(&contact);
								contact.solveContactDynamic();
							}
						}
						else
						{
							bool intersecting = GJK(&dynamic_bodies[i], &dynamic_bodies[j], glm::vec3(1.0f, 0.0f, 0.0f));
							if (intersecting)
							{
								ContactInfo contact = EPA(&dynamic_bodies[i], &dynamic_bodies[j]);
								if (contact.collided)
								{
									if (dynamic_dynamic_collision_listener != nullptr)
										dynamic_dynamic_collision_listener(&contact);
									contact.solveContactDynamic();
								}
							}
						}
					}
				}

				if (static_bodies.size() > 0 && static_bvh.is_built)
				{
					for (unsigned int i = 0; i < dynamic_bodies.size(); ++i)
					{
						if (!dynamic_bodies[i].is_awake)
							continue;

						std::vector<int> bodies;
						static_bvh.traverse(dynamic_bodies[i].aabb, bodies);

						for (unsigned int x = 0; x < bodies.size(); ++x)
						{
							solveDynamicStatic(dynamic_bodies[i], static_bodies[bodies[x]]);
						}
					}
				}
				else
				{
					// dynamic vs static collision detection
					for (unsigned int i = 0; i < dynamic_bodies.size(); ++i)
					{
						if (!dynamic_bodies[i].is_awake)
							continue;

						for (unsigned int j = 0; j < static_bodies.size(); ++j)
						{
							if (!dynamic_bodies[i].aabb.intersects(static_bodies[j].aabb))
								continue;
							
							solveDynamicStatic(dynamic_bodies[i], static_bodies[j]);
						}
					}
				}

				for (unsigned int i = 0; i < dynamic_bodies.size(); ++i)
				{
					dynamic_bodies[i].updateSleep(dt);
				}
				////std::cout << "contacts: " << contacts.size() << std::endl;
				//sort(contacts.begin(), contacts.end(), [](ContactInfo& a, ContactInfo& b) {return a.depth > b.depth; });

				//// solve contacts
				//for (unsigned int i = 0; i < contacts.size(); ++i)
				//{
				//	contacts[i].solveContact2();
				//}
			}
		}
	private:
		inline glm::vec3 intersectionNormal(Body* a, Body* b)
		{
			Sphere* s_a = (Sphere*)a->shapes[0];
			Sphere* s_b = (Sphere*)b->shapes[0];
			glm::vec3 dist = b->pos - a->pos;
			glm::vec3 normal = glm::normalize(dist);
			float dist2 = dist.x * dist.x + dist.y * dist.y + dist.z * dist.z;
			float r2 = s_a->rad + s_b->rad;
			if (dist2 < r2 * r2 && a->pos != b->pos)
			{
				float len = r2 - std::sqrtf(dist2);
				normal *= len;
				return normal;
			}
			return glm::vec3(0.0f, 0.0f, 0.0f);
		}
		inline void solveCollision(float m1, float m2, glm::vec3& v1, glm::vec3& v2, glm::vec3& normal)
		{
			static float friction = 0.0f;
			static float restitution = 0.0f;

			float fr = 1 - friction;

			normal = normalize(normal);
			glm::vec3 lat1 = glm::normalize(glm::cross(normal, glm::vec3(0.57f, 0.51f, 0.13f)));
			glm::vec3 lat2 = glm::normalize(glm::cross(normal, lat1));
			
			float m1m2 = m1 + m2;

			float u1 = glm::dot(normal, v1);
			float u2 = glm::dot(normal, v2);
			float l1_1 = glm::dot(lat1, v1);
			float l2_1 = glm::dot(lat1, v2);
			float l1_2 = glm::dot(lat2, v1);
			float l2_2 = glm::dot(lat2, v2);

			float v_1 = ((m1 - m2) / m1m2) * u1 + ((m2 * 2) / m1m2) * u2;
			float v_2 = ((m1 * 2) / m1m2) * u1 + ((m2 - m1) / m1m2) * u2;

			v1 = (normal * v_1 * restitution) + (lat1 * l1_1 * fr) + (lat2 * l1_2 * fr);
			v2 = (normal * v_1 * restitution) + (lat1 * l2_1 * fr) + (lat2 * l2_2 * fr);
		}
		inline void solveDynamicStatic(DynamicBody& dynamic_body, StaticBody& static_body)
		{
			if (dynamic_body.shapes[0]->shape_type == ShapeType::SPHERE_TYPE &&
				static_body.shapes[0]->shape_type == ShapeType::SPHERE_TYPE)
			{
				ContactInfo contact = checkCollisionSphereSphere(&static_body, &dynamic_body);
				if (contact.collided)
				{
					if (static_dynamic_collision_listener != nullptr)
						static_dynamic_collision_listener(&contact);
					if (!static_body.is_sensor)
						contact.solveContactStatic();
				}
			}
			else
			{
				bool intersecting = GJK(&static_body, &dynamic_body, glm::vec3(1.0f, 0.0f, 0.0f));
				if (intersecting)
				{
					ContactInfo contact = EPA(&static_body, &dynamic_body);
					if (contact.collided)
					{
						if (static_dynamic_collision_listener != nullptr)
							static_dynamic_collision_listener(&contact);
						if (!static_body.is_sensor)
							contact.solveContactStatic();
					}
				}
			}
		}
	};
}