#pragma once

#include <vector>

#include "../Body.h"
#include "Shape.h"

#define trip(a, b) glm::cross(glm::cross(a, b), a)

namespace fiz
{
	struct ContactInfo
	{
		bool collided;

		Body* body_a; // may be static or dynamic
		Body* body_b; // always dynamic

		glm::vec3 poc;
		glm::vec3 poc_a;
		glm::vec3 poc_b;
		glm::vec3 normal; // from a to b
		float depth;

		float friction;
		float restitution;

		void set(ContactInfo& other)
		{
			collided = other.collided;

			body_a = other.body_a;
			body_b = other.body_b;

			poc = other.poc;
			normal = other.normal;
			depth = other.depth;
		}

		inline glm::mat3 createSkew(glm::vec3 v)
		{
			glm::mat3 mat;
			mat[0].x = 0.0f;
			mat[0].y = v.z;
			mat[0].z = -v.y;
			mat[1].x = -v.z;
			mat[1].y = 0.0f;
			mat[1].z = v.x;
			mat[2].x = v.y;
			mat[2].y = -v.x;
			mat[2].z = 0.0f;
			return mat;
		}

		void solveContactStatic()
		{
			DynamicBody* b = (DynamicBody*)body_b;

			glm::vec3 rel_poc_b = poc - b->pos;

			float inv_mass = 1.0f / b->mass;

			// create contact transform basis
			glm::vec3 tangent;
			if (glm::abs(normal.x) > glm::abs(normal.y))
				tangent = glm::cross(normal, glm::vec3(0.0f, 1.0f, 0.0f));
			else
				tangent = glm::cross(normal, glm::vec3(1.0f, 0.0f, 0.0f));
			tangent = glm::normalize(tangent);
			glm::vec3 bitangent = glm::cross(normal, tangent);
			glm::mat3 contact_to_world = glm::mat3(tangent, bitangent, normal);

			// change in linear velocity
			glm::mat3 delta_vel_contact = glm::mat3(inv_mass);

			if (!b->rotation_locked)
			{
				// create matrix that converts impulse to velocity in world coordinates
				glm::mat3 impulse_to_torque = createSkew(rel_poc_b);
				glm::mat3 delta_vel_world = b->inertia_inv_world * impulse_to_torque;
				delta_vel_world = impulse_to_torque * delta_vel_world;
				delta_vel_world *= -1.0f;

				// convert change in velocity to contact coordinates
				delta_vel_contact += glm::transpose(contact_to_world) * delta_vel_world * contact_to_world;
			}

			// invert to get impulse per unit velocity
			glm::mat3 impulse_unit_velocity = glm::inverse(delta_vel_contact);

			// get closing velocity at contact
			glm::vec3 closing_vel = b->getVelocityWorld(poc);

			// closing velocity in contact coordinates
			glm::vec3 contact_closing_vel = glm::transpose(contact_to_world) * closing_vel;

			// make sure bodies are moving towards each other
			if (contact_closing_vel.z > 0.0f)
				return;

			// find desired velocity change
			float d_vel = -contact_closing_vel.z * (1 + restitution);

			// find the target velocity
			glm::vec3 desired_vel = glm::vec3(-contact_closing_vel.x, -contact_closing_vel.y, d_vel);

			// find the impulse to get target velocity
			glm::vec3 impulse_contact = impulse_unit_velocity * desired_vel;

			// find impulse on plane
			float planar_impulse = glm::sqrt(impulse_contact.x * impulse_contact.x + impulse_contact.y * impulse_contact.y);

			// check if it exceeds static friction
			if (planar_impulse > impulse_contact.z * friction)
			{
				// do dynamic friction
				impulse_contact.x /= planar_impulse;
				impulse_contact.y /= planar_impulse;

				impulse_contact.z = delta_vel_contact[2].z +
					delta_vel_contact[2].x * friction * impulse_contact.x +
					delta_vel_contact[2].y * friction * impulse_contact.y;
				impulse_contact.z = d_vel / impulse_contact.z;
				impulse_contact.x *= friction * impulse_contact.z;
				impulse_contact.y *= friction * impulse_contact.z;
			}

			// convert impulse to world vector
			glm::vec3 impulse_world = contact_to_world * impulse_contact;

			// apply impulse
			b->applyImpulse(impulse_world, poc);
			b->pos += depth * normal;
		}

		void solveContactDynamic()
		{
			DynamicBody* a = (DynamicBody*)body_a;
			DynamicBody* b = (DynamicBody*)body_b;

			glm::vec3 rel_poc_b = poc - b->pos;

			float inv_mass = 1.0f / b->mass + 1.0f / a->mass;

			// create contact transform basis
			glm::vec3 tangent;
			if (glm::abs(normal.x) > glm::abs(normal.y))
				tangent = glm::cross(normal, glm::vec3(0.0f, 1.0f, 0.0f));
			else
				tangent = glm::cross(normal, glm::vec3(1.0f, 0.0f, 0.0f));
			tangent = glm::normalize(tangent);
			glm::vec3 bitangent = glm::cross(normal, tangent);
			glm::mat3 contact_to_world = glm::mat3(tangent, bitangent, normal);

			// change in linear velocity
			glm::mat3 delta_vel_contact = glm::mat3(inv_mass);

			glm::mat3 delta_vel_world = glm::mat3(0.0f);
			if (!b->rotation_locked)
			{
				// create matrix that converts impulse to velocity in world coordinates
				glm::mat3 impulse_to_torque = createSkew(rel_poc_b);
				delta_vel_world = b->inertia_inv_world * impulse_to_torque;
				delta_vel_world = impulse_to_torque * delta_vel_world;
				delta_vel_world *= -1.0f;
			}

			glm::vec3 rel_poc_a = poc - a->pos;

			if (!a->rotation_locked)
			{
				// create matrix that converts impulse to velocity in world coordinates
				glm::mat3 impulse_to_torque = createSkew(rel_poc_a);
				glm::mat3 delta_vel_world2 = a->inertia_inv_world * impulse_to_torque;
				delta_vel_world2 = impulse_to_torque * delta_vel_world2;
				delta_vel_world2 *= -1.0f;

				// add total change in velocity
				delta_vel_world += delta_vel_world2;
			}

			// convert change in velocity to contact coordinates
			delta_vel_contact += glm::transpose(contact_to_world) * delta_vel_world * contact_to_world;

			// invert to get impulse per unit velocity
			glm::mat3 impulse_unit_velocity = glm::inverse(delta_vel_contact);

			// get closing velocity at contact
			glm::vec3 closing_vel = b->getVelocityWorld(poc);
			closing_vel -= a->getVelocityWorld(poc);

			// closing velocity in contact coordinates (transpose = inverse)
			glm::vec3 contact_closing_vel = glm::transpose(contact_to_world) * closing_vel;

			// make sure bodies are moving towards each other
			if (contact_closing_vel.z > 0.0f)
				return;

			// find desired velocity change
			float d_vel = -contact_closing_vel.z * (1 + restitution);

			// find the target velocity
			glm::vec3 desired_vel = glm::vec3(-contact_closing_vel.x, -contact_closing_vel.y, d_vel);

			// find the impulse to get target velocity
			glm::vec3 impulse_contact = impulse_unit_velocity * desired_vel;

			// find impulse on plane
			float planar_impulse = glm::sqrt(impulse_contact.x * impulse_contact.x + impulse_contact.y * impulse_contact.y);

			// check if it exceeds static friction
			if (planar_impulse > impulse_contact.z * friction)
			{
				// do dynamic friction
				impulse_contact.x /= planar_impulse;
				impulse_contact.y /= planar_impulse;

				impulse_contact.z = delta_vel_contact[2].z +
					delta_vel_contact[2].x * friction * impulse_contact.x +
					delta_vel_contact[2].y * friction * impulse_contact.y;
				impulse_contact.z = d_vel / impulse_contact.z;
				impulse_contact.x *= friction * impulse_contact.z;
				impulse_contact.y *= friction * impulse_contact.z;
			}

			// convert impulse to world vector
			glm::vec3 impulse_world = contact_to_world * impulse_contact;

			a->setAwake();
			b->setAwake();
			// apply impulse
			b->applyImpulse(impulse_world, poc);
			a->applyImpulse(-impulse_world, poc);
			b->pos += depth * normal * 0.5f;
			a->pos -= depth * normal * 0.5f;
		}

		//void solveContact()
		//{
		//	glm::vec3 rel_poc_b = poc - b->pos;

		//	// create orthonormal basis
		//	glm::vec3 tangent;
		//	if (normal.x > normal.y)
		//		tangent = glm::cross(normal, glm::vec3(0.0f, 1.0f, 0.0f));
		//	else
		//		tangent = glm::cross(normal, glm::vec3(1.0f, 0.0f, 0.0f));
		//	tangent = glm::normalize(tangent);
		//	glm::vec3 bitangent = glm::cross(normal, tangent);

		//	// find B velocity change per unit impulse
		//	float unit_impulse_linear = 1.0f / b->mass;

		//	glm::vec3 torque_per_unit_impulse = glm::cross(rel_poc_b, normal);
		//	glm::vec3 angular_vel_per_unit_impulse = b->inertia_inv_world * torque_per_unit_impulse;
		//	glm::vec3 linear_vel_per_unit_impulse = glm::cross(angular_vel_per_unit_impulse, rel_poc_b);
		//	float unit_impulse_angular = glm::dot(linear_vel_per_unit_impulse, normal);

		//	float unit_vel = unit_impulse_linear + unit_impulse_angular;

		//	if (a)
		//	{
		//		glm::vec3 rel_poc_a = poc - a->pos;

		//		// find A velocity change per unit impulse in opposite direction
		//		float unit_linear = 1.0f / a->mass;

		//		glm::vec3 tpui = glm::cross(rel_poc_a, normal);
		//		glm::vec3 avpui = a->inertia_inv_world * tpui;
		//		glm::vec3 lvpui = glm::cross(avpui, rel_poc_a);
		//		float unit_angular = glm::dot(lvpui, normal);

		//		unit_vel += unit_linear + unit_angular;
		//	}

		//	// get velocity at contact
		//	glm::vec3 closing_vel = b->getVelocityWorld(poc);
		//	if (a)
		//		closing_vel -= a->getVelocityWorld(poc);

		//	// transform to contact coordinates
		//	float contact_closing_vel = glm::dot(normal, closing_vel);
		//	if (contact_closing_vel > 0.0f) // bodies moving away from each other
		//		return;
		//	float contact_tangent_vel = glm::dot(tangent, closing_vel);
		//	float contact_bitangent_vel = glm::dot(bitangent, closing_vel);

		//	// find desired velocity change
		//	const float restitution = 0.4f;
		//	float d_vel = -contact_closing_vel * (1 + restitution);

		//	// find desired impulse
		//	float impulse = d_vel / (unit_vel);

		//	// convert impulse to world coordinates
		//	glm::vec3 world_impulse = normal * impulse;

		//	// apply impulse
		//	b->applyImpulse(world_impulse, poc);
		//	if (a)
		//	{
		//		a->applyImpulse(-world_impulse, poc);
		//		b->pos += depth * normal * 0.5f;
		//		a->pos -= depth * normal * 0.5f;
		//	}
		//	else
		//		b->pos += depth * normal;
		//}
	};

	struct ContactManifold
	{
		bool collided;

		Body* body_a;
		Body* body_b;

		glm::vec3 normal;

		unsigned int num_contacts;
		glm::vec3 local_a[4];
		glm::vec3 local_b[4];

		void addContact(ContactInfo& contact)
		{
			// find contact point with deepest penetration
			// try to maximize surface area
		}

		void updateContacts()
		{
			// remove contact if too far away
		}
	};

	struct Simplex
	{
		unsigned int n;
		glm::vec3 v[4];
		glm::vec3 support_a[4];
		glm::vec3 support_b[4];

		glm::vec3 D;

		Simplex() : n(0) {}

		void add(glm::vec3 p, glm::vec3 a, glm::vec3 b)
		{
			v[n] = p;
			support_a[n] = a;
			support_b[n] = b;
			n++;
		}

		void set(unsigned int a, unsigned int b)
		{
			v[a] = v[b];
			support_a[a] = support_a[b];
			support_b[a] = support_b[b];
		}

		void swap(unsigned int a, unsigned int b)
		{
			glm::vec3 temp = v[a];
			v[a] = v[b];
			v[b] = temp;

			temp = support_a[a];
			support_a[a] = support_a[b];
			support_a[b] = temp;

			temp = support_b[a];
			support_b[a] = support_b[b];
			support_b[b] = temp;
		}

		void clear()
		{
			n = 0;
		}
	};

	struct Polytope
	{
		// vertex data
		unsigned int n_vertices;
		glm::vec3 vertices[50];
		glm::vec3 support_a[50]; // support points of the vertices
		glm::vec3 support_b[50];

		// face data
		unsigned int n_faces;
		glm::uvec3 faces[50];
		glm::vec3 normals[50];
		float dist[50]; // distance from center

		// edge data (only used for adding vertices)
		unsigned int n_edges;
		glm::uvec2 edges[50];

		Polytope() : n_vertices(0), n_faces(0), n_edges(0)
		{
			
		}

		void set(Simplex& s)
		{
			n_vertices = 4;
			n_faces = 4;
			n_edges = 0;

			vertices[0] = s.v[0];
			vertices[1] = s.v[1];
			vertices[2] = s.v[2];
			vertices[3] = s.v[3];

			support_a[0] = s.support_a[0];
			support_a[1] = s.support_a[1];
			support_a[2] = s.support_a[2];
			support_a[3] = s.support_a[3];

			support_b[0] = s.support_b[0];
			support_b[1] = s.support_b[1];
			support_b[2] = s.support_b[2];
			support_b[3] = s.support_b[3];
			n_vertices = 4;

			faces[0] = glm::uvec3(0, 1, 2);
			faces[1] = glm::uvec3(0, 3, 1);
			faces[2] = glm::uvec3(1, 3, 2);
			faces[3] = glm::uvec3(2, 3, 0);
			n_faces = 4;

			calculateFaceNormals();

			calculateDistances();
		}

		inline glm::vec3 calculateFaceNormal(unsigned int index)
		{
			glm::vec3 a = vertices[faces[index].y] - vertices[faces[index].x];
			glm::vec3 b = vertices[faces[index].z] - vertices[faces[index].x];
			return glm::normalize(glm::cross(a, b));
		}
		void calculateFaceNormals()
		{
			for (unsigned int i = 0; i < n_faces; ++i)
				normals[i] = calculateFaceNormal(i);
		}

		inline float calculateDistanceToOrigin(unsigned int face)
		{
			glm::vec3 AO = -vertices[faces[face].x]; // direction from point on the face to the origin
			return -glm::dot(normals[face], AO);
		}
		void calculateDistances()
		{
			for (unsigned int i = 0; i < n_faces; ++i)
				dist[i] = calculateDistanceToOrigin(i);
		}

		// finds the face that is closest to the origin
		unsigned int findClosestFace()
		{
			float min_dist = dist[0];
			unsigned int min_index = 0;

			for (unsigned int i = 0; i < n_faces; ++i)
			{
				if (dist[i] < min_dist)
				{
					min_dist = dist[i];
					min_index = i;
				}
			}

			return min_index;
		}

		void populateEdges(unsigned int face)
		{
			unsigned int a = faces[face].x;
			unsigned int b = faces[face].y;
			unsigned int c = faces[face].z;
			addEdge(glm::uvec2(a, b));
			addEdge(glm::uvec2(b, c));
			addEdge(glm::uvec2(c, a));
		}

		// tests to see whether edges are equal
		bool compareEdges(glm::uvec2& a, glm::uvec2& b)
		{
			if (a == b)
				return true;
			if (a.x == b.y && b.x == a.y)
				return true;
			return false;
		}

		// returns true if an edge was a duplicate
		bool addEdge(glm::uvec2 edge)
		{
			// only edges that are not shared between removed faces can be added
			for (unsigned int i = 0; i < n_edges; ++i)
			{
				if (compareEdges(edge, edges[i]))
				{
					// remove duplicate edge
					removeEdge(i);
					return true;
				}
			}
			edges[n_edges] = edge;
			n_edges++;
			return false;
		}

		void removeEdge(unsigned int edge)
		{
			for (unsigned int i = edge; i < n_edges - 1; ++i)
			{
				edges[i] = edges[i + 1];
			}
			n_edges--;
		}

		void removeFace(unsigned int face)
		{
			for (unsigned int i = face; i < n_faces - 1; ++i)
			{
				faces[i]   = faces[i + 1];
				normals[i] = normals[i + 1];
				dist[i]    = dist[i + 1];
			}
			n_faces--;
		}

		void addPoint(glm::vec3 point, glm::vec3 a, glm::vec3 b)
		{
			// add point to vertices
			vertices[n_vertices] = point;

			// find all faces whose distance from point is > 0
			for (unsigned int i = 0; i < n_faces; ++i)
			{
				float dist = distanceToPoint(point, i);

				if (dist > 0)
				{
					populateEdges(i); // add unique edges to list
					removeFace(i); // remove face from polytope
					i--;
				}
			}
			// add new faces
			for (unsigned int i = 0; i < n_edges; ++i)
			{
				faces[n_faces] = glm::uvec3(edges[i].x, edges[i].y, n_vertices);
				normals[n_faces] = calculateFaceNormal(n_faces);
				dist[n_faces] = calculateDistanceToOrigin(n_faces);
				n_faces++;
			}

			support_a[n_vertices] = a;
			support_b[n_vertices] = b;

			n_vertices++;
			n_edges = 0;
		}

		// calculates the distance from a face to a point
		float distanceToPoint(glm::vec3 point, unsigned int face)
		{
			glm::vec3 on_face = vertices[faces[face].x];
			return glm::dot(normals[face], point - on_face);
		}
	};

	Simplex s;

	glm::vec3 support(Body* a, const glm::vec3& axis)
	{
		glm::vec3 local_axis = a->getLocalVec(axis);
		return a->getWorldVec(a->shapes[0]->support(local_axis)) + a->pos;
	}

	bool GJK(Body* body_a, Body* body_b, glm::vec3 axis)
	{
		Shape* a = body_a->shapes[0];
		Shape* b = body_b->shapes[0];
		glm::vec3 A_a = support(body_a, axis);
		glm::vec3 A_b = support(body_b, -axis);
		glm::vec3 A = A_a - A_b;//a->support(axis) - b->support(-axis);
		s.clear();
		s.add(A, A_a, A_b);
		glm::vec3 D = -A;
		s.D = D;

		unsigned int iters = 0;
		while (iters < 50)
		{
			++iters;
			A_a = support(body_a, D);
			A_b = support(body_b, -D);
			A = A_a - A_b;//a->support(D) - b->support(-D);
			if (glm::dot(A, D) < 0)
				return false;
			
			s.add(A, A_a, A_b);

			switch (s.n)
			{
			case 2:
			{
				// A: v[1]
				// B: v[0]
				glm::vec3 AB = s.v[0] - s.v[1];
				glm::vec3 AO = -s.v[1];
				D = trip(AB, AO);
				break;
			}
			case 3:
			{
				// A: v[2]
				// B: v[1]
				// C: v[0]
				glm::vec3 AB = s.v[1] - s.v[2];
				glm::vec3 AC = s.v[0] - s.v[2];
				glm::vec3 AO = -s.v[2];
				glm::vec3 norm = glm::cross(AB, AC);
				glm::vec3 AB_out = glm::cross(AB, norm);
				if (glm::dot(AB_out, AO) > 0) // AB was closest
				{
					D = trip(AB, AO);
					s.n = 2;
					s.set(0, 1);
					s.set(1, 2);
				}
				else
				{
					glm::vec3 AC_out = glm::cross(norm, AC);
					if (glm::dot(AC_out, AO) > 0) // AC was closest
					{
						D = trip(AC, AO);
						s.n = 2;
						s.set(1, 2);
					}
					else // inside triangular prism
					{
						// make sure new D is normal to new triangle
						if (glm::dot(norm, AO) > 0) // above triangle
						{
							D = norm;
						}
						else // reverse the triangle by swapping B and C
						{
							D = -norm;
							s.swap(0, 1);
						}
					}
				}
				break;
			}
			case 4:
			{
				// A: v[2]
				// B: v[1]
				// C: v[0]
				// D: v[3]
				glm::vec3 CD = s.v[3] - s.v[0];
				glm::vec3 BD = s.v[3] - s.v[1];
				glm::vec3 AD = s.v[3] - s.v[2];
				glm::vec3 DO = -s.v[3];
				glm::vec3 CDB = glm::cross(BD, CD);
				if (glm::dot(CDB, DO) > 0) // in front of CDB
				{
					// could be CDB, CD, or BD
					glm::vec3 CDB_CD_out = glm::cross(CD, CDB);
					glm::vec3 CDB_BD_out = glm::cross(CDB, BD);
					if (glm::dot(CDB_CD_out, DO) < 0)
					{
						// could be CDB or BD
						if (glm::dot(CDB_BD_out, DO) < 0)
						{
							// CDB [B, D, C]
							D = CDB;
							s.n = 3;
							s.set(2, 0);
							s.set(0, 1);
							s.set(1, 3);
							/*s.swap(0, 1);
							s.set(2, 3);*/
						}
						else
						{
							// BD [B, D]
							D = trip(BD, DO);
							s.n = 2;
							s.set(0, 1);
							s.set(1, 3);
						}
					}
					else
					{
						// CD [C, D]
						D = trip(CD, DO);
						s.n = 2;
						s.set(1, 3);
					}
				}
				else // behind CDB
				{
					// could be CD, BD, AD, ADC, BDA
					glm::vec3 ADC = glm::cross(CD, AD);
					if (glm::dot(ADC, DO) > 0) // in front of ADC
					{
						// could be CD, AD, ADC
						glm::vec3 ADC_AD_out = glm::cross(AD, ADC);
						glm::vec3 ADC_CD_out = glm::cross(ADC, CD);

						if (glm::dot(ADC_AD_out, DO) < 0)
						{
							// must be ADC or CD
							if (glm::dot(ADC_CD_out, DO) < 0)
							{
								// ADC [A, C, D]
								D = ADC;
								s.n = 3;
								s.set(1, 0);
								s.set(0, 2);
								s.set(2, 3);
								/*s.set(1, 2);
								s.set(2, 3);*/
							}
							else
							{
								// CD [C, D]
								D = trip(CD, DO);
								s.n = 2;
								s.set(1, 3);
							}
						}
						else
						{
							// AD [A, D]
							D = trip(AD, DO);
							s.n = 2;
							s.set(0, 2);
							s.set(1, 3);
						}
					}
					else
					{
						// must be BDA, BD, AD, ABCD
						glm::vec3 BDA = glm::cross(AD, BD);

						if (glm::dot(BDA, DO) > 0)
						{
							// could be BDA, BD, AD
							glm::vec3 BDA_AD_out = glm::cross(BDA, AD);
							glm::vec3 BDA_BD_out = glm::cross(BD, BDA);

							if (glm::dot(BDA_AD_out, DO) < 0)
							{
								// must be BDA, BD
								if (glm::dot(BDA_BD_out, DO) < 0)
								{
									// BDA [B, A, D]
									D = BDA;
									s.n = 3;
									s.set(0, 1);
									s.set(1, 2);
									s.set(2, 3);
									/*s.set(0, 2);
									s.set(2, 3);*/
								}
								else
								{
									// BD
									D = trip(BD, DO);
									s.n = 2;
									s.set(0, 1);
									s.set(1, 3);
								}
							}
							else
							{
								// AD
								D = trip(AD, DO);
								s.n = 2;
								s.set(0, 2);
								s.set(1, 3);
							}
						}
						else
						{
							// ABCD, terminate
							s.n = 4;
							return true;
						}
					}
				}
				break;
			}
			}
			s.D = D;
		}
		return false;
	}

	Polytope p;

	ContactInfo EPA(Body* a, Body* b)
	{
		p.set(s);

		unsigned int iters = 0;
		while (iters < 50 && p.n_faces < 40)
		{
			// find the closest face to the origin
			unsigned int closest_face = p.findClosestFace();
			// calculate the direction to search (the normal of the closest face
			glm::vec3 D = p.normals[closest_face];

			// find the point furthest in that direction on Minkowski difference
			glm::vec3 A_a = support(a, D);
			glm::vec3 A_b = support(b, -D);
			glm::vec3 A = A_a - A_b;//a->support(D) - b->support(-D);

			// find how far that point is from the face
			float dist = p.distanceToPoint(A, closest_face);

			// check to see if the point passed the face by a certain threshold
			if (dist < 0.0001f) // cloest face is on the edge of the Minkowski difference
			{
				// generate contact info and return
				ContactInfo contact;
				contact.body_a = a;
				contact.body_b = b;
				contact.collided = true;
				contact.normal = p.normals[closest_face];
				contact.depth = p.dist[closest_face];

				contact.restitution = glm::max(a->restitution, b->restitution);
				contact.friction = glm::min(a->friction, b->friction);

				// find vertices of closest face
				glm::uvec3 face = p.faces[closest_face];
				glm::vec3 v_a = p.vertices[face.x];
				glm::vec3 v_b = p.vertices[face.y];
				glm::vec3 v_c = p.vertices[face.z];
				// find vertex closest to minkowski difference
				glm::vec3 v_p = contact.normal * contact.depth;
				// find barycentric coordinates of that point from vertices
				glm::vec3 v0 = v_b - v_a;
				glm::vec3 v1 = v_c - v_a;
				glm::vec3 v2 = v_p - v_a;
				float d00 = glm::dot(v0, v0);
				float d01 = glm::dot(v0, v1);
				float d11 = glm::dot(v1, v1);
				float d20 = glm::dot(v2, v0);
				float d21 = glm::dot(v2, v1);
				float denom = d00 * d11 - d01 * d01;
				float v = (d11 * d20 - d01 * d21) / denom;
				float w = (d00 * d21 - d01 * d20) / denom;
				float u = 1.0f - v - w;
				// find vertices of shape A that correspond to face
				glm::vec3 a_a = p.support_a[face.x];
				glm::vec3 a_b = p.support_a[face.y];
				glm::vec3 a_c = p.support_a[face.z];
				// find vertices of shape B that correspond to face
				glm::vec3 b_a = p.support_b[face.x];
				glm::vec3 b_b = p.support_b[face.y];
				glm::vec3 b_c = p.support_b[face.z];
				// find world point using barycentric coordinates
				contact.poc_a = a_a * u + a_b * v + a_c * w;
				contact.poc_b = b_a * u + b_b * v + b_c * w;
				contact.poc = (contact.poc_a + contact.poc_b) * 0.5f;
				return contact;
			}

			// closest face was inside of the Minkowski difference
			// add the new point to the polytope
			// also add the points that correspond to each shape
			p.addPoint(A, A_a, A_b);

			// repeat until closest face is on Minkowski difference
			iters++;
		}
		//std::cout << "iters: " << iters << std::endl;
		// too many iterations before finding contact
		ContactInfo contact;
		contact.collided = false;
		return contact;

	}

	ContactInfo checkCollision(Body* a, Body* b)
	{
		bool collided = GJK(a, b, glm::vec3(1.0f, 0.0f, 0.0f));

		if (collided)
		{
			/*ContactInfo contact = EPA(a, b);
			if (contact.collided)
				contacts->push_back(contact);*/
			return EPA(a, b);
		}

		ContactInfo contact;
		contact.collided = false;
		return contact;
	}

	ContactInfo checkCollisionSphereSphere(Body* a, Body* b)
	{
		Sphere* sphere_a = (Sphere*)a->shapes[0];
		Sphere* sphere_b = (Sphere*)b->shapes[0];

		ContactInfo contact;
		contact.collided = false;

		float dx = b->pos.x - a->pos.x;
		float dy = b->pos.y - a->pos.y;
		float dz = b->pos.z - a->pos.z;
		float rad = sphere_a->rad + sphere_b->rad;
		float rad2 = rad * rad;
		float dist2 = dx * dx + dy * dy + dz * dz;
		if (dist2 < rad2)
		{
			float dist = glm::sqrt(dist2);
			glm::vec3 dir = (b->pos - a->pos) / dist;

			contact.collided = true;
			contact.body_a = a;
			contact.body_b = b;
			contact.poc = a->pos + dir * sphere_a->rad;
			contact.poc += b->pos - dir * sphere_b->rad;
			contact.poc *= 0.5f;
			contact.normal = dir;
			contact.depth = rad - dist;
			contact.restitution = glm::max(a->restitution, b->restitution);
			contact.friction = glm::min(a->friction, b->friction);
		}

		return contact;
	}

	ContactInfo checkCollisionSphereCapsule(Body* a, Body* b)
	{
		Capsule* capsule = (Capsule*)a->shapes[0];
		Sphere* sphere = (Sphere*)b->shapes[0];

		ContactInfo contact;
		contact.collided = false;

		glm::vec3 local = a->getLocalPos(b->pos);
		glm::vec3 top = glm::vec3(0.0f, 0.0f, capsule->height);
		glm::vec3 bottom = glm::vec3(0.0f, 0.0f, -capsule->height);
		float dist_top = glm::distance(local, top);
		float dist_bottom = glm::distance(local, bottom);

		float planar = glm::sqrt(local.x * local.x + local.y * local.y);
		if (local.z < capsule->height && local.z > -capsule->height && planar < capsule->rad + sphere->rad)
		{
			glm::vec3 normal = glm::vec3(local.x, local.y, 0.0f) / planar;

			contact.collided = true;
			contact.body_a = a;
			contact.body_b = b;
			contact.poc = normal * capsule->rad;
			contact.poc.z = local.z;
			contact.normal = normal;
			contact.depth = capsule->rad + sphere->rad - planar;
			contact.restitution = glm::max(a->restitution, b->restitution);
			contact.friction = glm::min(a->friction, b->friction);

			contact.poc = a->getWorldPos(contact.poc);
			contact.normal = a->getWorldVec(contact.normal);
		}
		else if (dist_top < capsule->rad + sphere->rad)
		{
			glm::vec3 dir = glm::normalize(local - top);

			contact.collided = true;
			contact.body_a = a;
			contact.body_b = b;
			contact.poc = top + dir * capsule->rad;
			contact.poc += local - dir * sphere->rad;
			contact.poc *= 0.5f;
			contact.normal = dir;
			contact.depth = capsule->rad + sphere->rad - dist_top;
			contact.restitution = glm::max(a->restitution, b->restitution);
			contact.friction = glm::min(a->friction, b->friction);

			contact.poc = a->getWorldPos(contact.poc);
			contact.normal = a->getWorldVec(contact.normal);
		}
		else if (glm::distance(local, bottom) < capsule->rad + sphere->rad)
		{
			glm::vec3 dir = glm::normalize(local - bottom);

			contact.collided = true;
			contact.body_a = a;
			contact.body_b = b;
			contact.poc = bottom + dir * capsule->rad;
			contact.poc += local - dir * sphere->rad;
			contact.poc *= 0.5f;
			contact.normal = dir;
			contact.depth = capsule->rad + sphere->rad - dist_bottom;
			contact.restitution = glm::max(a->restitution, b->restitution);
			contact.friction = glm::min(a->friction, b->friction);

			contact.poc = a->getWorldPos(contact.poc);
			contact.normal = a->getWorldVec(contact.normal);
		}

		return contact;
	}

	float ground_restitution = 0.1f;
	float ground_friction = 0.8f;
	ContactInfo checkCollisionGround(Body* body)
	{
		ContactInfo contact;
		contact.collided = false;

		glm::vec3 local_up = body->getLocalVec(glm::vec3(0.0f, 0.0f, -1.0f));
		glm::vec3 lowest = body->shapes[0]->support(local_up);
		lowest = body->getWorldPos(lowest);

		if (lowest.z < 0.0f)
		{
			contact.collided = true;
			contact.body_a = nullptr;
			contact.body_b = body;
			contact.poc = lowest;
			contact.normal = glm::vec3(0.0f, 0.0f, 1.0f);
			contact.depth = -lowest.z;
			contact.restitution = glm::max(ground_restitution, body->restitution);
			contact.friction = glm::min(ground_friction, body->friction);
		}

		return contact;
	}

	ContactInfo checkCollisionSphereGround(Body* body)
	{
		Sphere* sphere = (Sphere*)body->shapes[0];

		ContactInfo contact;
		contact.collided = false;

		if (body->pos.z - sphere->rad < 0)
		{
			
			contact.collided = true;
			contact.body_a = nullptr;
			contact.body_b = body;
			contact.poc = body->pos;
			contact.poc.z -= sphere->rad;
			contact.normal = glm::vec3(0.0f, 0.0f, 1.0f);
			contact.depth = -contact.poc.z;
			contact.restitution = glm::max(ground_restitution, body->restitution);
			contact.friction = glm::min(ground_friction, body->friction);
		}

		return contact;
	}

	ContactInfo checkCollisionBoxGround(Body* body)
	{
		Box* box = (Box*)body->shapes[0];

		ContactInfo contact;
		contact.collided = false;
		contact.depth = 0.0f;

		std::vector<glm::vec3> vertices;
		vertices.reserve(8);
		box->projectVertices(body->orientation_mat, body->pos, vertices);

		for (unsigned int i = 0; i < vertices.size(); ++i)
		{
			if (vertices[i].z < -contact.depth)
			{
				contact.collided = true;
				contact.body_a = nullptr;
				contact.body_b = body;
				contact.poc = vertices[i];
				contact.normal = glm::vec3(0.0f, 0.0f, 1.0f);
				contact.depth = -vertices[i].z;
				contact.restitution = glm::max(ground_restitution, body->restitution);
				contact.friction = glm::min(ground_friction, body->friction);
			}
		}

		return contact;
	}

	float collisionAxisDepth(glm::vec3 axis, std::vector<glm::vec3>& a, std::vector<glm::vec3>& b)
	{
		float min_a, max_a;
		float min_b, max_b;
		min_a = max_a = glm::dot(axis, a[0]);
		min_b = max_b = glm::dot(axis, b[0]);

		// projecting a
		for (unsigned int i = 1; i < a.size(); ++i)
		{
			float proj = glm::dot(axis, a[i]);
			min_a = glm::min(min_a, proj);
			max_a = glm::max(max_a, proj);
		}

		//projecting b
		for (unsigned int i = 1; i < b.size(); ++i)
		{
			float proj = glm::dot(axis, b[i]);
			min_b = glm::min(min_b, proj);
			max_b = glm::max(max_b, proj);
		}

		return glm::min(max_a - min_b, max_b - min_a);
	}

	// checks vertices of B along an axis of A
	inline ContactInfo checkContactBoxBox(Body* a, Body* b, float a_dim, int axis, std::vector<glm::vec3>& bv)
	{
		ContactInfo contact;
		contact.collided = false;

		float min_a = -a_dim;
		float max_a = a_dim;

		float min_b = glm::dot(bv[0] - a->pos, a->orientation_mat[axis]);
		float max_b = min_b;
		unsigned int min_index = 0;
		unsigned int max_index = 0;

		for (unsigned int i = 1; i < bv.size(); ++i)
		{
			float projected = glm::dot(bv[i] - a->pos, a->orientation_mat[0]);

			if (projected < min_b)
			{
				min_b = projected;
				min_index = i;
			}
			else if (projected > max_b)
			{
				max_b = projected;
				max_index = i;
			}
		}

		// b to right of a
		float right_separation = min_b - max_a;
		// b to left of a
		float left_separation = min_a - max_b;

		// check if separated
		if (right_separation > 0 || left_separation > 0)
			return contact;

		contact.collided = true;
		contact.body_a = a;
		contact.body_b = b;
		// b colliding to right of a
		if (right_separation > left_separation)
		{
			contact.poc = bv[min_index];
			contact.normal = a->orientation_mat[0];
			contact.depth = -right_separation;
		}
		// b colliding to left of a
		else
		{
			contact.poc = bv[max_index];
			contact.normal = -a->orientation_mat[0];
			contact.depth = -left_separation;
		}

		return contact;
	}
	bool addContact(std::vector<ContactInfo>& contacts, ContactInfo& contact)
	{
		if (!contact.collided)
			return false;
		contacts.push_back(contact);
		return true;
	}

	glm::vec4 calculatePenetration(Body* a, Box* box, glm::vec3 v)
	{
		glm::vec3 rel_v = a->getLocalPos(v);

		glm::vec3 axis = a->orientation_mat[0];
		if (rel_v.x < 0)
			axis *= -1.0f;

		// find furthest distance pointing away from face
		float max_dist = glm::abs(rel_v.x) - box->dim.x;

		float dist_y = glm::abs(rel_v.y) - box->dim.y;
		if (dist_y > max_dist)
		{
			max_dist = dist_y;
			axis = a->orientation_mat[1];
			if (rel_v.y < 0)
				axis *= -1.0f;
		}

		float dist_z = glm::abs(rel_v.z) - box->dim.z;
		if (dist_z > max_dist)
		{
			max_dist = dist_z;
			axis = a->orientation_mat[2];
		}

		return glm::vec4(axis.x, axis.y, axis.z, max_dist);
		if (rel_v.z < 0)
			axis *= -1.0f;
	}

	void checkCollisionBoxBox(std::vector<ContactInfo>& contacts, Body* a, Body* b)
	{
		ContactInfo deepest_vertex;
		deepest_vertex.collided = false;
		deepest_vertex.depth = -999.9f;

		ContactInfo deepest_edge;
		deepest_edge.collided = false;
		deepest_edge.depth = -999.9f;

		Box* ba = (Box*)a->shapes[0];
		Box* bb = (Box*)b->shapes[0];

		// vertices - can probably store these in the shape data, or use a more efficient algorithm
		std::vector<glm::vec3> av;
		std::vector<glm::vec3> bv;
		ba->projectVertices(a->orientation_mat, a->pos, av);
		bb->projectVertices(b->orientation_mat, b->pos, bv);

		// edges
		std::vector<Edge> ae;
		std::vector<Edge> be;
		ba->projectEdges(ae, av);
		bb->projectEdges(be, bv);

		// check vertices of B on A
		for (unsigned int i = 0; i < bv.size(); ++i)
		{
			glm::vec4 penetration = calculatePenetration(a, ba, bv[i]);
			if (penetration.w > 0) // did not penetrate
				continue;
			ContactInfo contact;
			contact.collided = true;
			contact.body_a = a;
			contact.body_b = b;
			contact.poc = bv[i];
			contact.normal = -glm::vec3(penetration.x, penetration.y, penetration.z);
			contact.depth = -penetration.w;
			//contacts.push_back(contact);
			if (contact.depth > deepest_vertex.depth)
				deepest_vertex.set(contact);

			std::cout << "vertex contact" << std::endl;
		}

		// check vertices of A on B
		for (unsigned int i = 0; i < av.size(); ++i)
		{
			glm::vec4 penetration = calculatePenetration(b, bb, av[i]);
			if (penetration.w > 0)
				continue;
			ContactInfo contact;
			contact.collided = true;
			contact.body_a = b;
			contact.body_b = a;
			contact.poc = av[i];
			contact.normal = glm::vec3(penetration.x, penetration.y, penetration.z);
			contact.depth = -penetration.w;
			//contacts.push_back(contact);
			if (contact.depth > deepest_vertex.depth)
				deepest_vertex.set(contact);

			//std::cout << "vertex contact" << std::endl;
		}

		// check edges of A vs edges of B
		for (unsigned int i = 0; i < ae.size(); ++i)
		{
			for (unsigned int j = 0; j < be.size(); ++j)
			{
				Edge& ea = ae[i];
				Edge& eb = be[j];
				// find closest points on each edge
				// test if point on edge A is in B
				// set contact to halfway between edge points
				glm::vec3 p1 = ea.a;
				glm::vec3 p2 = eb.a;
				glm::vec3 d1 = ea.b - p1;
				glm::vec3 d2 = eb.b - p2;
				glm::vec3 n = glm::cross(d1, d2);
				glm::vec3 n1 = glm::cross(d1, n);
				glm::vec3 n2 = glm::cross(d2, n);
				float t1 = glm::dot(p2 - p1, n2) / glm::dot(d1, n2);
				float t2 = glm::dot(p1 - p2, n1) / glm::dot(d2, n1);
				glm::vec3 ca = p1 + d1 * t1;
				glm::vec3 cb = p2 + d2 * t2;
				glm::vec3 b_local_ca = b->getLocalPos(ca);
				glm::vec3 a_local_cb = a->getLocalPos(cb);
				if (bb->intersects(b_local_ca) && ba->intersects(a_local_cb) && t1 > 0.0f && t1 < 1.0f && t2 > 0.0f && t2 < 1.0f)
				{
					ContactInfo contact;
					contact.collided = true;
					contact.body_a = a;
					contact.body_b = b;
					contact.poc = (ca + cb) * 0.5f;
					contact.normal = glm::normalize(ca - cb);
					contact.depth = glm::length(ca - cb);
					//contacts.push_back(contact);
					if (contact.depth > deepest_edge.depth && contact.depth < 0.05f)
						deepest_edge.set(contact);
					std::cout << "edge contact" << std::endl;
					std::cout << "  A edge: " << i << std::endl;
					std::cout << "  B edge: " << j << std::endl;
				}
			}
		}

		if (deepest_edge.collided)
		{
			if (deepest_vertex.collided)
			{
				// check the shallower depth
				if (deepest_edge.depth > deepest_vertex.depth)
					contacts.push_back(deepest_edge);
				else
					contacts.push_back(deepest_vertex);
			}
			else
			{
				contacts.push_back(deepest_edge);
			}
		}
		else
		{
			if (deepest_vertex.collided)
			{
				contacts.push_back(deepest_vertex);
			}
		}
	}
}