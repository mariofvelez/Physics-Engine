#pragma once

#include <stdlib.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "../ImGui/imgui.h"

#include"../physics/World.h"
#include "../physics/BodyDef.h"
#include "../physics/geometry/Shape.h"
#include "../physics/geometry/Collision.h"
#include "../physics/Joint.h"

#include "../physics/acceleration/BVH.h"

using namespace fiz;


class TestShapes
{
public:
	Shape* box;
	Shape* box2;
	Shape* cube;
	Shape* domino;
	Shape* sphere;
	Shape* long_cylinder;
	Shape* flat_cylinder;
	Shape* small_capsule;
	Shape* medium_capsule;
	Shape* d_4;
	Shape* d_8;
	Shape* d_20;
	Shape* car_chassis;

	void initShapes(DebugRenderer* renderer)
	{
		box = new Box(glm::vec3(0.0f), glm::vec3(0.8f, 0.6f, 0.4f));
		box2 = new Box(glm::vec3(0.0f), glm::vec3(0.6f, 0.4f, 0.4f));
		cube = new Box(glm::vec3(0.0f), glm::vec3(0.5f));
		domino = new Box(glm::vec3(0.0f), glm::vec3(0.1f, 0.4f, 0.8f));
		sphere = new Sphere(glm::vec3(0.0f), 0.5f);
		long_cylinder = new Cylinder(glm::vec3(0.0f), 0.2f, 0.5f);
		flat_cylinder = new Cylinder(glm::vec3(0.0f), 0.5f, 0.1f);
		small_capsule = new Capsule(glm::vec3(0.0f), 0.2f, 0.2f);
		medium_capsule = new Capsule(glm::vec3(0.0f), 0.3f, 0.4f);
		d_4 = renderer->loadPolyhedron("objects/d_4.obj", 15.0f);
		d_8 = renderer->loadPolyhedron("objects/d_8.obj", 15.0f);
		d_20 = renderer->loadPolyhedron("objects/d_20.obj", 15.0f);
		car_chassis = new Box(glm::vec3(0.0f), glm::vec3(0.4f, 0.8f, 0.4f));
	}

	void deleteShapes()
	{
		delete(box);
		delete(box2);
		delete(cube);
		delete(domino);
		delete(sphere);
		delete(long_cylinder);
		delete(flat_cylinder);
		delete(small_capsule);
		delete(medium_capsule);
		delete(car_chassis);
	}
};

TestShapes shapes;

class Test
{
public:

	fiz::World world;

	float last_update_time = 0.0f;

	virtual void initialize() = 0;

	virtual void update(float dt)
	{
		world.step(dt);
	}

	virtual void processInput(GLFWwindow* window, float dt) {}

	void setLastUpdateTime(float time)
	{
		last_update_time = time;
	}

	virtual void renderImGui() {}

	virtual void renderDebug(DebugRenderer* renderer) {}

	void setRenderer(DebugRenderer* debug_renderer)
	{
		renderer = debug_renderer;
	}

protected:

	DebugRenderer* renderer;

	inline float random()
	{
		return (float)(rand() % 1000) / 1000.0f;
	}
	inline float random(float min, float max)
	{
		return min + random() * (max - min);
	}
	
};

class GJKTest : public Test
{
public:

	Body* a;
	Body* b;

	Body* curr_body;

	bool intersecting;
	ContactInfo contact;

	GJKTest() : intersecting(false)
	{
		BodyDef bd;
		bd.pos = glm::vec3(-1.0f, 0.0f, 1.0f);
		//bd.shape = new Sphere(glm::vec3(0.0f), 0.5f);
		bd.shape = shapes.box;
		a = world.createBody(bd);

		bd.pos = glm::vec3(1.0f, 0.0f, 1.0f);
		b = world.createBody(bd);

		curr_body = a;
	}
	~GJKTest()
	{

	}

	void initialize()
	{

	}

	void renderImGui()
	{
		if (intersecting)
			ImGui::Text("GJK intersecting");
		else
			ImGui::Text("GJK not intersecting");

		if (contact.collided)
			ImGui::Text("EPA Colliding");
		else
			ImGui::Text("EPA not colliding");
	}

	void update(float dt)
	{
		world.contacts.clear();
		contact.collided = false;

		intersecting = GJK(a, b, glm::vec3(1.0f, 0.0f, 0.0f));
		/*for (unsigned int i = 0; i < s.n; ++i)
		{
			ContactInfo contact;
			contact.collided = true;
			contact.poc = s.v[i];
			world.contacts.push_back(contact);
			std::cout << s.n << std::endl;
		}*/
		if (intersecting)
		{
			contact = EPA(a, b);
			if (contact.collided)
			{
				/*a->pos -= contact.normal * contact.depth * 0.5f;
				b->pos += contact.normal * contact.depth * 0.5f;*/
				world.contacts.push_back(contact);
			}
		}
	}

	void rotateBody(glm::vec3 axis, float angle)
	{
		float a = glm::sin(0.5f * angle);
		glm::quat q = glm::quat(glm::cos(0.5f * angle), axis.x * a, axis.y * a, axis.z * a);

		curr_body->orientation = glm::normalize(q * curr_body->orientation);
		curr_body->updateOrientationMat();
		((DynamicBody*)curr_body)->updateInverseInertiaWorld();
	}

	void processInput(GLFWwindow* window, float dt)
	{
		float speed = 2.0f;
		
		// set current body
		if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
			curr_body = a;
		if (glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS)
			curr_body = b;

		// translate
		if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS)
			curr_body->pos.x -= speed * dt;
		if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
			curr_body->pos.x += speed * dt;
		if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS)
			curr_body->pos.y += speed * dt;
		if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS)
			curr_body->pos.y -= speed * dt;
		if (glfwGetKey(window, GLFW_KEY_Y) == GLFW_PRESS)
			curr_body->pos.z += speed * dt;
		if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS)
			curr_body->pos.z -= speed * dt;
		
		// rotate
		if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS)
			rotateBody(glm::vec3(0.0f, 0.0f, 1.0f), speed * dt);
		if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS)
			rotateBody(glm::vec3(0.0f, 0.0f, 1.0f), -speed * dt);
		if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
			rotateBody(glm::vec3(1.0f, 0.0f, 0.0f), -speed * dt);
		if (glfwGetKey(window, GLFW_KEY_SEMICOLON) == GLFW_PRESS)
			rotateBody(glm::vec3(1.0f, 0.0f, 0.0f), speed * dt);
	}

	void renderDebug(DebugRenderer* renderer)
	{
		renderer->renderSimplex(s);
		renderer->setSphereColor(glm::vec3(1.0f));
		renderer->renderSphere(glm::vec3(0.0f), 0.05f);

		std::vector<glm::vec3> vec_a;
		((Box*)a->shapes[0])->projectVertices(a->orientation_mat, a->pos, vec_a);
		std::vector<glm::vec3> vec_b;
		((Box*)b->shapes[0])->projectVertices(b->orientation_mat, b->pos, vec_b);

		renderer->renderMinkowskiDifference(vec_a, vec_b);
	}
};

class DominoTest : public Test
{
public:
	DominoTest()
	{
		world = World();
	}

	void initialize()
	{
		glm::vec3 curr_pos = glm::vec3(-4.0f, 0.0f, 0.8f);
		float curr_angle = 0.0f;
		float dist = 1.0f;
		
		BodyDef bd;
		bd.shape = shapes.domino;
		float tip_angle = 0.3f;
		bd.orientation = glm::angleAxis(tip_angle, glm::vec3(0.0f, 1.0f, 0.0f));
		for (unsigned int i = 0; i < 64; ++i)
		{
			bd.pos = curr_pos;
			world.createBody(bd);

			curr_angle += random(-0.5f, 0.5f);
			bd.orientation = glm::angleAxis(curr_angle, glm::vec3(0.0f, 0.0f, 1.0f));
			curr_pos.x += cos(curr_angle) * dist;
			curr_pos.y += sin(curr_angle) * dist;
		}
	}
};

class StackTest : public Test
{
	Shape* stack_shape;

	float offset = 0.2f;
	float height = 0.8f;
	float spacing = 0.1f;

public:
	StackTest()
	{
		world = World();
		stack_shape = new Box(glm::vec3(0.0f), glm::vec3(0.5f, 0.5f, height * 0.5f));
	}
	~StackTest()
	{
		delete(stack_shape);
	}

	void initialize()
	{
		BodyDef bd;
		bd.shape = stack_shape;
		bd.pos = glm::vec3(0.0f, 0.0f, height * 0.5f + spacing);
		bd.friction = 0.25f;
		bd.restitution = 0.1f;

		for (unsigned int i = 0; i < 3; ++i)
		{
			float angle = random(-3.14f, 3.14f);
			bd.orientation = glm::angleAxis(angle, glm::vec3(0.0f, 0.0f, 1.0f));

			world.createBody(bd);

			bd.pos.x = random(-offset, offset);
			bd.pos.y = random(-offset, offset);
			bd.pos.z += height + spacing;
		}
	}
};

class BowlingTest : public Test
{
	Shape* pin_shape;

public:
	BowlingTest()
	{
		world = World();
		pin_shape = new Cylinder(glm::vec3(0.0f), 0.2f, 1.0f);
	}
	~BowlingTest()
	{
		delete(pin_shape);
	}

	void initialize()
	{
		BodyDef bd;
		bd.shape = pin_shape;
		bd.friction = 0.3f;

		float spacing = 1.0f;
		float dist = 15.0f;
		glm::vec3 pos[] = {
			glm::vec3(0.0f, dist, 1.0f),

			glm::vec3(-spacing * 0.5f, dist + spacing, 1.0f),
			glm::vec3(spacing * 0.5f, dist + spacing, 1.0f),

			glm::vec3(-spacing, dist + spacing * 2.0f, 1.0f),
			glm::vec3(0.0f, dist + spacing * 2.0f, 1.0f),
			glm::vec3(spacing, dist + spacing * 2.0f, 1.0f),

			glm::vec3(-spacing * 1.5f, dist + spacing * 3.0f, 1.0f),
			glm::vec3(-spacing * 0.5f, dist + spacing * 3.0f, 1.0f),
			glm::vec3(spacing * 0.5f, dist + spacing * 3.0f, 1.0f),
			glm::vec3(spacing * 1.5f, dist + spacing * 3.0f, 1.0f)
		};

		for (unsigned int i = 0; i < 10; ++i)
		{
			bd.pos = pos[i];
			world.createBody(bd);
		}

		bd.shape = new Sphere(glm::vec3(0.0f), 0.7f);
		bd.pos = glm::vec3(0.0f, -2.0f, 1.5f);
		bd.friction = 0.1f;
		bd.vel = glm::vec3(1.0f, 10.0f, 0.0f);
		bd.angular_vel = glm::vec3(0.0f, -40.0f, 0.0f);
		bd.linear_damping = 1.0f;
		bd.angular_damping = 1.0f;
		world.createBody(bd);
	}
};

class BoxTest : public Test
{
public:
	DynamicBody* body;
	DynamicBody* body2;
	DynamicBody* player;
	BoxTest()
	{
		world = World();
	}

	void initialize()
	{
		float spawn_width = 5.0f;
		float spawn_depth = 5.0f;
		float spawn_height = 10.0f;

		BodyDef bd;
		bd.angular_damping = 0.99f;
		bd.restitution = 0.2f;
		/*Shape* box = new Box(glm::vec3(0.0f), glm::vec3(0.1f, 0.1f, 0.15f));
		Shape* sphere = new Sphere(glm::vec3(0.0f), 0.15f);
		Shape* cylinder = new Cylinder(glm::vec3(0.0f), 0.1f, 0.2f);*/
		for (unsigned int i = 0; i < 64; ++i)
		{
			glm::vec3 axis = glm::vec3(random(-1.0f, 1.0f), random(-1.0f, 1.0f), random(-1.0f, 1.0f));
			axis = glm::normalize(axis);
			float angle = random(2.0f, -2.0f);
			bd.orientation = glm::angleAxis(angle, axis);
			float rand_shape = random();
			if (rand_shape < 0.125f)
			{
				bd.shape = shapes.box2;
				bd.type = BodyType::DYNAMIC;
				bd.rotation_locked = false;
			}
			else if (rand_shape < 0.25f)
			{
				bd.shape = shapes.sphere;
				bd.type = BodyType::DYNAMIC;
				bd.rotation_locked = false;
			}
			else if (rand_shape < 0.375f)
			{
				bd.shape = shapes.long_cylinder;
				bd.type = BodyType::DYNAMIC;
				bd.rotation_locked = false;
			}
			else if (rand_shape < 0.5f)
			{
				bd.shape = shapes.medium_capsule;
				bd.type = BodyType::DYNAMIC;
				//bd.rotation_locked = true;
				bd.orientation = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
			}
			else if (rand_shape < 0.625f)
			{
				bd.shape = shapes.d_4;
				bd.type = BodyType::DYNAMIC;
				bd.rotation_locked = false;
			}
			else if (rand_shape < 0.75f)
			{
				bd.shape = shapes.d_8;
				bd.type = BodyType::DYNAMIC;
				bd.rotation_locked = false;
			}
			else if (rand_shape < 0.875f)
			{
				bd.shape = shapes.d_20;
				bd.type = BodyType::DYNAMIC;
				bd.rotation_locked = false;
			}
			else
			{
				bd.shape = shapes.flat_cylinder;
				bd.type = BodyType::DYNAMIC;
				bd.rotation_locked = false;
			}
			bd.friction = 0.5f;
			bd.pos.x = random(-spawn_width * 0.5f, spawn_width * 0.5f);
			bd.pos.y = random(-spawn_depth * 0.5f, spawn_depth * 0.5f);
			bd.pos.z = random(0.5f, spawn_height);
			world.createBody(bd);
		}

		bd.pos = glm::vec3(0.0f, 0.0f, 5.0f);
		bd.shape = shapes.medium_capsule;
		body = (DynamicBody*)world.createBody(bd);

		AnchoredBallJoint* joint = new AnchoredBallJoint();
		joint->body = (DynamicBody*)body;
		joint->anchor = glm::vec3(0.0f, 0.0f, 6.0f);
		joint->local = glm::vec3(0.0f, 0.0f, 0.4f);
		world.addJoint(joint);

		bd.pos.z = 4.0f;
		body2 = (DynamicBody*)world.createBody(bd);

		BallJoint* joint2 = new BallJoint();
		joint2->a = (DynamicBody*)body;
		joint2->b = (DynamicBody*)body2;
		joint2->local_a = glm::vec3(0.0f, 0.0f, -0.8f);
		joint2->local_b = glm::vec3(0.0f, 0.0f, 0.8f);
		world.addJoint(joint2);

		bd.pos = glm::vec3(0.0f, -5.0f, 1.0f);
		bd.shape = shapes.medium_capsule;
		bd.orientation = glm::quat(0.0f, 0.0f, 0.0f, 0.0f);
		bd.rotation_locked = true;
		bd.friction = 0.5f;
		bd.restitution = 0.0f;
		player = (DynamicBody*)world.createBody(bd);

		createChain(glm::vec3(5.0f, 0.0f, 5.0f));
		createChain(glm::vec3(6.0f, 0.0f, 5.0f));
		createChain(glm::vec3(7.0f, 0.0f, 5.0f));
		createChain(glm::vec3(8.0f, 0.0f, 5.0f));
		createChain(glm::vec3(9.0f, 0.0f, 5.0f));
		
		/*bd.pos = glm::vec3(1.0f, 0.0f, 5.4f);
		bd.shape = box;
		bd.rotation_locked = false;
		DynamicBody* rev_a = (DynamicBody*)world.createBody(bd);

		bd.pos = glm::vec3(2.6f, 1.2f, 5.4f);
		DynamicBody* rev_b = (DynamicBody*)world.createBody(bd);

		RevoluteJoint* rev_joint = new RevoluteJoint();
		rev_joint->a = rev_a;
		rev_joint->b = rev_b;
		rev_joint->local_a = glm::vec3(0.8f, 0.6f, 0.0f);
		rev_joint->local_b = glm::vec3(-0.8f, -0.6f, 0.0f);
		world.addJoint(rev_joint);*/
	}

	void update(float dt)
	{
		world.step(dt);
	}

	void processInput(GLFWwindow* window, float dt)
	{
		float speed = 4.0f;

		player->setAwake();
		player->vel.x = 0.0f;
		player->vel.y = 0.0f;
		if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS)
			player->vel.x = -speed;
		if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
			player->vel.x = speed;
		if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS)
			player->vel.y = speed;
		if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS)
			player->vel.y = -speed;

		if (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS)
			player->vel.z = speed;
	}

private:
	void createChain(glm::vec3 pos)
	{
		BodyDef bd;
		float capsule_rad = 0.2f;
		float height = capsule_rad * 4.0f;
		float spacing = 0.1f;
		bd.shape = shapes.small_capsule;
		bd.friction = 0.2f;
		bd.restitution = 0.1f;
		bd.linear_damping = 0.999f;
		bd.angular_damping = 0.999f;
		DynamicBody* prev = nullptr;
		for (unsigned int i = 0; i < 5; ++i)
		{
			bd.pos = pos;
			bd.pos.z -= i * (height + spacing);
			DynamicBody* body = (DynamicBody*)world.createBody(bd);

			if (prev == nullptr)
			{
				AnchoredBallJoint* joint = new AnchoredBallJoint();
				joint->body = body;
				joint->local = glm::vec3(0.0f, 0.0f, height * 0.5f + spacing * 0.5f);
				joint->anchor = pos;
				world.addJoint(joint);
			}
			else
			{
				BallJoint* joint = new BallJoint();
				joint->a = prev;
				joint->b = body;
				joint->local_a = glm::vec3(0.0f, 0.0f, -height * 0.5f - spacing * 0.5f);
				joint->local_b = glm::vec3(0.0f, 0.0f, height * 0.5f + spacing * 0.5f);
				world.addJoint(joint);
			}

			prev = body;
		}
	}

};

class BVHTest : public Test
{
public:

	float width = 100.0f;
	float height = 70.0f;

	BVHTest()
	{
		
	}

	void initialize()
	{
		BodyDef bd;
		bd.type = fiz::BodyType::STATIC;
		Shape* sphere = new fiz::Sphere(glm::vec3(0.0f), 0.5f);
		bd.shape = sphere;
		bd.pos = glm::vec3(0.0f);
		
		for (unsigned int i = 0; i < 10000; ++i)
		{
			bd.pos.x = random(-width * 0.5f, width * 0.5f);
			bd.pos.y = random(-width * 0.5f, width * 0.5f);
			bd.pos.z = random(5.0f, height + 5.0f);
			world.createBody(bd);
		}

		world.buildBVH();

		bd.type = fiz::BodyType::DYNAMIC;
		bd.pos.z = height + 6.0f;
		bd.shape = new fiz::Box(glm::vec3(0.0f), glm::vec3(0.5f));
		for (unsigned int i = 0; i < 100; ++i)
		{
			bd.pos.x = random(-width * 0.5f, width * 0.5f);
			bd.pos.y = random(-width * 0.5f, width * 0.5f);
			world.createBody(bd);
		}
	}

	void update(float dt)
	{
		world.step(dt);

		for (unsigned int i = 0; i < world.dynamic_bodies.size(); ++i)
		{
			DynamicBody& body = world.dynamic_bodies[i];
			if (body.pos.z < 2.5f)
			{
				body.pos.x = random(-width * 0.5f, width * 0.5f);
				body.pos.y = random(-width * 0.5f, width * 0.5f);
				body.pos.z = height + 6.0f;
				body.vel = glm::vec3(0.0f);
				body.angular_vel = glm::vec3(0.0f);
			}
		}
	}
};

class CarTest : public Test
{
public:
	RevoluteJoint* wheel_joints[4];

	CarTest()
	{
		
	}
	~CarTest()
	{
		for (unsigned int i = 0; i < 4; ++i)
			delete(wheel_joints[i]);
	}

	void initialize()
	{
		Body* wheel_bodies[4];
		Body* chassis_body;

		BodyDef wheel_bd;
		wheel_bd.type = BodyType::DYNAMIC;
		wheel_bd.shape = shapes.flat_cylinder;
		wheel_bd.orientation = glm::angleAxis(glm::half_pi<float>(), glm::vec3(0.0f, -1.0f, 0.0f));

		glm::vec3 wheel_pos[] = {
			glm::vec3(-1.0f,  1.0f, 1.0f), // front left
			glm::vec3( 1.0f,  1.0f, 1.0f), // front right
			glm::vec3(-1.0f, -1.0f, 1.0f), // back left
			glm::vec3( 1.0f, -1.0f, 1.0f)  // back right
		};
		for (unsigned int i = 0; i < 4; ++i)
		{
			wheel_bd.pos = wheel_pos[i];
			wheel_bodies[i] = world.createBody(wheel_bd);
		}
		
		BodyDef chassis_bd;
		chassis_bd.type = BodyType::DYNAMIC;
		chassis_bd.shape = shapes.car_chassis;
		chassis_bd.pos = glm::vec3(0.0f, 0.0f, 1.2f);
		chassis_body = world.createBody(chassis_bd);

		for (unsigned int i = 0; i < 4; ++i)
		{
			wheel_joints[i] = new RevoluteJoint();
			wheel_joints[i]->a = (DynamicBody*)chassis_body;
			wheel_joints[i]->b = (DynamicBody*)wheel_bodies[i];
			wheel_joints[i]->local_a = wheel_pos[i] - chassis_bd.pos;
			wheel_joints[i]->local_b = glm::vec3(0.0f);
			wheel_joints[i]->local_axis_a = glm::vec3(-1.0f, 0.0f, 0.0f);
			wheel_joints[i]->local_axis_b = glm::vec3(0.0f, 0.0f, 1.0f);
			world.addJoint(wheel_joints[i]);
		}
	}

	void update(float dt)
	{
		world.step(dt);
	}
};

class ForceTest : public Test
{
public:
	ForceTest()
	{
		world = World();
	}

	void initialize()
	{
		BodyDef box_def;
		box_def.pos.x = -5.0f;
		box_def.shape = new Box(glm::vec3(0.0f), glm::vec3(0.8f, 0.6f, 0.4f));
		world.createBody(box_def);

		box_def.pos.x = 0.0f;
		world.createBody(box_def);

		box_def.pos.x = 5.0f;
		world.createBody(box_def);
	}

	void update(float dt)
	{
		world.dynamic_bodies[0].applyForceWorld(glm::vec3(0.0f, 0.0f, 1.0f), world.dynamic_bodies[0].pos + glm::vec3(-1.0f, 0.0f, 0.0f));

		world.dynamic_bodies[1].applyForceWorld(glm::vec3(0.0f, 0.0f, 1.0f), world.dynamic_bodies[1].pos + glm::vec3(0.0f, 1.0f, 0.0f));

		world.step(dt);
	}
};

class SpringTest : public Test
{
public:
	AnchoredSpringJoint spring_joint;

	SpringTest() : spring_joint(4.0f, 2.0f)
	{
		world = World();
	}

	void initialize()
	{
		BodyDef box_def;
		box_def.pos.x = 0.0f;
		box_def.pos.y = 0.0f;
		box_def.linear_damping = 1.0f;
		box_def.angular_damping = 0.995f;
		box_def.shape = new Box(glm::vec3(0.0f), glm::vec3(0.8f, 0.6f, 0.4f));
		DynamicBody* box = (DynamicBody*)world.createBody(box_def);

		/*BodyDef sphere_def;
		sphere_def.pos.x = 1.0f;
		sphere_def.pos.y = 1.0f;
		sphere_def.shape = new Sphere(glm::vec3(0.0f), 0.7f);
		Body* sphere = world.createBody(sphere_def);*/

		/*spring_joint.a = box;
		spring_joint.b = sphere;
		spring_joint.local_a = glm::vec3(0.8f, 0.6f, -0.4f);
		spring_joint.local_b = glm::vec3(0.0f, 0.0f, 0.7f);*/
		spring_joint.body = box;
		spring_joint.anchor.z = 7.0f;
		spring_joint.local = glm::vec3(0.8f, 0.6f, 0.4f);

		world.addJoint(&spring_joint);
	}

	void update(float dt)
	{
		world.step(dt);
	}
};

class BlocksTest : public Test
{
public:
	BlocksTest()
	{
		world = World();
	}

	void initialize()
	{
		srand(5);

		for (int i = 0; i < 10; ++i)
		{
			if (random() < 0.5f) // sphere
			{
				float r = random() * 0.5f + 0.5f;
				float x = random() * 10.0f - 5.0f;
				float y = random() * 5.0f + 1.0f;
				float z = random() * 10.0f - 5.0f;
				Sphere* sphere = new Sphere(glm::vec3(0.0f, 0.0f, 0.0f), r);
				world.shapes.push_back((Shape*)sphere);
				world.dynamic_bodies.emplace_back(glm::vec3(x, y, z));
				world.dynamic_bodies[world.dynamic_bodies.size() - 1].addShape((Shape*)sphere);
			}
			else // box
			{
				float x = random() * 10.0f - 5.0f;
				float y = random() * 5.0f + 1.0f;
				float z = random() * 10.0f - 5.0f;

				float sx = random() * 0.5f + 0.5f;
				float sy = random() * 0.5f + 0.5f;
				float sz = random() * 0.5f + 0.5f;

				Box* box = new Box(glm::vec3(0.0f), glm::vec3(sx, sy, sz));
				world.shapes.push_back((Shape*)box);
				world.dynamic_bodies.emplace_back(glm::vec3(x, y, z));
				world.dynamic_bodies[world.dynamic_bodies.size() - 1].addShape((Shape*)box);

				float angle = random() * 10.0f;
				float ax = random() * 2.0f - 1.0f;
				float ay = random() * 2.0f - 1.0f;
				float az = random() * 2.0f - 1.0f;
				glm::vec3 axis = glm::vec3(ax, ay, az);
				axis = glm::normalize(axis);
				axis *= angle + 1.0f;
				//bodies[bodies.size() - 1].angular_vel = axis;

			}
			world.dynamic_bodies[world.dynamic_bodies.size() - 1].updateMassProperties();
		}
	}
};