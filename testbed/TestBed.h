#pragma once

#include <stdlib.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "Test.h"
#include "TestCamera.h"

#include "../debug/DebugRenderer.h"

#include "../ImGui/imgui.h"

class TestBed
{
public:
	Test* curr_test;
	TestCamera* camera;

	DebugRenderer renderer;

	bool paused;
	bool swapping_test;
	bool updating_physics;

	TestBed() : curr_test(nullptr), renderer(nullptr), paused(false), swapping_test(false), updating_physics(false)
	{
		camera = &renderer.camera;

		shapes.initShapes(&renderer);
	}
	~TestBed()
	{
		if (curr_test)
			delete(curr_test);

		shapes.deleteShapes();
	}

	void update(float dt)
	{
		if (curr_test && !paused)
		{
			updating_physics = true;
			curr_test->update(dt);
			updating_physics = false;
		}
	}

	void processInput(GLFWwindow* window, float dt)
	{
		if (curr_test)
			curr_test->processInput(window, dt);
	}

	void renderBodyGui(Body* body)
	{
		ImGui::DragFloat3("pos", &body->pos.x, 0.01f);

		glm::vec4 angle_axis = glm::vec4(0.0f);
		angle_axis.w = acosf(body->orientation.w) * 2.0f;
		float axis_div = sinf(angle_axis.w * 0.5f);
		angle_axis.x = body->orientation.x / axis_div;
		angle_axis.y = body->orientation.y / axis_div;
		angle_axis.z = body->orientation.z / axis_div;
		if (ImGui::DragFloat4("orientation", &angle_axis.x, 0.01f))
		{
			glm::vec3 axis = glm::vec3(angle_axis.x, angle_axis.y, angle_axis.z);
			axis = glm::normalize(axis);
			body->orientation = glm::angleAxis(angle_axis.w, axis);
		}

		if (body->type == fiz::BodyType::DYNAMIC)
		{
			DynamicBody* d_body = (DynamicBody*)body;
			ImGui::DragFloat3("vel", &d_body->vel.x, 0.01f);
			ImGui::DragFloat3("angular vel", &d_body->angular_vel.x, 0.01f);
		}

		ImGui::NewLine();

		ImGui::SliderFloat("restitution", &body->restitution, 0.0f, 1.0f);
		ImGui::SliderFloat("friction", &body->friction, 0.0f, 1.0f);

		if (ImGui::Button("Save as Prefab"))
		{
			std::cout << "Saving as prefab..." << std::endl;
			// save this body as a prefab
		}
	}

	void renderWorldImGui()
	{
		if (!curr_test)
			return;

		std::string sec_time = "sec/frame: " + std::to_string(curr_test->last_update_time);
		std::string fps = "fps: " + std::to_string(1.0f / curr_test->last_update_time);
		ImGui::Text(sec_time.c_str());
		ImGui::Text(fps.c_str());

		World* world = &curr_test->world;
		std::string dynamic_bodies_text = "Dynamic Bodies: " + std::to_string(world->dynamic_bodies.size());
		ImGui::Text(dynamic_bodies_text.c_str());

		std::string static_bodies_text = "Static Bodies: " + std::to_string(world->static_bodies.size());
		ImGui::Text(static_bodies_text.c_str());

		std::string joints_text = "Joints: " + std::to_string(world->joints.size());
		ImGui::Text(joints_text.c_str());

		std::string bvh_nodes_text = "BVH Nodes: " + std::to_string(world->static_bvh.nodes.size());
		ImGui::Text(bvh_nodes_text.c_str());

		ImGui::DragFloat3("Gravity", &world->gravity.x, 0.01f);

		ImGui::Checkbox("Show Velocities", &renderer.show_velocities);
		ImGui::Checkbox("Show Contact Points", &renderer.show_contact_points);
		ImGui::Checkbox("Show Contact Normals", &renderer.show_contact_normals);
		ImGui::Checkbox("Show Bounding Boxes", &renderer.show_aabb);
		ImGui::Checkbox("Show BVH", &renderer.show_BVH);
		ImGui::Checkbox("Show Sleep State", &renderer.show_sleep);
		ImGui::Checkbox("Render Outlines", &renderer.outline_shapes);

		static int render_mode = 0;
		ImGui::Text("Render Mode: ");
		ImGui::RadioButton("Render Shapes", &render_mode, 0);
		ImGui::RadioButton("Render Colors", &render_mode, 1);

		switch (render_mode)
		{
		case 0:
			renderer.render_mode = RenderMode::RENDER_SHAPES;
			break;
		case 1:
			renderer.render_mode = RenderMode::RENDER_COLORS;
		}

		if (paused)
		ImGui::Text("PAUSED");
		else
		ImGui::Text("Playing");

		if (ImGui::Button("Pause"))
		paused = true;
		ImGui::SameLine();
		if (ImGui::Button("Play"))
		paused = false;
		if (ImGui::Button("Step") && paused && curr_test)
		curr_test->update(0.01667f);

		ImGui::NewLine();

		if (ImGui::Button("Save World"))
		{
			// save world to file
			std::cout << "Saving world to file..." << std::endl;
		}

		if (curr_test)
		{
			curr_test->renderImGui();
		}
	}

	void renderCreateImGui()
	{
		// render primitive images
		ImVec4 draw_colf = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
		ImU32 draw_col = ImColor(draw_colf);
		ImDrawList* draw_list = ImGui::GetWindowDrawList();

		draw_list->AddCircleFilled(ImVec2(12, 12), 10, draw_col, 20);

		if (ImGui::Button("Load Polyhedron"))
		{
			// load from file
			std::cout << "Loading from file..." << std::endl;
		}
	}

	void renderTestImGui()
	{
		const char* tests[] = { "Box Test", "Domino Test", "Stack Test", "Bowling Test", "GJK Test", "BVH Test", "Car Test", "Raycast Test" };
		static int selected_test = 0;

		if (ImGui::BeginCombo("Tests", tests[selected_test]))
		{
			if (ImGui::Selectable(tests[0]))
			{
				selected_test = 0;
				setTest(new BoxTest());
			}
			if (ImGui::Selectable(tests[1]))
			{
				selected_test = 1;
				setTest(new DominoTest());
			}
			if (ImGui::Selectable(tests[2]))
			{
				selected_test = 2;
				setTest(new StackTest());
			}
			if (ImGui::Selectable(tests[3]))
			{
				selected_test = 3;
				setTest(new BowlingTest());
			}
			if (ImGui::Selectable(tests[4]))
			{
				selected_test = 4;
				setTest(new GJKTest());
			}
			if (ImGui::Selectable(tests[5]))
			{
				selected_test = 5;
				setTest(new BVHTest());
			}
			if (ImGui::Selectable(tests[6]))
			{
				selected_test = 6;
				setTest(new CarTest());
			}
			if (ImGui::Selectable(tests[7]))
			{
				selected_test = 7;
				setTest(new RaycastTest());
			}

			ImGui::EndCombo();
		}

		ImGui::BeginTabBar("Tab");

		if (ImGui::BeginTabItem("World"))
		{
			renderWorldImGui();
			ImGui::EndTabItem();
		}
		
		if (ImGui::BeginTabItem("Body"))
		{
			renderBodyGui(&curr_test->world.dynamic_bodies[0]);
			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Create"))
		{
			renderCreateImGui();
			ImGui::EndTabItem();
		}

		ImGui::EndTabBar();
	}

	void render()
	{
		renderer.render();
		if (curr_test)
			curr_test->renderDebug(&renderer);
	}

	void setTest(Test* test)
	{
		while (updating_physics) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
		swapping_test = true;
		srand(0);

		delete(curr_test);
		curr_test = test;
		curr_test->setRenderer(&renderer);
		test->initialize();
		renderer.world = &curr_test->world;
		renderer.randomizeColors();
		swapping_test = false;
	}

};