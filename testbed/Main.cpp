#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <iostream>
#include <stdlib.h>
#include <thread>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <gl_util/Shader.h>
#include <gl_util/Camera.h>

#include "../ImGui/imgui.h"
#include "../ImGui/imgui_impl_glfw.h"
#include "../ImGui/imgui_impl_opengl3.h"

#include "../physics/World.h"
#include "../debug/DebugRenderer.h"

#include "Test.h"
#include "TestBed.h"

#include <vector>
#include <ctime>

#define DEBUG_LOG
#define DEBIG_TIME

#ifdef DEBUG_LOG
#define print(x) std::cout << x << std::endl
#endif
#ifndef DEBUG_LOG
#define print(x)
#endif
#ifdef DEBIG_TIME
#define time(x) std::time_t t1 = std::time(nullptr); x; std::time_t t2 = std::time(nullptr); std::cout << "time: " << (t2 - t1) << std::endl
#endif
#ifndef DEBIG_TIME
#define time(x)
#endif

const int window_width = 900;
const int window_height = 600;

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}
void mouseCallback(GLFWwindow* window, double xpos, double ypos)
{
	static float last_X = window_width / 2;
	static float last_Y = window_height / 2;

	static bool firstMouse = true;
	if (firstMouse)
	{
		last_X = xpos;
		last_Y = ypos;
		firstMouse = false;
	}

	float offsetX = xpos - last_X;
	float offsetY = last_Y - ypos;

	last_X = xpos;
	last_Y = ypos;

	static float sensitivity = 0.001f;
	offsetX *= sensitivity;
	offsetY *= sensitivity;

	//camera->onMouseCallback(offsetX, offsetY);
}
void scrollCallback(GLFWwindow* window, double offsetX, double offsetY)
{
	//camera->onScrollCallback(offsetX, offsetY);
}
void processInput(GLFWwindow* window, DebugRenderer& renderer, float deltaTime)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	renderer.processInput(window, deltaTime);
}

int main()
{
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	GLFWwindow* window = glfwCreateWindow(window_width, window_height, "Physics Engine", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	glViewport(0, 0, window_width, window_height);

	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouseCallback);
	glfwSetScrollCallback(window, scrollCallback);

	// ImGui initialization
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(window, true);          // Second param install_callback=true will install GLFW callbacks and chain to existing ones.
	ImGui_ImplOpenGL3_Init();

	//glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// ground
	const float ground_width = 50;
	const float ground_height = 50;

	float ground_vertices[] = {
		-ground_width,  ground_height, 0.0f, 0.0f, 0.0f, 1.0f,
		 ground_width,  ground_height, 0.0f, 0.0f, 0.0f, 1.0f,
		 ground_width, -ground_height, 0.0f, 0.0f, 0.0f, 1.0f,
		-ground_width, -ground_height, 0.0f, 0.0f, 0.0f, 1.0f
	};
	int ground_indices[] = {
		0, 1, 2,
		2, 3, 0
	};

	unsigned int VBO;
	glGenBuffers(1, &VBO);

	unsigned int VAO;
	glGenVertexArrays(1, &VAO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(ground_vertices), ground_vertices, GL_STATIC_DRAW);

	// vertex positions
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	// vertex normals
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);

	unsigned int EBO;
	glGenBuffers(1, &EBO);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(ground_indices), ground_indices, GL_STATIC_DRAW);

	TestBed* testbed = new TestBed();
	testbed->setTest(new BoxTest());

	bool physics_running = true;
	bool* physics_running_ptr = &physics_running;
	std::thread physics_thread([physics_running_ptr, testbed]() {

		using namespace std::literals::chrono_literals;

		std::chrono::duration<double> target_frame_time = 0.0166s;

		while (*physics_running_ptr)
		{
			std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

			if (!testbed->swapping_test)
			{
				testbed->update(0.01667f);
			}
			
			std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

			std::chrono::duration<double> dt = end - begin;

			std::chrono::duration<double> sleep_time = target_frame_time - dt;
			if (sleep_time < 0.0s)
				sleep_time = 0.0s;

			float time = float(std::chrono::duration_cast<std::chrono::nanoseconds>(dt).count()) * 0.000000001f;
			if (testbed->curr_test)
				testbed->curr_test->setLastUpdateTime(time);
			//std::cout << "time: " << time << std::endl;

			std::this_thread::sleep_for(sleep_time);
		}

		std::cout << "physics finished" << std::endl;

	});

	glEnable(GL_DEPTH_TEST);

	//glEnable(GL_CULL_FACE);

	float deltaTime = 0.0f;
	float lastFrame = 0.0f;

	// render loop
	bool play_pressed = false;
	bool last_pressed = false;
	while (!glfwWindowShouldClose(window))
	{
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		//ImGui::ShowDemoWindow();

		if (ImGui::Begin("Debug Panel"))
		{
			testbed->renderTestImGui();
		}
		ImGui::End();

		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		//input
		processInput(window, testbed->renderer, deltaTime);
		
		testbed->processInput(window, deltaTime);

		last_pressed = play_pressed;

		if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
			play_pressed = true;
		if (glfwGetKey(window, GLFW_KEY_P) == GLFW_RELEASE)
			play_pressed = false;

		/*if (!last_pressed && play_pressed)
		{
			test.update(0.01667f);
		}*/

		//rendering commands here
		glm::vec3 clear_col = glm::vec3(0.0f);// glm::vec3(152.0f, 229.0f, 237.0f);
		//clear_col /= 255.0f;
		glClearColor(clear_col.x, clear_col.y, clear_col.z, 0.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		testbed->render();
		/*glm::vec3 world_a = test.spring_joint.body->getWorldPos(test.spring_joint.local);
		glm::vec3 world_b = test.spring_joint.anchor;
		renderer->drawLine(glm::vec3(1.0f, 0.129f, 0.318f), world_a, world_b);*/

		glBindVertexArray(VAO);
		//glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	physics_running = false;
	physics_thread.join();

	delete(testbed);

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwTerminate();

	return 0;
}