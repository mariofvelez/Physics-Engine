#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <glad/glad.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../stb_image.h"

class DebugModels
{
public:
	unsigned int sphereVAO;
	unsigned int boxVAO;
	unsigned int cylinderVAO;
	unsigned int capsule_cylinderVAO;
	unsigned int capsule_hemisphereVAO;
	std::vector<unsigned int> polyhedronVAO;
	std::vector<unsigned int> polyhedron_vertex_count;

	unsigned int sphere_linesVAO;
	unsigned int box_linesVAO;
	unsigned int cylinder_linesVAO;
	unsigned int capsule_linesVAO;

	unsigned int sphere_texture;
	unsigned int box_texture;
	unsigned int cylinder_texture;
	unsigned int capsule_texture;
	unsigned int polyhedron_texture;
	unsigned int sleep_texture;

	DebugModels(int polyhedron_capacity) : sphereVAO(0), boxVAO(0)
	{
		polyhedronVAO.reserve(polyhedron_capacity);

		// generate models
		{
			const int sphere_width = 20;
			const int sphere_height = 20;

			float sphere_vertices[8 * (sphere_width + 1) * (sphere_height + 1)];
			int sphere_indices[6 * sphere_width * sphere_height];

			createSphere(sphere_width, sphere_height, 1.0f, sphere_vertices, sphere_indices);

			unsigned int sphereVBO;
			glGenBuffers(1, &sphereVBO);
			glBindBuffer(GL_ARRAY_BUFFER, sphereVBO);

			glGenVertexArrays(1, &sphereVAO);
			glBindVertexArray(sphereVAO);

			glBufferData(GL_ARRAY_BUFFER, sizeof(sphere_vertices), sphere_vertices, GL_STATIC_DRAW);

			// vertex positions
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
			glEnableVertexAttribArray(0);

			// vertex normals
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
			glEnableVertexAttribArray(1);

			// vertex texture coordinates
			glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
			glEnableVertexAttribArray(2);

			unsigned int sphereEBO;
			glGenBuffers(1, &sphereEBO);

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sphereEBO);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(sphere_indices), sphere_indices, GL_STATIC_DRAW);

			float sphere_line_vertices[sphere_width * 18];
			int sphere_line_indices[sphere_width * 12];

			createSphereLines(sphere_width * 2, 1.0f, sphere_line_vertices, sphere_line_indices);

			unsigned int sphere_linesVBO;
			glGenBuffers(1, &sphere_linesVBO);
			glBindBuffer(GL_ARRAY_BUFFER, sphere_linesVBO);

			glGenVertexArrays(1, &sphere_linesVAO);
			glBindVertexArray(sphere_linesVAO);

			glBufferData(GL_ARRAY_BUFFER, sizeof(sphere_line_vertices), sphere_line_vertices, GL_STATIC_DRAW);

			// vertex positions
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
			glEnableVertexAttribArray(0);

			unsigned int sphere_linesEBO;
			glGenBuffers(1, &sphere_linesEBO);

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sphere_linesEBO);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(sphere_line_indices), sphere_line_indices, GL_STATIC_DRAW);
		}

		{
			float box_vertices[288];

			createBox(box_vertices);

			unsigned int boxVBO;
			glGenBuffers(1, &boxVBO);
			glBindBuffer(GL_ARRAY_BUFFER, boxVBO);

			glGenVertexArrays(1, &boxVAO);
			glBindVertexArray(boxVAO);

			glBufferData(GL_ARRAY_BUFFER, sizeof(box_vertices), box_vertices, GL_STATIC_DRAW);

			// vertex positions
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
			glEnableVertexAttribArray(0);

			// normals
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
			glEnableVertexAttribArray(1);

			// vertex texture coordinates
			glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
			glEnableVertexAttribArray(2);

			float box_line_vertices[24 * 3];

			createBoxLines(box_line_vertices);

			unsigned int box_linesVBO;
			glGenBuffers(1, &box_linesVBO);
			glBindBuffer(GL_ARRAY_BUFFER, box_linesVBO);

			glGenVertexArrays(1, &box_linesVAO);
			glBindVertexArray(box_linesVAO);

			glBufferData(GL_ARRAY_BUFFER, sizeof(box_line_vertices), box_line_vertices, GL_STATIC_DRAW);

			// vertex positions
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
			glEnableVertexAttribArray(0);
		}

		{
			const unsigned int sides = 40;
			const unsigned int vertex_size = 2 * (sides + 1) + 2 * sides;
			const unsigned int index_size = 2 * sides + 2 * (sides - 2);

			float cylinder_vertices[8 * vertex_size];
			int cylinder_indices[3 * index_size];

			createCylinder(sides, 1.0f, 1.0f, cylinder_vertices, cylinder_indices);

			unsigned int cylinderVBO;
			glGenBuffers(1, &cylinderVBO);
			glBindBuffer(GL_ARRAY_BUFFER, cylinderVBO);

			glGenVertexArrays(1, &cylinderVAO);
			glBindVertexArray(cylinderVAO);

			glBufferData(GL_ARRAY_BUFFER, sizeof(cylinder_vertices), cylinder_vertices, GL_STATIC_DRAW);

			// vertex positions
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
			glEnableVertexAttribArray(0);

			// vertex normals
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
			glEnableVertexAttribArray(1);

			// vertex texture coordinates
			glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
			glEnableVertexAttribArray(2);

			unsigned int cylinderEBO;
			glGenBuffers(1, &cylinderEBO);

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cylinderEBO);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(cylinder_indices), cylinder_indices, GL_STATIC_DRAW);
		}

		{ // cylinder
			const unsigned int sides = 40;
			const unsigned int vertex_size = 2 * (sides + 1);
			const unsigned int index_size = 2 * sides;

			float capsule_cylinder_vertices[8 * vertex_size];
			int capsule_cylinder_indices[3 * index_size];

			createCapsuleCylinder(sides, 1.0f, 1.0f, capsule_cylinder_vertices, capsule_cylinder_indices);

			unsigned int capsule_cylinderVBO;
			glGenBuffers(1, &capsule_cylinderVBO);
			glBindBuffer(GL_ARRAY_BUFFER, capsule_cylinderVBO);

			glGenVertexArrays(1, &capsule_cylinderVAO);
			glBindVertexArray(capsule_cylinderVAO);

			glBufferData(GL_ARRAY_BUFFER, sizeof(capsule_cylinder_vertices), capsule_cylinder_vertices, GL_STATIC_DRAW);

			// vertex positions
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
			glEnableVertexAttribArray(0);

			// vertex normals
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
			glEnableVertexAttribArray(1);

			// vertex texture coordinates
			glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
			glEnableVertexAttribArray(2);

			unsigned int capsuleEBO;
			glGenBuffers(1, &capsuleEBO);

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, capsuleEBO);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(capsule_cylinder_indices), capsule_cylinder_indices, GL_STATIC_DRAW);
		}

		{ // hemisphere
			const unsigned int sides = 40;
			const unsigned int vertex_size = (sides + 1) * (sides + 1);
			const unsigned int index_size = 2 * sides * sides;

			float capsule_hemisphere_vertices[8 * vertex_size];
			int capsule_hemisphere_indices[3 * index_size];

			createCapsuleHemisphere(sides, 1.0f, capsule_hemisphere_vertices, capsule_hemisphere_indices);

			unsigned int capsule_hemisphereVBO;
			glGenBuffers(1, &capsule_hemisphereVBO);
			glBindBuffer(GL_ARRAY_BUFFER, capsule_hemisphereVBO);

			glGenVertexArrays(1, &capsule_hemisphereVAO);
			glBindVertexArray(capsule_hemisphereVAO);

			glBufferData(GL_ARRAY_BUFFER, sizeof(capsule_hemisphere_vertices), capsule_hemisphere_vertices, GL_STATIC_DRAW);

			// vertex positions
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
			glEnableVertexAttribArray(0);

			// vertex normals
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
			glEnableVertexAttribArray(1);

			// vertex texture coordinates
			glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
			glEnableVertexAttribArray(2);

			unsigned int capsuleEBO;
			glGenBuffers(1, &capsuleEBO);

			glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, capsuleEBO);
			glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(capsule_hemisphere_indices), capsule_hemisphere_indices, GL_STATIC_DRAW);
		}

		sphere_texture = textureFromFile("debug/debug_res/sphere_texture.png");
		box_texture = textureFromFile("debug/debug_res/aabb_texture.png");
		cylinder_texture = textureFromFile("debug/debug_res/cylinder_texture.png");
		capsule_texture = textureFromFile("debug/debug_res/capsule_texture.png");
		polyhedron_texture = textureFromFile("debug/debug_res/polyhedron_texture3.png");

		sleep_texture = textureFromFile("debug/debug_res/sleep_texture.png");
	}

	~DebugModels()
	{
		glDeleteVertexArrays(1, &sphereVAO);
		glDeleteVertexArrays(1, &boxVAO);
		glDeleteVertexArrays(1, &cylinderVAO);
		glDeleteVertexArrays(1, &capsule_cylinderVAO);
		glDeleteVertexArrays(1, &capsule_hemisphereVAO);

		for (unsigned int i = 0; i < polyhedronVAO.size(); ++i)
			glDeleteVertexArrays(1, &(polyhedronVAO[i]));

		glDeleteVertexArrays(1, &sphere_linesVAO);
		glDeleteVertexArrays(1, &box_linesVAO);

		//glDeleteTextures(1, )
	}

	fiz::Shape* loadPolyhedron(const std::string& filepath, float scale)
	{
		std::string text;
		std::ifstream file(filepath);

		std::vector<glm::vec3> vertices;
		std::vector<glm::vec3> normals;
		std::vector<glm::vec2> textures;

		std::vector<glm::uvec3> faces;

		std::vector<float> vertex_buffer;
		unsigned int vertex_count = 0;

		while (std::getline(file, text))
		{
			// split current line into tokens
			unsigned int token_length = 0;
			std::string tokens[8];

			std::stringstream ss(text);
			while (ss >> tokens[token_length])
				token_length++;

			if (token_length == 0)
				continue;

			if (tokens[0] == "v")
			{
				float x = std::stof(tokens[1]);
				float y = std::stof(tokens[2]);
				float z = std::stof(tokens[3]);

				glm::vec3 vertex = glm::vec3(x, y, z) * scale;
				vertices.push_back(vertex);
			}
			else if (tokens[0] == "vt")
			{
				float u = std::stof(tokens[1]);
				float v = std::stof(tokens[2]);

				glm::vec2 texture = glm::vec2(u, v);
				textures.push_back(texture);
			}
			else if (tokens[0] == "vn")
			{
				float x = std::stof(tokens[1]);
				float y = std::stof(tokens[2]);
				float z = std::stof(tokens[3]);

				glm::vec3 vertex = glm::vec3(x, y, z);
				normals.push_back(vertex);
			}
			else if (tokens[0] == "f")
			{
				int slash_loc = tokens[1].find("/");
				int slash_loc_next = tokens[1].find("/", slash_loc + 1);
				int v1 = std::stoi(tokens[1].substr(0, slash_loc)) - 1;
				int t1 = std::stoi(tokens[1].substr(slash_loc + 1, slash_loc_next)) - 1;
				int vn1 = std::stoi(tokens[1].substr(slash_loc_next + 1, std::string::npos)) - 1;

				slash_loc = tokens[2].find("/");
				slash_loc_next = tokens[2].find("/", slash_loc + 1);
				int v2 = std::stoi(tokens[2].substr(0, slash_loc)) - 1;
				int t2 = std::stoi(tokens[2].substr(slash_loc + 1, slash_loc_next)) - 1;
				int vn2 = std::stoi(tokens[2].substr(slash_loc_next + 1, std::string::npos)) - 1;

				slash_loc = tokens[3].find("/");
				slash_loc_next = tokens[3].find("/", slash_loc + 1);
				int v3 = std::stoi(tokens[3].substr(0, slash_loc)) - 1;
				int t3 = std::stoi(tokens[3].substr(slash_loc + 1, slash_loc_next)) - 1;
				int vn3 = std::stoi(tokens[3].substr(slash_loc_next + 1, std::string::npos)) - 1;

				vertex_buffer.push_back(vertices[v1].x);
				vertex_buffer.push_back(vertices[v1].y);
				vertex_buffer.push_back(vertices[v1].z);
				vertex_buffer.push_back(normals[vn1].x);
				vertex_buffer.push_back(normals[vn1].y);
				vertex_buffer.push_back(normals[vn1].z);
				vertex_buffer.push_back(textures[t1].x);
				vertex_buffer.push_back(textures[t1].y);

				vertex_buffer.push_back(vertices[v2].x);
				vertex_buffer.push_back(vertices[v2].y);
				vertex_buffer.push_back(vertices[v2].z);
				vertex_buffer.push_back(normals[vn2].x);
				vertex_buffer.push_back(normals[vn2].y);
				vertex_buffer.push_back(normals[vn2].z);
				vertex_buffer.push_back(textures[t2].x);
				vertex_buffer.push_back(textures[t2].y);

				vertex_buffer.push_back(vertices[v3].x);
				vertex_buffer.push_back(vertices[v3].y);
				vertex_buffer.push_back(vertices[v3].z);
				vertex_buffer.push_back(normals[vn3].x);
				vertex_buffer.push_back(normals[vn3].y);
				vertex_buffer.push_back(normals[vn3].z);
				vertex_buffer.push_back(textures[t3].x);
				vertex_buffer.push_back(textures[t3].y);

				glm::uvec3 face = glm::uvec3(v1, v2, v3);
				faces.push_back(face);

				vertex_count += 3;
			}
		}

		unsigned int VBO;
		glGenBuffers(1, &VBO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);

		unsigned int VAO;
		glGenVertexArrays(1, &VAO);
		glBindVertexArray(VAO);

		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertex_buffer.size(), vertex_buffer.data(), GL_STATIC_DRAW);

		// vertex positions
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);

		// vertex normals
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(1);

		// vertex texture coordinates
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
		glEnableVertexAttribArray(2);

		file.close();

		fiz::Polyhedron* shape = new fiz::Polyhedron(vertices.size());

		for (unsigned int i = 0; i < vertices.size(); ++i)
			shape->addVertex(vertices[i]);

		for (unsigned int i = 0; i < faces.size(); ++i)
			shape->addIndex(faces[i]);

		polyhedronVAO.push_back(VAO);
		polyhedron_vertex_count.push_back(vertex_count);

		return (fiz::Shape*)shape;
	}

private:
	void createSphere(int width, int height, float rho, float* vertices, int* indices)
	{
		int vertex_index = 0;
		int indices_index = 0;
		for (int h = 0; h <= height; ++h)
		{
			for (int w = 0; w <= width; ++w)
			{
				// uv coordinates
				float u = ((float)w / (float)width);
				float v = ((float)h / (float)height);

				// spherical angles
				float theta = u * glm::two_pi<float>();
				float phi = v * glm::pi<float>();

				// coordinates
				float x = glm::sin(phi) * glm::cos(theta);
				float z = glm::sin(phi) * glm::sin(theta);
				float y = glm::cos(phi);

				// position
				vertices[vertex_index * 8 + 0] = x * rho;
				vertices[vertex_index * 8 + 1] = -y * rho;
				vertices[vertex_index * 8 + 2] = z * rho;
				// normal
				vertices[vertex_index * 8 + 3] = x;
				vertices[vertex_index * 8 + 4] = -y;
				vertices[vertex_index * 8 + 5] = z;
				// texture coords
				vertices[vertex_index * 8 + 6] = u;
				vertices[vertex_index * 8 + 7] = v;

				++vertex_index;
			}
		}
		for (int h = 0; h < height; ++h)
		{
			for (int w = 0; w < width; ++w)
			{
				int top_left = w * (width + 1) + h;
				int top_right = top_left + 1;
				int bot_left = top_left + width + 1;
				int bot_right = top_right + width + 1;
				indices[indices_index * 6 + 0] = top_left;
				indices[indices_index * 6 + 1] = bot_right;
				indices[indices_index * 6 + 2] = top_right;
				indices[indices_index * 6 + 3] = bot_right;
				indices[indices_index * 6 + 4] = top_left;
				indices[indices_index * 6 + 5] = bot_left;

				++indices_index;
			}
		}
	}

	void createBox(float* vertices)
	{
		static float box_vertices[] = {
			// position          normal              texture coords
			 0.5f, -0.5f, -0.5f, 0.0f,  0.0f, -1.0f, 1.0f, 0.0f,
			-0.5f, -0.5f, -0.5f, 0.0f,  0.0f, -1.0f, 0.0f, 0.0f,
			 0.5f,  0.5f, -0.5f, 0.0f,  0.0f, -1.0f, 1.0f, 1.0f,
			-0.5f,  0.5f, -0.5f, 0.0f,  0.0f, -1.0f, 0.0f, 1.0f,
			 0.5f,  0.5f, -0.5f, 0.0f,  0.0f, -1.0f, 1.0f, 1.0f,
			-0.5f, -0.5f, -0.5f, 0.0f,  0.0f, -1.0f, 0.0f, 0.0f,

			-0.5f, -0.5f,  0.5f, 0.0f,  0.0f,  1.0f, 0.0f, 0.0f,
			 0.5f, -0.5f,  0.5f, 0.0f,  0.0f,  1.0f, 1.0f, 0.0f,
			 0.5f,  0.5f,  0.5f, 0.0f,  0.0f,  1.0f, 1.0f, 1.0f,
			 0.5f,  0.5f,  0.5f, 0.0f,  0.0f,  1.0f, 1.0f, 1.0f,
			-0.5f,  0.5f,  0.5f, 0.0f,  0.0f,  1.0f, 0.0f, 1.0f,
			-0.5f, -0.5f,  0.5f, 0.0f,  0.0f,  1.0f, 0.0f, 0.0f,

			-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f,
			-0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f, 1.0f, 1.0f,
			-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f,
			-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f, 0.0f, 1.0f,
			-0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f, 0.0f, 0.0f,
			-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f, 1.0f, 0.0f,

			 0.5f,  0.5f, -0.5f, 1.0f,  0.0f,  0.0f, 1.0f, 1.0f,
			 0.5f,  0.5f,  0.5f, 1.0f,  0.0f,  0.0f, 1.0f, 0.0f,
			 0.5f, -0.5f, -0.5f, 1.0f,  0.0f,  0.0f, 0.0f, 1.0f,
			 0.5f, -0.5f,  0.5f, 1.0f,  0.0f,  0.0f, 0.0f, 0.0f,
			 0.5f, -0.5f, -0.5f, 1.0f,  0.0f,  0.0f, 0.0f, 1.0f,
			 0.5f,  0.5f,  0.5f, 1.0f,  0.0f,  0.0f, 1.0f, 0.0f,

			-0.5f, -0.5f, -0.5f, 0.0f, -1.0f,  0.0f, 0.0f, 1.0f,
			 0.5f, -0.5f, -0.5f, 0.0f, -1.0f,  0.0f, 1.0f, 1.0f,
			 0.5f, -0.5f,  0.5f, 0.0f, -1.0f,  0.0f, 1.0f, 0.0f,
			 0.5f, -0.5f,  0.5f, 0.0f, -1.0f,  0.0f, 1.0f, 0.0f,
			-0.5f, -0.5f,  0.5f, 0.0f, -1.0f,  0.0f, 0.0f, 0.0f,
			-0.5f, -0.5f, -0.5f, 0.0f, -1.0f,  0.0f, 0.0f, 1.0f,

			-0.5f,  0.5f, -0.5f, 0.0f,  1.0f,  0.0f, 0.0f, 1.0f,
			 0.5f,  0.5f,  0.5f, 0.0f,  1.0f,  0.0f, 1.0f, 0.0f,
			 0.5f,  0.5f, -0.5f, 0.0f,  1.0f,  0.0f, 1.0f, 1.0f,
			-0.5f,  0.5f,  0.5f, 0.0f,  1.0f,  0.0f, 0.0f, 0.0f,
			 0.5f,  0.5f,  0.5f, 0.0f,  1.0f,  0.0f, 1.0f, 0.0f,
			-0.5f,  0.5f, -0.5f, 0.0f,  1.0f,  0.0f, 0.0f, 1.0f
		};

		for (unsigned int i = 0; i < 288; ++i)
		{
			vertices[i] = box_vertices[i];
		}
	}

	void createCylinder(int sides, float rad, float height, float* vertices, int* indices)
	{
		unsigned int vertex_index = 0;
		// side
		// top
		for (unsigned int i = 0; i <= sides; ++i)
		{
			float u = (float)i / (float)sides;
			float angle = u * glm::two_pi<float>();
			float x = cos(angle);
			float y = sin(angle);

			// position
			vertices[vertex_index * 8] = x * rad;
			vertices[vertex_index * 8 + 1] = y * rad;
			vertices[vertex_index * 8 + 2] = height;

			// normals
			vertices[vertex_index * 8 + 3] = x;
			vertices[vertex_index * 8 + 4] = y;
			vertices[vertex_index * 8 + 5] = 0.0f;

			// tex coords
			vertices[vertex_index * 8 + 6] = u;
			vertices[vertex_index * 8 + 7] = 1.0f;
			vertex_index++;
		}
		// bottom
		for (unsigned int i = 0; i <= sides; ++i)
		{
			float u = (float)i / (float)sides;
			float angle = u * glm::two_pi<float>();
			float x = cos(angle);
			float y = sin(angle);

			// position
			vertices[vertex_index * 8] = x * rad;
			vertices[vertex_index * 8 + 1] = y * rad;
			vertices[vertex_index * 8 + 2] = -height;

			// normals
			vertices[vertex_index * 8 + 3] = x;
			vertices[vertex_index * 8 + 4] = y;
			vertices[vertex_index * 8 + 5] = 0.0f;

			// tex coords
			vertices[vertex_index * 8 + 6] = u;
			vertices[vertex_index * 8 + 7] = 0.0f;
			vertex_index++;
		}
		// top circle
		for (unsigned int i = 0; i < sides; ++i)
		{
			float angle = (float)i / (float)sides;
			angle *= glm::two_pi<float>();
			float x = cos(angle);
			float y = sin(angle);
			float u = x * 0.5f + 0.5f;
			float v = y * 0.5f + 0.5f;

			// position
			vertices[vertex_index * 8] = x * rad;
			vertices[vertex_index * 8 + 1] = y * rad;
			vertices[vertex_index * 8 + 2] = height;

			// normals
			vertices[vertex_index * 8 + 3] = 0.0f;
			vertices[vertex_index * 8 + 4] = 0.0f;
			vertices[vertex_index * 8 + 5] = 1.0f;

			// tex coords
			vertices[vertex_index * 8 + 6] = u * 0.5f;
			vertices[vertex_index * 8 + 7] = v;
			vertex_index++;
		}
		// bottom circle
		for (unsigned int i = 0; i < sides; ++i)
		{
			float angle = (float)i / (float)sides;
			angle *= glm::two_pi<float>();
			float x = cos(angle);
			float y = sin(angle);
			float u = x * 0.5f + 0.5f;
			float v = y * 0.5f + 0.5f;

			// position
			vertices[vertex_index * 8] = x * rad;
			vertices[vertex_index * 8 + 1] = y * rad;
			vertices[vertex_index * 8 + 2] = -height;

			// normals
			vertices[vertex_index * 8 + 3] = 0.0f;
			vertices[vertex_index * 8 + 4] = 0.0f;
			vertices[vertex_index * 8 + 5] = -1.0f;

			// tex coords
			vertices[vertex_index * 8 + 6] = u * 0.5f;
			vertices[vertex_index * 8 + 7] = v;
			vertex_index++;
		}

		unsigned int indices_index = 0;

		// start index of each set of vertices
		unsigned int top_side = 0;
		unsigned int bottom_side = sides + 1;
		unsigned int top = bottom_side + sides + 1;
		unsigned int bottom = top + sides;

		// sides
		for (unsigned int i = 0; i < sides; ++i)
		{
			unsigned int top_left = top_side + i;
			unsigned int top_right = top_left + 1;
			unsigned int bottom_left = bottom_side + i;
			unsigned int bottom_right = bottom_left + 1;

			indices[indices_index * 3] = top_right;
			indices[indices_index * 3 + 1] = top_left;
			indices[indices_index * 3 + 2] = bottom_left;
			indices[indices_index * 3 + 3] = bottom_left;
			indices[indices_index * 3 + 4] = bottom_right;
			indices[indices_index * 3 + 5] = top_right;

			indices_index += 2;
		}
		// top
		for (unsigned int i = 2; i < sides; ++i)
		{
			indices[indices_index * 3] = top;
			indices[indices_index * 3 + 1] = top + i - 1;
			indices[indices_index * 3 + 2] = top + i;
			indices_index++;
		}
		// bottom
		for (unsigned int i = 2; i < sides; ++i)
		{
			indices[indices_index * 3] = bottom;
			indices[indices_index * 3 + 1] = bottom + i;
			indices[indices_index * 3 + 2] = bottom + i - 1;
			indices_index++;
		}
	}

	void createCapsuleCylinder(int sides, float rad, float height, float* vertices, int* indices)
	{
		unsigned int vertex_index = 0;
		unsigned int indices_index = 0;

		// cylinder
		// top
		unsigned int cylinder_top_start = vertex_index;
		for (unsigned int i = 0; i <= sides; ++i)
		{
			float u = (float)i / (float)sides;
			float angle = u * glm::two_pi<float>();
			float x = cos(angle);
			float y = sin(angle);

			// position
			vertices[vertex_index * 8] = x * rad;
			vertices[vertex_index * 8 + 1] = y * rad;
			vertices[vertex_index * 8 + 2] = height;

			// normals
			vertices[vertex_index * 8 + 3] = x;
			vertices[vertex_index * 8 + 4] = y;
			vertices[vertex_index * 8 + 5] = 0.0f;

			// tex coords
			vertices[vertex_index * 8 + 6] = u;
			vertices[vertex_index * 8 + 7] = 1.0f;
			vertex_index++;
		}
		// bottom
		unsigned int cylinder_bottom_start = vertex_index;
		for (unsigned int i = 0; i <= sides; ++i)
		{
			float u = (float)i / (float)sides;
			float angle = u * glm::two_pi<float>();
			float x = cos(angle);
			float y = sin(angle);

			// position
			vertices[vertex_index * 8] = x * rad;
			vertices[vertex_index * 8 + 1] = y * rad;
			vertices[vertex_index * 8 + 2] = -height;

			// normals
			vertices[vertex_index * 8 + 3] = x;
			vertices[vertex_index * 8 + 4] = y;
			vertices[vertex_index * 8 + 5] = 0.0f;

			// tex coords
			vertices[vertex_index * 8 + 6] = u;
			vertices[vertex_index * 8 + 7] = 0.0f;
			vertex_index++;
		}
		// cylinder
		for (unsigned int i = 0; i < sides; ++i)
		{
			unsigned int top_left = cylinder_top_start + i;
			unsigned int top_right = top_left + 1;
			unsigned int bottom_left = cylinder_bottom_start + i;
			unsigned int bottom_right = bottom_left + 1;

			indices[indices_index * 3] = top_right;
			indices[indices_index * 3 + 1] = top_left;
			indices[indices_index * 3 + 2] = bottom_left;
			indices[indices_index * 3 + 3] = bottom_left;
			indices[indices_index * 3 + 4] = bottom_right;
			indices[indices_index * 3 + 5] = top_right;

			indices_index += 2;
		}
	}

	void createCapsuleHemisphere(int sides, float rad, float* vertices, int* indices)
	{
		unsigned int vertex_index = 0;
		unsigned int indices_index = 0;

		// top hemisphere
		for (int h = 0; h <= sides; ++h)
		{
			for (int w = 0; w <= sides; ++w)
			{
				// uv coordinates
				float u = ((float)w / (float)sides);
				float v = ((float)h / (float)sides);

				// spherical angles
				float theta = u * glm::two_pi<float>();
				float phi = v * glm::pi<float>() * 0.5f;

				// coordinates
				float x = glm::cos(theta) * glm::sin(phi);
				float y = glm::sin(theta) * glm::sin(phi);
				float z = glm::cos(phi);

				// position
				vertices[vertex_index * 8 + 0] = x * rad;
				vertices[vertex_index * 8 + 1] = y * rad;
				vertices[vertex_index * 8 + 2] = z * rad;
				// normal
				vertices[vertex_index * 8 + 3] = x;
				vertices[vertex_index * 8 + 4] = y;
				vertices[vertex_index * 8 + 5] = z;
				// texture coords
				vertices[vertex_index * 8 + 6] = u;
				vertices[vertex_index * 8 + 7] = v * 0.5f;

				++vertex_index;
			}
		}
		// top hemisphere
		for (int h = 0; h < sides; ++h)
		{
			for (int w = 0; w < sides; ++w)
			{
				int top_left = w * (sides + 1) + h;
				int top_right = top_left + 1;
				int bot_left = top_left + sides + 1;
				int bot_right = bot_left + 1;
				indices[indices_index * 3 + 0] = top_left;
				indices[indices_index * 3 + 1] = bot_right;
				indices[indices_index * 3 + 2] = top_right;
				indices[indices_index * 3 + 3] = bot_right;
				indices[indices_index * 3 + 4] = top_left;
				indices[indices_index * 3 + 5] = bot_left;

				indices_index += 2;
			}
		}
	}

	void createSphereLines(int length, float rho, float* vertices, int* indices)
	{;
		for (unsigned int i = 0; i < length; ++i)
		{
			float theta = ((float)i / length) * glm::two_pi<float>();
			float a = rho * glm::cos(theta);
			float b = rho * glm::sin(theta);

			// around x axis
			vertices[i * 3] = 0.0f;
			vertices[i * 3 + 1] = a;
			vertices[i * 3 + 2] = b;

			indices[i * 2] = i;
			indices[i * 2 + 1] = (i + 1) % length;

			// around y axis
			vertices[i * 3 + length * 3] = a;
			vertices[i * 3 + length * 3 + 1] = 0.0f;
			vertices[i * 3 + length * 3 + 2] = b;

			indices[i * 2 + length * 2] = i + length;
			indices[i * 2 + length * 2 + 1] = (i + 1) % length + (length);

			// around z axis
			vertices[i * 3 + length * 6] = a;
			vertices[i * 3 + length * 6 + 1] = b;
			vertices[i * 3 + length * 6 + 2] = 0.0f;

			indices[i * 2 + length * 4] = i + length * 2;
			indices[i * 2 + length * 4 + 1] = (i + 1) % length + (length * 2);
		}
	}

	void createBoxLines(float* vertices)
	{
		static float box_line_vertices[] = {
			// bottom
			-0.5f,  0.5f, -0.5f,
			 0.5f,  0.5f, -0.5f,
			 0.5f,  0.5f, -0.5f,
			 0.5f, -0.5f, -0.5f,
			 0.5f, -0.5f, -0.5f,
			-0.5f, -0.5f, -0.5f,
			-0.5f, -0.5f, -0.5f,
			-0.5f,  0.5f, -0.5f,
			// top
			-0.5f,  0.5f,  0.5f,
			 0.5f,  0.5f,  0.5f,
			 0.5f,  0.5f,  0.5f,
			 0.5f, -0.5f,  0.5f,
			 0.5f, -0.5f,  0.5f,
			-0.5f, -0.5f,  0.5f,
			-0.5f, -0.5f,  0.5f,
			-0.5f,  0.5f,  0.5f,
			// middle
			-0.5f,  0.5f, -0.5f,
			-0.5f,  0.5f,  0.5f,
			 0.5f,  0.5f, -0.5f,
			 0.5f,  0.5f,  0.5f,
			 0.5f, -0.5f, -0.5f,
			 0.5f, -0.5f,  0.5f,
			-0.5f, -0.5f, -0.5f,
			-0.5f, -0.5f,  0.5f
		};
		for (unsigned int i = 0; i < 24 * 3; ++i)
		{
			vertices[i] = box_line_vertices[i];
		}
	}

	unsigned int textureFromFile(const std::string& filename, unsigned int texture_type = GL_TEXTURE_2D)
	{
		stbi_set_flip_vertically_on_load(true);

		unsigned int textureID;
		glGenTextures(1, &textureID);

		int width, height, num_components;
		std::cout << "filename: " << filename << std::endl;
		unsigned char* data = stbi_load(filename.c_str(), &width, &height, &num_components, 0);
		std::cout << "stbi loaded texture: " << textureID << std::endl;
		if (data)
		{
			glBindTexture(texture_type, textureID);
			glTexImage2D(texture_type, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
			glGenerateMipmap(texture_type);

			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

			glTexParameteri(texture_type, GL_TEXTURE_WRAP_S, GL_REPEAT);
			glTexParameteri(texture_type, GL_TEXTURE_WRAP_T, GL_REPEAT);
			glTexParameteri(texture_type, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
			glTexParameteri(texture_type, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

			stbi_image_free(data);
		}
		else
		{
			std::cout << "Texture failed to load at path: " << filename << std::endl;
			stbi_image_free(data);
		}
		return textureID;
	}
};