#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <random>

#include <glad/glad.h>

#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <gl_util/Shader.h>
#include <gl_util/Camera.h>

#include "DebugModels.h"
#include "../physics/World.h"
#include "../physics/Body.h"
#include "../physics/Joint.h"
#include "../physics/geometry/Shape.h"
#include "../testbed/TestCamera.h"
#include "../physics/acceleration/BVH.h"

enum RenderMode
{
	RENDER_COLORS,
	RENDER_SHAPES
};

class DebugRenderer
{
public:
	fiz::World* world;

	TestCamera camera;

	DebugModels models;

	std::vector<fiz::Shape*> polyhedron_shapes;

	std::vector<glm::vec3> static_colors;
	std::vector<glm::vec3> dynamic_colors;

	RenderMode render_mode;

	bool show_contact_points;
	bool show_contact_normals;
	bool show_BVH;
	bool show_velocities;
	bool show_aabb;
	bool show_sleep;
	bool outline_shapes;

	DebugRenderer(fiz::World* world) : world(world), light_direction(0.0f, 0.0f, -1.0f), camera(glm::vec3(0.0f, -10.0f, 5.0f), glm::vec3(0.0f, 1.0f, -0.2f), glm::vec3(0.0f, 0.0f, 1.0f)), models(100), show_contact_points(false), show_contact_normals(false), show_BVH(false), show_velocities(false), show_aabb(false), show_sleep(false), outline_shapes(false), render_mode(RENDER_COLORS)
	{
		// generate shaders
		shape_shader = new Shader("shaders/Vertex.shader", "shaders/Fragment.shader");
		color_shader = new Shader("shaders/Vertex.shader", "shaders/ColorFragment.shader");

		shadow_shader = new Shader("shaders/Vertex.shader", "shaders/Fragment.shader");

		// dir light
		glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f);
		float ambient_strength = 0.5f;

		// shape shader
		shape_shader->use();

		shape_shader->setVec3("dirlight.direction", light_direction);

		shape_shader->setVec3("dirlight.ambient", color.x * ambient_strength, color.y * ambient_strength, color.z * ambient_strength);
		shape_shader->setVec3("dirlight.diffuse", color.x, color.y, color.z);
		shape_shader->setVec3("dirlight.specular", color.x, color.y, color.z);

		shape_model_loc = shape_shader->uniformLoc("model");
		shape_view_loc = shape_shader->uniformLoc("view");
		shape_proj_loc = shape_shader->uniformLoc("projection");

		shape_shader->setInt("uTexture", 0);

		// color shader
		color_shader->use();

		color_shader->setVec3("dirlight.direction", light_direction);

		color_shader->setVec3("dirlight.ambient", color.x * ambient_strength, color.y * ambient_strength, color.z * ambient_strength);
		color_shader->setVec3("dirlight.diffuse", color.x, color.y, color.z);
		color_shader->setVec3("dirlight.specular", color.x, color.y, color.z);

		color_model_loc = color_shader->uniformLoc("model");
		color_view_loc = color_shader->uniformLoc("view");
		color_proj_loc = color_shader->uniformLoc("projection");

		color_color_loc = color_shader->uniformLoc("col");

		// line shader
		line_shader = new Shader("Shaders/LineVertex.shader", "Shaders/BasicFragment.shader");
		line_shader->use();
		line_model_loc = line_shader->uniformLoc("model");
		line_view_loc = line_shader->uniformLoc("view");
		line_proj_loc = line_shader->uniformLoc("projection");
		line_color_loc = line_shader->uniformLoc("color");

		line_segment_shader = new Shader("Shaders/LineSegmentVertex.shader", "Shaders/BasicFragment.shader");
		line_segment_shader->use();
		line_segment_view_loc = line_segment_shader->uniformLoc("view");
		line_segment_proj_loc = line_segment_shader->uniformLoc("projection");
		line_segment_color_loc = line_segment_shader->uniformLoc("color");
		line_segment_a_loc = line_segment_shader->uniformLoc("pos[0]");
		line_segment_b_loc = line_segment_shader->uniformLoc("pos[1]");

		point_shader = new Shader("Shaders/PointVertex.shader", "Shaders/BasicFragment.shader");
		point_shader->use();
		point_view_loc = point_shader->uniformLoc("view");
		point_proj_loc = point_shader->uniformLoc("projection");
		point_scale_loc = point_shader->uniformLoc("scale");
		point_color_loc = point_shader->uniformLoc("color");
		point_pos_loc = point_shader->uniformLoc("position");

		edge_plane_shader = new Shader("Shaders/EdgePlaneVertex.shader", "Shaders/EdgePlaneFragment.shader");
		edge_plane_shader->use();
		edge_plane_view_loc = edge_plane_shader->uniformLoc("view");
		edge_plane_proj_loc = edge_plane_shader->uniformLoc("projection");
		edge_plane_pos_loc[0] = edge_plane_shader->uniformLoc("pos[0]");
		edge_plane_pos_loc[1] = edge_plane_shader->uniformLoc("pos[1]");
		edge_plane_pos_loc[2] = edge_plane_shader->uniformLoc("pos[2]");
		edge_plane_pos_loc[3] = edge_plane_shader->uniformLoc("pos[3]");
		edge_plane_opacity_loc[0] = edge_plane_shader->uniformLoc("opacity[0]");
		edge_plane_opacity_loc[1] = edge_plane_shader->uniformLoc("opacity[1]");
		edge_plane_opacity_loc[2] = edge_plane_shader->uniformLoc("opacity[2]");
		edge_plane_opacity_loc[3] = edge_plane_shader->uniformLoc("opacity[3]");
		edge_plane_color_loc = edge_plane_shader->uniformLoc("color");
		setEdgePlaneStartOpacity(0.5f);
		setEdgePlaneEndOpacity(0.0f);
	}

	~DebugRenderer()
	{
		delete(shape_shader);
		delete(color_shader);
		delete(line_shader);
		delete(line_segment_shader);
		delete(point_shader);
	}

	void randomizeColors()
	{
		dynamic_colors.clear();
		static_colors.clear();

		const float col_range = 0.8f;

		for (unsigned int i = 0; i < world->dynamic_bodies.size(); ++i)
		{
			glm::vec3 col = { random() * col_range, random() * col_range, random() * col_range };
			dynamic_colors.push_back(col + (1.0f - col_range));
		}
		for (unsigned int i = 0; i < world->static_bodies.size(); ++i)
		{
			glm::vec3 col = { random() * col_range, random() * col_range, random() * col_range };
			static_colors.push_back(col + (1.0f - col_range));
		}
	}

	fiz::Shape* loadPolyhedron(const std::string& filepath, float scale)
	{
		fiz::Shape* polyhedron_shape = models.loadPolyhedron(filepath, scale);
		polyhedron_shapes.push_back(polyhedron_shape);
		return polyhedron_shape;
	}

	void setSphereColor(glm::vec3 col)
	{
		point_shader->use();
		glUniform3f(point_color_loc, col.x, col.y, col.z);
	}

	void renderSphere(glm::vec3 pos, float rad)
	{
		point_shader->use();

		glUniformMatrix4fv(point_view_loc, 1, GL_FALSE, glm::value_ptr(camera.view));
		glUniformMatrix4fv(point_proj_loc, 1, GL_FALSE, glm::value_ptr(camera.projection));

		glBindVertexArray(models.sphereVAO);
		glUniform3f(point_scale_loc, rad, rad, rad);
		glUniform3f(point_pos_loc, pos.x, pos.y, pos.z);

		glDrawElements(GL_TRIANGLES, 6 * 20 * 20, GL_UNSIGNED_INT, 0);
	}

	void setLineSegmentColor(glm::vec3 col)
	{
		line_segment_shader->use();
		glUniform3f(line_segment_color_loc, col.x, col.y, col.z);
	}

	void renderLineSegment(glm::vec3 a, glm::vec3 b)
	{
		line_segment_shader->use();
		glUniformMatrix4fv(line_segment_view_loc, 1, GL_FALSE, glm::value_ptr(camera.view));
		glUniformMatrix4fv(line_segment_proj_loc, 1, GL_FALSE, glm::value_ptr(camera.projection));
		glUniform3f(line_segment_a_loc, a.x, a.y, a.z);
		glUniform3f(line_segment_b_loc, b.x, b.y, b.z);

		glDrawArrays(GL_LINES, 0, 2);
	}

	void setOutlineColor(glm::vec3 col)
	{
		line_shader->use();
		glUniform3f(line_color_loc, col.x, col.y, col.z);
	}

	void renderAABB(fiz::AABB& aabb)
	{
		glm::mat4 aabb_model = glm::mat4(1.0f);
		aabb_model = translate(aabb_model, (aabb.max + aabb.min) * 0.5f);
		line_shader->use();
		glUniformMatrix4fv(line_model_loc, 1, GL_FALSE, glm::value_ptr(aabb_model));
		glUniformMatrix4fv(line_view_loc, 1, GL_FALSE, glm::value_ptr(camera.view));
		glUniformMatrix4fv(line_proj_loc, 1, GL_FALSE, glm::value_ptr(camera.projection));

		glBindVertexArray(models.box_linesVAO);
		glm::vec3 aabb_scale = (aabb.max - aabb.min);
		line_shader->setVec3("scale", aabb_scale.x, aabb_scale.y, aabb_scale.z);
		glDrawArrays(GL_LINES, 0, 24);
	}

	const glm::vec3 simplex_colors[4] = {
		glm::vec3(1.0f, 0.0f, 0.0f),
		glm::vec3(0.0f, 1.0f, 0.0f),
		glm::vec3(0.0f, 0.0f, 1.0f),
		glm::vec3(1.0f, 1.0f, 0.0f)
	};

	void renderSimplex(fiz::Simplex s)
	{
		for (unsigned int i = 0; i < s.n; ++i)
		{
			setSphereColor(simplex_colors[i]);
			renderSphere(s.v[i], 0.04f);
		}

		setLineSegmentColor(glm::vec3(0.5f, 0.5f, 0.5f));

		switch (s.n)
		{
		case 4:
			renderLineSegment(s.v[0], s.v[3]);
			renderLineSegment(s.v[1], s.v[3]);
			renderLineSegment(s.v[2], s.v[3]);
		case 3:
			renderLineSegment(s.v[0], s.v[2]);
			renderLineSegment(s.v[1], s.v[2]);
		case 2:
			renderLineSegment(s.v[0], s.v[1]);
		}

		if (s.n >= 3)
		{
			setEdgePlaneColor(glm::vec3(1.0f, 0.0f, 1.0f));
			glm::vec3 norm = glm::cross(s.v[1] - s.v[2], s.v[0] - s.v[2]);
			norm = glm::normalize(norm);

			std::vector<fiz::Edge> edges;
			edges.reserve(3);
			edges.emplace_back(s.v[0], s.v[2]);
			edges.emplace_back(s.v[1], s.v[2]);
			edges.emplace_back(s.v[0], s.v[1]);
			TestCamera* cam = &camera;
			std::sort(edges.begin(), edges.end(), [cam](const fiz::Edge& a, const fiz::Edge& b) {
				glm::vec3 a_mid = (a.a + a.b) * 0.5f;
				glm::vec4 a_view4 = cam->view * glm::vec4(a_mid, 1.0f);
				glm::vec3 b_mid = (b.a + b.b) * 0.5f;
				glm::vec4 b_view4 = cam->view * glm::vec4(b_mid, 1.0f);

				return a_view4.z < b_view4.z;
			});

			for (unsigned int i = 0; i < edges.size(); ++i)
			{
				renderEdgePlane(edges[i].a, edges[i].b, norm, 3.0f);
			}
		}

		setLineSegmentColor(glm::vec3(1.0f, 0.0f, 0.0f));
		glm::vec3 avg = glm::vec3(0.0f);
		for (unsigned int i = 0; i < s.n; ++i)
			avg += s.v[i];
		if (s.n > 0)
			avg /= (float)s.n;
		renderLineSegment(avg, avg + glm::normalize(s.D) * 0.5f);
	}

	void renderPolytope(fiz::Polytope p)
	{

	}

	void renderMinkowskiDifference(std::vector<glm::vec3>& a, std::vector<glm::vec3>& b)
	{
		setSphereColor(glm::vec3(0.7f, 0.7f, 0.7f));
		for (unsigned int i = 0; i < a.size(); ++i)
		{
			for (unsigned int j = 0; j < b.size(); ++j)
			{
				renderSphere(a[i] - b[j], 0.02f);
			}
		}
	}

	void renderContactManifold(fiz::ContactManifold& manifold)
	{
		setSphereColor(glm::vec3(1.0f, 0.0f, 0.0f));
		for (unsigned int i = 0; i < manifold.num_contacts && i < 4; ++i)
		{
			renderSphere(manifold.local_a[i], 0.05f);
		}
	}

	void setEdgePlaneColor(glm::vec3 col)
	{
		edge_plane_shader->use();
		glUniform3f(edge_plane_color_loc, col.x, col.y, col.z);
	}

	void setEdgePlaneStartOpacity(float opacity)
	{
		edge_plane_shader->use();
		glUniform1f(edge_plane_opacity_loc[0], opacity);
		glUniform1f(edge_plane_opacity_loc[1], opacity);
	}

	void setEdgePlaneEndOpacity(float opacity)
	{
		edge_plane_shader->use();
		glUniform1f(edge_plane_opacity_loc[2], opacity);
		glUniform1f(edge_plane_opacity_loc[3], opacity);
	}

	void renderEdgePlane(glm::vec3 a, glm::vec3 b, glm::vec3 v, float len)
	{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDepthMask(GL_FALSE);

		edge_plane_shader->use();
		glUniformMatrix4fv(edge_plane_view_loc, 1, GL_FALSE, glm::value_ptr(camera.view));
		glUniformMatrix4fv(edge_plane_proj_loc, 1, GL_FALSE, glm::value_ptr(camera.projection));
		v = glm::normalize(v) * len;
		glm::vec3 c = a + v;
		glm::vec3 d = b + v;
		glUniform3f(edge_plane_pos_loc[0], a.x, a.y, a.z);
		glUniform3f(edge_plane_pos_loc[1], b.x, b.y, b.z);
		glUniform3f(edge_plane_pos_loc[2], c.x, c.y, c.z);
		glUniform3f(edge_plane_pos_loc[3], d.x, d.y, d.z);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

		glDisable(GL_BLEND);
		glDepthMask(GL_TRUE);
	}

	void renderCapsule(glm::mat4& model, float rad, float height)
	{
		glm::mat4 top_model = glm::mat4(1.0f);
		top_model[3].z = height;
		top_model = model * top_model;

		glm::mat4 bottom_model = glm::mat4(0.0f);
		bottom_model[0] = glm::vec4(0.0f, 1.0f, 0.0f, 0.0f);
		bottom_model[1] = glm::vec4(-1.0f, 0.0f, 0.0f, 0.0f);
		bottom_model[2] = glm::vec4(0.0f, 0.0f, -1.0f, 0.0f);
		bottom_model[3] = glm::vec4(0.0f, 0.0f, -height, 1.0f);
		bottom_model = model * bottom_model;


		glBindVertexArray(models.capsule_cylinderVAO);
		setShaderScale(rad, rad, height);
		const unsigned int sides = 40;
		unsigned int index_size = 2 * sides;
		glDrawElements(GL_TRIANGLES, index_size * 3, GL_UNSIGNED_INT, 0);

		if (render_mode == RenderMode::RENDER_SHAPES)
			glUniformMatrix4fv(shape_model_loc, 1, GL_FALSE, glm::value_ptr(top_model));
		else
			glUniformMatrix4fv(color_model_loc, 1, GL_FALSE, glm::value_ptr(top_model));

		glBindVertexArray(models.capsule_hemisphereVAO);
		setShaderScale(rad, rad, rad);
		index_size = 2 * sides * sides;
		glDrawElements(GL_TRIANGLES, index_size * 3, GL_UNSIGNED_INT, 0);

		if (render_mode == RenderMode::RENDER_SHAPES)
			glUniformMatrix4fv(shape_model_loc, 1, GL_FALSE, glm::value_ptr(bottom_model));
		else
			glUniformMatrix4fv(color_model_loc, 1, GL_FALSE, glm::value_ptr(bottom_model));
		glDrawElements(GL_TRIANGLES, index_size * 3, GL_UNSIGNED_INT, 0);
	}

	void renderPolyhedron(glm::mat4& model, unsigned int index)
	{
		glBindVertexArray(models.polyhedronVAO[index]);

		setShaderScale(1.0f, 1.0f, 1.0f);

		glDrawArrays(GL_TRIANGLES, 0, models.polyhedron_vertex_count[index]);
	}

	void renderBody(fiz::Body& body, unsigned int index)
	{
		glm::mat4 model(1.0f);
		// ADD - rotate with body orientation
		model = glm::translate(model, body.pos);
		model = model * glm::mat4_cast(body.orientation);

		bool is_asleep = false;
		if (body.type == fiz::BodyType::DYNAMIC)
			is_asleep = !((fiz::DynamicBody*)&body)->is_awake;

		if (render_mode == RenderMode::RENDER_SHAPES)
		{
			shape_shader->use();

			glUniformMatrix4fv(shape_model_loc, 1, GL_FALSE, glm::value_ptr(model));
			glUniformMatrix4fv(shape_view_loc, 1, GL_FALSE, glm::value_ptr(camera.view));
			glUniformMatrix4fv(shape_proj_loc, 1, GL_FALSE, glm::value_ptr(camera.projection));
		}
		else
		{
			color_shader->use();

			glUniformMatrix4fv(color_model_loc, 1, GL_FALSE, glm::value_ptr(model));
			glUniformMatrix4fv(color_view_loc, 1, GL_FALSE, glm::value_ptr(camera.view));
			glUniformMatrix4fv(color_proj_loc, 1, GL_FALSE, glm::value_ptr(camera.projection));

			if (body.type == fiz::BodyType::DYNAMIC)
				glUniform3f(color_color_loc, dynamic_colors[index].x, dynamic_colors[index].y, dynamic_colors[index].z);
			else
				glUniform3f(color_color_loc, static_colors[index].x, static_colors[index].y, static_colors[index].z);
		}

		for (unsigned int j = 0; j < body.shapes.size(); ++j)
		{
			fiz::Shape* shape = body.shapes[j];

			switch (shape->shape_type)
			{
			case fiz::SPHERE_TYPE:
			{
				fiz::Sphere* sphere = (fiz::Sphere*)shape;

				if (outline_shapes)
				{
					line_shader->use();
					glUniformMatrix4fv(line_model_loc, 1, GL_FALSE, glm::value_ptr(model));
					glUniformMatrix4fv(line_view_loc, 1, GL_FALSE, glm::value_ptr(camera.view));
					glUniformMatrix4fv(line_proj_loc, 1, GL_FALSE, glm::value_ptr(camera.projection));

					glBindVertexArray(models.sphere_linesVAO);
					line_shader->setVec3("scale", sphere->rad, sphere->rad, sphere->rad);
					glUniform3f(line_color_loc, 0.0f, 0.6823f, 1.0f);
					glDrawElements(GL_LINES, 12 * 20, GL_UNSIGNED_INT, 0);
				}
				else
				{
					if (is_asleep && show_sleep)
						glBindTexture(GL_TEXTURE_2D, models.sleep_texture);
					else
						glBindTexture(GL_TEXTURE_2D, models.sphere_texture);
					glBindVertexArray(models.sphereVAO);

					setShaderScale(sphere->rad, sphere->rad, sphere->rad);

					glDrawElements(GL_TRIANGLES, 6 * 20 * 20, GL_UNSIGNED_INT, 0);
				}

				
				break;
			}
			case fiz::BOX_TYPE:
			{
				fiz::Box* box = (fiz::Box*)shape;
				glm::vec3 center = box->pos;
				glm::vec3 scale = box->dim * 2.0f;

				if (outline_shapes)
				{
					line_shader->use();
					glUniformMatrix4fv(line_model_loc, 1, GL_FALSE, glm::value_ptr(model));
					glUniformMatrix4fv(line_view_loc, 1, GL_FALSE, glm::value_ptr(camera.view));
					glUniformMatrix4fv(line_proj_loc, 1, GL_FALSE, glm::value_ptr(camera.projection));

					glBindVertexArray(models.box_linesVAO);
					line_shader->setVec3("scale", scale.x, scale.y, scale.z);
					glUniform3f(line_color_loc, 0.302f, 1.0f, 0.251f);
					glDrawArrays(GL_LINES, 0, 24);
				}
				else
				{
					if (is_asleep && show_sleep)
						glBindTexture(GL_TEXTURE_2D, models.sleep_texture);
					else
						glBindTexture(GL_TEXTURE_2D, models.box_texture);
					glBindVertexArray(models.boxVAO);

					setShaderScale(scale.x, scale.y, scale.z);

					glDrawArrays(GL_TRIANGLES, 0, 36);
				}
				break;
			}
			case fiz::CYLINDER_TYPE:
			{
				if (is_asleep && show_sleep)
					glBindTexture(GL_TEXTURE_2D, models.sleep_texture);
				else
					glBindTexture(GL_TEXTURE_2D, models.cylinder_texture);
				glBindVertexArray(models.cylinderVAO);

				fiz::Cylinder* cylinder = (fiz::Cylinder*)shape;
				setShaderScale(cylinder->rad, cylinder->rad, cylinder->height);

				const unsigned int sides = 40;
				const unsigned int index_size = 2 * sides + 2 * (sides - 2);
				glDrawElements(GL_TRIANGLES, index_size * 3, GL_UNSIGNED_INT, 0);

				/*line_shader->use();
				glUniformMatrix4fv(line_model_loc, 1, GL_FALSE, glm::value_ptr(model));
				glUniformMatrix4fv(line_view_loc, 1, GL_FALSE, glm::value_ptr(camera.view));
				glUniformMatrix4fv(line_proj_loc, 1, GL_FALSE, glm::value_ptr(camera.projection));

				glBindVertexArray(models.sphere_linesVAO);
				line_shader->setVec3("scale", sphere->rad, sphere->rad, sphere->rad);
				glUniform3f(line_color_loc, 0.0f, 0.6823f, 1.0f);
				glDrawElements(GL_LINES, 12 * 20, GL_UNSIGNED_INT, 0);*/
				break;
			}
			case fiz::CAPSULE_TYPE:
			{
				if (is_asleep && show_sleep)
					glBindTexture(GL_TEXTURE_2D, models.sleep_texture);
				else
					glBindTexture(GL_TEXTURE_2D, models.capsule_texture);

				fiz::Capsule* capsule = (fiz::Capsule*)shape;
				
				renderCapsule(model, capsule->rad, capsule->height);

				/*line_shader->use();
				glUniformMatrix4fv(line_model_loc, 1, GL_FALSE, glm::value_ptr(model));
				glUniformMatrix4fv(line_view_loc, 1, GL_FALSE, glm::value_ptr(camera.view));
				glUniformMatrix4fv(line_proj_loc, 1, GL_FALSE, glm::value_ptr(camera.projection));

				glBindVertexArray(models.sphere_linesVAO);
				line_shader->setVec3("scale", sphere->rad, sphere->rad, sphere->rad);
				glUniform3f(line_color_loc, 0.0f, 0.6823f, 1.0f);
				glDrawElements(GL_LINES, 12 * 20, GL_UNSIGNED_INT, 0);*/
				break;
			}
			case fiz::POLYHEDRON_TYPE:
			{
				if (is_asleep && show_sleep)
					glBindTexture(GL_TEXTURE_2D, models.sleep_texture);
				else
					glBindTexture(GL_TEXTURE_2D, models.polyhedron_texture);

				for (unsigned int x = 0; x < polyhedron_shapes.size(); ++x)
				{
					if (body.shapes[0] == polyhedron_shapes[x])
					{
						renderPolyhedron(model, x);
						break;
					}
				}
			}
			}
		}

		if (show_aabb)
		{
			glm::mat4 aabb_model = glm::mat4(1.0f);
			aabb_model = translate(aabb_model, (body.aabb.max + body.aabb.min) * 0.5f);
			line_shader->use();
			glUniformMatrix4fv(line_model_loc, 1, GL_FALSE, glm::value_ptr(aabb_model));
			glUniformMatrix4fv(line_view_loc, 1, GL_FALSE, glm::value_ptr(camera.view));
			glUniformMatrix4fv(line_proj_loc, 1, GL_FALSE, glm::value_ptr(camera.projection));

			glBindVertexArray(models.box_linesVAO);
			glm::vec3 aabb_scale = (body.aabb.max - body.aabb.min);
			line_shader->setVec3("scale", aabb_scale.x, aabb_scale.y, aabb_scale.z);
			glUniform3f(line_color_loc, 0.5f, 0.5f, 0.5f);
			glDrawArrays(GL_LINES, 0, 24);
		}

		if (show_velocities && body.type == fiz::BodyType::DYNAMIC)
		{
			// linear velocity
			setLineSegmentColor(glm::vec3(1.0f, 0.0f, 0.0f));
			fiz::DynamicBody* d_body = (fiz::DynamicBody*)&body;
			renderLineSegment(body.pos, body.pos + d_body->vel);

			// angular velocity
			setLineSegmentColor(glm::vec3(0.0f, 0.0f, 1.0f));
			renderLineSegment(body.pos, body.pos + d_body->angular_vel);
		}
	}

	void renderBVHNode(fiz::LinearBVHNode* node)
	{
		switch (node->axis)
		{
		case 0:
			setOutlineColor(glm::vec3(1.0f, 0.0f, 0.0f));
			break;
		case 1:
			setOutlineColor(glm::vec3(0.0f, 1.0f, 0.0f));
			break;
		case 2:
			setOutlineColor(glm::vec3(0.0f, 0.0f, 1.0f));
		}
		renderAABB(node->aabb);
	}

	void render()
	{
		// draw all shapes to shadow map
		// draw all shapes to the scene

		if (render_mode == RenderMode::RENDER_SHAPES)
		{
			shape_shader->use();
			shape_shader->setVec3("viewPos", camera.m_pos);
		}
		else
		{
			color_shader->use();
			color_shader->setVec3("viewPos", camera.m_pos);
		}
		

		camera.updateView();
		camera.updateProjection(900.0f / 600.0f);

		for (unsigned int i = 0; i < world->dynamic_bodies.size(); ++i)
		{
			fiz::Body& body = world->dynamic_bodies[i];
			renderBody(body, i);
		}

		for (unsigned int i = 0; i < world->static_bodies.size(); ++i)
		{
			fiz::Body& body = world->static_bodies[i];
			renderBody(body, i);
		}

		for (unsigned int i = 0; i < world->contacts.size(); ++i)
		{
			fiz::ContactInfo& contact = world->contacts[i];
			setSphereColor(glm::vec3(1.0f, 0.0f, 0.0f));
			renderSphere(contact.poc, 0.05f);

			setSphereColor(glm::vec3(1.0f, 1.0f, 0.0f));
			renderSphere(contact.poc_a, 0.05f);

			setSphereColor(glm::vec3(0.0f, 0.0f, 1.0f));
			renderSphere(contact.poc_b, 0.05f);

			setLineSegmentColor(glm::vec3(0.0f, 1.0f, 1.0f));
			renderLineSegment(glm::vec3(), contact.normal * contact.depth);
			/*glm::mat4 model = glm::mat4(1.0f);
			model = glm::translate(model, contact.poc);

			shader->use();
			glUniformMatrix4fv(model_loc, 1, GL_FALSE, glm::value_ptr(model));
			glUniformMatrix4fv(view_loc, 1, GL_FALSE, glm::value_ptr(camera.view));
			glUniformMatrix4fv(proj_loc, 1, GL_FALSE, glm::value_ptr(camera.projection));

			glBindTexture(GL_TEXTURE_2D, models.sphere_texture);
			glBindVertexArray(models.sphereVAO);

			shader->setVec3("scale", 0.05f, 0.05f, 0.05f);

			glDrawElements(GL_TRIANGLES, 6 * 20 * 20, GL_UNSIGNED_INT, 0);

			line_shader->use();
			glUniformMatrix4fv(line_model_loc, 1, GL_FALSE, glm::value_ptr(model));
			glUniformMatrix4fv(line_view_loc, 1, GL_FALSE, glm::value_ptr(camera.view));
			glUniformMatrix4fv(line_proj_loc, 1, GL_FALSE, glm::value_ptr(camera.projection));

			glBindVertexArray(models.sphere_linesVAO);
			line_shader->setVec3("scale", 0.05f, 0.05f, 0.05f);
			glUniform3f(line_color_loc, 1.0f, 0.3f, 0.0f);
			glDrawElements(GL_LINES, 12 * 20, GL_UNSIGNED_INT, 0);*/
		}

		if (show_BVH)
		{
			for (unsigned int i = 0; i < world->static_bvh.nodes.size(); ++i)
			{
				fiz::LinearBVHNode* node = &world->static_bvh.nodes[i];
				renderBVHNode(node);
			}
		}

		// reset uniforms
		glm::mat4 model(1.0f);
		if (render_mode == RenderMode::RENDER_SHAPES)
		{
			shape_shader->use();

			glUniformMatrix4fv(shape_model_loc, 1, GL_FALSE, glm::value_ptr(model));
			shape_shader->setVec3("scale", 1.0f, 1.0f, 1.0f);
			glBindTexture(GL_TEXTURE_2D, models.box_texture);
		}
		else
		{
			color_shader->use();

			glUniformMatrix4fv(color_model_loc, 1, GL_FALSE, glm::value_ptr(model));
			color_shader->setVec3("scale", 1.0f, 1.0f, 1.0f);
			glBindTexture(GL_TEXTURE_2D, models.box_texture);
		}
		
	}

	void processInput(GLFWwindow* window, float delta_time)
	{
		camera.processInput(window, delta_time);
	}

private:
	Shader* shape_shader;
	Shader* color_shader;

	Shader* line_shader;
	Shader* shadow_shader;
	Shader* line_segment_shader;
	Shader* point_shader;
	Shader* edge_plane_shader;

	glm::vec3 light_direction; // directional light

	unsigned int shape_model_loc;
	unsigned int shape_view_loc;
	unsigned int shape_proj_loc;
	unsigned int shape_scale_loc;

	unsigned int color_model_loc;
	unsigned int color_view_loc;
	unsigned int color_proj_loc;
	unsigned int color_scale_loc;
	unsigned int color_color_loc;

	unsigned int line_model_loc;
	unsigned int line_view_loc;
	unsigned int line_proj_loc;
	unsigned int line_scale_loc;
	unsigned int line_color_loc;

	unsigned int line_segment_view_loc;
	unsigned int line_segment_proj_loc;
	unsigned int line_segment_color_loc;
	unsigned int line_segment_a_loc;
	unsigned int line_segment_b_loc;

	unsigned int point_view_loc;
	unsigned int point_proj_loc;
	unsigned int point_scale_loc;
	unsigned int point_color_loc;
	unsigned int point_pos_loc;

	unsigned int edge_plane_view_loc;
	unsigned int edge_plane_proj_loc;
	unsigned int edge_plane_pos_loc[4];
	unsigned int edge_plane_opacity_loc[4];
	unsigned int edge_plane_color_loc;

	inline float random()
	{
		return (float)(rand() % 1000) / 1000.0f;
	}
	inline float random(float min, float max)
	{
		return min + random() * (max - min);
	}

	inline void setShaderScale(float x, float y, float z)
	{
		if (render_mode == RenderMode::RENDER_SHAPES)
			shape_shader->setVec3("scale", x, y, z);
		else
			color_shader->setVec3("scale", x, y, z);
	}
};