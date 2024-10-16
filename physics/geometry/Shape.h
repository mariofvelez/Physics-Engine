#pragma once

#include <vector>

#include <glm/glm.hpp>

namespace fiz
{
	struct Ray
	{
		glm::vec3 start;
		glm::vec3 dir;
	};

	class AABB
	{
	public:
		glm::vec3 min;
		glm::vec3 max;

		AABB() : min(0.0f), max(0.0f)
		{

		}
		AABB(glm::vec3 min, glm::vec3 max) : min(min), max(max)
		{

		}

		void set(AABB& other)
		{
			min = other.min;
			max = other.max;
		}
		void combine(AABB& other)
		{
			min.x = glm::min(min.x, other.min.x);
			min.y = glm::min(min.y, other.min.y);
			min.z = glm::min(min.z, other.min.z);

			max.x = glm::max(max.x, other.max.x);
			max.y = glm::max(max.y, other.max.y);
			max.z = glm::max(max.z, other.max.z);
		}
		void combine(glm::vec3 vec)
		{
			min.x = glm::min(min.x, vec.x);
			min.y = glm::min(min.y, vec.y);
			min.z = glm::min(min.z, vec.z);

			max.x = glm::max(max.x, vec.x);
			max.y = glm::max(max.y, vec.y);
			max.z = glm::max(max.z, vec.z);
		}
		bool intersects(AABB& other) const
		{
			bool x = max.x > other.min.x && min.x < other.max.x;
			bool y = max.y > other.min.y && min.y < other.max.y;
			bool z = max.z > other.min.z && min.z < other.max.z;
			return x && y && z;
		}
		int maxExtent() const
		{
			glm::vec3 extent = max - min;
			if (extent.x > extent.y)
			{
				if (extent.x > extent.z)
				{
					return 0;
				}
				else
				{
					return 2;
				}
			}
			else
			{
				if (extent.y > extent.z)
				{
					return 1;
				}
				else
				{
					return 2;
				}
			}
		}
		bool intersects(const Ray* ray, glm::vec3& inv_dir, int* is_neg) const
		{
			float tx1 = (min.x - ray->start.x) * inv_dir.x;
			float tx2 = (max.x - ray->start.x) * inv_dir.x;

			float tmin = glm::min(tx1, tx2);
			float tmax = glm::max(tx1, tx2);

			float ty1 = (min.y - ray->start.y) * inv_dir.y;
			float ty2 = (max.y - ray->start.y) * inv_dir.y;

			tmin = glm::max(tmin, glm::min(ty1, ty2));
			tmax = glm::min(tmax, glm::max(ty1, ty2));

			float tz1 = (min.z - ray->start.z) * inv_dir.z;
			float tz2 = (max.z - ray->start.z) * inv_dir.z;

			tmin = glm::max(tmin, glm::min(tz1, tz2));
			tmax = glm::min(tmax, glm::max(tz1, tz2));

			return tmax > tmin;
		}
	};

	enum ShapeType
	{
		SPHERE_TYPE,
		CYLINDER_TYPE,
		BOX_TYPE,
		CAPSULE_TYPE,
		POLYHEDRON_TYPE
	};

	class Shape
	{
	public:
		ShapeType shape_type;

		float volume;
		glm::vec3 centroid;
		glm::vec3 local_inertia; // principal axes of inertia
		glm::vec3 local_products; // products of inertia (xy, yz, xz)

		virtual bool intersects(glm::vec3 point) { return false; }

		virtual glm::vec3 support(glm::vec3 axis) { return glm::vec3(0.0f); }

		virtual void setAABB(AABB* aabb, glm::vec3& position, glm::mat3& orientation) {}

		virtual void computeMassProperties() {}

		virtual float castRay(Ray& ray) { return -1.0f; }
	};

	class Sphere final : public Shape
	{
	public:
		glm::vec3 pos;
		float rad;

		Sphere(glm::vec3 pos, float rad) : pos(pos), rad(rad), rad2(rad * rad)
		{
			shape_type = SPHERE_TYPE;
		}

		bool intersects(glm::vec3 point)
		{
			return glm::distance(point, pos) <= rad;
		}

		glm::vec3 support(glm::vec3 axis)
		{
			return pos + glm::normalize(axis) * rad;
		}

		void setAABB(AABB* aabb, glm::vec3& position, glm::mat3& orientation)
		{
			aabb->min = position - glm::vec3(rad);
			aabb->max = position + glm::vec3(rad);
		}

		void computeMassProperties()
		{
			volume = (4.0f / 3.0f) * glm::pi<float>() * rad2 * rad;
			centroid = pos;
			float inertia = 0.4f * volume * rad2;
			local_inertia.x = inertia;
			local_inertia.y = inertia;
			local_inertia.z = inertia;
			local_products.x = 0.0f;
			local_products.y = 0.0f;
			local_products.z = 0.0f;
		}

		float castRay(Ray& ray)
		{
			float a = glm::dot(ray.dir, ray.dir);
			float b = 2.0f * glm::dot(ray.start - pos, ray.dir);
			float c = glm::dot(ray.start - pos, ray.start - pos) - rad2;
			float desc = b * b - 4.0f * a * c;
			if (desc < 0.0f)
				return 0.0f;
			float t = (-b - sqrtf(desc)) / (2.0f * a);

			if (t > 0.000001f)
				return t;
			return 0.0f;
		}
	private:
		float rad2;
	};

	struct Edge
	{
		glm::vec3 a;
		glm::vec3 b;

		Edge(glm::vec3 a, glm::vec3 b) : a(a), b(b) {}
	};

	class Box final : public Shape
	{
	public:
		glm::vec3 pos;
		glm::vec3 dim; // dimensions from center to face (half width, height, depth)

		Box(glm::vec3 pos, glm::vec3 dim) : pos(pos), dim(dim)
		{
			shape_type = BOX_TYPE;
		}

		bool intersects(glm::vec3 point)
		{
			return point.x >= pos.x - dim.x && point.x <= pos.x + dim.x &&
				   point.y >= pos.y - dim.y && point.y <= pos.y + dim.y &&
				   point.z >= pos.z - dim.z && point.z <= pos.z + dim.z;
		}

		glm::vec3 support(glm::vec3 axis)
		{
			float x = axis.x < 0.0f ? pos.x - dim.x : pos.x + dim.x;
			float y = axis.y < 0.0f ? pos.y - dim.y : pos.y + dim.y;
			float z = axis.z < 0.0f ? pos.z - dim.z : pos.z + dim.z;
			return glm::vec3(x, y, z);
		}

		void setAABB(AABB* aabb, glm::vec3& position, glm::mat3& orientation)
		{
			float extent = glm::length(dim);
			aabb->min = position - glm::vec3(extent);
			aabb->max = position + glm::vec3(extent);
		}

		void computeMassProperties()
		{
			volume = dim.x * dim.y * dim.z * 8.0f;
			centroid = pos;
			float dx2 = (dim.x * 2.0f) + (dim.x * 2.0f);
			float dy2 = (dim.y * 2.0f) + (dim.y * 2.0f);
			float dz2 = (dim.z * 2.0f) + (dim.z * 2.0f);
			local_inertia.x = (1 / 12.0f) * volume * (dy2 + dz2);
			local_inertia.y = (1 / 12.0f) * volume * (dx2 + dz2);
			local_inertia.z = (1 / 12.0f) * volume * (dx2 + dy2);
			local_products.x = 0.0f;
			local_products.y = 0.0f;
			local_products.z = 0.0f;
		}

		float castRay(Ray& ray)
		{
			AABB aabb = AABB(pos - dim, pos + dim);
			float tx1 = (aabb.min.x - ray.start.x) / ray.dir.x;
			float tx2 = (aabb.max.x - ray.start.x) / ray.dir.x;

			float tmin = glm::min(tx1, tx2);
			float tmax = glm::max(tx1, tx2);

			float ty1 = (aabb.min.y - ray.start.y) / ray.dir.y;
			float ty2 = (aabb.max.y - ray.start.y) / ray.dir.y;

			tmin = glm::max(tmin, glm::min(ty1, ty2));
			tmax = glm::min(tmax, glm::max(ty1, ty2));

			float tz1 = (aabb.min.z - ray.start.z) / ray.dir.z;
			float tz2 = (aabb.max.z - ray.start.z) / ray.dir.z;

			tmin = glm::max(tmin, glm::min(tz1, tz2));
			tmax = glm::min(tmax, glm::max(tz1, tz2));

			return tmax > tmin && tmin > 0? tmin : 0.0f;
		}

		void projectVertices(glm::mat3& orientation, glm::vec3 pos, std::vector<glm::vec3>& vertices)
		{
			vertices.push_back(orientation * glm::vec3( dim.x,  dim.y,  dim.z) + pos); // 0
			vertices.push_back(orientation * glm::vec3( dim.x,  dim.y, -dim.z) + pos); // 1
			vertices.push_back(orientation * glm::vec3( dim.x, -dim.y,  dim.z) + pos); // 2
			vertices.push_back(orientation * glm::vec3( dim.x, -dim.y, -dim.z) + pos); // 3
			vertices.push_back(orientation * glm::vec3(-dim.x,  dim.y,  dim.z) + pos); // 4
			vertices.push_back(orientation * glm::vec3(-dim.x,  dim.y, -dim.z) + pos); // 5
			vertices.push_back(orientation * glm::vec3(-dim.x, -dim.y,  dim.z) + pos); // 6
			vertices.push_back(orientation * glm::vec3(-dim.x, -dim.y, -dim.z) + pos); // 7
		}

		void projectEdges(std::vector<Edge>& edges, std::vector<glm::vec3>& vertices)
		{
			edges.reserve(12);
			// along x axis
			edges.emplace_back(vertices[5], vertices[1]);
			edges.emplace_back(vertices[7], vertices[3]);
			edges.emplace_back(vertices[4], vertices[0]);
			edges.emplace_back(vertices[6], vertices[2]);

			// along y axis
			edges.emplace_back(vertices[7], vertices[5]);
			edges.emplace_back(vertices[3], vertices[1]);
			edges.emplace_back(vertices[6], vertices[4]);
			edges.emplace_back(vertices[2], vertices[0]);

			// along z axis
			edges.emplace_back(vertices[5], vertices[4]);
			edges.emplace_back(vertices[1], vertices[0]);
			edges.emplace_back(vertices[7], vertices[6]);
			edges.emplace_back(vertices[3], vertices[2]);
		}
	};

	class Cylinder final : public Shape
	{
	public:
		glm::vec3 pos;
		float rad;
		float height; // half height

		Cylinder(glm::vec3 pos, float rad, float height) : pos(pos), rad(rad), height(height)
		{
			shape_type = CYLINDER_TYPE;
		}

		bool intersects(glm::vec3 point)
		{
			glm::vec3 rel = point - pos;
			float planar = glm::sqrt(rel.x * rel.x + rel.y * rel.y);
			return planar < rad && rel.z < height && rel.z > -height;
		}

		glm::vec3 support(glm::vec3 axis)
		{
			glm::vec3 planar = glm::normalize(glm::vec3(axis.x, axis.y, 0.0f)) * rad;
			if (planar.x != planar.x)
			{
				planar.x = 0.0f;
				planar.y = 0.0f;
			}
			planar.z = axis.z > 0 ? height : -height;
			return planar;
		}

		void setAABB(AABB* aabb, glm::vec3& position, glm::mat3& orientation)
		{
			float extent = glm::sqrt(rad * rad + height * height);
			aabb->min = position - glm::vec3(extent);
			aabb->max = position + glm::vec3(extent);
		}

		void computeMassProperties()
		{
			float rad2 = rad * rad;
			float h = height * 2.0f;
			volume = glm::pi<float>() * rad2 * h;
			centroid = pos;
			local_inertia.x = (1.0f / 12.0f) * volume * (3.0f * rad2 + h * h);
			local_inertia.y = local_inertia.x;
			local_inertia.z = 0.5f * volume * rad2;
			local_products.x = 0.0f;
			local_products.y = 0.0f;
			local_products.z = 0.0f;
		}
	};

	class Capsule final : public Shape
	{
	public:
		glm::vec3 pos;
		float rad;
		float height; // half height

		Capsule(glm::vec3 pos, float rad, float height) : pos(pos), rad(rad), height(height)
		{
			shape_type = CAPSULE_TYPE;
		}

		bool intersects(glm::vec3 point)
		{
			float planar = glm::sqrt(point.x * point.x + point.y * point.y);
			glm::vec3 top = pos;
			top.z += height;
			glm::vec3 bottom = pos;
			bottom.z -= height;
			if (planar < rad && glm::distance(point, top) < rad && glm::distance(point, bottom) < rad)
				return true;
			return false;
		}

		glm::vec3 support(glm::vec3 axis)
		{
			glm::vec3 on_sphere = pos + glm::normalize(axis) * rad;
			on_sphere.z += axis.z > 0 ? height : -height;
			return on_sphere;
		}

		void setAABB(AABB* aabb, glm::vec3& position, glm::mat3& orientation)
		{
			aabb->min = position + orientation[2] * height - glm::vec3(rad);
			aabb->max = position + orientation[2] * height + glm::vec3(rad);

			glm::vec3 bottom_min = position - orientation[2] * height - glm::vec3(rad);
			glm::vec3 bottom_max = position - orientation[2] * height + glm::vec3(rad);
			AABB bottom = AABB(bottom_min, bottom_max);

			aabb->combine(bottom);
		}

		void computeMassProperties()
		{
			float rad2 = rad * rad;
			float h = height * 2.0f;
			float cylinder_volume = glm::pi<float>() * rad2 * h;
			float hemisphere_volume = (2.0f / 3.0f) * glm::pi<float>() * rad2 * rad;
			volume = cylinder_volume + 2.0f * hemisphere_volume;
			centroid = pos;
			local_inertia.x = (1.0f / 12.0f) * cylinder_volume * (3.0f * rad2 + h * h);
			local_inertia.x += 2.0f * hemisphere_volume * (0.4f * rad2 + 0.5f * h * h + 0.375f * h * rad);
			local_inertia.y = local_inertia.x;
			local_inertia.z = 0.5f * cylinder_volume * rad2;
			local_inertia.z += 2.0f * hemisphere_volume * (0.4f * rad2);
			local_products.x = 0.0f;
			local_products.y = 0.0f;
			local_products.z = 0.0f;
		}
	};

	class Polyhedron final : Shape
	{
	public:
		std::vector<glm::vec3> vertices;
		std::vector<glm::uvec3> indices;

		Polyhedron(int vertex_count)
		{
			shape_type = POLYHEDRON_TYPE;
			vertices.reserve(vertex_count);
		}

		void addVertex(glm::vec3 vec)
		{
			vertices.push_back(vec);
		}

		void addIndex(glm::uvec3 vec)
		{
			indices.push_back(vec);
		}

		bool intersects(glm::vec3 point)
		{
			return false;
		}

		glm::vec3 support(glm::vec3 axis)
		{
			float max_dot = glm::dot(vertices[0], axis);
			glm::vec3 s = vertices[0];
			for (unsigned int i = 1; i < vertices.size(); ++i)
			{
				float dot = glm::dot(vertices[i], axis);
				if (dot > max_dot)
				{
					max_dot = dot;
					s = vertices[i];
				}
			}
			return s;
		}

		void setAABB(AABB* aabb, glm::vec3& position, glm::mat3& orientation)
		{
			glm::vec3 min = orientation * vertices[0];
			glm::vec3 max = orientation * vertices[0];
			for (unsigned int i = 1; i < vertices.size(); ++i)
			{
				glm::vec3 projected = orientation * vertices[i];
				min.x = glm::min(min.x, projected.x);
				min.y = glm::min(min.y, projected.y);
				min.z = glm::min(min.z, projected.z);

				max.x = glm::max(max.x, projected.x);
				max.y = glm::max(max.y, projected.y);
				max.z = glm::max(max.z, projected.z);
			}
			aabb->min = min + position;
			aabb->max = max + position;
		}

		void computeMassProperties()
		{
			const float mult[10] = { 1.0f / 6.0f, 1.0f / 24.0f, 1.0f / 24.0f, 1.0f / 24.0f, 1.0f / 60.0f, 1.0f / 60.0f, 1.0f / 60.0f, 1.0f / 120.0f, 1.0f / 120.0f, 1.0f / 120.0f };

			float intg[10] = { 0.0f };

			for (unsigned int i = 0; i < indices.size(); ++i)
			{
				glm::vec3 a = vertices[indices[i].x];
				glm::vec3 b = vertices[indices[i].y];
				glm::vec3 c = vertices[indices[i].z];

				glm::vec3 ab = b - a;
				glm::vec3 ac = c - a;
				
				glm::vec3 d = glm::cross(ab, ac);

				float f1x, f2x, f3x;
				float f1y, f2y, f3y;
				float f1z, f2z, f3z;
				float g0x, g1x, g2x;
				float g0y, g1y, g2y;
				float g0z, g1z, g2z;
				subexpr(a.x, b.x, c.x, f1x, f2x, f3x, g0x, g1x, g2x);
				subexpr(a.y, b.y, c.y, f1y, f2y, f3y, g0y, g1y, g2y);
				subexpr(a.z, b.z, c.z, f1z, f2z, f3z, g0z, g1z, g2z);

				intg[0] += d.x * f1x;
				intg[1] += d.x * f2x;
				intg[2] += d.y * f2y;
				intg[3] += d.z * f2z;
				intg[4] += d.x * f3x;
				intg[5] += d.y * f3y;
				intg[6] += d.z * f3z;
				intg[7] += d.x * (a.y * g0x + b.y * g1x + c.y * g2x);
				intg[8] += d.y * (a.z * g0y + b.z * g1y + c.z * g2y);
				intg[9] += d.z * (a.x * g0z + b.x * g1z + c.x * g2z);
			}
			for (unsigned int i = 0; i < 10; ++i)
				intg[i] *= mult[i];

			volume = intg[0];

			centroid.x = intg[1] / volume;
			centroid.y = intg[2] / volume;
			centroid.z = intg[3] / volume;

			local_inertia.x = intg[5] + intg[6] - volume * (centroid.y * centroid.y + centroid.z * centroid.z);
			local_inertia.y = intg[4] + intg[6] - volume * (centroid.x * centroid.x + centroid.z * centroid.z);
			local_inertia.z = intg[4] + intg[5] - volume * (centroid.x * centroid.x + centroid.y * centroid.y);
			local_products.x = -(intg[7] - volume * centroid.x * centroid.y);
			local_products.y = -(intg[8] - volume * centroid.y * centroid.z);
			local_products.z = -(intg[9] - volume * centroid.x * centroid.z);

			// temp moving centroid to origin
			for (unsigned int i = 0; i < vertices.size(); ++i)
				vertices[i] -= centroid;
		}

		float castRay(Ray& ray)
		{
			float closest_hit = 9999999.9f;
			for (unsigned int i = 0; i < indices.size(); ++i)
			{
				const glm::vec3& a = vertices[indices[i].x];
				const glm::vec3& b = vertices[indices[i].y];
				const glm::vec3& c = vertices[indices[i].z];

				glm::vec3 e1 = b - a;
				glm::vec3 e2 = c - a;

				glm::vec3 ray_cross_e2 = glm::cross(ray.dir, e2);

				float det = glm::dot(e1, ray_cross_e2);

				if (det > -0.000001f && det < 0.0000001f)
					continue;

				float inv_det = 1.0f / det;
				glm::vec3 s = ray.start - a;

				float u = inv_det * glm::dot(s, ray_cross_e2);

				if (u < 0 || u > 1)
					continue;

				glm::vec3 s_cross_e1 = glm::cross(s, e1);

				float v = inv_det * glm::dot(ray.dir, s_cross_e1);

				if (v < 0 || u + v > 1.0f)
					continue;

				float t = inv_det * glm::dot(e2, s_cross_e1);

				if (t > 0.000001f)
					closest_hit = t;
			}
			return closest_hit < 9999999.9f ? closest_hit : 0.0f;
		}

	private:
		inline void subexpr(float w0, float w1, float w2, float& f1, float& f2, float& f3, float& g0, float& g1, float& g2)
		{
			float temp0 = w0 + w1;
			f1 = temp0 + w2;
			float temp1 = w0 * w0;
			float temp2 = temp1 + w1 * temp0;
			f2 = temp2 + w2 * f1;
			f3 = w0 * temp1 + w1 * temp2 + w2 * f2;
			g0 = f2 + w0 * (f1 + w0);
			g1 = f2 + w1 * (f1 + w1);
			g2 = f2 + w2 * (f1 + w2);
		}
	};
}