#pragma once

#include <vector>
#include <memory>

#include <glm/glm.hpp>

#include "../geometry/Shape.h"

namespace fiz
{
	struct BVHPrimitive
	{
		AABB aabb;
		glm::vec3 centroid;
		int index; // index into static bodies
	};
	struct BVHNode
	{
		AABB aabb;
		BVHNode* left;
		BVHNode* right;
		int split_axis;
		int prim_start;
		int prim_count;

		void initLeaf(int start, int count, AABB& bounds)
		{
			prim_start = start;
			prim_count = count;
			aabb = bounds;
			left = nullptr;
			right = nullptr;
		}

		void initInterior(int axis, BVHNode* l, BVHNode* r)
		{
			left = l;
			right = r;
			aabb = l->aabb;
			aabb.combine(r->aabb);
			split_axis = axis;
			prim_count = 0;
		}
	};

	struct mat3x3
	{
		union
		{
			glm::vec3 a;
			glm::vec3 b;
			glm::vec3 c;
			float data[9];
		};
	};

	struct LinearBVHNode
	{
		AABB aabb;
		union {
			int primitive_offset;
			int second_child_offset;
		};
		uint16_t primitive_count;
		uint8_t axis;
		uint8_t pad[1];
	};

	enum BVHSplitMode
	{
		MIDPOINT,
		EQUAL_COUNTS
	};

	template<typename T>
	struct BVH
	{
		std::vector<T>* primitives;

		BVHSplitMode mode;

		std::vector<LinearBVHNode> nodes;

		bool is_built;

		BVH(std::vector<T>* primitives) : primitives(primitives), mode(BVHSplitMode::MIDPOINT), is_built(false)
		{
			
		}

		void createBVH()
		{
			std::vector<BVHPrimitive> primitive_info(primitives->size());
			std::vector<T> ordered_primitives;
			for (unsigned int i = 0; i < primitives->size(); ++i)
			{
				BVHPrimitive bvh_primitive;
				bvh_primitive.index = i;
				bvh_primitive.aabb = (*primitives)[i].aabb;
				bvh_primitive.centroid = (bvh_primitive.aabb.min + bvh_primitive.aabb.max) * 0.5f;
				primitive_info[i] = bvh_primitive;
			}

			int total_nodes = 0;
			BVHNode* root = recursiveBuild(primitive_info, 0, primitive_info.size(), &total_nodes, ordered_primitives);

			primitives->swap(ordered_primitives);

			nodes.resize(total_nodes);
			int offset = 0;
			flattenBVHTree(root, &offset);

			is_built = true;
		}

		BVHNode* recursiveBuild(std::vector<BVHPrimitive>& primitive_info,
							int start,
							int end,
							int* total_nodes,
							std::vector<T>& ordered_primitives)
		{
			// create new node
			BVHNode* node = new BVHNode();
			(*total_nodes)++;

			// compute bounds of all primitives in node
			AABB aabb = primitive_info[start].aabb;
			for (int i = start; i < end; ++i)
				aabb.combine(primitive_info[i].aabb);

			int n_primitives = end - start;
			if (n_primitives <= 4)
			{
				// create leaf BVH node
				int first_primitive_offset = ordered_primitives.size();
				for (int i = start; i < end; ++i)
				{
					int primitive_number = primitive_info[i].index;
					ordered_primitives.push_back((*primitives)[primitive_number]);
				}
				node->initLeaf(first_primitive_offset, n_primitives, aabb);
				return node;
			}
			else
			{
				// compute bound of primitive centroids
				AABB centroid_bounds = AABB(primitive_info[start].centroid, primitive_info[start].centroid);
				for (int i = start; i < end; ++i)
					centroid_bounds.combine(primitive_info[i].centroid);
				// choose split dimension
				int dim = centroid_bounds.maxExtent();

				// partition primitives into two sets
				int mid = (start + end) / 2;
				if (centroid_bounds.max[dim] == centroid_bounds.min[dim]) // all centroid points were at same position
				{
					// create leaf BVH node
					int first_primitive_offset = ordered_primitives.size();
					for (int i = start; i < end; ++i)
					{
						int primitive_number = primitive_info[i].index;
						ordered_primitives.push_back((*primitives)[primitive_number]);
					}
					node->initLeaf(first_primitive_offset, n_primitives, aabb);
					return node;
				}
				else
				{
					// partition primitives based on split mode
					switch (mode)
					{
					case BVHSplitMode::MIDPOINT:
					{
						float pmid = (centroid_bounds.min[dim] + centroid_bounds.max[dim]) * 0.5f;
						BVHPrimitive* mid_ptr = std::partition(&primitive_info[start], &primitive_info[end - 1], [dim, pmid](const BVHPrimitive& pi) {
							return pi.centroid[dim] < pmid;
							});
						mid = mid_ptr - &primitive_info[0];
						if (mid != start && mid != end)
							break;
					}
					case BVHSplitMode::EQUAL_COUNTS:
					{
						mid = (start + end) / 2;
						std::nth_element(&primitive_info[start], &primitive_info[mid], &primitive_info[end - 1] + 1, [dim](const BVHPrimitive& a, const BVHPrimitive& b) {
							return a.centroid[dim] < b.centroid[dim];
						});
					}
					}

					node->initInterior(dim,
						recursiveBuild(primitive_info, start, mid, total_nodes, ordered_primitives),
						recursiveBuild(primitive_info, mid, end, total_nodes, ordered_primitives));
				}
			}

			return node;
		}

		int flattenBVHTree(BVHNode* node, int* offset)
		{
			LinearBVHNode* linear_node = &nodes[*offset];
			linear_node->aabb = node->aabb;
			int my_offset = (*offset)++;
			if (node->prim_count > 0)
			{
				linear_node->primitive_offset = node->prim_start;
				linear_node->primitive_count = node->prim_count;
			}
			else
			{
				// create interior flattened BVH node
				linear_node->axis = node->split_axis;
				linear_node->primitive_count = 0;
				flattenBVHTree(node->left, offset);
				linear_node->second_child_offset = flattenBVHTree(node->right, offset);
			}
			return my_offset;
		}

		void traverse(AABB& aabb, std::vector<int>& collisions)
		{
			int to_visit[64];
			int to_visit_offset = 0;
			int current_node_index = 0;
			while (true)
			{
				LinearBVHNode* node = &nodes[current_node_index];

				// check AABB against BVH node
				if (aabb.intersects(node->aabb))
				{
					if (node->primitive_count > 0)
					{
						// intersect AABB with primitives in leaf node
						for (unsigned int i = 0; i < node->primitive_count; ++i)
						{
							if (aabb.intersects((*primitives)[i + node->primitive_offset].aabb))
							{
								collisions.push_back(i + node->primitive_offset);
							}
						}
						if (to_visit_offset == 0)
							break;
						current_node_index = to_visit[--to_visit_offset];
					}
					else
					{
						// put node on stack, advance to next node
						to_visit[to_visit_offset++] = current_node_index + 1;
						current_node_index = node->second_child_offset;
					}
				}
				else
				{
					if (to_visit_offset == 0)
						break;
					current_node_index = to_visit[--to_visit_offset];
				}
			}
		}

		float traverse(Ray* ray)
		{
			float closest_hit = 9999999.9f;
			bool hit = false;
			glm::vec3 inv_dir = { 1.0f / ray->dir.x, 1.0f / ray->dir.y, 1.0f / ray->dir.z };
			int is_neg[3] = { inv_dir.x < 0, inv_dir.y < 0, inv_dir.z < 0 };

			int to_visit_offset = 0;
			int current_node_index = 0;
			int to_visit[64];
			while (true)
			{
				const LinearBVHNode* node = &nodes[current_node_index];

				// check ray against BVH node
				if (node->aabb.intersects(ray, inv_dir, is_neg))
				{
					if (node->primitive_count > 0)
					{
						// intersect ray with primitives in leaf
						for (int i = 0; i < node->primitive_count; ++i)
						{
							Ray r = { (*primitives)[i + node->primitive_offset].getLocalPos(ray->start),
									  (*primitives)[i + node->primitive_offset].getLocalVec(ray->dir) };
							float dist = (*primitives)[i + node->primitive_offset].shapes[0]->castRay(r);
							if (dist > 0)
							{
								closest_hit = fmin(closest_hit, dist);
								hit = true; // change to check against shape, update closest hit
							}
						}
						if (to_visit_offset == 0)
							break;
						current_node_index = to_visit[--to_visit_offset];
					}
					else
					{
						// put far BVH nodes on to visit stack, advance to near node
						if (is_neg[node->axis])
						{
							to_visit[to_visit_offset++] = node->second_child_offset;
							current_node_index = current_node_index + 1;
						}
						else
						{
							to_visit[to_visit_offset++] = current_node_index + 1;
							current_node_index = node->second_child_offset;
						}
					}
				}
				else
				{
					if (to_visit_offset == 0)
						break;
					current_node_index = to_visit[--to_visit_offset];
				}
			}
			return hit ? 1 : 0;
		}
	};
}