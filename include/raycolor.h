#ifndef RAYCOLOR_H
#define RAYCOLOR_H
#include "Ray.h"
#include "Object.h"
#include "Light.h"
#include <Eigen/Core>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <limits>

typedef struct caustic_point {
	Eigen::Vector3d pos, rgb;
} LightPoint;

/*
Given the min and max corners of an AABB, insert another point into it.
*/
void insert_point_into_box(
	Eigen::Vector3d& min,
	Eigen::Vector3d& max,
	Eigen::Vector3d pos);

const int MAX_POINTS_IN_LEAF = 2;
const double infinity = std::numeric_limits<double>::infinity();
const double light_map_range = 0.2;

const int max_num_recursive_calls = 50;
const double fudge = 0.000001;

class KDTree {
private:

	bool ranges_overlap(double a1, double a2, double b1, double b2) {
		return
			(a1 <= b1 && b1 <= a2) ||
			(a1 <= b2 && b2 <= a2) ||
			(b1 <= a1 && a1 <= b2) ||
			(b1 <= a2 && a2 <= b2);
	}

	bool box_in_range(
		Eigen::Vector3d center,
		double radius,
		Eigen::Vector3d min,
		Eigen::Vector3d max)
	{
		Eigen::Vector3d range_min(center[0] - radius, center[1] - radius, center[2] - radius);
		Eigen::Vector3d range_max(center[0] + radius, center[1] + radius, center[2] + radius);
		for (int d = 0; d < 3; d++) {
			if (!ranges_overlap(min[d], max[d], range_min[d], range_max[d]))
				return false;
		}
		return true;
	}

	std::function<bool(LightPoint, LightPoint)> compare_points_func(int dim) {
		return [dim](LightPoint a, LightPoint b) {return a.pos[dim] > b.pos[dim]; };
	}

public:

	// Subtrees if needed
	std::shared_ptr<KDTree> left, right;
	// If leaf, containing points here
	std::vector<LightPoint> light_points;
	// Corners of bounding box
	Eigen::Vector3d min, max;

	// Recursive constructor
	KDTree(std::vector<LightPoint> points) {

		// Initializing corners of bounding box
		min = Eigen::Vector3d(infinity, infinity, infinity);
		max = -min;
		for (int i = 0; i < points.size(); i++) {
			insert_point_into_box(min, max, points[i].pos);
		}

		// Deciding if we need to branch or not
		if (points.size() <= MAX_POINTS_IN_LEAF) {
			light_points = points;
			left = std::shared_ptr<KDTree>(NULL);
			right = std::shared_ptr<KDTree>(NULL);
		}
		else {

			// Find longest dimension of bounding box
			int longest_dim = -1;
			double longest_dim_length = -1;
			for (int i = 0; i < 3; i++) {
				double cur_dim_length = max[i] - min[i];
				if (cur_dim_length > longest_dim_length) {
					longest_dim_length = cur_dim_length;
					longest_dim = i;
				}
			}

			// Split points down longest dimension into subtrees
			std::sort(points.begin(), points.end(), compare_points_func(longest_dim));
			//printf("n=%d, dim=%d, dim_length=%f, max[dim]=%f, min[dim]=%f...\n", (int)points.size(), longest_dim + 1, longest_dim_length, max[longest_dim], min[longest_dim]);
			std::vector<LightPoint> left_vec, right_vec;
			for (int i = 0; i < points.size(); i++) {
				if (i <= points.size() / 2) {
					left_vec.emplace_back(points[i]);
				}
				else {
					right_vec.emplace_back(points[i]);
				}
			}

			// Recurse into subtrees
			left = std::shared_ptr<KDTree>(new KDTree(left_vec));
			right = std::shared_ptr<KDTree>(new KDTree(right_vec));
		}
	}

	// Range checker
	// Inputs:
	//	center - positions to check for points around
	//	radius - maximum distance from center for a point to be counted
	// Outputs:
	//	points - LightPoints within radius of center
	//	sdists - #points vector where sdists[i] is the squared distance from points[i].pos to center
	void get_points_in_range(
		Eigen::Vector3d center,
		double radius,
		std::vector<LightPoint>& points,
		std::vector<double>& sdists)
	{
		//std::cout << "min:" << min << std::endl << "max:" << max << std::endl;

		// Return immediately if this box is not in the range
		if (!box_in_range(center, radius, min, max)) {
			return;
		}

		// If this KDTree is a leaf, check its points
		if (light_points.size() != 0) {
			double srad = radius * radius;
			for (int i = 0; i < light_points.size(); i++) {
				double sdist = (light_points[i].pos - center).squaredNorm();
				if (sdist <= srad) {
					points.emplace_back(light_points[i]);
					sdists.emplace_back(sdist);
				}
			}
		}
		else {
			// Recurse (will terminate early if it can, thanks to the break condition)
			left->get_points_in_range(center, radius, points, sdists);
			right->get_points_in_range(center, radius, points, sdists);
		}
	}

	int max_depth() {
		if (light_points.size() > 0)
			return 0;
		return 1 + std::max(left->max_depth(), right->max_depth());
	}

	int num_points() {
		int n = light_points.size();
		if (left != NULL) {
			n += left->num_points();
		}
		if (right != NULL) {
			n += right->num_points();
		}
		return n;
	}
};

// Shoot a ray into a lit scene and collect color information.
//
// Inputs:
//   ray  ray along which to search
//   min_t  minimum t value to consider (for viewing rays, this is typically at
//     least the _parametric_ distance of the image plane to the camera)
//   objects  list of objects (shapes) in the scene
//   lights  list of lights in the scene
//   num_recursive_calls  how many times has raycolor been called already
// Outputs:
//   rgb  collected color 
// Returns true iff a hit was found
bool raycolor(
	const Ray& ray,
	const double min_t,
	const std::vector< std::shared_ptr<Object> >& objects,
	const std::vector< std::shared_ptr<Light> >& lights,
	const int num_recursive_calls,
	const std::shared_ptr<KDTree> light_map_tree,
	Eigen::Vector3d& rgb);

/*
Sets up a light map for caustics
http://www.follick.ca/rt/

Inputs: Mostly the same as raycolour, with the addition of ray_rgb so we may know the colour of the light ray.
Outputs: light_points, the "light map" which is to be passed into a KDTree for range checking after.
*/
void cast_light(
	const Ray& ray,
	const Eigen::Vector3d ray_rgb,
	const double min_t,
	const std::vector< std::shared_ptr<Object> >& objects,
	const int num_recursive_calls,
	std::vector<LightPoint>& light_points);

#endif
