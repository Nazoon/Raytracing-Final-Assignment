#ifndef RAYCOLOR_H
#define RAYCOLOR_H
#include "Ray.h"
#include "Object.h"
#include "Light.h"
#include <Eigen/Core>
#include <vector>

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
  const Ray & ray, 
  const double min_t,
  const std::vector< std::shared_ptr<Object> > & objects,
  const std::vector< std::shared_ptr<Light> > & lights,
  const int num_recursive_calls,
  Eigen::Vector3d & rgb);

typedef struct caustic_point {
	Eigen::Vector3d pos, rgb;
} LightPoint;

const int MAX_POINTS_IN_LEAF = 8;
const double infinity = std::numeric_limits<double>::infinity();

class KDTree {
private:

	void insert_point_into_box(
		Eigen::Vector3d& min, 
		Eigen::Vector3d max, 
		Eigen::Vector3d pos) 
	{
		for (int d = 0; d < 3; d++) {
			min[d] = pos[d] < min[d] ? pos[d] : min[d];
			max[d] = pos[d] > max[d] ? pos[d] : max[d];
		}
	}

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
			double mid = longest_dim_length / 2.0 + min[longest_dim];
			std::vector<LightPoint> left_vec, right_vec;
			for (int i = 0; i < points.size(); i++) {
				if (points[i].pos[longest_dim] < mid) {
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
};

#endif
