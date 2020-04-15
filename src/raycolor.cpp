#include "raycolor.h"
#include "first_hit.h"
#include "blinn_phong_shading.h"
#include "reflect.h"
#include <math.h>
#include <Eigen/Core>
#include <stdio.h>

const int max_num_recursive_calls = 30;
const double fudge = 0.000001;

bool raycolor(
	const Ray& ray,
	const double min_t,
	const std::vector< std::shared_ptr<Object> >& objects,
	const std::vector< std::shared_ptr<Light> >& lights,
	const int num_recursive_calls,
	Eigen::Vector3d& rgb)
{
	int hit_id;
	double t;
	Eigen::Vector3d n;
	if (first_hit(ray, min_t, objects, hit_id, t, n)) {

		// Basic shading
		rgb += blinn_phong_shading(ray, hit_id, t, n, objects, lights);

		// This is the raytracing part
		if (num_recursive_calls < max_num_recursive_calls) {

			if (objects[hit_id]->material->refractive_index != -1) {

				// Refractive material!

				// Checking for exiting a translucent material
				double eta1 = ray.cur_medium_refractive_index;
				double eta2 = objects[hit_id]->material->refractive_index;
				if (eta1 == eta2) {
					// We assume the ray to be exiting the material into air.
					// This means we are not allowed to have overlapping translucent materials.
					eta2 = 1.0;
				}

				// Setting reflectance and transmittance variables
				// https://graphics.stanford.edu/courses/cs148-10-summer/docs/2006--degreve--reflection_refraction.pdf
				Eigen::Vector3d ray_dot_n_times_n = ray.direction.dot(n) * n;
				double ratio = eta1 / eta2;
				double cos_i = ray_dot_n_times_n.norm();
				double sin_i = (ray.direction - ray_dot_n_times_n).norm();
				double sin2_t = ratio * ratio * (1 - cos_i * cos_i);
				double cos_t = std::sqrt(1 - sin2_t);
				double T, R;
				double R_0 = (eta1 - eta2) / (eta1 + eta2);
				R_0 *= R_0;
				if (eta1 <= eta2) {
					double x = 1.0 - cos_i;
					R = R_0 + (1.0 - R_0) * x * x * x * x * x;
				}
				else {
					bool total_internal_reflection = sin_i > (eta2 / eta1);
					if (total_internal_reflection) {
						R = 1.0;
					}
					else {
						double x = 1.0 - cos_t;
						R = R_0 + (1.0 - R_0) * x * x * x * x * x;
					}
				}
				T = 1.0 - R;

				//if (T > 1.0) printf("Error: T=%f\n", T);
				if (T > 1.0) T = 1.0;

				// Combining relfected ray and refracted ray
				// Relfected light
				if (R > 0.0) {
					Eigen::Vector3d reflect_rgb(0, 0, 0);
					Ray reflect_ray;
					reflect_ray.origin = ray.origin + (t * ray.direction);
					reflect_ray.direction = reflect(ray.direction, n);
					reflect_ray.cur_medium_refractive_index = eta1;
					if (raycolor(reflect_ray, fudge, objects, lights, num_recursive_calls + 1, reflect_rgb)) {
						for (int i = 0; i < 3; i++) {
							rgb[i] += reflect_rgb[i] * R * objects[hit_id]->material->km[i];
						}
					}
				}
				// Refracted light
				if (T > 0.0) {
					Eigen::Vector3d refract_rgb(0, 0, 0);
					Ray refract_ray;
					refract_ray.origin = ray.origin + (t * ray.direction);
					refract_ray.direction = refract(ray.direction, n, eta1, eta2);
					refract_ray.cur_medium_refractive_index = eta2;
					if (raycolor(refract_ray, fudge, objects, lights, num_recursive_calls + 1, refract_rgb)) {
						for (int i = 0; i < 3; i++) {
							rgb[i] += refract_rgb[i] * T * (1.0 - objects[hit_id]->material->opacity[i]);
						}
					}
				}
			}
			else {
				// Opaque material, just compute reflected light
				Eigen::Vector3d reflect_rgb(0, 0, 0);
				Ray reflect_ray;
				reflect_ray.origin = ray.origin + (t * ray.direction);
				reflect_ray.direction = reflect(ray.direction, n);
				if (raycolor(reflect_ray, fudge, objects, lights, num_recursive_calls + 1, reflect_rgb)) {
					for (int i = 0; i < 3; i++) {
						rgb[i] += reflect_rgb[i] * objects[hit_id]->material->km[i];
					}
				}
			}
		}

		return true;
	}
	return false;
}

const int MAX_POINTS_IN_LEAF = 8;
const double infinity = std::numeric_limits<double>::infinity();

typedef struct caustic_point {
	Eigen::Vector3d pos, rgb;
} LightPoint;

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
