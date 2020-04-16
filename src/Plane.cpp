#include "Plane.h"
#include "Ray.h"
#include <limits.h>

bool Plane::intersect(
	const Ray& ray, const double min_t, double& t, Eigen::Vector3d& n) const
{
	double denom = normal.dot(ray.direction);
	if (denom == 0) {
		// Direction is parallel to plane, no-hit
		return false;
	}
	double q = normal.dot(point);
	double norm_dot_e = normal.dot(ray.origin);
	double dist = (q - norm_dot_e) / denom;
	if (dist >= min_t) {
		t = dist;
		n = normal;
		n.normalize();
		return true;
	}
	return false;
}

bool Plane::bounding_corners(Eigen::Vector3d& min, Eigen::Vector3d& max) const {
	return false;
}
