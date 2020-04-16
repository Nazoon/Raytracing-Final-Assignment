#include "Triangle.h"
#include "Ray.h"
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>

bool Triangle::intersect(
	const Ray& ray, const double min_t, double& t, Eigen::Vector3d& n) const
{
	Eigen::Vector3d p0, p1, p2, t1, t2, b, sols;
	Eigen::Matrix3d V;
	double alpha, beta, result_t;

	// Resolving vectors of legs of triangle
	std::tie(p0, p1, p2) = corners;
	t1 = p1 - p0;
	t2 = p2 - p0;

	// Solving values for 
	V << t1, t2, -ray.direction;
	b = ray.origin - p0;
	sols = V.householderQr().solve(b);
	alpha = sols[0];
	beta = sols[1];
	result_t = sols[2];

	if (alpha + beta <= 1 && alpha >= 0 && beta >= 0 && result_t >= min_t) {
		t = result_t;
		n = t1.cross(t2);
		n.normalize();
		return true;
	}
	return false;
}

bool Triangle::bounding_corners(Eigen::Vector3d& min, Eigen::Vector3d& max) const {
	Eigen::Vector3d p0, p1, p2;
	std::tie(p0, p1, p2) = corners;
	std::vector<Eigen::Vector3d> points = { p0, p1, p2 };
	for (int p = 0; p < 3; p++) {
		for (int dim = 0; dim < 3; dim++) {
			min[dim] = points[p][dim] < min[dim] ? points[p][dim] : min[dim];
			max[dim] = points[p][dim] > max[dim] ? points[p][dim] : max[dim];
		}
	}
	return true;
}

