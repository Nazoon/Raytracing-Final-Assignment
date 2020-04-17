#include <Eigen/Core>
#include <math.h>

Eigen::Vector3d reflect(const Eigen::Vector3d& in, const Eigen::Vector3d& n)
{
	Eigen::Vector3d reflected_ray = in - (2 * in.dot(n) * n);
	return reflected_ray.normalized();
}

// https://graphics.stanford.edu/courses/cs148-10-summer/docs/2006--degreve--reflection_refraction.pdf
Eigen::Vector3d refract(const Eigen::Vector3d & in, const Eigen::Vector3d & n, double eta1, double eta2)
{
	double index_ratio = eta1 / eta2;
	double cos_i = -in.dot(n);
	double norm_coeff = ((index_ratio * cos_i) - std::sqrt(1.0 - (index_ratio * index_ratio * (1.0 - cos_i * cos_i))));
	return (index_ratio * in) + (norm_coeff * n);
}
