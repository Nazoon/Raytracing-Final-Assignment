#include "first_hit.h"

bool first_hit(
	const Ray& ray,
	const double min_t,
	const std::vector< std::shared_ptr<Object> >& objects,
	int& hit_id,
	double& t,
	Eigen::Vector3d& n)
{
	double lowest_dist = std::numeric_limits<double>::infinity();
	double norm = ray.direction.norm();

	for (int i = 0; i < objects.size(); i++) {
		if (objects[i]->intersect(ray, min_t, t, n)) {
			if (lowest_dist > norm* t) {
				lowest_dist = norm * t;
				hit_id = i;
			}
		}
	}
	if (lowest_dist != std::numeric_limits<double>::infinity()) {
		objects[hit_id]->intersect(ray, min_t, t, n);
		return true;
	}
	return false;
}
