#include "Object.h"
#include "Camera.h"
#include "Light.h"
#include "read_json.h"
#include "write_ppm.h"
#include "viewing_ray.h"
#include "raycolor.h"
#include <Eigen/Core>
#include <vector>
#include <iostream>
#include <memory>
#include <limits>
#include <functional>
#include <string>
#include <algorithm>
#include <cstdlib> 

const int rays_per_dim = 100;
const double min_t = 1.0;

std::vector<LightPoint> setup_light_map(
	std::vector< std::shared_ptr<Object> > objects,
	std::vector< std::shared_ptr<Light> > lights,
	Eigen::Vector3d min,
	Eigen::Vector3d max,
	std::vector<LightPoint>& light_map
) {
	Ray light_ray;
	Eigen::Vector3d ray_target;
	for (std::shared_ptr<Light> l : lights) {
		for (int x = 0; x < rays_per_dim; x++) {
			for (int y = 0; x < rays_per_dim; x++) {
				for (int z = 0; x < rays_per_dim; x++) {
					ray_target[0] = ((max[0] - min[0]) / rays_per_dim) * x + min[0];
					ray_target[1] = ((max[1] - min[1]) / rays_per_dim) * y + min[1];
					ray_target[2] = ((max[2] - min[2]) / rays_per_dim) * z + min[2];

					light_ray = l->ray_to_target(ray_target);
					cast_light(light_ray, l->I, min_t, objects, 0, light_map);
				}
			}
		}
	}
}

int main(int argc, char* argv[])
{

	Eigen::Vector3d move_direction(0.05, 0.05, 0.0);
	int num_frames = 180;
	Camera camera;
	std::vector< std::shared_ptr<Object> > objects;
	std::vector< std::shared_ptr<Light> > lights;

	// Read a camera and scene description from given .json file
	std::string json_file = argv[1];
	int width = atoi(argv[2]);
	int height = atoi(argv[3]);
	read_json(
		json_file,
		camera,
		objects,
		lights);

	// Setting up bounding box for scene
	Eigen::Vector3d min(infinity, infinity, infinity);
	Eigen::Vector3d max = -min;
	for (std::shared_ptr<Object> o : objects) {
		Eigen::Vector3d obj_min, obj_max;
		o->bounding_corners(obj_min, obj_max);
		insert_point_into_box(min, max, obj_min);
		insert_point_into_box(min, max, obj_max);
	}
	// Setting up light map for scene
	std::vector<LightPoint> light_map = std::vector<LightPoint>();
	setup_light_map(objects, lights, min, max, light_map);

	// Turning light map into KD tree
	std::shared_ptr<KDTree> light_map_tree = std::shared_ptr<KDTree>(new KDTree(light_map));

	// Figuring out names for each frame so that they are processed in alphabetical order
	std::vector<std::string> names;
	for (int i = 0; i < num_frames; i++) {
		names.emplace_back("rgb" + std::to_string(i));
	}
	std::sort(names.begin(), names.end());

	// Rendering each frame
	std::vector<unsigned char> rgb_image(3 * width * height);
	for (int frame = 0; frame < num_frames; frame++) {
		// For each pixel (i,j)
		for (unsigned i = 0; i < height; ++i)
		{
			for (unsigned j = 0; j < width; ++j)
			{
				// Set background color
				Eigen::Vector3d rgb(0, 0, 0);

				// Compute viewing ray
				Ray ray;
				viewing_ray(camera, i, j, width, height, ray);

				// Shoot ray and collect color
				raycolor(ray, min_t, objects, lights, 0, light_map_tree, rgb);

				// Write double precision color into image
				auto clamp = [](double s) { return std::max(std::min(s, 1.0), 0.0); };
				rgb_image[0 + 3 * (j + width * i)] = 255.0 * clamp(rgb(0));
				rgb_image[1 + 3 * (j + width * i)] = 255.0 * clamp(rgb(1));
				rgb_image[2 + 3 * (j + width * i)] = 255.0 * clamp(rgb(2));

			}
		}
		write_ppm("frames/" + names[frame] + ".ppm", rgb_image, width, height, 3);
		objects[4]->center += move_direction;
	}
}
