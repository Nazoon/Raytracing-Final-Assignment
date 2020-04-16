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
#include <random>

const int rays_per_dim = 120;
const double min_t = 0.1;
const double skew = 0.01;

void setup_light_map(
	std::vector< std::shared_ptr<Object> > objects,
	std::vector< std::shared_ptr<Light> > lights,
	Eigen::Vector3d min,
	Eigen::Vector3d max,
	std::vector<LightPoint>& light_map
) {

	// We use a random skewing of each light ray to prevent banding
	// https://stackoverflow.com/questions/1340729/how-do-you-generate-a-random-double-uniformly-distributed-between-0-and-1-from-c/1340762
	std::random_device rd;
	std::mt19937 e2(rd());
	std::uniform_real_distribution<> dist(-skew, skew);

	Ray light_ray;
	Eigen::Vector3d ray_target;
	// For each light in the scene...
	for (std::shared_ptr<Light> l : lights) {
		// ...Point a ray of light from the source to a grid of points in the bounding box
		for (int x = 0; x < rays_per_dim; x++) {
			for (int y = 0; y < rays_per_dim; y++) {
				for (int z = 0; z < rays_per_dim; z++) {

					// Pointing a ray at the current position in the bounding box
					ray_target[0] = ((max[0] - min[0]) / rays_per_dim) * x + min[0];
					ray_target[1] = ((max[1] - min[1]) / rays_per_dim) * y + min[1];
					ray_target[2] = ((max[2] - min[2]) / rays_per_dim) * z + min[2];

					// Random skewing
					for (int i = 0; i < 3; i++)
						ray_target[i] += dist(e2);

					light_ray = l->ray_to_target(ray_target);
					//cast_light(light_ray, l->I / (rays_per_dim * rays_per_dim * rays_per_dim), min_t, objects, 0, light_map);
					cast_light(light_ray, l->I / (rays_per_dim * rays_per_dim) * 25, min_t, objects, 0, light_map);
					//cast_light(light_ray, l->I / (rays_per_dim), min_t, objects, 0, light_map);
					//cast_light(light_ray, l->I , min_t, objects, 0, light_map);
				}
			}
		}
	}
}

int main(int argc, char* argv[])
{

	Eigen::Vector3d move_direction(0.05, 0.05, 0.0);
	int num_frames = 120;
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

	// Figuring out names for each frame so that they are processed in alphabetical order
	std::vector<std::string> names;
	for (int i = 0; i < num_frames; i++) {
		names.emplace_back("rgb" + std::to_string(i));
	}
	std::sort(names.begin(), names.end());

	std::vector< std::shared_ptr<Object> > new_objects;
	new_objects.emplace_back(objects[0]);
	new_objects.emplace_back(objects[1]);
	objects = new_objects;

	// Rendering each frame
	std::vector<unsigned char> rgb_image(3 * width * height);
	for (int frame = 0; frame < num_frames; frame++) {
		printf("- Frame %d/%d...\n", frame, num_frames);

		// Setting up bounding box for scene
		Eigen::Vector3d min(infinity, infinity, infinity);
		Eigen::Vector3d max = -min;
		for (int i = 0; i < objects.size(); i++) {
			Eigen::Vector3d obj_min, obj_max;
			if (objects[i]->bounding_corners(obj_min, obj_max)) {
				insert_point_into_box(min, max, obj_min);
				insert_point_into_box(min, max, obj_max);
			}
		}
		// Expanding bounding box just a bit
		for (int i = 0; i < 3; i++) {
			min[i] -= 0.1;
			max[i] += 0.1;
		}
		// Setting up light map for scene
		std::vector<LightPoint> light_map = std::vector<LightPoint>();
		setup_light_map(objects, lights, min, max, light_map);
		printf("-- Light map done.\n");
		printf("--- # light rays cast = %d\n", rays_per_dim * rays_per_dim * rays_per_dim);
		printf("--- # caustic points  = %d\n", (int)light_map.size());

		// Turning light map into KD tree
		std::shared_ptr<KDTree> light_map_tree = std::shared_ptr<KDTree>(new KDTree(light_map));
		printf("-- KD tree done.\n");
		printf("--- # caustic points  = %d\n", light_map_tree->num_points());
		printf("--- max depth = %d\n", light_map_tree->max_depth());

		assert(light_map.size() == light_map_tree->num_points());

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
		objects[0]->center += move_direction;
		printf("-- Frame done.\n");
	}
}
