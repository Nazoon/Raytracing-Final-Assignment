# CSC418 Final Assignment

For my CSC418 final assignment, I augmented my existing A3 assignment solution with refraction and caustics. I also wrote a small script to compile the many frames which my code produces into a single gif.

## Lecture Section

I am in lecture section LEC0201.

## Usage

To see this project's output, you must set up as follows:

1. `cd` to `source/`
2. `mkdir build-release`, then `cd build-release`
3. `cmake ../ -DCMAKE_BUILD_TYPE=Release`

Now, the script `build_movie.sh` has everything it needs. Use it as follows:

1. `cd` to `source/` again.
2. `sh build_movie.sh ../data/my-scene.json <width> <height> <speed> <quality>` to create the output `build-release/movie.mp4`.
	* `<width>` is the desired width of the output.
	* `<height>` is the desired height of the output.
	* `<speed>` determines the "framerate" of the output. 4 is the recommended speed. Anything higher will add more delay per frame.
	* `<quality>` determines the quality of `movie.mp4`. 100 is the maximum, with 70 being about medium quality.

For example, you may run `sh build_movie.sh ../data/my-scene.json 1280 720 4 100` to get a high-quality 1280x720 video. This will take a while!

## Implementation

To implement refractive translucent materials, it was a matter of:
* Adding a refractive index and opacity to the Material struct
* Computing the direction that a refracted ray would take as it passes into a new medium. This was implemented in `raycolor.cpp`.
	* This was taken from a paper found on the https://graphics.stanford.edu/ website, which is cited in the documentation.

To implement caustics was a more interesting and difficult endeavour:
* Implemented a k-d tree similarly to the BVH tree found in A4.
	* The k-d tree is used to store points of caustic light in an easily searchable way.
* Implemented a range search function for the k-d tree.
* Implement a function `cast_light` in `raycolor.cpp`, which casts a ray of light from a light source into a scene, and adds a point of caustic light wherever the ray lands as it reflects and refracts about the scene.
* These implementation ideas were taken from an article cited in the file, but all the code is original.