# Raytracing Final Assignment

For my CSC418 final assignment, I augmented my existing A3 assignment solution with refraction and caustics. I also wrote a small script to compile the many frames which my code produces into a single `mp4` file.

## Lecture Section

I am in lecture section LEC0201.

## Requirements:

Make sure your operating system is able to run the commands `convert` and `ffmpeg`. If not, install them before proceeding.

## Usage

The script `build_movie.sh` handles compiling the project with `cmake` and `make`, as well as creating the actual output video. Use it as follows:

1. `cd` to the `source/` directory.
2. `sh build_movie.sh <width> <height> <speed> <quality>` to create the output `build-release/movie.mp4`.
	* `<width>` is the desired width of the output.
	* `<height>` is the desired height of the output.
	* `<speed>` determines the "framerate" of the output. 4 is the recommended speed. Anything higher will add more delay per frame.
	* `<quality>` determines the quality of `movie.mp4`. 100 is the maximum, with 70 being about medium quality.

For example, you may run `sh build_movie.sh 1280 720 4 100` to get a high-quality 1280x720 video. This will take a while!

## Implementation

The data for the scene is found in `source/data/my-scene.json`

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

To create the video output, the positions of some objects were moved once per frame, and each frame was outputted to its own `.ppm` file. Then, the `convert` and `ffmpeg` commands are used to compile these indivudal frames in a `.mp4` format.
