cd build-release
make
rm -rf frames
mkdir frames
date +"%c"
echo "Rendering frames..."
./raytracing $1 $2 $3
date +"%c"
echo "Creating mp4..."
convert -delay $4 -quality $5 frames/rgb*ppm movie.mp4
date +"%c"
echo "Done."
