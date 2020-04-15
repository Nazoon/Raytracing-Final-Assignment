cd build-release
make
rm -rf frames
mkdir frames
echo "Producing frames..."
./raytracing $1 $2 $3
echo "Creating gif..."
convert -delay $4 -quality $5 frames/rgb*ppm movie.gif
echo "Done."
