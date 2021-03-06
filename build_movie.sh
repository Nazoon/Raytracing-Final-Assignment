mkdir build-release
cd build-release
cmake ../ -DCMAKE_BUILD_TYPE=Release
make
rm -rf frames
mkdir frames
date +"%c"
echo "Rendering frames..."
./raytracing  ../data/my-scene.json $1 $2
date +"%c"
echo "Creating gif..."
convert -delay $3 -quality $4 frames/rgb*ppm movie.gif
date +"%c"
echo "Converting to mp4..."
ffmpeg -i movie.gif -movflags faststart -pix_fmt yuv420p -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" -y -v 0 movie.mp4
date +"%c"
echo "Done."
