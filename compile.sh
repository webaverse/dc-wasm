# emsdk 2.0.6
mkdir -p bin
echo 'building main...'
# m = 64*1024; s = 350000000; Math.floor(s/m)*m;
# emcc -s WASM=1 -s NO_EXIT_RUNTIME=1 -s TOTAL_MEMORY=419430400 -s ALLOW_MEMORY_GROWTH=1 -O3
emcc -s WASM=1 -s NO_EXIT_RUNTIME=1 -s TOTAL_MEMORY=52428800 -D__linux__ -s ALLOW_MEMORY_GROWTH=0 -O3 \
  objectize.cc \
  DualContouring/math/FastNoise.cpp DualContouring/math/vectorMath.cc DualContouring/math/util.cc DualContouring/math/vector.cc \
  DualContouring/math/qef.cc DualContouring/math/svd.cc  \
  DualContouring/chunk/noise.cc DualContouring/chunk/density.cc \
  DualContouring/mesh/mesh.cc \
  DualContouring/main.cc \
  -I. \
  -o bin/dc.js
  sed -Ei 's/dc.wasm/bin\/dc.wasm/g' bin/dc.js
  echo 'let accept, reject;const p = new Promise((a, r) => {  accept = a;  reject = r;});Module.postRun = () => {  accept();};Module.waitForLoad = () => p;run();export default Module;' >> bin/dc.js
echo done

# Prevent compile window auto close after error, to see the error details. https://askubuntu.com/a/20353/1012283
# exec $SHELL
