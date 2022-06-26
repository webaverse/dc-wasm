mkdir -p bin
echo 'building main...'
# m = 64*1024; s = 350000000; Math.floor(s/m)*m;
# emcc -s NO_EXIT_RUNTIME=1 -s TOTAL_MEMORY=52428800 -D__linux__ -s ALLOW_MEMORY_GROWTH=0 -sWASM_WORKERS=1 -g -s ASSERTIONS=1 \
emcc -s TOTAL_MEMORY=52428800 -s ALLOW_MEMORY_GROWTH=0 -sWASM_WORKERS -O3 \
  objectize.cc \
  FastNoise.cpp util.cc vector.cc worley.cc \
  DualContouring/main.cc DualContouring/noise.cc DualContouring/vectorMath.cc DualContouring/qef.cc DualContouring/svd.cc DualContouring/biomes.cc DualContouring/density.cc DualContouring/mesh.cc DualContouring/octree.cc DualContouring/chunk.cc DualContouring/instance.cc DualContouring/context.cc DualContouring/task.cc DualContouring/result.cc DualContouring/lock.cc \
  -I. \
  -o bin/dc.js
  sed -Ei 's/var Module = typeof Module/var Module = typeof globalThis.Module/g' bin/dc.js
  sed -Ei 's/var result=WebAssembly.instantiateStreaming/info.__cxa_atexit=()=>{};var result=WebAssembly.instantiateStreaming/g' bin/dc.js
  sed -Ei 's/new Worker\(locateFile\("dc.ww.js"\)\);/new Worker(locateFile("dc.ww.js"),{type:"module"}\);/g' bin/dc.js
  sed -Ei 's/importScripts\(d.js\);/d.js="\/dc.js";import(d.js).then(() => {globalThis.Module._runLoop();});/g' bin/dc.ww.js
  echo 'let accept, reject;const p = new Promise((a, r) => {accept = a;  reject = r;});Module.postRun = () => {accept();};Module.waitForLoad = () => p;export default Module;' >> bin/dc.js
echo done

# Prevent compile window auto close after error, to see the error details. https://askubuntu.com/a/20353/1012283
# exec $SHELL
