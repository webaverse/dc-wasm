mkdir -p bin
echo 'building main...'
# m = 64*1024; s = 512 * 1024 * 1024; Math.floor(s/m)*m;
# emcc -s NO_EXIT_RUNTIME=1 -s TOTAL_MEMORY=52428800 -D__linux__ -s ALLOW_MEMORY_GROWTH=0 -sWASM_WORKERS=1 -g -s ASSERTIONS=1 -fexceptions \
NUM_THREADS=8
emcc -D NUM_THREADS=$NUM_THREADS -sNO_EXIT_RUNTIME=1 -s TOTAL_MEMORY=52428800 -pthread -sPTHREAD_POOL_SIZE=$NUM_THREADS -sPTHREAD_POOL_SIZE_STRICT=$NUM_THREADS -s ALLOW_MEMORY_GROWTH=0 -O3 \
  objectize.cc \
  FastNoise.cpp util.cc vector.cc worley.cc xxhash.c \
  DualContouring/main.cc DualContouring/noise.cc DualContouring/vectorMath.cc DualContouring/qef.cc DualContouring/svd.cc DualContouring/biomes.cc DualContouring/density.cc DualContouring/mesh.cc DualContouring/octree.cc DualContouring/chunk.cc DualContouring/instance.cc DualContouring/context.cc DualContouring/task.cc DualContouring/result.cc DualContouring/lock.cc DualContouring/sync.cc DualContouring/promise.cc DualContouring/cache.cc DualContouring/damage.cc DualContouring/tracker.cc \
  -I. \
  -o bin/dc.js

  #sed -Ei 's/var Module=typeof Module!="undefined"\?Module:\{\};/var Module = globalThis.Module??{};/g' bin/dc.js
  # sed -Ei 's/var asm=createWasm\(\);/asmLibraryArg.__cxa_atexit=()=>{};var asm=createWasm();/g' bin/dc.js
  sed -Ei 's/importScripts\(e.data.urlOrBlob\)/e.data.urlOrBlob="\/dc.js";importScripts(e.data.urlOrBlob)/g' bin/dc.worker.js
  echo 'let accept, reject;const p = new Promise((a, r) => {accept = a;  reject = r;});Module.postRun = () => {accept();};Module.waitForLoad = () => p;' >> bin/dc.js
  cp bin/dc.js bin/dc.module.js
  echo 'export default Module;' >>bin/dc.module.js
  cp -R bin/* ../app/public/
echo done

# Prevent compile window auto close after error, to see the error details. https://askubuntu.com/a/20353/1012283
# exec $SHELL
