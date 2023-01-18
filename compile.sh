mkdir -p bin
echo 'building main...'
# m = 64*1024; s = 200 * 1024 * 1024; Math.floor(s/m)*m;
# emcc -s NO_EXIT_RUNTIME=1 -s TOTAL_MEMORY=52428800 -D__linux__ -s ALLOW_MEMORY_GROWTH=0 -sWASM_WORKERS=1 -g -s ASSERTIONS=1 -fexceptions \

# -O3
# -g -s ASSERTIONS=1 -fsanitize=address

emcc -sNO_EXIT_RUNTIME=1 -s TOTAL_MEMORY=314572800 -s ALLOW_MEMORY_GROWTH=0 \
  -O3 \
  objectize.cc \
  util.cc vector.cc MC.cc VHACD.cc \
  DualContouring/main.cc DualContouring/vectorMath.cc DualContouring/qef.cc DualContouring/svd.cc DualContouring/density.cc DualContouring/mesh.cc DualContouring/octree.cc DualContouring/instance.cc DualContouring/context.cc \
  -I. \
  -o bin/dc.js

  # sed -Ei 's/importScripts\(e.data.urlOrBlob\)/importScripts(e.data.urlOrBlob.replace(\/dc-worker\\.js.*$\/, "dc.js"))/g' bin/dc.worker.js
  echo 'let accept, reject;const p = new Promise((a, r) => {accept = a;  reject = r;});Module.postRun = () => {accept();};Module.waitForLoad = () => p;' >> bin/dc.js
  sed -Ei 's/scriptDirectory\+path/"\/"+path/g' bin/dc.js
  cp bin/dc.js bin/dc.module.js
  echo 'export default Module;' >>bin/dc.module.js

echo done