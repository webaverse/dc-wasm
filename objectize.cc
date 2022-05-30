#include <emscripten.h>
#include "DualContouring/main.h"
#include <deque>
#include <map>

extern "C" {

EMSCRIPTEN_KEEPALIVE void setChunkSize(int newChunkSize) {
    return DualContouring::setChunkSize(newChunkSize);
}

EMSCRIPTEN_KEEPALIVE void generateChunkDataDualContouring(float x, float y, float z) {
    return DualContouring::generateChunkData(x, y, z);
}

EMSCRIPTEN_KEEPALIVE void setChunkLodDualContouring(float x, float y, float z, int lod) {
    return DualContouring::setChunkLod(x, y, z, lod);
}

EMSCRIPTEN_KEEPALIVE void clearTemporaryChunkDataDualContouring() {
    return DualContouring::clearTemporaryChunkData();
}

EMSCRIPTEN_KEEPALIVE void clearChunkRootDualContouring(float x, float y, float z) {
    return DualContouring::clearChunkRoot(x, y, z);
}

EMSCRIPTEN_KEEPALIVE uint8_t *createChunkMeshDualContouring(float x, float y, float z) {
    return DualContouring::createChunkMesh(x, y, z);
}

EMSCRIPTEN_KEEPALIVE bool drawDamageSphere(float x, float y, float z, float radius, float value, float *outPositions, unsigned int *outPositionsCount) {
    return DualContouring::drawDamageSphere(x, y, z, radius, value, outPositions, outPositionsCount);
}

EMSCRIPTEN_KEEPALIVE void doFree(void *ptr) {
    free(ptr);
}

} // extern "C"