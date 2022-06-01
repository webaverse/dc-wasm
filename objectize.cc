#include <emscripten.h>
#include "DualContouring/main.h"
#include <deque>
#include <map>

extern "C" {

EMSCRIPTEN_KEEPALIVE void initialize(int chunkSize, int seed) {
    DualContouring::initialize(chunkSize, seed);
}

EMSCRIPTEN_KEEPALIVE float *getChunkHeightField(float x, float y, float z) {
    return DualContouring::getChunkHeightField(x, y, z);
}

EMSCRIPTEN_KEEPALIVE void clearChunkRootDualContouring(float x, float y, float z) {
    return DualContouring::clearChunkRoot(x, y, z);
}

EMSCRIPTEN_KEEPALIVE uint8_t *createChunkMeshDualContouring(float x, float y, float z, int lod) {
    return DualContouring::createChunkMesh(x, y, z, lod);
}

EMSCRIPTEN_KEEPALIVE bool drawSphereDamage(float x, float y, float z, float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages) {
    return DualContouring::drawSphereDamage(x, y, z, radius, outPositions, outPositionsCount, outDamages);
}

EMSCRIPTEN_KEEPALIVE bool eraseSphereDamage(float x, float y, float z, float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages) {
    return DualContouring::eraseSphereDamage(x, y, z, radius, outPositions, outPositionsCount, outDamages);
}

EMSCRIPTEN_KEEPALIVE bool drawCubeDamage(
    float x, float y, float z,
    float qx, float qy, float qz, float qw,
    float sx, float sy, float sz,
    float *outPositions,
    unsigned int *outPositionsCount,
    float *outDamages
) {
    return DualContouring::drawCubeDamage(
        x, y, z,
        qx, qy, qz, qw,
        sx, sy, sz,
        outPositions,
        outPositionsCount,
        outDamages
    );
}

EMSCRIPTEN_KEEPALIVE bool eraseCubeDamage(
    float x, float y, float z,
    float qx, float qy, float qz, float qw,
    float sx, float sy, float sz,
    float *outPositions,
    unsigned int *outPositionsCount,
    float *outDamages
) {
    return DualContouring::eraseCubeDamage(
        x, y, z,
        qx, qy, qz, qw,
        sx, sy, sz,
        outPositions,
        outPositionsCount,
        outDamages
    );
}

EMSCRIPTEN_KEEPALIVE void injectDamage(float x, float y, float z, float *damageBuffer) {
    DualContouring::injectDamage(x, y, z, damageBuffer);
}

EMSCRIPTEN_KEEPALIVE void *doMalloc(size_t size) {
    return malloc(size);
}

EMSCRIPTEN_KEEPALIVE void doFree(void *ptr) {
    free(ptr);
}

} // extern "C"