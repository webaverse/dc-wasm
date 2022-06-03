#include <emscripten.h>
#include "DualContouring/main.h"
#include <deque>
#include <map>

extern "C" {

EMSCRIPTEN_KEEPALIVE void initialize(int chunkSize, int seed) {
    DualContouring::initialize(chunkSize, seed);
}

/* EMSCRIPTEN_KEEPALIVE float *getChunkHeightField(float x, float y, float z) {
    return DualContouring::getChunkHeightField(x, y, z);
} */

EMSCRIPTEN_KEEPALIVE float getHeight(float x, float z) {
    return DualContouring::getHeight(x, z, 1);
}
EMSCRIPTEN_KEEPALIVE void getHeights(float *vec2s, int count, float *heights) {
    return DualContouring::getHeights(vec2s, count, heights, 1);        
}
EMSCRIPTEN_KEEPALIVE void getBiomesContainedInChunk(int x, int z, unsigned char *biomes, unsigned int *biomesCount) {
    return DualContouring::getBiomesContainedInChunk(x, z, biomes, biomesCount, 1);        
}

EMSCRIPTEN_KEEPALIVE void clearChunkRootDualContouring(float x, float y, float z) {
    return DualContouring::clearChunkRoot(x, y, z);
}

EMSCRIPTEN_KEEPALIVE uint8_t *createChunkMeshDualContouring(float x, float y, float z, int lod) {
    return DualContouring::createChunkMesh(x, y, z, lod);
}

EMSCRIPTEN_KEEPALIVE bool drawSphereDamage(float x, float y, float z, float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages) {
    return DualContouring::drawSphereDamage(x, y, z, radius, outPositions, outPositionsCount, outDamages, 1);
}

EMSCRIPTEN_KEEPALIVE bool eraseSphereDamage(float x, float y, float z, float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages) {
    return DualContouring::eraseSphereDamage(x, y, z, radius, outPositions, outPositionsCount, outDamages, 1);
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
        outDamages,
        1
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
        outDamages,
        1
    );
}

EMSCRIPTEN_KEEPALIVE void injectDamage(float x, float y, float z, float *damageBuffer) {
    DualContouring::injectDamage(x, y, z, damageBuffer, 1);
}

EMSCRIPTEN_KEEPALIVE void *doMalloc(size_t size) {
    return malloc(size);
}

EMSCRIPTEN_KEEPALIVE void doFree(void *ptr) {
    free(ptr);
}

} // extern "C"