#include <emscripten.h>
#include "DualContouring/main.h"

extern "C" {

EMSCRIPTEN_KEEPALIVE void initialize(int chunkSize, int seed) {
    DualContouring::initialize(chunkSize, seed);
}

EMSCRIPTEN_KEEPALIVE DCInstance *createInstance() {
    return DualContouring::createInstance();
}
EMSCRIPTEN_KEEPALIVE void destroyInstance(DCInstance *instance) {
    DualContouring::destroyInstance(instance);
}

EMSCRIPTEN_KEEPALIVE void getHeightfieldRange(DCInstance *inst, int x, int z, int w, int h, int lod, float *heights) {
    return inst->getHeightfieldRange(x, z, w, h, lod, heights);
}
EMSCRIPTEN_KEEPALIVE void getChunkSkylight(DCInstance *inst, int x, int y, int z, int lod, unsigned char *skylights) {
    return inst->getChunkSkylight(x, y, z, lod, skylights);
}
EMSCRIPTEN_KEEPALIVE void getChunkAo(DCInstance *inst, int x, int y, int z, int lod, unsigned char *aos) {
    return inst->getChunkAo(x, y, z, lod, aos);
}
EMSCRIPTEN_KEEPALIVE void getSkylightFieldRange(DCInstance *inst, int x, int y, int z, int w, int h, int d, int lod, unsigned char *skylights) {
    return inst->getSkylightFieldRange(x, y, z, w, h, d, lod, skylights);
}
EMSCRIPTEN_KEEPALIVE void getAoFieldRange(DCInstance *inst, int x, int y, int z, int w, int h, int d, int lod, unsigned char *aos) {
    return inst->getAoFieldRange(x, y, z, w, h, d, lod, aos);
}
/* EMSCRIPTEN_KEEPALIVE void getBiomesContainedInChunk(DCInstance *inst, int x, int z, unsigned char *biomes, unsigned int *biomesCount) {
    return inst->getBiomesContainedInChunk(x, z, biomes, biomesCount, 1);        
} */

EMSCRIPTEN_KEEPALIVE void createGrassSplat(DCInstance *inst, float x, float z, int lod, float *ps, float *qs, float *instances, unsigned int *count) {
    return inst->createGrassSplat(x, z, lod, ps, qs, instances, count);
}
EMSCRIPTEN_KEEPALIVE void createVegetationSplat(DCInstance *inst, float x, float z, int lod, float *ps, float *qs, float *instances, unsigned int *count) {
    return inst->createVegetationSplat(x, z, lod, ps, qs, instances, count);
}
EMSCRIPTEN_KEEPALIVE void createMobSplat(DCInstance *inst, float x, float z, int lod, float *ps, float *qs, float *instances, unsigned int *count) {
    return inst->createMobSplat(x, z, lod, ps, qs, instances, count);
}

/* EMSCRIPTEN_KEEPALIVE void clearChunkRootDualContouring(float x, float y, float z) {
    return DualContouring::clearChunkRoot(x, y, z);
} */

EMSCRIPTEN_KEEPALIVE uint8_t *createChunkMeshDualContouring(DCInstance *inst, float x, float y, float z, int *lodArray) {
    return inst->createChunkMesh(x, y, z, lodArray);
}

EMSCRIPTEN_KEEPALIVE bool drawSphereDamage(DCInstance *inst, float x, float y, float z, float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages) {
    return inst->drawSphereDamage(x, y, z, radius, outPositions, outPositionsCount, outDamages, 1);
}

EMSCRIPTEN_KEEPALIVE bool eraseSphereDamage(DCInstance *inst, float x, float y, float z, float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages) {
    return inst->eraseSphereDamage(x, y, z, radius, outPositions, outPositionsCount, outDamages, 1);
}

EMSCRIPTEN_KEEPALIVE bool drawCubeDamage(
    DCInstance *inst,
    float x, float y, float z,
    float qx, float qy, float qz, float qw,
    float sx, float sy, float sz,
    float *outPositions,
    unsigned int *outPositionsCount,
    float *outDamages
) {
    return inst->drawCubeDamage(
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
    DCInstance *inst,
    float x, float y, float z,
    float qx, float qy, float qz, float qw,
    float sx, float sy, float sz,
    float *outPositions,
    unsigned int *outPositionsCount,
    float *outDamages
) {
    return inst->eraseCubeDamage(
        x, y, z,
        qx, qy, qz, qw,
        sx, sy, sz,
        outPositions,
        outPositionsCount,
        outDamages,
        1
    );
}

EMSCRIPTEN_KEEPALIVE void injectDamage(DCInstance *inst, float x, float y, float z, float *damageBuffer) {
    inst->injectDamage(x, y, z, damageBuffer, 1);
}

EMSCRIPTEN_KEEPALIVE void setRange(DCInstance *inst, int minX, int minY, int minZ, int maxX, int maxY, int maxZ) {
    inst->setRange(vm::ivec3(minX, minY, minZ), vm::ivec3(maxX, maxY, maxZ));
}

EMSCRIPTEN_KEEPALIVE void *doMalloc(size_t size) {
    return malloc(size);
}

EMSCRIPTEN_KEEPALIVE void doFree(void *ptr) {
    free(ptr);
}

} // extern "C"