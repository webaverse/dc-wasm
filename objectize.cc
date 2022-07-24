#include <emscripten.h>
#include "DualContouring/tracker.h"
#include "DualContouring/main.h"

extern "C" {

EMSCRIPTEN_KEEPALIVE void initialize(int chunkSize, int seed) {
    DualContouring::initialize(chunkSize, seed);
}

// 

EMSCRIPTEN_KEEPALIVE DCInstance *createInstance() {
    return DualContouring::createInstance();
}
EMSCRIPTEN_KEEPALIVE void destroyInstance(DCInstance *instance) {
    DualContouring::destroyInstance(instance);
}

//

EMSCRIPTEN_KEEPALIVE void getHeightfieldRangeAsync(DCInstance *inst, uint32_t taskId, int x, int z, int w, int h, int lod, float *heights, int priority) {
    vm::ivec2 worldPositionXZ{x, z};
    vm::ivec2 sizeXZ{w, h};
    inst->getHeightfieldRangeAsync(taskId, worldPositionXZ, sizeXZ, lod, heights, priority);
}
EMSCRIPTEN_KEEPALIVE void getLightRangeAsync(DCInstance *inst, uint32_t taskId, int x, int y, int z, int w, int h, int d, int lod, uint8_t *skylights, uint8_t *aos, int priority) {
    vm::ivec3 worldPosition{x, y, z};
    vm::ivec3 size{w, h, d};
    inst->getLightRangeAsync(taskId, worldPosition, size, lod, skylights, aos, priority);
}

//

EMSCRIPTEN_KEEPALIVE void getChunkHeightfieldAsync(DCInstance *inst, uint32_t taskId, int x, int z, int lod, int priority) {
    inst->getChunkHeightfieldAsync(taskId, vm::ivec2{x, z}, lod, priority);
}
EMSCRIPTEN_KEEPALIVE void getChunkSkylightAsync(DCInstance *inst, uint32_t taskId, int x, int y, int z, int lod, int priority) {
    inst->getChunkSkylightAsync(taskId, vm::ivec3{x, y, z}, lod, priority);
}
EMSCRIPTEN_KEEPALIVE void getChunkAoAsync(DCInstance *inst, uint32_t taskId, int x, int y, int z, int lod, int priority) {
    inst->getChunkAoAsync(taskId, vm::ivec3{x, y, z}, lod, priority);
}
/* EMSCRIPTEN_KEEPALIVE void getSkylightFieldRange(DCInstance *inst, int x, int y, int z, int w, int h, int d, int lod, unsigned char *skylights) {
    return inst->getSkylightFieldRange(x, y, z, w, h, d, lod, skylights);
}
EMSCRIPTEN_KEEPALIVE void getAoFieldRange(DCInstance *inst, int x, int y, int z, int w, int h, int d, int lod, unsigned char *aos) {
    return inst->getAoFieldRange(x, y, z, w, h, d, lod, aos);
} */
/* EMSCRIPTEN_KEEPALIVE void getBiomesContainedInChunk(DCInstance *inst, int x, int z, unsigned char *biomes, unsigned int *biomesCount) {
    return inst->getBiomesContainedInChunk(x, z, biomes, biomesCount, 1);        
} */

// 

EMSCRIPTEN_KEEPALIVE void createGrassSplatAsync(DCInstance *inst, uint32_t taskId, int x, int z, int lod, int priority) {
    inst->createGrassSplatAsync(taskId, vm::ivec2{x, z}, lod, priority);
}
EMSCRIPTEN_KEEPALIVE void createVegetationSplatAsync(DCInstance *inst, uint32_t taskId, int x, int z, int lod, int priority) {
    inst->createVegetationSplatAsync(taskId, vm::ivec2{x, z}, lod, priority);
}
EMSCRIPTEN_KEEPALIVE void createMobSplatAsync(DCInstance *inst, uint32_t taskId, int x, int z, int lod, int priority) {
    inst->createMobSplatAsync(taskId, vm::ivec2{x, z}, lod, priority);
}

/* EMSCRIPTEN_KEEPALIVE void clearChunkRootDualContouring(float x, float y, float z) {
    return DualContouring::clearChunkRoot(x, y, z);
} */

//

EMSCRIPTEN_KEEPALIVE void createTerrainChunkMeshAsync(DCInstance *inst, uint32_t taskId, int x, int y, int z, int *lodArray) {
    inst->createTerrainChunkMeshAsync(taskId, vm::ivec3{x, y, z}, lodArray);
}

EMSCRIPTEN_KEEPALIVE void createLiquidChunkMeshAsync(DCInstance *inst, uint32_t taskId, int x, int y, int z, int *lodArray) {
    inst->createLiquidChunkMeshAsync(taskId, vm::ivec3{x, y, z}, lodArray);
}

//

EMSCRIPTEN_KEEPALIVE bool drawSphereDamage(DCInstance *inst, float x, float y, float z, float radius, float *outPositions, unsigned int *outPositionsCount) {
    return inst->drawSphereDamage(x, y, z, radius, outPositions, outPositionsCount, 1);
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

/* EMSCRIPTEN_KEEPALIVE void injectDamage(DCInstance *inst, float x, float y, float z, float *damageBuffer) {
    inst->injectDamage(x, y, z, damageBuffer, 1);
} */

//

EMSCRIPTEN_KEEPALIVE void setClipRange(DCInstance *inst, float minX, float minY, float minZ, float maxX, float maxY, float maxZ) {
    inst->setClipRange(vm::vec3{minX, minY, minZ}, vm::vec3{maxX, maxY, maxZ});
}

EMSCRIPTEN_KEEPALIVE void cancelTask(DCInstance *inst, uint32_t taskId) {
    DualContouring::taskQueue.cancelTask(taskId);
    DualContouring::resultQueue.cancelPromise(taskId);
}

//

EMSCRIPTEN_KEEPALIVE void setCamera(DCInstance *inst, float *worldPosition, float *cameraPosition, float *cameraQuaternion, float *projectionMatrix) {
    vm::vec3 _worldPosition{
        worldPosition[0],
        worldPosition[1],
        worldPosition[2]
    };
    vm::vec3 _cameraPosition{
        cameraPosition[0],
        cameraPosition[1],
        cameraPosition[2]
    };
    Quat _cameraQuaternion{
        cameraQuaternion[0],
        cameraQuaternion[1],
        cameraQuaternion[2],
        cameraQuaternion[3]
    };
    std::array<float, 16> _projectionMatrix;
    memcpy(&_projectionMatrix[0], projectionMatrix, sizeof(_projectionMatrix));
    inst->setCamera(
        _worldPosition,
        _cameraPosition,
        _cameraQuaternion,
        _projectionMatrix
    );
}

//

EMSCRIPTEN_KEEPALIVE Tracker *createTracker(DCInstance *inst, int lod, int minLodRange, bool trackY) {
    Tracker *tracker = new Tracker(lod, minLodRange, trackY, inst);
    return tracker;
}

EMSCRIPTEN_KEEPALIVE void trackerUpdateAsync(DCInstance *inst, uint32_t taskId, Tracker *tracker, float *position, int priority) {
    vm::vec3 worldPosition{
        position[0],
        position[1],
        position[2]
    };
    inst->trackerUpdateAsync(taskId, tracker, worldPosition, priority);
}

EMSCRIPTEN_KEEPALIVE void destroyTracker(DCInstance *inst, Tracker *tracker) {
    delete tracker;
}

//

EMSCRIPTEN_KEEPALIVE void *doMalloc(size_t size) {
    return malloc(size);
}

EMSCRIPTEN_KEEPALIVE void doFree(void *ptr) {
    free(ptr);
}

//

EMSCRIPTEN_KEEPALIVE void runLoop() {
    DualContouring::runLoop();
}

//

int main() {
    /* EM_ASM({
        console.log('run main 1');
    }); */
    DualContouring::start();
    /* EM_ASM({
        console.log('run main 2');
    }); */
    // std::cout << "run " << emscripten_wasm_worker_self_id() << " " << std::endl;
    return 0;
}

} // extern "C"