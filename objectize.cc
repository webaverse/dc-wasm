#include <emscripten.h>
// #include "DualContouring/tracker.h"
#include "DualContouring/main.h"
#include "DualContouring/vectorMath.h"
#include "MC.h"
#include "VHACD.h"

extern "C" {

EMSCRIPTEN_KEEPALIVE DCInstance *createInstance(
    int chunkSize,
    int range,
    float fatness
) {
    return DualContouring::createInstance(
        chunkSize,
        range,
        fatness
    );
}
EMSCRIPTEN_KEEPALIVE void destroyInstance(DCInstance *instance) {
    DualContouring::destroyInstance(instance);
}

//

EMSCRIPTEN_KEEPALIVE uint8_t *createPointCloudMesh(DCInstance *instance, float *points, unsigned int pointsSize, unsigned int *resultSize) {
    unsigned int numPoints = pointsSize / 3;
    std::vector<vm::vec3> pointcloud(numPoints);
    for (int i = 0; i < numPoints; i++) {
        pointcloud[i] = vm::vec3{
            points[i * 3],
            points[i * 3 + 1],
            points[i * 3 + 2]
        };
    }
    uint8_t *result = instance->createPointCloudMesh(pointcloud, resultSize);
    return result;
}

//

EMSCRIPTEN_KEEPALIVE void *doMalloc(size_t size) {
    return malloc(size);
}

EMSCRIPTEN_KEEPALIVE void doFree(void *ptr) {
    free(ptr);
}

//

int main() {
    // DualContouring::start();
    return 0;
}

} // extern "C"