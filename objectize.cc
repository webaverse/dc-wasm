#include <emscripten.h>
// #include "DualContouring/tracker.h"
#include "DualContouring/main.h"

extern "C" {

EMSCRIPTEN_KEEPALIVE DCInstance *createInstance() {
    return DualContouring::createInstance();
}
EMSCRIPTEN_KEEPALIVE void destroyInstance(DCInstance *instance) {
    DualContouring::destroyInstance(instance);
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