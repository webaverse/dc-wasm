#ifndef MAIN_H
#define MAIN_H

#include <iostream>
#include <vector>
#include <cstdint>
#include <ctime>
#include <string.h>
#include <memory>
#include <pthread.h>
// #include <emscripten/wasm_worker.h>
#include "instance.h"
// #include "noises.h"
// #include "result.h"
#include "vectorMath.h"
#include "constants.h"

//

class Noises;
class DCInstance;

//

namespace DualContouring {
    // globals
    // extern int chunkSize;
    // extern int gridPoints;
    // extern int cacheWidth;
    // extern Noises *noises;
    // extern std::vector<emscripten_wasm_worker_t> threads;
    // extern TaskQueue taskQueue;
    // extern ResultQueue resultQueue;

    // extern pthread_t parentThreadId;
    // extern std::vector<emscripten_wasm_worker_t> threads;

    // initialization
    // void initialize(int newChunkSize, int seed);
    
    // instances
    DCInstance *createInstance(
        int chunkSize,
        int range,
        float fatness
    );
    void destroyInstance(DCInstance *instance);

    // threads
    // void start();
    // void runLoop();

    // biomes
    // float getComputedBiomeHeight(unsigned char b, const vm::vec2 &worldPosition);
    
    // caves
    // float getComputedCaveNoise(int ax, int ay, int az);
};

#endif // MAIN_H
