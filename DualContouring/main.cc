#include "main.h"
#include "octree.h"
#include "instance.h"
#include "noises.h"
// #include "result.h"
#include "../worley.h"
#include <pthread.h>
#include <thread>

// namespace ChunkMesh
// {
//     OctreeNode *chunkRoot;
//     OctreeNode *chunkWithLod;
//     std::vector<OctreeNode *> neighbouringChunks;
//     OctreeNode *seamRoot;
// };

namespace DualContouring
{
    /* int getGridPoints(int chunkSize) {
       return chunkSize + 3 + 1;
    } */

    // chunk settings
    // int chunkSize = 16;
    // int gridPoints = getGridPoints(chunkSize);
    // int cacheWidth = chunkSize * CHUNK_CACHE_RANGE;

    Noises *noises = nullptr;
    TaskQueue taskQueue;
    ResultQueue resultQueue;

    pthread_t parentThreadId;
    // std::vector<emscripten_wasm_worker_t> threads;
    // bool first = true;

    // storing the octrees that we would delete after mesh construction
    // std::vector<OctreeNode *> neighbourNodes;

    // storing the octree roots here for search
    // std::unordered_map<uint64_t, OctreeNode *> chunksListHashMap;

    void start() {
        // threads.reserve(numThreads);
        /* EM_ASM({
            console.log('main thread', $0, $1);
        }, pthread_self(), numThreads); */
        for (int i = 0; i < numThreads; i++) {
            // std::cout << "create thread" << std::endl;
            std::thread([]() -> void {
                /* EM_ASM({
                    console.log('worker thread', $0);
                }, pthread_self()); */
                runLoop();
            }).detach();
            /* threads.push_back(emscripten_malloc_wasm_worker(stackSize));
            if (!threads[i]) {
            //   std::cout << "bad thread " << threads[i] << std::endl;
              abort();
            }
            emscripten_wasm_worker_post_function_v(threads[i], runLoop2); */
            /* std::thread thread([]() -> void {
                taskQueue.runLoop();
            });
            threads.push_back(std::move(thread)); */
        }
    }

    void runLoop() {
        /* if (first) {
            first = false;
            EM_ASM({
                debugger;
            });
        } */
        taskQueue.runLoop();
        // std::cout << "run loop 2" << std::endl;
    }

    void initialize(int newChunkSize, int seed)
    {
        // chunkSize = newChunkSize;
        // gridPoints = getGridPoints(chunkSize);
        // cacheWidth = chunkSize * CHUNK_CACHE_RANGE;
        noises = new Noises(seed);
        parentThreadId = pthread_self();
    }

    DCInstance *createInstance() {
        DCInstance *instance = new DCInstance();
        return instance;
    }
    void destroyInstance(DCInstance *instance) {
        delete instance;
    }

    // biomes
    float getComputedBiomeHeight(unsigned char b, const vm::vec2 &worldPosition) {
        const Biome &biome = BIOMES[b];
        float ax = worldPosition.x;
        float az = worldPosition.y;

        float biomeHeight = biome.baseHeight +
            DualContouring::noises->elevationNoise1.in2D(ax * biome.amps[0][0], az * biome.amps[0][0]) * biome.amps[0][1] +
            DualContouring::noises->elevationNoise2.in2D(ax * biome.amps[1][0], az * biome.amps[1][0]) * biome.amps[1][1] +
            DualContouring::noises->elevationNoise3.in2D(ax * biome.amps[2][0], az * biome.amps[2][0]) * biome.amps[2][1];
        return biomeHeight;
    }

    // caves
    float getComputedCaveNoise(int ax, int ay, int az) {
        std::vector<double> at = {
            (double)ax * 0.1f,
            (double)ay * 0.1f,
            (double)az * 0.1f
        };
        const size_t max_order = 3;
        std::vector<double> F;
        F.resize(max_order + 1);
        std::vector<dvec3> delta;
        delta.resize(max_order + 1);
        std::vector<uint32_t> ID;
        ID.resize(max_order + 1);
        // std::cout << "delete 0 " << delta.size() << std::endl;
        Worley(at, max_order, F, delta, ID);
        // std::cout << "delete 1" << std::endl;
        // delete[] F;
        // std::cout << "delete 2" << std::endl;
        // delete[] delta;
        // std::cout << "delete 3 " << delta[0].x << std::endl;

        vm::vec3 deltaPoint1(
            delta[0].x,
            delta[0].y,
            delta[0].z
        );
        float distance1 = length(deltaPoint1);
        
        // std::cout << "delete 3" << std::endl;

        vm::vec3 deltaPoint3(
            delta[2].x,
            delta[2].y,
            delta[2].z
        );
        // std::cout << "delete 4" << std::endl;
        float distance3 = length(deltaPoint3);
        float caveValue = std::min(std::max((distance3 != 0.f ? (distance1 / distance3) : 0.f) * 1.1f, 0.f), 1.f);
        // std::cout << "return" << std::endl;
        return caveValue;
    }
}