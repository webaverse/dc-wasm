#include "main.h"
#include "octree.h"
#include "instance.h"
// #include "noises.h"
// #include "result.h"
// #include "../worley.h"
#include <pthread.h>
#include <thread>

namespace DualContouring
{
    DCInstance *createInstance(int chunkSize, int range, float fatness) {
        DCInstance *instance = new DCInstance(chunkSize, range, fatness);
        return instance;
    }
    void destroyInstance(DCInstance *instance) {
        delete instance;
    }
}