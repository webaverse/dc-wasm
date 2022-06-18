#ifndef MAIN_H
#define MAIN_H

#include <iostream>
#include <vector>
#include <cstdint>
#include <ctime>
#include <string.h>
#include <memory>
#include "instance.h"
#include "noises.h"
#include "vectorMath.h"

class Noises;
class DCInstance;

namespace DualContouring {
    // globals
    extern int chunkSize;
    extern Noises *noises;

    // initialization
    void initialize(int newChunkSize, int seed);
    
    // instances
    DCInstance *createInstance();
    void destroyInstance(DCInstance *instance);

    // output
    uint8_t *constructOutputBuffer(VertexBuffer &vertexBuffer);

    // biomes
    float getComputedBiomeHeight(unsigned char b, const vm::vec2 &worldPosition, const int &lod);
    
    // caves
    float getComputedCaveNoise(int ax, int ay, int az);
};

#endif // MAIN_H
