#ifndef MAIN_H
#define MAIN_H
#include <iostream>
#include <vector>
#include <cstdint>
#include <ctime>
#include <string.h>
// #include "octree.h"
// #include "cachedNoise.h"
#include "vectorMath.h"
// #include "chunkDamageBuffer.h"

class ChunkDamageBuffer;

namespace DualContouring
{
    extern int chunkSize;
    // class Chunk
    // {
    // public:
    //     vec3 getMin();
    // };
    void setChunkSize(int newChunkSize);
    void clearTemporaryChunkData();
    void clearChunkRoot(float x, float y, float z);
    // void generateChunkData(float x, float y, float z, const int lod);
    // void setChunkLod(float x, float y, float z, const int lod);
    uint8_t *createChunkMesh(float x, float y, float z, const int lod);
    ChunkDamageBuffer &getChunkDamageBuffer(vm::ivec3 min);
    bool drawDamageSphere(const float &x, const float &y, const float &z, const float radius, const float value, float *outPositions, unsigned int *outPositionsCount);
};

#endif // MAIN_H
