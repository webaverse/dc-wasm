#ifndef MAIN_H
#define MAIN_H
#include <iostream>
#include <vector>
#include <cstdint>
#include <ctime>
#include <string.h>
// #include "octree.h"
// #include "cachedNoise.h"
#include "FastNoise.h"
#include "vectorMath.h"
// #include "chunkDamageBuffer.h"

class ChunkDamageBuffer;

namespace DualContouring
{
    extern int chunkSize;
    extern FastNoise *fastNoise;

    // class Chunk
    // {
    // public:
    //     vec3 getMin();
    // };
    void initialize(int newChunkSize, int seed);
    float *getChunkHeightField(float x, float y, float z);
    void clearTemporaryChunkData();
    void clearChunkRoot(float x, float y, float z);
    // void generateChunkData(float x, float y, float z, const int lod);
    // void setChunkLod(float x, float y, float z, const int lod);
    uint8_t *createChunkMesh(float x, float y, float z, const int lod);
    ChunkDamageBuffer &getChunkDamageBuffer(vm::ivec3 min);
    bool drawDamageSphere(const float &x, const float &y, const float &z, const float radius, float *outPositions, unsigned int *outPositionsCount);
    bool eraseDamageSphere(const float &x, const float &y, const float &z, const float radius, float *outPositions, unsigned int *outPositionsCount);
    bool addCubeDamage(
        float x, float y, float z,
        float qx, float qy, float qz, float qw,
        float sx, float sy, float sz,
        float *outPositions,
        unsigned int *outPositionsCount
    );
    bool eraseCubeDamage(
        float x, float y, float z,
        float qx, float qy, float qz, float qw,
        float sx, float sy, float sz,
        float *outPositions,
        unsigned int *outPositionsCount
    );
};

#endif // MAIN_H
