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
#include "noises.h"

class ChunkDamageBuffer;

namespace DualContouring
{
    extern int chunkSize;
    extern Noises *noises;

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
    bool drawSphereDamage(const float &x, const float &y, const float &z, const float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages);
    bool eraseSphereDamage(const float &x, const float &y, const float &z, const float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages);
    bool drawCubeDamage(
        float x, float y, float z,
        float qx, float qy, float qz, float qw,
        float sx, float sy, float sz,
        float *outPositions,
        unsigned int *outPositionsCount,
        float *outDamages
    );
    bool eraseCubeDamage(
        float x, float y, float z,
        float qx, float qy, float qz, float qw,
        float sx, float sy, float sz,
        float *outPositions,
        unsigned int *outPositionsCount,
        float *outDamages
    );
    void injectDamage(const float &x, const float &y, const float &z, float *damageBuffer);
    unsigned char getBiome(int x, int z);
    float getBiomeHeight(unsigned char b, float x, float z);
};

#endif // MAIN_H
