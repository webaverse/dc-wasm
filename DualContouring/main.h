#ifndef MAIN_H
#define MAIN_H
#include <iostream>
#include <vector>
#include <cstdint>
#include <ctime>
#include <string.h>
#include <memory>
// #include "octree.h"
// #include "cachedNoise.h"
#include "../FastNoise.h"
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
    // float *getChunkHeightField(float x, float y, float z);
    // float getHeight(float x, float z, const int &lod);
    // void getHeights(float *vec2s, int count, float *heights, const int &lod);
    void getChunkSkylight(int x, int y, int z, int lod, unsigned char *skylights);
    void getChunkAo(int x, int y, int z, int lod, unsigned char *ao);
    void getHeightfieldRange(int x, int z, int w, int h, int lod, float *heights);
    void getSkylightFieldRange(int x, int y, int z, int w, int h, int d, int lod, unsigned char *aos);
    void getAoFieldRange(int x, int y, int z, int w, int h, int d, int lod, unsigned char *aos);
    void createGrassSplat(float x, float z, int lod, float *ps, float *qs, float *instances, unsigned int *count);
    void createVegetationSplat(float x, float z, int lod, float *ps, float *qs, float *instances, unsigned int *count);
    void createMobSplat(float x, float z, int lod, float *ps, float *qs, float *instances, unsigned int *count);
    void clearTemporaryChunkData();
    void clearChunkRoot(float x, float y, float z);
    uint8_t *createChunkMesh(float x, float y, float z, int lodArray[8]);
    ChunkDamageBuffer &getChunkDamageBuffer(vm::ivec3 min);
    bool drawSphereDamage(const float &x, const float &y, const float &z,
                          const float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages,
                          const int &lod);
    bool eraseSphereDamage(const float &x, const float &y, const float &z,
                           const float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages,
                           const int &lod);
    bool drawCubeDamage(
        float x, float y, float z,
        float qx, float qy, float qz, float qw,
        float sx, float sy, float sz,
        float *outPositions,
        unsigned int *outPositionsCount,
        float *outDamages,
        const int &lod

    );
    bool eraseCubeDamage(
        float x, float y, float z,
        float qx, float qy, float qz, float qw,
        float sx, float sy, float sz,
        float *outPositions,
        unsigned int *outPositionsCount,
        float *outDamages,
        const int &lod

    );
    void injectDamage(const float &x, const float &y, const float &z, float *damageBuffer, const int &lod);

    unsigned char getBiome(const vm::ivec2 &worldPosition, const int &lod);
    float getComputedBiomeHeight(unsigned char b, const vm::vec2 &worldPosition, const int &lod);
    // void getBiomesContainedInChunk(int x, int z, unsigned char *biomes, unsigned int *biomesCount, const int &lod);
};

#endif // MAIN_H
