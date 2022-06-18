#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#include <iostream>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <ctime>
#include <string.h>
#include <memory>
#include "chunk.h"
#include "octree.h"

class DCInstance {
public:
    std::unordered_map<uint64_t, Chunk> chunksNoiseHashMap;

    DCInstance();
    ~DCInstance();

    // chunks
    Chunk &getChunk(const vm::ivec3 &min, GenerateFlags flags, const int &lod);
    Chunk &getChunkAt(const float x, const float y, const float z, GenerateFlags flags, const int &lod);
    Chunk &getChunkAt(const float x, const float z, GenerateFlags flags, const int &lod);

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
};

#endif // _INSTANCE_H_
