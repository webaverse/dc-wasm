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
#include "context.h"
#include "task.h"
#include "result.h"
#include "lock.h"
#include "../vector.h"

class DCInstance {
public:
    TaskQueue taskQueue;
    ResultQueue resultQueue;
    std::unordered_map<uint64_t, std::mutex> chunkLocks;
    std::unordered_map<uint64_t, Chunk2D> chunksCache2D;
    std::unordered_map<uint64_t, Chunk3D> chunksCache3D;
    std::unique_ptr<vm::box3> clipRange;

    //

    DCInstance();
    ~DCInstance();

    //

    Chunk3D &getChunk(const vm::ivec3 &min, const int lod, GenerateFlags flags);
    Chunk3D &getChunkAt(const float x, const float y, const float z, const int lod, GenerateFlags flags);

    Chunk2D &getChunk(const vm::ivec2 &min, const int lod, GenerateFlags flags);
    Chunk2D &getChunkAt(const float x, const float z, const int lod, GenerateFlags flags);
    
    //

    std::mutex &getChunkLock(const vm::ivec3 &worldPos, const int lod);

    //

    void getChunkHeightfield(const vm::ivec2 &worldPositionXZ, int lod, float *heights);
    void getChunkSkylight(const vm::ivec3 &worldPosition, int lod, unsigned char *skylights);
    void getChunkAo(const vm::ivec3 &worldPosition, int lod, unsigned char *ao);
    
    //
    
    /* void getHeightfieldRange(int x, int z, int w, int h, int lod, float *heights);
    void getSkylightFieldRange(int x, int y, int z, int w, int h, int d, int lod, unsigned char *aos);
    void getAoFieldRange(int x, int y, int z, int w, int h, int d, int lod, unsigned char *aos); */
    
    //

    void createGrassSplat(const vm::ivec2 &worldPositionXZ, const int lod, float *ps, float *qs, float *instances, unsigned int *count);
    void createVegetationSplat(const vm::ivec2 &worldPositionXZ, const int lod, float *ps, float *qs, float *instances, unsigned int *count);
    void createMobSplat(const vm::ivec2 &worldPositionXZ, const int lod, float *ps, float *qs, float *instances, unsigned int *count);
    
    //
    
    // void clearChunkRoot(float x, float y, float z);
    uint8_t *createTerrainChunkMesh(const vm::ivec3 &worldPosition, const int lodArray[8]);
    uint32_t createTerrainChunkMeshAsync(const vm::ivec3 &worldPosition, const int lodArray[8]);
    uint8_t *createLiquidChunkMesh(const vm::ivec3 &worldPosition, const int lodArray[8]);
    uint32_t createLiquidChunkMeshAsync(const vm::ivec3 &worldPosition, const int lodArray[8]);

    //

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

    //

    void injectDamage(const float &x, const float &y, const float &z, float *damageBuffer, const int &lod);
    
    //

    void setClipRange(const vm::vec3 &min, const vm::vec3 &max);

    //

    unsigned char getBiome(const vm::vec2 &worldPosition, const int &lod);
    void getInterpolatedBiomes(const vm::vec2 &worldPosition, const int &lod, vm::ivec4 &biome, vm::vec4 &biomeWeights);
    
    //

    float getTemperature(const vm::vec2 &worldPosition, const int &lod);
    float getHumidity(const vm::vec2 &worldPosition, const int &lod);
    float getWater(const vm::vec2 &worldPosition, const int &lod);

    //

    bool tryLock(const std::vector<vm::ivec3> &chunkPositions, int lod);
    void unlock(const std::vector<vm::ivec3> &chunkPositions, int lod);
    bool tryLock(const vm::ivec3 &chunkPosition, int lod);
    void unlock(const vm::ivec3 &chunkPosition, int lod);
};

#endif // _INSTANCE_H_
