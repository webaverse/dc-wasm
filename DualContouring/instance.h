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
// #include "result.h"
#include "../vector.h"

class DCInstance {
public:
    Mutex locksMutex;
    std::unordered_map<uint64_t, Mutex> chunkLocks2D;
    std::unordered_map<uint64_t, Mutex> chunkLocks3D;
    Mutex cachesMutex;
    // Mutex generateMutex;
    std::unordered_map<uint64_t, Chunk2D> chunksCache2D;
    std::unordered_map<uint64_t, Chunk3D> chunksCache3D;
    std::unique_ptr<vm::box3> clipRange;

    //

    DCInstance();
    ~DCInstance();

    //

    Chunk3D &getChunk(const vm::ivec3 &min, const int lod, GenerateFlags flags);
    Chunk3D &getChunkInternal(const vm::ivec3 &min, int lod);
    Chunk3D &getChunkAt(const float x, const float y, const float z, const int lod, GenerateFlags flags);

    Chunk2D &getChunk(const vm::ivec2 &min, const int lod, GenerateFlags flags);
    Chunk2D &getChunkInternal(const vm::ivec2 &min, int lod);
    Chunk2D &getChunkAt(const float x, const float z, const int lod, GenerateFlags flags);
    
    //

    Mutex *getChunkLock(const vm::ivec3 &worldPos, const int lod);
    Mutex *getChunkLock(const vm::ivec2 &worldPos, const int lod);

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
    uint8_t *createLiquidChunkMesh(const vm::ivec3 &worldPosition, const int lodArray[8]);

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
    // float getWater(const vm::vec2 &worldPosition, const int &lod);

    //
    
    template<typename PositionType>
    bool tryLock(const PositionType &chunkPosition, int lod) {
        Mutex *chunkLock = getChunkLock(chunkPosition, lod);
        return chunkLock->try_lock();
    }
    template<typename PositionType>
    void unlock(const PositionType &chunkPosition, int lod) {
        Mutex *chunkLock = getChunkLock(chunkPosition, lod);
        chunkLock->unlock();
    }

    //

    uint32_t createTerrainChunkMeshAsync(const vm::ivec3 &worldPosition, const int lodArray[8]);
    uint32_t createLiquidChunkMeshAsync(const vm::ivec3 &worldPosition, const int lodArray[8]);

    uint32_t getChunkHeightfieldAsync(const vm::ivec2 &worldPositionXZ, int lod, float *heights);
    uint32_t getChunkSkylightAsync(const vm::ivec3 &worldPosition, int lod, unsigned char *skylights);
    uint32_t getChunkAoAsync(const vm::ivec3 &worldPosition, int lod, unsigned char *aos);

    uint32_t createGrassSplatAsync(const vm::ivec2 &worldPositionXZ, const int lod, float *ps, float *qs, float *instances, unsigned int *count);
    uint32_t createVegetationSplatAsync(const vm::ivec2 &worldPositionXZ, const int lod, float *ps, float *qs, float *instances, unsigned int *count);
    uint32_t createMobSplatAsync(const vm::ivec2 &worldPositionXZ, const int lod, float *ps, float *qs, float *instances, unsigned int *count);

    //

    std::vector<Promise *> ensureChunks2D(const vm::ivec2 &position2D, int minChunkDelta, int maxChunkDelta, int lod, GenerateFlags flags);
    void ensureChunk(const vm::ivec2 &position2D, int lod, GenerateFlags flags);
    void ensureChunk(const vm::ivec3 &position2D, int lod, GenerateFlags flags);
};

#endif // _INSTANCE_H_
