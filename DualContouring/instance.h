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
#include "damage.h"
// #include "octree.h"
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

    // 2d caches

    static NoiseField initNoiseField(DCInstance *inst, int x, int y);
    static uint8_t initBiomesField(DCInstance *inst, int x, int y);
    static Heightfield initHeightField(DCInstance *inst, int x, int y);
    static float initWaterField(DCInstance *inst, int x, int y);

    ChunkCache2D<NoiseField, Chunk2D, initNoiseField> cachedNoiseField;
    ChunkCache2D<uint8_t, Chunk2D, initBiomesField> cachedBiomesField;
    ChunkCache2D<Heightfield, Chunk2D, initHeightField> cachedHeightField;
    ChunkCache2D<float, Chunk2D, initWaterField> cachedWaterField;

    // 3d caches

    static uint8_t initSkylightField(DCInstance *inst, int x, int y, int z);
    static uint8_t initAoField(DCInstance *inst, int x, int y, int z);
    static float initCaveField(DCInstance *inst, int x, int y, int z);
    static float initSdf(DCInstance *inst, int x, int y, int z);
    static float initWaterSdf(DCInstance *inst, int x, int y, int z);
    // static float initDamageSdf(DCInstance *inst, int x, int y, int z);

    ChunkCache3D<uint8_t, Chunk3D, initSkylightField> cachedSkylightField;
    ChunkCache3D<uint8_t, Chunk3D, initAoField> cachedAoField;
    ChunkCache3D<float, Chunk3D, initCaveField> cachedCaveField;
    ChunkCache3D<float, Chunk3D, initSdf> cachedSdf;
    ChunkCache3D<float, Chunk3D, initWaterSdf> cachedWaterSdf;
    // ChunkCache3D<float, Chunk3D, initDamageSdf> cachedDamageSdf;

    // 2d interpolation
    // unsigned char getCachedBiome(const int lx, const int lz) const;
    void getCachedInterpolatedBiome2D(const vm::vec2 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights);
    void getCachedInterpolatedBiome3D(const vm::vec3 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights);

    // 3d interpolation
    void getCachedHeightfield(float *heights);
    void getCachedSkylight(unsigned char *skylights);
    void getCachedAo(unsigned char *aos);
    float getCachedInterpolatedSdf(const float x, const float y, const float z, const int lod);
    float getCachedWaterInterpolatedSdf(const float x, const float y, const float z, const int lod);
    float getCachedDamageInterpolatedSdf(const float &x, const float &y, const float &z, const int &lod);

    // damage buffers
    DamageBuffers damageBuffers;

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

    Mutex *getChunkLock(const vm::ivec2 &worldPos, const int lod, const int flags);
    Mutex *getChunkLock(const vm::ivec3 &worldPos, const int lod);

    //

    float *getChunkHeightfield(const vm::ivec2 &worldPositionXZ, int lod);
    unsigned char *getChunkSkylight(const vm::ivec3 &worldPosition, int lod);
    unsigned char *getChunkAo(const vm::ivec3 &worldPosition, int lod);
    
    //
    
    /* void getHeightfieldRange(int x, int z, int w, int h, int lod, float *heights);
    void getSkylightFieldRange(int x, int y, int z, int w, int h, int d, int lod, unsigned char *aos);
    void getAoFieldRange(int x, int y, int z, int w, int h, int d, int lod, unsigned char *aos); */
    
    //

    uint8_t *createGrassSplat(const vm::ivec2 &worldPositionXZ, const int lod);
    uint8_t *createVegetationSplat(const vm::ivec2 &worldPositionXZ, const int lod);
    uint8_t *createMobSplat(const vm::ivec2 &worldPositionXZ, const int lod);
    
    //
    
    // void clearChunkRoot(float x, float y, float z);
    uint8_t *createTerrainChunkMesh(const vm::ivec3 &worldPosition, const int lodArray[8]);
    uint8_t *createLiquidChunkMesh(const vm::ivec3 &worldPosition, const int lodArray[8]);

    //
    bool bakeSphereDamage(std::vector<float> &bakedDamage, const vm::vec3 &worldPos,const vm::ivec3 &min, const float radius);

    bool drawSphereDamage(const float &x, const float &y, const float &z,
                          const float &radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages,
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

    // void injectDamage(const float &x, const float &y, const float &z, float *damageBuffer, const int &lod);
    
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
    bool tryLock(const PositionType &chunkPosition, int lod, GenerateFlags flags) {
        Mutex *chunkLock = getChunkLock(chunkPosition, lod, flags);
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

    uint32_t getChunkHeightfieldAsync(const vm::ivec2 &worldPositionXZ, int lod);
    uint32_t getChunkSkylightAsync(const vm::ivec3 &worldPosition, int lod);
    uint32_t getChunkAoAsync(const vm::ivec3 &worldPosition, int lod);

    uint32_t createGrassSplatAsync(const vm::ivec2 &worldPositionXZ, const int lod);
    uint32_t createVegetationSplatAsync(const vm::ivec2 &worldPositionXZ, const int lod);
    uint32_t createMobSplatAsync(const vm::ivec2 &worldPositionXZ, const int lod);

    //

    // std::vector<Promise *> ensureChunks2D(const vm::ivec2 &position2D, int minChunkDelta, int maxChunkDelta, int lod, GenerateFlags flags);
    // void ensureChunk(const vm::ivec2 &position2D, int lod, GenerateFlags flags);
    // void ensureChunk(const vm::ivec3 &position2D, int lod, GenerateFlags flags);

    //

    float signedDistanceToBox(float sx, float sy, float sz, float px, float py, float pz);
    float signedDistanceToSphere(float cx, float cy, float cz, float r, float px, float py, float pz);
    void patchFrontier(DCInstance *inst, std::unordered_map<uint64_t, bool> &erased);

    // bool bakeSphereDamage(const float &x, const float &y, const float &z, const float radius);
    bool removeSphereDamage(const float &x, const float &y, const float &z, const float radius);
    bool addCubeDamage(
        const float &x, const float &y, const float &z,
        const float &qx, const float &qy, const float &qz, const float &qw,
        const float &sx, const float &sy, const float &sz
    );
    bool removeCubeDamage(
        const float &x, const float &y, const float &z,
        const float &qx, const float &qy, const float &qz, const float &qw,
        const float &sx, const float &sy, const float &sz
    );
};

#endif // _INSTANCE_H_
