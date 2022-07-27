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

//

class Tracker;
class OctreeNode;
typedef std::shared_ptr<OctreeNode> OctreeNodePtr;

//

class PeekFaceIndices{
public:
  int array[8 * 8];
  PeekFaceIndices();
};

//

class DCInstance {
public:
    Mutex locksMutex;
    std::unordered_map<uint64_t, Mutex> chunkLocks2D;
    std::unordered_map<uint64_t, Mutex> chunkLocks3D;
    Mutex cachesMutex;
    
    std::unordered_map<uint64_t, Chunk2D> chunksCache2D;
    std::unordered_map<uint64_t, Chunk3D> chunksCache3D;
    std::unique_ptr<vm::box3> clipRange;

    vm::vec3 worldPosition;
    vm::vec3 cameraPosition;
    Quat cameraQuaternion;
    std::array<float, 16> projectionMatrix;

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

    // vertex interpolation
    void getCachedBiome2D(const vm::ivec2 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights, std::array<UV, 2> &biomeUvs1, std::array<UV, 2> &biomeUvs2);
    void getCachedInterpolatedBiome3D(const vm::vec3 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights, std::array<UV, 2> &biomeUvs1, std::array<UV, 2> &biomeUvs2);
    void getCachedInterpolatedLight(const vm::vec3 &worldPosition, uint8_t &skylight, uint8_t &ao);

    // chunk interpolation
    void getCachedHeightfield(float *heights);
    void getCachedSkylight(unsigned char *skylights);
    void getCachedAo(unsigned char *aos);

    float getCachedInterpolatedHeightfield(const vm::vec2 &worldPosition, const int lod);
    float getCachedInterpolatedSdf(const float x, const float y, const float z, const int lod);
    float getCachedWaterInterpolatedSdf(const float x, const float y, const float z, const int lod);
    float getCachedDamageInterpolatedSdf(const float &x, const float &y, const float &z, const int &lod);

    // damage buffers
    DamageBuffers damageBuffers;

    //

    DCInstance();
    ~DCInstance();

    //
    
    // peek buffer

    PeekFaceIndices PEEK_FACE_INDICES;

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

    void getHeightfieldRange(const vm::ivec2 &worldPositionXZ, const vm::ivec2 &size, int lod, float *heights);
    void getLightRange(const vm::ivec3 &worldPosition, const vm::ivec3 &size, int lod, uint8_t *skyLights, uint8_t *aos);

    //

    float *getChunkHeightfield(const vm::ivec2 &worldPositionXZ, int lod);
    unsigned char *getChunkSkylight(const vm::ivec3 &worldPosition, int lod);
    unsigned char *getChunkAo(const vm::ivec3 &worldPosition, int lod);
    
    //

    uint8_t *createGrassSplat(const vm::ivec2 &worldPositionXZ, const int lod);
    uint8_t *createVegetationSplat(const vm::ivec2 &worldPositionXZ, const int lod);
    uint8_t *createMobSplat(const vm::ivec2 &worldPositionXZ, const int lod);
    
    //
    
    // void clearChunkRoot(float x, float y, float z);
    uint8_t *createTerrainChunkMesh(const vm::ivec3 &worldPosition, const int lodArray[8]);
    uint8_t *createLiquidChunkMesh(const vm::ivec3 &worldPosition, const int lodArray[8]);

    //
    bool drawSphereDamage(const float &x, const float &y, const float &z,
                          const float &radius, float *outPositions, unsigned int *outPositionsCount,
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

    void setCamera(const vm::vec3 &worldPosition, const vm::vec3 &cameraPosition, const Quat &cameraQuaternion, const std::array<float, 16> &projectionMatrix);
    void setClipRange(const vm::vec3 &min, const vm::vec3 &max);

    //

    // unsigned char getBiome(const vm::vec2 &worldPosition, const int &lod);
    
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

    void createTerrainChunkMeshAsync(uint32_t id, const vm::ivec3 &worldPosition, const int lodArray[8]);
    void createLiquidChunkMeshAsync(uint32_t id, const vm::ivec3 &worldPosition, const int lodArray[8]);

    //

    void getHeightfieldRangeAsync(uint32_t id, const vm::ivec2 &worldPositionXZ, const vm::ivec2 &sizeXZ, int lod, float *heights, int priority);
    void getLightRangeAsync(uint32_t id, const vm::ivec3 &worldPosition, const vm::ivec3 &size, int lod, uint8_t *skylights, uint8_t *aos, int priority);

    //

    /* void getChunkHeightfieldAsync(uint32_t id, const vm::ivec2 &worldPositionXZ, int lod, int priority);
    void getChunkSkylightAsync(uint32_t id, const vm::ivec3 &worldPosition, int lod, int priority);
    void getChunkAoAsync(uint32_t id, const vm::ivec3 &worldPosition, int lod, int priority); */

    void createGrassSplatAsync(uint32_t id, const vm::ivec2 &worldPositionXZ, const int lod, const int priority);
    void createVegetationSplatAsync(uint32_t id, const vm::ivec2 &worldPositionXZ, const int lod, const int priority);
    void createMobSplatAsync(uint32_t id, const vm::ivec2 &worldPositionXZ, const int lod, const int priority);

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
    
    //
    
    void trackerUpdateAsync(uint32_t id, Tracker *tracker, const vm::vec3 &position, const std::vector<OctreeNodePtr> &renderChunks, int priority);
};

#endif // _INSTANCE_H_
