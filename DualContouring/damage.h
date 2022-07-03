#ifndef DAMAGE_H
#define DAMAGE_H

#include "constants.h"
#include <vector>
#include "sync.h"
#include "cache.h"
#include <unordered_map>

class ChunkDamageBuffer
{
public:
    // members
    std::vector<float> bakedDamage;

    // constructors
    ChunkDamageBuffer()
    {
        bakedDamage.resize(chunkSize * chunkSize * chunkSize, chunkSize);
    }
    bool bakeSphereDamage(const vm::vec3 &worldPos,const vm::ivec3 &min, const float radius);
    bool drawSphereDamage(const vm::vec3 &worldPos,
                          const float &radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages);
};

typedef std::unordered_map<uint64_t, std::shared_ptr<ChunkDamageBuffer>> DamageBuffersList;

class DamageBuffers
{
public:
    Mutex mutex;
    DamageBuffersList chunks;
    DamageBuffers(){};
    DamageBuffers(DamageBuffers &other)
    {
        std::unique_lock<Mutex> lock(other.mutex);
        chunks = other.chunks;
    }
    bool damage(const vm::vec3 &worldPos, const float &radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages, const int &lod);
};

#endif // DAMAGE_H
