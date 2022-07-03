#ifndef DAMAGE_H
#define DAMAGE_H

#include "constants.h"
#include <iostream>
#include <vector>
#include <array>
#include "sync.h"
#include "cache.h"
#include <unordered_map>

constexpr int bakedDamageSize = chunkSize * chunkSize * chunkSize;

class DamageSdf
{
public:
    std::array<float, bakedDamageSize> baked;
    vm::ivec3 min;

    DamageSdf(const vm::ivec3 &min) : min(min)
    {
        baked.fill(chunkSize); // initial value
    }
    DamageSdf(const DamageSdf &other) : min(other.min)
    {
        baked = other.baked;
    }

    float get(const int &x, const int &y, const int &z)
    {
        const vm::ivec3 localPos = vm::ivec3{x, y, z} - min;
        int index = localPos.x + localPos.y * chunkSize + localPos.z * chunkSize * chunkSize;
        return baked[index];
    }
    void set(const int &x, const int &y, const int &z, const float &value)
    {
        const vm::ivec3 localPos = vm::ivec3{x, y, z} - min;
        int index = localPos.x + localPos.y * chunkSize + localPos.z * chunkSize * chunkSize;
        baked[index] = value;
    }
};

class ChunkDamageBuffer
{
public:
    // members
    DamageSdf bakedDamageSdf;

    // constructors
    ChunkDamageBuffer(const vm::ivec3 &min) : bakedDamageSdf(min){};
    ChunkDamageBuffer(ChunkDamageBuffer &other) : bakedDamageSdf(other.bakedDamageSdf){};
    bool bakeSphereDamage(const vm::vec3 &worldPos, const vm::ivec3 &min, const float radius);
    bool drawSphereDamage(const vm::vec3 &worldPos, const vm::ivec3 &min,
                          const float &radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages);
};

typedef std::unordered_map<uint64_t, std::shared_ptr<ChunkDamageBuffer>> DamageBuffersMap;

class DamageBuffers
{
public:
    Mutex mutex;
    DamageBuffersMap chunks;
    DamageBuffers(){};
    DamageBuffers(DamageBuffers &other)
    {
        std::unique_lock<Mutex> lock(other.mutex);
        chunks = other.chunks;
    }
    bool damage(const vm::vec3 &worldPos, const float &radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages, const int &lod);
};

#endif // DAMAGE_H
