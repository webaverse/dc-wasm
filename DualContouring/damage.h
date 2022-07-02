#ifndef DAMAGE_H
#define DAMAGE_H

#include <memory.h>
#include <unordered_map>
#include "sync.h"
#include "vectorMath.h"
#include "octree.h"
#include "cache.h"

class ChunkDamageBuffer
{
    std::vector<float> bakedDamage;
    bool drawSphereDamage(const float &x, const float &y, const float &z,
                                  const float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages,
                                  const int &lod)
{
    unsigned int maxPositionsCount = *outPositionsCount;
    *outPositionsCount = 0;

    bool drew = false;
    std::set<uint64_t> seenHashes;

    // chunk min of the hit point
    vm::ivec3 chunkMin = chunkMinForPosition(vm::ivec3{(int)x, (int)y, (int)z});

    for (float dx = -1; dx <= 1; dx += 1)
    {
        for (float dz = -1; dz <= 1; dz += 1)
        {
            for (float dy = -1; dy <= 1; dy += 1)
            {
                vm::ivec3 min = chunkMin + vm::ivec3{(int)dx, (int)dy, (int)dz} * chunkSize;
                
                uint64_t minHash = hashOctreeMinLod(min, lod);
                if (seenHashes.find(minHash) == seenHashes.end())
                {
                    seenHashes.insert(minHash);

                    // Chunk3D &chunkNoise = getChunk(min, lod, GF_SDF);
                    if (addSphereDamage(x, y, z, radius))
                    {
                        if (*outPositionsCount < maxPositionsCount)
                        {
                            int damageBufferSize = chunkSize * chunkSize * chunkSize;
                            memcpy(outDamages + ((*outPositionsCount) * damageBufferSize), bakedDamage.data(), sizeof(float) * damageBufferSize);

                            outPositions[(*outPositionsCount) * 3] = min.x;
                            outPositions[(*outPositionsCount) * 3 + 1] = min.y;
                            outPositions[(*outPositionsCount) * 3 + 2] = min.z;

                            (*outPositionsCount)++;
                        }

                        drew = true;
                    }
                }
            }
        }
    }
    return drew;
}

}

class DamageBuffers
{
    Mutex mutex;
    std::unordered_map<uint64_t, ChunkDamageBuffer> chunks;
    DamageBuffers(DamageBuffers &other)
    {
        std::unique_lock<Mutex> lock(other.mutex);
        chunks = other.chunks;
    }
    void damage(const vm::ivec3 &worldPos)
    {
        uint64_t chunkHash = hashOctreeMinLod(worldPos, 1);
        std::unordered_map<uint64_t, std::shared_ptr<ChunkDamageBuffer>> chunkRefsCopy;
        {
            std::unique_lock<Mutex> lock(mutex);
            chunkRefsCopy = chunks;
        }
        auto iter = chunkRefsCopy.find(chunkHash);
        if (iter == end(chunkRefsCopy))
        {
            // chunk damage buffer doesn't exist already so create a new one
            std::shared_ptr<ChunkDamageBuffer> editedChunkCopy = std::make_shared<ChunkDamageBuffer>();
            // modify damage
            editedChunkCopy->drawSphereDamage();
            chunkRefsCopy[chunkHash] = editedChunkCopy;
        }else{
            // chunk damage buffer already exists
            // modify damage
            std::shared_ptr<ChunkDamageBuffer> editedChunkCopy = iter->second;
            editedChunkCopy->drawSphereDamage();
            chunkRefsCopy[chunkHash] = editedChunkCopy;
        }

        {
            std::unique_lock<Mutex> lock(mutex);
            chunks = chunkRefsCopy;
        }
    }
}

#endif // DAMAGE_H
