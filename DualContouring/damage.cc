#include "damage.h"
#include <memory.h>
#include "octree.h"

float signedDistanceToSphere(float cx, float cy, float cz, float r, float px, float py, float pz)
{
    float dx = px - cx;
    float dy = py - cy;
    float dz = pz - cz;
    float d = sqrt(dx * dx + dy * dy + dz * dz);
    return d - r;
}

bool ChunkDamageBuffer::bakeSphereDamage(const vm::vec3 &worldPos, const vm::ivec3 &min, const float radius)
{
    bool drew = false;

    for (int lz = 0; lz < chunkSize; lz++)
        for (int ly = 0; ly < chunkSize; ly++)
            for (int lx = 0; lx < chunkSize; lx++)
            {
                int ax = min.x + lx;
                int ay = min.y + ly;
                int az = min.z + lz;
                float newDistance = signedDistanceToSphere(worldPos.x, worldPos.y, worldPos.z, radius, ax, ay, az);
                float oldDistance = bakedDamageSdf.get(ax, ay, az);
                if (newDistance < oldDistance)
                {
                    bakedDamageSdf.set(ax, ay, az, newDistance);
                    drew = true;
                }
            }

    return drew;
}

bool ChunkDamageBuffer::drawSphereDamage(const vm::vec3 &worldPos, const vm::ivec3 &min, const float &radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages)
{
    unsigned int maxPositionsCount = *outPositionsCount;
    *outPositionsCount = 0;
    bool drew = false;

    if (bakeSphereDamage(worldPos, min, radius))
    {
        if (*outPositionsCount < maxPositionsCount)
        {
            // int damageBufferSize = chunkSize * chunkSize * chunkSize;
            // memcpy(outDamages + ((*outPositionsCount) * damageBufferSize), bakedDamageSdf.data(), sizeof(float) * damageBufferSize);

            outPositions[(*outPositionsCount) * 3] = min.x;
            outPositions[(*outPositionsCount) * 3 + 1] = min.y;
            outPositions[(*outPositionsCount) * 3 + 2] = min.z;

            (*outPositionsCount)++;
        }

        drew = true;
    }
    return drew;
}

bool DamageBuffers::damage(const vm::vec3 &worldPos, const float &radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages, const int &lod)
{
    bool drew = false;
    DamageBuffersMap chunkRefsCopy;
    {
        std::unique_lock<Mutex> lock(mutex);
        chunkRefsCopy = chunks;
    }

    const vm::ivec3 hitPosition = vm::ivec3{(int)worldPos.x, (int)worldPos.y, (int)worldPos.z};
    const float diameter = radius * 2;

    // chunk min of the hit point
    vm::ivec3 chunkMin = chunkMinForPosition(hitPosition);
    const vm::vec3 sphereBoundingBoxMin = worldPos - radius;
    const vm::vec3 sphereBoundingBoxMax = sphereBoundingBoxMin + diameter;
    std::set<uint64_t> seenHashes;
    for (float dz = sphereBoundingBoxMin.z; dz <= sphereBoundingBoxMax.z; dz += diameter)
        for (float dy = sphereBoundingBoxMin.y; dy <= sphereBoundingBoxMax.y; dy += diameter)
            for (float dx = sphereBoundingBoxMin.x; dx <= sphereBoundingBoxMax.x; dx += diameter)
            {
                vm::ivec3 min = chunkMinForPosition(vm::ivec3{(int)dx, (int)dy, (int)dz});
                uint64_t minHash = hashOctreeMin(min);
                if (seenHashes.find(minHash) == seenHashes.end())
                {
                    seenHashes.insert(minHash);
                    auto iter = chunkRefsCopy.find(minHash);
                    if (iter == end(chunkRefsCopy))
                    {
                        // chunk damage buffer doesn't exist already so create a new one
                        std::shared_ptr<ChunkDamageBuffer> editedChunkCopy = std::make_shared<ChunkDamageBuffer>(min);
                        // modify damage
                        // std::cout << "DAMAGED : Created" << std::endl;
                        drew = editedChunkCopy->drawSphereDamage(worldPos, min, radius, outPositions, outPositionsCount, outDamages);
                        chunkRefsCopy[minHash] = editedChunkCopy;
                    }
                    else
                    {
                        // chunk damage buffer already exists so replace it with a new one
                        // modify damage
                        // std::cout << "DAMAGED : Modified" << std::endl;
                        std::shared_ptr<ChunkDamageBuffer> editedChunkCopy = std::make_shared<ChunkDamageBuffer>(*iter->second);
                        drew = editedChunkCopy->drawSphereDamage(worldPos, min, radius, outPositions, outPositionsCount, outDamages);
                        chunkRefsCopy[minHash] = editedChunkCopy;
                    }
                }
            }

    {
        std::unique_lock<Mutex> lock(mutex);
        chunks = chunkRefsCopy;
    }
    return drew;
}