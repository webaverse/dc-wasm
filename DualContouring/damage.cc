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

bool ChunkDamageBuffer::bakeSphereDamage(std::vector<float> &bakedDamage, const vm::vec3 &worldPos,const vm::ivec3 &min, const float radius)
{
    bool drew = false;

    for (int ly = 0; ly < chunkSize; ly++)
        for (int lz = 0; lz < chunkSize; lz++)
            for (int lx = 0; lx < chunkSize; lx++)
            {
                int ax = min.x + lx;
                int ay = min.y + ly;
                int az = min.z + lz;
                float newDistance = signedDistanceToSphere(worldPos.x, worldPos.y, worldPos.z, radius, ax, ay, az);
                int index = lx + lz * chunkSize + ly * chunkSize * chunkSize;
                float oldDistance = bakedDamage[index];
                if (newDistance < oldDistance)
                {
                    bakedDamage[index] = newDistance;
                    drew = true;
                }
            }

    return drew;
}

bool ChunkDamageBuffer::drawSphereDamage(const vm::vec3 &worldPos, const float &radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages){
    unsigned int maxPositionsCount = *outPositionsCount;
    *outPositionsCount = 0;

    bool drew = false;
    std::set<uint64_t> seenHashes;

    // chunk min of the hit point
    vm::ivec3 chunkMin = chunkMinForPosition(vm::ivec3{(int)worldPos.x, (int)worldPos.y, (int)worldPos.z});

    for (float dx = -1; dx <= 1; dx += 1)
    {
        for (float dz = -1; dz <= 1; dz += 1)
        {
            for (float dy = -1; dy <= 1; dy += 1)
            {
                vm::ivec3 min = chunkMin + vm::ivec3{(int)dx, (int)dy, (int)dz} * chunkSize;

                if (bakeSphereDamage(bakedDamage, worldPos, min, radius))
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
    return drew;
}

bool DamageBuffers::damage(const vm::vec3 &worldPos, const float &radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages)
{
    bool drew = false;
    uint64_t chunkHash = hashOctreeMinLod(vm::ivec3{(int)worldPos.x, (int)worldPos.y, (int)worldPos.z}, 1);
    DamageBuffersList chunkRefsCopy;
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
        std::cout << "DAMAGED : Created" << std::endl;
        drew = editedChunkCopy->drawSphereDamage(worldPos, radius, outPositions, outPositionsCount, outDamages);
        chunkRefsCopy[chunkHash] = editedChunkCopy;
    }
    else
    {
        // chunk damage buffer already exists
        // modify damage
        std::cout << "DAMAGED : Modified" << std::endl;
        std::shared_ptr<ChunkDamageBuffer> editedChunkCopy = iter->second;
        drew = editedChunkCopy->drawSphereDamage(worldPos, radius, outPositions, outPositionsCount, outDamages);
    }

    {
        std::unique_lock<Mutex> lock(mutex);
        chunks = chunkRefsCopy;
    }
    return drew;
}