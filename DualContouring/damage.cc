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

void ChunkDamageBuffer::drawSphereDamage(bool &drew, const vm::vec3 &worldPos, const vm::ivec3 &min, const float &radius, float *outPositions, unsigned int *outPositionsCount)
{
    if (bakeSphereDamage(worldPos, min, radius))
    {
        outPositions[(*outPositionsCount) * 3] = min.x;
        outPositions[(*outPositionsCount) * 3 + 1] = min.y;
        outPositions[(*outPositionsCount) * 3 + 2] = min.z;
        (*outPositionsCount)++;

        drew = true;
    }
}

bool DamageBuffers::damage(const vm::vec3 &worldPos, const float &radius, float *outPositions, unsigned int *outPositionsCount, const int &lod)
{
    DamageBuffersMap chunkRefsCopy;
    {
        std::unique_lock<Mutex> lock(mutex);
        chunkRefsCopy = chunks;
    }

    *outPositionsCount = 0;
    bool drew = false;

    const vm::ivec3 hitPosition = vm::ivec3{(int)worldPos.x, (int)worldPos.y, (int)worldPos.z};
    const float diameter = radius * 2.f;

    // chunk min of the hit point
    vm::ivec3 chunkMin = chunkMinForPosition(hitPosition, lod);
    const vm::vec3 sphereBoundingBoxMin = worldPos - radius;
    const vm::vec3 sphereBoundingBoxMax = sphereBoundingBoxMin + vm::length(vm::vec2{diameter, diameter});

    std::set<uint64_t> seenHashes;

    for (float dz = sphereBoundingBoxMin.z; dz <= sphereBoundingBoxMax.z; dz += diameter)
        for (float dy = sphereBoundingBoxMin.y; dy <= sphereBoundingBoxMax.y; dy += diameter)
            for (float dx = sphereBoundingBoxMin.x; dx <= sphereBoundingBoxMax.x; dx += diameter)
            {
                vm::ivec3 min = chunkMinForPosition(vm::vec3{dx, dy, dz}, lod);
                addChunkDamageBuffer(chunkRefsCopy, min, seenHashes, drew, worldPos, radius, outPositions, outPositionsCount, lod);

                // increase the scan range to account for possible seam geo change in other chunks
                vm::vec3 localCornerPos = vm::vec3{dx, dy, dz} - vm::vec3{(float)min.x, (float)min.y, (float)min.z};
                if(localCornerPos.x < lod){
                    const float shiftedX = localCornerPos.x - lod;
                    const vm::vec3 shiftedWorldPos = vm::vec3{shiftedX, localCornerPos.y, localCornerPos.z} + vm::vec3{(float)min.x, (float)min.y, (float)min.z};
                    const vm::ivec3 damagedSeamChunkMin = chunkMinForPosition(shiftedWorldPos, lod);
                    addChunkDamageBuffer(chunkRefsCopy, damagedSeamChunkMin, seenHashes, drew, worldPos, radius, outPositions, outPositionsCount, lod);
                }
                if(localCornerPos.y < lod){
                    const float shiftedY = localCornerPos.y - lod;
                    const vm::vec3 shiftedWorldPos = vm::vec3{localCornerPos.x, shiftedY, localCornerPos.z} + vm::vec3{(float)min.x, (float)min.y, (float)min.z};
                    const vm::ivec3 damagedSeamChunkMin = chunkMinForPosition(shiftedWorldPos, lod);
                    addChunkDamageBuffer(chunkRefsCopy, damagedSeamChunkMin, seenHashes, drew, worldPos, radius, outPositions, outPositionsCount, lod);
                }
                if(localCornerPos.z < lod){
                    const float shiftedZ = localCornerPos.z - lod;
                    const vm::vec3 shiftedWorldPos = vm::vec3{localCornerPos.x, localCornerPos.y, shiftedZ} + vm::vec3{(float)min.x, (float)min.y, (float)min.z};
                    const vm::ivec3 damagedSeamChunkMin = chunkMinForPosition(shiftedWorldPos, lod);
                    addChunkDamageBuffer(chunkRefsCopy, damagedSeamChunkMin, seenHashes, drew, worldPos, radius, outPositions, outPositionsCount, lod);
                }
            }

    {
        std::unique_lock<Mutex> lock(mutex);
        chunks = chunkRefsCopy;
    }
    return drew;
}

void DamageBuffers::addChunkDamageBuffer(DamageBuffersMap &chunkRefsCopy, const vm::ivec3 &chunkMin,std::set<uint64_t> &seenHashes, bool &drew, const vm::vec3 &worldPos, const float &radius, float *outPositions, unsigned int *outPositionsCount, const int &lod)
{
    uint64_t minHash = hashOctreeMin(chunkMin);
    if (seenHashes.find(minHash) == seenHashes.end())
    {
        seenHashes.insert(minHash);
        auto iter = chunkRefsCopy.find(minHash);
        if (iter == end(chunkRefsCopy))
        {
            // chunk damage buffer doesn't exist already so create a new one
            std::shared_ptr<ChunkDamageBuffer> editedChunkCopy = std::make_shared<ChunkDamageBuffer>(chunkMin);
            // modify damage
            // std::cout << "DAMAGED : Created" << std::endl;
            editedChunkCopy->drawSphereDamage(drew, worldPos, chunkMin, radius, outPositions, outPositionsCount);
            chunkRefsCopy[minHash] = editedChunkCopy;
        }
        else
        {
            // chunk damage buffer already exists so replace it with a new one
            // modify damage
            // std::cout << "DAMAGED : Modified" << std::endl;
            std::shared_ptr<ChunkDamageBuffer> editedChunkCopy = std::make_shared<ChunkDamageBuffer>(*iter->second);
            editedChunkCopy->drawSphereDamage(drew, worldPos, chunkMin, radius, outPositions, outPositionsCount);
            chunkRefsCopy[minHash] = editedChunkCopy;
        }
    }
}