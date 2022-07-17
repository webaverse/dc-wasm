#include "instance.h"
#include "main.h"
#include "octree.h"
#include "lock.h"
#include "biomes.h"
#include "tracker.h"
#include "../vector.h"
#include "../util.h"
#include <emscripten.h>

constexpr int CHUNK_RANGE = 1;

// constructor/destructor
DCInstance::DCInstance() :
    cachedNoiseField(this),
    cachedBiomesField(this),
    cachedHeightField(this),
    cachedWaterField(this),
    cachedSkylightField(this),
    cachedAoField(this),
    cachedCaveField(this),
    cachedSdf(this),
    cachedWaterSdf(this)
    // cachedDamageSdf(this)
{}
DCInstance::~DCInstance() {}

// chunks
// 3d
Chunk3D &DCInstance::getChunk(const vm::ivec3 &min, const int lod, GenerateFlags flags) {
    abort();
    /* if (lod != 1) {
        EM_ASM({
          console.log('getChunk 3d bad lod', $0);
        }, lod);
        abort();
    } */
    uint64_t minHash = hashOctreeMin(min);

    /* if (tryLock(min, lod)) {
        EM_ASM({
            console.log('chunk was not locked 3d', $0, $1, $2, $3);
        }, min.x, min.y, min.z, lod);
        abort();
    } */

    Chunk3D *chunkNoise;
    {
        std::unique_lock<Mutex> lock(cachesMutex);
        chunkNoise = &getChunkInternal(min, lod);
    }
    // chunkNoise->chunk2d->generate(this, flags);
    // chunkNoise->generate(this, flags);
    return *chunkNoise;
}
Chunk3D &DCInstance::getChunkInternal(const vm::ivec3 &min, int lod) {
    uint64_t minHash = hashOctreeMin(min);

    const auto &iter = chunksCache3D.find(minHash);
    if (iter == chunksCache3D.end()) {
        /* if (tryLock(min, lod)) {
            EM_ASM({
                console.log('chunk was not locked 3d', $0, $1, $2, $3);
            }, min.x, min.y, min.z, lod);
            abort();
        } */

        vm::ivec2 min2D{min.x, min.z};
        Chunk2D *chunk2d = &getChunkInternal(min2D, lod);
        chunksCache3D.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(minHash),
            std::forward_as_tuple(min, lod, chunk2d)
        );
    }
    Chunk3D &chunkNoise = chunksCache3D.find(minHash)->second;
    return chunkNoise;
}
Chunk3D &DCInstance::getChunkAt(const float x, const float y, const float z, const int lod, GenerateFlags flags) {
    vm::ivec3 min = vm::ivec3{
                        (int)std::floor(x / (float)chunkSize),
                        (int)std::floor(y / (float)chunkSize),
                        (int)std::floor(z / (float)chunkSize)} *
                    chunkSize;
    return getChunk(min, lod, flags);
}

// 2d
Chunk2D &DCInstance::getChunk(const vm::ivec2 &min, const int lod, GenerateFlags flags) {
    abort();
    /* if (lod != 1) {
        EM_ASM({
          console.log('getChunk 2d bad lod', $0);
        }, lod);
        abort();
    } */
    uint64_t minHash = hashOctreeMin(min);

    /* if (tryLock(min, lod)) {
        EM_ASM({
            console.log('chunk was not locked 2d', $0, $1, $2);
        }, min.x, min.y, lod);
        abort();
    } */

    Chunk2D *chunkNoise;
    {
        std::unique_lock<Mutex> lock(cachesMutex);
        chunkNoise = &getChunkInternal(min, lod);
    }
    // chunkNoise->generate(this, flags);
    return *chunkNoise;
}
Chunk2D &DCInstance::getChunkInternal(const vm::ivec2 &min, int lod) {
    uint64_t minHash = hashOctreeMin(min);

    const auto &iter = chunksCache2D.find(minHash);
    if (iter == chunksCache2D.end()) {
        /* if (tryLock(min, lod)) {
            EM_ASM({
                console.log('chunk was not locked 2d', $0, $1, $2);
            }, min.x, min.y, lod);
            abort();
        } */

        chunksCache2D.emplace(
            std::piecewise_construct,
            std::forward_as_tuple(minHash),
            std::forward_as_tuple(min, lod)
        );
    }
    Chunk2D &chunkNoise = chunksCache2D.find(minHash)->second;
    return chunkNoise;
}
Chunk2D &DCInstance::getChunkAt(const float x, const float z, const int lod, GenerateFlags flags)
{
    vm::ivec2 min = vm::ivec2{
                        (int)std::floor(x / (float)chunkSize),
                        (int)std::floor(z / (float)chunkSize)} *
                    chunkSize;
    return getChunk(min, lod, flags);
}

// locks
Mutex *DCInstance::getChunkLock(const vm::ivec2 &worldPos, const int lod, const int flags) {
    /* if (lod != 1) {
        EM_ASM({
          console.log('getChunkLock 2d bad lod', $0);
        }, lod);
        abort();
    } */
    
    Mutex *chunkLock;
    uint64_t minLodHash = hashOctreeMin(worldPos);
    {
        std::unique_lock<Mutex> lock(locksMutex);
        chunkLock = &chunkLocks2D[minLodHash];
    }
    return chunkLock;
}
Mutex *DCInstance::getChunkLock(const vm::ivec3 &worldPos, const int lod) {
    /* if (lod != 1) {
        EM_ASM({
          console.log('getChunkLock 3d bad lod', $0);
        }, lod);
        abort();
    } */

    Mutex *chunkLock;
    uint64_t minLodHash = hashOctreeMin(worldPos);
    {
        std::unique_lock<Mutex> lock(locksMutex);
        chunkLock = &chunkLocks3D[minLodHash];
    }
    return chunkLock;
}

// fields
float *DCInstance::getChunkHeightfield(const vm::ivec2 &worldPositionXZ, int lod) {
    const int &size = chunkSize;
    
    float *heights = (float *)malloc(sizeof(float) * chunkSize * chunkSize);
    
    for (int z = 0; z < size; z++)
    {
        for (int x = 0; x < size; x++)
        {
            int index2D = x + z * size;

            // int gridX = x + 1;
            // int gridZ = z + 1;
            // int gridIndex = gridX + gridZ * gridPoints;

            int ax = worldPositionXZ.x + x;
            int az = worldPositionXZ.y + z;
            heights[index2D] = cachedHeightField.get(ax, az).heightField;
        }
    }
    
    return heights;
}
unsigned char *DCInstance::getChunkSkylight(const vm::ivec3 &worldPosition, int lod) {
    const int &size = chunkSize;

    unsigned char *skylights = (unsigned char *)malloc(sizeof(unsigned char) * size * size * size);

    for (int z = 0; z < size; z++)
    {
        for (int y = 0; y < size; y++)
        {
            for (int x = 0; x < size; x++)
            {
                int dstIndex = x + y * size + z * size * size;

                // int lx = x + 1;
                // int ly = y + 1;
                // int lz = z + 1;
                // int srcIndex = lx + lz * gridPoints + ly * gridPoints * gridPoints; // note: output is y-first, but storage is z-first

                int ax = worldPosition.x + x;
                int ay = worldPosition.y + y;
                int az = worldPosition.z + z;
                // int index = getIndex(ax, ay);
                skylights[dstIndex] = cachedSkylightField.get(ax, ay, az);
            }
        }
    }

    return skylights;
}
unsigned char *DCInstance::getChunkAo(const vm::ivec3 &worldPosition, int lod) {
    const int &size = chunkSize;

    unsigned char *aos = (unsigned char *)malloc(sizeof(unsigned char) * size * size * size);

    for (int z = 0; z < size; z++)
    {
        for (int y = 0; y < size; y++)
        {
            for (int x = 0; x < size; x++)
            {
                int dstIndex = x + y * size + z * size * size;
                // int srcIndex = x + z * size + y * size * size; // note: output is y-first, but storage is z-first

                int ax = worldPosition.x + x;
                int ay = worldPosition.y + y;
                int az = worldPosition.y + z;
                // int index = getIndex(ax, ay);
                aos[dstIndex] = cachedAoField.get(ax, ay, az);
            }
        }
    }

    return aos;
}

// splats
uint8_t *DCInstance::createGrassSplat(const vm::ivec2 &worldPositionXZ, const int lod)
{
    std::vector<float> ps;
    std::vector<float> qs;
    std::vector<float> instances;
    unsigned int count = 0;

    // accumulate
    // Chunk2D &chunk = getChunk(worldPositionXZ, lod, GF_HEIGHTFIELD);
    int minX = worldPositionXZ.x / chunkSize * chunkSize;
    int minZ = worldPositionXZ.y / chunkSize * chunkSize;

    float seed = DualContouring::noises->grassNoise.in2D(minX, minZ);
    unsigned int seedInt;
    memcpy(&seedInt, &seed, sizeof(unsigned int));
    std::mt19937 rng(seedInt);

    const int maxNumGrasses = 4 * 1024;
    // ps.resize(maxNumGrasses * 3);
    // qs.resize(maxNumGrasses * 4);
    // instances.resize(maxNumGrasses);
    for (int i = 0; i < maxNumGrasses; i++)
    {
        float dx = (float)rng() / (float)0xFFFFFFFF * (float)chunkSize;
        float dz = (float)rng() / (float)0xFFFFFFFF * (float)chunkSize;

        float ax = (float)minX + dx;
        float az = (float)minZ + dz;

        // int idx = (int)dx + 1;
        // int idz = (int)dz + 1;
        // int index2D = idx + idz * gridPoints;
        float height = cachedHeightField.get(ax, az).heightField;

        ps.push_back(ax);
        ps.push_back(height);
        ps.push_back(az);

        Quat q = Quat().setFromAxisAngle(Vec{0, 1, 0}, rng() * 2.0f * M_PI);
        qs.push_back(q.x);
        qs.push_back(q.y);
        qs.push_back(q.z);
        qs.push_back(q.w);

        instances.push_back((float)rng() / (float)0xFFFFFFFF);

        count++;
    }

    // serialize
    {
        const size_t size = sizeof(uint32_t) +
            sizeof(float) * ps.size() +
            sizeof(uint32_t) +
            sizeof(float) * qs.size() +
            sizeof(uint32_t) +
            sizeof(float) * instances.size();
        uint8_t *buffer = (uint8_t *)malloc(size);
        int index = 0;
        ((uint32_t *)(buffer + index))[0] = ps.size();
        index += sizeof(uint32_t);
        memcpy(buffer + index, ps.data(), sizeof(float) * ps.size());
        index += sizeof(float) * ps.size();
        ((uint32_t *)(buffer + index))[0] = qs.size();
        index += sizeof(uint32_t);
        memcpy(buffer + index, qs.data(), sizeof(float) * qs.size());
        index += sizeof(float) * qs.size();
        ((uint32_t *)(buffer + index))[0] = instances.size();
        index += sizeof(uint32_t);
        memcpy(buffer + index, instances.data(), sizeof(float) * instances.size());
        index += sizeof(float) * instances.size();
        return buffer;
    }
}
uint8_t *DCInstance::createVegetationSplat(const vm::ivec2 &worldPositionXZ, const int lod)
{
    std::vector<float> ps;
    std::vector<float> qs;
    std::vector<float> instances;
    unsigned int count = 0;

    // Chunk2D &chunk = getChunk(worldPositionXZ, lod, GF_HEIGHTFIELD);

    int minX = worldPositionXZ.x / chunkSize * chunkSize;
    int minZ = worldPositionXZ.y / chunkSize * chunkSize;

    float seed = DualContouring::noises->vegetationNoise.in2D(minX, minZ);
    unsigned int seedInt;
    memcpy(&seedInt, &seed, sizeof(unsigned int));
    std::mt19937 rng(seedInt);

    const int maxNumVeggies = 128;
    const float veggieRate = 0.35;
    for (int i = 0; i < maxNumVeggies; i++)
    {
        float dx = (float)rng() / (float)0xFFFFFFFF * (float)chunkSize;
        float dz = (float)rng() / (float)0xFFFFFFFF * (float)chunkSize;

        float ax = (float)minX + dx;
        float az = (float)minZ + dz;

        float noiseValue = DualContouring::noises->vegetationNoise.in2D(ax, az);

        if (noiseValue < veggieRate)
        {
            int index = 0;

            // int idx = (int)dx + 1;
            // int idz = (int)dz + 1;
            // int index2D = idx + idz * gridPoints;
            float height = cachedHeightField.get(ax, az).heightField;

            ps.push_back(ax);
            ps.push_back(height);
            ps.push_back(az);

            Quat q = Quat().setFromAxisAngle(Vec{0, 1, 0}, rng() * 2.0f * M_PI);
            qs.push_back(q.x);
            qs.push_back(q.y);
            qs.push_back(q.z);
            qs.push_back(q.w);

            instances.push_back((float)rng() / (float)0xFFFFFFFF);

            count++;
        }
    }
    // serialize
    {
        const size_t size = sizeof(uint32_t) +
            sizeof(float) * ps.size() +
            sizeof(uint32_t) +
            sizeof(float) * qs.size() +
            sizeof(uint32_t) +
            sizeof(float) * instances.size();
        uint8_t *buffer = (uint8_t *)malloc(size);
        int index = 0;
        ((uint32_t *)(buffer + index))[0] = ps.size();
        index += sizeof(uint32_t);
        memcpy(buffer + index, ps.data(), sizeof(float) * ps.size());
        index += sizeof(float) * ps.size();
        ((uint32_t *)(buffer + index))[0] = qs.size();
        index += sizeof(uint32_t);
        memcpy(buffer + index, qs.data(), sizeof(float) * qs.size());
        index += sizeof(float) * qs.size();
        ((uint32_t *)(buffer + index))[0] = instances.size();
        index += sizeof(uint32_t);
        memcpy(buffer + index, instances.data(), sizeof(float) * instances.size());
        index += sizeof(float) * instances.size();
        return buffer;
    }
}
uint8_t *DCInstance::createMobSplat(const vm::ivec2 &worldPositionXZ, const int lod)
{
    std::vector<float> ps;
    std::vector<float> qs;
    std::vector<float> instances;
    unsigned int count = 0;

    // Chunk2D &chunk = getChunk(worldPositionXZ, lod, GF_HEIGHTFIELD);

    int minX = worldPositionXZ.x / chunkSize * chunkSize;
    int minZ = worldPositionXZ.y / chunkSize * chunkSize;

    float seed = DualContouring::noises->mobNoise.in2D(minX, minZ);
    unsigned int seedInt;
    memcpy(&seedInt, &seed, sizeof(unsigned int));
    std::mt19937 rng(seedInt);

    const int maxNumMobs = 2;
    const float mobRate = 0.4;
    for (int i = 0; i < maxNumMobs; i++)
    {
        float dx = (float)rng() / (float)0xFFFFFFFF * (float)chunkSize;
        float dz = (float)rng() / (float)0xFFFFFFFF * (float)chunkSize;

        float ax = (float)minX + dx;
        float az = (float)minZ + dz;

        float noiseValue = DualContouring::noises->mobNoise.in2D(ax, az);

        if (noiseValue < mobRate)
        {
            // int idx = (int)dx + 1;
            // int idz = (int)dz + 1;
            // int index2D = idx + idz * gridPoints;
            float height = cachedHeightField.get(ax, az).heightField;

            ps.push_back(ax);
            ps.push_back(height);
            ps.push_back(az);

            Quat q = Quat().setFromAxisAngle(Vec{0, 1, 0}, rng() * 2.0f * M_PI);
            qs.push_back(q.x);
            qs.push_back(q.y);
            qs.push_back(q.z);
            qs.push_back(q.w);

            instances.push_back((float)rng() / (float)0xFFFFFFFF);

            count++;
        }
    }
    // serialize
    {
        const size_t size = sizeof(uint32_t) +
            sizeof(float) * ps.size() +
            sizeof(uint32_t) +
            sizeof(float) * qs.size() +
            sizeof(uint32_t) +
            sizeof(float) * instances.size();
        uint8_t *buffer = (uint8_t *)malloc(size);
        int index = 0;
        ((uint32_t *)(buffer + index))[0] = ps.size();
        index += sizeof(uint32_t);
        memcpy(buffer + index, ps.data(), sizeof(float) * ps.size());
        index += sizeof(float) * ps.size();
        ((uint32_t *)(buffer + index))[0] = qs.size();
        index += sizeof(uint32_t);
        memcpy(buffer + index, qs.data(), sizeof(float) * qs.size());
        index += sizeof(float) * qs.size();
        ((uint32_t *)(buffer + index))[0] = instances.size();
        index += sizeof(uint32_t);
        memcpy(buffer + index, instances.data(), sizeof(float) * instances.size());
        index += sizeof(float) * instances.size();
        return buffer;
    }
}

// biomes
// get biome value for a world point
/* unsigned char DCInstance::getBiome(const vm::vec2 &worldPosition, const int &lod) {
    return cachedBiomesField.get(worldPosition.x, worldPosition.y);
} */

//

/* float DCInstance::getTemperature(const vm::vec2 &worldPosition, const int &lod) {
    Chunk2D &chunkNoise = getChunkAt(worldPosition.x, worldPosition.y, lod, GF_BIOMES);
    int lx = (int)worldPosition.x - chunkNoise.min.x;
    int lz = (int)worldPosition.y - chunkNoise.min.y;
    return chunkNoise.getTemperatureLocal(lx, lz);
}

float DCInstance::getHumidity(const vm::vec2 &worldPosition, const int &lod) {
    Chunk2D &chunkNoise = getChunkAt(worldPosition.x, worldPosition.y, lod, GF_BIOMES);
    int lx = (int)worldPosition.x - chunkNoise.min.x;
    int lz = (int)worldPosition.y - chunkNoise.min.y;
    return chunkNoise.getHumidityLocal(lx, lz);
} */

/* float DCInstance::getWater(const vm::vec2 &worldPosition, const int &lod) {
    Chunk2D &chunkNoise = getChunkAt(worldPosition.x, worldPosition.y, lod, GF_WATERFIELD);
    int lx = (int)worldPosition.x - chunkNoise.min.x;
    int lz = (int)worldPosition.y - chunkNoise.min.y;
    return chunkNoise.getWaterFieldLocal(lx, lz);
} */

//

std::vector<vm::ivec3> getChunkRangeInclusive(const vm::ivec3 &worldPosition, int minChunkDelta, int maxChunkDelta, int chunkSize) {
    std::vector<vm::ivec3> result;
    for (int dy = minChunkDelta; dy <= maxChunkDelta; dy++)
    {
        for (int dz = minChunkDelta; dz <= maxChunkDelta; dz++)
        {
            for (int dx = minChunkDelta; dx <= maxChunkDelta; dx++)
            {
                result.push_back(vm::ivec3{
                    worldPosition.x + dx * chunkSize,
                    worldPosition.y + dy * chunkSize,
                    worldPosition.z + dz * chunkSize
                });
            }
        }
    }
    /* EM_ASM({
      console.log('range 3d', $0);
    }, result.size()); */
    return result;
}
std::vector<vm::ivec2> getChunkRangeInclusive(const vm::ivec2 &worldPosition, int minChunkDelta, int maxChunkDelta, int chunkSize) {
    std::vector<vm::ivec2> result;
    for (int dz = minChunkDelta; dz <= maxChunkDelta; dz++)
    {
        for (int dx = minChunkDelta; dx <= maxChunkDelta; dx++)
        {
            result.push_back(vm::ivec2{
                worldPosition.x + dx * chunkSize,
                worldPosition.y + dz * chunkSize
            });
        }
    }
    /* EM_ASM({
      console.log('range 2d', $0);
    }, result.size()); */
    return result;
}

//

uint8_t *DCInstance::createTerrainChunkMesh(const vm::ivec3 &worldPosition, const int lodArray[8])
{
    int lod = lodArray[0];
    {
        /* if (generateMutex.try_lock())
        {
            generateMutex.unlock();
        } else {
            EM_ASM(
                console.log('generateMutex failed to lock');
            );
            abort();
        }
        std::unique_lock<Mutex> lock(generateMutex); */

        ChunkOctree<TerrainDCContext> chunkOctree(this, worldPosition, lodArray);
        if (!chunkOctree.root)
        {
            // printf("Chunk Has No Data\n");
            return nullptr;
        }
        TerrainDCContext vertexContext;
        generateMeshFromOctree<TerrainDCContext, false>(chunkOctree.root, vertexContext);
        generateMeshFromOctree<TerrainDCContext, true>(chunkOctree.seamRoot, vertexContext);

        auto &vertexBuffer = vertexContext.vertexBuffer;
        if (vertexBuffer.indices.size() == 0)
        {
            // printf("Generated Mesh Is Not Valid\n");
            return nullptr;
        }

        return vertexBuffer.getBuffer();
    }
}
uint8_t *DCInstance::createLiquidChunkMesh(const vm::ivec3 &worldPosition, const int lodArray[8])
{
    int lod = lodArray[0];
    /* EM_ASM({
        console.log('createLiquidChunkMesh', $0, $1, $2, $3);
    }, worldPosition.x, worldPosition.y, worldPosition.z, lod); */

    /* if (chunk.cachedWaterSdf.value.size() == 0){
        EM_ASM({
            console.log('createLiquidChunkMesh did not have sdf', $0, $1, $2, $3);
        }, worldPosition.x, worldPosition.y, worldPosition.z, lod);
        abort();
    } */

    ChunkOctree<LiquidDCContext> chunkOctree(this, worldPosition, lodArray);
    if (!chunkOctree.root)
    {
        // printf("Chunk Has No Data\n");
        return nullptr;
    }
    LiquidDCContext vertexContext;
    generateMeshFromOctree<LiquidDCContext, false>(chunkOctree.root, vertexContext);
    generateMeshFromOctree<LiquidDCContext, true>(chunkOctree.seamRoot, vertexContext);

    auto &vertexBuffer = vertexContext.vertexBuffer;
    if (vertexBuffer.indices.size() == 0)
    {
        // printf("Generated Mesh Is Not Valid\n");
        return nullptr;
    }

    return vertexBuffer.getBuffer();
}

//

bool DCInstance::drawSphereDamage(const float &x, const float &y, const float &z,
                                  const float &radius, float *outPositions, unsigned int *outPositionsCount,
                                  const int &lod)
{
    return damageBuffers.damage(vm::vec3{x,y,z}, radius, outPositions, outPositionsCount, lod);
}

bool DCInstance::eraseSphereDamage(const float &x, const float &y, const float &z,
                                   const float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages,
                                   const int &lod)
{
    unsigned int maxPositionsCount = *outPositionsCount;
    *outPositionsCount = 0;

    bool drew = false;
    std::set<uint64_t> seenHashes;

    // chunk min of the hit point
    vm::ivec3 chunkMin = chunkMinForPosition(vm::ivec3{(int)x, (int)y, (int)z}, lod);

    for (float dx = -1; dx <= 1; dx += 1)
    {
        for (float dz = -1; dz <= 1; dz += 1)
        {
            for (float dy = -1; dy <= 1; dy += 1)
            {
               vm::ivec3 min = chunkMin + vm::ivec3{(int)dx, (int)dy, (int)dz} * chunkSize;

                uint64_t minHash = hashOctreeMin(min);
                if (seenHashes.find(minHash) == seenHashes.end())
                {
                    seenHashes.insert(minHash);

                    // Chunk3D &chunkNoise = getChunk(min, lod, GF_SDF);
                    if (removeSphereDamage(x, y, z, radius))
                    {
                        if (*outPositionsCount < maxPositionsCount)
                        {
                            // int gridSize = chunkSize + 3 + lod;
                            // int damageBufferSize = gridSize * gridSize * gridSize;
                            // memcpy(outDamages + (*outPositionsCount) * damageBufferSize, chunkNoise.cachedSdf.value.data(), sizeof(float) * damageBufferSize);

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

bool DCInstance::drawCubeDamage(
    float x, float y, float z,
    float qx, float qy, float qz, float qw,
    float sx, float sy, float sz,
    float *outPositions,
    unsigned int *outPositionsCount,
    float *outDamages,
    const int &lod)
{
    unsigned int maxPositionsCount = *outPositionsCount;
    *outPositionsCount = 0;

    Matrix m(Vec{x, y, z}, Quat{qx, qy, qz, qw}, Vec{sx, sy, sz});

    bool drew = false;
    std::set<uint64_t> seenHashes;
    for (float dx = -1.f; dx <= 1.f; dx += 2.f)
    {
        for (float dz = -1.f; dz <= 1.f; dz += 2.f)
        {
            for (float dy = -1.f; dy <= 1.f; dy += 2.f)
            {
                Vec p = (Vec(dx, dy, dz) * 0.5).applyMatrix(m);
                float ax = p.x;
                float ay = p.y;
                float az = p.z;
                vm::ivec3 min = vm::ivec3{
                                    (int)std::floor(ax / (float)chunkSize),
                                    (int)std::floor(ay / (float)chunkSize),
                                    (int)std::floor(az / (float)chunkSize)} *
                                chunkSize;
                uint64_t minHash = hashOctreeMin(min);
                if (seenHashes.find(minHash) == seenHashes.end())
                {
                    seenHashes.insert(minHash);

                    // Chunk3D &chunkNoise = getChunk(min, lod, GF_SDF);
                    if (addCubeDamage(
                            x, y, z,
                            qx, qy, qz, qw,
                            sx, sy, sz))
                    {
                        if (*outPositionsCount < maxPositionsCount)
                        {
                            // int gridSize = chunkSize + 3 + lod;
                            // int damageBufferSize = gridSize * gridSize * gridSize;
                            // memcpy(outDamages + (*outPositionsCount) * damageBufferSize, chunkNoise.cachedSdf.value.data(), sizeof(float) * damageBufferSize);

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

bool DCInstance::eraseCubeDamage(
    float x, float y, float z,
    float qx, float qy, float qz, float qw,
    float sx, float sy, float sz,
    float *outPositions,
    unsigned int *outPositionsCount,
    float *outDamages,
    const int &lod)
{
    unsigned int maxPositionsCount = *outPositionsCount;
    *outPositionsCount = 0;

    Matrix m(Vec{x, y, z}, Quat{qx, qy, qz, qw}, Vec{sx, sy, sz});

    bool drew = false;
    std::set<uint64_t> seenHashes;
    for (float dx = -1.f; dx <= 1.f; dx += 2.f)
    {
        for (float dz = -1.f; dz <= 1.f; dz += 2.f)
        {
            for (float dy = -1.f; dy <= 1.f; dy += 2.f)
            {
                Vec p = (Vec(dx, dy, dz) * 0.5).applyMatrix(m);
                float ax = p.x;
                float ay = p.y;
                float az = p.z;
                vm::ivec3 min = vm::ivec3{
                                    (int)std::floor(ax / (float)chunkSize),
                                    (int)std::floor(ay / (float)chunkSize),
                                    (int)std::floor(az / (float)chunkSize)} *
                                chunkSize;
                uint64_t minHash = hashOctreeMin(min);
                if (seenHashes.find(minHash) == seenHashes.end())
                {
                    seenHashes.insert(minHash);

                    // Chunk3D &chunkNoise = getChunk(min, lod, GF_SDF);
                    if (removeCubeDamage(
                            x, y, z,
                            qx, qy, qz, qw,
                            sx, sy, sz))
                    {
                        if (*outPositionsCount < maxPositionsCount)
                        {
                            // int gridSize = chunkSize + 3 + lod;
                            // int damageBufferSize = gridSize * gridSize * gridSize;
                            // memcpy(outDamages + (*outPositionsCount) * damageBufferSize, chunkNoise.cachedSdf.value.data(), sizeof(float) * damageBufferSize);

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

/* void DCInstance::injectDamage(const float &x, const float &y, const float &z, float *damageBuffer, const int &lod)
{
    const vm::ivec3 min = vm::ivec3(x, y, z);
    Chunk3D &chunk = getChunk(min, lod, GF_NONE);
    chunk.injectDamage(damageBuffer);
} */

void DCInstance::setSortPositionQuaternion(const vm::vec3 &worldPosition, const Quat &worldQuaternion) {
    this->worldPosition = worldPosition;
    this->worldQuaternion = worldQuaternion;
    
    DualContouring::taskQueue.setSortPositionQuaternion(worldPosition, worldQuaternion);
}

void DCInstance::setClipRange(const vm::vec3 &min, const vm::vec3 &max)
{
    clipRange.reset(new vm::box3{
        vm::vec3{min.x, min.y, min.z},
        vm::vec3{max.x, max.y, max.z}});
}

//

void DCInstance::createTerrainChunkMeshAsync(uint32_t id, const vm::ivec3 &worldPosition, const int lodArray[8])
{
    std::shared_ptr<Promise> promise = DualContouring::resultQueue.createPromise(id);

    int lod = lodArray[0];
    std::vector<int> lodVector(lodArray, lodArray + 8);

    vm::vec3 worldPositionF{
        (float)worldPosition.x,
        (float)worldPosition.y,
        (float)worldPosition.z
    };
    Task *terrainTask = new Task(id, worldPositionF, lod, [
        this,
        promise,
        worldPosition,
        lod,
        lodVector = std::move(lodVector)
    ]() -> void {
        uint8_t *result = createTerrainChunkMesh(worldPosition, lodVector.data());
        if (!promise->resolve(result)) {
            // XXX cleanup
            // XXX also cleanup the other async calls
        }
    });
    DualContouring::taskQueue.pushTask(terrainTask);
}
void DCInstance::createLiquidChunkMeshAsync(uint32_t id, const vm::ivec3 &worldPosition, const int lodArray[8])
{
    std::shared_ptr<Promise> promise = DualContouring::resultQueue.createPromise(id);

    int lod = lodArray[0];
    std::vector<int> lodVector(lodArray, lodArray + 8);

    vm::vec3 worldPositionF{
        (float)worldPosition.x,
        (float)worldPosition.y,
        (float)worldPosition.z
    };
    Task *liquidTask = new Task(id, worldPositionF, lod, [
        this,
        promise,
        worldPosition,
        lodVector = std::move(lodVector)
    ]() -> void {
        uint8_t *result = createLiquidChunkMesh(worldPosition, lodVector.data());
        promise->resolve(result);
    });
    DualContouring::taskQueue.pushTask(liquidTask);
}
void DCInstance::getChunkHeightfieldAsync(uint32_t id, const vm::ivec2 &worldPositionXZ, int lod) {
    std::shared_ptr<Promise> promise = DualContouring::resultQueue.createPromise(id);

    vm::vec3 worldPositionF{
        (float)worldPosition.x,
        0.f,
        (float)worldPosition.y
    };
    Task *heightfieldTask = new Task(id, worldPositionF, lod, [
        this,
        promise,
        worldPositionXZ,
        lod
    ]() -> void {
        void *result = getChunkHeightfield(worldPositionXZ, lod);
        promise->resolve(result);
    });
    DualContouring::taskQueue.pushTask(heightfieldTask);
}
void DCInstance::getChunkSkylightAsync(uint32_t id, const vm::ivec3 &worldPosition, int lod) {
    std::shared_ptr<Promise> promise = DualContouring::resultQueue.createPromise(id);

    vm::vec3 worldPositionF{
        (float)worldPosition.x,
        (float)worldPosition.y,
        (float)worldPosition.z
    };
    Task *skylightTask = new Task(id, worldPositionF, lod, [
        this,
        promise,
        worldPosition,
        lod
    ]() -> void {
        void *result = getChunkSkylight(worldPosition, lod);
        promise->resolve(result);
    });
    DualContouring::taskQueue.pushTask(skylightTask);
}
void DCInstance::getChunkAoAsync(uint32_t id, const vm::ivec3 &worldPosition, int lod) {
    std::shared_ptr<Promise> promise = DualContouring::resultQueue.createPromise(id);
    
    vm::vec3 worldPositionF{
        (float)worldPosition.x,
        (float)worldPosition.y,
        (float)worldPosition.z
    };
    Task *aoTask = new Task(id, worldPositionF, lod, [
        this,
        promise,
        worldPosition,
        lod
    ]() -> void {
        void *result = getChunkAo(worldPosition, lod);
        promise->resolve(result);
    });
    DualContouring::taskQueue.pushTask(aoTask);
}
void DCInstance::createGrassSplatAsync(uint32_t id, const vm::ivec2 &worldPositionXZ, const int lod) {
    std::shared_ptr<Promise> promise = DualContouring::resultQueue.createPromise(id);

    vm::vec3 worldPositionF{
        (float)worldPositionXZ.x,
        0.f,
        (float)worldPositionXZ.y
    };
    Task *grassSplatTask = new Task(id, worldPositionF, lod, [
        this,
        promise,
        worldPositionXZ,
        lod
    ]() -> void {
        uint8_t *result = createGrassSplat(worldPositionXZ, lod);
        promise->resolve(result);
    });
    DualContouring::taskQueue.pushTask(grassSplatTask);
}
void DCInstance::createVegetationSplatAsync(uint32_t id, const vm::ivec2 &worldPositionXZ, const int lod) {
    std::shared_ptr<Promise> promise = DualContouring::resultQueue.createPromise(id);

    vm::vec3 worldPositionF{
        (float)worldPositionXZ.x,
        0.f,
        (float)worldPositionXZ.y
    };
    Task *vegetationSplatTask = new Task(id, worldPositionF, lod, [
        this,
        promise,
        worldPositionXZ,
        lod
    ]() -> void {
        void *result = createVegetationSplat(worldPositionXZ, lod);
        promise->resolve(result);
    });
    DualContouring::taskQueue.pushTask(vegetationSplatTask);
}
void DCInstance::createMobSplatAsync(uint32_t id, const vm::ivec2 &worldPositionXZ, const int lod) {
    std::shared_ptr<Promise> promise = DualContouring::resultQueue.createPromise(id);

    vm::vec3 worldPositionF{
        (float)worldPositionXZ.x,
        0.f,
        (float)worldPositionXZ.y
    };
    Task *mobSplatTask = new Task(id, worldPositionF, lod, [
        this,
        promise,
        worldPositionXZ,
        lod
    ]() -> void {
        void *result = createMobSplat(worldPositionXZ, lod);
        promise->resolve(result);
    });
    DualContouring::taskQueue.pushTask(mobSplatTask);
}

// 2d caches

NoiseField DCInstance::initNoiseField(DCInstance *inst, int x, int z) {
    // const int &size = chunkSize;
    // const vm::ivec2 &min = chunk->min;
    // const int &lod = chunk->lod;
    
    /* NoiseField noiseField;
    noiseField.temperature.resize(size * size);
    noiseField.humidity.resize(size * size);
    noiseField.ocean.resize(size * size);
    noiseField.river.resize(size * size);
    for (int z = 0; z < size; z++)
    {
        for (int x = 0; x < size; x++)
        { */
            // int index = x + z * size;
            // int ax = x;
            // int az = y;

            float tNoise = (float)DualContouring::noises->temperatureNoise.in2D(x, z);
            // noiseField.temperature[index] = tNoise;

            float hNoise = (float)DualContouring::noises->humidityNoise.in2D(x, z);
            // noiseField.humidity[index] = hNoise;

            float oNoise = (float)DualContouring::noises->oceanNoise.in2D(x, z);
            // noiseField.ocean[index] = oNoise;

            float rNoise = (float)DualContouring::noises->riverNoise.in2D(x, z);
            // noiseField.river[index] = rNoise;

            return NoiseField{
                tNoise,
                hNoise,
                oNoise,
                rNoise
            };
        /* }
    }

    return noiseField; */
}
uint8_t DCInstance::initBiomesField(DCInstance *inst, int x, int z) {
    // const int &size = chunkSize;
    // const auto &cachedNoiseField = chunk->cachedNoiseField;
    
    // std::vector<uint8_t> biomesField(size * size);
    /* for (int z = 0; z < size; z++)
    {
        for (int x = 0; x < size; x++)
        { */
            // int index = x + z * size;
            // unsigned char biome = cachedBiomesField.get(x, z);
            unsigned char biome = 0xFF;

            const auto &noise = inst->cachedNoiseField.get(x, z);
            float temperatureNoise = noise.temperature;
            float humidityNoise = noise.humidity;
            float oceanNoise = noise.ocean;
            float riverNoise = noise.river;

            if (oceanNoise < (80.0f / 255.0f))
            {
                biome = (unsigned char)BIOME::biOcean;
            }
            if (biome == 0xFF)
            {
                const float range = 0.022f;
                if (riverNoise > 0.5f - range && riverNoise < 0.5f + range)
                {
                    biome = (unsigned char)BIOME::biRiver;
                }
            }
            if (std::pow(temperatureNoise, 1.3f) < ((4.0f * 16.0f) / 255.0f))
            {
                if (biome == (unsigned char)BIOME::biOcean)
                {
                    biome = (unsigned char)BIOME::biFrozenOcean;
                }
                else if (biome == (unsigned char)BIOME::biRiver)
                {
                    biome = (unsigned char)BIOME::biFrozenRiver;
                }
            }
            if (biome == 0xFF)
            {
                float temperatureNoise2 = vm::clamp(std::pow(temperatureNoise, 1.3f), 0.f, 1.f);
                float humidityNoise2 = vm::clamp(std::pow(humidityNoise, 1.3f), 0.f, 1.f);

                int t = (int)std::floor(temperatureNoise2 * 16.0f);
                int h = (int)std::floor(humidityNoise2 * 16.0f);
                biome = (unsigned char)BIOMES_TEMPERATURE_HUMIDITY[t + 16 * h];
            }
            return biome;
        /* }
    } */
    // return biomesField;
}
Heightfield DCInstance::initHeightField(DCInstance *inst, int x, int z) {
    const int &size = chunkSize;
    // const int &gridPoints = DualContouring::gridPoints;
    // const int &lod = chunk->lod;
    // const vm::ivec2 &min = chunk->min;
    
    Heightfield heightfield;
    // heightfield.biomesVectorField.resize(gridPoints * gridPoints * 4);
    // heightfield.biomesWeightsVectorField.resize(gridPoints * gridPoints * 4);
    // heightfield.heightField.resize(gridPoints * gridPoints);
    /* for (int z = 0; z < gridPoints; z++)
    {
        for (int x = 0; x < gridPoints; x++)
        { */
            // int index2D = x + z * gridPoints;
            // int ax = x;
            // int az = z;
            
            // int lx = x - 1;
            // int lz = z - 1;
            // int index2D2 = lx + lz * size;
            // bool isInRange = lx >= 0 && lx < size && lz >= 0 && lz < size;

            std::unordered_map<unsigned char, unsigned int> biomeCounts(numBiomes);
            int numSamples = 0;
            for (int dz = -size/2; dz < size/2; dz++)
            {
                for (int dx = -size/2; dx < size/2; dx++)
                {
                    // vm::vec2 worldPosition(ax + dx, az + dz);
                    unsigned char b = inst->cachedBiomesField.get(x + dx, z + dz);

                    biomeCounts[b]++;
                    numSamples++;
                }
            }

            std::vector<unsigned char> seenBiomes;
            for (auto kv : biomeCounts)
            {
                unsigned char b = kv.first;
                seenBiomes.push_back(b);
            }
            
            // sort by increasing occurence count of the biome
            std::sort(
                seenBiomes.begin(),
                seenBiomes.end(),
                [&](unsigned char b1, unsigned char b2) -> bool {
                    return biomeCounts[b1] > biomeCounts[b2];
                }
            );

            for (size_t i = 0; i < 4; i++)
            {
                if (i < seenBiomes.size())
                {
                    heightfield.biomesVectorField[i] = seenBiomes[i];
                    heightfield.biomesWeightsVectorField[i] = (float)biomeCounts[seenBiomes[i]] / (float)numSamples * 255.0f;
                }
                else
                {
                    heightfield.biomesVectorField[i] = 0;
                    heightfield.biomesWeightsVectorField[i] = 0;
                }
            }

            float elevationSum = 0.f;
            vm::vec2 fWorldPosition{(float)x, (float)z};
            for (auto const &iter : biomeCounts)
            {
                elevationSum += iter.second * DualContouring::getComputedBiomeHeight(iter.first, fWorldPosition);
            }

            float elevation = elevationSum / (float)numSamples;
            heightfield.heightField = elevation;
        /* }
    } */
    return heightfield;
}
float DCInstance::initWaterField(DCInstance *inst, int x, int z) {
    const int &size = chunkSize;

    float value = 0;
    // std::unordered_map<unsigned char, unsigned int> biomeCounts(numBiomes);
    // int numSamples = 0;
    for (int dz = -size/2; dz < size/2; dz++)
    {
        for (int dx = -size/2; dx < size/2; dx++)
        {
            unsigned char b = inst->cachedBiomesField.get(x + dx, z + dz);

            if (isWaterBiome(b)) {
                // waterField[index2D]++;
                value++;
            }
        }
    }

    return value;
}

// 3d caches

uint8_t DCInstance::initSkylightField(DCInstance *inst, int x, int y, int z) {
    // const int &gridPoints = DualContouring::gridPoints;
    // const vm::ivec3 &min = chunk->min;
    // auto &chunk2d = chunk->chunk2d;
    // auto &cachedSdf = chunk->cachedSdf;

    constexpr uint8_t maxSkyLightu8 = 8;
    constexpr int maxSkyLighti = (int)maxSkyLightu8;
    constexpr float maxSkyLightf = (float)maxSkyLighti;
    // std::vector<uint8_t> skylightField(gridPoints * gridPoints * gridPoints, maxSkyLight);
    /* for (int z = 0; z < gridPoints; z++)
    {
        // int lz = z + 1;

        for (int x = 0; x < gridPoints; x++)
        { */
            // int lx = x + 1;

            // int index2D = x + z * gridPoints;
            float height = inst->cachedHeightField.get(x, z).heightField;
            
            // int topAY = min.y + gridPoints - 1;
            // int topAY = 16;
            uint8_t skylight = std::min(std::max(/*(float)topAY */maxSkyLightf + y - height, 0.f), maxSkyLightf);

            /* for (int ay = y - 1; ay >= height - maxSkyLighti; ay--)
            {
                // int ly = y + 1;
                // int ay = y + dy;

                // int sdfIndex = x + z * gridPoints + y * gridPoints * gridPoints;
                if (inst->cachedSdf.get(x, ay, z) < 0.f)
                {
                    skylight--;
                }

                // int skylightIndex = x + z * gridPoints + y * gridPoints * gridPoints;
                // skylightField[skylightIndex] = skylight;
            } */
        /* }
    } */

    skylight = std::min(std::max(skylight, (uint8_t)0), maxSkyLightu8);

    // XXX should flood fill the light

    /* for (int x = 0; x < gridPoints; x++)
    {
        for (int z = 0; z < gridPoints; z++)
        {
            // int lz = z + 1;
            // int lx = x + 1;

            // for (int y = gridPoints - 1; y >= 0; y--) {
            for (int y = 0; y < gridPoints; y++)
            {
                // int ly = y + 1;

                int skylightIndex = x + z * gridPoints + y * gridPoints * gridPoints;
                float maxNeighborSkylight = cachedSkylightField[skylightIndex];
                for (int dz = -1; dz <= 1; dz += 2)
                {
                    for (int dx = -1; dx <= 1; dx += 2)
                    {
                        for (int dy = -1; dy <= 1; dy += 2)
                        {
                            int lx = x + dx;
                            int ly = y + dy;
                            int lz = z + dz;

                            float deltaRadius = std::sqrt(dx * dx + dy * dy + dz * dz);

                            if (lx >= 0 && lx < gridPoints && ly >= 0 && ly < gridPoints && lz >= 0 && lz < gridPoints)
                            {
                                int neighborIndex = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                                float skylight = cachedSkylightField[neighborIndex];
                                maxNeighborSkylight = std::max(maxNeighborSkylight, skylight - deltaRadius);
                            }
                        }
                    }
                }

                cachedSkylightField[skylightIndex] = maxNeighborSkylight;
            }
        }
    } */
    return skylight;
}
uint8_t DCInstance::initAoField(DCInstance *inst, int x, int y, int z) {
    const int &size = chunkSize;
    // const int &gridPoints = DualContouring::gridPoints;
    // auto &cachedSdf = chunk->cachedSdf;
    
    /* std::vector<uint8_t> aoField(size * size * size, 3 * 3 * 3);
    for (int y = 0; y < size; y++)
    {
        int ly = y + 1;

        for (int z = 0; z < size; z++)
        {
            int lz = z + 1;

            for (int x = 0; x < size; x++)
            {
                int lx = x + 1; */

                unsigned char numOpens = 0;
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            // int sdfIndex = (lx + dx) + (lz + dz) * gridPoints + (ly + dy) * gridPoints * gridPoints;
                            numOpens += (unsigned char)(inst->cachedSdf.get(x + dx, y + dy, z + dz) >= 0.f);
                        }
                    }
                }

                // int aoIndex = x + z * size + y * size * size;
                // aoField[aoIndex] = numOpens;
            /* }
        }
    } */
    return numOpens;
}
float DCInstance::initCaveField(DCInstance *inst, int x, int y, int z) {
    return DualContouring::getComputedCaveNoise(x, y, z);
}
float randomFromPoint(int x, int y, int z) {
    uint64_t hash = hashOctreeMin(vm::ivec3{x, y, z});
    uint32_t hash32 = (uint32_t)hash ^ (uint32_t)(hash >> 32);
    float f = (float)hash32 / (float)UINT32_MAX;
    return f;
}
float DCInstance::initSdf(DCInstance *inst, int x, int y, int z) {
    // const int &gridPoints = DualContouring::gridPoints;
    /* auto &chunk2d = chunk->chunk2d;
    const vm::ivec3 &min = chunk->min;
    const int &lod = chunk->lod; */

    /* std::vector<float> sdf(gridPoints * gridPoints * gridPoints, MAX_HEIGHT);
    for (int z = 0; z < gridPoints; z++)
    {
        for (int x = 0; x < gridPoints; x++)
        { */
            // int index2D = x + z * gridPoints;
            float height = inst->cachedHeightField.get(x, z).heightField;

            /* for (int y = 0; y < gridPoints; y++)
            { */
                // int index3D = x + z * gridPoints + y * gridPoints * gridPoints;

                // height
                float heightValue = (float)y - height;
                heightValue = std::min(
                    std::max(
                        heightValue,
                        (float)-1),
                    (float)1);

                float caveValue = inst->cachedCaveField.get(x, y, z);
                float f = heightValue + caveValue * 1.1f + randomFromPoint(x, y, z) * 0.0001f;
                /* f = std::min( // XXX does not fix
                    std::max(
                        f,
                        -1.f
                    ),
                    1.f
                ); */

                // result
                // sdf[index3D] = f;
            // }
        /* }
    } */
    return f;
}
/* float DCInstance::initDamageSdf(DCInstance *inst, int x, int y, int z) {
    return MAX_HEIGHT;
} */
float DCInstance::initWaterSdf(DCInstance *inst, int x, int y, int z) {
    /* EM_ASM({
        console.log('init water sdf');
    }); */
    // const int &gridPoints = DualContouring::gridPoints;
    // auto &chunk2d = chunk->chunk2d;
    // const vm::ivec3 &min = chunk->min;

    const float fSize = (float)chunkSize;

    // std::vector<float> waterSdf(gridPoints * gridPoints * gridPoints, MAX_HEIGHT);
    /* EM_ASM({
        console.log('water sdf set size', $0, $1, $2, $3, $4);
    }, chunk->min.x, chunk->min.y, chunk->min.z, chunk->lod, waterSdf.size()); */
    /* for (int z = 0; z < gridPoints; z++)
    {
        int az = min.z + z - 1;
        for (int x = 0; x < gridPoints; x++)
        {
            int ax = min.x + x - 1; */

            // int lx = x + 1;
            // int lz = z + 1;
            // int index2D = x + z * gridPoints;
            float waterValue = -inst->cachedWaterField.get(x, z) / fSize;
            // float waterValue = -inst->getWater(vm::vec2(ax, az), lod) / fSize;
            // waterValue *= -1.1f;
            /* for (int y = 0; y < gridPoints; y++)
            {
                int ay = min.y + y - 1; */

                float heightValue = (float)y - waterBaseHeight;
                heightValue = std::min(
                    std::max(
                        heightValue,
                        -1.f
                    ),
                    1.f
                );
                
                float value = std::max(waterValue, heightValue);

                // int index3D = x + z * gridPoints + y * gridPoints * gridPoints;
                // waterSdf[index3D] = value;
            // }
        /* }
    } */
    return value;
}

//

// biomes
/* unsigned char DCInstance::getCachedBiome(const int lx, const int lz) const {
    const int &size = chunkSize;
    int index = lx + lz * size;
    return cachedBiomesField.value[index];
} */
void DCInstance::getCachedBiome2D(const vm::ivec2 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights, std::array<UV, 2> &biomeUvs1, std::array<UV, 2> &biomeUvs2) {
    const auto &heightfield = cachedHeightField.get(worldPosition.x, worldPosition.y);
    biome.x = heightfield.biomesVectorField[0];
    biome.y = heightfield.biomesVectorField[1];
    biome.z = heightfield.biomesVectorField[2];
    biome.w = heightfield.biomesVectorField[3];

    biomeWeights.x = heightfield.biomesWeightsVectorField[0];
    biomeWeights.y = heightfield.biomesWeightsVectorField[1];
    biomeWeights.z = heightfield.biomesWeightsVectorField[2];
    biomeWeights.w = heightfield.biomesWeightsVectorField[3];

    biomeUvs1[0] = BIOME_UVS[(int)biome.x];
    biomeUvs1[1] = BIOME_UVS[(int)biome.y];
    biomeUvs2[0] = BIOME_UVS[(int)biome.z];
    biomeUvs2[1] = BIOME_UVS[(int)biome.w];
}
inline void shiftOverrideBiome(vm::ivec4 &biome, vm::vec4 &biomeWeights, std::array<UV, 2> &biomeUvs1, std::array<UV, 2> &biomeUvs2, BIOME b) {
    // move the biomes to make room
    biome.w = biome.z;
    biome.z = biome.y;
    biome.y = biome.x;
    biome.x = (unsigned char)b;

    biomeWeights.x = 1;
    biomeWeights.y = 0;
    biomeWeights.z = 0;
    biomeWeights.w = 0;

    biomeUvs1[0] = BIOME_UVS[(int)b];
    biomeUvs1[1] = {0, 0};
    biomeUvs2[0] = {0, 0};
    biomeUvs2[1] = {0, 0};
}
void DCInstance::getCachedInterpolatedBiome3D(const vm::vec3 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights, std::array<UV, 2> &biomeUvs1, std::array<UV, 2> &biomeUvs2) {
    vm::ivec3 iWorldPosition{(int)worldPosition.x, (int)worldPosition.y, (int)worldPosition.z};
    vm::ivec2 iWorldPositionXZ{(int)worldPosition.x, (int)worldPosition.z};

    const int &x = iWorldPosition.x;
    const int &y = iWorldPosition.y;
    const int &z = iWorldPosition.z;

    getCachedBiome2D(iWorldPositionXZ, biome, biomeWeights, biomeUvs1, biomeUvs2);

    float heightValue = cachedHeightField.get(x, z).heightField;
    float sdfValue = cachedSdf.get(x, y, z);

    bool neighborHeightsValid = true;
    for (int dx = -1; dx <= 1; dx += 2)
    {
        for (int dz = -1; dz <= 1; dz += 2)
        {
            int lx2 = x + dx;
            int lz2 = z + dz;
            float heightValue = cachedHeightField.get(lx2, lz2).heightField;
            if (y + 3 > heightValue)
            {
                neighborHeightsValid = false;
                break;
            }
        }
        if (!neighborHeightsValid)
        {
            break;
        }
    }

    if (neighborHeightsValid)
    {
        if (y < heightValue - 12)
        {
            shiftOverrideBiome(biome, biomeWeights, biomeUvs1, biomeUvs2, BIOME::teStone);
        }
        else if (y < heightValue - 2)
        {
            shiftOverrideBiome(biome, biomeWeights, biomeUvs1, biomeUvs2, BIOME::teDirt);
        }
    }
}

// lighting
void DCInstance::getCachedInterpolatedLight(const vm::vec3 &worldPosition, uint8_t &skylight, uint8_t &ao) {
    vm::ivec3 iWorldPosition{(int)worldPosition.x, (int)worldPosition.y, (int)worldPosition.z};

    const int &x = iWorldPosition.x;
    const int &y = iWorldPosition.y;
    const int &z = iWorldPosition.z;

    skylight = cachedSkylightField.get(x, y, z);
    ao = cachedAoField.get(x, y, z);
}

// sdf
float DCInstance::getCachedInterpolatedSdf(const float x, const float y, const float z, const int lod) {
    return trilinear<decltype(cachedSdf), float>(
        vm::vec3{x, y, z},
        lod,
        cachedSdf
    );
}
float DCInstance::getCachedWaterInterpolatedSdf(const float x, const float y, const float z, const int lod) {
    // const int &gridPoints = DualContouring::gridPoints;

    // const float localX = x + 1;
    // const float localY = y + 1;
    // const float localZ = z + 1;
    return trilinear<decltype(cachedWaterSdf), float>(
        vm::vec3{x, y, z},
        lod,
        cachedWaterSdf
    );
}
float DCInstance::getCachedDamageInterpolatedSdf(const float &x, const float &y, const float &z, const int &lod) {
    const float defaultDistance = chunkSize * lod;
    DamageBuffersMap &damageBuffersMap = damageBuffers.chunks;

    if(damageBuffersMap.size() > 0){
        const vm::ivec3 chunkMin = chunkMinForPosition(vm::ivec3{int(x),int(y),int(z)}, lod);
        const uint64_t hashedMin = hashOctreeMin(chunkMin);
        // search for the chunk of the voxel
        auto iter = damageBuffersMap.find(hashedMin);
        if (iter == end(damageBuffersMap))
        {
            // the requested voxel's chunk doesn't have damage, so return the default distance
            return defaultDistance;
        }
        else 
        {
            // the requested voxel's chunk has damage
            DamageSdf &cachedDamageSdf = iter->second->bakedDamageSdf;
            return trilinear<decltype(cachedDamageSdf),float>(
                vm::vec3{x, y, z},
                lod,
                cachedDamageSdf
            );
        }
    }
    else
    {
        return defaultDistance;
    }
    
}

//

// signed distance field function for a box at the origin
// returns negative for points inside the box, zero at the box's surface, and positive for points outside the box
// sx sy sz is the size of the box. the box goes from -sx/2 to sx/2, -sy/2 to sy/2, -sz/2 to sz/2
// px py pz is the point to check
float DCInstance::signedDistanceToBox(float sx, float sy, float sz, float px, float py, float pz)
{
    float dx = std::abs(px) - sx / 2;
    float dy = std::abs(py) - sy / 2;
    float dz = std::abs(pz) - sz / 2;
    float d = std::max(std::max(dx, dy), dz);
    return d;
}

// signed distance to sphere
// returns negative for points inside the sphere, zero at the sphere's surface, and positive for points outside the sphere
// cx, cy, cz is the center of the sphere. r is the radius. px, py, pz is the point to check
float DCInstance::signedDistanceToSphere(float cx, float cy, float cz, float r, float px, float py, float pz)
{
    float dx = px - cx;
    float dy = py - cy;
    float dz = pz - cz;
    float d = sqrt(dx * dx + dy * dy + dz * dz);
    return d - r;
}

void DCInstance::patchFrontier(DCInstance *inst, std::unordered_map<uint64_t, bool> &erased) {
    /* const int &gridPoints = DualContouring::gridPoints;

    std::function<void(const vm::ivec3 &)> tryIndex = [&](const vm::ivec3 &v) -> void {
        int index = getIndex(v.x, v.y, v.z);
        if (erased[index])
        {
            std::vector<vm::ivec3> unerasedNeighbors;
            for (int dx = -1; dx <= 1; dx += 2)
            {
                for (int dy = -1; dy <= 1; dy += 2)
                {
                    for (int dz = -1; dz <= 1; dz += 2)
                    {
                        int ax = v.x + dx;
                        int ay = v.y + dy;
                        int az = v.z + dz;

                        int neighborIndex = ax + ay * gridPoints + az * gridPoints * gridPoints;
                        if (!erased[neighborIndex])
                        {
                            unerasedNeighbors.push_back(vm::ivec3(dx, dy, dz));
                            break;
                        }
                    }
                }
            }
            if (unerasedNeighbors.size() > 0)
            {
                // compute the current sdf min distance from the neighbors
                float minDistance = inst->cachedDamageSdf.get(v.x, v.y, v.z);
                for (auto &neighborOffset : unerasedNeighbors)
                {
                    int ax = v.x + neighborOffset.x;
                    int ay = v.y + neighborOffset.y;
                    int az = v.z + neighborOffset.z;

                    // int neighborIndex = ax + ay * gridPoints + az * gridPoints * gridPoints;
                    float neighborDamageSdf = inst->cachedDamageSdf.get(ax, ay, az);
                    float extraDistance = length(neighborOffset);
                    minDistance = std::min(minDistance, neighborDamageSdf + extraDistance);
                }
                inst->cachedDamageSdf.set(v.x, v.y, v.z, minDistance);
                erased[index] = false;
                unerasedNeighbors.clear();

                for (int dx = -1; dx <= 1; dx += 2)
                {
                    for (int dy = -1; dy <= 1; dy += 2)
                    {
                        for (int dz = -1; dz <= 1; dz += 2)
                        {
                            int ax = v.x + dx;
                            int ay = v.y + dy;
                            int az = v.z + dz;
                            tryIndex(vm::ivec3(ax, ay, az));
                        }
                    }
                }
            }
        }
    };

    for (int ly = 0; ly < gridPoints; ly++)
    {
        for (int lz = 0; lz < gridPoints; lz++)
        {
            for (int lx = 0; lx < gridPoints; lx++)
            {
                int ax = lx + min.x - lod;
                int ay = ly + min.y - lod;
                int az = lz + min.z - lod;
                tryIndex(vm::ivec3(ax, ay, az));
            }
        }
    } */
}

bool DCInstance::removeSphereDamage(const float &x, const float &y, const float &z, const float radius)
{
    return false;
    /* const int &gridPoints = DualContouring::gridPoints;
    const int &size = chunkSize;

    std::unordered_map<uint64_t, bool> erased;

    bool drew = false;
    for (int ly = 0; ly < gridPoints; ly++)
    {
        for (int lz = 0; lz < gridPoints; lz++)
        {
            for (int lx = 0; lx < gridPoints; lx++)
            {
                int ax = min.x + lx - lod;
                int ay = min.y + ly - lod;
                int az = min.z + lz - lod;

                float newDistance = signedDistanceToSphere(x, y, z, radius, ax, ay, az);

                // int index = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                int index = getIndex(ax, ay, az);
                float oldDistance = inst->cachedDamageSdf.get(ax, ay, az);

                if (
                    newDistance <= 0.f ||      // new point is inside the sphere
                    newDistance <= oldDistance // new point affects this index
                )
                {
                    inst->cachedDamageSdf.set(ax, ay, az, (float)size);
                    erased[index] = true;
                    drew = true;
                }
            }
        }
    }

    if (drew)
    {
        patchFrontier(inst, erased);
    }

    return drew; */
}

bool DCInstance::addCubeDamage(
    const float &x, const float &y, const float &z,
    const float &qx, const float &qy, const float &qz, const float &qw,
    const float &sx, const float &sy, const float &sz)
{
    return false;

    /* const int &gridPoints = DualContouring::gridPoints;

    Matrix m(Vec{x, y, z}, Quat{qx, qy, qz, qw}, Vec{1, 1, 1});
    Matrix mInverse = m;
    mInverse.invert();

    bool drew = true;
    for (int ly = 0; ly < gridPoints; ly++)
    {
        for (int lz = 0; lz < gridPoints; lz++)
        {
            for (int lx = 0; lx < gridPoints; lx++)
            {
                int ax = min.x + lx - lod;
                int ay = min.y + ly - lod;
                int az = min.z + lz - lod;

                Vec p = Vec(ax, ay, az).applyMatrix(mInverse);
                float newDistance = signedDistanceToBox(sx, sy, sz, p.x, p.y, p.z);

                // int index = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                int index = getIndex(ax, ay, az);
                float oldDistance = inst->cachedDamageSdf.get(ax, ay, az);

                if (newDistance < oldDistance)
                {
                    inst->cachedDamageSdf.set(ax, ay, az, newDistance);
                    drew = true;
                }
            }
        }
    }
    return drew; */
}
bool DCInstance::removeCubeDamage(
    const float &x, const float &y, const float &z,
    const float &qx, const float &qy, const float &qz, const float &qw,
    const float &sx, const float &sy, const float &sz)
{
    return false;

    /* const int &gridPoints = DualContouring::gridPoints;
    const int &size = chunkSize;

    Matrix m(Vec{x, y, z}, Quat{qx, qy, qz, qw}, Vec{1, 1, 1});
    Matrix mInverse = m;
    mInverse.invert();

    std::unordered_map<uint64_t, bool> erased;

    bool drew = true;
    for (int ly = 0; ly < gridPoints; ly++)
    {
        for (int lz = 0; lz < gridPoints; lz++)
        {
            for (int lx = 0; lx < gridPoints; lx++)
            {
                int ax = min.x + lx - lod;
                int ay = min.y + ly - lod;
                int az = min.z + lz - lod;

                Vec p = Vec(ax, ay, az).applyMatrix(mInverse);
                float newDistance = signedDistanceToBox(sx, sy, sz, p.x, p.y, p.z);

                // int index = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                int index = getIndex(ax, ay, az);
                float oldDistance = inst->cachedDamageSdf.get(ax, ay, az);

                if (newDistance <= 0.f || oldDistance >= newDistance)
                {
                    inst->cachedDamageSdf.set(ax, ay, az, (float)size);
                    erased[index] = true;
                    drew = true;
                }
            }
        }
    }

    if (drew) {
        patchFrontier(inst, erased);
    }

    return drew; */
}

void DCInstance::trackerUpdateAsync(uint32_t id, Tracker *tracker, const vm::vec3 &position) {
    std::shared_ptr<Promise> promise = DualContouring::resultQueue.createPromise(id);

    Task *trackerUpdateTask = new Task(id, [
        this,
        promise,
        tracker,
        position
    ]() -> void {
        const TrackerUpdate &trackerUpdate = tracker->update(position);
        uint8_t *buffer = trackerUpdate.getBuffer();
        if (!promise->resolve(buffer)) {
          // XXX clean up
        }
    });
    DualContouring::taskQueue.pushTaskPre(trackerUpdateTask);
}