#include "instance.h"
#include "main.h"
#include "octree.h"
#include "lock.h"
#include "../vector.h"
#include "../util.h"
#include <emscripten.h>

constexpr int CHUNK_RANGE = 1;

// constructor/destructor
DCInstance::DCInstance() {}
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
    uint64_t minHash = hashOctreeMinLod(min, lod);

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
    uint64_t minHash = hashOctreeMinLod(min, lod);

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
    uint64_t minHash = hashOctreeMinLod(min, lod);

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
    uint64_t minHash = hashOctreeMinLod(min, lod);

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
    uint64_t minLodHash = hashOctreeMinLodLayer(worldPos, lod, flags);
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
    uint64_t minLodHash = hashOctreeMinLod(worldPos, lod);
    {
        std::unique_lock<Mutex> lock(locksMutex);
        chunkLock = &chunkLocks3D[minLodHash];
    }
    return chunkLock;
}

// fields
float *DCInstance::getChunkHeightfield(const vm::ivec2 &worldPositionXZ, int lod) {
    Caches caches;
    auto &heightField = caches.getHeightField();
    
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
            heights[index2D] = heightField.get(ax, az).heightField;
        }
    }
    
    return heights;
}
unsigned char *DCInstance::getChunkSkylight(const vm::ivec3 &worldPosition, int lod) {
    Caches caches;
    auto &skylightField = caches.getSkylight();
    
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
                skylights[dstIndex] = skylightField.get(ax, ay, az);
            }
        }
    }

    return skylights;
}
unsigned char *DCInstance::getChunkAo(const vm::ivec3 &worldPosition, int lod) {
    Caches caches;
    auto &ao = caches.getAo();

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
                aos[dstIndex] = ao.get(ax, ay, az);
            }
        }
    }

    return aos;
}

// splats
uint8_t *DCInstance::createGrassSplat(const vm::ivec2 &worldPositionXZ, const int lod)
{
    Caches caches;
    auto &heightField = caches.getHeightField();

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
        float height = heightField.get(ax, az).heightField;

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
    Caches caches;
    auto &heightField = caches.getHeightField();

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
            float height = heightField.get(ax, az).heightField;

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
    Caches caches;
    auto &heightField = caches.getHeightField();

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
            float height = heightField.get(ax, az).heightField;

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
/* unsigned char DCInstance::getBiome(const vm::vec2 &worldPosition, const int &lod) {
    return cachedBiomesField.get(worldPosition.x, worldPosition.y);
}
void DCInstance::getInterpolatedBiomes(const vm::vec2 &worldPosition, const int &lod, vm::ivec4 &biome, vm::vec4 &biomeWeights) {
    getCachedInterpolatedBiome2D(worldPosition, biome, biomeWeights);
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
    Caches caches;
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

    ChunkOctree<TerrainDCContext> chunkOctree(this, &caches, worldPosition, lodArray);
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
uint8_t *DCInstance::createLiquidChunkMesh(const vm::ivec3 &worldPosition, const int lodArray[8])
{
    int lod = lodArray[0];
    Caches caches;
    /* EM_ASM({
        console.log('createLiquidChunkMesh', $0, $1, $2, $3);
    }, worldPosition.x, worldPosition.y, worldPosition.z, lod); */

    /* if (chunk.cachedWaterSdf.value.size() == 0){
        EM_ASM({
            console.log('createLiquidChunkMesh did not have sdf', $0, $1, $2, $3);
        }, worldPosition.x, worldPosition.y, worldPosition.z, lod);
        abort();
    } */

    ChunkOctree<LiquidDCContext> chunkOctree(this, &caches, worldPosition, lodArray);
    if (!chunkOctree.root)
    {
        // printf("Chunk Has No Data\n");
        return nullptr;
    }
    LiquidDCContext vertexContext;
    generateMeshFromOctree<LiquidDCContext, false>(chunkOctree.root, vertexContext);
    generateMeshFromOctree<LiquidDCContext, false>(chunkOctree.seamRoot, vertexContext);

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
                            // int gridSize = chunkSize + 3 + lod;
                            // int damageBufferSize = gridSize * gridSize * gridSize;
                            // memcpy(outDamages + ((*outPositionsCount) * damageBufferSize), chunkNoise.cachedDamageSdf.value.data(), sizeof(float) * damageBufferSize);

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

bool DCInstance::eraseSphereDamage(const float &x, const float &y, const float &z,
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
                uint64_t minHash = hashOctreeMinLod(min, lod);
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
                uint64_t minHash = hashOctreeMinLod(min, lod);
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

void DCInstance::setClipRange(const vm::vec3 &min, const vm::vec3 &max)
{
    clipRange.reset(new vm::box3{
        vm::vec3{min.x, min.y, min.z},
        vm::vec3{max.x, max.y, max.z}});
}

//

uint32_t DCInstance::createTerrainChunkMeshAsync(const vm::ivec3 &worldPosition, const int lodArray[8])
{
    uint32_t id = DualContouring::resultQueue.getNextId();

    int lod = lodArray[0];
    std::vector<int> lodVector(lodArray, lodArray + 8);

    Task *terrainTask = new Task([
        this,
        id,
        worldPosition,
        lod,
        lodVector = std::move(lodVector)
    ]() -> void {
        uint8_t *result = createTerrainChunkMesh(worldPosition, lodVector.data());
        DualContouring::resultQueue.pushResult(id, result);
    });
    DualContouring::taskQueue.pushTask(terrainTask);

    return id;
}
uint32_t DCInstance::createLiquidChunkMeshAsync(const vm::ivec3 &worldPosition, const int lodArray[8])
{
    uint32_t id = DualContouring::resultQueue.getNextId();

    int lod = lodArray[0];
    std::vector<int> lodVector(lodArray, lodArray + 8);

    Task *liquidTask = new Task([
        this,
        id,
        worldPosition,
        lodVector = std::move(lodVector)
    ]() -> void {
        uint8_t *result = createLiquidChunkMesh(worldPosition, lodVector.data());
        DualContouring::resultQueue.pushResult(id, result);
    });
    DualContouring::taskQueue.pushTask(liquidTask);

    return id;
}
uint32_t DCInstance::getChunkHeightfieldAsync(const vm::ivec2 &worldPositionXZ, int lod) {
    uint32_t id = DualContouring::resultQueue.getNextId();

    Task *heightfieldTask = new Task([
        this,
        id,
        worldPositionXZ,
        lod
    ]() -> void {
        void *result = getChunkHeightfield(worldPositionXZ, lod);
        DualContouring::resultQueue.pushResult(id, result);
    });
    DualContouring::taskQueue.pushTask(heightfieldTask);

    return id;
}
uint32_t DCInstance::getChunkSkylightAsync(const vm::ivec3 &worldPosition, int lod) {
    uint32_t id = DualContouring::resultQueue.getNextId();

    Task *skylightTask = new Task([
        this,
        id,
        worldPosition,
        lod
    ]() -> void {
        void *result = getChunkSkylight(worldPosition, lod);
        DualContouring::resultQueue.pushResult(id, result);
    });
    DualContouring::taskQueue.pushTask(skylightTask);

    return id;
}
uint32_t DCInstance::getChunkAoAsync(const vm::ivec3 &worldPosition, int lod) {
    uint32_t id = DualContouring::resultQueue.getNextId();
    
    Task *aoTask = new Task([
        this,
        id,
        worldPosition,
        lod
    ]() -> void {
        void *result = getChunkAo(worldPosition, lod);
        DualContouring::resultQueue.pushResult(id, result);
    });
    DualContouring::taskQueue.pushTask(aoTask);

    return id;
}
uint32_t DCInstance::createGrassSplatAsync(const vm::ivec2 &worldPositionXZ, const int lod)
{
    uint32_t id = DualContouring::resultQueue.getNextId();

    Task *grassSplatTask = new Task([
        this,
        id,
        worldPositionXZ,
        lod
    ]() -> void {
        uint8_t *result = createGrassSplat(worldPositionXZ, lod);
        DualContouring::resultQueue.pushResult(id, result);
    });
    DualContouring::taskQueue.pushTask(grassSplatTask);

    return id;
}
uint32_t DCInstance::createVegetationSplatAsync(const vm::ivec2 &worldPositionXZ, const int lod)
{
    uint32_t id = DualContouring::resultQueue.getNextId();

    Task *vegetationSplatTask = new Task([
        this,
        id,
        worldPositionXZ,
        lod
    ]() -> void {
        void *result = createVegetationSplat(worldPositionXZ, lod);
        DualContouring::resultQueue.pushResult(id, result);
    });
    DualContouring::taskQueue.pushTask(vegetationSplatTask);

    return id;
}
uint32_t DCInstance::createMobSplatAsync(const vm::ivec2 &worldPositionXZ, const int lod)
{
    uint32_t id = DualContouring::resultQueue.getNextId();

    Task *mobSplatTask = new Task([
        this,
        id,
        worldPositionXZ,
        lod
    ]() -> void {
        void *result = createMobSplat(worldPositionXZ, lod);
        DualContouring::resultQueue.pushResult(id, result);
    });
    DualContouring::taskQueue.pushTask(mobSplatTask);

    return id;
}

//

/* std::vector<Promise *> DCInstance::ensureChunks2D(const vm::ivec2 &position2D, int minChunkDelta, int maxChunkDelta, int lod, GenerateFlags flags) {
    std::vector<vm::ivec2> chunkPositions2D = getChunkRangeInclusive(position2D, minChunkDelta, maxChunkDelta, chunkSize);
    std::vector<Promise *> promises(chunkPositions2D.size());
    for (int i = 0; i < chunkPositions2D.size(); i++) {
        const vm::ivec2 &position2D = chunkPositions2D[i];

        MultiChunkLock *multiChunkLock = new MultiChunkLock(this);
        multiChunkLock->pushPosition(position2D, lod, flags);

        Promise *promise = new Promise();
        promises[i] = promise;

        Task *task = new Task(multiChunkLock, [
            this,
            position2D,
            lod,
            flags,
            promise
        ]() -> void {
            ensureChunk(position2D, lod, flags);
            promise->resolve();
        });
        DualContouring::taskQueue.pushTask(task);
    }
    return promises;
}
void DCInstance::ensureChunk(const vm::ivec2 &position2D, int lod, GenerateFlags flags) {
    Chunk2D &chunk = getChunk(position2D, lod, flags);
}
void DCInstance::ensureChunk(const vm::ivec3 &position3D, int lod, GenerateFlags flags) {
    Chunk3D &chunk = getChunk(position3D, lod, flags);
} */

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

bool DCInstance::addSphereDamage(const float &x, const float &y, const float &z, const float radius)
{
    return false;
    /* const int &gridPoints = DualContouring::gridPoints;

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