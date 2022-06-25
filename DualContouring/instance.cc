#include "instance.h"
#include "main.h"
#include "../vector.h"

// constructor/destructor
DCInstance::DCInstance() :
  taskQueue(this)
  {}
DCInstance::~DCInstance() {}

// chunks
// 3d
Chunk3D &DCInstance::getChunk(const vm::ivec3 &min, const int lod, GenerateFlags flags)
{
    uint64_t minHash = hashOctreeMinLod(min, lod);

    const auto &iter = chunksCache3D.find(minHash);
    if (iter == chunksCache3D.end())
    {
        vm::ivec2 min2D(min.x, min.z);
        Chunk2D *chunk2d = &getChunk(min2D, lod, flags);
        chunksCache3D.emplace(std::make_pair(minHash, Chunk3D(min, lod, chunk2d)));
    }

    Chunk3D &chunkNoise = chunksCache3D.find(minHash)->second;
    chunkNoise.generate(this, flags);
    return chunkNoise;
}
Chunk3D &DCInstance::getChunkAt(const float x, const float y, const float z, const int lod, GenerateFlags flags)
{
    vm::ivec3 min = vm::ivec3(
                        (int)std::floor(x / (float)DualContouring::chunkSize),
                        (int)std::floor(y / (float)DualContouring::chunkSize),
                        (int)std::floor(z / (float)DualContouring::chunkSize)) *
                    DualContouring::chunkSize;
    return getChunk(min, lod, flags);
}

// 2d
Chunk2D &DCInstance::getChunk(const vm::ivec2 &min, const int lod, GenerateFlags flags) {
    uint64_t minHash = hashOctreeMinLod(min, lod);

    const auto &iter = chunksCache2D.find(minHash);
    if (iter == chunksCache2D.end())
    {
        chunksCache2D.emplace(std::make_pair(minHash, Chunk2D(min, lod)));
    }

    Chunk2D &chunkNoise = chunksCache2D.find(minHash)->second;
    chunkNoise.generate(this, flags);
    return chunkNoise;
}
Chunk2D &DCInstance::getChunkAt(const float x, const float z, const int lod, GenerateFlags flags)
{
    vm::ivec2 min = vm::ivec2(
                        (int)std::floor(x / (float)DualContouring::chunkSize),
                        (int)std::floor(z / (float)DualContouring::chunkSize)) *
                    DualContouring::chunkSize;
    return getChunk(min, lod, flags);
}

// locks
std::mutex &DCInstance::getChunkLock(const vm::ivec3 &worldPos, const int lod) {
    vm::ivec3 min(worldPos.x, 0, worldPos.z);
    uint64_t minLodHash = hashOctreeMinLod(min, lod);
    std::mutex &chunkLock = chunkLocks[minLodHash];
    return chunkLock;
}

// fields
void DCInstance::getChunkHeightfield(const vm::ivec2 &worldPositionXZ, int lod, float *heights) {
    Chunk2D &chunkNoise = getChunk(worldPositionXZ, lod, GF_HEIGHTFIELD);
    chunkNoise.getCachedHeightfield(heights);
}
void DCInstance::getChunkSkylight(const vm::ivec3 &worldPosition, int lod, unsigned char *skylights)
{
    Chunk3D &chunkNoise = getChunk(worldPosition, lod, GF_AOFIELD);
    chunkNoise.getCachedSkylight(skylights);
}
void DCInstance::getChunkAo(const vm::ivec3 &worldPosition, int lod, unsigned char *aos)
{
    Chunk3D &chunkNoise = getChunk(worldPosition, lod, GF_AOFIELD);
    chunkNoise.getCachedAo(aos);
}
void DCInstance::createGrassSplat(const vm::ivec2 &worldPositionXZ, const int lod, float *ps, float *qs, float *instances, unsigned int *count)
{
    unsigned int &countBinding = *count;
    countBinding = 0;

    Chunk2D &chunk = getChunk(worldPositionXZ, lod, GF_HEIGHTFIELD);

    float seed = DualContouring::noises->grassNoise.in2D(chunk.min.x, chunk.min.y);
    unsigned int seedInt;
    memcpy(&seedInt, &seed, sizeof(unsigned int));
    std::mt19937 rng(seedInt);

    const int maxNumGrasses = 4 * 1024;
    for (int i = 0; i < maxNumGrasses; i++)
    {
        float dx = (float)rng() / (float)0xFFFFFFFF * (float)DualContouring::chunkSize;
        float dz = (float)rng() / (float)0xFFFFFFFF * (float)DualContouring::chunkSize;

        float ax = (float)chunk.min.x + dx;
        float az = (float)chunk.min.y + dz;

        int idx = (int)dx + 1;
        int idz = (int)dz + 1;
        int index2D = idx + idz * DualContouring::chunkSize;
        float height = chunk.cachedHeightField[index2D];

        ps[countBinding * 3] = ax;
        ps[countBinding * 3 + 1] = height;
        ps[countBinding * 3 + 2] = az;

        Quat q = Quat().setFromAxisAngle(Vec{0, 1, 0}, rng() * 2.0f * M_PI);
        qs[countBinding * 4] = q.x;
        qs[countBinding * 4 + 1] = q.y;
        qs[countBinding * 4 + 2] = q.z;
        qs[countBinding * 4 + 3] = q.w;

        instances[countBinding] = (float)rng() / (float)0xFFFFFFFF;

        countBinding++;
    }
}
void DCInstance::createVegetationSplat(const vm::ivec2 &worldPositionXZ, const int lod, float *ps, float *qs, float *instances, unsigned int *count)
{
    unsigned int &countBinding = *count;

    countBinding = 0;
    Chunk2D &chunk = getChunk(worldPositionXZ, lod, GF_HEIGHTFIELD);

    float seed = DualContouring::noises->vegetationNoise.in2D(chunk.min.x, chunk.min.y);
    unsigned int seedInt;
    memcpy(&seedInt, &seed, sizeof(unsigned int));
    std::mt19937 rng(seedInt);

    const int maxNumVeggies = 128;
    const float veggieRate = 0.35;
    for (int i = 0; i < maxNumVeggies; i++)
    {
        float dx = (float)rng() / (float)0xFFFFFFFF * (float)DualContouring::chunkSize;
        float dz = (float)rng() / (float)0xFFFFFFFF * (float)DualContouring::chunkSize;

        float ax = (float)chunk.min.x + dx;
        float az = (float)chunk.min.y + dz;

        float noiseValue = DualContouring::noises->vegetationNoise.in2D(ax, az);

        if (noiseValue < veggieRate)
        {
            int index = 0;

            int idx = (int)dx + 1;
            int idz = (int)dz + 1;
            int index2D = idx + idz * DualContouring::chunkSize;
            float height = chunk.cachedHeightField[index2D];

            ps[countBinding * 3] = ax;
            ps[countBinding * 3 + 1] = height;
            ps[countBinding * 3 + 2] = az;

            Quat q = Quat().setFromAxisAngle(Vec{0, 1, 0}, rng() * 2.0f * M_PI);
            qs[countBinding * 4] = q.x;
            qs[countBinding * 4 + 1] = q.y;
            qs[countBinding * 4 + 2] = q.z;
            qs[countBinding * 4 + 3] = q.w;

            instances[countBinding] = (float)rng() / (float)0xFFFFFFFF;

            countBinding++;
        }
    }
}
void DCInstance::createMobSplat(const vm::ivec2 &worldPositionXZ, const int lod, float *ps, float *qs, float *instances, unsigned int *count)
{
    unsigned int &countBinding = *count;
    countBinding = 0;

    Chunk2D &chunk = getChunk(worldPositionXZ, lod, GF_HEIGHTFIELD);

    float seed = DualContouring::noises->mobNoise.in2D(chunk.min.x, chunk.min.y);
    unsigned int seedInt;
    memcpy(&seedInt, &seed, sizeof(unsigned int));
    std::mt19937 rng(seedInt);

    const int maxNumMobs = 2;
    const float mobRate = 0.4;
    for (int i = 0; i < maxNumMobs; i++)
    {
        float dx = (float)rng() / (float)0xFFFFFFFF * (float)DualContouring::chunkSize;
        float dz = (float)rng() / (float)0xFFFFFFFF * (float)DualContouring::chunkSize;

        float ax = (float)chunk.min.x + dx;
        float az = (float)chunk.min.y + dz;

        float noiseValue = DualContouring::noises->mobNoise.in2D(ax, az);

        if (noiseValue < mobRate)
        {
            int idx = (int)dx + 1;
            int idz = (int)dz + 1;
            int index2D = idx + idz * DualContouring::chunkSize;
            float height = chunk.cachedHeightField[index2D];

            ps[countBinding * 3] = ax;
            ps[countBinding * 3 + 1] = height;
            ps[countBinding * 3 + 2] = az;

            Quat q = Quat().setFromAxisAngle(Vec{0, 1, 0}, rng() * 2.0f * M_PI);
            qs[countBinding * 4] = q.x;
            qs[countBinding * 4 + 1] = q.y;
            qs[countBinding * 4 + 2] = q.z;
            qs[countBinding * 4 + 3] = q.w;

            instances[countBinding] = (float)rng() / (float)0xFFFFFFFF;

            countBinding++;
        }
    }
}

// biomes
// get biome value for a world point
unsigned char DCInstance::getBiome(const vm::vec2 &worldPosition, const int &lod) {
    Chunk2D &chunkNoise = getChunkAt(worldPosition.x, worldPosition.y, lod, GF_BIOMES);
    int lx = (int)worldPosition.x - chunkNoise.min.x;
    int lz = (int)worldPosition.y - chunkNoise.min.y;
    return chunkNoise.getCachedBiome(lx, lz);
}
// get biomes weights for a world point
void DCInstance::getInterpolatedBiomes(const vm::vec2 &worldPosition, const int &lod, vm::ivec4 &biome, vm::vec4 &biomeWeights) {
    Chunk2D &chunkNoise = getChunkAt(worldPosition.x, worldPosition.y, lod, GF_BIOMES);
    chunkNoise.getCachedInterpolatedBiome2D(worldPosition, biome, biomeWeights);
}

//

float DCInstance::getTemperature(const vm::vec2 &worldPosition, const int &lod) {
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
}

float DCInstance::getWater(const vm::vec2 &worldPosition, const int &lod) {
    Chunk2D &chunkNoise = getChunkAt(worldPosition.x, worldPosition.y, lod, GF_WATERFIELD);
    int lx = (int)worldPosition.x - chunkNoise.min.x;
    int lz = (int)worldPosition.y - chunkNoise.min.y;
    return chunkNoise.getWaterFieldLocal(lx, lz);
}

//

std::vector<vm::ivec3> getChunkRangeInclusive(const vm::ivec3 &worldPosition, int minChunkDelta, int maxChunkDelta) {
    std::vector<vm::ivec3> result;
    for (int dy = -minChunkDelta; dy <= maxChunkDelta; dy++)
    {
        for (int dz = -minChunkDelta; dz <= maxChunkDelta; dz++)
        {
            for (int dx = -minChunkDelta; dx <= maxChunkDelta; dx++)
            {
                result.push_back(vm::ivec3(worldPosition.x + dx, worldPosition.y + dy, worldPosition.z + dz));
            }
        }
    }
    return result;
}

//

uint8_t *DCInstance::createTerrainChunkMesh(const vm::ivec3 &worldPosition, const int lodArray[8])
{
    int lod = lodArray[0];
    Chunk3D &chunk = getChunk(worldPosition, lod, GF_SDF);
    ChunkOctree<TerrainDCContext> chunkOctree(this, chunk, lodArray);
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
uint32_t DCInstance::createTerrainChunkMeshAsync(const vm::ivec3 &worldPosition, const int lodArray[8])
{
    uint32_t id = resultQueue.getNextId();

    int lod = lodArray[0];
    std::vector<int> lodVector(lodArray, lodArray + 8);
    std::vector<vm::ivec3> chunkPositions = getChunkRangeInclusive(worldPosition, -1, 1);
    MultiChunkLock multiChunkLock(this, std::move(chunkPositions), lod);
    Task *task = new Task(std::move(multiChunkLock.tryLockFn), std::move(multiChunkLock.unlockFn), [
        this,
        worldPosition,
        lod,
        lodVector = std::move(lodVector),
        id
    ]() -> void {
        uint8_t *result = createTerrainChunkMesh(worldPosition, lodVector.data());
        resultQueue.pushResult(id, result);
    });
    taskQueue.pushTask(task);

    return id;
}

uint8_t *DCInstance::createLiquidChunkMesh(const vm::ivec3 &worldPosition, const int lodArray[8])
{
    int lod = lodArray[0];
    Chunk3D &chunk = getChunk(worldPosition, lod, GF_LIQUIDS);
    ChunkOctree<LiquidDCContext> chunkOctree(this, chunk, lodArray);
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
uint32_t DCInstance::createLiquidChunkMeshAsync(const vm::ivec3 &worldPosition, const int lodArray[8])
{
    uint32_t id = resultQueue.getNextId();

    int lod = lodArray[0];
    std::vector<int> lodVector(lodArray, lodArray + 8);
    std::vector<vm::ivec3> chunkPositions = getChunkRangeInclusive(worldPosition, -1, 1);
    MultiChunkLock multiChunkLock(this, std::move(chunkPositions), lod);
    Task *task = new Task(std::move(multiChunkLock.tryLockFn), std::move(multiChunkLock.unlockFn), [
        this,
        worldPosition,
        lod,
        lodVector = std::move(lodVector),
        id
    ]() -> void {
        uint8_t *result = createLiquidChunkMesh(worldPosition, lodVector.data());
        resultQueue.pushResult(id, result);
    });
    taskQueue.pushTask(task);

    return id;
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
    vm::ivec3 chunkMin = chunkMinForPosition(vm::ivec3(x, y, z));

    for (float dx = -1; dx <= 1; dx += 1)
    {
        for (float dz = -1; dz <= 1; dz += 1)
        {
            for (float dy = -1; dy <= 1; dy += 1)
            {
                vm::ivec3 min = chunkMin + vm::ivec3(dx,dy,dz) * DualContouring::chunkSize;
                
                uint64_t minHash = hashOctreeMinLod(min, lod);
                if (seenHashes.find(minHash) == seenHashes.end())
                {
                    seenHashes.insert(minHash);

                    Chunk3D &chunkNoise = getChunk(min, lod, GF_SDF);
                    if (chunkNoise.addSphereDamage(x, y, z, radius))
                    {
                        if (*outPositionsCount < maxPositionsCount)
                        {
                            int gridSize = DualContouring::chunkSize + 3 + lod;
                            int damageBufferSize = gridSize * gridSize * gridSize;
                            memcpy(outDamages + ((*outPositionsCount) * damageBufferSize), chunkNoise.cachedDamageSdf.data(), sizeof(float) * damageBufferSize);

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
    vm::ivec3 chunkMin = chunkMinForPosition(vm::ivec3(x, y, z));

    for (float dx = -1; dx <= 1; dx += 1)
    {
        for (float dz = -1; dz <= 1; dz += 1)
        {
            for (float dy = -1; dy <= 1; dy += 1)
            {
               vm::ivec3 min = chunkMin + vm::ivec3(dx,dy,dz) * DualContouring::chunkSize;

                uint64_t minHash = hashOctreeMinLod(min, lod);
                if (seenHashes.find(minHash) == seenHashes.end())
                {
                    seenHashes.insert(minHash);

                    Chunk3D &chunkNoise = getChunk(min, lod, GF_SDF);
                    if (chunkNoise.removeSphereDamage(x, y, z, radius))
                    {
                        if (*outPositionsCount < maxPositionsCount)
                        {
                            int gridSize = DualContouring::chunkSize + 3 + lod;
                            int damageBufferSize = gridSize * gridSize * gridSize;
                            memcpy(outDamages + (*outPositionsCount) * damageBufferSize, chunkNoise.cachedSdf.data(), sizeof(float) * damageBufferSize);

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
                vm::ivec3 min = vm::ivec3(
                                    std::floor(ax / (float)DualContouring::chunkSize),
                                    std::floor(ay / (float)DualContouring::chunkSize),
                                    std::floor(az / (float)DualContouring::chunkSize)) *
                                DualContouring::chunkSize;
                uint64_t minHash = hashOctreeMinLod(min, lod);
                if (seenHashes.find(minHash) == seenHashes.end())
                {
                    seenHashes.insert(minHash);

                    Chunk3D &chunkNoise = getChunk(min, lod, GF_SDF);
                    if (chunkNoise.addCubeDamage(
                            x, y, z,
                            qx, qy, qz, qw,
                            sx, sy, sz))
                    {
                        if (*outPositionsCount < maxPositionsCount)
                        {
                            int gridSize = DualContouring::chunkSize + 3 + lod;
                            int damageBufferSize = gridSize * gridSize * gridSize;
                            memcpy(outDamages + (*outPositionsCount) * damageBufferSize, chunkNoise.cachedSdf.data(), sizeof(float) * damageBufferSize);

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
                vm::ivec3 min = vm::ivec3(
                                    std::floor(ax / (float)DualContouring::chunkSize),
                                    std::floor(ay / (float)DualContouring::chunkSize),
                                    std::floor(az / (float)DualContouring::chunkSize)) *
                                DualContouring::chunkSize;
                uint64_t minHash = hashOctreeMinLod(min, lod);
                if (seenHashes.find(minHash) == seenHashes.end())
                {
                    seenHashes.insert(minHash);

                    Chunk3D &chunkNoise = getChunk(min, lod, GF_SDF);
                    if (chunkNoise.removeCubeDamage(
                            x, y, z,
                            qx, qy, qz, qw,
                            sx, sy, sz))
                    {
                        if (*outPositionsCount < maxPositionsCount)
                        {
                            int gridSize = DualContouring::chunkSize + 3 + lod;
                            int damageBufferSize = gridSize * gridSize * gridSize;
                            memcpy(outDamages + (*outPositionsCount) * damageBufferSize, chunkNoise.cachedSdf.data(), sizeof(float) * damageBufferSize);

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

void DCInstance::injectDamage(const float &x, const float &y, const float &z, float *damageBuffer, const int &lod)
{
    const vm::ivec3 min = vm::ivec3(x, y, z);
    Chunk3D &chunk = getChunk(min, lod, GF_NONE);
    chunk.injectDamage(damageBuffer);
}

void DCInstance::setClipRange(const vm::vec3 &min, const vm::vec3 &max)
{
    clipRange.reset(new vm::box3(
        vm::vec3(min.x, min.y, min.z),
        vm::vec3(max.x, max.y, max.z)));
}

//

bool DCInstance::tryLock(const std::vector<vm::ivec3> &chunkPositions, int lod) {
    bool lockedAll = true;
    for (int i = 0; i < chunkPositions.size(); i++) {
        const vm::ivec3 &chunkPosition = chunkPositions[i];
        std::mutex &chunkLock = getChunkLock(chunkPosition, lod);
        if (chunkLock.try_lock()) {
            // nothing
        } else {
            // bail out; unlock all locks
            for (int j = 0; j < i; j++) {
                const vm::ivec3 &chunkPosition = chunkPositions[j];
                std::mutex &chunkLock = getChunkLock(chunkPosition, lod);
                chunkLock.unlock();
            }
            lockedAll = false;
            break;
        }
    }
    return lockedAll;
}
void DCInstance::unlock(const std::vector<vm::ivec3> &chunkPositions, int lod) {
    for (int i = 0; i < chunkPositions.size(); i++) {
        const vm::ivec3 &chunkPosition = chunkPositions[i];
        std::mutex &chunkLock = getChunkLock(chunkPosition, lod);
        chunkLock.unlock();
    }
}
bool DCInstance::tryLock(const vm::ivec3 &chunkPosition, int lod) {
    std::mutex &chunkLock = getChunkLock(chunkPosition, lod);
    return chunkLock.try_lock();
}
void DCInstance::unlock(const vm::ivec3 &chunkPosition, int lod) {
    std::mutex &chunkLock = getChunkLock(chunkPosition, lod);
    chunkLock.unlock();
}