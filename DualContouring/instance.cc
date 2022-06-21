#include "instance.h"
#include "main.h"
#include "../vector.h"

// constructor/destructor
DCInstance::DCInstance() {}
DCInstance::~DCInstance() {}

// chunks
Chunk &DCInstance::getChunk(const vm::ivec3 &min, GenerateFlags flags, const int &lod)
{
    uint64_t minHash = hashOctreeMin(min);

    const auto &iter = chunksNoiseHashMap.find(minHash);
    if (iter == chunksNoiseHashMap.end())
    {
        chunksNoiseHashMap.emplace(std::make_pair(minHash, Chunk(min, lod)));
    }

    Chunk &chunkNoise = chunksNoiseHashMap.find(minHash)->second;
    chunkNoise.generate(this, flags);
    return chunkNoise;
}
Chunk &DCInstance::getChunkAt(const float x, const float y, const float z, GenerateFlags flags, const int &lod)
{
    vm::ivec3 min = vm::ivec3(
                        (int)std::floor(x / (float)DualContouring::chunkSize),
                        (int)std::floor(y / (float)DualContouring::chunkSize),
                        (int)std::floor(z / (float)DualContouring::chunkSize)) *
                    DualContouring::chunkSize;
    return getChunk(min, flags, lod);
}
Chunk &DCInstance::getChunkAt(const float x, const float z, GenerateFlags flags, const int &lod)
{
    vm::ivec3 min = vm::ivec3(
                        (int)std::floor(x / (float)DualContouring::chunkSize),
                        0,
                        (int)std::floor(z / (float)DualContouring::chunkSize)) *
                    DualContouring::chunkSize;
    return getChunk(min, flags, lod);
}

// chunks
void DCInstance::getHeightfieldRange(int x, int z, int w, int h, int lod, float *heights)
{
    for (int dz = 0; dz < h; dz++)
    {
        for (int dx = 0; dx < w; dx++)
        {
            int ax = x + dx;
            int az = z + dz;

            Chunk &chunkNoise = getChunkAt(ax, az, GF_HEIGHTFIELD, lod);
            float height = chunkNoise.getCachedInterpolatedHeight(ax, az);
            float caveValue = DualContouring::getComputedCaveNoise(ax, height, az) * 1.1f;
            heights[dz * w + dx] = height - caveValue;
        }
    }
}
void DCInstance::getChunkSkylight(int x, int y, int z, int lod, unsigned char *skylights)
{
    const vm::ivec3 octreeMin = vm::ivec3(x, y, z);
    Chunk &chunkNoise = getChunk(octreeMin, GF_AOFIELD, lod);
    chunkNoise.getCachedSkylight(skylights);
}
void DCInstance::getChunkAo(int x, int y, int z, int lod, unsigned char *aos)
{
    const vm::ivec3 octreeMin = vm::ivec3(x, y, z);
    Chunk &chunkNoise = getChunk(octreeMin, GF_AOFIELD, lod);
    chunkNoise.getCachedAo(aos);
}
void DCInstance::getSkylightFieldRange(int x, int y, int z, int w, int h, int d, int lod, unsigned char *skylights)
{
    for (int dz = 0; dz < d; dz++)
    {
        for (int dy = 0; dy < h; dy++)
        {
            for (int dx = 0; dx < w; dx++)
            {
                // absolute
                int ax = x + dx;
                int ay = y + dy;
                int az = z + dz;

                Chunk &chunkNoise = getChunkAt(ax, ay, az, GF_AOFIELD, lod);

                // chunk-local
                int lx = ax - chunkNoise.min.x + 1;
                int ly = ay - chunkNoise.min.y + 1;
                int lz = az - chunkNoise.min.z + 1;
                int skylightIndex = dx + dy * w + dz * w * h; // note: output is y-first, but storage is z-first

                skylights[skylightIndex] = chunkNoise.getSkylightLocal(lx, ly, lz);
            }
        }
    }
}
void DCInstance::getAoFieldRange(int x, int y, int z, int w, int h, int d, int lod, unsigned char *aos)
{
    for (int dz = 0; dz < d; dz++)
    {
        for (int dy = 0; dy < h; dy++)
        {
            for (int dx = 0; dx < w; dx++)
            {
                // absolute
                int ax = x + dx;
                int ay = y + dy;
                int az = z + dz;

                Chunk &chunkNoise = getChunkAt(ax, ay, az, GF_AOFIELD, lod);

                // chunk-local
                int lx = ax - chunkNoise.min.x;
                int ly = ay - chunkNoise.min.y;
                int lz = az - chunkNoise.min.z;
                int aoIndex = dx + dy * w + dz * w * h; // note: output is y-first, but storage is z-first

                aos[aoIndex] = chunkNoise.getAoLocal(lx, ly, lz);
            }
        }
    }
}
void DCInstance::createGrassSplat(float x, float z, int lod, float *ps, float *qs, float *instances, unsigned int *count)
{
    unsigned int &countBinding = *count;
    countBinding = 0;

    Chunk &chunk = getChunkAt(x, z, GF_HEIGHTFIELD, lod);

    float seed = DualContouring::noises->grassNoise.in2D(chunk.min.x, chunk.min.z);
    unsigned int seedInt;
    memcpy(&seedInt, &seed, sizeof(unsigned int));
    std::mt19937 rng(seedInt);

    const int maxNumGrasses = 4 * 1024;
    for (int i = 0; i < maxNumGrasses; i++)
    {
        float dx = (float)rng() / (float)0xFFFFFFFF * (float)DualContouring::chunkSize;
        float dz = (float)rng() / (float)0xFFFFFFFF * (float)DualContouring::chunkSize;

        float ax = (float)chunk.min.x + dx;
        float az = (float)chunk.min.z + dz;

        float height = chunk.getCachedInterpolatedHeight(ax, az);
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
void DCInstance::createVegetationSplat(float x, float z, int lod, float *ps, float *qs, float *instances, unsigned int *count)
{
    unsigned int &countBinding = *count;
    countBinding = 0;

    Chunk &chunk = getChunkAt(x, z, GF_HEIGHTFIELD, lod);

    float seed = DualContouring::noises->vegetationNoise.in2D(chunk.min.x, chunk.min.z);
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
        float az = (float)chunk.min.z + dz;

        float noiseValue = DualContouring::noises->vegetationNoise.in2D(ax, az);

        if (noiseValue < veggieRate)
        {
            int index = 0;

            float height = chunk.getCachedInterpolatedHeight(ax, az);
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
void DCInstance::createMobSplat(float x, float z, int lod, float *ps, float *qs, float *instances, unsigned int *count)
{
    unsigned int &countBinding = *count;
    countBinding = 0;

    Chunk &chunk = getChunkAt(x, z, GF_HEIGHTFIELD, lod);

    float seed = DualContouring::noises->mobNoise.in2D(chunk.min.x, chunk.min.z);
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
        float az = (float)chunk.min.z + dz;

        float noiseValue = DualContouring::noises->mobNoise.in2D(ax, az);

        if (noiseValue < mobRate)
        {
            float height = chunk.getCachedInterpolatedHeight(ax, az);
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
unsigned char DCInstance::getBiome(const vm::ivec2 &worldPosition, const int &lod)
{
    Chunk &chunkNoise = getChunkAt(worldPosition.x, worldPosition.y, GF_BIOMES, lod);
    int lx = int(worldPosition.x) - chunkNoise.min.x;
    // int ly = int(worldPosition.y) - chunkNoise.min.z;
    int lz = int(worldPosition.y) - chunkNoise.min.z;
    return chunkNoise.getCachedBiome(lx, /*ly,*/ lz);
}

uint8_t *DCInstance::createChunkMesh(float x, float y, float z, int lodArray[8])
{
    int lod = lodArray[0];
    // const int maxLodNumber = *std::max_element(lodArray, lodArray + 8);
    const vm::ivec3 octreeMin = vm::ivec3(x, y, z);
    // OctreeNode *chunkRoot = getChunkRootFromHashMap(octreeMin, chunksListHashMap);

    Chunk &chunk = getChunk(octreeMin, GF_SDF, lod);
    ChunkOctree chunkOctree(this, chunk, lodArray);
    if (!chunkOctree.root)
    {
        // printf("Chunk Has No Data\n");
        return nullptr;
    }
    VertexBuffer vertexBuffer;
    generateMeshFromOctree(chunkOctree.root, vertexBuffer, false);
    generateMeshFromOctree(chunkOctree.seamRoot, vertexBuffer, true);

    if (vertexBuffer.indices.size() == 0)
    {
        // printf("Generated Mesh Is Not Valid\n");
        return nullptr;
    }

    return DualContouring::constructOutputBuffer(vertexBuffer);
}

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
                
                uint64_t minHash = hashOctreeMin(min);
                if (seenHashes.find(minHash) == seenHashes.end())
                {
                    seenHashes.insert(minHash);

                    Chunk &chunkNoise = getChunk(min, GF_SDF, lod);
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

                uint64_t minHash = hashOctreeMin(min);
                if (seenHashes.find(minHash) == seenHashes.end())
                {
                    seenHashes.insert(minHash);

                    Chunk &chunkNoise = getChunk(min, GF_SDF, lod);
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
                uint64_t minHash = hashOctreeMin(min);
                if (seenHashes.find(minHash) == seenHashes.end())
                {
                    seenHashes.insert(minHash);

                    Chunk &chunkNoise = getChunk(min, GF_SDF, lod);
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
                uint64_t minHash = hashOctreeMin(min);
                if (seenHashes.find(minHash) == seenHashes.end())
                {
                    seenHashes.insert(minHash);

                    Chunk &chunkNoise = getChunk(min, GF_SDF, lod);
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
    Chunk &chunk = getChunk(min, GF_NONE, lod);
    chunk.injectDamage(damageBuffer);
}

void DCInstance::setRange(const vm::ivec3 &min, const vm::ivec3 &max)
{
    range.reset(new vm::ibox3(
        vm::ivec3(min.x, min.y, min.z),
        vm::ivec3(max.x, max.y, max.z)));
    // rangeMin = min;
    // rangeMax = max;
    // hasRange = true;
}