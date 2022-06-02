#include "main.h"
#include "octree.h"

// namespace ChunkMesh
// {
//     OctreeNode *chunkRoot;
//     OctreeNode *chunkWithLod;
//     std::vector<OctreeNode *> neighbouringChunks;
//     OctreeNode *seamRoot;
// };

struct AllocationsMetrics
{
    uint32_t TotalAllocated = 0;
    uint32_t TotalFreed = 0;
    uint32_t CurrentUsage() { return TotalAllocated - TotalFreed; }
};

static AllocationsMetrics sAllocationsMetrics;

void *operator new(size_t size)
{
    // std::cout << "ALLOCATING : " << size << std::endl;
    sAllocationsMetrics.TotalAllocated += size;
    return std::malloc(size);
}
void operator delete(void *memory, size_t size)
{
    // std::cout << "FREEING : " << size << std::endl;
    sAllocationsMetrics.TotalFreed += size;
    std::free(memory);
}

static void PrintMemoryUsage(){
    std::cout << "Memory Usage : " << sAllocationsMetrics.CurrentUsage() << std::endl;
}

namespace DualContouring
{
    // chunk settings
    int chunkSize = 16;
    Noises *noises = nullptr;

    // storing the octrees that we would delete after mesh construction
    std::vector<OctreeNode *> neighbourNodes;

    // storing the octree roots here for search
    std::unordered_map<uint64_t, OctreeNode *> chunksListHashMap;
    std::unordered_map<uint64_t, Chunk> chunksNoiseHashMap;

    void initialize(int newChunkSize, int seed)
    {
        chunkSize = newChunkSize;

        noises = new Noises(seed);
    }

    // geometries
    uint8_t *constructOutputBuffer(VertexBuffer &vertexBuffer)
    {
        return vertexBuffer.getBuffer();
    }

    // chunks
    Chunk &getChunk(const vm::ivec3 &min, GenerateFlags flags)
    {
        uint64_t minHash = hashOctreeMin(min);

        const auto &iter = chunksNoiseHashMap.find(minHash);
        if (iter == chunksNoiseHashMap.end())
        {
            chunksNoiseHashMap.emplace(std::make_pair(minHash, Chunk(min, GF_NONE)));
        }

        Chunk &chunkNoise = chunksNoiseHashMap.find(minHash)->second;
        chunkNoise.generate(flags);
        return chunkNoise;
    }
    Chunk &getChunkAt(const float x, const float y, const float z, GenerateFlags flags)
    {
        vm::ivec3 min = vm::ivec3(
            (int)std::floor(x / (float)chunkSize),
            (int)std::floor(y / (float)chunkSize),
            (int)std::floor(z / (float)chunkSize)
        ) * chunkSize;
        return getChunk(min, flags);
    }
    Chunk &getChunkAt(int x, int z, GenerateFlags flags)
    {
        vm::ivec3 min = vm::ivec3(
            (int)std::floor((float)x / (float)chunkSize),
            0,
            (int)std::floor((float)z / (float)chunkSize)
        ) * chunkSize;
        return getChunk(min, flags);
    }
    float *getChunkHeightField(float x, float y, float z) {
        const vm::ivec3 min = vm::ivec3(x, y, z);
        Chunk &chunkNoise = getChunk(min, GF_HEIGHTFIELD);
        
        std::vector<float> &heightfieldRaw = chunkNoise.cachedHeightField;
        float *heightfieldOut = (float *)malloc(chunkSize * chunkSize * chunkSize * sizeof(float));
        int gridSize = chunkSize + 3;
        for (int x = 0; x < chunkSize; x++)
        {
            for (int z = 0; z < chunkSize; z++)
            {
                for (int y = 0; y < chunkSize; y++)
                {
                    int lx = x + 1;
                    int ly = y + 1;
                    int lz = z + 1;

                    int outIndex = x + z * chunkSize + y * chunkSize * chunkSize;
                    int inIndex = lx + lz * gridSize + ly * gridSize * gridSize;
                    heightfieldOut[outIndex] = heightfieldRaw[inIndex];
                }
            }
        }
        return heightfieldOut;
    }

    unsigned char getBiome(const vm::ivec2 &worldPosition) {
        Chunk &chunkNoise = getChunkAt(worldPosition.x, worldPosition.y, GF_BIOMES);
        int lx = int(worldPosition.x) - chunkNoise.min.x;
        int lz = int(worldPosition.y) - chunkNoise.min.z;
        return chunkNoise.getBiome(lx, lz);
    }
    float getBiomeHeight(unsigned char b, const vm::vec2 &worldPosition) {
        const Biome &biome = BIOMES[b];
        float ax = worldPosition.x;
        float az = worldPosition.y;
        
        float biomeHeight = std::min<float>(biome.baseHeight +
        DualContouring::noises->elevationNoise1.in2D(ax * biome.amps[0][0], az * biome.amps[0][0]) * biome.amps[0][1] +
        DualContouring::noises->elevationNoise2.in2D(ax * biome.amps[1][0], az * biome.amps[1][0]) * biome.amps[1][1] +
        DualContouring::noises->elevationNoise3.in2D(ax * biome.amps[2][0], az * biome.amps[2][0]) * biome.amps[2][1], 128 - 0.1);
        return biomeHeight;
    }

    // octrees
    void clearChunkRoot(float x, float y, float z)
    {
        // we destroy the chunk root separately because we might need it for LOD switch if it's already generated
        const vm::ivec3 octreeMin = vm::ivec3(x, y, z);
        OctreeNode *chunkRoot = getChunkRootFromHashMap(octreeMin, chunksListHashMap);
        if (!chunkRoot)
        {
            return;
        }
        // sort and remove duplicates
        std::sort(neighbourNodes.begin(), neighbourNodes.end());
        neighbourNodes.erase(std::unique(neighbourNodes.begin(), neighbourNodes.end()), neighbourNodes.end());
        for (int i = 0; i < neighbourNodes.size(); i++)
        {
            const OctreeNode *node = neighbourNodes[i];
            if (node->drawInfo)
            {
                // std::cout << "Draw Info Dead : " << node->drawInfo << std::endl;
                delete node->drawInfo;
            }
            delete node;
        }
        neighbourNodes.clear();
        removeOctreeFromHashMap(octreeMin, chunksListHashMap);
        destroyOctree(chunkRoot);
        std::cout << "Chunk Deletion :" << std::endl;
        PrintMemoryUsage();
    }

    uint8_t *createChunkMesh(float x, float y, float z, const int lod)
    {
        VertexBuffer vertexBuffer;
        // PrintMemoryUsage();

        const vm::ivec3 octreeMin = vm::ivec3(x, y, z);
        // OctreeNode *chunkRoot = getChunkRootFromHashMap(octreeMin, chunksListHashMap);

        Chunk &chunk = getChunk(octreeMin, GF_ALL);
        ChunkOctree chunkOctree(chunk, octreeMin, chunkSize, lod);
        if (!chunkOctree.root)
        {
            // printf("Chunk Has No Data\n");
            return nullptr;
        }

        generateMeshFromOctree(chunkOctree.root, lod, false, vertexBuffer);

        std::vector<OctreeNode *> seamNodes = generateSeamNodes(chunk, chunkOctree, neighbourNodes);
        OctreeNode *seamRoot = constructOctreeUpwards(seamRoot, seamNodes, chunkOctree.root->min, chunkOctree.root->size * 2);
        generateMeshFromOctree(seamRoot, lod, true, vertexBuffer);

        // mesh is not valid
        if (vertexBuffer.indices.size() == 0)
        {
            // printf("Generated Mesh Is Not Valid\n");
            return nullptr;
        }

        addChunkRootToHashMap(chunkOctree.root, chunksListHashMap);
        std::cout << "Chunk Creation :" << std::endl;
        PrintMemoryUsage();

        return constructOutputBuffer(vertexBuffer);
    }

    bool drawSphereDamage(const float &x, const float &y, const float &z, const float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages)
    {
        unsigned int maxPositionsCount = *outPositionsCount;
        *outPositionsCount = 0;

        bool drew = false;
        std::set<uint64_t> seenHashes;
        for (float dx = -1; dx <= 1; dx += 2)
        {
            for (float dz = -1; dz <= 1; dz += 2)
            {
                for (float dy = -1; dy <= 1; dy += 2)
                {
                    float ax = x + dx * radius;
                    float ay = y + dy * radius;
                    float az = z + dz * radius;
                    vm::ivec3 min = vm::ivec3(std::floor(ax / (float)chunkSize), std::floor(ay / (float)chunkSize), std::floor(az / (float)chunkSize)) * chunkSize;
                    uint64_t minHash = hashOctreeMin(min);
                    if (seenHashes.find(minHash) == seenHashes.end())
                    {
                        seenHashes.insert(minHash);

                        Chunk &chunkNoise = getChunk(min, GF_ALL);
                        if (chunkNoise.addSphereDamage(ax, ay, az, radius))
                        {
                            if (*outPositionsCount < maxPositionsCount)
                            {
                                int gridSize = chunkSize + 3;
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

    bool eraseSphereDamage(const float &x, const float &y, const float &z, const float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages)
    {
        unsigned int maxPositionsCount = *outPositionsCount;
        *outPositionsCount = 0;

        bool drew = false;
        std::set<uint64_t> seenHashes;
        for (float dx = -1; dx <= 1; dx += 2)
        {
            for (float dz = -1; dz <= 1; dz += 2)
            {
                for (float dy = -1; dy <= 1; dy += 2)
                {
                    float ax = x + dx * radius;
                    float ay = y + dy * radius;
                    float az = z + dz * radius;
                    vm::ivec3 min = vm::ivec3(std::floor(ax / (float)chunkSize), std::floor(ay / (float)chunkSize), std::floor(az / (float)chunkSize)) * chunkSize;
                    uint64_t minHash = hashOctreeMin(min);
                    if (seenHashes.find(minHash) == seenHashes.end())
                    {
                        seenHashes.insert(minHash);

                        Chunk &chunkNoise = getChunk(min, GF_ALL);
                        if (chunkNoise.removeSphereDamage(ax, ay, az, radius))
                        {
                            if (*outPositionsCount < maxPositionsCount)
                            {
                                int gridSize = chunkSize + 3;
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

    bool drawCubeDamage(
        float x, float y, float z,
        float qx, float qy, float qz, float qw,
        float sx, float sy, float sz,
        float *outPositions,
        unsigned int *outPositionsCount,
        float *outDamages)
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
                    vm::ivec3 min = vm::ivec3(std::floor(ax / (float)chunkSize), std::floor(ay / (float)chunkSize), std::floor(az / (float)chunkSize)) * chunkSize;
                    uint64_t minHash = hashOctreeMin(min);
                    if (seenHashes.find(minHash) == seenHashes.end())
                    {
                        seenHashes.insert(minHash);

                        Chunk &chunkNoise = getChunk(min, GF_ALL);
                        if (chunkNoise.addCubeDamage(
                                x, y, z,
                                qx, qy, qz, qw,
                                sx, sy, sz))
                        {
                            if (*outPositionsCount < maxPositionsCount)
                            {
                                int gridSize = chunkSize + 3;
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

    bool eraseCubeDamage(
        float x, float y, float z,
        float qx, float qy, float qz, float qw,
        float sx, float sy, float sz,
        float *outPositions,
        unsigned int *outPositionsCount,
        float *outDamages)
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
                    vm::ivec3 min = vm::ivec3(std::floor(ax / (float)chunkSize), std::floor(ay / (float)chunkSize), std::floor(az / (float)chunkSize)) * chunkSize;
                    uint64_t minHash = hashOctreeMin(min);
                    if (seenHashes.find(minHash) == seenHashes.end())
                    {
                        seenHashes.insert(minHash);

                        Chunk &chunkNoise = getChunk(min, GF_ALL);
                        if (chunkNoise.removeCubeDamage(
                                x, y, z,
                                qx, qy, qz, qw,
                                sx, sy, sz))
                        {
                            if (*outPositionsCount < maxPositionsCount)
                            {
                                int gridSize = chunkSize + 3;
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

    void injectDamage(const float &x, const float &y, const float &z, float *damageBuffer)
    {
        const vm::ivec3 min = vm::ivec3(x, y, z);
        Chunk &chunk = getChunk(min, GF_NONE);

        int gridSize = chunkSize + 3;
        std::vector<float> sdf(gridSize * gridSize * gridSize);
        memcpy(sdf.data(), damageBuffer, sdf.size() * sizeof(float));
        chunk.cachedSdf = std::move(sdf);
    }
}