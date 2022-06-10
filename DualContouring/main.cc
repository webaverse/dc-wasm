#include "main.h"
#include "octree.h"

// namespace ChunkMesh
// {
//     OctreeNode *chunkRoot;
//     OctreeNode *chunkWithLod;
//     std::vector<OctreeNode *> neighbouringChunks;
//     OctreeNode *seamRoot;
// };

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
    Chunk &getChunk(const vm::ivec3 &min, GenerateFlags flags, const int &lod)
    {
        uint64_t minHash = hashOctreeMin(min);

        const auto &iter = chunksNoiseHashMap.find(minHash);
        if (iter == chunksNoiseHashMap.end())
        {
            chunksNoiseHashMap.emplace(std::make_pair(minHash, Chunk(min, lod, GF_NONE)));
        }

        Chunk &chunkNoise = chunksNoiseHashMap.find(minHash)->second;
        chunkNoise.generate(flags);
        return chunkNoise;
    }
    Chunk &getChunkAt(const float x, const float y, const float z, GenerateFlags flags, const int &lod)
    {
        vm::ivec3 min = vm::ivec3(
                            (int)std::floor(x / (float)chunkSize),
                            (int)std::floor(y / (float)chunkSize),
                            (int)std::floor(z / (float)chunkSize)) *
                        chunkSize;
        return getChunk(min, flags, lod);
    }
    Chunk &getChunkAt(const float x, const float z, GenerateFlags flags, const int &lod)
    {
        vm::ivec3 min = vm::ivec3(
            (int)std::floor(x / (float)chunkSize),
            0,
            (int)std::floor(z / (float)chunkSize)
        ) * chunkSize;
        return getChunk(min, flags, lod);
    }
    /* float *getChunkHeightField(float x, float y, float z) {
        const vm::ivec3 min = vm::ivec3(x, y, z);
        Chunk &chunkNoise = getChunk(min, GF_HEIGHTFIELD, lod);

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
    } */
    /* float getHeight(float x, float z, const int &lod) {
        Chunk &chunkNoise = getChunkAt(x, z, GF_HEIGHTFIELD, lod);
        float height = chunkNoise.getCachedInterpolatedHeight(x, z);
        return height;
    }
    void getHeights(float *vec2s, int count, float *heights, const int &lod) {
        for (int i = 0; i < count; i++) {
            float x = vec2s[i * 2 + 0];
            float z = vec2s[i * 2 + 1];
            Chunk &chunkNoise = getChunkAt(x, z, GF_HEIGHTFIELD, lod);
            heights[i] = chunkNoise.getCachedInterpolatedHeight(x, z);
        }        
    } */
    void getHeightfieldRange(int x, int z, int w, int h, int lod, float *heights) {
        for (int dz = 0; dz < h; dz++) {
            for (int dx = 0; dx < w; dx++) {
                int ax = x + dx;
                int az = z + dz;

                Chunk &chunkNoise = getChunkAt(ax, az, GF_HEIGHTFIELD, lod);
                float height = chunkNoise.getCachedInterpolatedHeight(ax, az);
                float caveValue = chunkNoise.getComputedCaveNoise(ax, height, az) * 1.1f;
                heights[dz * w + dx] = height - caveValue;
            }
        }
    }
    void getAoFieldRange(int x, int y, int z, int w, int h, int d, int lod, unsigned char *aos) {
        for (int dy = 0; dy < h; dy++) {
            for (int dz = 0; dz < d; dz++) {
                for (int dx = 0; dx < w; dx++) {
                    // absolute
                    int ax = x + dx;
                    int ay = x + dx;
                    int az = z + dz;

                    Chunk &chunkNoise = getChunkAt(ax, az, GF_AOFIELD, lod);

                    // chunk-local
                    int lx = ax - chunkNoise.min.x;
                    int ly = ay - chunkNoise.min.y;
                    int lz = az - chunkNoise.min.z;

                    int aoIndex = dx + dz * w + dy * w * d;
                    aos[aoIndex] = chunkNoise.getAoLocal(lx, ly, lz);
                }
            }
        }
    }
    void createGrassSplat(float x, float z, int lod, float *ps, float *qs, float *instances, unsigned int *count) {
        unsigned int &countBinding = *count;
        countBinding = 0;

        Chunk &chunk = getChunkAt(x, z, GF_HEIGHTFIELD, lod);

        float seed = DualContouring::noises->grassNoise.in2D(chunk.min.x, chunk.min.z);
        unsigned int seedInt;
        memcpy(&seedInt, &seed, sizeof(unsigned int));
        std::mt19937 rng(seedInt);

        const int maxNumGrasses = 4 * 1024;
        for (int i = 0; i < maxNumGrasses; i++) {
            float dx = (float)rng() / (float)0xFFFFFFFF * (float)chunkSize;
            float dz = (float)rng() / (float)0xFFFFFFFF * (float)chunkSize;

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
    void createVegetationSplat(float x, float z, int lod, float *ps, float *qs, float *instances, unsigned int *count) {
        unsigned int &countBinding = *count;
        countBinding = 0;

        Chunk &chunk = getChunkAt(x, z, GF_HEIGHTFIELD, lod);

        float seed = DualContouring::noises->vegetationNoise.in2D(chunk.min.x, chunk.min.z);
        unsigned int seedInt;
        memcpy(&seedInt, &seed, sizeof(unsigned int));
        std::mt19937 rng(seedInt);
        
        const int maxNumVeggies = 128;
        const float veggieRate = 0.35;
        for (int i = 0; i < maxNumVeggies; i++) {
            float dx = (float)rng() / (float)0xFFFFFFFF * (float)chunkSize;
            float dz = (float)rng() / (float)0xFFFFFFFF * (float)chunkSize;

            float ax = (float)chunk.min.x + dx;
            float az = (float)chunk.min.z + dz;

            float noiseValue = DualContouring::noises->vegetationNoise.in2D(ax, az);

            if (noiseValue < veggieRate) {
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
    void createMobSplat(float x, float z, int lod, float *ps, float *qs, float *instances, unsigned int *count) {
        unsigned int &countBinding = *count;
        countBinding = 0;

        Chunk &chunk = getChunkAt(x, z, GF_HEIGHTFIELD, lod);

        float seed = DualContouring::noises->mobNoise.in2D(chunk.min.x, chunk.min.z);
        unsigned int seedInt;
        memcpy(&seedInt, &seed, sizeof(unsigned int));
        std::mt19937 rng(seedInt);

        const int maxNumMobs = 2;
        const float mobRate = 0.4;
        for (int i = 0; i < maxNumMobs; i++) {
            float dx = (float)rng() / (float)0xFFFFFFFF * (float)chunkSize;
            float dz = (float)rng() / (float)0xFFFFFFFF * (float)chunkSize;
            
            float ax = (float)chunk.min.x + dx;
            float az = (float)chunk.min.z + dz;
            
            float noiseValue = DualContouring::noises->mobNoise.in2D(ax, az);
            
            if (noiseValue < mobRate) {
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
    unsigned char getBiome(const vm::ivec2 &worldPosition, const int &lod) {
        Chunk &chunkNoise = getChunkAt(worldPosition.x, worldPosition.y, GF_BIOMES, lod);
        int lx = int(worldPosition.x) - chunkNoise.min.x;
        // int ly = int(worldPosition.y) - chunkNoise.min.z;
        int lz = int(worldPosition.y) - chunkNoise.min.z;
        return chunkNoise.getCachedBiome(lx, /*ly,*/ lz);
    }
    /* unsigned char getComputedBiome(const vm::ivec3 &worldPosition, const int &lod) {
        Chunk &chunkNoise = getChunkAt(worldPosition.x, worldPosition.z, GF_BIOMES, lod);
        int lx = int(worldPosition.x) - chunkNoise.min.x;
        int ly = int(worldPosition.y) - chunkNoise.min.z;
        int lz = int(worldPosition.y) - chunkNoise.min.z;
        return chunkNoise.getBiome(lx, ly, lz);
    } */
    float getComputedBiomeHeight(unsigned char b, const vm::vec2 &worldPosition, const int &lod) {
        const Biome &biome = BIOMES[b];
        float ax = worldPosition.x;
        float az = worldPosition.y;

        float biomeHeight = biome.baseHeight +
            DualContouring::noises->elevationNoise1.in2D(ax * biome.amps[0][0], az * biome.amps[0][0]) * biome.amps[0][1] +
            DualContouring::noises->elevationNoise2.in2D(ax * biome.amps[1][0], az * biome.amps[1][0]) * biome.amps[1][1] +
            DualContouring::noises->elevationNoise3.in2D(ax * biome.amps[2][0], az * biome.amps[2][0]) * biome.amps[2][1];
        return biomeHeight;
    }
    /* void getBiomesContainedInChunk(int x, int z, unsigned char *biomes, unsigned int *biomesCount, const int &lod) {
        Chunk &chunk = getChunk(vm::ivec3(x, 0, z), GF_BIOMES, lod);
        const std::vector<unsigned char> &result = chunk.getBiomesContainedInChunk();

        for (int i = 0; i < result.size(); i++) {
            biomes[i] = result[i];
        }
        *biomesCount = result.size();
    } */

    // octrees
    void clearChunkRoot(float x, float y, float z)
    {
        const vm::ivec3 octreeMin = vm::ivec3(x, y, z);
        // OctreeNode *chunkRoot = getChunkRootFromHashMap(octreeMin, chunksListHashMap);

        // if (!chunkRoot)
        // {
        //     return;
        // }
        // // std::cout << chunkRoot->lod << std::endl;
        // // sort and remove duplicates
        // // std::sort(neighbourNodes.begin(), neighbourNodes.end());
        // neighbourNodes.erase(std::unique(neighbourNodes.begin(), neighbourNodes.end()), neighbourNodes.end());
        // for (int i = 0; i < neighbourNodes.size(); i++)
        // {
        //     const OctreeNode *node = neighbourNodes[i];
        //     if (node->drawInfo)
        //     {
        //         delete node->drawInfo;
        //     }
        //     delete node;
        // }
        // neighbourNodes.clear();
        // destroyOctree(chunkRoot);
        // removeOctreeFromHashMap(octreeMin, chunksListHashMap);
    }

    uint8_t *createChunkMesh(float x, float y, float z, int lodArray[8])
    {
        int lod = lodArray[0];
        const int maxLodNumber = *std::max_element(lodArray, lodArray + 8);
        const vm::ivec3 octreeMin = vm::ivec3(x, y, z);
        // OctreeNode *chunkRoot = getChunkRootFromHashMap(octreeMin, chunksListHashMap);
    
        Chunk &chunk = getChunk(octreeMin, GF_SDF, maxLodNumber);
        ChunkOctree chunkOctree(chunk, lodArray);
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

        return constructOutputBuffer(vertexBuffer);
    }

    bool drawSphereDamage(const float &x, const float &y, const float &z,
                          const float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages,
                          const int &lod)
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

                        Chunk &chunkNoise = getChunk(min, GF_SDF, lod);
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

    bool eraseSphereDamage(const float &x, const float &y, const float &z,
                           const float radius, float *outPositions, unsigned int *outPositionsCount, float *outDamages,
                           const int &lod)
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

                        Chunk &chunkNoise = getChunk(min, GF_SDF, lod);
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
                    vm::ivec3 min = vm::ivec3(std::floor(ax / (float)chunkSize), std::floor(ay / (float)chunkSize), std::floor(az / (float)chunkSize)) * chunkSize;
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
                    vm::ivec3 min = vm::ivec3(std::floor(ax / (float)chunkSize), std::floor(ay / (float)chunkSize), std::floor(az / (float)chunkSize)) * chunkSize;
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

    void injectDamage(const float &x, const float &y, const float &z, float *damageBuffer, const int &lod)
    {
        const vm::ivec3 min = vm::ivec3(x, y, z);
        Chunk &chunk = getChunk(min, GF_NONE, lod);

        int gridSize = chunkSize + 3;
        std::vector<float> sdf(gridSize * gridSize * gridSize);
        memcpy(sdf.data(), damageBuffer, sdf.size() * sizeof(float));
        chunk.cachedSdf = std::move(sdf);
    }
}