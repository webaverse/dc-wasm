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
    FastNoise *fastNoise = nullptr;

    // storing the octrees that we would delete after mesh construction
    std::vector<OctreeNode *> temporaryNodesList;

    // storing the octree roots here for search
    std::unordered_map<uint64_t, OctreeNode *> chunksListHashMap;
    std::unordered_map<uint64_t, CachedNoise> chunksNoiseHashMap;

    void initialize(int newChunkSize, int seed)
    {
        chunkSize = newChunkSize;

        std::mt19937 rng(seed);
        fastNoise = new FastNoise(rng());
        fastNoise->SetFrequency(0.02);
        fastNoise->SetFractalOctaves(4);
    }
    uint8_t *constructOutputBuffer(VertexBuffer &vertexBuffer)
    {
        return vertexBuffer.getBuffer();
    }
    CachedNoise &getChunkNoise(vm::ivec3 min)
    {
        uint64_t minHash = hashOctreeMin(min);

        const auto &iter = chunksNoiseHashMap.find(minHash);
        if (iter == chunksNoiseHashMap.end())
        {
            chunksNoiseHashMap.emplace(std::make_pair(minHash, CachedNoise(min)));
        }

        CachedNoise &chunkNoise = chunksNoiseHashMap.find(minHash)->second;
        return chunkNoise;
    }
    float *getChunkHeightField(float x, float y, float z) {
        const vm::ivec3 min = vm::ivec3(x, y, z);
        CachedNoise &chunkNoise = getChunkNoise(min);
        std::vector<float> &heightfieldRaw = chunkNoise.cachedHeightField;
        float *heightfieldOut = (float *)malloc(chunkSize * chunkSize * chunkSize * sizeof(float));
        int gridSize = chunkSize + 3;
        for (int x = 0; x < chunkSize; x++) {
            for (int z = 0; z < chunkSize; z++) {
                for (int y = 0; y < chunkSize; y++) {
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
    OctreeNode *generateChunkData(const vm::ivec3 octreeMin, const int lod)
    {
        CachedNoise &chunkNoise = getChunkNoise(octreeMin);
        OctreeNode *chunk = constructOctreeDownwards(octreeMin, lod, chunkNoise);
        // printf("CHUNK DATA IS GENERATED\n");
        return chunk;
    }

    void clearTemporaryChunkData()
    {
        std::vector<OctreeNode *> nodesList;
        for (int i = 0; i < temporaryNodesList.size(); i++)
        {
            addNodesToVector(temporaryNodesList[i], nodesList);
        }
        // sort and remove duplicates
        std::sort(nodesList.begin(), nodesList.end());
        nodesList.erase(std::unique(nodesList.begin(), nodesList.end()), nodesList.end());
        for (int i = 0; i < nodesList.size(); i++)
        {
            const OctreeNode *node = nodesList[i];
            if (node->drawInfo)
            {
                delete node->drawInfo;
            }
            delete node;
        }
        temporaryNodesList.clear();
    }

    void clearChunkRoot(float x, float y, float z)
    {
        // we destroy the chunk root separately because we might need it for LOD switch if it's already generated
        const vm::ivec3 octreeMin = vm::ivec3(x, y, z);
        OctreeNode *chunkRoot = getChunkRootFromHashMap(octreeMin, chunksListHashMap);
        if (!chunkRoot)
        {
            return;
        }
        removeOctreeFromHashMap(octreeMin, chunksListHashMap);
        destroyOctree(chunkRoot);
    }

    uint8_t *createChunkMesh(float x, float y, float z, const int lod)
    {
        VertexBuffer vertexBuffer;

        const vm::ivec3 octreeMin = vm::ivec3(x, y, z);
        // OctreeNode *chunkRoot = getChunkRootFromHashMap(octreeMin, chunksListHashMap);

        OctreeNode *chunkRoot = generateChunkData(octreeMin, lod);
        if (!chunkRoot)
        {
            // printf("Chunk Has No Data\n");
            return nullptr;
        }

        addChunkRootToHashMap(chunkRoot, chunksListHashMap);
        generateMeshFromOctree(chunkRoot, lod, false, vertexBuffer);

        // mesh is not valid
        if(vertexBuffer.indices.size() == 0){
            printf("Generated Mesh Is Not Valid\n");
            return nullptr;
        }

        // std::vector<OctreeNode *> neighbouringChunks;
        // std::vector<OctreeNode *> seamNodes = findSeamNodes(chunkWithLod, neighbouringChunks, chunksListHashMap, getChunkRootFromHashMap);
        // OctreeNode *seamRoot = constructOctreeUpwards(seamRoot, seamNodes, chunkWithLod->min, chunkWithLod->size * 2);
        // generateMeshFromOctree(seamRoot, true, vertexBuffer);

        // adding the chunk clone + neighbouring chunk clones to the destroyable list
        // for (int i = 0; i < neighbouringChunks.size(); i++)
        // {
        //     temporaryNodesList.push_back(neighbouringChunks[i]);
        // }

        // add the seam octree to the destroyable list
        // temporaryNodesList.push_back(seamRoot);

        // add the chunk clone octree to the destroyable list
        // temporaryNodesList.push_back(chunkWithLod);
        // temporaryNodesList.push_back(chunkRoot);
        // printf("CHUNK MESH IS GENERATED");

        return constructOutputBuffer(vertexBuffer);
    }

    bool drawDamageSphere(const float &x, const float &y, const float &z, const float radius, float *outPositions, unsigned int *outPositionsCount)
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

                        CachedNoise &chunkNoise = getChunkNoise(min);
                        if (chunkNoise.addDamage(ax, ay, az, radius))
                        {
                            if (*outPositionsCount < maxPositionsCount)
                            {
                                outPositions[(*outPositionsCount)++] = min.x;
                                outPositions[(*outPositionsCount)++] = min.y;
                                outPositions[(*outPositionsCount)++] = min.z;
                            }

                            drew = true;
                        }
                    }
                }
            }
        }
        return drew;
    }
    
    bool eraseDamageSphere(const float &x, const float &y, const float &z, const float radius, float *outPositions, unsigned int *outPositionsCount) {
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

                        CachedNoise &chunkNoise = getChunkNoise(min);
                        if (chunkNoise.removeDamage(ax, ay, az, radius))
                        {
                            if (*outPositionsCount < maxPositionsCount)
                            {
                                outPositions[(*outPositionsCount)++] = min.x;
                                outPositions[(*outPositionsCount)++] = min.y;
                                outPositions[(*outPositionsCount)++] = min.z;
                            }

                            drew = true;
                        }
                    }
                }
            }
        }
        return drew;
    }

    bool addCubeDamage(
        float x, float y, float z,
        float qx, float qy, float qz, float qw,
        float sx, float sy, float sz,
        float *outPositions,
        unsigned int *outPositionsCount
    ) {
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

                        CachedNoise &chunkNoise = getChunkNoise(min);
                        if (chunkNoise.addCubeDamage(
                            sx, sy, sz,
                            qx, qy, qz, qw,
                            ax, ay, az
                        )) {
                            if (*outPositionsCount < maxPositionsCount)
                            {
                                outPositions[(*outPositionsCount)++] = min.x;
                                outPositions[(*outPositionsCount)++] = min.y;
                                outPositions[(*outPositionsCount)++] = min.z;
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
        unsigned int *outPositionsCount
    ) {
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

                        CachedNoise &chunkNoise = getChunkNoise(min);
                        if (chunkNoise.removeCubeDamage(
                            sx, sy, sz,
                            qx, qy, qz, qw,
                            ax, ay, az
                        )) {
                            if (*outPositionsCount < maxPositionsCount)
                            {
                                outPositions[(*outPositionsCount)++] = min.x;
                                outPositions[(*outPositionsCount)++] = min.y;
                                outPositions[(*outPositionsCount)++] = min.z;
                            }

                            drew = true;
                        }
                    }
                }
            }
        }
        return drew;
    }

    void injectDamage(const float &x, const float &y, const float &z, float *damageBuffer) {
        const vm::ivec3 min = vm::ivec3(x, y, z);
        uint64_t minHash = hashOctreeMin(min);
        const auto &iter = chunksNoiseHashMap.find(minHash);
        if (iter != chunksNoiseHashMap.end())
        {
            CachedNoise &chunkNoise = iter->second;
            chunkNoise.injectDamage(damageBuffer);
        } else {
            std::vector<float> cachedHeightField;

            int gridSize = chunkSize + 3;
            std::vector<float> cachedSdf(gridSize * gridSize * gridSize);
            memcpy(cachedSdf.data(), damageBuffer, cachedSdf.size() * sizeof(float));
            
            CachedNoise chunkNoise(min, std::move(cachedHeightField), std::move(cachedSdf));
            chunkNoise.initHeightField();
            
            chunksNoiseHashMap.emplace(std::make_pair(minHash, std::move(chunkNoise)));
        }
    }
}