#include "instance.h"
#include "main.h"
#include "octree.h"
// #include "lock.h"
// #include "biomes.h"
// #include "tracker.h"
#include "../vector.h"
#include "../util.h"
#include <emscripten.h>
#include "peek.h"

constexpr int CHUNK_RANGE = 1;

// constructor/destructor
DCInstance::DCInstance() {}
DCInstance::~DCInstance() {}

//

uint8_t *DCInstance::createPointCloudMesh(const std::vector<vm::vec3> &pointcloud) {
    TerrainDCContext vertexContext(pointcloud);
    
    ChunkOctree<TerrainDCContext> chunkOctree(this, &vertexContext);
    if (!chunkOctree.root)
    {
        printf("Chunk Has No Data\n");
        return nullptr;
    }
    generateMeshFromOctree<TerrainDCContext, false>(chunkOctree.root, vertexContext);
    // generateMeshFromOctree<TerrainDCContext, true>(chunkOctree.seamRoot, vertexContext);

    auto &vertexBuffer = vertexContext.vertexBuffer;
    if (vertexBuffer.indices.size() == 0)
    {
        printf("Generated Mesh Is Not Valid\n");
        return nullptr;
    }

    // const vm::ivec3 chunkMax = worldPosition + (chunkSize * lod);
    // setPeeks<TerrainDCContext>(this, worldPosition, chunkMax, lod, vertexBuffer.peeks, PEEK_FACE_INDICES.array);

    return vertexBuffer.getBuffer();
}

/* float randomFromPoint(int x, int y, int z) {
    uint64_t hash = hashOctreeMin(vm::ivec3{x, y, z});
    uint32_t hash32 = (uint32_t)hash ^ (uint32_t)(hash >> 32);
    float f = (float)hash32 / (float)UINT32_MAX;
    return f;
} */