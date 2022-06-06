#define _USE_MATH_DEFINES
#include "../math/vectorMath.h"
#include "../math/qef.h"
#include "mesh.h"
#include <iostream>
#include <vector>
#include <functional>
#ifndef OCTREE_H
#define OCTREE_H

#include <algorithm>
#include <stdint.h>
#include <cstdlib>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include "../chunk/chunk.h"

uint64_t hashOctreeMin(const vm::ivec3 &min);

struct VertexInfo
{
    int index;
    vm::ivec4 biome;
    vm::vec4 biomeWeights;
    vm::vec3 position;
    vm::vec3 normal;
};

class OctreeNode
{
    std::weak_ptr<OctreeNode> root;
    std::unique_ptr<OctreeNode> children[8];
    std::unique_ptr<VertexInfo> vertexInfo;
    vm::ivec3 min;
    int size;
};

class ChunkOctree
{
public:
    std::unique_ptr<OctreeNode> rootNode;
    vm::ivec3 min;
    int minVoxelSize; // determined by level of detail
    int size;
};

#endif // OCTREE_H