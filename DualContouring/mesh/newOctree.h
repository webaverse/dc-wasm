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

#define PI 3.14159265358979323846

const int MATERIAL_AIR = 0;
const int MATERIAL_SOLID = 1;

const float QEF_ERROR = 1e-6f;
const int QEF_SWEEPS = 4;

const vm::ivec3 CHILD_MIN_OFFSETS[] =
    {
        // needs to match the vertMap from Dual Contouring impl
        vm::ivec3(0, 0, 0),
        vm::ivec3(0, 0, 1),
        vm::ivec3(0, 1, 0),
        vm::ivec3(0, 1, 1),
        vm::ivec3(1, 0, 0),
        vm::ivec3(1, 0, 1),
        vm::ivec3(1, 1, 0),
        vm::ivec3(1, 1, 1),
};
// data from the original DC impl, drives the contouring process

const int edgevmap[12][2] =
    {
        {0, 4}, {1, 5}, {2, 6}, {3, 7}, // x-axis
        {0, 2},
        {1, 3},
        {4, 6},
        {5, 7}, // y-axis
        {0, 1},
        {2, 3},
        {4, 5},
        {6, 7} // z-axis
};

const int edgemask[3] = {5, 3, 6};

const int vertMap[8][3] =
    {
        {0, 0, 0},
        {0, 0, 1},
        {0, 1, 0},
        {0, 1, 1},
        {1, 0, 0},
        {1, 0, 1},
        {1, 1, 0},
        {1, 1, 1}};

const int faceMap[6][4] = {{4, 8, 5, 9}, {6, 10, 7, 11}, {0, 8, 1, 10}, {2, 9, 3, 11}, {0, 4, 2, 6}, {1, 5, 3, 7}};
const int cellProcFaceMask[12][3] = {{0, 4, 0}, {1, 5, 0}, {2, 6, 0}, {3, 7, 0}, {0, 2, 1}, {4, 6, 1}, {1, 3, 1}, {5, 7, 1}, {0, 1, 2}, {2, 3, 2}, {4, 5, 2}, {6, 7, 2}};
const int cellProcEdgeMask[6][5] = {{0, 1, 2, 3, 0}, {4, 5, 6, 7, 0}, {0, 4, 1, 5, 1}, {2, 6, 3, 7, 1}, {0, 2, 4, 6, 2}, {1, 3, 5, 7, 2}};

const int faceProcFaceMask[3][4][3] = {
    {{4, 0, 0}, {5, 1, 0}, {6, 2, 0}, {7, 3, 0}},
    {{2, 0, 1}, {6, 4, 1}, {3, 1, 1}, {7, 5, 1}},
    {{1, 0, 2}, {3, 2, 2}, {5, 4, 2}, {7, 6, 2}}};

const int faceProcEdgeMask[3][4][6] = {
    {{1, 4, 0, 5, 1, 1}, {1, 6, 2, 7, 3, 1}, {0, 4, 6, 0, 2, 2}, {0, 5, 7, 1, 3, 2}},
    {{0, 2, 3, 0, 1, 0}, {0, 6, 7, 4, 5, 0}, {1, 2, 0, 6, 4, 2}, {1, 3, 1, 7, 5, 2}},
    {{1, 1, 0, 3, 2, 0}, {1, 5, 4, 7, 6, 0}, {0, 1, 5, 0, 4, 1}, {0, 3, 7, 2, 6, 1}}};

const int edgeProcEdgeMask[3][2][5] = {
    {{3, 2, 1, 0, 0}, {7, 6, 5, 4, 0}},
    {{5, 1, 4, 0, 1}, {7, 3, 6, 2, 1}},
    {{6, 4, 2, 0, 2}, {7, 5, 3, 1, 2}},
};

const int processEdgeMask[3][4] = {{3, 2, 1, 0}, {7, 5, 6, 4}, {11, 10, 9, 8}};

const vm::ivec3 NEIGHBOUR_CHUNKS_OFFSETS[8] =
    {
        vm::ivec3(0, 0, 0),
        vm::ivec3(1, 0, 0),
        vm::ivec3(0, 0, 1),
        vm::ivec3(1, 0, 1),
        vm::ivec3(0, 1, 0),
        vm::ivec3(1, 1, 0),
        vm::ivec3(0, 1, 1),
        vm::ivec3(1, 1, 1),
};

uint64_t hashOctreeMin(const vm::ivec3 &min);

struct VertexData
{
    int index;
    vm::ivec4 biome;
    vm::vec4 biomeWeights;
    vm::vec3 position;
    vm::vec3 normal;
};

class OctreeNode
{
public:
    OctreeNode(const vm::ivec3 &min, const int &size) : min(min), size(size){};
    OctreeNode(const vm::ivec3 &min, const int &size, std::weak_ptr<OctreeNode> root) : min(min), size(size), root(root) {};
    std::vector<OctreeNode> children[8];    // only internal nodes have children
    std::unique_ptr<VertexData> vertexData; // only leaf nodes (our voxels) have vertex data
    std::weak_ptr<OctreeNode> root;         // reference to the root node
    vm::ivec3 min;
    int size;
};

void constructOctreeNodes(OctreeNode &node, std::weak_ptr<OctreeNode> rootNode , int &minVoxelSize)
{
    if(node.size == minVoxelSize){
        // constructVoxel();
    }
    const int childSize = node.size / 2;
    for (int i = 0; i < 8; i++)
    {
        const vm::ivec3 childMin = node.min + (CHILD_MIN_OFFSETS[i] * childSize);
        OctreeNode childNode(childMin, childSize, rootNode);
        node.children->emplace_back(childNode);
        constructOctreeNodes(node.children->at(i), rootNode, minVoxelSize);
    }
}

OctreeNode constructOctree(vm::ivec3 &min, int &size, int &minVoxelSize)
{
    OctreeNode octreeRootNode(min, size); 
    std::weak_ptr<OctreeNode> rootRef = std::make_shared<OctreeNode>(octreeRootNode);
    constructOctreeNodes(octreeRootNode, rootRef, minVoxelSize);
    return octreeRootNode;
}

class ChunkOctree
{
public:
    ChunkOctree(Chunk &chunk)
    {
        rootNode = std::make_unique<OctreeNode>(constructOctree(chunk.min, chunk.size, chunk.lod));
    }
    std::unique_ptr<OctreeNode> rootNode;
    vm::ivec3 min;
    int minVoxelSize; // determined by level of detail
    int size;         // influenced by level of detail
};

#endif // OCTREE_H