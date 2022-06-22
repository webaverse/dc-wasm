#ifndef OCTREE_H
#define OCTREE_H

#include "vectorMath.h"
#define _USE_MATH_DEFINES

#include "qef.h"
#include "mesh.h"
#include <iostream>
#include <vector>
#include <functional>
#include <algorithm>
#include <stdint.h>
#include <cstdlib>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include "density.h"

#define M_PI 3.14159265358979323846

constexpr int MATERIAL_AIR = 0;
constexpr int MATERIAL_SOLID = 1;

constexpr float QEF_ERROR = 1e-6f;
constexpr int QEF_SWEEPS = 4;

//

extern const vm::ivec3 CHILD_MIN_OFFSETS[];
extern const int edgevmap[12][2];
extern const int edgemask[3];
extern const int vertMap[8][3];

extern const int faceMap[6][4];
extern const int cellProcFaceMask[12][3];
extern const int cellProcEdgeMask[6][5];

extern const int faceProcFaceMask[3][4][3];

extern const int faceProcEdgeMask[3][4][6];

extern const int edgeProcEdgeMask[3][2][5];

extern const int processEdgeMask[3][4];

extern const vm::ivec3 NEIGHBOUR_CHUNKS_OFFSETS[8];

//

typedef std::function<bool(const vm::ivec3 &, const vm::ivec3 &)> FilterNodesFunc;

//

const vm::ivec3 chunkMinForPosition(const vm::ivec3 &p);

uint64_t hashOctreeMin(const vm::ivec3 &min);

//

enum OctreeNodeType
{
    Node_Internal,
    Node_Leaf
};

//

class OctreeNode
{
public:
    OctreeNode() = delete;
    OctreeNode(const vm::ivec3 &min, const int &size, const OctreeNodeType &type);
    
    std::vector<std::shared_ptr<OctreeNode>> children; // only internal nodes have children
    std::shared_ptr<VertexData> vertexData;            // only leaf nodes (our voxels) have vertex data
    vm::ivec3 min;
    int size;
    OctreeNodeType type;
};

//

template<typename VertexContextType>
vm::vec3 approximateZeroCrossingPosition(const vm::vec3 &p0, const vm::vec3 &p1, DCInstance *inst, Chunk &chunk)
{
    // approximate the zero crossing by finding the min value along the edge
    float minValue = 100000.f;
    float t = 0.0;
    const int steps = 8; // sample 8 times
    for (int i = 0; i < steps; i++)
    {
        const float percentage = i / steps;
        const vm::vec3 p = p0 + ((p1 - p0) * percentage);
        const float density = abs(VertexContextType::densityFn(p, inst, chunk));
        if (density < minValue)
        {
            minValue = density;
            t = percentage;
        }
    }

    return p0 + ((p1 - p0) * t);
}
template<typename VertexContextType>
vm::vec3 calculateSurfaceNormal(const vm::vec3 &p, DCInstance *inst, Chunk &chunkNoise)
{
    // finding the surface normal with the derivative
    const float H = 0.001f;
    const float dx = VertexContextType::densityFn(p + vm::vec3(H, 0.f, 0.f), inst, chunkNoise) -
                     VertexContextType::densityFn(p - vm::vec3(H, 0.f, 0.f), inst, chunkNoise);
    const float dy = VertexContextType::densityFn(p + vm::vec3(0.f, H, 0.f), inst, chunkNoise) -
                     VertexContextType::densityFn(p - vm::vec3(0.f, H, 0.f), inst, chunkNoise);
    const float dz = VertexContextType::densityFn(p + vm::vec3(0.f, 0.f, H), inst, chunkNoise) -
                     VertexContextType::densityFn(p - vm::vec3(0.f, 0.f, H), inst, chunkNoise);
    return vm::normalize(vm::vec3(dx, dy, dz));
}

void clampPositionToMassPoint(std::shared_ptr<OctreeNode> &voxelNode, svd::QefSolver &qef, vm::vec3 &vertexPosition);

template<typename VertexContextType>
int findEdgeIntersection(std::shared_ptr<OctreeNode> &voxelNode, svd::QefSolver &qef, vm::vec3 &averageNormal, int &corners, const int &minVoxelSize, DCInstance *inst, Chunk &chunk)
{
    const int MAX_CROSSINGS = 6;
    int edgeCount = 0;
    for (int i = 0; i < 12 && edgeCount < MAX_CROSSINGS; i++)
    {
        const int c1 = edgevmap[i][0];
        const int c2 = edgevmap[i][1];
        const int m1 = (corners >> c1) & 1;
        const int m2 = (corners >> c2) & 1;
        if ((m1 == MATERIAL_AIR && m2 == MATERIAL_AIR) ||
            (m1 == MATERIAL_SOLID && m2 == MATERIAL_SOLID))
        {
            continue;
        }
        const vm::ivec3 ip1 = voxelNode->min + CHILD_MIN_OFFSETS[c1] * minVoxelSize;
        const vm::ivec3 ip2 = voxelNode->min + CHILD_MIN_OFFSETS[c2] * minVoxelSize;
        const vm::vec3 p1 = vm::vec3(ip1.x, ip1.y, ip1.z);
        const vm::vec3 p2 = vm::vec3(ip2.x, ip2.y, ip2.z);
        const vm::vec3 p = approximateZeroCrossingPosition<VertexContextType>(p1, p2, inst, chunk);
        const vm::vec3 n = calculateSurfaceNormal<VertexContextType>(p, inst, chunk);
        qef.add(p.x, p.y, p.z, n.x, n.y, n.z);
        averageNormal += n;
        edgeCount++;
    }
    return edgeCount;
}

//

class ChunkOctree
{
public:
    ChunkOctree(DCInstance *inst, Chunk &chunk, int lodArray[8]);

    std::shared_ptr<OctreeNode> root;
    std::shared_ptr<OctreeNode> seamRoot;

    vm::ivec3 min;
    int minVoxelSize; // determined by level of detail
    int size;

    std::vector<std::shared_ptr<OctreeNode>> generateVoxelNodes(DCInstance *inst, Chunk &chunk);
    VertexData generateVoxelData(std::shared_ptr<OctreeNode> &voxelNode, int &corners, DCInstance *inst, Chunk &chunk);
    std::shared_ptr<OctreeNode> constructLeaf(std::shared_ptr<OctreeNode> &voxelNode, DCInstance *inst, Chunk &chunk);
    void findOctreeNodesRecursively(std::shared_ptr<OctreeNode> &node, FilterNodesFunc &func, std::vector<std::shared_ptr<OctreeNode>> &nodes);

    std::vector<std::shared_ptr<OctreeNode>> findOctreeNodes(std::shared_ptr<OctreeNode> root, FilterNodesFunc filterFunc);
    std::vector<std::shared_ptr<OctreeNode>> constructChunkSeamNodes(DCInstance *inst, Chunk &chunk, const int &lod, const vm::ivec3 &chunkMin, FilterNodesFunc filterFunc, const int &chunkSize);
    std::vector<std::shared_ptr<OctreeNode>> generateSeamNodes(DCInstance *inst, Chunk &chunk, const int lodArray[]);

    std::vector<std::shared_ptr<OctreeNode>> constructParents(
        std::shared_ptr<OctreeNode> &octree,
        const std::vector<std::shared_ptr<OctreeNode>> &nodes,
        const int &parentSize,
        const vm::ivec3 &rootMin);

    std::shared_ptr<OctreeNode> constructOctreeUpwards(
        std::shared_ptr<OctreeNode> &octree,
        const std::vector<std::shared_ptr<OctreeNode>> &inputNodes,
        const vm::ivec3 &rootMin,
        const int rootNodeSize);
};

//

void contourProcessEdge(std::shared_ptr<OctreeNode> (&node)[4], int dir, IndexBuffer &indexBuffer, bool isSeam);
void contourEdgeProc(std::shared_ptr<OctreeNode> (&node)[4], int dir, IndexBuffer &indexBuffer, bool isSeam);
void contourFaceProc(std::shared_ptr<OctreeNode> (&node)[2], int dir, IndexBuffer &indexBuffer, bool isSeam);
void contourCellProc(std::shared_ptr<OctreeNode> &node, IndexBuffer &indexBuffer, bool isSeam);

template <typename VertexContextType>
void generateMeshFromOctree(std::shared_ptr<OctreeNode> &node, VertexContextType &vertexContext, bool isSeam)
{
    auto &vertexBuffer = vertexContext.vertexBuffer;
    generateVertexIndices(node, vertexBuffer);
    contourCellProc(node, vertexBuffer.indices, isSeam);
}
template <typename VertexContextType>
void generateVertexIndices(std::shared_ptr<OctreeNode> &node, VertexContextType &vertexContext)
{
    auto &vertexBuffer = vertexContext.vertexBuffer;
    if (!node)
    {
        return;
    }
    if (node->type == Node_Leaf)
    {
        if (!node->vertexData)
        {
            printf("Error! The provided voxel has no vertex data!\n");
            abort();
        }
        node->vertexData->index = vertexBuffer.positions.size();
        vertexBuffer.pushVertexData(*(node->vertexData));
    }
    else
    {
        for (int i = 0; i < 8; i++)
        {
            generateVertexIndices(node->children[i], vertexBuffer);
        }
    }
}

#endif // OCTREE_H