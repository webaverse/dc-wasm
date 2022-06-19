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

struct VertexData
{
    VertexData();
    
    int index;
    int corners;
    vm::ivec4 biome;
    vm::vec4 biomeWeights;
    vm::vec3 position;
    vm::vec3 normal;
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

vm::vec3 approximateZeroCrossingPosition(const vm::vec3 &p0, const vm::vec3 &p1, DCInstance *inst, Chunk &chunk);

vm::vec3 calculateSurfaceNormal(const vm::vec3 &p, DCInstance *inst, Chunk &chunkNoise);

void clampPositionToMassPoint(std::shared_ptr<OctreeNode> &voxelNode, svd::QefSolver &qef, vm::vec3 &vertexPosition);

int findEdgeIntersection(std::shared_ptr<OctreeNode> &voxelNode, svd::QefSolver &qef, vm::vec3 &averageNormal, int &corners, const int &minVoxelSize, DCInstance *inst, Chunk &chunk);

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

void generateVertexIndices(std::shared_ptr<OctreeNode> &node, VertexBuffer &vertexBuffer);
void contourProcessEdge(std::shared_ptr<OctreeNode> (&node)[4], int dir, IndexBuffer &indexBuffer, bool &isSeam);
void contourEdgeProc(std::shared_ptr<OctreeNode> (&node)[4], int dir, IndexBuffer &indexBuffer, bool &isSeam);
void contourFaceProc(std::shared_ptr<OctreeNode> (&node)[2], int dir, IndexBuffer &indexBuffer, bool &isSeam);
void contourCellProc(std::shared_ptr<OctreeNode> &node, IndexBuffer &indexBuffer, bool &isSeam);
void generateMeshFromOctree(std::shared_ptr<OctreeNode> &node, VertexBuffer &vertexBuffer, bool isSeam);

#endif // OCTREE_H