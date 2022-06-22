#include "octree.h"
#include "main.h"

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

typedef std::function<bool(const vm::ivec3 &, const vm::ivec3 &)> FilterNodesFunc;

const vm::ivec3 chunkMinForPosition(const vm::ivec3 &p)
{
    const unsigned int mask = ~(DualContouring::chunkSize - 1);
    return vm::ivec3(p.x & mask, p.y & mask, p.z & mask);
}

uint64_t hashOctreeMin(const vm::ivec3 &min)
{
    uint64_t result = uint16_t(min.x);
    result = (result << 16) + uint16_t(min.y);
    result = (result << 16) + uint16_t(min.z);
    return result;
}

//

VertexData::VertexData() : index(-1), corners(0) {}

//

OctreeNode::OctreeNode(const vm::ivec3 &min, const int &size, const OctreeNodeType &type) : min(min), size(size), vertexData(nullptr), type(type)
{
    children.resize(8);
}

void clampPositionToMassPoint(std::shared_ptr<OctreeNode> &voxelNode, svd::QefSolver &qef, vm::vec3 &vertexPosition)
{

    const vm::vec3 min = vm::vec3(voxelNode->min.x, voxelNode->min.y, voxelNode->min.z);
    const vm::vec3 max = vm::vec3(voxelNode->min.x + vm::ivec3(voxelNode->size).x,
                                  voxelNode->min.y + vm::ivec3(voxelNode->size).y,
                                  voxelNode->min.z + vm::ivec3(voxelNode->size).z);
    if (vertexPosition.x < min.x || vertexPosition.x > max.x ||
        vertexPosition.y < min.y || vertexPosition.y > max.y ||
        vertexPosition.z < min.z || vertexPosition.z > max.z)
    {
        const auto &mp = qef.getMassPoint();
        vertexPosition = vm::vec3(mp.x, mp.y, mp.z);
    }
}