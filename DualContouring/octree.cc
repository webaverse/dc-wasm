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

//

void contourProcessEdge(std::shared_ptr<OctreeNode> (&node)[4], int dir, IndexBuffer &indexBuffer, bool isSeam)
{
    int minSize = 2147483647; // arbitrary big number
    int minIndex = 0;
    int indices[4] = {-1, -1, -1, -1};
    bool flip = false;
    bool signChange[4] = {false, false, false, false};

    for (int i = 0; i < 4; i++)
    {
        const int edge = processEdgeMask[dir][i];
        const int c1 = edgevmap[edge][0];
        const int c2 = edgevmap[edge][1];

        const int m1 = (node[i]->vertexData->corners >> c1) & 1;
        const int m2 = (node[i]->vertexData->corners >> c2) & 1;

        if (node[i]->size < minSize)
        {
            minSize = node[i]->size;
            minIndex = i;
            flip = m1 != MATERIAL_AIR;
        }

        indices[i] = node[i]->vertexData->index;

        signChange[i] =
            (m1 == MATERIAL_AIR && m2 != MATERIAL_AIR) ||
            (m1 != MATERIAL_AIR && m2 == MATERIAL_AIR);
    }

    if (signChange[minIndex])
    {
        if (!flip)
        {
            indexBuffer.push_back(indices[0]);
            indexBuffer.push_back(indices[1]);
            indexBuffer.push_back(indices[3]);

            indexBuffer.push_back(indices[0]);
            indexBuffer.push_back(indices[3]);
            indexBuffer.push_back(indices[2]);
        }
        else
        {
            indexBuffer.push_back(indices[0]);
            indexBuffer.push_back(indices[3]);
            indexBuffer.push_back(indices[1]);

            indexBuffer.push_back(indices[0]);
            indexBuffer.push_back(indices[2]);
            indexBuffer.push_back(indices[3]);
        }
    }
}

void contourEdgeProc(std::shared_ptr<OctreeNode> (&node)[4], int dir, IndexBuffer &indexBuffer, bool isSeam)
{
    if (!node[0] || !node[1] || !node[2] || !node[3])
    {
        return;
    }

    const bool isBranch[4] =
        {
            node[0]->type == Node_Internal,
            node[1]->type == Node_Internal,
            node[2]->type == Node_Internal,
            node[3]->type == Node_Internal,
        };

    if (!isBranch[0] && !isBranch[1] && !isBranch[2] && !isBranch[3])
    {
        // prevents seams geometry from overlapping with the chunk geometry
        if (isSeam &&
            chunkMinForPosition(node[0]->min) == chunkMinForPosition(node[1]->min) &&
            chunkMinForPosition(node[1]->min) == chunkMinForPosition(node[2]->min) &&
            chunkMinForPosition(node[2]->min) == chunkMinForPosition(node[3]->min))
        {
            return;
        }
        contourProcessEdge(node, dir, indexBuffer, isSeam);
    }
    else
    {
        for (int i = 0; i < 2; i++)
        {
            std::shared_ptr<OctreeNode> edgeNodes[4];
            const int c[4] =
                {
                    edgeProcEdgeMask[dir][i][0],
                    edgeProcEdgeMask[dir][i][1],
                    edgeProcEdgeMask[dir][i][2],
                    edgeProcEdgeMask[dir][i][3],
                };

            for (int j = 0; j < 4; j++)
            {
                if (!isBranch[j])
                {
                    edgeNodes[j] = node[j];
                }
                else
                {
                    edgeNodes[j] = node[j]->children[c[j]];
                }
            }

            contourEdgeProc(edgeNodes, edgeProcEdgeMask[dir][i][4], indexBuffer, isSeam);
        }
    }
}

void contourFaceProc(std::shared_ptr<OctreeNode> (&node)[2], int dir, IndexBuffer &indexBuffer, bool isSeam)
{
    if (!node[0] || !node[1])
    {
        return;
    }

    const bool isBranch[2] =
        {
            node[0]->type == Node_Internal,
            node[1]->type == Node_Internal,
        };

    if (isBranch[0] || isBranch[1])
    {
        // prevents seams geometry from overlapping with the chunk geometry
        if (isSeam && chunkMinForPosition(node[0]->min) == chunkMinForPosition(node[1]->min))
        {
            return;
        }
        for (int i = 0; i < 4; i++)
        {
            std::shared_ptr<OctreeNode> faceNodes[2];
            const int c[2] =
                {
                    faceProcFaceMask[dir][i][0],
                    faceProcFaceMask[dir][i][1],
                };

            for (int j = 0; j < 2; j++)
            {
                if (!isBranch[j])
                {
                    faceNodes[j] = node[j];
                }
                else
                {
                    faceNodes[j] = node[j]->children[c[j]];
                }
            }

            contourFaceProc(faceNodes, faceProcFaceMask[dir][i][2], indexBuffer, isSeam);
        }

        const int orders[2][4] =
            {
                {0, 0, 1, 1},
                {0, 1, 0, 1},
            };
        for (int i = 0; i < 4; i++)
        {
            std::shared_ptr<OctreeNode> edgeNodes[4];
            const int c[4] =
                {
                    faceProcEdgeMask[dir][i][1],
                    faceProcEdgeMask[dir][i][2],
                    faceProcEdgeMask[dir][i][3],
                    faceProcEdgeMask[dir][i][4],
                };

            const int *order = orders[faceProcEdgeMask[dir][i][0]];
            for (int j = 0; j < 4; j++)
            {
                if (!isBranch[order[j]])
                {
                    edgeNodes[j] = node[order[j]];
                }
                else
                {
                    edgeNodes[j] = node[order[j]]->children[c[j]];
                }
            }

            contourEdgeProc(edgeNodes, faceProcEdgeMask[dir][i][5], indexBuffer, isSeam);
        }
    }
}

void contourCellProc(std::shared_ptr<OctreeNode> &node, IndexBuffer &indexBuffer, bool isSeam)
{
    if (!node || node->type == Node_Leaf)
    {
        return;
    }

    for (int i = 0; i < 8; i++)
    {
        contourCellProc(node->children[i], indexBuffer, isSeam);
    }

    for (int i = 0; i < 12; i++)
    {
        std::shared_ptr<OctreeNode> faceNodes[2];
        const int c[2] = {cellProcFaceMask[i][0], cellProcFaceMask[i][1]};

        faceNodes[0] = node->children[c[0]];
        faceNodes[1] = node->children[c[1]];

        contourFaceProc(faceNodes, cellProcFaceMask[i][2], indexBuffer, isSeam);
    }

    for (int i = 0; i < 6; i++)
    {
        std::shared_ptr<OctreeNode> edgeNodes[4];
        const int c[4] =
            {
                cellProcEdgeMask[i][0],
                cellProcEdgeMask[i][1],
                cellProcEdgeMask[i][2],
                cellProcEdgeMask[i][3],
            };

        for (int j = 0; j < 4; j++)
        {
            edgeNodes[j] = node->children[c[j]];
        }

        contourEdgeProc(edgeNodes, cellProcEdgeMask[i][4], indexBuffer, isSeam);
    }
}