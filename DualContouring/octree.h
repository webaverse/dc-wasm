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

template<typename DCContextType>
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
        const float density = abs(DCContextType::densityFn(p, inst, chunk));
        if (density < minValue)
        {
            minValue = density;
            t = percentage;
        }
    }

    return p0 + ((p1 - p0) * t);
}
template<typename DCContextType>
vm::vec3 calculateSurfaceNormal(const vm::vec3 &p, DCInstance *inst, Chunk &chunkNoise)
{
    // finding the surface normal with the derivative
    const float H = 0.001f;
    const float dx = DCContextType::densityFn(p + vm::vec3(H, 0.f, 0.f), inst, chunkNoise) -
                     DCContextType::densityFn(p - vm::vec3(H, 0.f, 0.f), inst, chunkNoise);
    const float dy = DCContextType::densityFn(p + vm::vec3(0.f, H, 0.f), inst, chunkNoise) -
                     DCContextType::densityFn(p - vm::vec3(0.f, H, 0.f), inst, chunkNoise);
    const float dz = DCContextType::densityFn(p + vm::vec3(0.f, 0.f, H), inst, chunkNoise) -
                     DCContextType::densityFn(p - vm::vec3(0.f, 0.f, H), inst, chunkNoise);
    return vm::normalize(vm::vec3(dx, dy, dz));
}

void clampPositionToMassPoint(std::shared_ptr<OctreeNode> &voxelNode, svd::QefSolver &qef, vm::vec3 &vertexPosition);

template<typename DCContextType>
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
        const vm::vec3 p = approximateZeroCrossingPosition<DCContextType>(p1, p2, inst, chunk);
        const vm::vec3 n = calculateSurfaceNormal<DCContextType>(p, inst, chunk);
        qef.add(p.x, p.y, p.z, n.x, n.y, n.z);
        averageNormal += n;
        edgeCount++;
    }
    return edgeCount;
}

//

template <typename DCContextType>
class ChunkOctree
{
public:
    // members
    std::shared_ptr<OctreeNode> root;
    std::shared_ptr<OctreeNode> seamRoot;
    vm::ivec3 min;
    int minVoxelSize; // determined by level of detail
    int size;

    // constructors
    ChunkOctree(DCInstance *inst, Chunk &chunk, int lodArray[8]) : min(chunk.min), size(chunk.size), minVoxelSize(chunk.lod)
    {
        std::shared_ptr<OctreeNode> rootNode = std::make_shared<OctreeNode>(OctreeNode(min, size, Node_Internal));
        std::vector<std::shared_ptr<OctreeNode>> voxelNodes = generateVoxelNodes(inst, chunk);
        root = constructOctreeUpwards(rootNode, voxelNodes, chunk.min, chunk.size);
        std::vector<std::shared_ptr<OctreeNode>> seamNodes = generateSeamNodes(inst, chunk, lodArray);
        seamRoot = constructOctreeUpwards(seamRoot, seamNodes, chunk.min, chunk.size * 2);
    }

    // methods
    std::vector<std::shared_ptr<OctreeNode>> generateVoxelNodes(DCInstance *inst, Chunk &chunk)
    {
        std::vector<std::shared_ptr<OctreeNode>> nodes;
        const vm::ivec3 chunkMax = chunk.min + chunk.size;

        for (int x = chunk.min.x; x < chunkMax.x; x += chunk.lod)
            for (int y = chunk.min.y; y < chunkMax.y; y += chunk.lod)
                for (int z = chunk.min.z; z < chunkMax.z; z += chunk.lod)
                {
                    const vm::ivec3 min = vm::ivec3(x, y, z);
                    std::shared_ptr<OctreeNode> node = std::make_shared<OctreeNode>(OctreeNode(min, chunk.lod, Node_Leaf));
                    std::shared_ptr<OctreeNode> seamNode = constructLeaf(node, inst, chunk);
                    if (seamNode)
                        nodes.push_back(seamNode);
                }
        return nodes;
    }
    VertexData generateVoxelData(std::shared_ptr<OctreeNode> &voxelNode, int &corners, DCInstance *inst, Chunk &chunk)
    {
        svd::QefSolver qef;
        vm::vec3 averageNormal(0.f);
        int edgeCount = findEdgeIntersection<DCContextType>(voxelNode, qef, averageNormal, corners, minVoxelSize, inst, chunk);
        svd::Vec3 qefPosition;
        qef.solve(qefPosition, QEF_ERROR, QEF_SWEEPS, QEF_ERROR);
        vm::vec3 vertexPosition = vm::vec3(qefPosition.x, qefPosition.y, qefPosition.z);
        // if the generated position is outside of the voxel bounding box :
        clampPositionToMassPoint(voxelNode, qef, vertexPosition);
        VertexData vertexData;
        vertexData.position = vertexPosition;
        vertexData.normal = vm::normalize(averageNormal / (float)edgeCount);
        vertexData.corners = corners;
        chunk.getCachedInterpolatedBiome3D(vertexData.position, vertexData.biomes, vertexData.biomesWeights);
        return vertexData;
    }

    std::shared_ptr<OctreeNode> constructLeaf(std::shared_ptr<OctreeNode> &voxelNode, DCInstance *inst, Chunk &chunk)
    {
        int corners = 0;
        for (int i = 0; i < 8; i++)
        {
            const vm::ivec3 cornerPos = voxelNode->min + CHILD_MIN_OFFSETS[i] * minVoxelSize;
            const float density = DCContextType::densityFn(vm::vec3(cornerPos.x, cornerPos.y, cornerPos.z), inst, chunk);
            const int material = density < 0.f ? MATERIAL_SOLID : MATERIAL_AIR;
            corners |= (material << i);
        }
        if (corners == 0 || corners == 255)
        {
            voxelNode = 0;
        }
        else
        {
            // voxel is touching the surface
            voxelNode->vertexData = std::make_shared<VertexData>(generateVoxelData(voxelNode, corners, inst, chunk));
        }
        return voxelNode;
    }

    void findOctreeNodesRecursively(std::shared_ptr<OctreeNode> &node, FilterNodesFunc &func, std::vector<std::shared_ptr<OctreeNode>> &nodes)
    {
        if (!node)
        {
            return;
        }

        const vm::ivec3 max = node->min + vm::ivec3(node->size);
        if (!func(node->min, max))
        {
            return;
        }

        if (node->type == Node_Leaf)
        {
            nodes.push_back(node);
        }
        else
        {
            for (int i = 0; i < 8; i++)
                findOctreeNodesRecursively(node->children[i], func, nodes);
        }
    }

    std::vector<std::shared_ptr<OctreeNode>> findOctreeNodes(std::shared_ptr<OctreeNode> root, FilterNodesFunc filterFunc)
    {
        std::vector<std::shared_ptr<OctreeNode>> nodes;
        findOctreeNodesRecursively(root, filterFunc, nodes);
        return nodes;
    }

    std::vector<std::shared_ptr<OctreeNode>> constructChunkSeamNodes(DCInstance *inst, Chunk &chunk, const int &lod, const vm::ivec3 &chunkMin, FilterNodesFunc filterFunc, const int &chunkSize)
    {
        std::vector<std::shared_ptr<OctreeNode>> nodes;
        const vm::ivec3 chunkMax = chunkMin + chunkSize;

        for (int x = chunkMin.x; x < chunkMax.x; x += lod)
            for (int y = chunkMin.y; y < chunkMax.y; y += lod)
                for (int z = chunkMin.z; z < chunkMax.z; z += lod)
                {
                    const vm::ivec3 min = vm::ivec3(x, y, z);
                    const vm::ivec3 max = min + vm::ivec3(lod);
                    if (filterFunc(min, max))
                    {
                        std::shared_ptr<OctreeNode> node = std::make_shared<OctreeNode>(OctreeNode(min, lod, Node_Leaf));
                        std::shared_ptr<OctreeNode> seamNode = constructLeaf(node, inst, chunk);
                        if (seamNode)
                            nodes.push_back(seamNode);
                    }
                }
        return nodes;
    }

    std::vector<std::shared_ptr<OctreeNode>> generateSeamNodes(DCInstance *inst, Chunk &chunk, const int lodArray[])
    {
        const vm::ivec3 baseChunkMin = vm::ivec3(chunk.min);
        const vm::ivec3 seamValues = baseChunkMin + vm::ivec3(chunk.size);

        std::vector<std::shared_ptr<OctreeNode>> seamNodes;

        FilterNodesFunc selectionFuncs[8] =
            {[&](const vm::ivec3 &min, const vm::ivec3 &max)
                {
                    return max.x == seamValues.x || max.y == seamValues.y || max.z == seamValues.z;
                },
                [&](const vm::ivec3 &min, const vm::ivec3 &max)
                {
                    return min.x == seamValues.x;
                },
                [&](const vm::ivec3 &min, const vm::ivec3 &max)
                {
                    return min.z == seamValues.z;
                },
                [&](const vm::ivec3 &min, const vm::ivec3 &max)
                {
                    return min.x == seamValues.x && min.z == seamValues.z;
                },
                [&](const vm::ivec3 &min, const vm::ivec3 &max)
                {
                    return min.y == seamValues.y;
                },
                [&](const vm::ivec3 &min, const vm::ivec3 &max)
                {
                    return min.x == seamValues.x && min.y == seamValues.y;
                },
                [&](const vm::ivec3 &min, const vm::ivec3 &max)
                {
                    return min.y == seamValues.y && min.z == seamValues.z;
                },
                [&](const vm::ivec3 &min, const vm::ivec3 &max)
                {
                    return min.x == seamValues.x && min.y == seamValues.y && min.z == seamValues.z;
                }};

        std::vector<std::shared_ptr<OctreeNode>> rootChunkSeamNodes = findOctreeNodes(root, selectionFuncs[0]);

        // adding root chunk seam nodes
        seamNodes.insert(std::end(seamNodes), std::begin(rootChunkSeamNodes), std::end(rootChunkSeamNodes));

        // creating the seam nodes of the neighbouring chunks
        // NEIGHBOUR_CHUNKS_OFFSETS[i]
        std::vector<std::shared_ptr<OctreeNode>> neighbourNodes;
        for (int i = 1; i < 8; i++)
        {
            const vm::ivec3 offsetMin = NEIGHBOUR_CHUNKS_OFFSETS[i] * chunk.size;
            const vm::ivec3 chunkMin = baseChunkMin + offsetMin;
            std::vector<std::shared_ptr<OctreeNode>> chunkSeamNodes = constructChunkSeamNodes(inst, chunk, lodArray[i], chunkMin, selectionFuncs[i], chunk.size);
            neighbourNodes.insert(std::end(neighbourNodes), std::begin(chunkSeamNodes), std::end(chunkSeamNodes));
        }

        seamNodes.insert(std::end(seamNodes), std::begin(neighbourNodes), std::end(neighbourNodes));

        return seamNodes;
    }

    std::vector<std::shared_ptr<OctreeNode>> constructParents(
        std::shared_ptr<OctreeNode> &octree,
        const std::vector<std::shared_ptr<OctreeNode>> &nodes,
        const int &parentSize,
        const vm::ivec3 &rootMin)
    {
        std::unordered_map<uint64_t, std::shared_ptr<OctreeNode>> parentsHashmap;

        for_each(begin(nodes), end(nodes), [&](std::shared_ptr<OctreeNode> node)
                    {
                    // because the octree is regular we can calculate the parent min
                    const vm::ivec3 localPos = (node->min - rootMin);
                    const vm::ivec3 parentPos = node->min - (localPos % parentSize);

                    const uint64_t parentIndex = hashOctreeMin(parentPos - rootMin);
                    std::shared_ptr<OctreeNode> parentNode;

                    auto iter = parentsHashmap.find(parentIndex);
                    if (iter == end(parentsHashmap))
                    {
                        parentNode = std::make_shared<OctreeNode>(OctreeNode(parentPos,parentSize, Node_Internal));
                        parentsHashmap.insert(std::pair<uint64_t, std::shared_ptr<OctreeNode>>(parentIndex, parentNode));
                    //  std::cout << parentNode->size << std::endl;
                    }
                    else
                    {
                        parentNode = iter->second;
                    }

                    bool foundParentNode = false;
                    for (int i = 0; i < 8; i++)
                    {
                        const vm::ivec3 childPos = parentPos + ((parentSize / 2) * CHILD_MIN_OFFSETS[i]);
                        if (childPos == node->min)
                        {
                            parentNode->children[i] = node;
                            foundParentNode = true;
                            break;
                        }
                    } });

        std::vector<std::shared_ptr<OctreeNode>> parents;
        for_each(begin(parentsHashmap), end(parentsHashmap), [&](std::pair<uint64_t, std::shared_ptr<OctreeNode>> pair)
                    { parents.push_back(pair.second); });

        return parents;
    }

    std::shared_ptr<OctreeNode> constructOctreeUpwards(
        std::shared_ptr<OctreeNode> &octree,
        const std::vector<std::shared_ptr<OctreeNode>> &inputNodes,
        const vm::ivec3 &rootMin,
        const int rootNodeSize)
    {
        if (inputNodes.empty())
        {
            return nullptr;
        }

        std::vector<std::shared_ptr<OctreeNode>> nodes(begin(inputNodes), end(inputNodes));
        std::sort(std::begin(nodes), std::end(nodes),
                    [](std::shared_ptr<OctreeNode> &lhs, std::shared_ptr<OctreeNode> &rhs)
                    {
                        return lhs->size < rhs->size;
                    });

        // the input nodes may be different sizes if a seam octree is being constructed
        // in that case we need to process the input nodes in stages along with the newly
        // constructed parent nodes until all the nodes have the same size
        while (nodes.front()->size != nodes.back()->size)
        {
            // find the end of this run
            auto iter = std::begin(nodes);
            int size = (*iter)->size;
            do
            {
                ++iter;
            } while ((*iter)->size == size);

            // construct the new parent nodes for this run
            std::vector<std::shared_ptr<OctreeNode>> newNodes(std::begin(nodes), iter);
            newNodes = constructParents(octree, newNodes, size * 2, rootMin);

            // set up for the next iteration: the parents produced plus any remaining input nodes
            newNodes.insert(std::end(newNodes), iter, std::end(nodes));
            std::swap(nodes, newNodes);
        }

        int parentSize = nodes.front()->size * 2;
        while (parentSize <= rootNodeSize)
        {
            nodes = constructParents(octree, nodes, parentSize, rootMin);
            parentSize *= 2;
        }
        std::shared_ptr<OctreeNode> root = nodes.front();

        return root;
    }
};

//

void contourProcessEdge(std::shared_ptr<OctreeNode> (&node)[4], int dir, IndexBuffer &indexBuffer, bool isSeam);
void contourEdgeProc(std::shared_ptr<OctreeNode> (&node)[4], int dir, IndexBuffer &indexBuffer, bool isSeam);
void contourFaceProc(std::shared_ptr<OctreeNode> (&node)[2], int dir, IndexBuffer &indexBuffer, bool isSeam);
void contourCellProc(std::shared_ptr<OctreeNode> &node, IndexBuffer &indexBuffer, bool isSeam);

template <typename DCContextType>
void generateMeshFromOctree(std::shared_ptr<OctreeNode> &node, DCContextType &dcContext, bool isSeam)
{
    generateVertexIndices(node, dcContext);
    contourCellProc(node, dcContext.vertexBuffer.indices, isSeam);
}
template <typename DCContextType>
void generateVertexIndices(std::shared_ptr<OctreeNode> &node, DCContextType &dcContext)
{
    auto &vertexBuffer = dcContext.vertexBuffer;
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
            generateVertexIndices(node->children[i], dcContext);
        }
    }
}

#endif // OCTREE_H