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

//

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
        const float density = abs(Density_Func(p, inst, chunk));
        if (density < minValue)
        {
            minValue = density;
            t = percentage;
        }
    }

    return p0 + ((p1 - p0) * t);
}

vm::vec3 calculateSurfaceNormal(const vm::vec3 &p, DCInstance *inst, Chunk &chunkNoise)
{
    // finding the surface normal with the derivative
    const float H = 0.001f;
    const float dx = Density_Func(p + vm::vec3(H, 0.f, 0.f), inst, chunkNoise) -
                     Density_Func(p - vm::vec3(H, 0.f, 0.f), inst, chunkNoise);
    const float dy = Density_Func(p + vm::vec3(0.f, H, 0.f), inst, chunkNoise) -
                     Density_Func(p - vm::vec3(0.f, H, 0.f), inst, chunkNoise);
    const float dz = Density_Func(p + vm::vec3(0.f, 0.f, H), inst, chunkNoise) -
                     Density_Func(p - vm::vec3(0.f, 0.f, H), inst, chunkNoise);
    return vm::normalize(vm::vec3(dx, dy, dz));
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
        const vm::vec3 p = approximateZeroCrossingPosition(p1, p2, inst, chunk);
        const vm::vec3 n = calculateSurfaceNormal(p, inst, chunk);
        qef.add(p.x, p.y, p.z, n.x, n.y, n.z);
        averageNormal += n;
        edgeCount++;
    }
    return edgeCount;
}

//

ChunkOctree::ChunkOctree(DCInstance *inst, Chunk &chunk, int lodArray[8]) : min(chunk.min), size(chunk.size), minVoxelSize(chunk.lod)
{
    std::shared_ptr<OctreeNode> rootNode = std::make_shared<OctreeNode>(OctreeNode(min, size, Node_Internal));
    std::vector<std::shared_ptr<OctreeNode>> voxelNodes = generateVoxelNodes(inst, chunk);
    root = constructOctreeUpwards(rootNode, voxelNodes, chunk.min, chunk.size);
    std::vector<std::shared_ptr<OctreeNode>> seamNodes = generateSeamNodes(inst, chunk, lodArray);
    seamRoot = constructOctreeUpwards(seamRoot, seamNodes, chunk.min, chunk.size * 2);
}
std::vector<std::shared_ptr<OctreeNode>> ChunkOctree::generateVoxelNodes(DCInstance *inst, Chunk &chunk)
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
VertexData ChunkOctree::generateVoxelData(std::shared_ptr<OctreeNode> &voxelNode, int &corners, DCInstance *inst, Chunk &chunk)
{
    svd::QefSolver qef;
    vm::vec3 averageNormal(0.f);
    int edgeCount = findEdgeIntersection(voxelNode, qef, averageNormal, corners, minVoxelSize, inst, chunk);
    svd::Vec3 qefPosition;
    qef.solve(qefPosition, QEF_ERROR, QEF_SWEEPS, QEF_ERROR);
    vm::vec3 vertexPosition = vm::vec3(qefPosition.x, qefPosition.y, qefPosition.z);
    // if the generated position is outside of the voxel bounding box :
    clampPositionToMassPoint(voxelNode, qef, vertexPosition);
    VertexData vertexData;
    vertexData.position = vertexPosition;
    vertexData.normal = vm::normalize(averageNormal / (float)edgeCount);
    vertexData.corners = corners;
    chunk.getCachedInterpolatedBiome3D(vertexData.position, vertexData.biome, vertexData.biomeWeights);
    return vertexData;
}

std::shared_ptr<OctreeNode> ChunkOctree::constructLeaf(std::shared_ptr<OctreeNode> &voxelNode, DCInstance *inst, Chunk &chunk)
{
    int corners = 0;
    for (int i = 0; i < 8; i++)
    {
        const vm::ivec3 cornerPos = voxelNode->min + CHILD_MIN_OFFSETS[i] * minVoxelSize;
        const float density = Density_Func(vm::vec3(cornerPos.x, cornerPos.y, cornerPos.z), inst, chunk);
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

void ChunkOctree::findOctreeNodesRecursively(std::shared_ptr<OctreeNode> &node, FilterNodesFunc &func, std::vector<std::shared_ptr<OctreeNode>> &nodes)
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

std::vector<std::shared_ptr<OctreeNode>> ChunkOctree::findOctreeNodes(std::shared_ptr<OctreeNode> root, FilterNodesFunc filterFunc)
{
    std::vector<std::shared_ptr<OctreeNode>> nodes;
    findOctreeNodesRecursively(root, filterFunc, nodes);
    return nodes;
}

std::vector<std::shared_ptr<OctreeNode>> ChunkOctree::constructChunkSeamNodes(DCInstance *inst, Chunk &chunk, const int &lod, const vm::ivec3 &chunkMin, FilterNodesFunc filterFunc, const int &chunkSize)
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

std::vector<std::shared_ptr<OctreeNode>> ChunkOctree::generateSeamNodes(DCInstance *inst, Chunk &chunk, const int lodArray[])
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

std::vector<std::shared_ptr<OctreeNode>> ChunkOctree::constructParents(
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

std::shared_ptr<OctreeNode> ChunkOctree::constructOctreeUpwards(
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

//

void generateVertexIndices(std::shared_ptr<OctreeNode> &node, VertexBuffer &vertexBuffer)
{
    if (!node)
    {
        return;
    }
    if (node->type == Node_Leaf)
    {
        if (!node->vertexData)
        {
            printf("Error! The provided voxel has no vertex data!\n");
            return;
        }
        node->vertexData->index = vertexBuffer.positions.size();
        vertexBuffer.positions.push_back(node->vertexData->position);
        vertexBuffer.normals.push_back(node->vertexData->normal);
        vertexBuffer.biomes.push_back(node->vertexData->biome);
        vertexBuffer.biomesWeights.push_back(node->vertexData->biomeWeights);
    }
    else
    {
        for (int i = 0; i < 8; i++)
        {
            generateVertexIndices(node->children[i], vertexBuffer);
        }
    }
}

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

void generateMeshFromOctree(std::shared_ptr<OctreeNode> &node, VertexBuffer &vertexBuffer, bool isSeam)
{
    generateVertexIndices(node, vertexBuffer);
    contourCellProc(node, vertexBuffer.indices, isSeam);
}