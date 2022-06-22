#ifndef MESH_H
#define MESH_H

#include "vectorMath.h"

#include <vector>
#include <stdint.h>
#include <cstring>

//

typedef std::vector<vm::vec3> PositionBuffer;
typedef std::vector<vm::vec3> NormalBuffer;
typedef std::vector<vm::ivec4> BiomesBuffer;
typedef std::vector<vm::vec4> BiomesWeightBuffer;
typedef std::vector<int> IndexBuffer;
typedef std::vector<int> BiomeBuffer;

//

struct VertexData
{
    VertexData();
    
    int index;
    int corners;
    vm::ivec4 biomes;
    vm::vec4 biomesWeights;
    vm::vec3 position;
    vm::vec3 normal;
};

//

class VertexBuffer {
public:
    PositionBuffer positions;
    NormalBuffer normals;
    BiomesBuffer biomes;
    BiomesWeightBuffer biomesWeights;
    IndexBuffer indices;

    uint8_t *getBuffer() const;
    
    void pushVertexData(const VertexData &vertexData);
};

class VertexWaterBuffer {
public:
    PositionBuffer positions;
    NormalBuffer normals;
    BiomeBuffer biomes;
    IndexBuffer indices;

    uint8_t *getBuffer() const;

    void pushVertexData(const VertexData &vertexData);
};

#endif // MESH_H
