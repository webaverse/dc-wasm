#ifndef MESH_H
#define MESH_H

#include "vectorMath.h"

#include <vector>
#include <stdint.h>
#include <cstring>

typedef std::vector<vm::vec3> PositionBuffer;
typedef std::vector<vm::vec3> NormalBuffer;
typedef std::vector<vm::ivec4> BiomesBuffer;
typedef std::vector<vm::vec4> BiomesWeightBuffer;
typedef std::vector<int> IndexBuffer;
typedef std::vector<int> BiomeBuffer;

class VertexBuffer {
public:
    PositionBuffer positions;
    NormalBuffer normals;
    BiomesBuffer biomes;
    BiomesWeightBuffer biomesWeights;
    IndexBuffer indices;

    uint8_t *getBuffer() const;
};

class VertexWaterBuffer {
public:
    PositionBuffer positions;
    NormalBuffer normals;
    BiomeBuffer biomes;
    IndexBuffer indices;

    uint8_t *getBuffer() const;
};

#endif // MESH_H
