#ifndef MESH_H
#define MESH_H

#include "vectorMath.h"
#include "biomes.h"

#include <vector>
#include <array>
#include <stdint.h>
#include <cstring>

//

typedef std::vector<vm::vec3> PositionBuffer;
typedef std::vector<vm::vec3> NormalBuffer;
typedef std::vector<vm::ivec4> BiomesBuffer;
typedef std::vector<vm::vec4> BiomesWeightBuffer;
typedef std::vector<std::array<UV, 4>> BiomesUvsBuffer;
typedef std::vector<int> IndexBuffer;
typedef std::vector<int> BiomeBuffer;
typedef std::vector<uint8_t> LightBuffer;

//

struct VertexData
{
    int index;
    int corners;

    vm::vec3 position;
    vm::vec3 normal;

    vm::ivec4 biomes;
    vm::vec4 biomesWeights;
    std::array<UV, 4> biomeUvs;

    uint8_t skylight;
    uint8_t ao;
};

//

class TerrainVertexBuffer {
public:
    PositionBuffer positions;
    NormalBuffer normals;
    IndexBuffer indices;

    BiomesBuffer biomes;
    BiomesWeightBuffer biomesWeights;
    BiomesUvsBuffer biomesUvs;

    LightBuffer skylights;
    LightBuffer aos;

    uint8_t *getBuffer() const;
    
    void pushVertexData(const VertexData &vertexData);
};

class LiquidVertexBuffer {
public:
    PositionBuffer positions;
    NormalBuffer normals;
    IndexBuffer indices;

    BiomeBuffer biomes;

    uint8_t *getBuffer() const;

    void pushVertexData(const VertexData &vertexData);
};

#endif // MESH_H
