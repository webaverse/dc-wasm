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
typedef std::vector<std::array<UV, 2>> BiomesUvsBuffer;
typedef std::vector<int> IndexBuffer;
typedef std::vector<int> BiomeBuffer;
typedef std::vector<uint8_t> LightBuffer;
typedef unsigned char PeekBuffer[15];

//

struct VertexData
{
    int index;
    int corners;

    vm::vec3 position;
    vm::vec3 normal;

    vm::ivec4 biomes;
    vm::vec4 biomesWeights;

    std::array<UV, 2> biomeUvs1; // needs to be broken up to fit into vec4
    std::array<UV, 2> biomeUvs2;

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
    BiomesUvsBuffer biomesUvs1;
    BiomesUvsBuffer biomesUvs2;

    LightBuffer skylights;
    LightBuffer aos;

    PeekBuffer peeks;

    uint8_t *getBuffer() const;
    
    void pushVertexData(const VertexData &vertexData);
};

class LiquidVertexBuffer {
public:
    PositionBuffer positions;
    NormalBuffer normals;
    IndexBuffer indices;

    BiomeBuffer biomes;

    PeekBuffer peeks;

    uint8_t *getBuffer() const;

    void pushVertexData(const VertexData &vertexData);
};

#endif // MESH_H
