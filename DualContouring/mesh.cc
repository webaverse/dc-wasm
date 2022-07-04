#include "mesh.h"
#include "util.h"
#include <iostream>

uint8_t *TerrainVertexBuffer::getBuffer() const {
  // calculate size
  size_t neededSize = 
    // positions
    sizeof(uint32_t) +
    positions.size() * sizeof(positions[0]) +
    // normals
    sizeof(uint32_t) +
    normals.size() * sizeof(normals[0]) +
    // biomes
    sizeof(uint32_t) +
    biomes.size() * sizeof(biomes[0]) +
    // biomesWeights
    sizeof(uint32_t) +
    biomesWeights.size() * sizeof(biomesWeights[0]) +
    // biomesUvs1
    sizeof(uint32_t) +
    biomesUvs1.size() * sizeof(biomesUvs1[0]) +
    // biomesUvs2
    sizeof(uint32_t) +
    biomesUvs2.size() * sizeof(biomesUvs2[0]);

  neededSize +=
    // indices
    sizeof(uint32_t) +
    indices.size() * sizeof(indices[0]);

  neededSize +=
    // skylights
    sizeof(uint32_t) +
    skylights.size() * sizeof(skylights[0]);
  neededSize = align4(neededSize);
  
  neededSize +=
    // aos
    sizeof(uint32_t) +
    aos.size() * sizeof(aos[0]);
  neededSize = align4(neededSize);

  // allocate buffer
  uint8_t *buffer = (uint8_t *)malloc(neededSize);
  int index = 0;

  // positions
  *((uint32_t *)(buffer + index)) = positions.size();
  index += sizeof(uint32_t);
  std::memcpy(buffer + index, &positions[0], positions.size() * sizeof(positions[0]));
  index += positions.size() * sizeof(positions[0]);

  // normals
  *((uint32_t *)(buffer + index)) = normals.size();
  index += sizeof(uint32_t);
  std::memcpy(buffer + index, &normals[0], normals.size() * sizeof(normals[0]));
  index += normals.size() * sizeof(normals[0]);

  // biomes
  *((uint32_t *)(buffer + index)) = biomes.size();
  index += sizeof(uint32_t);
  std::memcpy(buffer + index, &biomes[0], biomes.size() * sizeof(biomes[0]));
  index += biomes.size() * sizeof(biomes[0]);

  // biomesWeights
  *((uint32_t *)(buffer + index)) = biomesWeights.size();
  index += sizeof(uint32_t);
  std::memcpy(buffer + index, &biomesWeights[0], biomesWeights.size() * sizeof(biomesWeights[0]));
  index += biomesWeights.size() * sizeof(biomesWeights[0]);

  // biomesUvs1
  *((uint32_t *)(buffer + index)) = biomesUvs1.size();
  index += sizeof(uint32_t);
  std::memcpy(buffer + index, &biomesUvs1[0], biomesUvs1.size() * sizeof(biomesUvs1[0]));
  index += biomesUvs1.size() * sizeof(biomesUvs1[0]);

  // biomesUvs2
  *((uint32_t *)(buffer + index)) = biomesUvs2.size();
  index += sizeof(uint32_t);
  std::memcpy(buffer + index, &biomesUvs2[0], biomesUvs2.size() * sizeof(biomesUvs2[0]));
  index += biomesUvs2.size() * sizeof(biomesUvs2[0]);

  // indices
  *((uint32_t *)(buffer + index)) = indices.size();
  index += sizeof(uint32_t);
  std::memcpy(buffer + index, &indices[0], indices.size() * sizeof(indices[0]));
  index += indices.size() * sizeof(indices[0]);

  // skylights
  *((uint32_t *)(buffer + index)) = skylights.size();
  index += sizeof(uint32_t);
  std::memcpy(buffer + index, &skylights[0], skylights.size() * sizeof(skylights[0]));
  index += skylights.size() * sizeof(skylights[0]);
  index = align4(index);

  // aos
  *((uint32_t *)(buffer + index)) = aos.size();
  index += sizeof(uint32_t);
  std::memcpy(buffer + index, &aos[0], aos.size() * sizeof(aos[0]));
  index += aos.size() * sizeof(aos[0]);
  index = align4(index);

  return buffer;
}
void TerrainVertexBuffer::pushVertexData(const VertexData &vertexData) {
  positions.push_back(vertexData.position);
  normals.push_back(vertexData.normal);
  biomes.push_back(vertexData.biomes);
  biomesWeights.push_back(vertexData.biomesWeights);
  
  biomesUvs1.push_back(vertexData.biomeUvs1);
  biomesUvs2.push_back(vertexData.biomeUvs2);
  
  skylights.push_back(vertexData.skylight);
  aos.push_back(vertexData.ao);
}

//

uint8_t *LiquidVertexBuffer::getBuffer() const {
  // calculate size
  size_t neededSize =
    // positions
    sizeof(uint32_t) +
    positions.size() * sizeof(positions[0]) +
    // normals
    sizeof(uint32_t) +
    normals.size() * sizeof(normals[0]) +
    // biomes
    sizeof(uint32_t) +
    biomes.size() * sizeof(biomes[0]) +
    // indices
    sizeof(uint32_t) +
    indices.size() * sizeof(indices[0]);

  // allocate buffer
  uint8_t *buffer = (uint8_t *)malloc(neededSize);
  int index = 0;

  // positions
  *((uint32_t *)(buffer + index)) = positions.size();
  index += sizeof(uint32_t);
  std::memcpy(buffer + index, &positions[0], positions.size() * sizeof(positions[0]));
  index += positions.size() * sizeof(positions[0]);

  // normals
  *((uint32_t *)(buffer + index)) = normals.size();
  index += sizeof(uint32_t);
  std::memcpy(buffer + index, &normals[0], normals.size() * sizeof(normals[0]));
  index += normals.size() * sizeof(normals[0]);

  // biomes
  *((uint32_t *)(buffer + index)) = biomes.size();
  index += sizeof(uint32_t);
  std::memcpy(buffer + index, &biomes[0], biomes.size() * sizeof(biomes[0]));
  index += biomes.size() * sizeof(biomes[0]);

  // indices
  *((uint32_t *)(buffer + index)) = indices.size();
  index += sizeof(uint32_t);
  std::memcpy(buffer + index, &indices[0], indices.size() * sizeof(indices[0]));
  index += indices.size() * sizeof(indices[0]);

  return buffer;
}
void LiquidVertexBuffer::pushVertexData(const VertexData &vertexData) {
  positions.push_back(vertexData.position);
  normals.push_back(vertexData.normal);
  biomes.push_back(vertexData.biomes.x);
}