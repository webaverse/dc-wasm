#include "context.h"
#include <iostream>

//

float TerrainVertexContext::densityFn(const vm::vec3 &position, DCInstance *inst, Chunk &chunk) {
  return terrainDensityFn(position, inst, chunk);
}

//

float LiquidVertexContext::densityFn(const vm::vec3 &position, DCInstance *inst, Chunk &chunk) {
  return liquidDensityFn(position, inst, chunk);
}