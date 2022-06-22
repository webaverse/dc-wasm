#include "context.h"
#include <iostream>

//

float TerrainDCContext::densityFn(const vm::vec3 &position, DCInstance *inst, Chunk &chunk) {
  return terrainDensityFn(position, inst, chunk);
}

//

float LiquidDCContext::densityFn(const vm::vec3 &position, DCInstance *inst, Chunk &chunk) {
  return liquidDensityFn(position, inst, chunk);
}