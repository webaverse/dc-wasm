#include "context.h"
#include <iostream>

//

float TerrainDCContext::densityFn(const vm::vec3 &position, const int lod, DCInstance *inst) {
  return terrainDensityFn(position, lod, inst);
}

//

float LiquidDCContext::densityFn(const vm::vec3 &position, const int lod, DCInstance *inst) {
  return liquidDensityFn(position, lod, inst);
}