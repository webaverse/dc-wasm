#include "context.h"
#include <iostream>

//

float TerrainDCContext::densityFn(const vm::vec3 &position, DCInstance *inst) {
  return terrainDensityFn(position, inst);
}

//

float LiquidDCContext::densityFn(const vm::vec3 &position, DCInstance *inst) {
  return liquidDensityFn(position, inst);
}