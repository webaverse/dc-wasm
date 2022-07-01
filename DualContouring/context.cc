#include "context.h"
#include <iostream>

//

float TerrainDCContext::densityFn(const vm::vec3 &position, DCInstance *inst, Caches *caches) {
  return terrainDensityFn(position, inst, caches);
}

//

float LiquidDCContext::densityFn(const vm::vec3 &position, DCInstance *inst, Caches *caches) {
  return liquidDensityFn(position, inst, caches);
}