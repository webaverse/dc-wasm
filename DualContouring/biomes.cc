#include "biomes.h"

bool isWaterBiome(unsigned char b) {
  return b == (unsigned char)BIOME::biOcean ||
    b == (unsigned char)BIOME::biRiver ||
    b == (unsigned char)BIOME::biSwampland ||
    b == (unsigned char)BIOME::biFrozenRiver ||
    b == (unsigned char)BIOME::biFrozenOcean;
}