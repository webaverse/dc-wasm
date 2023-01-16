#include "context.h"
#include "../util.h"
#include <iostream>

//

SdfAccessor::SdfAccessor(std::vector<float> &sdf) : sdf(sdf) {}
float SdfAccessor::get(float x, float y, float z) const {
  const int ix = (int)x;
  const int iy = (int)y;
  const int iz = (int)z;
  const int index = ix + iy * chunkSize + iz * chunkSize * chunkSize;
  return sdf[index];
}

//

TerrainDCContext::TerrainDCContext(const std::vector<vm::vec3> &pointcloud) :
  sdf(chunkSize * chunkSize * chunkSize),
  sdfAccessor(sdf)
{
  constexpr int rangeRadius = 5;
  constexpr float fatness = 1.f;
  constexpr float maxValue = -fatness + (float)rangeRadius;

  std::fill(sdf.begin(), sdf.end(), maxValue);

  for (int i = 0; i < pointcloud.size(); i++) {
    const vm::vec3 &p = pointcloud[i];
    const vm::ivec3 ip{
      (int)p.x,
      (int)p.y,
      (int)p.z
    };

    for (int dx = -rangeRadius; dx <= rangeRadius; dx++) {
      for (int dy = -rangeRadius; dy <= rangeRadius; dy++) {
        for (int dz = -rangeRadius; dz <= rangeRadius; dz++) {
          const vm::ivec3 ip2{
            ip.x + dx,
            ip.y + dy,
            ip.z + dz
          };
          const vm::vec3 p2{
            (float)ip2.x + 0.5f,
            (float)ip2.y + 0.5f,
            (float)ip2.z + 0.5f
          };
          float distance = vm::length(p2 - p);

          const int index = ip2.x + ip2.y * chunkSize + ip2.z * chunkSize * chunkSize;
          sdf[index] = std::min(
            sdf[index],
            -fatness + distance
          );
        }
      }
    }
  }
}

float TerrainDCContext::densityFn(const vm::vec3 &position) {
  return trilinear<SdfAccessor, float>(position, 1, sdfAccessor);
}