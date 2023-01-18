#include "context.h"
#include "../util.h"
#include <iostream>

//

SdfAccessor::SdfAccessor(std::vector<float> &sdf, int chunkSize) :
  sdf(sdf),
  chunkSize(chunkSize)
{}
float SdfAccessor::get(float x, float y, float z) const {
  int ix = (int)x;
  int iy = (int)y;
  int iz = (int)z;

  // snap
  ix = std::min(std::max(ix, 0), chunkSize - 1);
  iy = std::min(std::max(iy, 0), chunkSize - 1);
  iz = std::min(std::max(iz, 0), chunkSize - 1);

  const int index = ix + iy * chunkSize + iz * chunkSize * chunkSize;
  
  // // XXX debug
  // if (ix >= 0 && ix < chunkSize && iy >= 0 && iy < chunkSize && iz >= 0 && iz < chunkSize) {
  //   // nothing
  // } else {
  //   std::cerr << "out of bounds 1: " << x << " " << y << " " << z << " " << index << " " << sdf.size() << " " << chunkSize << std::endl;
  //   abort();
  // }
  // if (index >= 0 && index < sdf.size()) {
  //   // nothing
  // } else {
  //   std::cerr << "out of bounds 2: " << x << " " << y << " " << z << " " << index << " " << sdf.size() << " " << chunkSize << std::endl;
  //   abort();
  // }
    
  return sdf[index];
}

//

TerrainDCContext::TerrainDCContext(const std::vector<vm::vec3> &pointcloud, int chunkSize, int range, float fatness) :
  sdf(chunkSize * chunkSize * chunkSize),
  sdfAccessor(sdf, chunkSize)
{
  const float maxValue = -fatness + (float)range;

  // std::cout << "fill 1" << std::endl;
  std::fill(sdf.begin(), sdf.end(), maxValue);
  // std::cout << "fill 2 " << pointcloud.size() << std::endl;

  // int numFills = 0;
  for (int i = 0; i < pointcloud.size(); i++) {
    const vm::vec3 &p = pointcloud[i];
    const vm::ivec3 ip{
      (int)p.x,
      (int)p.y,
      (int)p.z
    };

    for (int dx = -range; dx <= range; dx++) {
      for (int dy = -range; dy <= range; dy++) {
        for (int dz = -range; dz <= range; dz++) {
          const vm::ivec3 ip2{
            ip.x + dx,
            ip.y + dy,
            ip.z + dz
          };
          if (
            ip2.x >= 0 && ip2.x < chunkSize &&
            ip2.y >= 0 && ip2.y < chunkSize &&
            ip2.z >= 0 && ip2.z < chunkSize
          ) {
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

            // numFills++;
          }
        }
      }
    }
  }
  // std::cout << "fill 3 num fills: " << numFills << std::endl;
}

float TerrainDCContext::densityFn(const vm::vec3 &position) {
  return trilinear<SdfAccessor, float>(position, sdfAccessor);
}