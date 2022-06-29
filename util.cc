#include "./DualContouring/vectorMath.h"
#include "util.h"
#include <vector>

float lerp(const float &a, const float &b, const float &f) {
    return a + f * (b - a);
}

template<>
float trilinear<ChunkCache3D<float, Chunk3D, DCInstance::initWaterSdf>, float>(
  const vm::vec3 &location,
  ChunkCache3D<float, Chunk3D, DCInstance::initWaterSdf> &data
)
{
  if (isnan(location.x) || isnan(location.y) || isnan(location.z)) {
    EM_ASM({
      console.log("NaN detected in location");
    });
    abort();
  }
  float rx = std::round(location.x);
  float ry = std::round(location.y);
  float rz = std::round(location.z);

  int ix = int(rx);
  int iy = int(ry);
  int iz = int(rz);
  
  vm::ivec3 p000 = vm::ivec3(ix, iy, iz);
  vm::ivec3 p100 = vm::ivec3(ix + 1, iy, iz);
  vm::ivec3 p010 = vm::ivec3(ix, iy + 1, iz);
  vm::ivec3 p110 = vm::ivec3(ix + 1, iy + 1, iz);
  vm::ivec3 p001 = vm::ivec3(ix, iy, iz + 1);
  vm::ivec3 p101 = vm::ivec3(ix + 1, iy, iz + 1);
  vm::ivec3 p011 = vm::ivec3(ix, iy + 1, iz + 1);
  vm::ivec3 p111 = vm::ivec3(ix + 1, iy + 1, iz + 1);

  /* int i000 = p000.x + p000.z * gridPoints + p000.y * gridPoints * gridPoints;
  int i100 = p100.x + p100.z * gridPoints + p100.y * gridPoints * gridPoints;
  int i010 = p010.x + p010.z * gridPoints + p010.y * gridPoints * gridPoints;
  int i110 = p110.x + p110.z * gridPoints + p110.y * gridPoints * gridPoints;
  int i001 = p001.x + p001.z * gridPoints + p001.y * gridPoints * gridPoints;
  int i101 = p101.x + p101.z * gridPoints + p101.y * gridPoints * gridPoints;
  int i011 = p011.x + p011.z * gridPoints + p011.y * gridPoints * gridPoints;
  int i111 = p111.x + p111.z * gridPoints + p111.y * gridPoints * gridPoints; */

  const float &v000 = data.get(p000.x, p000.y, p000.z);
  const float &v100 = data.get(p100.x, p100.y, p100.z);
  const float &v010 = data.get(p010.x, p010.y, p010.z);
  const float &v110 = data.get(p110.x, p110.y, p110.z);
  const float &v001 = data.get(p001.x, p001.y, p001.z);
  const float &v101 = data.get(p101.x, p101.y, p101.z);
  const float &v011 = data.get(p011.x, p011.y, p011.z);
  const float &v111 = data.get(p111.x, p111.y, p111.z);

  if (
    isnan(v000) || isnan(v100) || isnan(v010) || isnan(v110) ||
    isnan(v001) || isnan(v101) || isnan(v011) || isnan(v111)
  ) {
    EM_ASM({
      console.log("NaN detected in data");
    });
    abort();
  }

  float tx = location.x - p000.x;
  float ty = location.y - p000.y;
  float tz = location.z - p000.z;

  const float &e = bilinear<float>(tx, ty, v000, v100, v010, v110); 
  const float &f = bilinear<float>(tx, ty, v001, v101, v011, v111); 
  return e * (1 - tz) + f * tz; 
}