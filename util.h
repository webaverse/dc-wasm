#ifndef _UTIL_H_
#define _UTIL_H_

#include "./DualContouring/cache.h"
#include "./DualContouring/instance.h"
#include <functional>

float lerp(const float &a, const float &b, const float &f);
int align(int x, int N);
int align4(int x);

template<typename T>
T bilinear( 
    const float &tx, 
    const float &ty, 
    const T &c00, 
    const T &c10, 
    const T &c01, 
    const T &c11) 
{
  T a = c00 * (1.f - tx) + c10 * tx; 
  T b = c01 * (1.f - tx) + c11 * tx; 
  return a * (1.f - ty) + b * ty; 
}
template<typename T, typename R>
R trilinear(
  const vm::vec3 &location,
  const int lod,
  T &data
)
{
  float rx = std::round(location.x);
  float ry = std::round(location.y);
  float rz = std::round(location.z);

  int ix = int(rx);
  int iy = int(ry);
  int iz = int(rz);
  
  vm::ivec3 p000{ix, iy, iz};
  vm::ivec3 p100{ix + lod, iy, iz};
  vm::ivec3 p010{ix, iy + lod, iz};
  vm::ivec3 p110{ix + lod, iy + lod, iz};
  vm::ivec3 p001{ix, iy, iz + lod};
  vm::ivec3 p101{ix + lod, iy, iz + lod};
  vm::ivec3 p011{ix, iy + lod, iz + lod};
  vm::ivec3 p111{ix + lod, iy + lod, iz + lod};

  const R &v000 = data.get(p000.x, p000.y, p000.z);
  const R &v100 = data.get(p100.x, p100.y, p100.z);
  const R &v010 = data.get(p010.x, p010.y, p010.z);
  const R &v110 = data.get(p110.x, p110.y, p110.z);
  const R &v001 = data.get(p001.x, p001.y, p001.z);
  const R &v101 = data.get(p101.x, p101.y, p101.z);
  const R &v011 = data.get(p011.x, p011.y, p011.z);
  const R &v111 = data.get(p111.x, p111.y, p111.z);

  float tx = location.x - p000.x;
  float ty = location.y - p000.y;
  float tz = location.z - p000.z;

  const R &e = bilinear<R>(tx, ty, v000, v100, v010, v110); 
  const R &f = bilinear<R>(tx, ty, v001, v101, v011, v111); 
  return e * (1 - tz) + f * tz; 
}

#endif // _UTIL_H_