#ifndef _UTIL_H_
#define _UTIL_H_

float lerp(const float &a, const float &b, const float &f);

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
template<typename T>
T trilinear(
  const vm::vec3 &location,
  const std::vector<T> &data,
  int gridPoints
)
{
  vm::ivec3 p000 = vm::ivec3(int(location.x), int(location.y), int(location.z));
  vm::ivec3 p100 = vm::ivec3(int(location.x) + 1, int(location.y), int(location.z));
  vm::ivec3 p010 = vm::ivec3(int(location.x), int(location.y) + 1, int(location.z));
  vm::ivec3 p110 = vm::ivec3(int(location.x) + 1, int(location.y) + 1, int(location.z));
  vm::ivec3 p001 = vm::ivec3(int(location.x), int(location.y), int(location.z) + 1);
  vm::ivec3 p101 = vm::ivec3(int(location.x) + 1, int(location.y), int(location.z) + 1);
  vm::ivec3 p011 = vm::ivec3(int(location.x), int(location.y) + 1, int(location.z) + 1);
  vm::ivec3 p111 = vm::ivec3(int(location.x) + 1, int(location.y) + 1, int(location.z) + 1);

  int i000 = p000.x + p000.z * gridPoints + p000.y * gridPoints * gridPoints;
  int i100 = p100.x + p100.z * gridPoints + p100.y * gridPoints * gridPoints;
  int i010 = p010.x + p010.z * gridPoints + p010.y * gridPoints * gridPoints;
  int i110 = p110.x + p110.z * gridPoints + p110.y * gridPoints * gridPoints;
  int i001 = p001.x + p001.z * gridPoints + p001.y * gridPoints * gridPoints;
  int i101 = p101.x + p101.z * gridPoints + p101.y * gridPoints * gridPoints;
  int i011 = p011.x + p011.z * gridPoints + p011.y * gridPoints * gridPoints;
  int i111 = p111.x + p111.z * gridPoints + p111.y * gridPoints * gridPoints;

  const T &v000 = data.at(i000);
  const T &v100 = data.at(i100);
  const T &v010 = data.at(i010);
  const T &v110 = data.at(i110);
  const T &v001 = data.at(i001);
  const T &v101 = data.at(i101);
  const T &v011 = data.at(i011);
  const T &v111 = data.at(i111);

  float tx = location.x - p000.x;
  float ty = location.y - p000.y;
  float tz = location.z - p000.z;

  const T &e = bilinear<T>(tx, ty, v000, v100, v010, v110); 
  const T &f = bilinear<T>(tx, ty, v001, v101, v011, v111); 
  return e * (1 - tz) + f * tz; 
}

#endif // _UTIL_H_