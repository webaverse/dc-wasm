#ifndef _CACHES_H_
#define _CACHES_H_

#include "cache.h"

//

class Caches {
public:
  Caches();
  ~Caches();

  // 2d caches

  static NoiseField initNoiseField(Caches *caches, int x, int y);
  static uint8_t initBiomesField(Caches *caches, int x, int y);
  static Heightfield initHeightField(Caches *caches, int x, int y);
  static float initWaterField(Caches *caches, int x, int y);

  ChunkCache2D<NoiseField, initNoiseField> *noiseField;
  ChunkCache2D<uint8_t, initBiomesField> *biomesField;
  ChunkCache2D<Heightfield, initHeightField> *heightField;
  ChunkCache2D<float, initWaterField> *waterField;

  // 3d caches

  static uint8_t initSkylightField(Caches *caches, int x, int y, int z);
  static uint8_t initAoField(Caches *caches, int x, int y, int z);
  static float initCaveField(Caches *caches, int x, int y, int z);
  static float initSdf(Caches *caches, int x, int y, int z);
  static float initWaterSdf(Caches *caches, int x, int y, int z);
  // static float initDamageSdf(Caches *caches, int x, int y, int z);

  ChunkCache3D<uint8_t, initSkylightField> *skylight;
  ChunkCache3D<uint8_t, initAoField> *ao;
  ChunkCache3D<float, initCaveField> *cave;
  ChunkCache3D<float, initSdf> *sdf;
  ChunkCache3D<float, initWaterSdf> *waterSdf;
  // ChunkCache3D<float, initDamageSdf> *cachedDamageSdf;

  // getters
  ChunkCache2D<NoiseField, initNoiseField> &getNoiseField();
  ChunkCache2D<uint8_t, initBiomesField> &getBiomesField();
  ChunkCache2D<Heightfield, initHeightField> &getHeightField();
  ChunkCache2D<float, initWaterField> &getWaterField();
  ChunkCache3D<uint8_t, initSkylightField> &getSkylight();
  ChunkCache3D<uint8_t, initAoField> &getAo();
  ChunkCache3D<float, initCaveField> &getCave();
  ChunkCache3D<float, initSdf> &getSdf();
  ChunkCache3D<float, initWaterSdf> &getWaterSdf();

  // 2d interpolation
  // unsigned char getCachedBiome(const int lx, const int lz) const;
  void getCachedInterpolatedBiome2D(const vm::vec2 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights);
  void getCachedInterpolatedBiome3D(const vm::vec3 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights);

  // 3d interpolation
  void getCachedHeightfield(float *heights);
  void getCachedSkylight(unsigned char *skylights);
  void getCachedAo(unsigned char *aos);
  float getCachedInterpolatedSdf(const float x, const float y, const float z);
  float getCachedWaterInterpolatedSdf(const float x, const float y, const float z);
  float getCachedDamageInterpolatedSdf(const float x, const float y, const float z);
};

#endif // _CACHES_H_