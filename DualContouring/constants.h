#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

constexpr int numThreads = NUM_THREADS;
constexpr int chunkSize = 16;
constexpr int CHUNK_CACHE_RANGE = 8;
constexpr int cacheWidth = chunkSize * CHUNK_CACHE_RANGE;

constexpr float frustumCullDistancePenalty = 1000.f;
constexpr float priorityDistancePenalty = 10000.f;

enum class PEEK_FACES : int {
  FRONT = 0,
  BACK,
  LEFT,
  RIGHT,
  TOP,
  BOTTOM,
  NONE
};

#endif // _CONSTANTS_H_