#ifndef _CONSTANTS_H_
#define _CONSTANTS_H_

constexpr int numThreads = NUM_THREADS;
// constexpr uint32_t stackSize = 1024 * 1024;
constexpr int chunkSize = 16;
constexpr int CHUNK_CACHE_RANGE = 16;
constexpr int cacheWidth = chunkSize * CHUNK_CACHE_RANGE;

#endif // _CONSTANTS_H_