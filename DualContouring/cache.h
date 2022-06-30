#ifndef _CACHE_H_
#define _CACHE_H_

#include <array>
#include <unordered_map>
#include <mutex>
#include "sync.h"
#include "constants.h"
#include <emscripten.h>

//

class DCInstance;

//

template<typename T>
inline int modulo(T x, T N){
    return (x % N + N) % N;
}

//

class NoiseField {
public:
    float temperature;
    float humidity;
    float ocean;
    float river;
};

class Heightfield {
public:
    float heightField;
    std::array<unsigned char, 4> biomesVectorField;
    std::array<float, 4> biomesWeightsVectorField;
};

//

template <typename T>
class ChunkCacheValue {
public:
    T value;
    Mutex mutex;

    ChunkCacheValue() : mutex(true) {}
};

//

int getCacheIndexWorld(int x, int y);
int getCacheIndexWorld(int x, int y, int z);

//

uint32_t getCacheIndexLocal(int x, int y);
uint32_t getCacheIndexLocal(int x, int y, int z);

//

template <
    typename T,
    typename ChunkType,
    T(fn)(DCInstance *, int, int)
>
class ChunkCache2D {
public:
    DCInstance *inst;
    // std::unordered_map<uint64_t, ChunkCacheValue<T>> values;
    // std::vector<std::atomic<int>> valueSources;
    std::array<int, cacheWidth * cacheWidth> valueSources;
    std::array<T, cacheWidth * cacheWidth> values;
    Mutex mutex;

    ChunkCache2D(DCInstance *inst) : inst(inst), mutex(false) {
        // values.resize(cacheWidth * cacheWidth);
        // valueSources.resize(cacheWidth * cacheWidth);
        for (size_t i = 0; i < valueSources.size(); i++) {
            valueSources[i] = INT_MAX/2;
        }
    }
    ~ChunkCache2D() {}

    T get(int x, int y) {
        uint32_t localIndex = getCacheIndexLocal(x, y);
        int worldIndex = getCacheIndexWorld(x, y);

        // found in cache; fast case
        {
          // std::unique_lock<Mutex> lock(mutex);
          if (valueSources[localIndex] == worldIndex) {
            return values[localIndex];
          }
        }
        // not found in cache; recompute
        T value = fn(inst, x, y);
        {
          // std::unique_lock<Mutex> lock(mutex);
          values[localIndex] = value;
          valueSources[localIndex] = worldIndex;
        }
        return value;
    }
    void set(DCInstance *inst, ChunkType *chunk, int x, int z, const T &value) {
        uint32_t localIndex = getCacheIndexLocal(x, z);
        int worldIndex = getCacheIndexWorld(x, z);
        {
          // std::unique_lock<Mutex> lock(mutex);
          values[localIndex] = value;
          valueSources[localIndex] = worldIndex;
        }
    }
};
template <
    typename T,
    typename ChunkType,
    T(fn)(DCInstance *, int, int, int)
>
class ChunkCache3D {
public:
    DCInstance *inst;
    std::array<int, cacheWidth * cacheWidth * cacheWidth> valueSources;
    std::array<T, cacheWidth * cacheWidth * cacheWidth> values;
    Mutex mutex;

    ChunkCache3D(DCInstance *inst) : inst(inst), mutex(false) {
        /* if (!mutex.test()) {
          EM_ASM({
            console.log('mutex test failed');
          });
          abort();
        } */
        // values.reserve(RESERVE_SIZE);
        // values.resize(cacheWidth * cacheWidth * cacheWidth);
        // valueSources.resize(cacheWidth * cacheWidth * cacheWidth);
        for (size_t i = 0; i < valueSources.size(); i++) {
            valueSources[i] = INT_MAX/2;
        }
    }
    ~ChunkCache3D() {}

    T get(int x, int y, int z) {

        /* uint64_t getCacheIndex(int x, int y) {
            return x + y * cacheWidth; 
        }
        uint64_t getCacheIndex(int x, int y, int z) {
            return x + z * cacheWidth + y * cacheWidth * cacheWidth; 
        } */

        uint32_t localIndex = getCacheIndexLocal(x, y, z);
        int worldIndex = getCacheIndexWorld(x, y, z);

        // std::cout << "index " << x << " " << y << " " << z << " : " << localIndex << " " << worldIndex << std::endl;

        // found in cache; fast case
        {
          // std::unique_lock<Mutex> lock(mutex);
          if (valueSources[localIndex] == worldIndex) {
            return values[localIndex];
          }
        }
        // not found in cache; recompute
        T value = fn(inst, x, y, z);
        {
          // std::unique_lock<Mutex> lock(mutex);
          values[localIndex] = value;
          valueSources[localIndex] = worldIndex;
        }
        return value;
    }
    void set(int x, int y, int z, const T &value) {
        uint32_t localIndex = getCacheIndexLocal(x, y, z);
        int worldIndex = getCacheIndexWorld(x, y, z);
        {
          // std::unique_lock<Mutex> lock(mutex);
          values[localIndex] = value;
          valueSources[localIndex] = worldIndex;
        }
    }
};

#endif // _CACHE_H_