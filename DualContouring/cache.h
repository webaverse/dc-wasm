#ifndef _CACHE_H_
#define _CACHE_H_

#include <array>
#include <unordered_map>
#include <mutex>
#include "sync.h"
#include <emscripten.h>

//

class DCInstance;

//

constexpr size_t RESERVE_SIZE = 1024 * 1024;

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

uint64_t getIndex(int x, int y);
uint64_t getIndex(int x, int y, int z);

//

template <
    typename T,
    typename ChunkType,
    T(fn)(DCInstance *, int, int)
>
class ChunkCache2D {
public:
    DCInstance *inst;
    std::unordered_map<uint64_t, ChunkCacheValue<T>> values;
    Mutex mutex;

    ChunkCache2D(DCInstance *inst) : inst(inst), mutex(false) {
        values.reserve(RESERVE_SIZE);
    }
    ~ChunkCache2D() {}

    T get(int x, int y) {
        ChunkCacheValue<T> *cacheValue;
        uint64_t index = getIndex(x, y);
        {
          std::unique_lock<Mutex> lock(mutex);
          cacheValue = &values[index];
        }
        if (cacheValue->mutex.test()) {
            cacheValue->value = fn(inst, x, y);
            cacheValue->mutex.unlock();
        }
        return cacheValue->value;
    }
    void set(DCInstance *inst, ChunkType *chunk, int x, int z, const T &value) {
        ChunkCacheValue<T> *cacheValue;
        uint64_t index = getIndex(x, z);
        {
          std::unique_lock<Mutex> lock(mutex);
          cacheValue = &values[index];
        }
        cacheValue->value = value;
        cacheValue->mutex.unlock();
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
    std::unordered_map<uint64_t, ChunkCacheValue<T>> values;
    Mutex mutex;

    ChunkCache3D(DCInstance *inst) : inst(inst), mutex(false) {
        /* if (!mutex.test()) {
          EM_ASM({
            console.log('mutex test failed');
          });
          abort();
        } */
        values.reserve(RESERVE_SIZE);
    }
    ~ChunkCache3D() {}

    T get(int x, int y, int z) {
        ChunkCacheValue<T> *cacheValue;
        uint64_t index = getIndex(x, y, z);
        {
          std::unique_lock<Mutex> lock(mutex);
          cacheValue = &values[index];
        }
        if (cacheValue->mutex.test()) {
            cacheValue->value = fn(inst, x, y, z);
            cacheValue->mutex.unlock();
        }
        return cacheValue->value;
    }
    void set(int x, int y, int z, const T &value) {
        ChunkCacheValue<T> *cacheValue;
        uint64_t index = getIndex(x, y, z);
        {
          std::unique_lock<Mutex> lock(mutex);
          cacheValue = &values[index];
        }
        cacheValue->value = value;
        cacheValue->mutex.unlock();
    }
};

#endif // _CACHE_H_