#ifndef _CACHE_H_
#define _CACHE_H_

#include <array>
#include <unordered_map>
#include <mutex>
#include <bitset>
#include "sync.h"
#include "constants.h"
#include <emscripten.h>

//

class Caches;

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

int16_t getCacheIndexWorld(int x, int y);
int getCacheIndexWorld(int x, int y, int z);

//

uint32_t getCacheIndexLocal(int x, int y);
uint32_t getCacheIndexLocal(int x, int y, int z);

//

template <
    typename T,
    T(fn)(Caches *, int, int)
>
class ChunkCache2D {
public:
    Caches *caches;
    // std::unordered_map<uint64_t, ChunkCacheValue<T>> values;
    // std::vector<std::atomic<int>> valueSources;
    // std::array<int16_t, cacheWidth * cacheWidth> valueSources;
    std::bitset<cacheWidth * cacheWidth> bitset;
    std::array<T, cacheWidth * cacheWidth> values;
    // Mutex mutex;

    ChunkCache2D(Caches *caches) : caches(caches) {
        // values.resize(cacheWidth * cacheWidth);
        // valueSources.resize(cacheWidth * cacheWidth);
        /* for (size_t i = 0; i < valueSources.size(); i++) {
            valueSources[i] = 128;
        } */
    }
    ~ChunkCache2D() {}

    T get(int x, int y) {
        uint32_t localIndex = getCacheIndexLocal(x, y);

        // found in cache; fast case
        if (bitset[localIndex]) {
          return values[localIndex];
        }

        // not found in cache; recompute
        T value = fn(caches, x, y);
        values[localIndex] = value;
        bitset.set(localIndex);
        
        return value;
    }
    void set(Caches *caches, int x, int z, const T &value) {
        uint32_t localIndex = getCacheIndexLocal(x, z);
        // int16_t worldIndex = getCacheIndexWorld(x, z);
        
        values[localIndex] = value;
        bitset.set(localIndex);
    }
};
template <
    typename T,
    T(fn)(Caches *, int, int, int)
>
class ChunkCache3D {
public:
    Caches *caches;
    // std::array<int, cacheWidth * cacheWidth * cacheWidth> valueSources;
    std::bitset<cacheWidth * cacheWidth * cacheWidth> bitset;
    std::array<T, cacheWidth * cacheWidth * cacheWidth> values;
    // Mutex mutex;

    ChunkCache3D(Caches *caches) : caches(caches) {
        /* if (!mutex.test()) {
          EM_ASM({
            console.log('mutex test failed');
          });
          abort();
        } */
        // values.reserve(RESERVE_SIZE);
        // values.resize(cacheWidth * cacheWidth * cacheWidth);
        // valueSources.resize(cacheWidth * cacheWidth * cacheWidth);
        /* for (size_t i = 0; i < valueSources.size(); i++) {
            valueSources[i] = 128;
        } */
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
        // int worldIndex = getCacheIndexWorld(x, y, z);

        // std::cout << "index " << x << " " << y << " " << z << " : " << localIndex << " " << worldIndex << std::endl;

        // found in cache; fast case
        if (bitset[localIndex]) {
          return values[localIndex];
        }

        // not found in cache; recompute
        T value = fn(caches, x, y, z);
        values[localIndex] = value;
        bitset.set(localIndex);

        return value;
    }
    void set(int x, int y, int z, const T &value) {
        uint32_t localIndex = getCacheIndexLocal(x, y, z);
        // int worldIndex = getCacheIndexWorld(x, y, z);

        values[localIndex] = value;
        bitset.set(localIndex);
    }
};

#endif // _CACHE_H_