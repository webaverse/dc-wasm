#ifndef _CHUNK_H_
#define _CHUNK_H_

#include "vectorMath.h"
#include "biomes.h"
#include "sync.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>
#include <vector>
#include <deque>
#include <emscripten.h>

class DCInstance;

//

const float MAX_HEIGHT = 20.f;
const int numBiomes = (int)BIOME::NUM_BIOMES;

enum GenerateFlags : int {
    GF_NONE = 0,
    GF_NOISE = 1 << 0,
    GF_BIOMES = 1 << 1,
    GF_HEIGHTFIELD = 1 << 2,
    GF_WATERFIELD = 1 << 3,
    GF_AOFIELD = 1 << 4,
    GF_SDF = 1 << 5,
    GF_LIQUIDS = 1 << 6,
};

//

int resolveGenerateFlags(int flags);

//

class NoiseField {
public:
    // int size;
    std::vector<float> temperature;
    std::vector<float> humidity;
    std::vector<float> ocean;
    std::vector<float> river;

    // NoiseField &operator=(const NoiseField &&other);

    // NoiseField() = delete;
    /* NoiseField();
    NoiseField(int size);
    NoiseField(int size, std::vector<float> &&temperature, std::vector<float> &&humidity, std::vector<float> &&ocean, std::vector<float> &&river);
    // NoiseField(const NoiseField &other) = delete;
    NoiseField(const NoiseField &&other);
    NoiseField &operator=(const NoiseField &other) = delete;
    NoiseField &operator=(const NoiseField &&other); */

    // size_t size() const;
};

class Heightfield {
public:
    std::vector<float> heightField;
    std::vector<unsigned char> biomesVectorField;
    std::vector<float> biomesWeightsVectorField;

    // Heightfield &operator=(const Heightfield &&other);
};

//

template <typename T, typename ChunkType, T(fn)(DCInstance *, ChunkType *)>
class CachedValue {
public:
    T value;
    Mutex mutex;
    std::atomic<bool> flag;

    CachedValue() : flag(false) {}
    ~CachedValue() {}

    void ensure(DCInstance *inst, ChunkType *chunk) {
        /* EM_ASM({
            console.log('cached value check', $0);
        }, (int)flag.load()); */
        if (!flag.load()) {
            std::unique_lock<Mutex> lock(mutex);
            if (!flag.load()) {
                value = fn(inst, chunk);
                flag.store(true);
            }
        }
    }
};

//

class Chunk2D
{
public:
    static NoiseField initNoiseField(DCInstance *inst, Chunk2D *chunk);
    static std::vector<uint8_t> initBiomesField(DCInstance *inst, Chunk2D *chunk);
    static Heightfield initHeightField(DCInstance *inst, Chunk2D *chunk);
    static std::vector<float> initWaterField(DCInstance *inst, Chunk2D *chunk);

    //

    vm::ivec2 min;
    // int size;
    int lod;
    // int gridPoints;

    CachedValue<NoiseField, Chunk2D, initNoiseField> cachedNoiseField;
    CachedValue<std::vector<uint8_t>, Chunk2D, initBiomesField> cachedBiomesField;
    CachedValue<Heightfield, Chunk2D, initHeightField> cachedHeightField;
    CachedValue<std::vector<float>, Chunk2D, initWaterField> cachedWaterField;

    // Chunk2D() = delete;
    // Chunk2D(Chunk2D &&other);
    // Chunk2D(const Chunk2D &other) = delete;
    Chunk2D(const vm::ivec2 chunkMin, const int lod);
    // Chunk2D &operator=(const Chunk2D &other) = delete;
    // Chunk2D &operator=(const Chunk2D &&other);

    // generation
    void generate(DCInstance *inst, int flags);

    // noises
    float getTemperatureLocal(const int lx, const int lz) const;
    float getHumidityLocal(const int lx, const int lz) const;
    // float getWaterFieldLocal(const int lx, const int lz) const;

    // biomes
    unsigned char getCachedBiome(const int lx, const int lz) const;
    void getCachedInterpolatedBiome2D(const vm::vec2 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights) const;

    // height
    void getCachedHeightfield(float *heights) const;
};

//

class Chunk3D {
public:
    static std::vector<uint8_t> initSkylightField(DCInstance *inst, Chunk3D *chunk);
    static std::vector<uint8_t> initAoField(DCInstance *inst, Chunk3D *chunk);
    static std::vector<float> initSdf(DCInstance *inst, Chunk3D *chunk);
    static std::vector<float> initWaterSdf(DCInstance *inst, Chunk3D *chunk);
    static std::vector<float> initDamageSdf(DCInstance *inst, Chunk3D *chunk);

    //

    vm::ivec3 min;
    // int size;
    int lod;
    // int gridPoints;
    Chunk2D *chunk2d;
    // std::vector<uint8_t> cachedSkylightField;
    // std::vector<uint8_t> cachedAoField;
    // std::vector<float> cachedSdf;
    // std::vector<float> cachedWaterSdf;
    // std::vector<float> cachedDamageSdf;
    CachedValue<std::vector<uint8_t>, Chunk3D, initSkylightField> cachedSkylightField;
    CachedValue<std::vector<uint8_t>, Chunk3D, initAoField> cachedAoField;
    CachedValue<std::vector<float>, Chunk3D, initSdf> cachedSdf;
    CachedValue<std::vector<float>, Chunk3D, initWaterSdf> cachedWaterSdf;
    CachedValue<std::vector<float>, Chunk3D, initDamageSdf> cachedDamageSdf;
    
    // Chunk3D() = delete;
    // Chunk3D(Chunk3D &&other);
    // Chunk3D(const Chunk3D &other) = delete;
    Chunk3D(const vm::ivec3 chunkMin, const int lod, Chunk2D *chunk2d);
    // Chunk3D &operator=(const Chunk3D &other) = delete;
    // Chunk3D &operator=(const Chunk3D &&other);

    // generation
    void generate(DCInstance *inst, int flags);

    // biomes
    void getCachedInterpolatedBiome3D(const vm::vec3 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights) const;

    // lighting

    // sdf
    float getCachedInterpolatedSdf(const float x, const float y, const float z) const;
    float getCachedDamageInterpolatedSdf(const float x, const float y, const float z) const;
    float getCachedWaterInterpolatedSdf(const float x, const float y, const float z) const;

    // skylight
    void getCachedSkylight(unsigned char *skylights) const;
    void getCachedAo(unsigned char *aos) const;
    // unsigned char getSkylightLocal(const int lx, const int ly, const int lz) const; // XXX not used?
    // unsigned char getAoLocal(const int lx, const int ly, const int lz) const;

    // signed distance field function for a box at the origin
    // returns negative for points inside the box, zero at the box's surface, and positive for points outside the box
    // sx sy sz is the size of the box. the box goes from -sx/2 to sx/2, -sy/2 to sy/2, -sz/2 to sz/2
    // px py pz is the point to check
    static float signedDistanceToBox(float sx, float sy, float sz, float px, float py, float pz);

    // signed distance to sphere
    // returns negative for points inside the sphere, zero at the sphere's surface, and positive for points outside the sphere
    // cx, cy, cz is the center of the sphere. r is the radius. px, py, pz is the point to check
    static float signedDistanceToSphere(float cx, float cy, float cz, float r, float px, float py, float pz);

    void patchFrontier(std::vector<bool> &erased);

    bool addSphereDamage(const float &x, const float &y, const float &z, const float radius);
    bool removeSphereDamage(const float &x, const float &y, const float &z, const float radius);

    bool addCubeDamage(
        const float &x, const float &y, const float &z,
        const float &qx, const float &qy, const float &qz, const float &qw,
        const float &sx, const float &sy, const float &sz
    );
    bool removeCubeDamage(
        const float &x, const float &y, const float &z,
        const float &qx, const float &qy, const float &qz, const float &qw,
        const float &sx, const float &sy, const float &sz
    );

    // void injectDamage(float *damageBuffer);
};

#endif // _CHUNK_H_