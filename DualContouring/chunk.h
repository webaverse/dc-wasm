#ifndef _CHUNK_H_
#define _CHUNK_H_

#include "vectorMath.h"
#include "biomes.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>
#include <vector>
#include <deque>

class DCInstance;

//

const float MAX_HEIGHT = 20.f;
const int numBiomes = (int)BIOME::NUM_BIOMES;

enum GenerateFlags : int {
    GF_NONE = 0,
    GF_NOISE = 1 << 0,
    GF_BIOMES = 1 << 1,
    GF_HEIGHTFIELD = 1 << 2,
    GF_AOFIELD = 1 << 3,
    GF_SDF = 1 << 4,
    GF_LIQUIDS = 1 << 5,
};

class NoiseField {
public:
    // int size;
    std::vector<float> temperature;
    std::vector<float> humidity;
    std::vector<float> ocean;
    std::vector<float> river;

    // NoiseField() = delete;
    NoiseField();
    NoiseField(int size);
    NoiseField(int size, std::vector<float> &&temperature, std::vector<float> &&humidity, std::vector<float> &&ocean, std::vector<float> &&river);
    // NoiseField(const NoiseField &other) = delete;
    NoiseField(const NoiseField &&other);
    NoiseField &operator=(const NoiseField &other) = delete;
    NoiseField &operator=(const NoiseField &&other);

    size_t size() const;
};

class Chunk
{
public:
    // private:
    int size;
    int gridPoints;
    int lod;
    vm::ivec3 min;
    NoiseField cachedNoiseField;
    std::vector<uint8_t> cachedBiomesField;
    std::vector<unsigned char> cachedBiomesVectorField;
    std::vector<float> cachedBiomesWeightsVectorField;
    std::vector<float> cachedHeightField;
    std::vector<uint8_t> cachedSkylightField;
    std::vector<uint8_t> cachedAoField;
    std::vector<float> cachedSdf;
    std::vector<float> cachedWaterSdf;
    std::vector<float> cachedDamageSdf;
    
    Chunk() = delete;
    Chunk(Chunk &&other);
    Chunk(const Chunk &other) = delete;
    Chunk(const vm::ivec3 chunkMin, const int &lod);
    Chunk &operator=(const Chunk &other) = delete;
    Chunk &operator=(const Chunk &&other);

    void generate(DCInstance *inst, int flags);
    void initNoiseField();
    void initBiomesField();
    void initHeightField(DCInstance *inst);
    void initSkylightField();
    void initAoField();
    void initSdf();
    void initWaterSdf(DCInstance *inst);
    void initDamageSdf();

    // noises
    float getTemperatureLocal(const int lx, const int lz) const;
    float getHumidityLocal(const int lx, const int lz) const;

    // biomes
    unsigned char getCachedBiome(const int lx, const int lz) const;
    void getCachedInterpolatedBiome2D(const vm::vec2 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights) const;
    void getCachedInterpolatedBiome3D(const vm::vec3 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights) const;

    // height
    float interpolateHeight1D(const float x, const float z) const;
    float interpolateHeight2D(const float x, const float z) const;

    float getCachedInterpolatedHeight(const float x, const float z) const;

    // lighting

    // sdf
    float getCachedInterpolatedSdf(const float x, const float y, const float z) const;
    float getCachedDamageInterpolatedSdf(const float x, const float y, const float z) const;
    float getCachedWaterInterpolatedSdf(const float x, const float y, const float z) const;

    // skylight
    void getCachedSkylight(unsigned char *skylights) const;
    void getCachedAo(unsigned char *aos) const;
    unsigned char getSkylightLocal(const int lx, const int ly, const int lz) const; // XXX not used?
    unsigned char getAoLocal(const int lx, const int ly, const int lz) const;

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

    void injectDamage(float *damageBuffer);
};

#endif // _CHUNK_H_