#ifndef _CHUNK_H_
#define _CHUNK_H_

#include "vectorMath.h"
#include "biomes.h"
#include "sync.h"
#include "cache.h"
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

class Chunk2D
{
public:
    vm::ivec2 min;
    // int size;
    int lod;
    // int gridPoints;

    // Chunk2D() = delete;
    // Chunk2D(Chunk2D &&other);
    // Chunk2D(const Chunk2D &other) = delete;
    Chunk2D(const vm::ivec2 chunkMin, const int lod);
    // Chunk2D &operator=(const Chunk2D &other) = delete;
    // Chunk2D &operator=(const Chunk2D &&other);

    // generation
    // void generate(DCInstance *inst, int flags);

    // noises
    // float getTemperatureLocal(const int lx, const int lz) const;
    // float getHumidityLocal(const int lx, const int lz) const;
    // float getWaterFieldLocal(const int lx, const int lz) const;

    // height
    // void getCachedHeightfield(float *heights) const;
};

//

class Chunk3D {
public:
    vm::ivec3 min;
    int lod;
    Chunk2D *chunk2d;
    
    // Chunk3D() = delete;
    // Chunk3D(Chunk3D &&other);
    // Chunk3D(const Chunk3D &other) = delete;
    Chunk3D(const vm::ivec3 chunkMin, const int lod, Chunk2D *chunk2d);
    // Chunk3D &operator=(const Chunk3D &other) = delete;
    // Chunk3D &operator=(const Chunk3D &&other);

    // generation
    // void generate(DCInstance *inst, int flags);

    // biomes
    // void getCachedInterpolatedBiome3D(const vm::vec3 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights) const;

    // lighting

    // signed distance field function for a box at the origin
    // returns negative for points inside the box, zero at the box's surface, and positive for points outside the box
    // sx sy sz is the size of the box. the box goes from -sx/2 to sx/2, -sy/2 to sy/2, -sz/2 to sz/2
    // px py pz is the point to check
    static float signedDistanceToBox(float sx, float sy, float sz, float px, float py, float pz);

    // signed distance to sphere
    // returns negative for points inside the sphere, zero at the sphere's surface, and positive for points outside the sphere
    // cx, cy, cz is the center of the sphere. r is the radius. px, py, pz is the point to check
    static float signedDistanceToSphere(float cx, float cy, float cz, float r, float px, float py, float pz);

    void patchFrontier(DCInstance *inst, std::unordered_map<uint64_t, bool> &erased);

    bool addSphereDamage(DCInstance *inst, const float &x, const float &y, const float &z, const float radius);
    bool removeSphereDamage(DCInstance *inst, const float &x, const float &y, const float &z, const float radius);

    bool addCubeDamage(
        DCInstance *inst,
        const float &x, const float &y, const float &z,
        const float &qx, const float &qy, const float &qz, const float &qw,
        const float &sx, const float &sy, const float &sz
    );
    bool removeCubeDamage(
        DCInstance *inst,
        const float &x, const float &y, const float &z,
        const float &qx, const float &qy, const float &qz, const float &qw,
        const float &sx, const float &sy, const float &sz
    );

    // void injectDamage(float *damageBuffer);
};

#endif // _CHUNK_H_