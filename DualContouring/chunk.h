#ifndef _CHUNK_H_
#define _CHUNK_H_

#include "main.h"
#include "vectorMath.h"
// #include "../FastNoise.h"
#include "../hash.h"
#include "../util.h"
#include "../vector.h"
#include "../worley.h"
#include "biomes.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>
#include <vector>
#include <deque>

const float MAX_HEIGHT = 20.f;
const int numBiomes = (int)BIOME::NUM_BIOMES;

enum GenerateFlags : int {
    GF_NONE = 0,
    GF_NOISE = 1 << 0,
    GF_BIOMES = 1 << 1,
    GF_HEIGHTFIELD = 1 << 2,
    GF_AOFIELD = 1 << 3,
    GF_SDF = 1 << 4,
    // GF_ALL = GF_NOISE | GF_BIOMES | GF_HEIGHTFIELD | GF_SDF
};

class NoiseField {
public:
    // int size;
    std::vector<float> temperature;
    std::vector<float> humidity;
    std::vector<float> ocean;
    std::vector<float> river;

    // NoiseField() = delete;
    NoiseField() {}
    NoiseField(int size) :
        // size(size),
        temperature(size * size),
        humidity(size * size),
        ocean(size * size),
        river(size * size)
        {}
    NoiseField(int size, std::vector<float> &&temperature, std::vector<float> &&humidity, std::vector<float> &&ocean, std::vector<float> &&river) :
        // size(size),
        temperature(std::move(temperature)),
        humidity(std::move(humidity)),
        ocean(std::move(ocean)),
        river(std::move(river))
        {}
    // NoiseField(const NoiseField &other) = delete;
    NoiseField(const NoiseField &&other) :
        // size(other.size),
        temperature(std::move(other.temperature)),
        humidity(std::move(other.humidity)),
        ocean(std::move(other.ocean)),
        river(std::move(other.river))
        {}
    NoiseField &operator=(const NoiseField &other) = delete;
    NoiseField &operator=(const NoiseField &&other) {
        // size = other.size;
        temperature = std::move(other.temperature);
        humidity = std::move(other.humidity);
        ocean = std::move(other.ocean);
        river = std::move(other.river);
        return *this;
    }

    size_t size() const {
        return temperature.size() + humidity.size() + ocean.size() + river.size();
    }
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
    std::vector<uint8_t> cachedAoField;
    std::vector<float> cachedSdf;
    
    Chunk() = delete;
    Chunk(Chunk &&other) :
        size(other.size),
        gridPoints(other.gridPoints),
        min(other.min),
        lod(other.lod),
        cachedNoiseField(std::move(other.cachedNoiseField)),
        cachedBiomesField(std::move(other.cachedBiomesField)),
        cachedBiomesVectorField(std::move(other.cachedBiomesVectorField)),
        cachedBiomesWeightsVectorField(std::move(other.cachedBiomesWeightsVectorField)),
        cachedHeightField(std::move(other.cachedHeightField)),
        cachedAoField(std::move(other.cachedAoField)),
        cachedSdf(std::move(other.cachedSdf))
        {}
    Chunk(const Chunk &other) = delete;
    Chunk(const vm::ivec3 chunkMin, const int &lod, GenerateFlags flags) :
                                       min(chunkMin),
                                       size(DualContouring::chunkSize),
                                       gridPoints(size + 3 + lod),
                                       lod(lod)
    {
        generate(flags);
    };
    Chunk &operator=(const Chunk &other) = delete;
    Chunk &operator=(const Chunk &&other) {
        size = other.size;
        gridPoints = other.gridPoints;
        min = other.min;
        lod = other.lod;
        cachedNoiseField = std::move(other.cachedNoiseField);
        cachedBiomesField = std::move(other.cachedBiomesField);
        cachedBiomesVectorField = std::move(other.cachedBiomesVectorField);
        cachedBiomesWeightsVectorField = std::move(other.cachedBiomesWeightsVectorField);
        cachedHeightField = std::move(other.cachedHeightField);
        cachedAoField = std::move(other.cachedAoField);
        cachedSdf = std::move(other.cachedSdf);
        return *this;
    }

    void generate(int flags) {
        // dependencies
        if (flags & GenerateFlags::GF_AOFIELD) {
            flags |= (GenerateFlags)GenerateFlags::GF_SDF;
        }
        if (flags & GenerateFlags::GF_SDF) {
            flags |= (GenerateFlags)GenerateFlags::GF_HEIGHTFIELD;
        }
        if (flags & GenerateFlags::GF_HEIGHTFIELD) {
            flags |=  (GenerateFlags)GenerateFlags::GF_BIOMES;
        }
        if (flags & GenerateFlags::GF_BIOMES) {
            flags |= (GenerateFlags)GenerateFlags::GF_NOISE;
        }

        // generate
        if (flags & GenerateFlags::GF_NOISE) {
            if (cachedNoiseField.size() == 0) {
                initNoiseField();
            }
        }
        if (flags & GenerateFlags::GF_BIOMES) {
            if (cachedBiomesField.size() == 0) {
                initBiomesField();
            }
        }
        if (flags & GenerateFlags::GF_HEIGHTFIELD) {
            if (cachedHeightField.size() == 0) {
                initHeightField();
            }
        }
        if (flags & GenerateFlags::GF_SDF) {
            if (cachedSdf.size() == 0) {
                initSdf();
            }
        }
        if (flags & GenerateFlags::GF_AOFIELD) {
            if (cachedAoField.size() == 0) {
                initAoField();
            }
        }
    }
    void initNoiseField() {
        cachedNoiseField.temperature.resize(size * size);
        cachedNoiseField.humidity.resize(size * size);
        cachedNoiseField.ocean.resize(size * size);
        cachedNoiseField.river.resize(size * size);
        for (int dz = 0; dz < size; dz++)
        {
            for (int dx = 0; dx < size; dx++)
            {
                int index = dx + dz * size;
                int ax = dx + min.x;
                int az = dz + min.z;

                float tNoise = (float)DualContouring::noises->temperatureNoise.in2D(ax, az);
                cachedNoiseField.temperature[index] = tNoise;

                float hNoise = (float)DualContouring::noises->humidityNoise.in2D(ax, az);
                cachedNoiseField.humidity[index] = hNoise;

                float oNoise = (float)DualContouring::noises->oceanNoise.in2D(ax, az);
                cachedNoiseField.ocean[index] = oNoise;

                float rNoise = (float)DualContouring::noises->riverNoise.in2D(ax, az);
                cachedNoiseField.river[index] = rNoise;
            }
        }
    }
    void initBiomesField() {
        cachedBiomesField.resize(size * size);
        for (int dz = 0; dz < size; dz++)
        {
            for (int dx = 0; dx < size; dx++)
            {
                int index = dx + dz * size;
                unsigned char &biome = cachedBiomesField[index];
                biome = 0xFF;

                float temperatureNoise = cachedNoiseField.temperature[index];
                float humidityNoise = cachedNoiseField.humidity[index];
                float oceanNoise = cachedNoiseField.ocean[index];
                float riverNoise = cachedNoiseField.river[index];
                
                if (oceanNoise < (80.0f / 255.0f)) {
                    biome = (unsigned char)BIOME::biOcean;
                }
                if (biome == 0xFF) {
                    const float range = 0.022f;
                    if (riverNoise > 0.5f - range && riverNoise < 0.5f + range) {
                        biome = (unsigned char)BIOME::biRiver;
                    }
                }
                if (std::pow(temperatureNoise, 1.3f) < ((4.0f * 16.0f) / 255.0f)) {
                    if (biome == (unsigned char)BIOME::biOcean) {
                        biome = (unsigned char)BIOME::biFrozenOcean;
                    } else if (biome == (unsigned char)BIOME::biRiver) {
                        biome = (unsigned char)BIOME::biFrozenRiver;
                    }
                }
                if (biome == 0xFF) {
                    float temperatureNoise2 = vm::clamp(std::pow(temperatureNoise, 1.3f), 0.f, 1.f);
                    float humidityNoise2 = vm::clamp(std::pow(humidityNoise, 1.3f), 0.f, 1.f);
                    
                    int t = (int)std::floor(temperatureNoise2 * 16.0f);
                    int h = (int)std::floor(humidityNoise2 * 16.0f);
                    biome = (unsigned char)BIOMES_TEMPERATURE_HUMIDITY[t + 16 * h];
                }
            }
        }
    }
    void initHeightField() {
        cachedBiomesVectorField.resize(gridPoints * gridPoints * 4);
        cachedBiomesWeightsVectorField.resize(gridPoints * gridPoints * 4);
        cachedHeightField.resize(gridPoints * gridPoints);
        for (int dz = 0; dz < gridPoints; dz++)
        {
            for (int dx = 0; dx < gridPoints; dx++)
            {
                int index2D = dx + dz * gridPoints;
                int ax = dx + min.x - 1;
                int az = dz + min.z - 1;

                std::unordered_map<unsigned char, unsigned int> biomeCounts(numBiomes);
                int numSamples = 0;
                for (int dz = -size/2; dz < size/2; dz++) {
                    for (int dx = -size/2; dx < size/2; dx++) {
                        vm::ivec2 iWorldPosition(ax + dx, az + dz);
                        unsigned char b = DualContouring::getBiome(iWorldPosition, lod);
                        biomeCounts[b]++;
                        numSamples++;
                    }
                }

                std::vector<unsigned char> biomes;
                biomes.resize(biomeCounts.size());
                int index = 0;
                for(auto kv : biomeCounts) {
                    biomes[index++] = kv.first;
                }
                // sort by increasing occurence count of the biome
                std::sort(biomes.begin(), biomes.end(), [&biomeCounts](unsigned char b1, unsigned char b2) {
                    return biomeCounts[b1] > biomeCounts[b2];
                });

                for (size_t i = 0; i < 4; i++) {
                    if (i < biomes.size()) {
                        cachedBiomesVectorField[index2D * 4 + i] = biomes[i];
                        cachedBiomesWeightsVectorField[index2D * 4 + i] = (float)biomeCounts[biomes[i]] / (float)numSamples;
                    } else {
                        cachedBiomesVectorField[index2D * 4 + i] = 0;
                        cachedBiomesWeightsVectorField[index2D * 4 + i] = 0;
                    }
                }

                float elevationSum = 0.f;
                vm::vec2 fWorldPosition(ax, az);
                for (auto const &iter : biomeCounts) {
                    elevationSum += iter.second * DualContouring::getComputedBiomeHeight(iter.first, fWorldPosition, lod);
                }
                
                float elevation = elevationSum / (float)numSamples;
                cachedHeightField[index2D] = elevation;
            }
        }
    }
    void initAoField() {
        cachedAoField.resize(size * size * size);
        for (int y = 0; y < size; y++) {
            int ly = y + 1;
            
            for (int z = 0; z < size; z++) {
                int lz = z + 1;

                for (int x = 0; x < size; x++) {
                    int lx = x + 1;

                    unsigned char numOpens = 0;
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dz = -1; dz <= 1; dz++) {
                            for (int dx = -1; dx <= 1; dx++) {
                                int sdfIndex = (lx + dx) + (ly + dy) * gridPoints + (lz + dz) * gridPoints * gridPoints;
                                numOpens += (unsigned char)(cachedSdf[sdfIndex] >= 0.f);
                            }
                        }
                    }

                    int aoIndex = lx + ly * size + lz * size * size;
                    cachedAoField[aoIndex] = numOpens;
                }
            }
        }
    }
    void initSdf() {
        cachedSdf.resize(gridPoints * gridPoints * gridPoints, MAX_HEIGHT);
        for (int dz = 0; dz < gridPoints; dz++)
        {
            for (int dx = 0; dx < gridPoints; dx++)
            {
                int index2D = dx + dz * gridPoints;
                float height = cachedHeightField[index2D];
                
                for (int dy = 0; dy < gridPoints; dy++)
                {
                    int index3D = dx + dz * gridPoints + dy * gridPoints * gridPoints;
                    
                    int ax = min.x + dx - 1;
                    int ay = min.y + dy - 1;
                    int az = min.z + dz - 1;

                    // height
                    float heightValue = (float)ay - height;
                    heightValue = std::min(
                        std::max(
                            heightValue,
                            (float)-1
                        ),
                        (float)1
                    );

                    // caves
                    float caveValue = getComputedCaveNoise(ax, ay, az) * 1.1f;
                    float f = heightValue + caveValue;

                    // result
                    cachedSdf[index3D] = f;
                }
            }
        }
    }

    // noises
    float getTemperatureLocal(const int lx, const int lz) const {
        int index = lx + lz * size;
        return cachedNoiseField.temperature[index];
    }
    float getHumidityLocal(const int lx, const int lz) const {
        int index = lx + lz * size;
        return cachedNoiseField.humidity[index];
    }

    // biomes
    unsigned char getCachedBiome(const int lx, const int lz) const {
        int index = lx + lz * size;
        return cachedBiomesField[index];
    }
    void getCachedInterpolatedBiome2D(const float x, const float z, vm::ivec4 &biome, vm::vec4 &biomeWeights) const {
        int lx = int(x) - min.x + 1;
        int lz = int(z) - min.z + 1;
        int index2D = lx + lz * gridPoints;

        biome.x = cachedBiomesVectorField[index2D * 4];
        biome.y = cachedBiomesVectorField[index2D * 4 + 1];
        biome.z = cachedBiomesVectorField[index2D * 4 + 2];
        biome.w = cachedBiomesVectorField[index2D * 4 + 3];

        biomeWeights.x = cachedBiomesWeightsVectorField[index2D * 4];
        biomeWeights.y = cachedBiomesWeightsVectorField[index2D * 4 + 1];
        biomeWeights.z = cachedBiomesWeightsVectorField[index2D * 4 + 2];
        biomeWeights.w = cachedBiomesWeightsVectorField[index2D * 4 + 3];
    }
    void getCachedInterpolatedBiome3D(const float x, const float y, const float z, vm::ivec4 &biome, vm::vec4 &biomeWeights) const {
        getCachedInterpolatedBiome2D(x, z, biome, biomeWeights);

        int lx = int(x) - min.x + 1;
        int ly = int(y) - min.y + 1;
        int lz = int(z) - min.z + 1;
        int heightfieldIndex = lx + lz * gridPoints;
        float heightValue = cachedHeightField[heightfieldIndex];
        int sdfIndex = lx + lz * gridPoints + ly * gridPoints * gridPoints;
        float sdfValue = cachedSdf[sdfIndex];

        bool neighborHeightsValid = true;
        for (int dx = -1; dx <= 1; dx += 2) {
            for (int dz = -1; dz <= 1; dz += 2) {
                int lx2 = lx + dx;
                int lz2 = lz + dz;
                if (lx2 < 0 || lx2 >= gridPoints || lz2 < 0 || lz2 >= gridPoints) {
                    std::cout << "ERROR: getCachedInterpolatedBiome3D out of bounds: " << lx2 << " " << lz2 << std::endl;
                    abort();
                }
                int neighborHeightfieldIndex = lx2 + lz2 * gridPoints;
                float heightValue = cachedHeightField[heightfieldIndex];
                if (y + 3 > heightValue) {
                    neighborHeightsValid = false;
                    break;
                }
            }
            if (!neighborHeightsValid) {
                break;
            }
        }

        if (neighborHeightsValid) {
            if (y < heightValue - 12) {
                unsigned char firstBiome = (unsigned char)BIOME::teStone;
                biome.w = biome.z;
                biome.z = biome.y;
                biome.y = biome.x;
                biome.x = firstBiome;
            } else if (y < heightValue - 2) {
                unsigned char firstBiome = (unsigned char)BIOME::teDirt;
                biome.w = biome.z;
                biome.z = biome.y;
                biome.y = biome.x;
                biome.x = firstBiome;
            }
        }
    }
    /* std::vector<unsigned char> getBiomesContainedInChunk() {
        std::unordered_map<unsigned char, bool> seenBiomes;
        for (int dz = 0; dz < gridPoints; dz++)
        {
            for (int dx = 0; dx < gridPoints; dx++)
            {
                int index2D = dx + dz * gridPoints;
                for (int i = 0; i < 4; i++) {
                    unsigned char b = cachedBiomesVectorField[index2D * 4 + i];
                    seenBiomes[b] = true;
                }
            }
        }
        // convert the unordered_map to a vector
        std::vector<unsigned char> biomesVector;
        biomesVector.reserve(seenBiomes.size());
        for(auto kv : seenBiomes) {
            biomesVector.push_back(kv.first);
        }
        return std::move(biomesVector);
    } */

    // height
    float interpolateHeight1D(const float x, const float z) const
    {
        const int xf = std::floor(x);
        const int xc = std::ceil(x);
        const int indexF = xf + z * gridPoints;
        const int indexC = xc + z * gridPoints;
        const float dx = x - xf;
        return lerp(cachedHeightField[indexF], cachedHeightField[indexC], dx);
    }
    float interpolateHeight2D(const float x, const float z) const
    {
        const int zf = std::floor(z);
        const int zc = std::ceil(z);
        const float dz = z - zf;
        return lerp(interpolateHeight1D(x, zf), interpolateHeight1D(x, zc), dz);
    }

    float getRawHeight(const int x, const int z) const
    {
        const int localX = x - min.x + 1;
        const int localZ = z - min.z + 1;
        const int index = localX + localZ * gridPoints;
        return (cachedHeightField.at(index) + 1.f) / 2.f;
    }
    float getCachedInterpolatedHeight(const float x, const float z) const
    {
        const float localX = x - min.x + 1;
        const float localZ = z - min.z + 1;
        return interpolateHeight2D(localX, localZ);
    }

    // cave
    float getComputedCaveNoise(int ax, int ay, int az) {
        double at[3] = {
            (double)ax * 0.1f,
            (double)ay * 0.1f,
            (double)az * 0.1f
        };
        const size_t max_order = 3;
        double *F = new double[max_order];
        double (*delta)[3] = new double[max_order][3];
        uint32_t id;
        Worley(at, max_order, F, delta, &id);
        delete[] F;
        delete[] delta;

        vm::vec3 deltaPoint1(
            delta[0][0],
            delta[0][1],
            delta[0][2]
        );
        float distance1 = length(deltaPoint1);
        
        vm::vec3 deltaPoint3(
            delta[2][0],
            delta[2][1],
            delta[2][2]
        );
        float distance3 = length(deltaPoint3);
        float caveValue = std::min(std::max((distance3 != 0.f ? (distance1 / distance3) : 0.f) * 1.1f, 0.f), 1.f);
        return caveValue;
    }
    /* float getComputedCaveNoiseMulti(int ax, int ay, int az) {
        float result = std::numeric_limits<float>::infinity();
        for (int dy = -2; dy <= 2; dy++) {
          for (int dz = -2; dz <= 2; dz++) {
            for (int dx = -2; dx <= 2; dx++) {
                float caveValue = getComputedCaveNoise(ax + dx, ay + dy, az + dz);
                result = std::min(result, caveValue);
            }
          }
        }
        return result;
    } */

    // sdf
    float getCachedInterpolatedSdf(const float x, const float y, const float z) const {
        const float localX = x - min.x + 1;
        const float localY = y - min.y + 1;
        const float localZ = z - min.z + 1;
        return trilinear<float>(
            vm::vec3(localX, localY, localZ),
            cachedSdf,
            gridPoints
        );
    }

    // ao
    unsigned char getAoLocal(const int lx, const int ly, const int lz) const {
        int index = lx + lz * size + ly * size * size;
        return cachedAoField[index];
    }

    // signed distance field function for a box at the origin
    // returns negative for points inside the box, zero at the box's surface, and positive for points outside the box
    // sx sy sz is the size of the box. the box goes from -sx/2 to sx/2, -sy/2 to sy/2, -sz/2 to sz/2
    // px py pz is the point to check
    static float signedDistanceToBox(float sx, float sy, float sz, float px, float py, float pz) {
        float dx = std::abs(px) - sx / 2;
        float dy = std::abs(py) - sy / 2;
        float dz = std::abs(pz) - sz / 2;
        float d = std::max(std::max(dx, dy), dz);
        return d;
    }

    // signed distance to sphere
    // returns negative for points inside the sphere, zero at the sphere's surface, and positive for points outside the sphere
    // cx, cy, cz is the center of the sphere. r is the radius. px, py, pz is the point to check
    static float signedDistanceToSphere(float cx, float cy, float cz, float r, float px, float py, float pz) {
        float dx = px - cx;
        float dy = py - cy;
        float dz = pz - cz;
        float d = sqrt(dx * dx + dy * dy + dz * dz);
        return d - r;
    }

    void patchFrontier(std::vector<bool> &erased) {
        // frontier points are those that are erased and have a neighbor that is not erased
        std::deque<vm::ivec3> frontierPoints;
        std::vector<bool> frontierSeenPoints(gridPoints * gridPoints * gridPoints, false);
        for (int y = 0; y < gridPoints; y++) {
            for (int z = 0; z < gridPoints; z++) {
                for (int x = 0; x < gridPoints; x++) {
                    int index = x + z * gridPoints + y * gridPoints * gridPoints;
                    
                    if (erased[index]) {
                        bool someNeighborIsNotErased = false;
                        for (int dx = -1; dx <= 1; dx += 2) {
                            for (int dy = -1; dy <= 1; dy += 2) {
                                for (int dz = -1; dz <= 1; dz += 2) {
                                    int neighborIndex = index + dx + dz * gridPoints + dy * gridPoints * gridPoints;
                                    if (!erased[neighborIndex]) {
                                        someNeighborIsNotErased = true;
                                        break;
                                    }
                                }
                                if (someNeighborIsNotErased) {
                                    break;
                                }
                            }
                            if (someNeighborIsNotErased) {
                                break;
                            }
                        }
                        if (someNeighborIsNotErased) {
                            frontierPoints.push_back(vm::ivec3(x, y, z));

                            int index = x + z * gridPoints + y * gridPoints * gridPoints;
                            frontierSeenPoints[index] = true;
                        }
                    }
                }
            }
        }

        // patch each frontier point by extending the min distance from its neighbors
        while (frontierPoints.size() > 0) {
            vm::ivec3 frontierPoint = frontierPoints.front();
            frontierPoints.pop_front();

            int x = frontierPoint.x;
            int y = frontierPoint.y;
            int z = frontierPoint.z;
            int index = x + z * gridPoints + y * gridPoints * gridPoints;

            // compute candidate distances
            std::vector<float> candidateDistances;
            candidateDistances.reserve(8);
            for (int dy = -1; dy <= 1; dy += 2) {
                for (int dz = -1; dz <= 1; dz += 2) {
                    for (int dx = -1; dx <= 1; dx += 2) {
                        int ax = x + dx;
                        int ay = y + dy;
                        int az = z + dz;

                        if (ax >= 0 && ax < gridPoints && az >= 0 && az < gridPoints && ay >= 0 && ay < gridPoints) {
                            int neighborIndex = ax + az * gridPoints + ay * gridPoints * gridPoints;
                            if (!erased[neighborIndex]) {
                                float neighborDistance = cachedSdf[neighborIndex];
                                float localCandidateDistance = neighborDistance + Vec(dx, dy, dz).magnitude();
                                candidateDistances.push_back(localCandidateDistance);
                            }
                        }
                    }
                }
            }
            // sanity check: if we don't have any candidate distances, this should not have been a frontier point!
            if (candidateDistances.size() == 0) {
                std::cerr << "Error: candidateDistances.size() == 0; invalid frontier" << std::endl;
                abort();
            }

            // set new sdf value
            float minDistance = *std::min_element(candidateDistances.begin(), candidateDistances.end());
            cachedSdf[index] = minDistance;
            erased[index] = false;

            // add neighbors to frontier if not seen yet
            for (int dy = -1; dy <= 1; dy += 2) {
                for (int dz = -1; dz <= 1; dz += 2) {
                    for (int dx = -1; dx <= 1; dx += 2) {
                        int ax = x + dx;
                        int ay = y + dy;
                        int az = z + dz;

                        if (ax >= 0 && ax < gridPoints && az >= 0 && az < gridPoints && ay >= 0 && ay < gridPoints) {
                            int neighborIndex = ax + az * gridPoints + ay * gridPoints * gridPoints;
                            if (erased[neighborIndex] && !frontierSeenPoints[neighborIndex]) {
                                frontierPoints.push_back(vm::ivec3(ax, ay, az));
                                frontierSeenPoints[neighborIndex] = true;
                            }
                        }
                    }
                }
            }
        }
    }

    bool addSphereDamage(const float &x, const float &y, const float &z, const float radius) {
        int bx = int(x);
        int by = int(y);
        int bz = int(z);

        bool drew = false;
        for (int ly = 0; ly < gridPoints; ly++) {
            for (int lz = 0; lz < gridPoints; lz++) {
                for (int lx = 0; lx < gridPoints; lx++) {
                    int ax = min.x + lx - 1;
                    int ay = min.y + ly - 1;
                    int az = min.z + lz - 1;

                    float newDistance = signedDistanceToSphere(bx, by, bz, radius, ax, ay, az);

                    int index = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                    float oldDistance = cachedSdf[index];

                    if (newDistance < oldDistance) {
                        cachedSdf[index] = newDistance;
                        drew = true;
                    }
                }
            }
        }
        return drew;
    }
    bool removeSphereDamage(const float &x, const float &y, const float &z, const float radius) {
        int bx = int(x);
        int by = int(y);
        int bz = int(z);

        std::vector<bool> erased(gridPoints * gridPoints * gridPoints, false);

        bool drew = false;
        for (int ly = 0; ly < gridPoints; ly++) {
            for (int lz = 0; lz < gridPoints; lz++) {
                for (int lx = 0; lx < gridPoints; lx++) {
                    int ax = min.x + lx - 1;
                    int ay = min.y + ly - 1;
                    int az = min.z + lz - 1;

                    float newDistance = signedDistanceToSphere(bx, by, bz, radius, ax, ay, az);

                    int index = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                    float oldDistance = cachedSdf[index];

                    if (newDistance <= 0.f || oldDistance >= newDistance) {
                        cachedSdf[index] = (float)size;
                        erased[index] = true;
                        drew = true;
                    }
                }
            }
        }

        if (drew) {
            patchFrontier(erased);
        }

        return drew;
    }

    bool addCubeDamage(
        const float &x, const float &y, const float &z,
        const float &qx, const float &qy, const float &qz, const float &qw,
        const float &sx, const float &sy, const float &sz
    ) {
        Matrix m(Vec{x, y, z}, Quat{qx, qy, qz, qw}, Vec{1, 1, 1});
        Matrix mInverse = m;
        mInverse.invert();

        bool drew = true;
        for (int ly = 0; ly < gridPoints; ly++) {
            for (int lz = 0; lz < gridPoints; lz++) {
                for (int lx = 0; lx < gridPoints; lx++) {
                    int ax = min.x + lx - 1;
                    int ay = min.y + ly - 1;
                    int az = min.z + lz - 1;

                    Vec p = Vec(ax, ay, az).applyMatrix(mInverse);
                    float newDistance = signedDistanceToBox(sx, sy, sz, p.x, p.y, p.z);

                    int index = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                    float oldDistance = cachedSdf[index];

                    if (newDistance < oldDistance) {
                        cachedSdf[index] = newDistance;
                        drew = true;
                    }
                }
            }
        }
        return drew;
    }
    bool removeCubeDamage(
        const float &x, const float &y, const float &z,
        const float &qx, const float &qy, const float &qz, const float &qw,
        const float &sx, const float &sy, const float &sz
    ) {
        Matrix m(Vec{x, y, z}, Quat{qx, qy, qz, qw}, Vec{1, 1, 1});
        Matrix mInverse = m;
        mInverse.invert();

        std::vector<bool> erased(gridPoints * gridPoints * gridPoints, false);

        bool drew = true;
        for (int ly = 0; ly < gridPoints; ly++) {
            for (int lz = 0; lz < gridPoints; lz++) {
                for (int lx = 0; lx < gridPoints; lx++) {
                    int ax = min.x + lx - 1;
                    int ay = min.y + ly - 1;
                    int az = min.z + lz - 1;

                    Vec p = Vec(ax, ay, az).applyMatrix(mInverse);
                    float newDistance = signedDistanceToBox(sx, sy, sz, p.x, p.y, p.z);

                    int index = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                    float oldDistance = cachedSdf[index];

                    if (newDistance <= 0.f || oldDistance >= newDistance) {
                        cachedSdf[index] = (float)size;
                        erased[index] = true;
                        drew = true;
                    }
                }
            }
        }

        if (drew) {
            patchFrontier(erased);
        }

        return drew;
    }

    void injectDamage(float *damageBuffer) {
        std::vector<float> cachedHeightField;
        std::vector<float> cachedSdf(gridPoints * gridPoints * gridPoints);
        memcpy(cachedSdf.data(), this->cachedSdf.data(), sizeof(float) * gridPoints * gridPoints * gridPoints);
    }
};
#endif // _CHUNK_H_