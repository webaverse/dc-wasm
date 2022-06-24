#include "main.h"
#include "vectorMath.h"
// #include "../FastNoise.h"
#include "../hash.h"
#include "../util.h"
#include "../vector.h"
#include "../worley.h"
#include "biomes.h"
#include "chunk.h"
#include "density.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>
#include <vector>
#include <deque>

//

NoiseField::NoiseField() {}
NoiseField::NoiseField(int size) : // size(size),
                                   temperature(size * size),
                                   humidity(size * size),
                                   ocean(size * size),
                                   river(size * size)
{
}
NoiseField::NoiseField(int size, std::vector<float> &&temperature, std::vector<float> &&humidity, std::vector<float> &&ocean, std::vector<float> &&river) : // size(size),
                                                                                                                                                            temperature(std::move(temperature)),
                                                                                                                                                            humidity(std::move(humidity)),
                                                                                                                                                            ocean(std::move(ocean)),
                                                                                                                                                            river(std::move(river))
{
}
NoiseField::NoiseField(const NoiseField &&other) : // size(other.size),
                                                   temperature(std::move(other.temperature)),
                                                   humidity(std::move(other.humidity)),
                                                   ocean(std::move(other.ocean)),
                                                   river(std::move(other.river))
{
}
NoiseField &NoiseField::operator=(const NoiseField &&other)
{
    // size = other.size;
    temperature = std::move(other.temperature);
    humidity = std::move(other.humidity);
    ocean = std::move(other.ocean);
    river = std::move(other.river);
    return *this;
}

size_t NoiseField::size() const
{
    return temperature.size() + humidity.size() + ocean.size() + river.size();
}

//

Chunk::Chunk(Chunk &&other) : size(other.size),
                              gridPoints(other.gridPoints),
                              min(other.min),
                              lod(other.lod),
                              cachedNoiseField(std::move(other.cachedNoiseField)),
                              cachedBiomesField(std::move(other.cachedBiomesField)),
                              cachedBiomesVectorField(std::move(other.cachedBiomesVectorField)),
                              cachedBiomesWeightsVectorField(std::move(other.cachedBiomesWeightsVectorField)),
                              cachedWaterField(std::move(other.cachedWaterField)),
                              cachedHeightField(std::move(other.cachedHeightField)),
                              cachedSkylightField(std::move(other.cachedSkylightField)),
                              cachedAoField(std::move(other.cachedAoField)),
                              cachedSdf(std::move(other.cachedSdf)),
                              cachedDamageSdf(std::move(other.cachedDamageSdf))
{
}
Chunk::Chunk(const vm::ivec3 chunkMin, const int &lod) : min(chunkMin),
                                                         size(DualContouring::chunkSize),
                                                         gridPoints(size + 3 + lod),
                                                         lod(lod) {}
Chunk &Chunk::operator=(const Chunk &&other)
{
    size = other.size;
    gridPoints = other.gridPoints;
    min = other.min;
    lod = other.lod;
    cachedNoiseField = std::move(other.cachedNoiseField);
    cachedBiomesField = std::move(other.cachedBiomesField);
    cachedBiomesVectorField = std::move(other.cachedBiomesVectorField);
    cachedBiomesWeightsVectorField = std::move(other.cachedBiomesWeightsVectorField);
    cachedWaterField = std::move(other.cachedWaterField);
    cachedHeightField = std::move(other.cachedHeightField);
    cachedSkylightField = std::move(other.cachedSkylightField);
    cachedAoField = std::move(other.cachedAoField);
    cachedSdf = std::move(other.cachedSdf);
    cachedDamageSdf = std::move(other.cachedDamageSdf);
    return *this;
}

void Chunk::generate(DCInstance *inst, int flags)
{
    // dependencies
    if (flags & GenerateFlags::GF_AOFIELD)
    {
        flags |= (GenerateFlags)GenerateFlags::GF_SDF;
    }
    if (flags & GenerateFlags::GF_SDF) {
        flags |= (GenerateFlags)GenerateFlags::GF_HEIGHTFIELD;
    }
    if (flags & GenerateFlags::GF_LIQUIDS) {
        flags |= (GenerateFlags)GenerateFlags::GF_WATERFIELD;
    }
    if ((flags & GenerateFlags::GF_HEIGHTFIELD) | (flags & GenerateFlags::GF_WATERFIELD)) {
        flags |= (GenerateFlags)GenerateFlags::GF_BIOMES;
    }
    if (flags & GenerateFlags::GF_BIOMES)
    {
        flags |= (GenerateFlags)GenerateFlags::GF_NOISE;
    }

    // generate
    if (flags & GenerateFlags::GF_NOISE)
    {
        if (cachedNoiseField.size() == 0)
        {
            initNoiseField();
        }
    }
    if (flags & GenerateFlags::GF_BIOMES)
    {
        if (cachedBiomesField.size() == 0)
        {
            initBiomesField();
        }
    }
    if (flags & GenerateFlags::GF_HEIGHTFIELD)
    {
        if (cachedHeightField.size() == 0)
        {
            initHeightField(inst);
        }
    }
    if (flags & GenerateFlags::GF_WATERFIELD)
    {
        if (cachedWaterField.size() == 0)
        {
            initWaterField(inst);
        }
    }
    if (flags & GenerateFlags::GF_SDF)
    {
        if (cachedSdf.size() == 0)
        {
            initSdf();
        }
        if (cachedDamageSdf.size() == 0)
        {
            initDamageSdf();
        }
    }
    if (flags & GenerateFlags::GF_LIQUIDS) {
        if (cachedWaterSdf.size() == 0) {
            initWaterSdf(inst);
        }
    }
    if (flags & GenerateFlags::GF_AOFIELD) {
        if (cachedSkylightField.size() == 0) {
            initSkylightField();
        }
        if (cachedAoField.size() == 0)
        {
            initAoField();
        }
    }
}
void Chunk::initNoiseField()
{
    cachedNoiseField.temperature.resize(size * size);
    cachedNoiseField.humidity.resize(size * size);
    cachedNoiseField.ocean.resize(size * size);
    cachedNoiseField.river.resize(size * size);
    for (int z = 0; z < size; z++)
    {
        for (int x = 0; x < size; x++)
        {
            int index = x + z * size;
            int ax = x * lod + min.x;
            int az = z * lod + min.z;

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
void Chunk::initBiomesField()
{
    cachedBiomesField.resize(size * size);
    for (int z = 0; z < size; z++)
    {
        for (int x = 0; x < size; x++)
        {
            int index = x + z * size;
            unsigned char &biome = cachedBiomesField[index];
            biome = 0xFF;

            float temperatureNoise = cachedNoiseField.temperature[index];
            float humidityNoise = cachedNoiseField.humidity[index];
            float oceanNoise = cachedNoiseField.ocean[index];
            float riverNoise = cachedNoiseField.river[index];

            if (oceanNoise < (80.0f / 255.0f))
            {
                biome = (unsigned char)BIOME::biOcean;
            }
            if (biome == 0xFF)
            {
                const float range = 0.022f;
                if (riverNoise > 0.5f - range && riverNoise < 0.5f + range)
                {
                    biome = (unsigned char)BIOME::biRiver;
                }
            }
            if (std::pow(temperatureNoise, 1.3f) < ((4.0f * 16.0f) / 255.0f))
            {
                if (biome == (unsigned char)BIOME::biOcean)
                {
                    biome = (unsigned char)BIOME::biFrozenOcean;
                }
                else if (biome == (unsigned char)BIOME::biRiver)
                {
                    biome = (unsigned char)BIOME::biFrozenRiver;
                }
            }
            if (biome == 0xFF)
            {
                float temperatureNoise2 = vm::clamp(std::pow(temperatureNoise, 1.3f), 0.f, 1.f);
                float humidityNoise2 = vm::clamp(std::pow(humidityNoise, 1.3f), 0.f, 1.f);

                int t = (int)std::floor(temperatureNoise2 * 16.0f);
                int h = (int)std::floor(humidityNoise2 * 16.0f);
                biome = (unsigned char)BIOMES_TEMPERATURE_HUMIDITY[t + 16 * h];
            }
        }
    }
}
void Chunk::initHeightField(DCInstance *inst)
{
    cachedBiomesVectorField.resize(gridPoints * gridPoints * 4);
    cachedBiomesWeightsVectorField.resize(gridPoints * gridPoints * 4);
    cachedHeightField.resize(gridPoints * gridPoints);
    for (int z = 0; z < gridPoints; z++)
    {
        for (int x = 0; x < gridPoints; x++)
        {
            int index2D = x + z * gridPoints;
            int ax = (x - 1) * lod + min.x;
            int az = (z - 1) * lod + min.z;
            
            // int lx = x - 1;
            // int lz = z - 1;
            // int index2D2 = lx + lz * size;
            // bool isInRange = lx >= 0 && lx < size && lz >= 0 && lz < size;

            std::unordered_map<unsigned char, unsigned int> biomeCounts(numBiomes);
            int numSamples = 0;
            for (int dz = -size/2; dz < size/2; dz++)
            {
                for (int dx = -size/2; dx < size/2; dx++)
                {
                    vm::vec2 worldPosition(ax + dx, az + dz);
                    unsigned char b = inst->getBiome(worldPosition, lod);

                    biomeCounts[b]++;
                    numSamples++;
                }
            }

            std::vector<unsigned char> biomes;
            biomes.resize(biomeCounts.size());
            int index = 0;
            for (auto kv : biomeCounts)
            {
                biomes[index++] = kv.first;
            }
            // sort by increasing occurence count of the biome
            std::sort(biomes.begin(), biomes.end(), [&biomeCounts](unsigned char b1, unsigned char b2)
                      { return biomeCounts[b1] > biomeCounts[b2]; });

            for (size_t i = 0; i < 4; i++)
            {
                if (i < biomes.size())
                {
                    cachedBiomesVectorField[index2D * 4 + i] = biomes[i];
                    cachedBiomesWeightsVectorField[index2D * 4 + i] = (float)biomeCounts[biomes[i]] / (float)numSamples;
                }
                else
                {
                    cachedBiomesVectorField[index2D * 4 + i] = 0;
                    cachedBiomesWeightsVectorField[index2D * 4 + i] = 0;
                }
            }

            float elevationSum = 0.f;
            vm::vec2 fWorldPosition(ax, az);
            for (auto const &iter : biomeCounts)
            {
                elevationSum += iter.second * DualContouring::getComputedBiomeHeight(iter.first, fWorldPosition, lod);
            }

            float elevation = elevationSum / (float)numSamples;
            cachedHeightField[index2D] = elevation;
        }
    }
}
void Chunk::initWaterField(DCInstance *inst)
{
    cachedWaterField.resize(size * size);
    for (int z = 0; z < size; z++)
    {
        for (int x = 0; x < size; x++)
        {
            int ax = x * lod + min.x;
            int az = z * lod + min.z;

            int lx = x;
            int lz = z;
            int index2D = x + z * size;
            
            std::unordered_map<unsigned char, unsigned int> biomeCounts(numBiomes);
            int numSamples = 0;
            for (int dz = -size/2; dz < size/2; dz++)
            {
                for (int dx = -size/2; dx < size/2; dx++)
                {
                    vm::vec2 worldPosition(ax + dx, az + dz);
                    unsigned char b = inst->getBiome(worldPosition, lod);

                    if (isWaterBiome(b)) {
                        cachedWaterField[index2D]++;
                    }
                }
            }
        }
    }
}
void Chunk::initSkylightField()
{
    // std::cout << "init skylight " << min.x << " " << min.y << " " << min.z << std::endl;

    constexpr float maxSkyLight = 8.f;
    cachedSkylightField.resize(gridPoints * gridPoints * gridPoints, maxSkyLight);
    for (int z = 0; z < gridPoints; z++)
    {
        // int lz = z + 1;

        for (int x = 0; x < gridPoints; x++)
        {
            // int lx = x + 1;

            int index2D = x + z * gridPoints;
            float height = cachedHeightField[index2D];
            int topAY = min.y + gridPoints - 1;
            float skylight = std::min(std::max((float)topAY - height + maxSkyLight, 0.f), maxSkyLight);

            for (int y = gridPoints - 1; y >= 0; y--)
            {
                // int ly = y + 1;

                int sdfIndex = x + z * gridPoints + y * gridPoints * gridPoints;
                if (cachedSdf[sdfIndex] < 0.f)
                {
                    skylight = std::min(std::max(skylight - 1.f, 0.f), maxSkyLight);
                }

                int skylightIndex = x + z * gridPoints + y * gridPoints * gridPoints;
                cachedSkylightField[skylightIndex] = skylight;
            }
        }
    }

    // XXX should flood fill the light

    /* for (int x = 0; x < gridPoints; x++)
    {
        for (int z = 0; z < gridPoints; z++)
        {
            // int lz = z + 1;
            // int lx = x + 1;

            // for (int y = gridPoints - 1; y >= 0; y--) {
            for (int y = 0; y < gridPoints; y++)
            {
                // int ly = y + 1;

                int skylightIndex = x + z * gridPoints + y * gridPoints * gridPoints;
                float maxNeighborSkylight = cachedSkylightField[skylightIndex];
                for (int dz = -1; dz <= 1; dz += 2)
                {
                    for (int dx = -1; dx <= 1; dx += 2)
                    {
                        for (int dy = -1; dy <= 1; dy += 2)
                        {
                            int lx = x + dx;
                            int ly = y + dy;
                            int lz = z + dz;

                            float deltaRadius = std::sqrt(dx * dx + dy * dy + dz * dz);

                            if (lx >= 0 && lx < gridPoints && ly >= 0 && ly < gridPoints && lz >= 0 && lz < gridPoints)
                            {
                                int neighborIndex = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                                float skylight = cachedSkylightField[neighborIndex];
                                maxNeighborSkylight = std::max(maxNeighborSkylight, skylight - deltaRadius);
                            }
                        }
                    }
                }

                cachedSkylightField[skylightIndex] = maxNeighborSkylight;
            }
        }
    } */
}
void Chunk::initAoField()
{
    cachedAoField.resize(size * size * size, 3 * 3 * 3);
    for (int y = 0; y < size; y++)
    {
        int ly = y + 1;

        for (int z = 0; z < size; z++)
        {
            int lz = z + 1;

            for (int x = 0; x < size; x++)
            {
                int lx = x + 1;

                unsigned char numOpens = 0;
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            int sdfIndex = (lx + dx) + (lz + dz) * gridPoints + (ly + dy) * gridPoints * gridPoints;
                            numOpens += (unsigned char)(cachedSdf[sdfIndex] >= 0.f);
                        }
                    }
                }

                int aoIndex = x + z * size + y * size * size;
                cachedAoField[aoIndex] = numOpens;
            }
        }
    }
}
void Chunk::initSdf()
{
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

                int ax = min.x + (dx - 1) * lod;
                int ay = min.y + (dy - 1) * lod;
                int az = min.z + (dz - 1) * lod;

                // height
                float heightValue = (float)ay - height;
                heightValue = std::min(
                    std::max(
                        heightValue,
                        (float)-1),
                    (float)1);

                // caves
                float caveValue = DualContouring::getComputedCaveNoise(ax, ay, az) * 1.1f;
                float f = heightValue + caveValue;
                /* f = std::min( // XXX does not fix
                    std::max(
                        f,
                        -1.f
                    ),
                    1.f
                ); */

                // result
                cachedSdf[index3D] = f;
            }
        }
    }
}
void Chunk::initDamageSdf()
{
    cachedDamageSdf.resize(gridPoints * gridPoints * gridPoints, MAX_HEIGHT);
}

// liquids
void Chunk::initWaterSdf(DCInstance *inst) {
    cachedWaterSdf.resize(gridPoints * gridPoints * gridPoints, MAX_HEIGHT);

    const float fSize = (float)gridPoints;
    for (int dz = 0; dz < gridPoints; dz++)
    {
        int az = min.z + dz - 1;
        for (int dx = 0; dx < gridPoints; dx++)
        {
            int ax = min.x + dx - 1;

            float waterValue = -inst->getWater(vm::vec2(ax, az), lod) / fSize;
            // waterValue *= -1.f;
            // waterValue *= -1.1f;
            for (int dy = 0; dy < gridPoints; dy++)
            {
                int ay = min.y + dy - 1;

                float heightValue = (float)ay - waterBaseHeight;
                heightValue = std::min(
                    std::max(
                        heightValue,
                        -1.f
                    ),
                    1.f
                );
                
                float value = std::max(waterValue, heightValue);

                int index3D = dx + dz * gridPoints + dy * gridPoints * gridPoints;
                cachedWaterSdf[index3D] = value;
            }
        }
    }
}

// noises
float Chunk::getTemperatureLocal(const int lx, const int lz) const
{
    int index = lx + lz * size;
    return cachedNoiseField.temperature[index];
}
float Chunk::getHumidityLocal(const int lx, const int lz) const
{
    int index = lx + lz * size;
    return cachedNoiseField.humidity[index];
}
float Chunk::getWaterFieldLocal(const int lx, const int lz) const
{
    int index = lx + lz * size;
    return cachedWaterField[index];
}

// biomes
unsigned char Chunk::getCachedBiome(const int lx, const int lz) const
{
    int index = lx + lz * size;
    return cachedBiomesField[index];
}
void Chunk::getCachedInterpolatedBiome2D(const vm::vec2 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights) const {
    const float &x = worldPosition.x;
    const float &z = worldPosition.y;
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
void Chunk::getCachedInterpolatedBiome3D(const vm::vec3 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights) const {
    const float &x = worldPosition.x;
    const float &z = worldPosition.z;
    const float &y = worldPosition.y;
    if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
        std::cout << "got nan getCachedInterpolatedBiome3D: " << x << " " << y << " " << z << std::endl;
        abort();
    }

    getCachedInterpolatedBiome2D(vm::vec2(worldPosition.x, worldPosition.z), biome, biomeWeights);

    int lx = int(x) - min.x + 1;
    int ly = int(y) - min.y + 1;
    int lz = int(z) - min.z + 1;
    int heightfieldIndex = lx + lz * gridPoints;
    float heightValue = cachedHeightField[heightfieldIndex];
    int sdfIndex = lx + lz * gridPoints + ly * gridPoints * gridPoints;
    float sdfValue = cachedSdf[sdfIndex];

    bool neighborHeightsValid = true;
    for (int dx = -1; dx <= 1; dx += 2)
    {
        for (int dz = -1; dz <= 1; dz += 2)
        {
            int lx2 = lx + dx;
            int lz2 = lz + dz;
            int neighborHeightfieldIndex = lx2 + lz2 * gridPoints;
            float heightValue = cachedHeightField[heightfieldIndex];
            if (y + 3 > heightValue)
            {
                neighborHeightsValid = false;
                break;
            }
        }
        if (!neighborHeightsValid)
        {
            break;
        }
    }

    if (neighborHeightsValid)
    {
        if (y < heightValue - 12)
        {
            unsigned char firstBiome = (unsigned char)BIOME::teStone;
            biome.w = biome.z;
            biome.z = biome.y;
            biome.y = biome.x;
            biome.x = firstBiome;
        }
        else if (y < heightValue - 2)
        {
            unsigned char firstBiome = (unsigned char)BIOME::teDirt;
            biome.w = biome.z;
            biome.z = biome.y;
            biome.y = biome.x;
            biome.x = firstBiome;
        }
    }
}
/* std::vector<unsigned char> Chunk::getBiomesContainedInChunk() {
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
float Chunk::interpolateHeight1D(const float x, const float z) const
{
    const int xf = std::floor(x);
    const int xc = std::ceil(x);
    const int indexF = xf + z * gridPoints;
    const int indexC = xc + z * gridPoints;
    const float dx = x - xf;
    return lerp(cachedHeightField[indexF], cachedHeightField[indexC], dx);
}
float Chunk::interpolateHeight2D(const float x, const float z) const
{
    const int zf = std::floor(z);
    const int zc = std::ceil(z);
    const float dz = z - zf;
    return lerp(interpolateHeight1D(x, zf), interpolateHeight1D(x, zc), dz);
}

/* float Chunk::getRawHeight(const int x, const int z) const
{
    const int localX = x - min.x + 1;
    const int localZ = z - min.z + 1;
    const int index = localX + localZ * gridPoints;
    return (cachedHeightField[index] + 1.f) / 2.f;
} */
void Chunk::getCachedHeightfield(float *heights) const
{
    for (int z = 0; z < size; z++)
    {
        for (int x = 0; x < size; x++)
        {
            int index2D = x + z * size;

            int gridX = x + lod;
            int gridZ = z + lod;
            int gridIndex = gridX + gridZ * gridPoints;

            heights[index2D] = cachedHeightField[gridIndex];
        }
    }
}
float Chunk::getCachedInterpolatedHeight(const float x, const float z) const
{
    const float localX = x - min.x + 1;
    const float localZ = z - min.z + 1;
    return interpolateHeight2D(localX, localZ);
}

// lighting
void Chunk::getCachedSkylight(unsigned char *skylights) const
{
    for (int z = 0; z < size; z++)
    {
        for (int y = 0; y < size; y++)
        {
            for (int x = 0; x < size; x++)
            {
                int dstIndex = x + y * size + z * size * size;

                int lx = x + 1;
                int ly = y + 1;
                int lz = z + 1;
                int srcIndex = lx + lz * gridPoints + ly * gridPoints * gridPoints; // note: output is y-first, but storage is z-first

                skylights[dstIndex] = cachedSkylightField[srcIndex];
            }
        }
    }
}
void Chunk::getCachedAo(unsigned char *aos) const
{
    for (int z = 0; z < size; z++)
    {
        for (int y = 0; y < size; y++)
        {
            for (int x = 0; x < size; x++)
            {
                int dstIndex = x + y * size + z * size * size;
                int srcIndex = x + z * size + y * size * size; // note: output is y-first, but storage is z-first

                aos[dstIndex] = cachedAoField[srcIndex];
            }
        }
    }
}

// sdf
float Chunk::getCachedInterpolatedSdf(const float x, const float y, const float z) const
{
    const float localX = x - min.x + 1;
    const float localY = y - min.y + 1;
    const float localZ = z - min.z + 1;
    return trilinear<float>(
        vm::vec3(localX, localY, localZ),
        cachedSdf,
        gridPoints);
}
float Chunk::getCachedWaterInterpolatedSdf(const float x, const float y, const float z) const {
    const float localX = x - min.x + 1;
    const float localY = y - min.y + 1;
    const float localZ = z - min.z + 1;
    return trilinear<float>(
        vm::vec3(localX, localY, localZ),
        cachedWaterSdf,
        gridPoints
    );
}
float Chunk::getCachedDamageInterpolatedSdf(const float x, const float y, const float z) const {
    const float localX = x - min.x + 1;
    const float localY = y - min.y + 1;
    const float localZ = z - min.z + 1;
    return trilinear<float>(
        vm::vec3(localX, localY, localZ),
        cachedDamageSdf,
        gridPoints);
}

// skylight
/* unsigned char Chunk::getSkylightLocal(const int lx, const int ly, const int lz) const
{
    int index = lx + lz * gridPoints + ly * gridPoints * gridPoints;
    return cachedSkylightField[index];
}

// ao
unsigned char Chunk::getAoLocal(const int lx, const int ly, const int lz) const
{
    int index = lx + lz * size + ly * size * size;
    return cachedAoField[index];
} */

// signed distance field function for a box at the origin
// returns negative for points inside the box, zero at the box's surface, and positive for points outside the box
// sx sy sz is the size of the box. the box goes from -sx/2 to sx/2, -sy/2 to sy/2, -sz/2 to sz/2
// px py pz is the point to check
float Chunk::signedDistanceToBox(float sx, float sy, float sz, float px, float py, float pz)
{
    float dx = std::abs(px) - sx / 2;
    float dy = std::abs(py) - sy / 2;
    float dz = std::abs(pz) - sz / 2;
    float d = std::max(std::max(dx, dy), dz);
    return d;
}

// signed distance to sphere
// returns negative for points inside the sphere, zero at the sphere's surface, and positive for points outside the sphere
// cx, cy, cz is the center of the sphere. r is the radius. px, py, pz is the point to check
float Chunk::signedDistanceToSphere(float cx, float cy, float cz, float r, float px, float py, float pz)
{
    float dx = px - cx;
    float dy = py - cy;
    float dz = pz - cz;
    float d = sqrt(dx * dx + dy * dy + dz * dz);
    return d - r;
}

void Chunk::patchFrontier(std::vector<bool> &erased)
{
    std::function<void(const vm::ivec3 &)> tryIndex = [&](const vm::ivec3 &v) -> void
    {
        int index = v.x + v.y * gridPoints + v.z * gridPoints * gridPoints;
        if (erased[index])
        {
            std::vector<vm::ivec3> unerasedNeighbors;
            for (int dx = -1; dx <= 1; dx += 2)
            {
                for (int dy = -1; dy <= 1; dy += 2)
                {
                    for (int dz = -1; dz <= 1; dz += 2)
                    {
                        int ax = v.x + dx;
                        int ay = v.y + dy;
                        int az = v.z + dz;

                        int neighborIndex = ax + ay * gridPoints + az * gridPoints * gridPoints;
                        if (!erased[neighborIndex])
                        {
                            unerasedNeighbors.push_back(vm::ivec3(dx, dy, dz));
                            break;
                        }
                    }
                }
            }
            if (unerasedNeighbors.size() > 0)
            {
                // compute the current sdf min distance from the neighbors
                float minDistance = cachedDamageSdf[index];
                for (auto &neighborOffset : unerasedNeighbors)
                {
                    int ax = v.x + neighborOffset.x;
                    int ay = v.y + neighborOffset.y;
                    int az = v.z + neighborOffset.z;

                    int neighborIndex = ax + ay * gridPoints + az * gridPoints * gridPoints;
                    float neighborDamageSdf = cachedDamageSdf[neighborIndex];
                    float extraDistance = length(neighborOffset);
                    minDistance = std::min(minDistance, neighborDamageSdf + extraDistance);
                }
                cachedDamageSdf[index] = minDistance;
                erased[index] = false;
                unerasedNeighbors.clear();

                for (int dx = -1; dx <= 1; dx += 2)
                {
                    for (int dy = -1; dy <= 1; dy += 2)
                    {
                        for (int dz = -1; dz <= 1; dz += 2)
                        {
                            int ax = v.x + dx;
                            int ay = v.y + dy;
                            int az = v.z + dz;
                            tryIndex(vm::ivec3(ax, ay, az));
                        }
                    }
                }
            }
        }
    };

    for (int ly = 0; ly < gridPoints; ly++)
    {
        for (int lz = 0; lz < gridPoints; lz++)
        {
            for (int lx = 0; lx < gridPoints; lx++)
            {
                tryIndex(vm::ivec3(lx, ly, lz));
            }
        }
    }
}

bool Chunk::addSphereDamage(const float &x, const float &y, const float &z, const float radius)
{
    // int bx = int(x);
    // int by = int(y);
    // int bz = int(z);

    bool drew = false;
    for (int ly = 0; ly < gridPoints; ly++)
    {
        for (int lz = 0; lz < gridPoints; lz++)
        {
            for (int lx = 0; lx < gridPoints; lx++)
            {
                int ax = min.x + lx - 1;
                int ay = min.y + ly - 1;
                int az = min.z + lz - 1;

                float newDistance = signedDistanceToSphere(x, y, z, radius, ax, ay, az);

                int index = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                float oldDistance = cachedDamageSdf[index];

                if (newDistance < oldDistance)
                {
                    cachedDamageSdf[index] = newDistance;
                    drew = true;
                }
            }
        }
    }
    return drew;
}
bool Chunk::removeSphereDamage(const float &x, const float &y, const float &z, const float radius)
{
    // int bx = int(x);
    // int by = int(y);
    // int bz = int(z);

    std::vector<bool> erased(gridPoints * gridPoints * gridPoints, false);

    bool drew = false;
    for (int ly = 0; ly < gridPoints; ly++)
    {
        for (int lz = 0; lz < gridPoints; lz++)
        {
            for (int lx = 0; lx < gridPoints; lx++)
            {
                int ax = min.x + lx - 1;
                int ay = min.y + ly - 1;
                int az = min.z + lz - 1;

                float newDistance = signedDistanceToSphere(x, y, z, radius, ax, ay, az);

                int index = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                float oldDistance = cachedDamageSdf[index];

                if (
                    newDistance <= 0.f ||      // new point is inside the sphere
                    newDistance <= oldDistance // new point affects this index
                )
                {
                    cachedDamageSdf[index] = (float)size; // max outside distance for a chunk
                    erased[index] = true;
                    drew = true;
                }
            }
        }
    }

    if (drew)
    {
        patchFrontier(erased);
    }

    return drew;
}

bool Chunk::addCubeDamage(
    const float &x, const float &y, const float &z,
    const float &qx, const float &qy, const float &qz, const float &qw,
    const float &sx, const float &sy, const float &sz)
{
    Matrix m(Vec{x, y, z}, Quat{qx, qy, qz, qw}, Vec{1, 1, 1});
    Matrix mInverse = m;
    mInverse.invert();

    bool drew = true;
    for (int ly = 0; ly < gridPoints; ly++)
    {
        for (int lz = 0; lz < gridPoints; lz++)
        {
            for (int lx = 0; lx < gridPoints; lx++)
            {
                int ax = min.x + lx - 1;
                int ay = min.y + ly - 1;
                int az = min.z + lz - 1;

                Vec p = Vec(ax, ay, az).applyMatrix(mInverse);
                float newDistance = signedDistanceToBox(sx, sy, sz, p.x, p.y, p.z);

                int index = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                float oldDistance = cachedDamageSdf[index];

                if (newDistance < oldDistance)
                {
                    cachedDamageSdf[index] = newDistance;
                    drew = true;
                }
            }
        }
    }
    return drew;
}
bool Chunk::removeCubeDamage(
    const float &x, const float &y, const float &z,
    const float &qx, const float &qy, const float &qz, const float &qw,
    const float &sx, const float &sy, const float &sz)
{
    Matrix m(Vec{x, y, z}, Quat{qx, qy, qz, qw}, Vec{1, 1, 1});
    Matrix mInverse = m;
    mInverse.invert();

    std::vector<bool> erased(gridPoints * gridPoints * gridPoints, false);

    bool drew = true;
    for (int ly = 0; ly < gridPoints; ly++)
    {
        for (int lz = 0; lz < gridPoints; lz++)
        {
            for (int lx = 0; lx < gridPoints; lx++)
            {
                int ax = min.x + lx - 1;
                int ay = min.y + ly - 1;
                int az = min.z + lz - 1;

                Vec p = Vec(ax, ay, az).applyMatrix(mInverse);
                float newDistance = signedDistanceToBox(sx, sy, sz, p.x, p.y, p.z);

                int index = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                float oldDistance = cachedDamageSdf[index];

                if (newDistance <= 0.f || oldDistance >= newDistance)
                {
                    cachedDamageSdf[index] = (float)size;
                    erased[index] = true;
                    drew = true;
                }
            }
        }
    }

    if (drew)
    {
        patchFrontier(erased);
    }

    return drew;
}

void Chunk::injectDamage(float *damageBuffer)
{
    cachedDamageSdf.resize(gridPoints * gridPoints * gridPoints, MAX_HEIGHT);
    memcpy(cachedDamageSdf.data(), damageBuffer, cachedDamageSdf.size() * sizeof(float));
}