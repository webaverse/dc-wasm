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

/* NoiseField::NoiseField() {}
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
} */
/* size_t NoiseField::size() const
{
    return temperature.size() + humidity.size() + ocean.size() + river.size();
} */

/* NoiseField &NoiseField::operator=(const NoiseField &&other) {
    temperature = std::move(other.temperature);
    humidity = std::move(other.humidity);
    ocean = std::move(other.ocean);
    river = std::move(other.river);
    return *this;
}
Heightfield &Heightfield::operator=(const Heightfield &&other) {
    heightField = std::move(other.heightField);
    biomesVectorField = std::move(other.biomesVectorField);
    biomesWeightsVectorField = std::move(other.biomesWeightsVectorField);
    return *this;
} */

//

int resolveGenerateFlags(int flags) {
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
    return flags;
}

//

Chunk2D::Chunk2D(const vm::ivec2 chunkMin, const int lod) :
    min(chunkMin),
    lod(lod)
{}
/* Chunk2D &Chunk2D::operator=(const Chunk2D &&other)
{
    min = other.min;
    lod = other.lod;
    cachedNoiseField = std::move(other.cachedNoiseField);
    cachedBiomesField = std::move(other.cachedBiomesField);
    cachedWaterField = std::move(other.cachedWaterField);
    cachedHeightField = std::move(other.cachedHeightField);
    return *this;
} */

/* void Chunk2D::generate(DCInstance *inst, int flags)
{
    // dependencies
    flags = resolveGenerateFlags(flags);

    // generate
    if (flags & GenerateFlags::GF_NOISE) {
        cachedNoiseField.ensure(inst, this);
    }
    if (flags & GenerateFlags::GF_BIOMES) {
        cachedBiomesField.ensure(inst, this);
    }
    if (flags & GenerateFlags::GF_HEIGHTFIELD) {
        cachedHeightField.ensure(inst, this);
    }
    if (flags & GenerateFlags::GF_WATERFIELD) {
        cachedWaterField.ensure(inst, this);
    }
} */

// noises
/* float Chunk2D::getTemperatureLocal(const int lx, const int lz) const {
    const int &size = DualContouring::chunkSize;
    int index = lx + lz * size;
    return cachedNoiseField.value.temperature[index];
}
float Chunk2D::getHumidityLocal(const int lx, const int lz) const {
    const int &size = DualContouring::chunkSize;
    int index = lx + lz * size;
    return cachedNoiseField.value.humidity[index];
} */
/* float Chunk2D::getWaterFieldLocal(const int lx, const int lz) const
{
    int index = lx + lz * size;
    return cachedWaterField[index];
} */

//

Chunk3D::Chunk3D(const vm::ivec3 chunkMin, const int lod, Chunk2D *chunk2d) :
                                                         min(chunkMin),
                                                         lod(lod),
                                                         chunk2d(chunk2d)
                                                         {}
/* Chunk3D &Chunk3D::operator=(const Chunk3D &&other)
{
    min = other.min;
    lod = other.lod;
    chunk2d = other.chunk2d;
    cachedSkylightField = std::move(other.cachedSkylightField);
    cachedAoField = std::move(other.cachedAoField);
    cachedSdf = std::move(other.cachedSdf);
    cachedWaterSdf = std::move(other.cachedWaterSdf);
    cachedDamageSdf = std::move(other.cachedDamageSdf);
    return *this;
} */

/* void Chunk3D::generate(DCInstance *inst, int flags)
{
    chunk2d->generate(inst, flags);

    // dependencies
    flags = resolveGenerateFlags(flags);

    // generate
    if (flags & GenerateFlags::GF_SDF) {
        cachedSdf.ensure(inst, this);
        cachedDamageSdf.ensure(inst, this);
    }
    if (flags & GenerateFlags::GF_LIQUIDS) {
        cachedWaterSdf.ensure(inst, this);
    }
    if (flags & GenerateFlags::GF_AOFIELD) {
        cachedSkylightField.ensure(inst, this);
        cachedAoField.ensure(inst, this);
    }
} */