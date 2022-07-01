#include "caches.h"
#include "main.h"
#include "util.h"

//

Caches::Caches() :
  noiseField(nullptr),
  biomesField(nullptr),
  heightField(nullptr),
  waterField(nullptr),
  skylight(nullptr),
  ao(nullptr),
  cave(nullptr),
  sdf(nullptr),
  waterSdf(nullptr)
  // cachedDamageSdf(nullptr)
{}
Caches::~Caches() {
  delete noiseField;
  delete biomesField;
  delete heightField;
  delete waterField;
  delete skylight;
  delete ao;
  delete cave;
  delete sdf;
  delete waterSdf;
  // delete cachedDamageSdf;
}

//

// 2d caches

NoiseField Caches::initNoiseField(Caches *caches, int x, int z) {
    // const int &size = chunkSize;
    // const vm::ivec2 &min = chunk->min;
    // const int &lod = chunk->lod;
    
    /* NoiseField noiseField;
    noiseField.temperature.resize(size * size);
    noiseField.humidity.resize(size * size);
    noiseField.ocean.resize(size * size);
    noiseField.river.resize(size * size);
    for (int z = 0; z < size; z++)
    {
        for (int x = 0; x < size; x++)
        { */
            // int index = x + z * size;
            // int ax = x;
            // int az = y;

            float tNoise = (float)DualContouring::noises->temperatureNoise.in2D(x, z);
            // noiseField.temperature[index] = tNoise;

            float hNoise = (float)DualContouring::noises->humidityNoise.in2D(x, z);
            // noiseField.humidity[index] = hNoise;

            float oNoise = (float)DualContouring::noises->oceanNoise.in2D(x, z);
            // noiseField.ocean[index] = oNoise;

            float rNoise = (float)DualContouring::noises->riverNoise.in2D(x, z);
            // noiseField.river[index] = rNoise;

            return NoiseField{
                tNoise,
                hNoise,
                oNoise,
                rNoise
            };
        /* }
    }

    return noiseField; */
}
uint8_t Caches::initBiomesField(Caches *caches, int x, int z) {
    auto &noiseField = caches->getNoiseField();
    // const int &size = chunkSize;
    // const auto &cachedNoiseField = chunk->cachedNoiseField;
    
    // std::vector<uint8_t> biomesField(size * size);
    /* for (int z = 0; z < size; z++)
    {
        for (int x = 0; x < size; x++)
        { */
            // int index = x + z * size;
            // unsigned char biome = cachedBiomesField.get(x, z);
            unsigned char biome = 0xFF;

            const auto &noise = noiseField.get(x, z);
            float temperatureNoise = noise.temperature;
            float humidityNoise = noise.humidity;
            float oceanNoise = noise.ocean;
            float riverNoise = noise.river;

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
            return biome;
        /* }
    } */
    // return biomesField;
}
Heightfield Caches::initHeightField(Caches *caches, int x, int z) {
    auto &biomesField = caches->getBiomesField();
    
    const int &size = chunkSize;
    // const int &gridPoints = DualContouring::gridPoints;
    // const int &lod = chunk->lod;
    // const vm::ivec2 &min = chunk->min;
    
    Heightfield heightfield;
    // heightfield.biomesVectorField.resize(gridPoints * gridPoints * 4);
    // heightfield.biomesWeightsVectorField.resize(gridPoints * gridPoints * 4);
    // heightfield.heightField.resize(gridPoints * gridPoints);
    /* for (int z = 0; z < gridPoints; z++)
    {
        for (int x = 0; x < gridPoints; x++)
        { */
            // int index2D = x + z * gridPoints;
            // int ax = x;
            // int az = z;
            
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
                    // vm::vec2 worldPosition(ax + dx, az + dz);
                    unsigned char b = biomesField.get(x + dx, z + dz);

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
                    heightfield.biomesVectorField[i] = biomes[i];
                    heightfield.biomesWeightsVectorField[i] = (float)biomeCounts[biomes[i]] / (float)numSamples;
                }
                else
                {
                    heightfield.biomesVectorField[i] = 0;
                    heightfield.biomesWeightsVectorField[i] = 0;
                }
            }

            float elevationSum = 0.f;
            vm::vec2 fWorldPosition{(float)x, (float)z};
            for (auto const &iter : biomeCounts)
            {
                elevationSum += iter.second * DualContouring::getComputedBiomeHeight(iter.first, fWorldPosition);
            }

            float elevation = elevationSum / (float)numSamples;
            heightfield.heightField = elevation;
        /* }
    } */
    return heightfield;
}
float Caches::initWaterField(Caches *caches, int x, int z) {
    auto &biomesField = caches->getBiomesField();
    const int &size = chunkSize;

    float value = 0;
    // std::unordered_map<unsigned char, unsigned int> biomeCounts(numBiomes);
    // int numSamples = 0;
    for (int dz = -size/2; dz < size/2; dz++)
    {
        for (int dx = -size/2; dx < size/2; dx++)
        {
            unsigned char b = biomesField.get(x + dx, z + dz);

            if (isWaterBiome(b)) {
                // waterField[index2D]++;
                value++;
            }
        }
    }

    return value;
}

// 3d caches

uint8_t Caches::initSkylightField(Caches *caches, int x, int y, int z) {
    // const int &gridPoints = DualContouring::gridPoints;
    // const vm::ivec3 &min = chunk->min;
    // auto &chunk2d = chunk->chunk2d;
    // auto &cachedSdf = chunk->cachedSdf;

    auto &heightField = caches->getHeightField();

    constexpr uint8_t maxSkyLightu8 = 8;
    constexpr int maxSkyLighti = (int)maxSkyLightu8;
    constexpr float maxSkyLightf = (float)maxSkyLighti;
    // std::vector<uint8_t> skylightField(gridPoints * gridPoints * gridPoints, maxSkyLight);
    /* for (int z = 0; z < gridPoints; z++)
    {
        // int lz = z + 1;

        for (int x = 0; x < gridPoints; x++)
        { */
            // int lx = x + 1;

            // int index2D = x + z * gridPoints;
            float height = heightField.get(x, z).heightField;
            
            // int topAY = min.y + gridPoints - 1;
            // int topAY = 16;
            uint8_t skylight = std::min(std::max(/*(float)topAY */y - height, 0.f), maxSkyLightf);

            /* for (int ay = y - 1; ay >= height - maxSkyLighti; ay--)
            {
                // int ly = y + 1;
                // int ay = y + dy;

                // int sdfIndex = x + z * gridPoints + y * gridPoints * gridPoints;
                if (inst->cachedSdf.get(x, ay, z) < 0.f)
                {
                    skylight--;
                }

                // int skylightIndex = x + z * gridPoints + y * gridPoints * gridPoints;
                // skylightField[skylightIndex] = skylight;
            } */
        /* }
    } */

    skylight = std::min(std::max(skylight, (uint8_t)0), maxSkyLightu8);

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
    return skylight;
}
uint8_t Caches::initAoField(Caches *caches, int x, int y, int z) {
    auto &sdf = caches->getSdf();
    
    const int &size = chunkSize;
    // const int &gridPoints = DualContouring::gridPoints;
    // auto &cachedSdf = chunk->cachedSdf;
    
    /* std::vector<uint8_t> aoField(size * size * size, 3 * 3 * 3);
    for (int y = 0; y < size; y++)
    {
        int ly = y + 1;

        for (int z = 0; z < size; z++)
        {
            int lz = z + 1;

            for (int x = 0; x < size; x++)
            {
                int lx = x + 1; */

                unsigned char numOpens = 0;
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            // int sdfIndex = (lx + dx) + (lz + dz) * gridPoints + (ly + dy) * gridPoints * gridPoints;
                            numOpens += (unsigned char)(sdf.get(x + dx, y + dy, z + dz) >= 0.f);
                        }
                    }
                }

                // int aoIndex = x + z * size + y * size * size;
                // aoField[aoIndex] = numOpens;
            /* }
        }
    } */
    return numOpens;
}
float Caches::initCaveField(Caches *caches, int x, int y, int z) {
    return DualContouring::getComputedCaveNoise(x, y, z);
}
float Caches::initSdf(Caches *caches, int x, int y, int z) {
    // const int &gridPoints = DualContouring::gridPoints;
    /* auto &chunk2d = chunk->chunk2d;
    const vm::ivec3 &min = chunk->min;
    const int &lod = chunk->lod; */

    auto &heightField = caches->getHeightField();
    auto &cave = caches->getCave();

    /* std::vector<float> sdf(gridPoints * gridPoints * gridPoints, MAX_HEIGHT);
    for (int z = 0; z < gridPoints; z++)
    {
        for (int x = 0; x < gridPoints; x++)
        { */
            // int index2D = x + z * gridPoints;
            float height = heightField.get(x, z).heightField;

            /* for (int y = 0; y < gridPoints; y++)
            { */
                // int index3D = x + z * gridPoints + y * gridPoints * gridPoints;

                // height
                float heightValue = (float)y - height;
                heightValue = std::min(
                    std::max(
                        heightValue,
                        (float)-1),
                    (float)1);

                float caveValue = cave.get(x, y, z);
                float f = heightValue + caveValue * 1.1f;
                /* f = std::min( // XXX does not fix
                    std::max(
                        f,
                        -1.f
                    ),
                    1.f
                ); */

                // result
                // sdf[index3D] = f;
            // }
        /* }
    } */
    return f;
}
/* float DCInstance::initDamageSdf(DCInstance *inst, int x, int y, int z) {
    return MAX_HEIGHT;
} */
float Caches::initWaterSdf(Caches *caches, int x, int y, int z) {
    /* EM_ASM({
        console.log('init water sdf');
    }); */
    // const int &gridPoints = DualContouring::gridPoints;
    // auto &chunk2d = chunk->chunk2d;
    // const vm::ivec3 &min = chunk->min;

    auto &waterField = caches->getWaterField();

    const float fSize = (float)chunkSize;

    // std::vector<float> waterSdf(gridPoints * gridPoints * gridPoints, MAX_HEIGHT);
    /* EM_ASM({
        console.log('water sdf set size', $0, $1, $2, $3, $4);
    }, chunk->min.x, chunk->min.y, chunk->min.z, chunk->lod, waterSdf.size()); */
    /* for (int z = 0; z < gridPoints; z++)
    {
        int az = min.z + z - 1;
        for (int x = 0; x < gridPoints; x++)
        {
            int ax = min.x + x - 1; */

            // int lx = x + 1;
            // int lz = z + 1;
            // int index2D = x + z * gridPoints;
            float waterValue = -waterField.get(x, z) / fSize;
            // float waterValue = -inst->getWater(vm::vec2(ax, az), lod) / fSize;
            // waterValue *= -1.1f;
            /* for (int y = 0; y < gridPoints; y++)
            {
                int ay = min.y + y - 1; */

                float heightValue = (float)y - waterBaseHeight;
                heightValue = std::min(
                    std::max(
                        heightValue,
                        -1.f
                    ),
                    1.f
                );
                
                float value = std::max(waterValue, heightValue);

                // int index3D = x + z * gridPoints + y * gridPoints * gridPoints;
                // waterSdf[index3D] = value;
            // }
        /* }
    } */
    return value;
}

//

ChunkCache2D<NoiseField, Caches::initNoiseField> &Caches::getNoiseField() {
  if (noiseField == nullptr) {
      noiseField = new ChunkCache2D<NoiseField, initNoiseField>(this);
  }
  return *noiseField;
}
ChunkCache2D<uint8_t, Caches::initBiomesField> &Caches::getBiomesField() {
  if (biomesField == nullptr) {
      biomesField = new ChunkCache2D<uint8_t, initBiomesField>(this);
  }
  return *biomesField;
}
ChunkCache2D<Heightfield, Caches::initHeightField> &Caches::getHeightField() {
  if (heightField == nullptr) {
      heightField = new ChunkCache2D<Heightfield, initHeightField>(this);
  }
  return *heightField;
}
ChunkCache2D<float, Caches::initWaterField> &Caches::getWaterField() {
  if (waterField == nullptr) {
      waterField = new ChunkCache2D<float, initWaterField>(this);
  }
  return *waterField;
}

//

ChunkCache3D<uint8_t, Caches::initSkylightField> &Caches::getSkylight() {
  if (skylight == nullptr) {
      skylight = new ChunkCache3D<uint8_t, initSkylightField>(this);
  }
  return *skylight;
}
ChunkCache3D<uint8_t, Caches::initAoField> &Caches::getAo() {
  if (ao == nullptr) {
      ao = new ChunkCache3D<uint8_t, initAoField>(this);
  }
  return *ao;
}
ChunkCache3D<float, Caches::initCaveField> &Caches::getCave() {
  if (cave == nullptr) {
      cave = new ChunkCache3D<float, initCaveField>(this);
  }
  return *cave;
}
ChunkCache3D<float, Caches::initSdf> &Caches::getSdf() {
  if (sdf == nullptr) {
      sdf = new ChunkCache3D<float, initSdf>(this);
  }
  return *sdf;
}
ChunkCache3D<float, Caches::initWaterSdf> &Caches::getWaterSdf() {
  if (waterSdf == nullptr) {
      waterSdf = new ChunkCache3D<float, initWaterSdf>(this);
  }
  return *waterSdf;
}

//

// biomes
/* unsigned char Caches::getCachedBiome(const int lx, const int lz) const {
    const int &size = chunkSize;
    int index = lx + lz * size;
    return cachedBiomesField.value[index];
} */
void Caches::getCachedInterpolatedBiome2D(const vm::vec2 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights) {
    // const int &gridPoints = DualContouring::gridPoints;
    
    auto &heightField = getHeightField();

    /* const float &x = worldPosition.x;
    const float &z = worldPosition.y;
    int lx = int(x) - min.x + 1;
    int lz = int(z) - min.y + 1;
    int index2D = lx + lz * gridPoints; */

    const auto &heightfield = heightField.get(worldPosition.x, worldPosition.y);
    biome.x = heightfield.biomesVectorField[0];
    biome.y = heightfield.biomesVectorField[1];
    biome.z = heightfield.biomesVectorField[2];
    biome.w = heightfield.biomesVectorField[3];

    biomeWeights.x = heightfield.biomesWeightsVectorField[0];
    biomeWeights.y = heightfield.biomesWeightsVectorField[1];
    biomeWeights.z = heightfield.biomesWeightsVectorField[2];
    biomeWeights.w = heightfield.biomesWeightsVectorField[3];
}
void Caches::getCachedInterpolatedBiome3D(const vm::vec3 &worldPosition, vm::ivec4 &biome, vm::vec4 &biomeWeights) {
    // const int &gridPoints = DualContouring::gridPoints;

    auto &heightField = getHeightField();
    auto &sdf = getSdf();

    const int x = worldPosition.x;
    const int y = worldPosition.y;
    const int z = worldPosition.z;
    /* if (std::isnan(x) || std::isnan(y) || std::isnan(z)) {
        EM_ASM({
           console.log('got nan getCachedInterpolatedBiome3D', $0, $1, $2);
        }, x, y, z);
        abort();
    } */

    getCachedInterpolatedBiome2D(vm::vec2{worldPosition.x, worldPosition.z}, biome, biomeWeights);

    // int heightfieldIndex = lx + lz * gridPoints;
    float heightValue = heightField.get(x, z).heightField;
    // int sdfIndex = lx + lz * gridPoints + ly * gridPoints * gridPoints;
    float sdfValue = sdf.get(x, y, z);

    bool neighborHeightsValid = true;
    for (int dx = -1; dx <= 1; dx += 2)
    {
        for (int dz = -1; dz <= 1; dz += 2)
        {
            int lx2 = x + dx;
            int lz2 = z + dz;
            // int neighborHeightfieldIndex = lx2 + lz2 * gridPoints;
            float heightValue = heightField.get(lx2, lz2).heightField;
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

// sdf
float Caches::getCachedInterpolatedSdf(const float x, const float y, const float z) {
    auto &sdf = getSdf();
    return trilinear<decltype(sdf), float>(
        vm::vec3{x, y, z},
        sdf
    );
}
float Caches::getCachedWaterInterpolatedSdf(const float x, const float y, const float z) {
    // const int &gridPoints = DualContouring::gridPoints;

    auto &waterSdf = getWaterSdf();

    // const float localX = x + 1;
    // const float localY = y + 1;
    // const float localZ = z + 1;
    return trilinear<decltype(waterSdf), float>(
        vm::vec3{x, y, z},
        waterSdf
    );
}
float Caches::getCachedDamageInterpolatedSdf(const float x, const float y, const float z) {
    // const int &gridPoints = DualContouring::gridPoints;

    // const float localX = x + 1;
    // const float localY = y + 1;
    // const float localZ = z + 1;
    return MAX_HEIGHT;
    /* return trilinear<decltype(cachedDamageSdf), float>(
        vm::vec3{x, y, z},
        cachedDamageSdf
    ); */
}