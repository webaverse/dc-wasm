#include "main.h"
#include "density.h"
#include "./biomes.h"

float sphere(const vm::vec3 &worldPosition, const vm::vec3 &origin, float radius)
{
	return vm::length(worldPosition - origin) - radius;
}

float cuboid(const vm::vec3 &worldPosition, const vm::vec3 &origin, const vm::vec3 &halfDimensions)
{
	const vm::vec3 &local_pos = worldPosition - origin;
	const vm::vec3 &pos = local_pos;

	const vm::vec3 &d = vm::abs(pos) - halfDimensions;
	const float m = std::max(d.x, std::max(d.y, d.z));
	return std::min(m, vm::length(vm::max(d, 0.0)));
}

// float falloffMap(const vm::vec2 &position)
// {
// 	return (1 / ((position.x * position.y) * (1 - (position.x / 1)) * (1 - position.y)));
// }

// template <typename T>
// float FBM(const T &position)
// {
// 	const int octaves = 4;
// 	const float frequency = 0.54;
// 	const float lacunarity = 2.24;
// 	const float persistence = 0.68;
// 	const float SCALE = 1.f / 128.f;
// 	T p = position * SCALE;
// 	float noise = 0.f;

// 	float amplitude = 1.f;
// 	p = p * frequency;

// 	for (int i = 0; i < octaves; i++)
// 	{
// 		noise += glm::simplex(glm::vec2(p.x, p.y)) * amplitude;
// 		p = p * lacunarity;
// 		amplitude *= persistence;
// 	}

// 	// move into [0, 1] range
// 	return 0.5f + (0.5f * noise);
// }

// float warpedNoise(const vm::vec3 &position)
// {
// 	const float q = FBM(vm::vec3(position.x + 5.3f, position.y + 0.8, position.z)) * 80.0;
// 	return FBM(vm::vec3(position.x + q, position.y + q, position.z + q));
// }

// float temperatureNoise(const vm::vec3 &position)
// {
// 	const vm::vec3 p = position / 5.f;
// 	float a = warpedNoise(p - vm::vec3(100.0));
// 	return a;
// 	// return a - falloffMap(vm::vec2(position.x - 100, position.y + 100));
// }
// float humidityNoise(const vm::vec3 &position)
// {
// 	const vm::vec3 p = position / 5.f;
// 	float a = warpedNoise(p + vm::vec3(100.0, 20.0, 40.0));
// 	return a;
// 	// return a - falloffMap(vm::vec2(position.x + 100, position.y - 100));
// }

/* unsigned char getBiome(const vm::ivec2 &worldPosition, Chunk &chunkNoise)
{
	unsigned char biome = 0xFF;
	float tNoise = chunkNoise.getTemperature(worldPosition.x, worldPosition.y);
	float hNoise = chunkNoise.getHumidity(worldPosition.x, worldPosition.y);
	const int t = (int)std::floor(tNoise * 16.0);
	const int h = (int)std::floor(hValue * 16.0);
	biome = (unsigned char)BIOMES_TEMPERATURE_HUMIDITY[t + 16 * h];

	return biome;
} */

template <typename BoxType>
inline float clampPointToRange(float minDistance, const vm::vec3 &position, const BoxType &range) {
  const auto &rangeMin = range.min;
	const auto &rangeMax = range.max;

	auto w = rangeMax.x - rangeMin.x;
	auto h = rangeMax.y - rangeMin.y;
	auto d = rangeMax.z - rangeMin.z;

	const float cube = cuboid(
		position,
		(vm::vec3(rangeMin.x, rangeMin.y, rangeMin.z) + vm::vec3(rangeMax.x, rangeMax.y, rangeMax.z)) / 2.f,
		vm::vec3(w, h, d) / 2.f
	);
	minDistance = std::max(minDistance, cube);
	return minDistance;
}

// negative density means inside the chunk, positive density means outside the chunk
// when the clipper is enabled, we contain the SDF into a AABB (sdf increases with distance away from the range AABB)
float terrainDensityFn(const vm::vec3 &position, DCInstance *inst, Chunk3D &chunk)
{
	const float terrain = chunk.getCachedInterpolatedSdf(position.x, position.y, position.z);
	const float damage = chunk.getCachedDamageInterpolatedSdf(position.x, position.y, position.z);
	const float addition = chunk.getCachedAdditionInterpolatedSdf(position.x, position.y, position.z);

	// float minDistance = std::min(terrain, addition);
	float minDistance = std::min(terrain, -addition);

	if (inst->clipRange) { // range clipper enabled
	  minDistance = clampPointToRange(minDistance, position, *inst->clipRange);
	}

	// const float cube = cuboid(position, vm::vec3(-4., 10.f, -4.f), vm::vec3(12.f));
	// const float orb = sphere(position, vm::vec3(15.f, 2.5f, 1.f), 16.f);

	// return orb;
	// return cube;
	// return terrain;
	// return std::max(-cube, terrain);
	return minDistance;
}

float liquidDensityFn(const vm::vec3 &position, DCInstance *inst, Chunk3D &chunk)
{
	const float water = chunk.getCachedWaterInterpolatedSdf(position.x, position.y, position.z);

	float minDistance = water;
	if (inst->clipRange) { // range clipper enabled
    minDistance = clampPointToRange(minDistance, position, *inst->clipRange);
	}
	return minDistance;
}