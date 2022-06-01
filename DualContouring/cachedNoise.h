#ifndef CHACHEDNOISE_H
#define CHACHEDNOISE_H

#include "main.h"
#include "vectorMath.h"
#include "../FastNoise.h"
#include "../hash.h"
#include "../util.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>
#include <vector>

constexpr float MAX_HEIGHT = 20.f;

class CachedNoise
{
public:
    // private:
    int size;
    int gridPoints;
    vm::ivec3 min;
    std::vector<float> cachedHeightField;
    std::vector<float> cachedSdf;
    
    CachedNoise() = delete;
    CachedNoise(CachedNoise &&other) {
        size = other.size;
        gridPoints = other.gridPoints;
        min = other.min;
        cachedHeightField = std::move(other.cachedHeightField);
        cachedSdf = std::move(other.cachedSdf);
    }
    CachedNoise(const CachedNoise &other) = delete;
    CachedNoise(const vm::ivec3 chunkMin) :
                                       min(chunkMin),
                                       size(DualContouring::chunkSize),
                                       gridPoints(size + 3)
    {
        init();
    };
    CachedNoise &operator=(const CachedNoise &other) = delete;
    CachedNoise &operator=(const CachedNoise &&other) {
        size = other.size;
        gridPoints = other.gridPoints;
        min = other.min;
        cachedHeightField = std::move(other.cachedHeightField);
        cachedSdf = std::move(other.cachedSdf);
        return *this;
    }

    void init()
    {
        cachedHeightField.resize(gridPoints * gridPoints, -std::numeric_limits<float>::infinity());
        cachedSdf.resize(gridPoints * gridPoints * gridPoints, MAX_HEIGHT);

        // height field
        for (int dz = 0; dz < gridPoints; dz++)
        {
            for (int dx = 0; dx < gridPoints; dx++)
            {
                int index2D = dx + dz * gridPoints;
                int ax = dx + min.x - 1;
                int az = dz + min.z - 1;
                float heightNoise = (float)DualContouring::fastNoise->GetSimplexFractal(ax, az);
                float height = MAX_HEIGHT * heightNoise;
                cachedHeightField[index2D] = height;
            }
        }

        // noise field
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

                    cachedSdf[index3D] = std::min(
                        std::max(
                            (float)ay - height,
                            (float)(-DualContouring::chunkSize)
                        ),
                        (float)DualContouring::chunkSize
                    );
                }
            }
        }
    }

    float interpolateHeight1D(const float &x, const float &z)
    {
        const int xf = std::floor(x);
        const int xc = std::ceil(x);
        const int indexF = xf + z * gridPoints;
        const int indexC = xc + z * gridPoints;
        const float dx = x - xf;
        return lerp(cachedHeightField.at(indexF), cachedHeightField.at(indexC), dx);
    }
    float interpolateHeight2D(const float &x, const float &z)
    {
        const int zf = std::floor(z);
        const int zc = std::ceil(z);
        const float dz = z - zf;
        return lerp(interpolateHeight1D(x, zf), interpolateHeight1D(x, zc), dz);
    } 

    float getRawHeight(const int &x, const int &z)
    {
        const int localX = x - min.x + 1;
        const int localZ = z - min.z + 1;
        const int index = localX + localZ * gridPoints;
        return (cachedHeightField.at(index) + 1.f) / 2.f;
    }
    float getInterpolatedHeight(const float &x, const float &z)
    {
        const float localX = x - min.x + 1;
        const float localZ = z - min.z + 1;
        return interpolateHeight2D(localX, localZ);
    }

    float getInterpolatedSdf(const float &x, const float &y, const float &z) {
        const float localX = x - min.x + 1;
        const float localY = y - min.y + 1;
        const float localZ = z - min.z + 1;
        return trilinear<float>(
            vm::vec3(localX, localY, localZ),
            cachedSdf,
            gridPoints
        );
    }

    bool addDamage(const float &x, const float &y, const float &z, const float radius) {
        int ax = int(x) - min.x + 1;
        int az = int(z) - min.z + 1;
        int ay = int(y) - min.y + 1;
        bool drew = false;
        for (float dx = -radius; dx <= radius; dx++) {
            for (float dz = -radius; dz <= radius; dz++) {
                for (float dy = -radius; dy <= radius; dy++) {
                    int lx = ax + dx;
                    int lz = az + dz;
                    int ly = ay + dy;
                    if (
                        lx >= 0 && lx < gridPoints &&
                        lz >= 0 && lz < gridPoints &&
                        ly >= 0 && ly < gridPoints
                    ) {
                        float distance = sqrt(dx * dx + dy * dy + dz * dz);
                        float distanceFactor = distance - radius;
                        int index = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                        
                        cachedSdf[index] += std::min(cachedSdf[index], distanceFactor);
                        drew = true;
                    }
                }
            }
        }
        return drew;
    }
    bool removeDamage(const float &x, const float &y, const float &z, const float radius) {
      return false;
    }
};
#endif // CHACHEDNOISE_H