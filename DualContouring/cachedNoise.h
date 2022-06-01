#ifndef CHACHEDNOISE_H
#define CHACHEDNOISE_H

#include "main.h"
#include "vectorMath.h"
#include "../FastNoise.h"
#include "../hash.h"
#include "../util.h"
#include "../vector.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <random>
#include <unordered_map>
#include <vector>
#include <deque>

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
    CachedNoise(const vm::ivec3 chunkMin, std::vector<float> &&cachedHeightField, std::vector<float> &&cachedSdf) :
        min(chunkMin),
        size(DualContouring::chunkSize),
        gridPoints(size + 3),
        cachedHeightField(std::move(cachedHeightField)),
        cachedSdf(std::move(cachedSdf))
    {
        // nothing
    }
    CachedNoise &operator=(const CachedNoise &other) = delete;
    CachedNoise &operator=(const CachedNoise &&other) {
        size = other.size;
        gridPoints = other.gridPoints;
        min = other.min;
        cachedHeightField = std::move(other.cachedHeightField);
        cachedSdf = std::move(other.cachedSdf);
        return *this;
    }

    void init() {
        initHeightField();
        initSdf();
    }
    void initHeightField() {
        cachedHeightField.resize(gridPoints * gridPoints, -std::numeric_limits<float>::infinity());
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
        int ax = int(x) - min.x + 1;
        int az = int(z) - min.z + 1;
        int ay = int(y) - min.y + 1;
        bool drew = false;
        for (float dy = -radius; dy <= radius; dy++) {
            for (float dx = -radius; dx <= radius; dx++) {
                for (float dz = -radius; dz <= radius; dz++) {
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
    bool removeSphereDamage(const float &x, const float &y, const float &z, const float radius) {
        int ax = int(x) - min.x + 1;
        int az = int(z) - min.z + 1;
        int ay = int(y) - min.y + 1;

        std::vector<bool> erased(gridPoints * gridPoints * gridPoints, false);

        bool drew = false;
        for (float dy = -radius; dy <= radius; dy++) {
            for (float dz = -radius; dz <= radius; dz++) {
                for (float dx = -radius; dx <= radius; dx++) {
                    int lx = ax + dx;
                    int lz = az + dz;
                    int ly = ay + dy;
                    if (
                        lx >= 0 && lx < gridPoints &&
                        lz >= 0 && lz < gridPoints &&
                        ly >= 0 && ly < gridPoints
                    ) {
                        float distance = sqrt(dx * dx + dy * dy + dz * dz);
                        float newDistance = distance - radius;

                        int index = lx + lz * gridPoints + ly * gridPoints * gridPoints;
                        float oldDistance = cachedSdf[index];

                        if (newDistance < 0.f || oldDistance >= newDistance) {
                            cachedSdf[index] = (float)size;
                            erased[index] = true;
                            drew = true;
                        }
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
        Matrix m(Vec{x, y, z}, Quat{qx, qy, qz, qw}, Vec{sx, sy, sz});
        Matrix mInverse = m;
        mInverse.invert();

        bool drew = true;
        for (int y = -1; y < size + 2; y++) {
            for (int z = -1; z < size + 2; z++) {
                for (int x = -1; x < size + 2; x++) {
                    Vec p = Vec(x, y, z).applyMatrix(mInverse);
                    Vec clampedP = Vec{
                        std::min(std::max(p.x, -.5f), 0.5f),
                        std::min(std::max(p.y, -.5f), 0.5f),
                        std::min(std::max(p.z, -.5f), 0.5f)
                    };
                    Vec diff = p - clampedP;
                    float newDistance = diff.magnitude();

                    int ax = int(x) - min.x + 1;
                    int ay = int(y) - min.y + 1;
                    int az = int(z) - min.z + 1;

                    int index = ax + az * gridPoints + ay * gridPoints * gridPoints;
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
        Matrix m(Vec{x, y, z}, Quat{qx, qy, qz, qw}, Vec{sx, sy, sz});
        Matrix mInverse = m;
        mInverse.invert();

        std::vector<bool> erased(gridPoints * gridPoints * gridPoints, false);

        bool drew = true;
        for (int y = -1; y < size + 2; y++) {
            for (int z = -1; z < size + 2; z++) {
                for (int x = -1; x < size + 2; x++) {

                    Vec p = Vec(x, y, z).applyMatrix(mInverse);
                    Vec clampedP = Vec{
                        std::min(std::max(p.x, -.5f), 0.5f),
                        std::min(std::max(p.y, -.5f), 0.5f),
                        std::min(std::max(p.z, -.5f), 0.5f)
                    };
                    Vec diff = p - clampedP;
                    float newDistance = diff.magnitude();

                    int ax = int(x) - min.x + 1;
                    int ay = int(y) - min.y + 1;
                    int az = int(z) - min.z + 1;

                    int index = ax + az * gridPoints + ay * gridPoints * gridPoints;
                    float oldDistance = cachedSdf[index];

                    if (newDistance < 0.f || oldDistance >= newDistance) {
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
#endif // CHACHEDNOISE_H