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

class Chunk
{
public:
    // private:
    int size;
    int gridPoints;
    vm::ivec3 min;
    std::vector<float> cachedHeightField;
    std::vector<float> cachedSdf;
    
    Chunk() = delete;
    Chunk(Chunk &&other) {
        size = other.size;
        gridPoints = other.gridPoints;
        min = other.min;
        cachedHeightField = std::move(other.cachedHeightField);
        cachedSdf = std::move(other.cachedSdf);
    }
    Chunk(const Chunk &other) = delete;
    Chunk(const vm::ivec3 chunkMin) :
                                       min(chunkMin),
                                       size(DualContouring::chunkSize),
                                       gridPoints(size + 4)
    {
        init();
    };
    Chunk(const vm::ivec3 chunkMin, std::vector<float> &&cachedHeightField, std::vector<float> &&cachedSdf) :
        min(chunkMin),
        size(DualContouring::chunkSize),
        gridPoints(size + 4),
        cachedHeightField(std::move(cachedHeightField)),
        cachedSdf(std::move(cachedSdf))
    {
        // nothing
    }
    Chunk &operator=(const Chunk &other) = delete;
    Chunk &operator=(const Chunk &&other) {
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

    // signed distance field function for a box at the origin
    // returns negative for points inside the box, zero at the box's surface, and positive for points outside the box
    // sx sy sz is the size of the box. the box goes from -sx/2 to sx/2, -sy/2 to sy/2, -sz/2 to sz/2
    // px py pz is the point to check
    float signedDistanceToBox(float sx, float sy, float sz, float px, float py, float pz) {
        float dx = std::abs(px) - sx / 2;
        float dy = std::abs(py) - sy / 2;
        float dz = std::abs(pz) - sz / 2;
        float d = std::max(std::max(dx, dy), dz);
        return d;
    }

    // signed distance to box.
    // returns negative for points inside the sphere, zero at the sphere's surface, and positive for points outside the sphere
    // cx, cy, cz is the center of the sphere. r is the radius. px, py, pz is the point to check
    float signedDistanceToSphere(float cx, float cy, float cz, float r, float px, float py, float pz) {
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
#endif // CHACHEDNOISE_H