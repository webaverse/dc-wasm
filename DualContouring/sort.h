#ifndef SORT_H
#define SORT_H

// #include "vectorMath.h"
// #include "sync.h"
// #include "lock.h"
// #include "vector.h"
#include <array>
#include <vector>
#include <deque>
// #include <semaphore>
// #include <atomic>
#include <emscripten.h>

//

template<typename PointerType>
class DistanceSpec {
public:
    PointerType node;
    int priority;
    float distance;
    bool intersectsFrustum;

    // sort by priority (low to high), then by intersectsFrustum (true to false), then by distance (low to high)
    bool operator<(const DistanceSpec<PointerType> &other) const {
        int priorityDiff = priority - other.priority;
        if (priorityDiff != 0) {
            return priorityDiff < 0;
        } else {
            int intersectsFrustumDiff = (+other.intersectsFrustum) - (+intersectsFrustum);
            if (intersectsFrustumDiff != 0) {
                return intersectsFrustumDiff < 0;
            } else {
                return distance < other.distance;
            }
        }
    }
};

template<typename PointerType>
DistanceSpec<PointerType> getDistanceSpec(PointerType node, const vm::vec3 &worldPosition, const Frustum &frustum) {
    const int &priority = node->getPriority();
    const Sphere &sphere = node->getSphere();

    const bool hasRadius = (bool)sphere;
    const bool intersectsFrustum = hasRadius ? frustum.intersectsSphere(sphere) : true;
    vm::vec3 center{
        sphere.center.x,
        sphere.center.y,
        sphere.center.z
    };
    float distance = vm::length(center - worldPosition);

    DistanceSpec<PointerType> distanceSpec{
        node,
        priority,
        distance,
        intersectsFrustum
    };
    return distanceSpec;

    /* Sphere sphere(
        Vec{
            task->worldPosition.x,
            task->worldPosition.y,
            task->worldPosition.z
        },
        (float)std::sqrt(
            task->halfSize.x * task->halfSize.x +
            task->halfSize.y * task->halfSize.y +
            task->halfSize.z * task->halfSize.z
        )
    );
    if (!frustum.intersectsSphere(sphere)) {
        distance += frustumCullDistancePenalty;
    }
    distance += task->priority * priorityDistancePenalty;
    return distance; */
}

template<typename PointerType, typename ArrayType>
void sort(ArrayType &values, const vm::vec3 &worldPosition, const Frustum &frustum) {
    // map values to DistanceSpec
    std::vector<DistanceSpec<PointerType>> distanceSpecs(values.size());
    for (size_t i = 0; i < values.size(); i++) {
        distanceSpecs[i] = getDistanceSpec(values[i], worldPosition, frustum);
    }
    // sort based on DistanceSpec
    std::sort(distanceSpecs.begin(), distanceSpecs.end());
    // set output values
    for (size_t i = 0; i < values.size(); i++) {
        values[i] = distanceSpecs[i].node;
    }
}

#endif // SORT_H