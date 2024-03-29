#ifndef _INSTANCE_H_
#define _INSTANCE_H_

#include <iostream>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <ctime>
#include <string.h>
#include <memory>
// #include "chunk.h"
// #include "damage.h"
// #include "octree.h"
#include "context.h"
// #include "task.h"
// #include "result.h"
#include "../vector.h"

//

class DCInstance {
public:
    int chunkSize;
    int range;
    float fatness;

    DCInstance(int chunkSize, int range, float fatness);
    ~DCInstance();

    uint8_t *createPointCloudMesh(const std::vector<vm::vec3> &pointcloud, unsigned int *resultSize);
};

#endif // _INSTANCE_H_
