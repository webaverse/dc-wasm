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
    DCInstance();
    ~DCInstance();

    uint8_t *createPointCloudMesh(const std::vector<vm::vec3> &pointcloud);
};

#endif // _INSTANCE_H_
