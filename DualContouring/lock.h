#ifndef LOCK_H
#define LOCK_H

#include "vectorMath.h"
#include <vector>
#include <deque>
#include <mutex>
#include <semaphore>

//

class DCInstance;

//

class ChunkLock {
public:
    DCInstance *inst;
    vm::ivec3 chunkPosition;
    int lod;
    std::function<bool()> lockFn;
    std::function<void()> unlockFn;

    ChunkLock(DCInstance *inst, const vm::ivec3 &chunkPosition, int lod);
    ~ChunkLock();
};

//

class ChunkMultiLock {
public:
    DCInstance *inst;
    std::vector<vm::ivec3> chunkPositions;
    int lod;
    std::function<bool()> lockFn;
    std::function<void()> unlockFn;

    ChunkMultiLock(DCInstance *inst, std::vector<vm::ivec3> &&chunkPositions, int lod);
    ~ChunkMultiLock();
};

#endif // LOCK_H
