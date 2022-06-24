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

class MultiChunkLock {
public:
    DCInstance *inst;
    std::vector<vm::ivec3> chunkPositions;
    int lod;
    std::function<bool()> lockFn;
    std::function<void()> unlockFn;

    MultiChunkLock(DCInstance *inst, std::vector<vm::ivec3> &&chunkPositions, int lod);
    ~MultiChunkLock();
};

//

class AutoChunkLock {
public:
    ChunkLock chunkLock;

    AutoChunkLock(DCInstance *inst, const vm::ivec3 &chunkPosition, int lod);
    ~AutoChunkLock();
};

class AutoMultiChunkLock {
public:
    MultiChunkLock multiChunkLock;

    AutoMultiChunkLock(DCInstance *inst, std::vector<vm::ivec3> &&chunkPositions, int lod);
    ~AutoMultiChunkLock();
};

#endif // LOCK_H
