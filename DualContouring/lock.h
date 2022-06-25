#ifndef LOCK_H
#define LOCK_H

#include "vectorMath.h"
#include <vector>
#include <deque>
#include <mutex>
#include <semaphore>
#include "instance.h"

//

template<typename PositionType>
class ChunkLock {
public:
    DCInstance *inst;
    PositionType chunkPosition;
    int lod;
    std::function<bool()> tryLockFn;
    std::function<void()> unlockFn;

    ChunkLock(DCInstance *inst, const PositionType &chunkPosition, int lod) :
        inst(inst),
        chunkPosition(chunkPosition),
        lod(lod)
    {
        tryLockFn = [&]() -> bool {
            return inst->tryLock(chunkPosition, lod);
        };
        unlockFn = [&]() -> void {
            inst->unlock(chunkPosition, lod);
        };
    }
    ~ChunkLock() {}
};

template<typename PositionType>
class MultiChunkLock {
public:
    DCInstance *inst;
    std::vector<PositionType> chunkPositions;
    int lod;
    std::function<bool()> tryLockFn;
    std::function<void()> unlockFn;

    MultiChunkLock(DCInstance *inst, std::vector<PositionType> &&chunkPositions, int lod) :
        inst(inst),
        chunkPositions(std::move(chunkPositions)),
        lod(lod)
    {
        tryLockFn = [&]() -> bool {
            return inst->tryLockAll(chunkPositions, lod);    
        };
        unlockFn = [&]() -> void {
            inst->unlockAll(chunkPositions, lod);
        };
    }
    ~MultiChunkLock() {}
};

//

template<typename PositionType>
class AutoChunkLock {
public:
    ChunkLock<PositionType> chunkLock;

    AutoChunkLock(DCInstance *inst, const vm::ivec3 &chunkPosition, int lod) :
        chunkLock(inst, chunkPosition, lod)
    {
        if (!chunkLock.tryLockFn()) {
            std::cerr << "AutoChunkLock failed to lock chunk!" << std::endl;
            abort();
        }
    }
    ~AutoChunkLock() {
        chunkLock.unlockFn();
    }
};

template<typename PositionType>
class AutoMultiChunkLock {
public:
    MultiChunkLock<PositionType> multiChunkLock;

    AutoMultiChunkLock(DCInstance *inst, std::vector<vm::ivec3> &&chunkPositions, int lod) :
        multiChunkLock(inst, std::move(chunkPositions), lod)
    {
        if (!multiChunkLock.tryLockFn()) {
            std::cerr << "AutoMultiChunkLock failed to lock chunks!" << std::endl;
            abort();
        }
    }
    ~AutoMultiChunkLock() {
        multiChunkLock.unlockFn();
    }
};

typedef ChunkLock<vm::ivec2> ChunkLock2D;
typedef ChunkLock<vm::ivec3> ChunkLock3D;

typedef MultiChunkLock<vm::ivec2> MultiChunkLock2D;
typedef MultiChunkLock<vm::ivec3> MultiChunkLock3D;

#endif // LOCK_H
