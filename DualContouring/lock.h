#ifndef LOCK_H
#define LOCK_H

#include "vectorMath.h"
#include "promise.h"
#include <vector>
#include <deque>

//

class DCInstance;

//

class MultiChunkLock {
public:
    DCInstance *inst;
    std::vector<std::pair<vm::ivec2, int>> chunkPositions2D;
    std::vector<std::pair<vm::ivec3, int>> chunkPositions3D;
    std::vector<Promise *> promises;

    MultiChunkLock() = delete;
    MultiChunkLock(DCInstance *inst);
    ~MultiChunkLock();

    //

    bool tryLockFn();
    void unlockFn();

    //

    void pushPosition(const vm::ivec2 &position, int lod);
    void pushPosition(const vm::ivec3 &position, int lod);
    void pushPositions(const std::vector<vm::ivec2> &positions, int lod);
    void pushPositions(const std::vector<vm::ivec3> &positions, int lod);
    
    void pushPromise(Promise *promise);
    void pushPromises(const std::vector<Promise *> &promises);

    //

    bool lock2D();
    bool lock3D();

    bool unlock2D();
    bool unlock3D();

    bool lockPromises();
};

//

/* template<typename PositionType>
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

class AutoMultiChunkLock {
public:
    MultiChunkLock multiChunkLock;

    AutoMultiChunkLock(DCInstance *inst) :
        multiChunkLock(inst)
    {
        if (!multiChunkLock.tryLockFn()) {
            std::cerr << "AutoMultiChunkLock failed to lock chunks!" << std::endl;
            abort();
        }
    }
    ~AutoMultiChunkLock() {
        multiChunkLock.unlockFn();
    }

    void pushPosition(const vm::ivec2 &position, int lod) {
        multiChunkLock.pushPosition(position, lod);
    }
    void pushPosition(const vm::ivec3 &position, int lod) {
        multiChunkLock.pushPosition(position, lod);
    }
    void pushPositions(const std::vector<vm::ivec2> &positions, int lod) {
        multiChunkLock.pushPositions(positions, lod);
    }
    void pushPositions(const std::vector<vm::ivec3> &position, int lod) {
        multiChunkLock.pushPositions(position, lod);
    }
}; */

#endif // LOCK_H
