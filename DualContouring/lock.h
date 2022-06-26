#ifndef LOCK_H
#define LOCK_H

#include "vectorMath.h"
#include <vector>
#include <deque>
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

// template<typename PositionType>
class MultiChunkLock {
public:
    DCInstance *inst;
    // std::vector<PositionType> chunkPositions;
    // int lod;
    std::vector<std::pair<vm::ivec2, int>> chunkPositions2D;
    std::vector<std::pair<vm::ivec3, int>> chunkPositions3D;
    std::function<bool()> tryLockFn;
    std::function<void()> unlockFn;

    MultiChunkLock(DCInstance *inst) :
        inst(inst)
    {
        tryLockFn = [&]() -> bool {
            for (int i = 0; i < chunkPositions2D.size(); i++) {
                const std::pair<vm::ivec2, int> &chunkSpec = chunkPositions2D[i];
                const vm::ivec2 &chunkPosition = chunkSpec.first;
                int lod = chunkSpec.second;
                
                Mutex &chunkLock = inst->getChunkLock(chunkPosition, lod);
                if (chunkLock.try_lock()) {
                    // nothing
                } else {
                    // bail out; unlock all locks
                    for (int j = 0; j < i; j++) {
                        const std::pair<vm::ivec2, int> &chunkSpec = chunkPositions2D[i];
                        const vm::ivec2 &chunkPosition = chunkSpec.first;
                        int lod = chunkSpec.second;

                        Mutex &chunkLock = inst->getChunkLock(chunkPosition, lod);
                        chunkLock.unlock();
                    }
                    return false;
                }
            }
            for (int i = 0; i < chunkPositions3D.size(); i++) {
                const std::pair<vm::ivec3, int> &chunkSpec = chunkPositions3D[i];
                const vm::ivec3 &chunkPosition = chunkSpec.first;
                int lod = chunkSpec.second;

                Mutex &chunkLock = inst->getChunkLock(chunkPosition, lod);
                if (chunkLock.try_lock()) {
                    // nothing
                } else {
                    // bail out; unlock all locks
                    for (int j = 0; j < chunkPositions2D.size(); j++) {
                        const std::pair<vm::ivec2, int> &chunkSpec = chunkPositions2D[j];
                        const vm::ivec2 &chunkPosition = chunkSpec.first;
                        int lod = chunkSpec.second;

                        Mutex &chunkLock = inst->getChunkLock(chunkPosition, lod);
                        chunkLock.unlock();
                    }
                    for (int j = 0; j < i; j++) {
                        const std::pair<vm::ivec3, int> &chunkSpec = chunkPositions3D[j];
                        const vm::ivec3 &chunkPosition = chunkSpec.first;
                        int lod = chunkSpec.second;

                        Mutex &chunkLock = inst->getChunkLock(chunkPosition, lod);
                        chunkLock.unlock();
                    }
                    return false;
                    break;
                }
            }
            return true;
        };
        unlockFn = [&]() -> void {
            for (int j = 0; j < chunkPositions2D.size(); j++) {
                const std::pair<vm::ivec2, int> &chunkSpec = chunkPositions2D[j];
                const vm::ivec2 &chunkPosition = chunkSpec.first;
                int lod = chunkSpec.second;

                Mutex &chunkLock = inst->getChunkLock(chunkPosition, lod);
                chunkLock.unlock();
            }
            for (int j = 0; j < chunkPositions3D.size(); j++) {
                const std::pair<vm::ivec3, int> &chunkSpec = chunkPositions3D[j];
                const vm::ivec3 &chunkPosition = chunkSpec.first;
                int lod = chunkSpec.second;

                Mutex &chunkLock = inst->getChunkLock(chunkPosition, lod);
                chunkLock.unlock();
            }
        };
    }
    ~MultiChunkLock() {}

    void pushPosition(const vm::ivec2 &position, int lod) {
        chunkPositions2D.push_back(std::make_pair(position, lod));
    }
    void pushPosition(const vm::ivec3 &position, int lod) {
        chunkPositions3D.push_back(std::make_pair(position, lod));
    }
    void pushPositions(const std::vector<vm::ivec2> &positions, int lod) {
        for (const std::pair<vm::ivec2, int> &chunkSpec : chunkPositions2D) {
            const vm::ivec2 &chunkPosition = chunkSpec.first;
            int lod = chunkSpec.second;
            pushPosition(chunkPosition, lod);
        }
    }
    void pushPositions(const std::vector<vm::ivec3> &position, int lod) {
        for (const std::pair<vm::ivec3, int> &chunkSpec : chunkPositions3D) {
            const vm::ivec3 &chunkPosition = chunkSpec.first;
            int lod = chunkSpec.second;
            pushPosition(chunkPosition, lod);
        }
    }



    /* bool tryLockAll(const std::vector<PositionType> &chunkPositions, int lod) {
        bool lockedAll = true;
        for (int i = 0; i < chunkPositions.size(); i++) {
            const PositionType &chunkPosition = chunkPositions[i];
            Mutex &chunkLock = getChunkLock(chunkPosition, lod);
            if (chunkLock.try_lock()) {
                // nothing
            } else {
                // bail out; unlock all locks
                for (int j = 0; j < i; j++) {
                    const PositionType &chunkPosition = chunkPositions[j];
                    Mutex &chunkLock = getChunkLock(chunkPosition, lod);
                    chunkLock.unlock();
                }
                lockedAll = false;
                break;
            }
        }
        return lockedAll;
    }
    void unlockAll(const std::vector<PositionType> &chunkPositions, int lod) {
        for (int i = 0; i < chunkPositions.size(); i++) {
            const PositionType &chunkPosition = chunkPositions[i];
            Mutex &chunkLock = getChunkLock(chunkPosition, lod);
            chunkLock.unlock();
        }
    } */
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
};

#endif // LOCK_H
