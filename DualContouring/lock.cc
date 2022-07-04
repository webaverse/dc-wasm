#include "lock.h"
#include "instance.h"
#include <iostream>
#include <emscripten.h>

//

// Mutex testMutex;

MultiChunkLock::MultiChunkLock(DCInstance *inst) :
  inst(inst)
{}
MultiChunkLock::~MultiChunkLock() {
  for (auto promise : promises) {
    delete promise;
  }
}
/* MultiChunkLock::MultiChunkLock(MultiChunkLock &&other) :
  inst(other.inst),
  chunkPositions2D(std::move(other.chunkPositions2D)),
  chunkPositions3D(std::move(other.chunkPositions3D))
{} */
bool MultiChunkLock::tryLockFn() {
  return lockPromises() &&
    lock2D() &&
    (lock3D() || unlock2D());
}
void MultiChunkLock::unlockFn() {
  unlock2D();
  unlock3D();
}

//

bool MultiChunkLock::lock2D() {
  for (int i = 0; i < chunkPositions2D.size(); i++) {
    const std::tuple<vm::ivec2, int, int> &chunkSpec = chunkPositions2D[i];
    const vm::ivec2 &chunkPosition = std::get<0>(chunkSpec);
    int lod = std::get<1>(chunkSpec);
    int flags = std::get<2>(chunkSpec);

    Mutex *chunkLock = inst->getChunkLock(chunkPosition, lod, flags);
    if (chunkLock->try_lock()) {
        // nothing
    } else {
        // bail out; unlock all locks
        for (int j = 0; j < i; j++) {
            const std::tuple<vm::ivec2, int, int> &chunkSpec = chunkPositions2D[j];
            const vm::ivec2 &chunkPosition = std::get<0>(chunkSpec);
            int lod = std::get<1>(chunkSpec);
            int flags = std::get<2>(chunkSpec);

            Mutex *chunkLock = inst->getChunkLock(chunkPosition, lod, flags);
            chunkLock->unlock();
        }
        return false;
    }
  }
  return true;
}
bool MultiChunkLock::lock3D() {
  for (int i = 0; i < chunkPositions3D.size(); i++) {
    const std::pair<vm::ivec3, int> &chunkSpec = chunkPositions3D[i];
    const vm::ivec3 &chunkPosition = chunkSpec.first;
    int lod = chunkSpec.second;

    Mutex *chunkLock = inst->getChunkLock(chunkPosition, lod);
    if (chunkLock->try_lock()) {
      // nothing
    } else {
      // bail out; unlock all locks
      for (int j = 0; j < i; j++) {
        const std::pair<vm::ivec3, int> &chunkSpec = chunkPositions3D[j];
        const vm::ivec3 &chunkPosition = chunkSpec.first;
        int lod = chunkSpec.second;

        Mutex *chunkLock = inst->getChunkLock(chunkPosition, lod);
        chunkLock->unlock();
      }
      return false;
    }
  }
  return true;
}
bool MultiChunkLock::lockPromises() {
  abort(); // XXX API outdated
  /* for (int i = 0; i < promises.size(); i++) {
    Promise *promise = promises[i];
    if (!promise->test()) {
      return false;
    }
  }
  return true; */
}

//

bool MultiChunkLock::unlock2D() {
  for (int j = 0; j < chunkPositions2D.size(); j++) {
    const std::tuple<vm::ivec2, int, int> &chunkSpec = chunkPositions2D[j];
    const vm::ivec2 &chunkPosition = std::get<0>(chunkSpec);
    int lod = std::get<1>(chunkSpec);
    int flags = std::get<2>(chunkSpec);

    Mutex *chunkLock = inst->getChunkLock(chunkPosition, lod, flags);
    chunkLock->unlock();
  }
  return false;
}
bool MultiChunkLock::unlock3D() {
  for (int j = 0; j < chunkPositions3D.size(); j++) {
    const std::pair<vm::ivec3, int> &chunkSpec = chunkPositions3D[j];
    const vm::ivec3 &chunkPosition = chunkSpec.first;
    int lod = chunkSpec.second;

    Mutex *chunkLock = inst->getChunkLock(chunkPosition, lod);
    chunkLock->unlock();
  }
  return false;
}

//

void MultiChunkLock::pushPosition(const vm::ivec2 &position, int lod, int flags) {
  chunkPositions2D.push_back(std::make_tuple(position, lod, flags));
}
void MultiChunkLock::pushPosition(const vm::ivec3 &position, int lod) {
  chunkPositions3D.push_back(std::make_pair(position, lod));
}
/* void MultiChunkLock::pushPositions(const std::vector<vm::ivec2> &positions, int lod) {
  for (const vm::ivec2 &position : positions) {
    pushPosition(position, lod);
  }
}
void MultiChunkLock::pushPositions(const std::vector<vm::ivec3> &positions, int lod) {
  for (const vm::ivec3 &position : positions) {
    pushPosition(position, lod);
  }
} */

//

void MultiChunkLock::pushPromise(Promise *promise) {
  this->promises.push_back(promise);
}
void MultiChunkLock::pushPromises(const std::vector<Promise *> &promises) {
  for (Promise *promise : promises) {
    pushPromise(promise);
  }
}