#include "lock.h"
#include "instance.h"
#include <iostream>
#include <emscripten.h>

//

// Mutex testMutex;

MultiChunkLock::MultiChunkLock(DCInstance *inst) :
    inst(inst)
{}
MultiChunkLock::~MultiChunkLock() {}
MultiChunkLock::MultiChunkLock(MultiChunkLock &&other) :
    inst(other.inst),
    chunkPositions2D(std::move(other.chunkPositions2D)),
    chunkPositions3D(std::move(other.chunkPositions3D))
{}
bool MultiChunkLock::tryLockFn() {
  // return testMutex.try_lock();
  /* if (chunkPositions2D.size() > 0 && chunkPositions3D.size() > 0) {
    EM_ASM({
      console.log('try lock fn', $0, $1);
    }, chunkPositions2D.size(), chunkPositions3D.size());
  } */
  for (int i = 0; i < chunkPositions2D.size(); i++) {
      const std::pair<vm::ivec2, int> &chunkSpec = chunkPositions2D[i];
      const vm::ivec2 &chunkPosition = chunkSpec.first;
      int lod = chunkSpec.second;
      
      /* EM_ASM({
        console.log('try lock 2d', $0, $1, $2);
      }, chunkPosition.x, chunkPosition.y, lod); */

      Mutex *chunkLock = inst->getChunkLock(chunkPosition, lod);
      if (chunkLock->try_lock()) {
          // nothing
      } else {
          /* EM_ASM({
            console.log('bail 1', $0, $1);
          }, chunkPositions2D.size(), chunkPositions3D.size()); */
          // bail out; unlock all locks
          for (int j = 0; j < i; j++) {
              const std::pair<vm::ivec2, int> &chunkSpec = chunkPositions2D[j];
              const vm::ivec2 &chunkPosition = chunkSpec.first;
              int lod = chunkSpec.second;

              Mutex *chunkLock = inst->getChunkLock(chunkPosition, lod);
              chunkLock->unlock();
          }
          return false;
      }
  }
  for (int i = 0; i < chunkPositions3D.size(); i++) {
      const std::pair<vm::ivec3, int> &chunkSpec = chunkPositions3D[i];
      const vm::ivec3 &chunkPosition = chunkSpec.first;
      int lod = chunkSpec.second;

      /* EM_ASM({
        console.log('try lock 3d', $0, $1, $2, $3);
      }, chunkPosition.x, chunkPosition.y, chunkPosition.z, lod); */

      Mutex *chunkLock = inst->getChunkLock(chunkPosition, lod);
      if (chunkLock->try_lock()) {
          // nothing
      } else {
          /* EM_ASM({
            console.log('bail 2', $0, $1);
          }, chunkPositions2D.size(), chunkPositions3D.size()); */
          // bail out; unlock all locks
          for (int j = 0; j < chunkPositions2D.size(); j++) {
              const std::pair<vm::ivec2, int> &chunkSpec = chunkPositions2D[j];
              const vm::ivec2 &chunkPosition = chunkSpec.first;
              int lod = chunkSpec.second;

              Mutex *chunkLock = inst->getChunkLock(chunkPosition, lod);
              chunkLock->unlock();
          }
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
};
void MultiChunkLock::unlockFn() {
  // return testMutex.unlock();
  /* if (chunkPositions2D.size() > 0 && chunkPositions3D.size() > 0) {
    EM_ASM({
      console.log('unlock fn', $0, $1);
    }, chunkPositions2D.size(), chunkPositions3D.size());
  } */
  for (int j = 0; j < chunkPositions2D.size(); j++) {
      const std::pair<vm::ivec2, int> &chunkSpec = chunkPositions2D[j];
      const vm::ivec2 &chunkPosition = chunkSpec.first;
      int lod = chunkSpec.second;

      Mutex *chunkLock = inst->getChunkLock(chunkPosition, lod);
      chunkLock->unlock();
  }
  for (int j = 0; j < chunkPositions3D.size(); j++) {
      const std::pair<vm::ivec3, int> &chunkSpec = chunkPositions3D[j];
      const vm::ivec3 &chunkPosition = chunkSpec.first;
      int lod = chunkSpec.second;

      Mutex *chunkLock = inst->getChunkLock(chunkPosition, lod);
      chunkLock->unlock();
  }
}
void MultiChunkLock::pushPosition(const vm::ivec2 &position, int lod) {
  chunkPositions2D.push_back(std::make_pair(position, lod));
}
void MultiChunkLock::pushPosition(const vm::ivec3 &position, int lod) {
  chunkPositions3D.push_back(std::make_pair(position, lod));
}
void MultiChunkLock::pushPositions(const std::vector<vm::ivec2> &positions, int lod) {
  for (const vm::ivec2 &chunkPosition : positions) {
      pushPosition(chunkPosition, lod);
  }
}
void MultiChunkLock::pushPositions(const std::vector<vm::ivec3> &positions, int lod) {
  for (const vm::ivec3 &chunkPosition : positions) {
      pushPosition(chunkPosition, lod);
  }
}