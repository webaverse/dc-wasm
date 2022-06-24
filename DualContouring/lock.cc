#include "lock.h"
#include "instance.h"
#include <iostream>

//

ChunkLock::ChunkLock(DCInstance *inst, const vm::ivec3 &chunkPosition, int lod) :
  inst(inst),
  chunkPosition(chunkPosition),
  lod(lod)
{
  lockFn = [&]() -> bool {
    return inst->tryLock(chunkPosition, lod);
  };
  unlockFn = [&]() -> void {
    inst->unlock(chunkPosition, lod);
  };
}
ChunkLock::~ChunkLock() {}

//

MultiChunkLock::MultiChunkLock(DCInstance *inst, std::vector<vm::ivec3> &&chunkPositions, int lod) :
  inst(inst),
  chunkPositions(std::move(chunkPositions)),
  lod(lod)
{
  lockFn = [&]() -> bool {
    return inst->tryLock(chunkPositions, lod);    
  };
  unlockFn = [&]() -> void {
    inst->unlock(chunkPositions, lod);
  };
}
MultiChunkLock::~MultiChunkLock() {}

//

AutoChunkLock::AutoChunkLock(DCInstance *inst, const vm::ivec3 &chunkPosition, int lod) :
  chunkLock(inst, chunkPosition, lod)
{
  chunkLock.lockFn();
}
AutoChunkLock::~AutoChunkLock() {
  chunkLock.unlockFn();
}

AutoMultiChunkLock::AutoMultiChunkLock(DCInstance *inst, std::vector<vm::ivec3> &&chunkPositions, int lod) :
  multiChunkLock(inst, std::move(chunkPositions), lod)
{
  multiChunkLock.lockFn();
}
AutoMultiChunkLock::~AutoMultiChunkLock() {
  multiChunkLock.unlockFn();
}