#include "task.h"
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

ChunkMultiLock::ChunkMultiLock(DCInstance *inst, std::vector<vm::ivec3> &&chunkPositions, int lod) :
  inst(inst),
  chunkPositions(chunkPositions),
  lod(lod)
{
  lockFn = [&]() -> bool {
    return inst->tryLock(chunkPositions, lod);    
  };
  unlockFn = [&]() -> void {
    inst->unlock(chunkPositions, lod);
  };
}
ChunkMultiLock::~ChunkMultiLock() {}

//

Task::Task(std::function<bool()> lockFn, std::function<bool()> unlockFn, std::function<void *()> fn) :
  lockFn(lockFn),
  unlockFn(unlockFn),
  fn(fn)
  {}
Task::~Task() {}

bool Task::tryLock() {
  return lockFn();
}
void Task::unlock() {
  unlockFn();
}
void *Task::run() {
  return fn();
}

//

TaskQueue::TaskQueue(DCInstance *inst) :
  inst(inst)
  {}
TaskQueue::~TaskQueue() {}

void TaskQueue::pushTask(Task *task) {
  std::unique_lock<std::mutex> lock(taskLock);
  tasks.push_back(task);
  taskCondVar.notify_all();
}
Task *TaskQueue::popLockTask() {
  std::unique_lock<std::mutex> lock(taskLock);

  Task *task = nullptr;
  taskCondVar.wait(lock, [&]() -> bool {
    for (int i = 0; i < tasks.size(); i++) {
      if (tasks[i]->tryLock()) {
        task = tasks[i];
        tasks.erase(tasks.begin() + i);
        return true;
      }
    }
    return false;
  });
  if (task == nullptr) {
    std::cout << "failed to pop task!" << std::endl;
    abort();
  }
  return task;
}