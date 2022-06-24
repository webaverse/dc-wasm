#include "task.h"
#include "instance.h"
#include <iostream>

//

Task::Task(DCInstance *inst, std::vector<vm::ivec3> &&chunkPositions, int lod, int flags, std::function<void *()> &&fn) :
  inst(inst),
  chunkPositions(chunkPositions),
  lod(lod),
  flags(flags),
  fn(std::move(fn))
  {}
Task::~Task() {}

bool Task::tryLock() {
  return inst->tryLock(chunkPositions, lod, flags);
}
void Task::unlock() {
  inst->unlock(chunkPositions, lod, flags);
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