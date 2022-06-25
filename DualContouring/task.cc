#include "task.h"
#include "instance.h"
#include <iostream>

//

Task::Task(std::function<bool()> &&tryLockFn, std::function<void()> &&unlockFn, std::function<void()> &&fn) :
  tryLockFn(std::move(tryLockFn)),
  unlockFn(std::move(unlockFn)),
  fn(std::move(fn))
  {}
Task::~Task() {}

bool Task::tryLock() {
  return tryLockFn();
  // return inst->tryLock(chunkPositions, lod);
}
void Task::unlock() {
  unlockFn();
  // inst->unlock(chunkPositions, lod);
}
void Task::run() {
  fn();
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
void TaskQueue::runLoop() {
  Task *task;
  while ((task = popLockTask())) {
    task->run();
    task->unlockFn();
    delete task;
  }
  std::cerr << "thread exited due to no task!" << std::endl;
  abort();
}