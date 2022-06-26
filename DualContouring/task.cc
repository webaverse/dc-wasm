#include "task.h"
#include "instance.h"
#include <iostream>
#include <emscripten.h>

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

TaskQueue::TaskQueue() {}
TaskQueue::~TaskQueue() {}

void TaskQueue::pushTask(Task *task) {
  EM_ASM({
    console.log('push task start', SharedArrayBuffer, $0, $1);
  }, tasks.size(), (void *)this);
  {
    std::unique_lock<std::mutex> lock(taskLock);
    tasks.push_back(task);
    numTasks++;
  }
  EM_ASM({
    console.log('push task end', $0);
  }, tasks.size());
  taskCondVar.notify_all();
}
Task *TaskQueue::tryLockTask() {
  /* EM_ASM({
    console.log('check lock task', $0);
  }, tasks.size()); */
  for (int i = 0; i < numTasks; i++) {
    EM_ASM({
      console.log('check lock task ok start A', $0, $1, $2, $3);
    }, i, tasks.size(), (size_t)numTasks, (void *)this);
    if (tasks[i]->tryLock()) {
      EM_ASM({
        console.log('check lock task ok start B', $0, $1, $2, $3);
      }, i, tasks.size(), (size_t)numTasks, (void *)this);
      Task *task = tasks[i];
      tasks.erase(tasks.begin() + i);
      numTasks--;
      EM_ASM({
        console.log('check lock task ok end', $0, $1, $2);
      }, i, tasks.size(), (size_t)numTasks);
      return task;
    }
  }
  return nullptr;
}
Task *TaskQueue::popLockTask() {
  /* EM_ASM(
    console.log('pop lock task 1');
  ); */
  std::unique_lock<std::mutex> lock(taskLock);
  // lock.lock();
  /* EM_ASM(
    console.log('pop lock task 2');
  ); */

  Task *task = nullptr;
  for (;;) {
    taskCondVar.wait(lock);
    if ((task = tryLockTask())) {
      break;
    }
  }
  if (task == nullptr) {
    EM_ASM(
      console.log('failed to pop task!');
    );
    abort();
  }
  return task;
}
void TaskQueue::runLoop() {
  EM_ASM(
    console.log('run loop');
  );
  Task *task;
  while ((task = popLockTask())) {
    task->run();
    task->unlockFn();
    delete task;
  }
  EM_ASM(
    console.log('thread exited due to no task!');
  );
  abort();
}