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
}
void Task::unlock() {
  unlockFn();
}
void Task::run() {
  fn();
}

//

TaskQueue::TaskQueue() {}
TaskQueue::~TaskQueue() {}

void TaskQueue::pushTask(Task *task) {
  {
    std::unique_lock<Mutex> lock(taskMutex);
    /* EM_ASM({
      console.log('push task start', SharedArrayBuffer, $0, $1);
    }, tasks.size(), (void *)this); */
    tasks.push_back(task);
    numTasks++;
    /* EM_ASM({
      console.log('push task end', $0);
    }, tasks.size()); */
  }
  taskSemaphore.signal();
}
Task *TaskQueue::popLockTask() {
  /* EM_ASM(
    console.log('pop lock task 1');
  ); */
  taskSemaphore.wait();

  /* EM_ASM(
    console.log('pop lock sema waited');
  ); */

  Task *task;
  {
    std::unique_lock<Mutex> lock(taskMutex);
    // lock.lock();

    task = tasks.front();
    tasks.pop_front();
  }
  if (task == nullptr) {
    /* EM_ASM(
      console.log('failed to pop task!');
    ); */
    abort();
  }
  return task;
}
void TaskQueue::runLoop() {
  /* EM_ASM(
    console.log('run loop');
  ); */
  Task *task;
  while ((task = popLockTask())) {
    task->run();
    task->unlockFn();
    delete task;
  }
  /* EM_ASM(
    console.log('thread exited due to no task!');
  ); */
  abort();
}