#include "task.h"
#include "instance.h"
#include <iostream>
#include <emscripten.h>

//

Task::Task(MultiChunkLock &&multiChunkLock, std::function<void()> &&fn) :
  multiChunkLock(std::move(multiChunkLock)),
  fn(std::move(fn))
  {}
Task::~Task() {}

bool Task::tryLock() {
  /* EM_ASM({
    console.log('task try lock');
  }); */
  return multiChunkLock.tryLockFn();
}
void Task::unlock() {
  multiChunkLock.unlockFn();
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
    // numTasks++;
    /* EM_ASM({
      console.log('push task end', $0);
    }, tasks.size()); */
  }
  // taskSemaphore.signal();
  flushTasks();
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

    if (lockedTasks.size() == 0) {
      abort();
    }

    // XXX lock here; perhaps have a queue of only requirement fulfilled tasks

    task = lockedTasks.front();
    lockedTasks.pop_front();
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
  {
    Task *task = nullptr;
    while ((task = popLockTask())) {
      task->run();
      task->unlock();
      delete task;
      task = nullptr;

      flushTasks();
    }
  }
  /* EM_ASM(
    console.log('thread exited due to no task!');
  ); */
  abort();
}
void TaskQueue::flushTasks() {
  // {
    std::unique_lock<Mutex> lock(taskMutex);

    bool lockedTask = true;
    while (lockedTask) {
      lockedTask = false;
      for (int i = 0; i < tasks.size(); i++) {
        if (tasks[i]->tryLock()) {
          lockedTasks.push_back(tasks[i]);
          tasks.erase(tasks.begin() + i);

          taskSemaphore.signal();

          lockedTask = true;
          break;
        }
      }
    }
  // }
}