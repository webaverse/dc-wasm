#include "task.h"
#include "instance.h"
#include <iostream>
#include <emscripten/atomic.h>

//

Task::Task(std::function<void()> fn) :
  fn(fn)
{}
Task::~Task() {}

void Task::run() {
  fn();
}

//

TaskQueue::TaskQueue() {}
TaskQueue::~TaskQueue() {
  EM_ASM({
    console.log('task queue destructor');
  });
  abort();
}

void TaskQueue::pushTask(Task *task) {
  {
    std::unique_lock<Mutex> lock(taskMutex);
    /* EM_ASM({
      console.log('push task start', SharedArrayBuffer, $0, $1);
    }, tasks.size(), (void *)this); */
    tasks.push_back(task);

    EM_ASM({
      console.log('push task', $0);
    }, tasks.size());

    // numTasks++;
    /* EM_ASM({
      console.log('push task end', $0);
    }, tasks.size()); */
  }
  taskSemaphore.signal();
  // flushTasks();
}
std::atomic<int> numActiveThreads(8);
Task *TaskQueue::popLockTask() {
  /* EM_ASM(
    console.log('pop lock task 1');
  ); */
  int currentNumActiveThreads = numActiveThreads.fetch_sub(1) - 1;

  EM_ASM({
    console.log('try to pop task', $0);
  }, currentNumActiveThreads);
  
  taskSemaphore.wait();

  currentNumActiveThreads = numActiveThreads.fetch_add(1) + 1;

  /* EM_ASM(
    console.log('pop lock sema waited');
  ); */

  Task *task;
  {
    std::unique_lock<Mutex> lock(taskMutex);
    // lock.lock();

    /* if (lockedTasks.size() == 0) {
      abort();
    } */

    // XXX lock here; perhaps have a queue of only requirement fulfilled tasks

    task = tasks.front();
    tasks.pop_front();

    if (currentNumActiveThreads < 8) {
      EM_ASM({
        console.log('fewer than 8 threads', $0, $1);
      }, currentNumActiveThreads, tasks.size());
    }

    // task->ensurePop();
  }
  /* if (task == nullptr) {
    EM_ASM(
      console.log('failed to pop task!');
    );
    abort();
  } */
  return task;
}
void TaskQueue::runLoop() {
  /* EM_ASM(
    console.log('run loop');
  ); */
  {
    Task *task;
    while ((task = popLockTask())) {
      task->run();
      // task->unlock();
      EM_ASM({
        console.log('done running task');
      });
      delete task;
      // task = nullptr;

      // flushTasks();
    }
  }
  /* EM_ASM(
    console.log('thread exited due to no task!');
  ); */
  abort();
}
/* void TaskQueue::flushTasks() {
  int numSignals = 0;
  {
    std::unique_lock<Mutex> lock(taskMutex);

    bool lockedTask = true;
    while (lockedTask) {
      lockedTask = false;
      for (auto iter = tasks.begin(); iter != tasks.end(); iter++) {
        Task *task = *iter;
        if (task->tryLock()) {
          lockedTasks.push_back(task);
          tasks.erase(iter);

          numSignals++;

          lockedTask = true;
          break;
        }
      }
    }
  }

  for (int i = 0; i < numSignals; i++) {
    taskSemaphore.signal();
  }
} */