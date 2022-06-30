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
  /* EM_ASM({
      // console.time('push task ' + $0);

      const now = performance.now();
      if (globalThis.lastPushTime) {
        const timeDiff = now - globalThis.lastPushTime;
        console.log('time since push', timeDiff);
      }
      globalThis.lastPushTime = now;
  }, pthread_self()); */
  {
    std::unique_lock<Mutex> lock(taskMutex);
    /* EM_ASM({
      console.log('push task start', SharedArrayBuffer, $0, $1);
    }, tasks.size(), (void *)this); */
    tasks.push_back(task);

    /* EM_ASM({
      console.log('push task', $0);
    }, tasks.size()); */

    // numTasks++;
    /* EM_ASM({
      console.log('push task end', $0);
    }, tasks.size()); */
  }
  taskSemaphore.signal();
  // flushTasks();

  /* EM_ASM({
      console.timeEnd('push task ' + $0);
      console.log('num ready tasks', $1);
  }, pthread_self(), tasks.size()); */
}
// std::atomic<int> numActiveThreads(NUM_THREADS);
Task *TaskQueue::popLockTask() {
  /* EM_ASM(
    console.log('pop lock task 1');
  ); */
  // int currentNumActiveThreads = numActiveThreads.fetch_sub(1) - 1;

  /* EM_ASM({
    console.log('try to pop task', $0);
  }, currentNumActiveThreads); */
  
  /* EM_ASM({
    globalThis.lockStartTime = performance.now();
  }); */
  
  /* if (currentNumActiveThreads < 2) {
    std::cout << "pop task 1 " << currentNumActiveThreads << std::endl;
  } */
  taskSemaphore.wait();
  
  /* double time = EM_ASM_DOUBLE({
    const lockEndTime = performance.now();
    return lockEndTime - globalThis.lockStartTime;
  });
  int currentNumActiveThreads2 = numActiveThreads.fetch_add(1) + 1;
  if (currentNumActiveThreads2 < 2) {
    std::cout << "pop task 2 " << currentNumActiveThreads2 << " " << time << std::endl;
  } */

  EM_ASM({
    // console.time('pop task ' + $0);
    globalThis.requestStartTime = performance.now();
  });

  /* EM_ASM({
    console.timeEnd('pop task ' + $0);
    console.log('num active threads', $1, $2, $3);
  }, pthread_self(), currentNumActiveThreads, currentNumActiveThreads2, tasks.size()); */

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

    task = tasks.front();
    tasks.pop_front();

    /* if (currentNumActiveThreads < 8) {
      EM_ASM({
        console.log('fewer than 8 threads', $0, $1);
      }, currentNumActiveThreads, tasks.size());
    } */

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
  // {
    for (;;) {
      Task *task = popLockTask();
      if (!task) {
        abort();
      }
      task->run();
      // task->unlock();
      /* EM_ASM({
        console.log('done running task');
      }); */
      delete task;
      // task = nullptr;

      // flushTasks();
    }
  // }
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