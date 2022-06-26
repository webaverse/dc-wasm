#ifndef TASK_H
#define TASK_H

#include "vectorMath.h"
#include <vector>
#include <deque>
#include <mutex>
// #include <semaphore>
#include <atomic>

//

class DCInstance;

//

class Mutex {
public:
  std::atomic_flag flag;

  Mutex();
  ~Mutex();
  void lock();
  void unlock();
};

// implements a semaphore using only c++ atomic value
class Semaphore {
public:
  Mutex mutex;
  std::atomic<int> value;

  Semaphore(int value);
  Semaphore();
  ~Semaphore();
  void wait();
  void signal();
};

//

class Task {
public:
    std::vector<vm::ivec3> chunkPositions;
    std::function<bool()> tryLockFn;
    std::function<void()> unlockFn;
    std::function<void()> fn;

    Task(std::function<bool()> &&tryLockFn, std::function<void()> &&unlockFn, std::function<void()> &&fn);
    ~Task();

    bool tryLock();
    void lock();
    void unlock();
    void run();
    std::pair<bool, void *> tryLockRun();
};

//

class TaskQueue {
public:
    DCInstance *inst;
    std::deque<Task *> tasks;
    std::atomic<size_t> numTasks;
    Mutex taskMutex;
    Semaphore taskSemaphore;

    TaskQueue();
    ~TaskQueue();
    
    void pushTask(Task *task);
    Task *popLockTask();
    void runLoop();
};

#endif // TASK_H
