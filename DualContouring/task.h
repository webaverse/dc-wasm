#ifndef TASK_H
#define TASK_H

#include "vectorMath.h"
#include "sync.h"
#include "lock.h"
#include <vector>
#include <deque>
// #include <semaphore>
#include <atomic>
#include <emscripten.h>

//

class DCInstance;

//

class Task {
public:
    MultiChunkLock *multiChunkLock;
    std::function<void()> fn;
    std::atomic_flag popped;

    Task(MultiChunkLock *multiChunkLock, std::function<void()> fn);
    ~Task();

    bool tryLock();
    void lock();
    void unlock();
    void run();
    // std::pair<bool, void *> tryLockRun();
    void ensurePop();
};

//

class TaskQueue {
public:
    DCInstance *inst;
    std::deque<Task *> tasks;
    std::deque<Task *> lockedTasks;
    // std::atomic<size_t> numTasks;
    Mutex taskMutex;
    Semaphore taskSemaphore;

    TaskQueue();
    ~TaskQueue();
    
    void pushTask(Task *task);
    Task *popLockTask();
    void runLoop();
    void flushTasks();
};

#endif // TASK_H
