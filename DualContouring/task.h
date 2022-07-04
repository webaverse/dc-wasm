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
    uint32_t id;
    std::function<void()> fn;
    std::atomic<bool> live;

    Task(uint32_t id, std::function<void()> fn);
    ~Task();

    // bool tryLock();
    // void lock();
    // void unlock();
    void run();
    void cancel();
    // void ensurePop();
};

//

class TaskQueue {
public:
    DCInstance *inst;
    std::deque<Task *> tasks;
    // std::deque<Task *> lockedTasks;
    Mutex taskMutex;
    Semaphore taskSemaphore;

    TaskQueue();
    ~TaskQueue();
    
    void pushTask(Task *task);
    Task *popLockTask();
    void runLoop();
    void cancelTask(uint32_t taskId);
    // void flushTasks();
};

#endif // TASK_H
