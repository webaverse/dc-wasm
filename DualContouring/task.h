#ifndef TASK_H
#define TASK_H

#include "vectorMath.h"
#include "sync.h"
#include "lock.h"
#include "vector.h"
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

    vm::vec3 worldPosition;
    int lod;

    Task(uint32_t id, std::function<void()> fn);
    Task(uint32_t id, const vm::vec3 &worldPosition, int lod, std::function<void()> fn);
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
    
    Mutex taskMutex;
    Semaphore taskSemaphore;

    vm::vec3 worldPosition;
    Quat worldQuaternion;

    TaskQueue();
    ~TaskQueue();
    
    void pushTask(Task *task);
    void pushTaskPre(Task *task);
    Task *popLockTask();
    void runLoop();
    void cancelTask(uint32_t taskId);
    // void flushTasks();

    void setSortPositionQuaternion(const vm::vec3 &worldPosition, const Quat &worldQuaternion);

    void sortTasksInternal();
};

#endif // TASK_H
