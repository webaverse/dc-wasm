#ifndef TASK_H
#define TASK_H

#include "vectorMath.h"
#include <vector>
#include <deque>
#include <mutex>
#include <semaphore>

//

class DCInstance;

//

class Task {
public:
    std::vector<vm::ivec3> chunkPositions;
    std::function<bool()> lockFn;
    std::function<void()> unlockFn;
    std::function<void *()> fn;

    Task(std::function<bool()> lockFn, std::function<bool()> unlockFn, std::function<void *()> fn);
    ~Task();

    bool tryLock();
    void lock();
    void unlock();
    void *run();
    std::pair<bool, void *> tryLockRun();
};

//

class TaskQueue {
public:
    DCInstance *inst;
    std::deque<Task *> tasks;
    std::mutex taskLock;
    std::condition_variable taskCondVar;

    TaskQueue(DCInstance *inst);
    ~TaskQueue();
    
    void pushTask(Task *task);
    Task *popLockTask();
};

#endif // TASK_H
