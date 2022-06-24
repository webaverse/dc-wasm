#ifndef TASK_H
#define TASK_H

#include "vectorMath.h"
#include <vector>
#include <mutex>
#include <semaphore>

//

class DCInstance;

//

class Task {
public:
    DCInstance *inst;
    std::vector<vm::ivec3> chunkPositions;
    int lod;
    int flags;
    std::function<void *()> fn;

    Task(DCInstance *inst, std::vector<vm::ivec3> &&chunkPositions, int lod, int flags, std::function<void *()> &&fn);
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
    std::vector<Task *> tasks;
    std::mutex taskLock;
    std::condition_variable taskCondVar;

    TaskQueue(DCInstance *inst);
    ~TaskQueue();
    
    void pushTask(Task *task);
    Task *popLockTask();
};

#endif // TASK_H
