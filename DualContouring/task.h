#ifndef TASK_H
#define TASK_H

#include "vectorMath.h"
#include "sync.h"
#include "lock.h"
#include "vector.h"
#include <array>
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
    vm::vec3 halfSize;
    // int lod;
    int priority;
    // Sphere sphere;

    Task(uint32_t id, std::function<void()> fn);
    Task(uint32_t id, const vm::vec3 &worldPosition, const vm::vec3 &halfSize, std::function<void()> fn);
    Task(uint32_t id, int priority, std::function<void()> fn);
    Task(uint32_t id, const vm::vec3 &worldPosition, const vm::vec3 &halfSize, int priority, std::function<void()> fn);
    ~Task();

    // bool tryLock();
    // void lock();
    // void unlock();
    void run();
    void cancel();
    // void ensurePop();

    int getPriority();
    Sphere getSphere();
};

//

class TaskQueue {
public:
    DCInstance *inst;
    std::deque<Task *> tasks;
    
    Mutex taskMutex;
    Semaphore taskSemaphore;

    vm::vec3 worldPosition;
    vm::vec3 cameraPosition;
    Quat cameraQuaternion;
    std::array<float, 16> projectionMatrix;

    TaskQueue();
    ~TaskQueue();
    
    void pushTask(Task *task);
    Task *popLockTask();
    void runLoop();
    void cancelTask(uint32_t taskId);

    void setCamera(const vm::vec3 &worldPosition, const vm::vec3 &cameraPosition, const Quat &cameraQuaternion, const std::array<float, 16> &projectionMatrix);
    Frustum getFrustum();
    float getTaskDistanceSq(Task *task, const Frustum &frustum);

    void sortTasksInternal();
};

#endif // TASK_H
