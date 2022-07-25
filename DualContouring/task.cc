#include "task.h"
#include "instance.h"
#include "vector.h"
#include "sort.h"
#include <limits>
#include <iostream>
#include <emscripten/atomic.h>

//

/* Task::Task(uint32_t id, std::function<void()> fn) :
  id(id),
  fn(fn),
  live(true),
  worldPosition{
    0,
    0,
    0
  },
  lod(0),
  priority(0)
{} */
Task::Task(uint32_t id, const vm::vec3 &worldPosition, const vm::vec3 &halfSize, std::function<void()> fn) :
  id(id),
  fn(fn),
  live(true),
  worldPosition(worldPosition),
  halfSize(halfSize),
  priority(0)
{}
Task::Task(uint32_t id, int priority, std::function<void()> fn) :
  id(id),
  fn(fn),
  live(true),
  worldPosition{
    0,
    0,
    0
  },
  halfSize{
    0,
    0,
    0
  },
  priority(priority)
{}
Task::Task(uint32_t id, const vm::vec3 &worldPosition, const vm::vec3 &halfSize, int priority, std::function<void()> fn) :
  id(id),
  fn(fn),
  live(true),
  worldPosition(worldPosition),
  halfSize(halfSize),
  priority(priority)
{}

Task::~Task() {}

void Task::run() {
  fn();
}
void Task::cancel() {
  live.store(false);
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
  Frustum frustum = getFrustum();
  const float taskDistance = getTaskDistance(task, frustum);
  {
    std::unique_lock<Mutex> lock(taskMutex);

    bool found = false;
    for (size_t i = 0; i < tasks.size(); i++) {
      auto iter = tasks.begin() + i;
      const float taskDistance2 = getTaskDistance(*iter, frustum);
      if (taskDistance < taskDistance2) {
        tasks.insert(iter, task);
        found = true;
        break;
      }
    }
    if (!found) {
      tasks.push_back(task);
    }
  }
  taskSemaphore.signal();
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

  /* EM_ASM({
    // console.time('pop task ' + $0);
    globalThis.requestStartTime = performance.now();
  }); */

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
void TaskQueue::cancelTask(uint32_t taskId) {
  std::unique_lock<Mutex> lock(taskMutex);
  for (auto it = tasks.begin(); it != tasks.end(); it++) {
    Task *task = (*it);
    if (task->id == taskId && task->live) {
      /* EM_ASM({
        console.log('cancel task', $0);
      }, task->id); */
      task->cancel();
      break;
    }
  }
}
void TaskQueue::runLoop() {
  /* EM_ASM(
    console.log('run loop');
  ); */
  // {
    for (;;) {
      Task *task = popLockTask();
      if (!task) {
        std::cout << "failed to pop task" << std::endl;
        abort();
      }
      if (task->live) {
        task->run();
      }
      /* EM_ASM({
        console.log('done running task');
      }); */
      delete task;
      // task = nullptr;
    }
  // }
  /* EM_ASM(
    console.log('thread exited due to no task!');
  ); */
  std::cout << "main loop exited" << std::endl;
  abort();
}
void TaskQueue::setCamera(const vm::vec3 &worldPosition, const vm::vec3 &cameraPosition, const Quat &cameraQuaternion, const std::array<float, 16> &projectionMatrix) {
  std::unique_lock<Mutex> lock(taskMutex);

  this->worldPosition = worldPosition;
  this->cameraPosition = cameraPosition;
  this->cameraQuaternion = cameraQuaternion;
  this->projectionMatrix = projectionMatrix;

  sortTasksInternal();
}
Frustum TaskQueue::getFrustum() {
  Matrix matrixWorld(
    Vec{
      cameraPosition.x,
      cameraPosition.y,
      cameraPosition.z
    },
    Quat{
      cameraQuaternion.x,
      cameraQuaternion.y,
      cameraQuaternion.z,
      cameraQuaternion.w
    },
    Vec{1, 1, 1}
  );
  Matrix matrixWorldInverse(matrixWorld);
  matrixWorldInverse.invert();
  Frustum frustum = Frustum::fromMatrix(
    Matrix::fromArray(projectionMatrix.data()) *= matrixWorldInverse
  );
  return frustum;
}
float TaskQueue::getTaskDistance(Task *task, const Frustum &frustum) {
  float distance = vm::length(task->worldPosition - worldPosition);

  Sphere sphere(
    Vec{
      task->worldPosition.x,
      task->worldPosition.y,
      task->worldPosition.z
    },
    (float)std::sqrt(
      task->halfSize.x * task->halfSize.x +
      task->halfSize.y * task->halfSize.y +
      task->halfSize.z * task->halfSize.z
    )
  );
  if (!frustum.intersectsSphere(sphere)) {
    distance += frustumCullDistancePenalty;
  }
  distance += task->priority * priorityDistancePenalty;
  return distance;
}
void TaskQueue::sortTasksInternal() {
  const vm::vec3 &worldPosition = this->worldPosition;
  Frustum frustum = getFrustum();

  sort<Task *>(tasks, worldPosition, frustum);

  /* std::vector<std::pair<Task *, float>> taskDistances;
  taskDistances.reserve(tasks.size());
  for (size_t i = 0; i < tasks.size(); i++) {
    Task *task = tasks[i];
    float distance = getTaskDistance(task, frustum);
    taskDistances.push_back(std::pair<Task *, float>(task, distance));
  }

  std::sort(
    taskDistances.begin(),
    taskDistances.end(),
    [&](const std::pair<Task *, float> &a, const std::pair<Task *, float> &b) -> bool {
      return a.second < b.second;
    }
  );

  for (size_t i = 0; i < taskDistances.size(); i++) {
    tasks[i] = taskDistances[i].first;
  } */
}