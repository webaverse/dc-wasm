#include "task.h"
#include "instance.h"
#include "vector.h"
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
Task::Task(uint32_t id, const vm::vec3 &worldPosition, int lod, std::function<void()> fn) :
  id(id),
  fn(fn),
  live(true),
  worldPosition(worldPosition),
  lod(lod),
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
  lod(0),
  priority(priority)
{}
Task::Task(uint32_t id, const vm::vec3 &worldPosition, int lod, int priority, std::function<void()> fn) :
  id(id),
  fn(fn),
  live(true),
  worldPosition(worldPosition),
  lod(lod),
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

void TaskQueue::pushTask(Task *task, bool log) {
  Frustum frustum = getFrustum();
  const float taskDistanceSq = getTaskDistanceSq(task, frustum);
  if (log) {
    std::cout << "push task distance " << task->priority << " " << taskDistanceSq << std::endl;
  }
  {
    std::unique_lock<Mutex> lock(taskMutex);

    bool found = false;
    for (size_t i = 0; i < tasks.size(); i++) {
      auto iter = tasks.begin() + i;
      const float taskDistanceSq2 = getTaskDistanceSq(*iter, frustum);
      if (taskDistanceSq < taskDistanceSq2) {
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
/* void TaskQueue::pushTaskPre(Task *task) {
  {
    std::unique_lock<Mutex> lock(taskMutex);
    tasks.push_front(task);
  }
  taskSemaphore.signal();
} */
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

      // flushTasks();
    }
  // }
  /* EM_ASM(
    console.log('thread exited due to no task!');
  ); */
  std::cout << "main loop exited" << std::endl;
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
void TaskQueue::setCamera(const vm::vec3 &worldPosition, const Quat &worldQuaternion, const std::array<float, 16> &projectionMatrix) {
  std::unique_lock<Mutex> lock(taskMutex);

  this->worldPosition = worldPosition;
  this->worldQuaternion = worldQuaternion;
  this->projectionMatrix = projectionMatrix;

  sortTasksInternal();
}
Frustum TaskQueue::getFrustum() {
  Matrix matrixWorld(
    Vec{
      worldPosition.x,
      worldPosition.y,
      worldPosition.z
    },
    Quat{
      worldQuaternion.x,
      worldQuaternion.y,
      worldQuaternion.z,
      worldQuaternion.w
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
float TaskQueue::getTaskDistanceSq(Task *task, const Frustum &frustum) {
  if (task->lod > 0) {  
    float distanceSq = vm::lengthSq(task->worldPosition - worldPosition);

    /* std::cout << "task sort world position " <<
      task->worldPosition.x << " " <<
      task->worldPosition.y << " " <<
      task->worldPosition.z << " " <<
      std::endl; */

    Sphere sphere(
      Vec{
        task->worldPosition.x,
        task->worldPosition.y,
        task->worldPosition.z
      },
      (float)std::sqrt(3.f * ((float)task->lod/2.f) * ((float)task->lod/2.f))
    );
    if (!frustum.intersectsSphere(sphere)) {
      distanceSq += frustumCullDistancePenalty;
    }
    return distanceSq;
  } else {
    return 0;
  }
}
void TaskQueue::sortTasksInternal() {
  Frustum frustum = getFrustum();

  std::vector<std::pair<Task *, float>> taskDistances;
  taskDistances.reserve(tasks.size());
  for (size_t i = 0; i < tasks.size(); i++) {
    Task *task = tasks[i];
    float distanceSq = getTaskDistanceSq(task, frustum);
    taskDistances.push_back(std::pair<Task *, float>(task, distanceSq));
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
  }
}