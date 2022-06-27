#ifndef SYNC_H
#define SYNC_H

#include "vectorMath.h"
#include <vector>
#include <deque>
#include <atomic>

//

class DCInstance;

//

class Mutex {
public:
  // volatile std::atomic_flag flag;
  uint32_t flag;

  Mutex();
  Mutex(const Mutex &other);
  Mutex(Mutex &&other);
  Mutex &operator=(const Mutex &other);
  ~Mutex();
  void lock();
  void unlock();
  bool try_lock();
};

/* class Mutex2 {
public:
  volatile std::atomic_flag flag;

  Mutex2();
  Mutex2(const Mutex &other);
  Mutex2(Mutex &&other);
  Mutex2 &operator=(const Mutex2 &other);
  ~Mutex2();
  // void lock();
  void unlock();
  bool try_lock();
}; */

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

#endif // SYNC_H
