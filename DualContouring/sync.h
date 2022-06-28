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
  std::atomic_flag flag;

  Mutex();
  Mutex(bool locked);
  ~Mutex();
  void lock();
  void unlock();
  bool try_lock();
  bool test();
  void wait();
};

// implements a semaphore using only c++ atomic value
class Semaphore {
public:
  // Mutex mutex;
  std::atomic<int> value;

  Semaphore(int value);
  Semaphore();
  ~Semaphore();
  void wait();
  void signal();
};

#endif // SYNC_H
