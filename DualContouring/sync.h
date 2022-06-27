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
  Mutex(const Mutex &other);
  Mutex(Mutex &&other);
  Mutex &operator=(const Mutex &other);
  ~Mutex();
  void lock();
  void unlock();
  bool try_lock();
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
