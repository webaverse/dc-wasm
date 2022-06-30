#ifndef SYNC_H
#define SYNC_H

#include "vectorMath.h"
#include <vector>
#include <deque>
#include <atomic>
#include <mutex>
#include <condition_variable>

//

class DCInstance;

//

class Mutex {
public:
  std::mutex mutex;

  Mutex();
  Mutex(bool locked);
  ~Mutex();
  void lock();
  void unlock();
  bool try_lock();
  // bool test();
  // void wait();
};

class Semaphore {
public:
    std::mutex mutex_;
    std::condition_variable condition_;
    unsigned long count_; // Initialized as locked.

    Semaphore(unsigned long count = 0);

    void signal();
    void wait();
    /* bool try_wait(); */
};

#endif // SYNC_H
