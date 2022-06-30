#include "sync.h"
#include <iostream>
#include <emscripten.h>

//

Mutex::Mutex() {}
Mutex::Mutex(bool locked) {
  if (locked) {
    mutex.try_lock();
  }
}
Mutex::~Mutex() {}
void Mutex::lock() {
  mutex.lock();
}
void Mutex::unlock() {
  mutex.unlock();
}
bool Mutex::try_lock() {
  return mutex.try_lock();
}
/* bool Mutex::test() {
  return !flag.test();
}
void Mutex::wait() {
  lock();
  unlock();
} */

//

Semaphore::Semaphore(unsigned long count) : count_(count) {}
void Semaphore::signal() {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    ++count_;
    condition_.notify_one();
}
void Semaphore::wait() {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    while(!count_) // Handle spurious wake-ups.
        condition_.wait(lock);
    --count_;
}
/* bool Semaphore::try_wait() {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    if(count_) {
        --count_;
        return true;
    }
    return false;
} */