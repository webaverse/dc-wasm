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

Semaphore::Semaphore(int count) : count(count) {}
void Semaphore::signal() {
    std::unique_lock<std::mutex> lock(mtx);
    count++;
    cv.notify_one();
}
void Semaphore::wait() {
    std::unique_lock<std::mutex> lock(mtx);
    while (count == 0) {
        cv.wait(lock);
    }
    count--;
}
/* bool Semaphore::try_wait() {
    std::lock_guard<decltype(mutex_)> lock(mutex_);
    if(count_) {
        --count_;
        return true;
    }
    return false;
} */