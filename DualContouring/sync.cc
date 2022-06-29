#include "sync.h"
#include <iostream>
#include <emscripten.h>

//

Mutex::Mutex() {}
Mutex::Mutex(bool locked) {
  if (locked) {
    flag.test_and_set(std::memory_order_acquire);
  }
}
Mutex::~Mutex() {}
void Mutex::lock() {
  for (;;) {
    bool oldValue = flag.test_and_set(std::memory_order_acquire);
    if (!oldValue) {
      break;
    } else {
      flag.wait(oldValue);
    }
  }
}
void Mutex::unlock() {
  flag.clear(std::memory_order_release);
  flag.notify_one();
}
bool Mutex::try_lock() {
  bool oldValue = flag.test_and_set(std::memory_order_acquire);
  return !oldValue;
}
bool Mutex::test() {
  return !flag.test();
}
void Mutex::wait() {
  lock();
  unlock();
}

//

Semaphore::Semaphore(int value) : value(value) {}
Semaphore::Semaphore() : value(0) {}
Semaphore::~Semaphore() {}
void Semaphore::wait() {
  for (;;) {
    int oldValue;
    {
      oldValue = value.fetch_sub(1, std::memory_order_acquire);
      if (oldValue > 0) {
        break;
      } else {
        value.fetch_add(1, std::memory_order_release);
      }
    }
    value.wait(oldValue);
  }
}
void Semaphore::signal() {
  {
    value.fetch_add(1, std::memory_order_release);
    value.notify_one();
  }
}