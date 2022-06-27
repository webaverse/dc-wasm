#include "sync.h"
#include <iostream>
#include <emscripten.h>

//

Mutex::Mutex() : flag(false) {}
Mutex::~Mutex() {}
void Mutex::lock() {
  for (;;) {
    bool oldValue = flag.test_and_set(std::memory_order_seq_cst);
    if (!oldValue) {
      break;
    } else {
      flag.wait(oldValue);
    }
  }
}
void Mutex::unlock() {
  flag.clear(std::memory_order_seq_cst);
  flag.notify_one();
}
bool Mutex::try_lock() {
  bool oldValue = flag.test_and_set(std::memory_order_seq_cst);
  return !oldValue;
}

//

Semaphore::Semaphore(int value) : value(value) {}
Semaphore::Semaphore() : value(0) {}
Semaphore::~Semaphore() {}
void Semaphore::wait() {
  for (;;) {
    int oldValue;
    {
      std::unique_lock<Mutex> lock(mutex);
      /* EM_ASM({
        console.log('test value', $0);
      }, (int)value); */
      oldValue = value.fetch_sub(1, std::memory_order_seq_cst);
      if (oldValue > 0) {
        break;
      } else {
        value.fetch_add(1, std::memory_order_seq_cst);
      }
    }
    value.wait(oldValue);
  }
}
void Semaphore::signal() {
  {
    std::unique_lock<Mutex> lock(mutex);
    value++;
  }
  value.notify_one();
}