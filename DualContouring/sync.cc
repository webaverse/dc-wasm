#include "sync.h"
#include <iostream>
#include <emscripten.h>
#include <emscripten/atomic.h>

//

Mutex::Mutex() : flag(0) {}
Mutex::Mutex(const Mutex &other) : flag(other.flag) {}
Mutex::Mutex(Mutex &&other) : flag(other.flag) {}
Mutex::~Mutex() {
  EM_ASM({
    console.log('mutex destroy');
  });
  // abort();
}
void Mutex::lock() {
  for (;;) {
    uint32_t oldValue = emscripten_atomic_exchange_u32(&flag, 1);
    if (oldValue == 0) {
      break;
    } else {
      // flag.wait(oldValue);
      EM_ASM({
        // console.log('atomic wait', Module.HEAP32, $0 / Module.HEAP32.BYTES_PER_ELEMENT, $1);
         Atomics.wait(Module.HEAP32, $0 / Module.HEAP32.BYTES_PER_ELEMENT, $1);
      }, (void *)&flag, oldValue);
    }
  }
}
void Mutex::unlock() {
  emscripten_atomic_store_u32(&flag, 0);
  EM_ASM({
    Atomics.notify(Module.HEAP32, $0 / Module.HEAP32.BYTES_PER_ELEMENT);
  }, (void *)&flag);
}
bool Mutex::try_lock() {
  uint32_t oldValue = emscripten_atomic_exchange_u32(&flag, 1);
  return oldValue == 0;
  // bool oldValue = flag.test_and_set(std::memory_order_seq_cst);
  // return !oldValue;
}
Mutex &Mutex::operator=(const Mutex &other) {
  flag = other.flag;
  /* if (other.flag.test()) {
    flag.test_and_set(std::memory_order_seq_cst);
  } else {
    flag.clear(std::memory_order_seq_cst);
  } */
  return *this;
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

      oldValue = value.fetch_sub(1, std::memory_order_seq_cst);
      if (oldValue > 0) {
        break;
      } else {
        value.fetch_add(1, std::memory_order_seq_cst);
      }
    }
    EM_ASM({
      // console.log('atomic notify', Module.HEAP32, $0 / Module.HEAP32.BYTES_PER_ELEMENT);
      Atomics.wait(Module.HEAP32, $0 / Module.HEAP32.BYTES_PER_ELEMENT, $1);
    }, (void *)&value, oldValue);
    // value.wait(oldValue);
  }
}
void Semaphore::signal() {
  {
    std::unique_lock<Mutex> lock(mutex);
    value.fetch_add(1, std::memory_order_seq_cst);
    // value.notify_all();
  }
  EM_ASM({
    // console.log('atomic notify', Module.HEAP32, $0 / Module.HEAP32.BYTES_PER_ELEMENT);
    Atomics.notify(Module.HEAP32, $0 / Module.HEAP32.BYTES_PER_ELEMENT);
  }, (void *)&value);
}