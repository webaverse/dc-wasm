#include "sync.h"
#include <iostream>

//

Mutex::Mutex() : flag(0) {
  emscripten_lock_init(&flag);
}
Mutex::Mutex(const Mutex &other) : flag(other.flag) {
  emscripten_lock_init(&flag);
}
Mutex::Mutex(Mutex &&other) : flag(other.flag) {
  emscripten_lock_init(&flag);
}
Mutex::~Mutex() {
  EM_ASM({
    console.log('mutex destroy');
  });
  // abort();
}
void Mutex::lock() {
  emscripten_lock_waitinf_acquire(&flag);
}
void Mutex::unlock() {
  emscripten_lock_release(&flag);
}
bool Mutex::try_lock() {
  return emscripten_lock_try_acquire(&flag);
}
Mutex &Mutex::operator=(const Mutex &other) {
  flag = other.flag;
  emscripten_lock_init(&flag);
  /* if (other.flag.test()) {
    flag.test_and_set(std::memory_order_seq_cst);
  } else {
    flag.clear(std::memory_order_seq_cst);
  } */
  return *this;
}

//

Semaphore::Semaphore(int value) : sema(value) {
  emscripten_semaphore_init(&sema, value);
}
Semaphore::Semaphore() : sema(0) {}
Semaphore::~Semaphore() {}
void Semaphore::wait() {
  emscripten_semaphore_waitinf_acquire(&sema, 1);
}
void Semaphore::signal() {
  emscripten_semaphore_release(&sema, 1);
}