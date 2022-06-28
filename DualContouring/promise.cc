#include "promise.h"
#include "instance.h"
#include <iostream>
#include <emscripten/atomic.h>

//

Promise::Promise() : flag(0), value(nullptr) {}
Promise::~Promise() {}

/* void *Promise::get() {
  return value;
} */
void Promise::resolve(void *newValue) {
  value = newValue;
  flag.store(1);
  flag.notify_all();
}
bool Promise::test() {
  return flag.load() == 1;
}
void Promise::wait() {
  flag.wait(0);
}