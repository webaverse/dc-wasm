#include "promise.h"
#include "instance.h"
#include <iostream>
#include <emscripten/atomic.h>

//

Promise::Promise() : mutex(true), value(nullptr) {}
Promise::~Promise() {}

void *Promise::get() {
  return value;
}
void Promise::resolve(void *newValue) {
  value = newValue;
  mutex.unlock();
}
bool Promise::test() {
  return mutex.test();
}
void Promise::wait() {
  mutex.wait();
}