#include "promise.h"
#include "instance.h"
#include <iostream>
#include <emscripten/atomic.h>

//

Promise::Promise(uint32_t id, ResultQueue *resultQueue) : id(id), resultQueue(resultQueue), live(true) {}
Promise::~Promise() {}

/* void *Promise::get() {
  return value;
} */
bool Promise::resolve(void *value) {
  if (live.exchange(false)) {
    resultQueue->resolvePromise(id, value);
    // value = newValue;
    // flag.store(1);
    // flag.notify_all();
    return true;
  } else {
    return false;
  }
}
bool Promise::kill() {
  if (live.exchange(false)) {
    resultQueue->killPromise(id);
    return true;
  } else {
    return false;
  }
}
/* bool Promise::test() {
  return flag.load() == 1;
}
void Promise::wait() {
  flag.wait(0);
} */