#include "result.h"
#include "main.h"
#include "promise.h"
#include <iostream>
#include <emscripten/threading.h>

//

ResultQueue::ResultQueue() {}
ResultQueue::~ResultQueue() {}

std::shared_ptr<Promise> ResultQueue::createPromise(uint32_t id) {
  std::unique_lock<Mutex> lock(mutex);

  // uint32_t id = ++ids;
  std::shared_ptr<Promise> promise(new Promise(id, this));
  livePromises.push_back(promise);
  return promise;
}
std::shared_ptr<Promise> ResultQueue::findPromise(uint32_t id) {
  std::unique_lock<Mutex> lock(mutex);

  for (auto it = livePromises.begin(); it != livePromises.end(); it++) {
    if ((*it)->id == id) {
      return *it;
    }
  }
  return nullptr;
}
void ResultQueue::cancelPromise(uint32_t id) {
  std::shared_ptr<Promise> promise = findPromise(id);
  if (promise) {
    // std::cout << "cancel promise " << id << std::endl;
    promise->kill();
  } else {
    // std::cout << "no cancel promise " << id << std::endl;
  }
}

// calling these functions makes the promise dead. only one of them may be called for any given promise.
void ResultQueue::resolvePromise(uint32_t id, void *result) {
  /* EM_ASM({
    console.log('post result');
  }); */
  /* double lol = EM_ASM_DOUBLE({
    return performance.now() - globalThis.requestStartTime;
  }); */
  MAIN_THREAD_ASYNC_EM_ASM({
    handleResult($0, $1);
  }, id, result);
  // emscripten_async_run_in_main_runtime_thread(EM_FUNC_SIG_VII, result_em_func_vii, (int)id, (int)result);
  // emscripten_wasm_worker_post_function_vii(DualContouring::parentThreadId, result_em_func_vii, (int)id, (int)result);
  // emscripten_async_run_in_main_runtime_thread(EM_FUNC_SIG_VII, result_em_func_vii, (int)id, result);
}
void ResultQueue::killPromise(uint32_t id) {
  std::unique_lock<Mutex> lock(mutex);
  for (auto it = livePromises.begin(); it != livePromises.end(); it++) {
    if ((*it)->id == id) {
      livePromises.erase(it);
      break;
    }
  }
}