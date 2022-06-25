#include "result.h"
#include "emscripten/threading.h"
#include <iostream>

//

ResultQueue::ResultQueue() :
  ids(0)
  {}
ResultQueue::~ResultQueue() {}

uint32_t ResultQueue::getNextId() {
  return ++ids;
}
void result_em_func_vii(int id, void *result) {
  EM_ASM({
    const id = $0;
    const result = $1;
    console.log('dc threader got result', {id, result, lol: globalThis.lol});
    // alert('hello world!');
    // throw 'all done';
  }, id, result);
}
void ResultQueue::pushResult(uint32_t id, void *result) {
  // std::unique_lock<std::mutex> lock(resultLock);
  // results.emplace(std::make_pair(id, result));

  emscripten_async_run_in_main_runtime_thread(EM_FUNC_SIG_VII, result_em_func_vii, (int)id, result);
}
/* void *ResultQueue::popResult(uint32_t id) {
  std::unique_lock<std::mutex> lock(resultLock);
  const auto &iter = results.find(id);
  if (iter != results.end()) {
    void *result = iter->second;
    results.erase(iter);
    return result;
  } else {
    return nullptr;
  }
} */