#include "result.h"
#include "main.h"
#include <iostream>
#include <emscripten/threading.h>

//

ResultQueue::ResultQueue() : ids(0) {}
ResultQueue::~ResultQueue() {}

uint32_t ResultQueue::getNextId() {
  return ++ids;
}

void ResultQueue::pushResult(uint32_t id, void *result) {
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