#include "result.h"
#include "main.h"
#include <iostream>
#include <emscripten/threading.h>

//

ResultQueue::ResultQueue() :
  ids(0)
  {}
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
    const id = $0;
    const result = $1;
    // const time = $2;
    // console.log('result', time);
    // console.log('dc threader got result', {id, result, lol: globalThis.lol});
    // alert('hello world!');
    // throw 'all done';
    if (!globalThis.resultEvent) {
      globalThis.resultEvent = new MessageEvent('result', {
        data: {
          id: 0,
          result: 0,
        },
      });
    }
    globalThis.resultEvent.data.id = id;
    globalThis.resultEvent.data.result = result;
    globalThis.dispatchEvent(globalThis.resultEvent);
    // console.timeEnd('result');
  }, id, result);
  // emscripten_async_run_in_main_runtime_thread(EM_FUNC_SIG_VII, result_em_func_vii, (int)id, (int)result);
  // emscripten_wasm_worker_post_function_vii(DualContouring::parentThreadId, result_em_func_vii, (int)id, (int)result);
  // emscripten_async_run_in_main_runtime_thread(EM_FUNC_SIG_VII, result_em_func_vii, (int)id, result);
}