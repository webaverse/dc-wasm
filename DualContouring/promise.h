#ifndef PROMISE_H
#define PROMISE_H

#include "vectorMath.h"
#include "sync.h"
#include "result.h"
// #include "lock.h"
#include <vector>
#include <deque>
#include <atomic>
#include <emscripten.h>

//

class Promise {
public:
    uint32_t id;
    ResultQueue *resultQueue;
    // void *value;
    // Mutex mutex;
    std::atomic<bool> live;

    Promise(uint32_t id, ResultQueue *resultQueue);
    ~Promise();

    // void *get();
    bool resolve(void *value = nullptr);
    bool kill();
    // void wait();
};

#endif // PROMISE_H
