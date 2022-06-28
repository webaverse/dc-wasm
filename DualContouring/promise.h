#ifndef PROMISE_H
#define PROMISE_H

#include "vectorMath.h"
#include "sync.h"
// #include "lock.h"
#include <vector>
#include <deque>
#include <atomic>
#include <emscripten.h>

//

class Promise {
public:
    Mutex mutex;
    void *value;

    Promise();
    ~Promise();

    void *get();
    void resolve(void *value = nullptr);
    bool test();
    void wait();
};

#endif // PROMISE_H
