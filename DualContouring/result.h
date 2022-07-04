#ifndef RESULT_H
#define RESULT_H

#include "vectorMath.h"
#include "sync.h"
#include <vector>
#include <deque>
#include <unordered_map>
#include <atomic>

//

class Promise;

//

class ResultQueue {
public:
    Mutex mutex;
    uint32_t ids;
    std::deque<std::shared_ptr<Promise>> livePromises;

    ResultQueue();
    ~ResultQueue();
    
    // uint32_t getNextId();
    std::shared_ptr<Promise> createPromise();
    std::shared_ptr<Promise> findPromise(uint32_t id);
    void cancelPromise(uint32_t id);
    void resolvePromise(uint32_t id, void *value);
    void killPromise(uint32_t id);
};

#endif // RESULT_H
