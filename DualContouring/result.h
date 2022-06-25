#ifndef RESULT_H
#define RESULT_H

#include "vectorMath.h"
#include <vector>
#include <deque>
#include <mutex>
#include <semaphore>
#include <unordered_map>

//

class ResultQueue {
public:
    uint32_t ids;
    // std::unordered_map<uint32_t, void *> results;
    // std::mutex resultLock;

    ResultQueue();
    ~ResultQueue();
    
    uint32_t getNextId();
    void pushResult(uint32_t id, void *result);
    // void *popResult(uint32_t id);
};

#endif // RESULT_H
