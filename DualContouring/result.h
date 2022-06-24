#ifndef RESULT_H
#define RESULT_H

#include "vectorMath.h"
#include <vector>
#include <deque>
#include <mutex>
#include <semaphore>

//

class Result {
public:
    unsigned int id;
    void *result;

    Result(unsigned int id, void *result);
    ~Result();
};

//

class ResultQueue {
public:
    std::deque<Result *> results;
    std::mutex resultLock;

    ResultQueue();
    ~ResultQueue();
    
    void pushResult(Result *result);
    Result *popResult();
};

#endif // RESULT_H
