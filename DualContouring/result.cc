#include "result.h"
#include <iostream>

//

Result::Result(unsigned int id, void *result) : id(id), result(result) {}
Result::~Result() {}

//

ResultQueue::ResultQueue() {}
ResultQueue::~ResultQueue() {}

void ResultQueue::pushResult(Result *result) {
  std::unique_lock<std::mutex> lock(resultLock);
  results.push_back(result);
}
Result *ResultQueue::popResult() {
  std::unique_lock<std::mutex> lock(resultLock);
  if (results.size() == 0) {
    return nullptr;
  }
  Result *result = results.front();
  results.pop_front();
  return result;
}