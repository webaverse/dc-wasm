#include "result.h"
#include <iostream>

//

ResultQueue::ResultQueue() :
  ids(0)
  {}
ResultQueue::~ResultQueue() {}

uint32_t ResultQueue::getNextId() {
  return ++ids;
}
void ResultQueue::pushResult(uint32_t id, void *result) {
  std::unique_lock<std::mutex> lock(resultLock);
  results.emplace(std::make_pair(id, result));
}
void *ResultQueue::popResult(uint32_t id) {
  std::unique_lock<std::mutex> lock(resultLock);
  const auto &iter = results.find(id);
  if (iter != results.end()) {
    void *result = iter->second;
    results.erase(iter);
    return result;
  } else {
    return nullptr;
  }
}