#include "cache.h"

uint64_t getIndex(int x, int y) {
    uint64_t result = uint16_t(x);
    result = (result << 16) | uint16_t(0);
    result = (result << 16) | uint16_t(y);
    result = (result << 16) | uint16_t(0);
    return result;
}
uint64_t getIndex(int x, int y, int z) {
    uint64_t result = uint16_t(x);
    result = (result << 16) | uint16_t(y);
    result = (result << 16) | uint16_t(z);
    result = (result << 16) | uint16_t(1);
    return result;
}