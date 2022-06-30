#include "cache.h"

//

int16_t getCacheIndexWorld(int x, int y) {
    int16_t result = uint8_t(x / cacheWidth);
    result = (result << 8) | uint8_t(y / cacheWidth);
    return result;
}
int getCacheIndexWorld(int x, int y, int z) {
    int result = uint8_t(x / cacheWidth);
    result = (result << 8) | uint8_t(y / cacheWidth);
    result = (result << 8) | uint8_t(z / cacheWidth);
    return result;
}

//

uint32_t getCacheIndexLocal(int x, int y) {
    x = modulo(x, cacheWidth);
    y = modulo(y, cacheWidth);
    return x + y * cacheWidth; 
}
uint32_t getCacheIndexLocal(int x, int y, int z) {
    x = modulo(x, cacheWidth);
    y = modulo(y, cacheWidth);
    z = modulo(z, cacheWidth); 
    return x + z * cacheWidth + y * cacheWidth * cacheWidth; 
}