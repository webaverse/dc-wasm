#include "cache.h"
// #include "xxhash.h"

//

uint32_t getCacheIndexWorld(int x, int y) {
    uint32_t x2 = (((uint32_t)x >> 16) ^ (uint32_t)x) * 0x45d9f3b;
    x2 = ((x2 >> 16) ^ x2) * 0x45d9f3b;
    x2 = (x2 >> 16) ^ x2;

    uint32_t y2 = (((uint32_t)y >> 16) ^ (uint32_t)y) * 0x119de1f3;
    y2 = ((y2 >> 16) ^ y2) * 0x119de1f3;
    y2 = (y2 >> 16) ^ y2;

    return x2 ^ y2;
}
uint32_t getCacheIndexWorld(int x, int y, int z) {
    uint32_t x2 = (((uint32_t)x >> 16) ^ (uint32_t)x) * 0x45d9f3b;
    x2 = ((x2 >> 16) ^ x2) * 0x45d9f3b;
    x2 = (x2 >> 16) ^ x2;

    uint32_t y2 = (uint32_t)y * 2654435761;

    uint32_t z2 = (((uint32_t)z >> 16) ^ (uint32_t)z) * 0x119de1f3;
    z2 = ((z2 >> 16) ^ z2) * 0x119de1f3;
    z2 = (z2 >> 16) ^ z2;

    return x2 ^ y2 ^ z2;
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