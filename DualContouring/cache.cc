#include "cache.h"
#include "xxhash.h"

//

constexpr int hashSeed = 0;

uint32_t getCacheIndexWorld(int x, int y) {
    std::array<int, 2> coords = {x, y};
    return XXH32(coords.data(), coords.size(), hashSeed);
}
uint32_t getCacheIndexWorld(int x, int y, int z) {
    std::array<int, 3> coords = {x, y, z};
    return XXH32(coords.data(), coords.size(), hashSeed);
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