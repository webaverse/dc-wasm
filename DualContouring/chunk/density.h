#ifndef DENSITY_H
#define DENSITY_H

#include "../math/vectorMath.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include "../chunk/chunk.h"

float Density_Func(const vm::vec3 &worldPosition, Chunk &chunkNoise);

#endif //	DENSITY_H
