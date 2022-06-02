#ifndef DENSITY_H
#define DENSITY_H

#include "vectorMath.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include "chunk.h"

float Density_Func(const vm::vec3 &worldPosition, Chunk &chunkNoise);

#endif //	DENSITY_H
