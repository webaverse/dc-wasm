#ifndef DENSITY_H
#define DENSITY_H

#include "vectorMath.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <cstdio>
#include "chunk.h"
#include "instance.h"

float cuboid(const vm::vec3 &worldPosition, const vm::vec3 &origin, const vm::vec3 &halfDimensions);
float terrainDensityFn(const vm::vec3 &worldPosition, const int lod, DCInstance *inst);
float liquidDensityFn(const vm::vec3 &worldPosition, const int lod, DCInstance *inst);

#endif //	DENSITY_H
