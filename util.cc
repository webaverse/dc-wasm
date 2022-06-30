#include "./DualContouring/vectorMath.h"
#include "util.h"
#include <vector>

float lerp(const float &a, const float &b, const float &f) {
    return a + f * (b - a);
}