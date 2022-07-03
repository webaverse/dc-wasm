#include "./DualContouring/vectorMath.h"
#include "util.h"
#include <vector>

float lerp(const float &a, const float &b, const float &f) {
    return a + f * (b - a);
}
int align(int x, int N) {
    int r = x % N;
    return r == 0 ? x : x - r + N;
}
int align4(int x) {
    return align(x, 4);
}