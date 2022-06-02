#include "noise.h"
#include "vectorMath.h"

Noise::Noise(int s, double frequency, int octaves) : fastNoise(s) {
  fastNoise.SetFrequency(frequency);
  fastNoise.SetFractalOctaves(octaves);
}

Noise::~Noise() {}

double Noise::in2DRaw(double x, double y) {
  return fastNoise.GetSimplexFractal(x, y);
}
double Noise::in3DRaw(double x, double y, double z) {
  return fastNoise.GetSimplexFractal(x, y, z);
}

double Noise::in2D(double x, double y) {
  return vm::clamp((1.0 + in2DRaw(x, y)) / 2.0, 0., 1.);
}
double Noise::in3D(double x, double y, double z) {
  return vm::clamp((1.0 + in3DRaw(x, y, z)) / 2.0, 0., 1.);
}