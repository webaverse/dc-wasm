#ifndef _NOISES_H_
#define _NOISES_H_

#include "./noise.h"
#include <random>

class Noises {
public:
  int seed;
  std::mt19937 rng;
  Noise elevationNoise1;
  Noise elevationNoise2;
  Noise elevationNoise3;
  Noise temperatureNoise;
  Noise humidityNoise;
  Noise oceanNoise;
  Noise riverNoise;
  Noise lavaNoise;
  Noise grassNoise;
  Noise vegetationNoise;
  Noise mobNoise;

  Noises() = delete;
  Noises(int seed) :
    seed(seed),
    rng(seed),
    elevationNoise1(rng(), 2, 1),
    elevationNoise2(rng(), 2, 1),
    elevationNoise3(rng(), 2, 1),
    temperatureNoise(rng(), 0.001, 4),
    humidityNoise(rng(), 0.001, 4),
    oceanNoise(rng(), 0.001, 4),
    riverNoise(rng(), 0.001, 4),
    lavaNoise(rng(), 0.01, 4),
    grassNoise(rng(), 0.01, 4),
    vegetationNoise(rng(), 0.1, 4),
    mobNoise(rng(), 2, 4)
  {}
  Noises(const Noises&) = delete;
};

#endif // _NOISES_H_