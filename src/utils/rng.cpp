#include "rng.h"

std::random_device _RD;
std::mt19937 _GEN(_RD());
std::uniform_real_distribution<> _PDF(0, 1);

// returns a random float from 0 to 1
float rand_f() { return _PDF(_GEN); }
