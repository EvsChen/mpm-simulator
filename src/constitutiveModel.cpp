#include "constitutiveModel.h"

/// Young's modulus
const static float E = 50.f;
/// Poisson's ratio
const static float vu = 0.2;
/// Shear modulus
const static float mu = E / 2.f / (1 + vu);
/// Lame's first parameter
const static float lambda = E * vu / (1 + vu) / (1 - 2 * vu);

Mat3f fixedCorotated(Particle *p) {
  return Mat3f::Constant(1.f);
}