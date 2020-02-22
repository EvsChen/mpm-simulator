#include "constitutiveModel.h"
#include "SVD.h"

/// Young's modulus
const static float E = 10.f;
/// Poisson's ratio
const static float vu = 0.3;
/// Shear modulus
const static float mu = E / 2.f / (1 + vu);
/// Lame's first parameter
const static float lambda = E * vu / (1 + vu) / (1 - 2 * vu);

Mat3f fixedCorotated(const Particle &p) {
  SVDResult res = SVDDecompose(p.F);
  Mat3f R = res.U * res.V.transpose();
  Float J = p.F.determinant();
  return 2.f * mu * (p.F - R) + lambda * (J - 1) * J * p.F.inverse().transpose();
}