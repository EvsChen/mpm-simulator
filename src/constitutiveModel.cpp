#include "constitutiveModel.h"
#include "SVD.h"

/// Young's modulus
const static float E = 5.f;
/// Poisson's ratio
const static float nu = 0.2f;
/// Shear modulus
const static float mu = E / 2.f / (1 + nu);
/// Lame's first parameter
const static float lambda = E * nu / (1 + nu) / (1 - 2 * nu);

Mat3f fixedCorotated(const Particle &p) {
  SVDResult res = SVDDecompose(p.F);
  Mat3f R = res.U * res.V.transpose();
  Float J = p.F.determinant();
  return 2.f * mu * (p.F - R) + lambda * (J - 1) * J * p.F.inverse().transpose();
}