#include "constitutiveModel.h"
#include "SVD.h"

/// Hardening coefficient
const float xi = 15.f;

Mat3f fixedCorotated(const Mat3f &F) {
  PolarResult res = PolarDecompose(F);
  Mat3f R = res.R;
  Float J = F.determinant();
  return 2.f * params.mu * (F - R) + params.lambda * (J - 1) * J * F.inverse().transpose();
}

Mat3f fixedCorotatedSnow(const Mat3f &Fe, const Mat3f &Fp) {
  PolarResult res = PolarDecompose(Fe);
  Mat3f Re = res.R;
  Float Je = Fe.determinant(),
        Jp = Fp.determinant();
  // Hardening
  Float hard = std::exp(xi * (1 - Jp));
  Float mu = params.mu * hard,
        lambda = params.lambda * hard;
  return 2.f * mu * (Fe - Re) + lambda * (Je - 1) * Je * Fe.inverse().transpose();
}

Mat3f stVenant(const Mat3f &Fe, bool needProjected) {
  SVDResult res = SVDDecompose(Fe);
  Mat3f lnSigma = res.Sigma;
  for (int i = 0; i < 3; i++) {
    lnSigma(i, i) = std::log(lnSigma(i, i));
  }
  Mat3f invSigma = res.Sigma.inverse();
  // Energy derivative
  Mat3f T = 2 * params.mu * invSigma * lnSigma + params.lambda * lnSigma.trace() * invSigma;
  return res.U * T * res.V.transpose();
}