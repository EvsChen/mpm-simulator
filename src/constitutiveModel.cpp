#include "constitutiveModel.h"
#include "SVD.h"

Mat3f fixedCorotated(const Mat3f &F) {
  SVDResult res = SVDDecompose(F);
  Mat3f R = res.U * res.V.transpose();
  Float J = F.determinant();
  return 2.f * params.mu * (F - R) + params.lambda * (J - 1) * J * F.inverse().transpose();
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