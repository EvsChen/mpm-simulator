#include "SVD.h"

SVDResult SVDDecompose(const Mat3f &m) {
  Eigen::JacobiSVD<Mat3f> res(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vec3f singularVals = res.singularValues();
  Mat3f Sigma = Mat3f::Constant(0.f);
  for (int i = 0; i < 3; i++) Sigma(i, i) = singularVals(i);
  Mat3f U = res.matrixU(), V = res.matrixV();
  // Since both U, V are unitary matrices, their determinant are either 1 or -1
  if (U.determinant() < 0) {
    U(0, 2) *= -1;
    U(1, 2) *= -1;
    U(2, 2) *= -1;
    Sigma(2, 2) *= -1;
  }
  if (V.determinant() < 0) {
    V(0, 2) *= -1;
    V(1, 2) *= -1;
    V(2, 2) *= -1;
    Sigma(2, 2) *= -1;
  }
  if (Sigma(0, 0) < Sigma(1, 1)) {
    std::swap(Sigma(0, 0), Sigma(1, 1));
  }
  return SVDResult(U, Sigma, V);
}