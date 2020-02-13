#include "SVD.h"

SVDResult SVDDecompose(const Mat3f &m) {
  Eigen::JacobiSVD<Mat3f> res(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Vec3f singularVals = res.singularValues();
  Mat3f Sigma = Mat3f::Constant(0.f);
  for (int i = 0; i < 3; i++) Sigma(i, i) = singularVals(i);
  Mat3f U = res.matrixU(), V = res.matrixV();
  // Since both U, V are unitary matrices, their determinant are either 1 or -1
  // TODO: Check SVD calculation
  return SVDResult(U, Sigma, V);
}