#include "SVD.h"
#include "ext/ImplicitQRSVD.h"

PolarResult PolarDecompose(const Mat3f &m) {
  Eigen::JacobiSVD<Mat3f> res(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Mat3f U = res.matrixU(), V = res.matrixV();
  Mat3f R = U * V.transpose();
  Mat3f S = V * res.singularValues().asDiagonal() * V.transpose();
  return PolarResult(R, S);
}

SVDResult SVDDecompose(const Mat3f &m) {
  Eigen::JacobiSVD<Mat3f> res(m, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Mat3f S = res.singularValues().asDiagonal();
  return SVDResult(res.matrixU(), S, res.matrixV());
}

