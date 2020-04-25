#include "plasticity.h"
#include "SVD.h"

Float h0 = 35.f, h1 = 9.f, h2 = 0.2f, h3 = 10.f;

void project(const Mat3f &sigma, Float alpha, Mat3f *T, Float *dq) {
  Mat3f epsilon = sigma;
  for (int i = 0; i < 3; i++) {
    epsilon(i, i) = std::log(epsilon(i, i));
  }
  Float epsilonTr = epsilon.trace();
  Mat3f epsilonHat = epsilon - epsilonTr / 3.f * Mat3f::Identity();
  // ||epsilon||_F: Frobenius norm
  Float epsilonNorm = epsilon.norm();
  Float epsilonHatNorm = epsilonHat.norm();
  if (epsilonHatNorm == 0.f || epsilonTr > 0) {
    *T = Mat3f::Identity();
    *dq = epsilonNorm;
    return;
  }
  Float dGamma = epsilonHatNorm + ((3 * params.lambda) / (2 * params.mu) + 1) * epsilonTr * alpha;
  if (dGamma <= 0.f) {
    *T = sigma;
    *dq = 0.f;
    return;
  }
  Mat3f H = epsilon - dGamma / epsilonHatNorm * epsilonHat;
  for (int i = 0; i < 3; i++) {
    H(i, i) = std::exp(H(i, i));
  }
  *T = H;
  *dq = dGamma;
}

void plasticityHardening(Particle *p) {
  SVDResult res = SVDDecompose(p->Fe);
  Mat3f T;
  Float dq;
  project(res.Sigma, p->alpha, &T, &dq);
  p->Fe = res.U * T * res.V.transpose();
  p->Fp = res.V * T.inverse() * res.Sigma * res.V.transpose() * p->Fp;
  p->q += dq;
  // internal friction angle
  Float phiF = h0 + (h1 * p->q - h3) * std::exp(-h2 * p->q);
  Float sinF = std::sin(phiF * M_PI / 180.f);
  p->alpha = std::sqrt(2.f / 3.f) * 2 * sinF / (3 - sinF);
}

void snowHardening(Particle *p) {
  // Notice here the Fe matrix has been updated
  // assume all the deformation is elastic
  SVDResult res = SVDDecompose(p->Fe);
  for (int i = 0; i < 3; i++) {
    Float s = res.Sigma(i, i);
    // Clamp the value of singular values
    res.Sigma(i, i) = std::min(std::max(s, 1.f - params.thetaC), 1.f + params.thetaS);
  }
  p->Fe = res.U * res.Sigma * res.V.transpose();
  // Get the inverse of the Sigma matrix
  Mat3f invSigma = res.Sigma;
  for (int i = 0; i < 3; i++) {
    invSigma(i, i) = 1.f / invSigma(i, i);
  }
  p->Fp = res.V * invSigma * res.U.transpose() * p->Fe;
}