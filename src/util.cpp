#include "util.h"

Vec3i floor(const Vec3f &v) {
  Vec3i vi;
  vi << static_cast<int>(std::floor(v[0])),
        static_cast<int>(std::floor(v[1])),
        static_cast<int>(std::floor(v[2]));
  return vi;
}

Mat3f quadWeight(const Vec3f &particlePosIdx) {
  Vec3i basePos = floor(particlePosIdx - Vec3f::Constant(0.5f));
  Mat3f result;
  for (int i = 0; i < 3; i++) {
    Float d = particlePosIdx[i] - basePos[i];
    // 0.5 <= d < 1.5
    result(i, 0) = 0.5f * (1.5f - d) * (1.5f - d); 
    d -= 1.f;
    // -0.5 <= d < 0.5
    result(i, 1) = 0.75f - d * d;
    d -= 1.f;
    // -1.5 <= d < -0.5
    result(i, 2) = 0.5f * (1.5f + d) * (1.5f + d);
  }
  return result;
}

Mat3f quadWeightDeriv(const Vec3f &particlePosIdx) {
  Vec3i basePos = floor(particlePosIdx - Vec3f::Constant(0.5f));
  Mat3f result;
  for (int i = 0; i < 3; i++) {
    Float d = particlePosIdx[i] - basePos[i];
    // 0.5 <= d < 1.5
    result(i, 0) = d - 1.5f;
    d -= 1.f;
    // -0.5 <= d < 0.5
    result(i, 1) = -2.f * d;
    d -= 1.f;
    // -0.5 <= d < 0.5
    result(i, 2) = d + 1.5f;
  }
  return result;
}
