#include "util.h"

Float cubicKernel(Float x) {
  x = std::abs(x);
  if (x < 1) {
    return 0.5 * x * x * x - x * x + 2.0 / 3.0;
  } else if (x < 2) {
    return 1.0 / 6.0 * (2 - x) * (2 - x) * (2 - x);
  }
  return 0;
}

Float cubicWeight(const Vec3f &pos, const Vec3i &blockPos, Float h) {
  return cubicKernel(pos[0] / h - blockPos[0]) *
         cubicKernel(pos[1] / h - blockPos[1]) *
         cubicKernel(pos[2] / h - blockPos[2]);
}

Vec3f cubicWeightGrad(const Vec3f &pos, const Vec3i &blockPos, Float h) {

}

Vec3i floor(const Vec3f &v) {
  return Vec3i(
    static_cast<int>(std::floor(v[0])),
    static_cast<int>(std::floor(v[1])),
    static_cast<int>(std::floor(v[2]))
  );
}