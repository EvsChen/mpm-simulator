#include "levelSet.h"

Sphere::Sphere(const Vec3f &center, Float radius) : center_(center), radius_(radius) {}

Float Sphere::sdf(const Vec3f &xi) const {
  Vec3f d = xi - center_;
  return d.norm() - radius_;
}

Box::Box(const Vec3f &center, const Vec3f &bound) : center_(center), bound_(bound) {}

Float Box::sdf(const Vec3f &xi) const {
  Vec3f q = (xi - center_).cwiseAbs() - bound_;
  return -(q.cwiseMax(0.f).norm() + std::min(q.maxCoeff(), 0.f));
}
