#include "levelSet.h"

Transform Transform::Inverse(const Transform &t) {
  Transform invT;
  invT.translate_ = -t.translate_;
  invT.rotate_ = -t.rotate_;
  for (int i = 0; i < 3; i++) {
    invT.scale_(i) = 1.f / t.scale_(i);
  }
  invT.t_ = t.t_.inverse();
  return invT; 
}

Transform::Transform() : Transform(Vec3f::Constant(0.f), Vec3f::Constant(0.f), Vec3f::Constant(1.f))
{}

Transform::Transform(const Vec3f &translate, const Vec3f &rotate, const Vec3f &scale)
  : translate_(translate), rotate_(rotate_), scale_(scale) {
    t_ *= Eigen::Scaling(scale_(0), scale_(1), scale_(2));
    t_ *= Eigen::AngleAxisf(rotate_(0), Vec3f::UnitX());
    t_ *= Eigen::AngleAxisf(rotate_(1), Vec3f::UnitY());
    t_ *= Eigen::AngleAxisf(rotate_(2), Vec3f::UnitZ());
    t_ *= Eigen::Translation3f(translate_(0), translate_(1), translate_(2));
}

Vec3f Transform::tPoint(const Vec3f &pt) const {
  return t_ * pt;
}

Vec3f Transform::tVec(const Vec3f &v) const {
  return t_.linear() * v;
}

Vec3f Transform::tNorm(const Vec3f &n) const {
  Mat3f normM = t_.linear().inverse().transpose();
  return (normM * n).normalized();
}

LevelSet::LevelSet(const Transform &t) : t_(t), inv_(Transform::Inverse(t)){}

Sphere::Sphere(const Transform &t, Float radius) : LevelSet(t), radius_(radius) {}

bool Sphere::sdf(const Vec3f &xi, Float *dist, Vec3f *norm) const {
  Vec3f localP = inv_.tPoint(xi);
  Float len = localP.norm() - radius_;
  *dist = t_.scale_.minCoeff() * len;
  *norm = t_.tNorm(localP);
  return *dist > 0.f;
}

Plane::Plane(const Transform &t) : LevelSet(t) {}

bool Plane::sdf(const Vec3f &xi, Float *dist, Vec3f *norm) const {
  Vec3f localP = inv_.tPoint(xi);
  Float len = localP(2);
  *dist = t_.scale_.minCoeff() * len;
  Vec3f localN; localN << 0.f, 0.f, 1.f;
  *norm = t_.tNorm(localN);
  return *dist > 0.f;
}
