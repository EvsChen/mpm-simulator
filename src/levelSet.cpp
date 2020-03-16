#include "levelSet.h"

Transform Transform::Inverse(const Transform &t) {
  Transform invT;
  invT.translate_ = -t.translate_;
  invT.rotate_ = -t.rotate_;
  invT.scale_ = 1.f / t.scale_;
  invT.t_ = Eigen::Translation3f(invT.translate_(0), invT.translate_(1), invT.translate_(2));
  invT.t_ *= Eigen::AngleAxisf(invT.rotate_(2), Vec3f::UnitZ());
  invT.t_ *= Eigen::AngleAxisf(invT.rotate_(1), Vec3f::UnitY());
  invT.t_ *= Eigen::AngleAxisf(invT.rotate_(0), Vec3f::UnitX());
  invT.t_ *= Eigen::Scaling(invT.scale_);
  return invT; 
}

Transform::Transform() : Transform(Vec3f::Constant(0.f), Vec3f::Constant(0.f), 1.f)
{}

Transform::Transform(const Vec3f &translate, const Vec3f &rotate, Float scale)
  : translate_(translate), rotate_(rotate), scale_(scale) {
    t_ = Eigen::Scaling(scale);
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
  *dist = t_.scale_ * len;
  *norm = t_.tNorm(localP);
  return *dist > 0.f;
}

Plane::Plane(const Transform &t) : LevelSet(t) {}

bool Plane::sdf(const Vec3f &xi, Float *dist, Vec3f *norm) const {
  // Default normal pointing y-plus, infinite large
  Vec3f localP = inv_.tPoint(xi);
  Float len = localP(1);
  *dist = len;
  Vec3f localN; localN << 0.f, 1.f, 0.f;
  *norm = t_.tNorm(localN);
  return *dist > 0.f;
}
