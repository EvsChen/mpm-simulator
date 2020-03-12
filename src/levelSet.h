#pragma once

#include "global.h"
#include "ext/Eigen/Geometry"

class Transform {
public:
  static Transform Inverse(const Transform &t);
  Transform();
  Transform(const Vec3f &translate, const Vec3f &rotate, const Vec3f &scale);
  Vec3f tPoint(const Vec3f &pt) const;
  Vec3f tVec(const Vec3f &v) const;
  Vec3f tNorm(const Vec3f &n) const;

  Vec3f translate_, rotate_, scale_;
  Eigen::Affine3f t_;
};

class LevelSet {
public:
  LevelSet(const Transform &t);
  virtual bool sdf(const Vec3f &xi, Float *dist, Vec3f *norm) const;

protected:
  Transform t_, inv_;
};

class Sphere : public LevelSet {
public:
  Sphere(const Transform &t, Float radius);
  virtual bool sdf(const Vec3f &xi, Float *dist, Vec3f *norm) const;

private:
  Float radius_;
};

class Plane : public LevelSet {
public:
  Plane(const Transform &t);
  virtual bool sdf(const Vec3f &xi, Float *dist, Vec3f *norm) const;
};