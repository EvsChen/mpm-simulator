#pragma once

#include "global.h"
#include "ext/Eigen/Geometry"

class Transform {
public:
  static Transform Inverse(const Transform &t);
  Transform();
  Transform(const Vec3f &translate, const Vec3f &rotate, Float scale);
  Vec3f tPoint(const Vec3f &pt) const;
  Vec3f tVec(const Vec3f &v) const;
  Vec3f tNorm(const Vec3f &n) const;

  Vec3f translate_, rotate_;
  Float scale_;
  Eigen::Affine3f t_;
};

class LevelSet {
public:
  LevelSet(const Transform &t);
  virtual ~LevelSet() {}
  virtual bool sdf(const Vec3f &xi, Float *dist, Vec3f *norm) const = 0;

protected:
  Transform t_, inv_;
};

class Sphere : public LevelSet {
public:
  Sphere(const Transform &t, Float radius);
  ~Sphere() {}
  virtual bool sdf(const Vec3f &xi, Float *dist, Vec3f *norm) const;

private:
  Float radius_;
};

class Plane : public LevelSet {
public:
  Plane(const Transform &t);
  ~Plane() {}
  virtual bool sdf(const Vec3f &xi, Float *dist, Vec3f *norm) const;
};