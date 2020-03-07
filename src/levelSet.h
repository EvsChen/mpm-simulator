#pragma once

#include "global.h"

class Transform {
public:
  Transform(const Vec3f &trans, const Vec3f &rot, const Vec3f &scale);
  Vec3f localPt2World(const Vec3f &p);
  Vec3f localNorm2World(const Vec3f &normal);
  Mat4f T, invT;
};

class LevelSet {
public:
  LevelSet(const Transform &t);
  virtual Float sdf(const Vec3f &xi) const = 0;
  virtual Vec3f normal(const Vec3f &xi) const = 0;

private:
  Transform t_;
};

class Sphere : public LevelSet {
public:
  Sphere(const Transform &t, Float radius);
  virtual Float sdf(const Vec3f &xi) const;
  virtual Vec3f normal(const Vec3f &xi) const;

private:
  Float radius_;
};

class Plane : public LevelSet {
public:
  Plane(const Transform &t);
  virtual Float sdf(const Vec3f &xi) const;
  virtual Vec3f normal(const Vec3f &xi) const;
};