#pragma once

#include "global.h"

class LevelSet {
public:
  LevelSet() {}
  virtual ~LevelSet() {}
  virtual Float sdf(const Vec3f &xi) const = 0;
};

class Sphere : public LevelSet {
public:
  Sphere(const Vec3f &center, Float radius);
  ~Sphere() {}
  virtual Float sdf(const Vec3f &xi) const;

private:
  Vec3f center_;
  Float radius_;
};

class Box : public LevelSet {
public:
  Box(const Vec3f &center, const Vec3f &bound);
  ~Box() {}
  virtual Float sdf(const Vec3f &xi) const;
private:
  Vec3f center_;
  Vec3f bound_;
};
