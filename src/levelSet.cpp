#include "levelSet.h"

Transform::Transform(const Vec3f &trans, const Vec3f &rot, const Vec3f &scale) {
  Mat3f rotX;
  
}

LevelSet::LevelSet(const Transform &t) : t_(t) {}

Sphere::Sphere(const Transform &t, Float radius) : LevelSet(t), radius_(radius) {}

Float Sphere::sdf(const Vec3f &xi) const {

}

