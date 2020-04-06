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

SDF::SDF(const Vec3i & res) : res_(res)
{
	value_.resize(res.x() * res.y() * res.z());
}


Float SDF::sdf(const Vec3f & xi) const
{
	// [TODO] Interpolate
	Vec3i id = (xi / params.spacing).cast<int>();
	return value_[id.z() * res_.x() * res_.y() + id.y() * res_.x() + id.x()];
}

void SDF::setSdf(const Vec3i & xi, Float v)
{
	value_[xi.z() * res_.x() * res_.y() + xi.y() * res_.x() + xi.x()] = v;
}
