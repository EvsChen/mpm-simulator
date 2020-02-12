#pragma once
 
#include "global.h"
#include <vector>

struct Particle {
  Particle(const Vec3f &p, Float m) : originalPos(p), pos(p), mass(m) {};
  /// Original position
  Vec3f originalPos;
  Vec3f pos;
  Float mass;
  Float volume;
  Vec3f vel = Vec3f::Constant(0.f);
  /// The APIC Bp matrix
  Mat3f Bp = Mat3f::Constant(0.f);
  /// Deformation gradient
  Mat3f F = Mat3f::Constant(1.f);
  /// Elastic part of F
  Mat3f Fe = Mat3f::Constant(1.f);
  /// Plastic part of F
  Mat3f Fp = Mat3f::Constant(1.f);
};

class ParticleList {
public:
  ParticleList();
  /**
   * Get grid force to transfer to grid
   * @param idx grid index
   */
  Vec3f getGridForce(Vec3i idx) const;
  /**
   * Update deformation gradient
   * @param idx grid index
   */
  Vec3f updateDeformGrad(Vec3i idx);
  void collideWithBody();
  
  /// List of unique pointer to particles 
  std::vector<Particle> *particles_;
};

