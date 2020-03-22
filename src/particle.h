#pragma once
 
#include "global.h"
#include <vector>


// TODO: Come up with iterator function?
struct Particle {
  Particle(const Vec3f &p, Float m) :
    originalPos(p), pos(p), mass(m), volume(m / params.pDensity) {};

  Particle() {}
  /// Original position
  Vec3f originalPos;
  Vec3f pos = Vec3f::Constant(0.f);
  Float mass = 1.0;
  Float volume = 1.0;
  Vec3f vel = Vec3f::Constant(0.f);
  /// The APIC Bp matrix
  Mat3f Bp = Mat3f::Constant(0.f);
  /// Deformation gradient
  Mat3f F = Mat3f::Identity();
  /// Elastic part of F
  Mat3f Fe = Mat3f::Identity();
  /// Plastic part of F
  Mat3f Fp = Mat3f::Identity();
  // TODO: alpha initialization?
  /// Yield surface size, used in plasticity hardening
  Float alpha = 0.f;
  /// Hardening state, used in plasticity hardening
  Float q = 0.f;
};

class ParticleList {
public:
	ParticleList() : particles_(new std::vector<Particle>()) {}
  ~ParticleList();
  void initToSquare();
  
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

  Vec3f calcMomentum() const;

  /// Update particle velocity
  void advection();

  void hardening();
  
  /// List of unique pointer to particles 
  std::vector<Particle> *particles_;
  ParticleType type_;
};

