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
  [[deprecated]] Float volume = 1.0;
  Vec3f vel = Vec3f::Constant(0.f);
  /// The APIC Bp matrix
  Mat3f Bp = Mat3f::Constant(0.f);
  /// Deformation gradient should use Fe and Fp instead
  [[deprecated]] Mat3f F = Mat3f::Identity();
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

  Vec3f calcMomentum() const;

  /// Update particle velocity
  void advection();
  
  /// List of unique pointer to particles 
  std::vector<Particle> *particles_;
  ParticleType type_;
};

