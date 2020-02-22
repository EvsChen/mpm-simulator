#pragma once

#include <cstring>

#include "global.h"
#include "particle.h"
#include "grid.h"

class Engine {
public:
  Engine();
  ~Engine();
  /// Transfer the mass and velocity from particles to grid using APIC
  void P2GTransfer();

  /// Transfer from grid to particles
  void G2PTransfer();
  
  /// Check mass of particles & grids
  /// Only for debug purpose
  void checkMass();

  /// Calculate grid forces 
  void computeGridForce();

  void updateDeformGrad();

  void visualize(const std::string &prefix, int idx);

  Grid grid_;
  ParticleList particleList_;
};