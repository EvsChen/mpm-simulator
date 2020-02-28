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
  
  /// Check mass of particles & grids, DEBUG only
  void checkMass();

  /// Update grid velocities
  void updateGridState();

  /// Update deformation gradient
  void updateDeformGrad();

  void visualize(const std::string &prefix, int idx);

  Grid grid_;
  ParticleList particleList_;

private:
  /// Calculate grid forces 
  void computeGridForce();
};