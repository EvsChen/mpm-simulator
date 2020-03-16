#pragma once

#include <cstring>
#include <vector>

#include "global.h"
#include "particle.h"
#include "grid.h"
#include "util.h"
#include "levelSet.h"

class Engine {
public:
  Engine();
  ~Engine();
  /// Transfer the mass and velocity from particles to grid using APIC
  void P2GTransfer();

  /// Transfer from grid to particles
  void G2PTransfer();
  
  /// Check mass of particles & grids, DEBUG only
  void CHECK_MASS();

  /// Update grid velocities
  void updateGridState();

  /// Update deformation gradient
  void updateDeformGrad();

  void visualize(int idx);

  /// Write particle positions
  void writePositions(const std::string &filename);

  /// Write grid velocities
  void writeVelocity(const std::string &filename);

  Grid grid_;
  ParticleList particleList_;
  std::vector<uPtr<LevelSet>> levelSets;

private:
  /// Calculate grid forces 
  void computeGridForce();
};