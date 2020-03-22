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
   
  /**
   * Iterator function to iterate over nearby 3x3 cells
   * @param posInGrid position in grid coordinate
   * @param updateFunc modify or use the value of block or weight
   */
  template<typename F>
  void iterWeight(const Vec3f &posInGrid, F&& updateFunc);

  /**
   * Iterator function to iterate over nearby 3x3 cells
   * @param posInGrid position in grid coordinate
   * @param updateFunc modify or use the value of block or weight gradient
   */
  template<typename F>
  void iterWeightGrad(const Vec3f &posInGrid, F&& updateFunc);

  /// execute one time step, combines the major functions
  void execOneStep();
  
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

  void CHECK_PARTICLE_BOUND();

  void initGrid(int x, int y, int z, Float spacing);
  void initBoundary(int offset = 3);

  std::vector<Particle>* getParticleVecPointer() {
	  return particleList_.particles_;
  }

  Grid grid_;
  ParticleList particleList_;
  std::vector<uPtr<LevelSet>> levelSets;

private:
  /// Calculate grid forces 
  void computeGridForce();
};