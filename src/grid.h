#pragma once

#include <array>
#include <stdexcept>
#include <vector>
#include <unordered_set>
#include <iostream>

#include "global.h"

struct Block {
  Float mass = 0.0;
  Vec3f vel = Vec3f::Constant(0.f);
  /// Block force
  Vec3f f = Vec3f::Constant(0.f);
};

class Grid {
public:
  Grid(int, int, int, Float);
  /// Default constructor
  Grid();
  ~Grid() {}

  void checkBoundaryVel();

  /// Add external forces
  void addExternalForces() {
    Vec3f g;
    g << 0, 9.8f, 0;
    for (int idx : nonEmptyBlocks_) {
      Block &block = (*blocks_)[idx];
      block.f += block.mass * g;
    }
  }
  /// Update grid velocity
  void updateGridVel() {
    for (int idx : nonEmptyBlocks_) {
      Block &block = (*blocks_)[idx];
      block.vel += block.f * params.timeStep / block.mass;
    }
  }

  void collideWithBody();

  Vec3f calcMomentum() const;

  /// Clear grid force, mass and velocity
  void reset();

  /**
   * Check whether a given index is valid
   * This function should be called before using getBlockAt
   * @param idx Block index
   */
  bool isValidIdx(const Vec3i &idx) const {
    return idx[0] < size_[0] && idx[0] >= 0 &&
           idx[1] < size_[1] && idx[1] >= 0 &&
           idx[2] < size_[2] && idx[2] >= 0;
  }

  /**
   * Get block pointer at idx
   * @param idx block index
   */
  Block &getBlockAt(const Vec3i &idx) {
#ifdef NDEBUG
    if (!isValidIdx(idx)) {
      std::cerr << "Idx: " << idx << std::endl;
      throw std::invalid_argument("Index of out range");
    }
#endif
    return (*blocks_)[idx[0] + idx[1] * size_[0] + idx[2] * size_[0] * size_[1]];
  }

  Float spacing_;
  Vec3i size_;
  std::vector<Block>* blocks_;
  /// The node with non-zero mass
  std::unordered_set<int> nonEmptyBlocks_;
};
