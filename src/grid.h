#pragma once

#include <array>
#include <stdexcept>
#include <vector>
#include <unordered_set>
#include <iostream>

#include "global.h"
#include "levelSet.h"

struct Block {
  Float mass = 0.f;
  Vec3f vel = Vec3f::Constant(0.f);
  /// Block force
  Vec3f f = Vec3f::Constant(0.f);
  /// Level set sdf
  Float sdf;
  /// Level set normal
  Vec3f sdfNorm;
};

class Grid {
public:
  Grid(int, int, int, Float);
  /// Default constructor
  Grid() {}
  ~Grid();

  void checkBoundaryVel();
  
  /// Take in a list of level sets and compute the sdf and normal at grid nodes.
  /// The values in-between will be interpolated
  void parseLevelSets(const std::vector<uPtr<LevelSet>> &levelSets);

  /// Update grid velocity
  void updateGridVel();

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

  Vec3i getBlockIndex(int idx) const {
    Vec3i i;
    int z = idx / (size_[0] * size_[1]);
    int xy = idx % (size_[0] * size_[1]);
    int y = xy / size_[0];
    int x = xy % size_[0];
    i << x, y, z;
    return i;
  }

  int getBlockOffset(const Vec3i &idx) const {
    CHECK(isValidIdx(idx)) << "getBlockAt idx out of range: " << idx[0] << " " << idx[1] << " " << idx[2];
    return idx[0] + idx[1] * size_[0] + idx[2] * size_[0] * size_[1];
  }

  /// Get sdf and normal at a point
  void trilinearInterp(const Vec3i &base, const Vec3f &frac, Float *sdf, Vec3f *normal) const;

  Block &getBlockAt(const Vec3i &idx) {
    return (*blocks_)[getBlockOffset(idx)];
  }

  const Block &getBlockAt(const Vec3i &idx) const {
    return (*blocks_).at(getBlockOffset(idx));
  }

  Float spacing_;
  Vec3i size_;
  std::vector<Block>* blocks_;
  /// The node with non-zero mass
  std::unordered_set<int> nonEmptyBlocks_;
};
