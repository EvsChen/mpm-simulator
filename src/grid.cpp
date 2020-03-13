#include "grid.h"

Grid::Grid(int gridX, int gridY, int gridZ, Float space) :
  spacing_(space),
  blocks_(new std::vector<Block>(gridX * gridY * gridZ))
{
  size_ << gridX, gridY, gridZ;
}

void Grid::parseLevelSets(const std::vector<uPtr<LevelSet>> &levelSets) {
  for (int i = 0; i < (*blocks_).size(); i++) {
    Vec3i idx = getBlockIndex(i);
    Vec3f blockPos = idx.cast<Float>() * spacing_;
    Float sdf, minSdf = std::numeric_limits<Float>::max();
    Vec3f norm, minNorm;
    for (const uPtr<LevelSet> &ls : levelSets) {
      ls->sdf(blockPos, &sdf, &norm);
      if (sdf < minSdf) {
        minSdf = sdf;
        minNorm = norm;
      }
    }
    Block &block = (*blocks_)[i];
    block.sdf = minSdf;
    block.sdfNorm = minNorm;
  }
}

Vec3f Grid::calcMomentum() const {
  Vec3f momentum = Vec3f::Constant(0.f);
  for (int idx : nonEmptyBlocks_) {
    const Block &block = (*blocks_)[idx];
    momentum += block.vel * block.mass;
  }
  return momentum;
}

void Grid::updateGridVel() {
  for (int idx : nonEmptyBlocks_) {
    Block &block = (*blocks_)[idx];
    block.vel += block.f * params.timeStep / block.mass;
  }
}

void Grid::reset() {
  for (int idx : nonEmptyBlocks_) {
    Block &block = (*blocks_)[idx];
    block.mass = 0.f;
    block.vel = Vec3f::Constant(0.f);
    block.f = Vec3f::Constant(0.f);
  }
  nonEmptyBlocks_.clear();
}

void Grid::checkBoundaryVel() {
  // Wall thickness
  int thickness = 2;
  int xMax = size_[0], yMax = size_[1], zMax = size_[2];

  for (int j = 0; j < yMax; j++) {
    for (int k = 0; k < zMax; k++) {
      for (int i = 0; i < thickness; i++) {
        int idx = i + j * xMax + k * xMax * yMax;
        Block &block = (*blocks_)[idx];
        if (block.vel[0] < 0) block.vel[0] = 0;
      }
      for (int i = 0; i < thickness; i++) {
        int idx = (xMax - 1 - i) + j * xMax + k * xMax * yMax;
        Block &block = (*blocks_)[idx];
        if (block.vel[0] > 0) block.vel[0] = 0;
      }
    }
  }

  for (int i = 0; i < xMax; i++) {
    for (int k = 0; k < zMax; k++) {
      for (int j = 0; j < thickness; j++) {
        int idx = i + j * xMax + k * xMax * yMax;
        Block &block = (*blocks_)[idx];
        if (block.vel[1] < 0) block.vel[1] = 0;
      }
      for (int j = 0; j < thickness; j++) {
        int idx = i + (yMax - 1 - j) * xMax + k * xMax * yMax;
        Block &block = (*blocks_)[idx];
        if (block.vel[1] > 0) block.vel[1] = 0;
      }
    }
  }

  for (int i = 0; i < xMax; i++) {
    for (int j = 0; j < yMax; j++) {
      for (int k = 0; k < thickness; k++) {
        int idx = i + j * xMax + k * xMax * yMax;
        Block &block = (*blocks_)[idx];
        if (block.vel[2] < 0) block.vel[2] = 0;
      }
      for (int k = 0; k < thickness; k++) {
        int idx = i + j * xMax + (zMax - 1 - k) * xMax * yMax;
        Block &block = (*blocks_)[idx];
        if (block.vel[2] > 0) block.vel[2] = 0;
      }
    }
  }
}