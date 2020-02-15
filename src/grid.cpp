#include "grid.h"

Grid::Grid(const Vec3i &s, Float space) :
  spacing_(space), size_(s),
  blocks_(new std::vector<Block>(s[0] * s[1] * s[2]))
{}

Grid::Grid() : Grid(Vec3i::Constant(GRID_SIZE), GRID_SPACING) {}

void Grid::addExternalForces() {
  Vec3f g;
  g << 0, 0, 9.8f;
  for (int idx : nonEmptyBlocks_) {
    Block &block = (*blocks_)[idx];
    block.f += block.mass * g;
  }
}

void Grid::updateGridVel() {
  for (int idx : nonEmptyBlocks_) {
    Block &block = (*blocks_)[idx];
    block.vel += block.f * TIME_STEP / block.mass;
  }
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