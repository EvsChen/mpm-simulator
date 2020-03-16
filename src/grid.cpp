#include "grid.h"
#include "util.h"

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

template <typename T, typename F>
T Grid::trilinearInterp(const Vec3i &base, const Vec3f &frac, F&& getProp) const {
  T result;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      for (int k = 0; k < 2; k++) {
        Vec3i bIdx; bIdx << base(0) + i, base(1) + j, base(2) + k;
        const Block &neighbor = getBlockAt(bIdx);
        result += getProp(neighbor) * (i == 0 ? 1 - frac(0) : frac(0))
                                    * (j == 0 ? 1 - frac(1) : frac(1))
                                    * (k == 0 ? 1 - frac(2) : frac(2));

      }
    }
  }
  return result;
}

void Grid::updateGridVel() {
  profiler.profStart(ProfType::GRID_VEL_UPDATE);
  Vec3f g; g << 0.f, -9.8f, 0.f;
  Float maxSpeed = 0.5f * spacing_ / params.timeStep;
  for (int idx : nonEmptyBlocks_) {
    Block &block = (*blocks_)[idx];
    // Add external forces    
    block.f += block.mass * g;
    block.vel += block.f * params.timeStep / block.mass;
    // Set max velocity
    block.vel = block.vel.cwiseMin(maxSpeed);
    block.vel = block.vel.cwiseMax(-maxSpeed);
    // Collision detection
    Vec3i blockIdx = getBlockIndex(idx);
    // Block pos in GRID coordinate!
    Vec3f blockPosHat = blockIdx.cast<Float>() + block.vel * params.timeStep / params.spacing;
    Vec3i base = floor(blockPosHat);
    Vec3f frac = blockPosHat - base.cast<Float>();
    Float sdf = trilinearInterp<Float>(base, frac, [](Block b) { return b.sdf; });
    Float phiHat = sdf - std::min(block.sdf, 0.f);
    if ((params.collision == CollisionType::SEPARATING && phiHat < 0) ||
        (params.collision == CollisionType::SLIPPING && block.sdf < 0))
    {
      Vec3f normal = trilinearInterp<Vec3f>(base, frac, [](Block b) { return b.sdfNorm; });
      normal.normalize();
      // Collided
      Vec3f delV = -phiHat * normal / params.timeStep;
      Vec3f velHat = block.vel + delV;
      // Tangent component
      Vec3f vt = velHat - normal * normal.dot(velHat);
      Vec3f tangent = vt.normalized();
      // Friction calculation
      velHat -= std::min(vt.norm(), params.muB * delV.norm()) * tangent;
      block.vel = velHat;
    }
  }
  profiler.profEnd(ProfType::GRID_VEL_UPDATE);
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