#include "grid.h"
#include "util.h"

Grid::Grid(int gridX, int gridY, int gridZ, Float space) :
  spacing_(space),
  blocks_(new std::vector<Block>(gridX * gridY * gridZ))
{
  size_ << gridX, gridY, gridZ;
}

Grid::~Grid() {
  delete blocks_;
}

void Grid::parseLevelSets(const std::vector<uPtr<LevelSet>> &levelSets) {
  for (int i = 0; i < (*blocks_).size(); i++) {
    Vec3i idx = getBlockIndex(i);
    Vec3f blockPos = idx.cast<Float>() * spacing_;
    Float minSdf = std::numeric_limits<Float>::max();
    for (const uPtr<LevelSet> &ls : levelSets) {
      Float sdf = ls->sdf(blockPos);
      if (sdf < minSdf) minSdf = sdf;
    }
    (*blocks_)[i].sdf = minSdf;
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

void Grid::trilinearInterp(const Vec3i &base, const Vec3f &frac, Float *sdf, Vec3f *normal) const {
  Float res = 0.f;
  Vec3f resNorm = Vec3f::Constant(0.f);
  Float multp[3][2];
  for (int i = 0; i < 3; i++) {
    multp[i][0] = 1 - frac[i];
    multp[i][1] = frac[i];
  }
  for (int i = 0; i < 8; i++) {
    // Map 8 to binary
    Vec3i idx = base;
    int diffX = i & 1,
        diffY = (i >> 1) & 1,
        diffZ = (i >> 2) & 1;
    idx(0) += diffX;
    idx(1) += diffY;
    idx(2) += diffZ;
    Float s = getBlockAt(idx).sdf;
    res += s * multp[0][diffX] * multp[1][diffY] * multp[2][diffZ];
    // Map (0, 1) to (-1, 1)
    resNorm[0] += s * (diffX * 2 - 1) * multp[1][diffY] * multp[2][diffZ];
    resNorm[1] += s * multp[0][diffX] * (diffY * 2 - 1) * multp[2][diffZ];
    resNorm[2] += s * multp[0][diffX] * multp[1][diffY] * (diffZ * 2 - 1);
  }
  resNorm.normalize();
  *sdf = res;
  *normal = resNorm;
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
    Float sdf;
    Vec3f normal;
    trilinearInterp(base, frac, &sdf, &normal);
    // If sdf > 0, phiHat > 0
    // else if sdf <= 0, phiHat < 0 only if it's "entering" the surface
    Float phiHat = sdf - std::min(block.sdf, 0.f);
    if ((params.collision == CollisionType::SEPARATING && phiHat < 0) ||
        (params.collision == CollisionType::STICKY && phiHat < 0) ||
        (params.collision == CollisionType::SLIPPING && block.sdf < 0))
    {
      // Collided
      Vec3f delV = -phiHat * normal / params.timeStep;
      Vec3f velHat = block.vel + delV;
      // Tangent component
      Vec3f vn = normal * normal.dot(velHat);
      Vec3f vt = velHat - vn;
      Float vtNorm = vt.norm();
      if (params.collision == CollisionType::STICKY && vtNorm <= params.muB * vn.norm()) {
        // Sticky response
        velHat = Vec3f::Constant(0.f);
      } else {
        // Dynamic friction calculation
        Vec3f tangent = vt.normalized();
        velHat -= std::min(vtNorm, params.muB * delV.norm()) * tangent;
      }      
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