#include "engine.h"

#include "util.h"

const static bool USE_QUADRATIC_WEIGHT = true;

Engine::Engine() :
  grid_(), particleList_()
{}

Engine::~Engine() {}

void Engine::P2GTransfer() {
  for (const Particle &p : *(particleList_.particles_)) {
    Vec3f posIdx = p.pos / grid_.spacing_;

    if (USE_QUADRATIC_WEIGHT) {
      Mat3f weight = quadWeight(posIdx);
      Vec3i baseIdx = floor(posIdx - Vec3f::Constant(0.5f));
      for (int offsetX = 0; offsetX < 3; offsetX++) {
        for (int offsetY = 0; offsetY < 3; offsetY++) {
          for (int offsetZ = 0; offsetZ < 3; offsetZ++) {
            Vec3i t;
            t << offsetX, offsetY, offsetZ;
            Vec3i blockPosIdx = baseIdx + t;
            Block &block = grid_.getBlockAt(blockPosIdx);
            float w = weight(0, offsetX) * weight(1, offsetY) * weight(2, offsetZ);

            block.mass += w * p.mass;
            // TODO: Check affine term calculation
            Vec3f affineTerm = 4.f * p.Bp * (blockPosIdx.cast<Float>() - posIdx);
            block.vel += w * p.mass * (p.vel + affineTerm);
          }
        }
      }
    }
  }

  for (int i = 0; i < (*grid_.blocks_).size(); i++) {
    Block &block = (*grid_.blocks_)[i];
    if (block.mass != 0.f) {
      block.vel /= block.mass;
      // TODO: Cleanup nonEmptyBlocks_
      grid_.nonEmptyBlocks_.push_back(i);
    } else {
      block.mass = 0.f;
      block.vel = Vec3f::Constant(0.f);
    }
  }
}