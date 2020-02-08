#include "engine.h"
#include "util.h"

void Engine::P2GTransfer() {
  for (int i = 0; i < particleList_.particles_.size(); i++) {
    Particle *p = particleList_.particles_[i].get();
    Vec3f posOverSpacing = p->pos / grid_->spacing_;
    Vec3i particleGridPos = floor(posOverSpacing);

    for (int offsetX = -1; offsetX <= 1; offsetX++) {
      for (int offsetY = -1; offsetY <= 1; offsetY++) {
        for (int offsetZ = -1; offsetZ <= 1; offsetZ++) {
          Vec3i blockPos = particleGridPos + Vec3i(offsetX, offsetY, offsetZ);
          // TODO: Optimize weight calculation
          Float w = cubicWeight(p->pos, blockPos, grid_->spacing_);
          Block *block = grid_->getBlockAt(blockPos);
          block->mass += w * p->mass;
          Vec3f affineTerm = 4.f * (p->affine * (Vec3f(blockPos) - posOverSpacing));
          block->vel += w * p->mass * (p->vel + affineTerm);
        }
      }
    }
  }
  for (int i = 0; i < grid_->blocks_.size(); i++) {
    const uPtr<Block> &blockPtr= grid_->blocks_[i];
    if (blockPtr->mass == 0.f) {
      blockPtr->vel /= blockPtr->mass;
      // TODO: Cleanup nonEmptyBlocks_
      grid_->nonEmptyBlocks_.push_back(i);
    } else {
      blockPtr->mass = 0.f;
      blockPtr->vel = Vec3f(0.f);
    }
  }
}