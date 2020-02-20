#include "engine.h"

#include <fstream>
#include <iostream>

#include "util.h"
#include "constitutiveModel.h"

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

void Engine::computeGridForce() {
  for (const Particle &p : (*particleList_.particles_)) {
    Mat3f piola = fixedCorotated(p);
    Vec3f posIdx = p.pos / grid_.spacing_;
    if (USE_QUADRATIC_WEIGHT) {
        Mat3f weight = quadWeight(posIdx);
        Mat3f dweight = quadWeightDeriv(posIdx);
        Vec3i baseIdx = floor(posIdx - Vec3f::Constant(0.5f));
        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
              Vec3f weightGrad;
              weightGrad(0) = dweight(0, i) * weight(1, j) * weight(2, k);
              weightGrad(1) = weight(0, i) * dweight(1, j) * weight(2, k);
              weightGrad(2) = weight(0, i) * weight(1, j) * dweight(2, k);
              weightGrad /= grid_.spacing_;
              Vec3i t;
              t << i, j, k;
              Block &block = grid_.getBlockAt(baseIdx + t);
              block.f += -p.volume * piola * p.F.transpose() * weightGrad;
            }
          }
        }
    }
  }
}

void Engine::updateDeformGrad() {
  for (Particle &p : (*particleList_.particles_)) {
    Vec3f posIdx = p.pos / grid_.spacing_;
    if (USE_QUADRATIC_WEIGHT) {
      Mat3f weight = quadWeight(posIdx);
      Mat3f dweight = quadWeightDeriv(posIdx);
      Vec3i baseIdx = floor(posIdx - Vec3f::Constant(0.5f));
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          for (int k = 0; k < 3; k++) {
            Vec3f weightGrad;
            weightGrad(0) = dweight(0, i) * weight(1, j) * weight(2, k);
            weightGrad(1) = weight(0, i) * dweight(1, j) * weight(2, k);
            weightGrad(2) = weight(0, i) * weight(1, j) * dweight(2, k);
            weightGrad /= grid_.spacing_;
            Vec3i t;
            t << i, j, k;
            Block &block = grid_.getBlockAt(baseIdx + t);
            p.F += TIME_STEP * block.vel * weightGrad. transpose();
          }
        }
      }
    }
  }
}

void Engine::visualize(const std::string &fileName) {
  int imgSize = 400;
  Vec3f black = Vec3f::Constant(0.f);
  Float gridWidth = grid_.size_[0] * grid_.spacing_ ,
        gridHeight = grid_.size_[1] * grid_.spacing_;
  std::vector<int> output(imgSize * imgSize, 0);
  int maxParticles = 0;
  for (const Particle &p : (*particleList_.particles_)) {
    int gridX = p.pos(0) / gridWidth * imgSize;
    int gridY = p.pos(1) / gridHeight * imgSize;
    output[gridX + gridY * imgSize]++;
    maxParticles = std::max(maxParticles, output[gridX + gridY * imgSize]);
  }
  std::ofstream outFile;
  outFile.open(fileName);
  Vec3f baseCol = Vec3f::Constant(255);
  Vec3f maxCol; maxCol << 255, 77, 98;
  if (outFile.is_open()) {
    outFile << "P3\n" << imgSize << " " << imgSize << "\n255\n";
    for (int i = 0; i < imgSize * imgSize; i ++) {
      Float frac = ((Float) output[i] / maxParticles); 
      Vec3f col = frac * maxCol + (1 - frac) * baseCol;
      outFile << (int)col(0) << " " << (int)col(1) << " " << (int)col(2) << std::endl;
    }
    outFile.close();
  } else {
    std::cout << "Open file failed" << std::endl;
  }    
}