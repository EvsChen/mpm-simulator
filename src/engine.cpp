#include "engine.h"

#include <fstream>
#include <iostream>

#include "util.h"
#include "constitutiveModel.h"

const static bool USE_QUADRATIC_WEIGHT = true;

Engine::Engine() :
  grid_(params.gridX, params.gridY, params.gridZ, params.spacing), particleList_(params.pType)
{}

Engine::~Engine() {}

void Engine::P2GTransfer() {
#ifdef PROFILE
  profiler.profStart(ProfType::P2G_TRANSFER);
#endif
  for (Particle &p : *(particleList_.particles_)) {
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
            Float w = weight(0, offsetX) * weight(1, offsetY) * weight(2, offsetZ);

            block.mass += w * p.mass;
            Vec3f affineTerm = 4.f * p.Bp / grid_.spacing_ * (blockPosIdx.cast<Float>() - posIdx);
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
      grid_.nonEmptyBlocks_.insert(i);
    } else {
      block.mass = 0.f;
      block.vel = Vec3f::Constant(0.f);
    }
  }
#ifdef PROFILE
  profiler.profEnd(ProfType::P2G_TRANSFER);
#endif
}

void Engine::checkMass() {
  Float particlesMass = 0.f, gridMass = 0.f;
  for (const Particle &p : (*particleList_.particles_)) {
    particlesMass += p.mass;
  }
  for (int idx : grid_.nonEmptyBlocks_) {
    gridMass += (*grid_.blocks_)[idx].mass;
  }
  DLOG(INFO) << "Particle mass: " << particlesMass;
  DLOG(INFO) << "Non-empty grid mass: " << gridMass;
}

void Engine::G2PTransfer() {
#ifdef PROFILE
  profiler.profStart(ProfType::G2P_TRANSFER);
#endif
  for (Particle &p : *(particleList_.particles_)) {
    Vec3f posIdx = p.pos / grid_.spacing_;
    p.vel = Vec3f::Constant(0.f);
    p.Bp = Mat3f::Constant(0.f);

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
            Float w = weight(0, offsetX) * weight(1, offsetY) * weight(2, offsetZ);
            p.vel += w * block.vel;
            Vec3f diffPos = blockPosIdx.cast<Float>() * grid_.spacing_ - p.pos;
            p.Bp += w * block.vel * diffPos.transpose();
          }
        }
      }
    }
#ifdef NDEBUG
    Float maxSpeed = 5.f;
    if (p.vel[0] > maxSpeed || p.vel[1] > maxSpeed || p.vel[2] > maxSpeed) {
      DLOG(WARNING) << "Dangerous particle speed: (" << p.vel[0] << ", " << p.vel[1] << ", " << p.vel[2] << ")";
    } 
#endif
  }
#ifdef PROFILE
  profiler.profEnd(ProfType::G2P_TRANSFER);
#endif
}

void Engine::updateGridState() {
  computeGridForce();
  // Add external forces
  grid_.addExternalForces();
  // TODO: Grid collision and friction
  grid_.updateGridVel();
  grid_.checkBoundaryVel();
}

void Engine::computeGridForce() {
#ifdef PROFILE
  profiler.profStart(ProfType::CALC_GRID_FORCE);
#endif
  for (const Particle &p : (*particleList_.particles_)) {
    Mat3f Ap;
    switch (particleList_.type_) {
      case ParticleType::SAND: {
        // Only use the elastic part
        Mat3f piola = stVenant(p.Fe, false);
        // Mat3f piola = fixedCorotated(p.Fe);
        Ap = p.volume * piola * p.Fe.transpose();
        break;
      }
      case ParticleType::ELASTIC: {
        Mat3f piola = fixedCorotated(p.F);
        Ap = p.volume * piola * p.F.transpose();
        break;
      }
      default:
        LOG(FATAL) << "Particle type not specified!" << std::endl;
        break;
    }

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
              block.f += - Ap * weightGrad;
            }
          }
        }
    }
  }
#ifdef PROFILE
  profiler.profEnd(ProfType::CALC_GRID_FORCE);
#endif
}

void Engine::updateDeformGrad() {
#ifdef PROFILE
  profiler.profStart(ProfType::UPDATE_DEFORM_GRAD);
#endif
  for (Particle &p : (*particleList_.particles_)) {
    Vec3f posIdx = p.pos / grid_.spacing_;
    Mat3f updateF = Mat3f::Identity();
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
            updateF += params.timeStep * block.vel * weightGrad.transpose();
          }
        }
      }
    }
    if (particleList_.type_ == ParticleType::ELASTIC) {
      p.F = updateF * p.F;  
    } else if (particleList_.type_ == ParticleType::SAND) {
      p.Fe = updateF * p.Fe;
    }  
  }
#ifdef PROFILE
  profiler.profEnd(ProfType::UPDATE_DEFORM_GRAD);
#endif
}

void Engine::visualize(const std::string &prefix, int idx) {
#ifdef PROFILE
  profiler.profStart(ProfType::VISUALIZATION);
#endif
  int imgSize = 400;
  std::vector<int> output(imgSize * imgSize, 0);
  Vec3f baseCol = Vec3f::Constant(255);
  Vec3f maxCol; maxCol << 255, 38, 6;
  Vec3i gridCol; gridCol << 39, 120, 153;
  Float gridWidth = grid_.size_[0] * grid_.spacing_ ,
        gridHeight = grid_.size_[1] * grid_.spacing_;
  int imgGridWidth = imgSize / grid_.size_[0],
      imgGridHeight = imgSize / grid_.size_[1];
  
  // Count particles in each grid
  int maxParticles = 0;
  for (const Particle &p : (*particleList_.particles_)) {
    int gridX = p.pos(0) / gridWidth * imgSize;
    int gridY = p.pos(1) / gridHeight * imgSize;
    int idx = gridX + gridY * imgSize;
#ifdef NDEBUG    
    if (gridX >= imgSize || gridY >= imgSize) {
      DLOG(FATAL) << "Invalid grid X: " << gridX << " Y: " << gridY << std::endl;
      continue;
    }
#endif
    output[gridX + gridY * imgSize]++;
    maxParticles = std::max(maxParticles, output[gridX + gridY * imgSize]);
  }
  
  std::ofstream outFile;
  std::string fileName = prefix + "_" + std::to_string(idx) + ".ppm";
  outFile.open(fileName);
  if (outFile.is_open()) {
    outFile << "P3\n" << imgSize << " " << imgSize << "\n255\n";
    for (int i = 0; i < imgSize * imgSize; i ++) {
      if (output[i] == 0) {
        // Draw grids
        int pixelX = i % imgSize, pixelY = i / imgSize;
        if (pixelX % imgGridWidth == 0 || pixelY % imgGridHeight == 0) {
          outFile << gridCol[0] << " " << gridCol[1] << " " << gridCol[2] << std::endl;
        } else {
          outFile << baseCol[0] << " " << baseCol[1] << " " << baseCol[2] << std::endl; 
        }
      } else {
        Float frac = ((Float) output[i] / maxParticles); 
        Vec3f col = frac * maxCol + (1 - frac) * baseCol;
        outFile << (int)col(0) << " " << (int)col(1) << " " << (int)col(2) << std::endl;
      }
    }
    outFile.close();
  } else {
    LOG(FATAL) << "Open file failed" << std::endl;
  }    
#ifdef PROFILE
  profiler.profEnd(ProfType::VISUALIZATION);
#endif
}