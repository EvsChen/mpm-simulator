#include "engine.h"

#include <fstream>
#include <iostream>

#include "util.h"
#include "constitutiveModel.h"

const static bool USE_QUADRATIC_WEIGHT = true;

Engine::Engine() :
  grid_(params.gridX, params.gridY, params.gridZ, params.spacing), particleList_(params.pType)
{
  int offset = 3;
  Vec3f center; center << params.gridX, params.gridY, params.gridZ;
  center *= params.spacing / 2.f;
  Vec3f bound = center;
  bound -= Vec3f::Constant(offset * params.spacing);
  levelSets.push_back(mkU<Box>(center, bound));
  grid_.parseLevelSets(levelSets);
}

Engine::Engine(const std::vector<Vec3f>& positions)
	: grid_(params.gridX, params.gridY, params.gridZ, params.spacing), particleList_(positions, params.pType)
{
	int offset = 3;
	Vec3f center; center << params.gridX, params.gridY, params.gridZ;
	center *= params.spacing / 2.f;
	Vec3f bound = center;
	bound -= Vec3f::Constant(offset * params.spacing);
	levelSets.push_back(mkU<Box>(center, bound));
	grid_.parseLevelSets(levelSets);
}

Engine::~Engine() {}

void Engine::P2GTransfer() {
  profiler.profStart(ProfType::P2G_TRANSFER);
  for (Particle &p : *(particleList_.particles_)) {
    Vec3f posIdx = p.pos / grid_.spacing_;
    if (USE_QUADRATIC_WEIGHT) {
      Mat3f weight = quadWeight(posIdx);
      Vec3i baseIdx = floor(posIdx - Vec3f::Constant(0.5f));
      for (int offsetX = 0; offsetX < 3; offsetX++) {
        for (int offsetY = 0; offsetY < 3; offsetY++) {
          for (int offsetZ = 0; offsetZ < 3; offsetZ++) {
            Vec3i t; t << offsetX, offsetY, offsetZ;
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
  profiler.profEnd(ProfType::P2G_TRANSFER);
}

void Engine::CHECK_MASS() {
#ifdef MPM_DEBUG
  Float particlesMass = 0.f, gridMass = 0.f;
  for (const Particle &p : (*particleList_.particles_)) {
    particlesMass += p.mass;
  }
  for (int idx : grid_.nonEmptyBlocks_) {
    gridMass += (*grid_.blocks_)[idx].mass;
  }
  LOG(INFO) << "Particle mass: " << particlesMass;
  LOG(INFO) << "Non-empty grid mass: " << gridMass;
#endif
}

void Engine::G2PTransfer() {
  profiler.profStart(ProfType::G2P_TRANSFER);
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
            Vec3i blockPosIdx = baseIdx;
            blockPosIdx += t;
            Block &block = grid_.getBlockAt(blockPosIdx);
            Float w = weight(0, offsetX) * weight(1, offsetY) * weight(2, offsetZ);
            p.vel += w * block.vel;
            Vec3f diffPos = blockPosIdx.cast<Float>() * grid_.spacing_ - p.pos;
            p.Bp += w * block.vel * diffPos.transpose();
          }
        }
      }
    }
  }
  profiler.profEnd(ProfType::G2P_TRANSFER);
}

void Engine::updateGridState() {
  computeGridForce();
  grid_.updateGridVel();
  // grid_.checkBoundaryVel();
}

void Engine::computeGridForce() {
  profiler.profStart(ProfType::CALC_GRID_FORCE);
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
  profiler.profEnd(ProfType::CALC_GRID_FORCE);
}

void Engine::updateDeformGrad() {
  profiler.profStart(ProfType::UPDATE_DEFORM_GRAD);
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
            Vec3i t; t << i, j, k;
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
  profiler.profEnd(ProfType::UPDATE_DEFORM_GRAD);
}

void Engine::visualize(int idx) {
  if (!params.visualize) {
    return;
  }
  profiler.profStart(ProfType::VISUALIZATION);
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
    int gridY = imgSize - p.pos(1) / gridHeight * imgSize;
    int idx = gridX + gridY * imgSize;
    CHECK(gridX < imgSize && gridY < imgSize) << "Invalid grid X: " << gridX << " Y: " << gridY << std::endl;
    output[gridX + gridY * imgSize]++;
    maxParticles = std::max(maxParticles, output[gridX + gridY * imgSize]);
  }
  
  std::ofstream outFile;
  std::string fileName = params.outFolder + "/" + std::to_string(idx) + ".ppm";
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
  profiler.profEnd(ProfType::VISUALIZATION);
}

void Engine::writePositions(const std::string &filename)
{
  if (!params.outputFile) {
    return;
  }
  profiler.profStart(ProfType::OUTPUT_FILE);
	std::ofstream out(filename, std::ios::binary);
	if (!out) {
		throw std::runtime_error("[writePositions] cannot open file");
	}
	int size = particleList_.particles_->size();
	out.write((char*)&size, sizeof(int));
	for (const Particle &p : (*particleList_.particles_)) {
		Vec3f pos = p.pos;
		out.write((char*)&pos.x(), sizeof(Float));
		out.write((char*)&pos.y(), sizeof(Float));
		out.write((char*)&pos.z(), sizeof(Float));
	}
	out.close();
  profiler.profEnd(ProfType::OUTPUT_FILE);
}

void Engine::writeVelocity(const std::string & filename)
{
  if (!params.outputFile) {
    return;
  }
  profiler.profStart(ProfType::OUTPUT_FILE);
	std::ofstream out(filename, std::ios::binary);
	if (!out) {
		throw std::runtime_error("[writeVelocity] cannot open file");
	}
	Vec3i size = grid_.size_;
	Float spacing = grid_.spacing_;
	out.write((char*)&size.x(), sizeof(int));
	out.write((char*)&size.y(), sizeof(int));
	out.write((char*)&size.z(), sizeof(int));
	out.write((char*)&spacing, sizeof(Float));
	for (const Block &b : (*grid_.blocks_)) {
		Vec3f vel= b.vel;
		out.write((char*)&vel.x(), sizeof(Float));
		out.write((char*)&vel.y(), sizeof(Float));
		out.write((char*)&vel.z(), sizeof(Float));
	}
	out.close();
  profiler.profEnd(ProfType::OUTPUT_FILE);
}

void Engine::CHECK_PARTICLE_BOUND()
{
	Vec3f bound = grid_.size_.cast<Float>() * grid_.spacing_;
	int count = 0;
	for (Particle& particle : *(particleList_.particles_)) {
		Vec3f pos = particle.pos;
		if (pos.x() > bound.x() || pos.y() > bound.y() || pos.z() > bound.z())
		{
			LOG(INFO) << "Particle mass: " << pos;
			count++;
		}
	}
	LOG(INFO) << "Wrong Paticles Number: " << count;
}
