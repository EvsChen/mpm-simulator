#pragma once

#include <cassert>

// TODO: Change eigen build to cmake
#include "ext/Eigen/Eigen"
#include <glog/logging.h>

#include "profiler.h"

// Custom float definition
typedef float Float;

typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector3i Vec3i;
typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Matrix4f Mat4f;

#define mkU std::make_unique
#define mkS std::make_shared
#define uPtr std::unique_ptr
#define sPtr std::shared_ptr

#define NDEBUG
// #define PROFILE

extern Profiler profiler;
// TODO: Pass in constants variables

/// Particle types
enum class ParticleType : int {
  SNOW, SAND, ELASTIC
};

/// Global params object
class Params {
public:
  Params() {}
  void init(Float pMass_ = 1.f,
            Float timeStep_ = 5e-4, int stepSize_ = 1000, int gridX_ = 20, int gridY_ = 20, int gridZ_ = 20, Float spacing_ = 1e-2) {
    pMass = pMass_;
    timeStep = timeStep_;
    stepSize = stepSize_;
    gridX = gridX_;
    gridY = gridY_;
    gridZ = gridZ_;
    spacing = spacing_;
  }
  void setMaterial(ParticleType type) {
    pType = type;
    switch(type) {
      case ParticleType::SAND: {
        E = 3.537e5;
        // E = 50.f;
        nu = 0.3f;
        pDensity = 2.2e3f;
        break;
      }
      case ParticleType::ELASTIC: {
        E = 5.f;
        nu = 0.2f;
        pDensity = 1e3f;
        break;
      }
      default:
        break;
    }
    mu = E / 2.f / (1 + nu);
    lambda = E * nu / (1 + nu) / (1 - 2 * nu);
  }

  void log() {
    LOG(INFO) << "Particle type: " << (int) pType;
    LOG(INFO) << "Grid size: " << gridX << " * " << gridY << " * " << gridZ << " * " << spacing;
    LOG(INFO) << "Time Step: " << timeStep << " * " << stepSize;
    LOG(INFO) << "Particle mass: " << pMass << " Density: " << pDensity;
    LOG(INFO) << "E: " << E << " " << "nu: " << nu; 
    google::FlushLogFiles(google::INFO);
  }

  /// Young's modulus
  Float E;
  /// Poisson's ratio
  Float nu;
  /// Shear modulus
  Float mu;
  /// Lame's first parameter
  Float lambda;
  /// Particle mass
  Float pMass;
  /// Particle density
  Float pDensity;
  /// Particle type 
  ParticleType pType;
  /// Time step
  Float timeStep;
  /// Step size
  int stepSize;
  /// Grid size
  int gridX, gridY, gridZ;
  /// Grid spacing
  Float spacing;
};

extern Params params;