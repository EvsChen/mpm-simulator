#pragma once

#include <cassert>
#include <ctime>
#include <cstring>

#include "ext/Eigen/Eigen"
#include <glog/logging.h>

#include "profiler.h"

// Custom float definition
typedef float Float;

typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector3i Vec3i;
typedef Eigen::Vector4f Vec4f;
typedef Eigen::Matrix3f Mat3f;
typedef Eigen::Matrix4f Mat4f;

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#define mkU std::make_unique
#define mkS std::make_shared
#define uPtr std::unique_ptr
#define sPtr std::shared_ptr

#define MPM_DEBUG
// #define PROFILE

extern Profiler profiler;

/// Particle types
enum class ParticleType : int { SNOW, SAND, ELASTIC };

/// Collision type
enum class CollisionType : int { STICKY, SEPARATING, SLIPPING };

/// Global params object
class Params {
public:
  Params() {}

  void setOutput(bool visualize_, bool outputFile_) {
    visualize = visualize_;
    outputFile = outputFile_;
  }

  void setMaterial(ParticleType type) {
    pType = type;
    switch(type) {
      case ParticleType::SAND: {
        E = 3.537e5;
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
    google::FlushLogFiles(google::GLOG_INFO);
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
  Float pMass = 1.f;
  /// Particle density
  Float pDensity;
  /// Particle type 
  ParticleType pType;
  /// Time step
  Float timeStep = 5e-4;
  /// Step size
  int stepSize = 500;
  /// Grid size
  int gridX = 30, gridY = 30, gridZ = 30;
  /// Grid spacing
  Float spacing = 1e-2;
  /// Collision status 
  CollisionType collision = CollisionType::SEPARATING;
  /// Friction Coefficient
  Float muB = 0.2f;
  /// Whether output simple visualization
  bool visualize = true;
  /// Whether output position and velocity bin file
  bool outputFile = true;
  /// Output folder name
  std::string outFolder = std::to_string(std::time(0));
};

extern Params params;