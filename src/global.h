#pragma once

#include <cassert>

// #include "ext/glm/glm.hpp"
#include "ext/Eigen/Eigen"

// Custom float definition
typedef float Float;

typedef Eigen::Vector3f Vec3f;
typedef Eigen::Vector3i Vec3i;
typedef Eigen::Matrix3f Mat3f;

#define mkU std::make_unique
#define mkS std::make_shared
#define uPtr std::unique_ptr
#define sPtr std::shared_ptr

#define NDEBUG

#define GRID_SPACING 1e-2
#define P_MASS 1e-3
#define TIME_STEP 1e-2
#define GRID_SIZE 100