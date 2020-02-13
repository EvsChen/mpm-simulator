#pragma once

#include "global.h"

/**
 * Get the matrix representing the weight of a particle for its
 * surrounding 27 cells.
 * @param particlePosIdx the posIdx (pos / h) of particle
 */
Mat3f quadWeight(const Vec3f &particlePosIdx);

/**
 * Get the matrix representing the derivative of a particle for its
 * surrounding 27 cells.
 * @param particlePosIdx the posIdx (pos / h) of particle
 */
Mat3f quadWeightDeriv(const Vec3f &particlePosIdx);

/**
 * Cast a Vec3f to Vec3i by calculating the floor of each item
 * @param v Vec3f
 */
Vec3i floor(const Vec3f &v);

