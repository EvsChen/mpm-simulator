#include "global.h"

/**
 * Calculate the weight function of a particle on a block position
 * @param pos Particle position
 * @param blockPos Block index
 * @param h Grid spacing
 */
Float cubicWeight(const Vec3f &pos, const Vec3i &blockPos, Float h);

/**
 * Calculate the grdient of weight function of a particle on a block position
 * @param pos Particle position
 * @param blockPos Block index
 * @param h Grid spacing
 */
Vec3f cubicWeightGrad(const Vec3f &pos, const Vec3i &blockPos, Float h);

/**
 * Cast a Vec3f to Vec3i by calculating the floor of each item
 * @param v Vec3f
 */
Vec3i floor(const Vec3f &v);

