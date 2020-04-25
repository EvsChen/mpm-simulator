#pragma once

#include "global.h"
#include "particle.h"

/**
 * Fixed corotated model, refer to mpm2016course p20
 * @param F Deformation gradient
 * @return piola-kirchoff stress
 */
Mat3f fixedCorotated(const Mat3f &F);

/**
 * Fixed corotated model for snow with hardening effect, refer to mpm2016course p20
 * @param Fe Elastic deformation gradient
 * @param Fp Plastic deformation gradient
 * @return piola-kirchoff stress
 */
Mat3f fixedCorotatedSnow(const Mat3f &Fe, const Mat3f &Fp);


/**
 * Calculate sand stress using St.Venant model, refer to drucker2016 tech doc
 * @param Fe elastic deformation gradient
 * @param needProjected used in implicit integration
 * @return piola-kirschoff stress
 */
Mat3f stVenant(const Mat3f &Fe, bool needProjected);