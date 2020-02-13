#pragma once

#include "global.h"
#include "particle.h"

/**
 * Fixed corotated model, refer to mpm2016course p20
 * @param p particle to be calculated
 * @return piola-kirchoff stress
 */
Mat3f fixedCorotated(const Particle &p);