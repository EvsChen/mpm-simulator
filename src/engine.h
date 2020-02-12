#pragma once

#include "global.h"
#include "particle.h"
#include "grid.h"

class Engine {
public:
  Engine();
  ~Engine();
  /**
   * Transfer the mass and velocity from particles to grid using APIC
   */
  void P2GTransfer();

  Grid grid_;
  ParticleList particleList_;
};