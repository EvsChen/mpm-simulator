#include "particle.h"

void ParticleList::init() {
  int x0 = 47, y0 = 47, z0 = 47;
  int density = 10;
  int cubeLen = 6;
  Float mass = 1e-3;
  for (int x = x0; x < x0 + cubeLen; x++) {
    for (int y = y0; y < y0 + cubeLen; y++) {
      for (int z = z0; z < z0 + cubeLen; z++) {
        for (int i = 0; i < density; i++) {
          Float xc = x0 + static_cast<Float>(rand()) / RAND_MAX,
                yc = y0 + static_cast<Float>(rand()) / RAND_MAX,
                zc = z0 + static_cast<Float>(rand()) / RAND_MAX;
          particles_.push_back(mkU<Particle>(Vec3f(xc, yc, zc), mass));
        }
      }
    }
  }
}