#include "particle.h"

ParticleList::ParticleList() {
  int x0 = 7, y0 = 10, z0 = 7;
  int density = 10;
  int cubeLen = 6;
  particles_ = new std::vector<Particle>();

  for (int x = x0; x < x0 + cubeLen; x++) {
    for (int y = y0; y < y0 + cubeLen; y++) {
      for (int z = z0; z < z0 + cubeLen; z++) {
        for (int i = 0; i < density; i++) {
          Float xc = x + static_cast<Float>(rand()) / RAND_MAX,
                yc = y + static_cast<Float>(rand()) / RAND_MAX,
                zc = z + static_cast<Float>(rand()) / RAND_MAX;
          Vec3f pos; pos << xc, yc, zc;
          pos *= GRID_SPACING;
          (*particles_).push_back(Particle(pos, P_MASS));
        }
      }
    }
  }
}

Vec3f ParticleList::calcMomentum() const {
  Vec3f momentum = Vec3f::Constant(0.f);
  for (const Particle &p : (*particles_)) {
    momentum += p.mass * p.vel;
  }
  return momentum;
}

void ParticleList::advection() {
  for (Particle &p : *particles_) {
    p.pos += p.vel * TIME_STEP;
  }
}