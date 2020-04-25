#include "particle.h"
#include "plasticity.h"

ParticleList::~ParticleList() {
  delete particles_;
}

void ParticleList::initToSquare() {
  int x0 = 6, y0 = 6, z0 = 6;
  int density = 20;
  int cubeLen = 3;

  for (int x = x0; x < x0 + cubeLen; x++) {
    for (int y = y0; y < y0 + cubeLen; y++) {
      for (int z = z0; z < z0 + cubeLen; z++) {
        for (int i = 0; i < density; i++) {
          Float xc = x + static_cast<Float>(rand()) / RAND_MAX,
                yc = y + static_cast<Float>(rand()) / RAND_MAX,
                zc = z + static_cast<Float>(rand()) / RAND_MAX;
          Vec3f pos; pos << xc, yc, zc;
          pos *= params.spacing;
          (*particles_).push_back(Particle(pos, params.pMass));
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
    p.pos += p.vel * params.timeStep;
  }
}