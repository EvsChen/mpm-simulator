#include "engine.h"
#include <cstring>
#include <ctime>

Profiler profiler;
Params params;

int main() {
  params.init();
  params.setMaterial(ParticleType::SAND);
  Engine engine;
  std::time_t timer = std::time(0);
  for (int i = 0; i < params.stepSize; i++) {
    engine.P2GTransfer();

#ifdef NDEBUG
    engine.checkMass();  
#endif

    engine.updateGridState();
    engine.updateDeformGrad();

#ifdef NDEBUG
    Vec3f gridM = engine.grid_.calcMomentum();
#endif
    engine.particleList_.hardening();
    engine.G2PTransfer();
    engine.grid_.reset();

#ifdef NDEBUG
    Vec3f particleM = engine.particleList_.calcMomentum();
    std::cout << "Momentum difference is: " << std::endl;
    std::cout << (gridM - particleM) << std::endl;
#endif
    engine.particleList_.advection();

    engine.visualize(std::to_string(timer), i);

#ifdef PROFILE
    profiler.reportLoop(i);
#endif
  }
#ifdef PROFILE
  profiler.report();
#endif
}