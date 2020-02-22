#include "engine.h"
#include <cstring>
#include <ctime>

// TODO: timer

int main() {
  Engine engine;
  Float time = 0.f;
  std::time_t timer = std::time(0);

  for (int i = 0; i < 300; i++) {
    engine.P2GTransfer();
#ifdef NDEBUG
    engine.checkMass();  
#endif
    engine.computeGridForce();
    engine.grid_.addExternalForces();
    engine.grid_.updateGridVel();
    engine.grid_.checkBoundaryVel();
    engine.updateDeformGrad();
#ifdef NDEBUG
    Vec3f gridM = engine.grid_.calcMomentum();
#endif
    engine.G2PTransfer();
    engine.grid_.reset();
#ifdef NDEBUG
    Vec3f particleM = engine.particleList_.calcMomentum();
    std::cout << "Momentum difference is: " << std::endl;
    std::cout << (gridM - particleM) << std::endl;
#endif
    engine.particleList_.advection();

    engine.visualize(std::to_string(timer), i);
    time += TIME_STEP;
  }
}