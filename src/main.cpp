#include "engine.h"
#include <cstring>

// TODO: timer

int main() {
  Engine engine;
  Float time = 0.f;
  for (int i = 0; i < 10; i++) {
    engine.P2GTransfer();
    engine.computeGridForce();
    engine.grid_.addExternalForces();
    // TODO: Update grid position
    engine.grid_.updateGridVel();
    engine.grid_.checkBoundaryVel();
    engine.updateDeformGrad();

    engine.visualize("time" + std::to_string(time) + ".ppm");
    time += TIME_STEP;
  }
}