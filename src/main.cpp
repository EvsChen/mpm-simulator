#include "engine.h"
#include <cstring>
#include <ctime>

Profiler profiler;
Params params;

int main(int argc, char *argv[]) {
  FLAGS_log_dir = "./";
  google::InitGoogleLogging(argv[0]);
  params.setMaterial(ParticleType::SAND);
  params.setOutput(true, true);
  params.log();
  Engine engine;
  std::time_t timer = std::time(0);
  for (int i = 0; i < params.stepSize; i++) {
    engine.P2GTransfer();
    engine.CHECK_MASS();  

    engine.updateGridState();
    engine.updateDeformGrad();

    engine.particleList_.hardening();
    engine.G2PTransfer();

	  engine.writeVelocity("v_" + paddingStr(std::to_string(i), '0', 4) + ".bin");

    engine.grid_.reset();
    engine.particleList_.advection();

    engine.visualize(std::to_string(timer), i);
	  engine.writePositions("particles_" + paddingStr(std::to_string(i), '0', 4) + ".bin");
    profiler.reportLoop(i);
    google::FlushLogFiles(google::GLOG_INFO);
  }
  profiler.report();
}