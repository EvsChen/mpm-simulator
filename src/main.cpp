#include "engine.h"
#include "util.h"
#include <cstring>
#include <ctime>

Profiler profiler;
Params params;

int main(int argc, char *argv[]) {
  FLAGS_log_dir = "./";
  google::InitGoogleLogging(argv[0]);
  params.init();
  params.setMaterial(ParticleType::SAND);
  params.log();
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

#ifdef NDEBUG
	engine.writeVelocity("v_" + paddingStr(std::to_string(i), '0', 4) + ".bin");
#endif

    engine.grid_.reset();

#ifdef NDEBUG
    Vec3f particleM = engine.particleList_.calcMomentum();
    DLOG(INFO) << "Momentum difference is: ";
    DLOG(INFO) << (gridM - particleM);
#endif

    engine.particleList_.advection();

#ifdef NDEBUG
    engine.visualize(std::to_string(timer), i);
	engine.writePositions("particles_" + paddingStr(std::to_string(i), '0', 4) + ".bin");
#endif
	

#ifdef PROFILE
    profiler.reportLoop(i);
#endif
    google::FlushLogFiles(google::GLOG_INFO);
  }
#ifdef PROFILE
  profiler.report();
#endif
}