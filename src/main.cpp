#include "engine.h"
#include <cstring>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
  #include "direct.h"
#else
  #include <sys/stat.h>
#endif

Profiler profiler;
Params params;

int main(int argc, char *argv[]) {
  #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
    _mkdir(params.outFolder.c_str());
  #else
    mkdir(params.outFolder.c_str(), 0777);
  #endif
  FLAGS_log_dir = params.outFolder;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  params.setMaterial(ParticleType::SAND);
  params.setOutput(true, false);
  params.log();
  Engine engine;
  
  for (int i = 0; i < params.stepSize; i++) {
#ifdef MPM_DEBUG
    LOG(INFO) << "Start loop " << i;
#endif
    engine.P2GTransfer();
    // engine.CHECK_MASS();  

    engine.updateGridState();
    engine.updateDeformGrad();

    engine.particleList_.hardening();
    engine.G2PTransfer();

	  engine.writeVelocity(params.outFolder + "/" + "v_" + paddingStr(std::to_string(i), '0', 4) + ".bin");

    engine.grid_.reset();
    engine.particleList_.advection();

    engine.visualize(i);
	  engine.writePositions(params.outFolder + "/" + "particles_" + paddingStr(std::to_string(i), '0', 4) + ".bin");
    profiler.reportLoop(i);
    google::FlushLogFiles(google::GLOG_INFO);
  }
  profiler.report();
}