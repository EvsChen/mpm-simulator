#pragma once

#include <chrono>
#include <unordered_map>
#include <iostream>

enum class ProfType {
  P2G_TRANSFER, G2P_TRANSFER, VISUALIZATION, CALC_GRID_FORCE, UPDATE_DEFORM_GRAD, PLASTICITY_HARDENING
};

/// Class for profiling
class Profiler {
public:
  using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
  using Duration = std::chrono::milliseconds;

  std::unordered_map<ProfType, std::string> profName = {
    { ProfType::P2G_TRANSFER, "P2G_transfer" },
    { ProfType::G2P_TRANSFER, "G2P_transfer" },
    { ProfType::VISUALIZATION, "Visualization" },
    { ProfType::CALC_GRID_FORCE, "Calc_grid_force" },
    { ProfType::UPDATE_DEFORM_GRAD, "Update_deform_grad" },
    { ProfType::PLASTICITY_HARDENING, "Plasticity_hardening" }
  };

  Profiler() {
    for (auto &p : profName) {
      totalTime_[p.first] = Duration::zero();
      loopTime_[p.first] = Duration::zero();
    }
  }

  void profStart(ProfType type) {
    loopStart_[type] = std::chrono::high_resolution_clock::now();
  }

  void profEnd(ProfType type) {
    loopTime_[type] += std::chrono::duration_cast<Duration>(std::chrono::high_resolution_clock::now() - loopStart_[type]);
  }

  /// Report the time distribution for this loop
  void reportLoop(int idx) {
    std::cout << "Report for loop " << idx << std::endl;
    double totalTime = 0.0;
    for (auto &p : loopTime_) {
      totalTime += p.second.count();
    }
    for (auto &p : loopTime_) {
      std::cout << profName[p.first] << " " << p.second.count() << "ms ";
      std::cout << p.second.count() / totalTime * 100.0 << "%" << std::endl;
      totalTime_[p.first] += p.second;
      p.second = Duration::zero();
    }
  }

  /// Report the overall time distribution
  void report() {
    std::cout << "Report for total" << std::endl;
    double totalTime = 0.0;
    for (auto &p : totalTime_) {
      totalTime += p.second.count();
    }
    for (auto &p : totalTime_) {
      std::cout << profName[p.first] << " " << p.second.count() << "ms ";
      std::cout << p.second.count() / totalTime * 100.0 << "%" << std::endl;
      p.second = Duration::zero();
    }
  }
  
private:
  std::unordered_map<ProfType, Duration> totalTime_, loopTime_;
  std::unordered_map<ProfType, TimePoint> loopStart_;
};