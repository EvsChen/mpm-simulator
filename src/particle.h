#include "global.h"
#include <vector>

struct Particle {
  Particle(Vec3f p, Float m) : originalPos(p), pos(p), mass(m) {};
  /// Original position
  Vec3f originalPos;
  Vec3f pos;
  Float mass;
  Float volume;
  Vec3f vel = Vec3f(0.f);
  /// Deformation Gradient
  Mat3f deformGrad = Mat3f(1.f);
  /// The APIC Bp matrix
  Mat3f affine = Mat3f(0.f);
};

class ParticleList {
public:
  void init();
  /**
   * Get grid force to transfer to grid
   * @param idx grid index
   */
  Vec3f getGridForce(Vec3i idx) const;
  /**
   * Update deformation gradient
   * @param idx grid index
   */
  Vec3f updateDeformGrad(Vec3i idx);
  void collideWithBody();
  
  /// List of unique pointer to particles 
  std::vector<uPtr<Particle>> particles_;
};

