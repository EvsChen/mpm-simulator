#include <array>
#include <stdexcept>
#include <vector>

#include "global.h"

struct Block {
  Float mass = 0.0;
  Vec3f vel = Vec3f(0.0);
};

class Grid {
public:
  Grid();
  ~Grid();
  void init();
  void updateGridVel();
  void collideWithBody();

  /**
   * Check whether a given index is valid
   * This function should be called before using getBlockAt
   * @param idx Block index
   */
  bool isValidIdx(const Vec3i &idx) const {
    return idx[0] < size_[0] && idx[0] >= 0 &&
           idx[1] < size_[1] && idx[1] >= 0 &&
           idx[2] < size_[2] && idx[2] >= 0;
  }

  /**
   * Get block pointer at idx
   * @param idx block index
   */
  Block *getBlockAt(const Vec3i &idx) {
#ifdef NDEBUG
    if (!isValidIdx(idx)) {
      throw std::invalid_argument("Index of out range");
    }
#endif
    return blocks_[idx[0] + idx[1] * size_[0] + idx[2] * size_[0] * size_[1]].get();
  }

  Float spacing_;
  std::array<int, 3> size_;
  std::vector<uPtr<Block>> blocks_;
  /// The node with non-zero mass
  std::vector<int> nonEmptyBlocks_;
};
