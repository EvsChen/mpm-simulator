#include "grid.h"

Grid::Grid(const Vec3i &s, Float space) :
  spacing_(space), size_(s),
  blocks_(new std::vector<Block>((s[0] + 1) * (s[1] + 1) * (s[2] + 1)))
{}

Grid::Grid() : Grid(Vec3i::Constant(GRID_SIZE), GRID_SPACING) {}