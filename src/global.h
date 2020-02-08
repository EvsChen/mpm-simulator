#include <cassert>

#include "ext/glm/glm.hpp"

// Custom float definition
typedef float Float;

typedef glm::vec3 Vec3f;
typedef glm::ivec3 Vec3i;

typedef glm::mat3x3 Mat3f;

#define mkU std::make_unique
#define mkS std::make_shared
#define uPtr std::unique_ptr
#define sPtr std::shared_ptr

#define NDEBUG