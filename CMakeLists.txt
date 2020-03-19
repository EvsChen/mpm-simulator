cmake_minimum_required(VERSION 3.10)

# set the project name
project(MPMSimulator)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(LIBRARY_NAME MPMSolverPlugin)
set(ENGINE_LIBRARY MPMEngine)
# NDEBUG flag will cause strange bugs
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
message("CMAKE_CXX_FLAGS_RELWITHDEBINFO is ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

if (APPLE)
  list(APPEND CMAKE_PREFIX_PATH "/Applications/Houdini/Current/Frameworks/Houdini.framework/Versions/Current/Resources/toolkit/cmake")
endif()

# Locate Houdini's libraries and header files.
# Registers an imported library target named 'Houdini'.
find_package( Houdini REQUIRED )

# glog
add_subdirectory(src/ext/glog)

add_library(${ENGINE_LIBRARY} STATIC
  ./src/grid.cpp
  ./src/util.cpp
  ./src/engine.cpp
  ./src/particle.cpp
  ./src/SVD.cpp
  ./src/constitutiveModel.cpp
  ./src/plasticity.cpp
  ./src/levelSet.cpp
)

target_link_libraries(${ENGINE_LIBRARY} glog::glog)

add_executable(${PROJECT_NAME}
  ./src/main.cpp
)

target_link_libraries(${PROJECT_NAME} ${ENGINE_LIBRARY})

add_library(${LIBRARY_NAME} SHARED
  ./src/houdini/MPM_Solver.C
  ./src/houdini/MPM_Solver.h
)

target_link_libraries(${LIBRARY_NAME} ${ENGINE_LIBRARY})

# Link against the Houdini libraries, and add required include directories and compile definitions.
target_link_libraries(${LIBRARY_NAME} Houdini)

# Configure several common target properties, such as its output directory.
houdini_configure_target( ${LIBRARY_NAME} )
