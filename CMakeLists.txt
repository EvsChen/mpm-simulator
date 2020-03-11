cmake_minimum_required(VERSION 3.10)

# set the project name
project(MPMSimulator)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(LIBRARY_NAME MPMSolverPlugin)

# Locate Houdini's libraries and header files.
# Registers an imported library target named 'Houdini'.
find_package( Houdini REQUIRED )

# glog
add_subdirectory(src/ext/glog)

add_executable(${PROJECT_NAME}
  ./src/main.cpp
  ./src/grid.cpp
  ./src/util.cpp
  ./src/engine.cpp
  ./src/particle.cpp
  ./src/SVD.cpp
  ./src/constitutiveModel.cpp
  ./src/plasticity.cpp
)

add_library( ${LIBRARY_NAME} SHARED
    ./src/houdini/MPM_Solver.C
    ./src/houdini/MPM_Solver.h
)


target_link_libraries(${PROJECT_NAME} glog::glog)
target_link_libraries(${LIBRARY_NAME} glog::glog)

# Link against the Houdini libraries, and add required include directories and compile definitions.
target_link_libraries(${LIBRARY_NAME} Houdini)

target_include_directories(${PROJECT_NAME} PUBLIC src)
target_include_directories(${LIBRARY_NAME} PUBLIC src)

# Configure several common target properties, such as its output directory.
houdini_configure_target( ${LIBRARY_NAME} )