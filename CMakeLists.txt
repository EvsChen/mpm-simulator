cmake_minimum_required(VERSION 3.10)

# set the project name
project(MPMSimulator)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

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
target_link_libraries(${PROJECT_NAME} glog::glog)
target_include_directories(${PROJECT_NAME} PUBLIC src)
