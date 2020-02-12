cmake_minimum_required(VERSION 3.10)

# set the project name
project(MPMSimulator)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_executable(${PROJECT_NAME}
  ./src/main.cpp
  ./src/grid.cpp
  ./src/util.cpp
  ./src/engine.cpp
  ./src/particle.cpp
  ./src/constitutiveModel.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC src)
