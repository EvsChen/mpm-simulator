cmake_minimum_required(VERSION 3.10)

# set the project name
project(MPMSimulator)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
if (MSVC)
	set(DEFAULT_CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
	SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
	set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
	set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
	set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
endif()


# glog
add_subdirectory(src/ext/glog)

if (MSVC)
	set(EXECUTABLE_OUTPUT_PATH  ${PROJECT_SOURCE_DIR}/build/)
endif()

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
if (MSVC)
	set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")
endif()