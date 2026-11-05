cmake_minimum_required(VERSION 3.22)
project(kstone VERSION 0.0.1 LANGUAGES C CXX)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(KSTONE_BUILD_TESTS "Build kstone tests" ${PROJECT_IS_TOP_LEVEL})
option(KSTONE_BUILD_TOOLS "Build the kstone command-line tool" ${PROJECT_IS_TOP_LEVEL})
option(KSTONE_BUILD_EXAMPLES "Build the C++ usage examples" ${PROJECT_IS_TOP_LEVEL})
option(KSTONE_BUILD_BENCH "Build kstone benchmarks (not part of ctest)" OFF)
option(KSTONE_BUILD_PYTHON "Build the kstone Python extension (needs pybind11)" OFF)

find_program(KSTONE_CCACHE ccache)
if(KSTONE_CCACHE AND NOT CMAKE_CXX_COMPILER_LAUNCHER)
  set(CMAKE_C_COMPILER_LAUNCHER ${KSTONE_CCACHE})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${KSTONE_CCACHE})
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(KstoneOptions)
include(KstoneDependencies)

add_subdirectory(src)

if(KSTONE_BUILD_TOOLS)
  add_subdirectory(tools)
endif()

if(KSTONE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(KSTONE_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(KSTONE_BUILD_BENCH)
  add_subdirectory(bench)
endif()

if(KSTONE_BUILD_PYTHON)
  add_subdirectory(python)
endif()
