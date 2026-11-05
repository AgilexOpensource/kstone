configure_file(${PROJECT_SOURCE_DIR}/include/kstone/core/version.hpp.in
               ${PROJECT_BINARY_DIR}/include/kstone/core/version.hpp @ONLY)

add_library(kstone
  core/clock.cpp
  core/context.cpp
  core/error.cpp
  core/histogram.cpp
  core/log.cpp
  core/mask.cpp
  core/resources.cpp
  core/version.cpp
  model/loader.cpp
  model/model.cpp
  model/workspace.cpp
  qp/backend.cpp
  qp/daqp.cpp
  qp/problem.cpp
  control/task.cpp
  control/tasks.cpp
  control/constraints.cpp
  control/solver.cpp
  chassis/chassis.cpp
  motion/joint_trajectory.cpp
  motion/pose_trajectory.cpp
  collision/geometry.cpp
  collision/hull.cpp
  collision/model.cpp
  collision/world.cpp
  collision/query.cpp
  collision/constraint.cpp
  module/module.cpp
  module/homing.cpp
  module/builtins.cpp
  module/config.cpp
  module/schema.cpp
  module/registry.cpp
  module/builtin_schemas.cpp
  module/yaml.cpp
  runtime/engine.cpp
  runtime/engine_c.cpp
  runtime/snapshot.cpp
  runtime/config.cpp
  runtime/debug_server.cpp
  runtime/doctor.cpp
  runtime/model_assets.cpp
  runtime/robot_plugin.cpp
  $<TARGET_OBJECTS:kstone_daqp>
  $<TARGET_OBJECTS:kstone_ruckig>
  $<TARGET_OBJECTS:kstone_yaml>)
add_library(kstone::kstone ALIAS kstone)
# Header-only target for robot driver plugins: the C ABI needs no kstone library at link time.
add_library(kstone_robot_plugin INTERFACE)
add_library(kstone::robot_plugin ALIAS kstone_robot_plugin)
set_target_properties(kstone_robot_plugin PROPERTIES EXPORT_NAME robot_plugin)
target_include_directories(kstone_robot_plugin INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
kstone_target_defaults(kstone)
# 0.x may break ABI on every minor release; from 1.0 only on major releases.
if(PROJECT_VERSION_MAJOR EQUAL 0)
  set(_kstone_soversion ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR})
else()
  set(_kstone_soversion ${PROJECT_VERSION_MAJOR})
endif()
set_target_properties(kstone PROPERTIES
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  POSITION_INDEPENDENT_CODE ON
  VERSION ${PROJECT_VERSION}
  SOVERSION ${_kstone_soversion})
include(GenerateExportHeader)
generate_export_header(kstone BASE_NAME kstone
  EXPORT_FILE_NAME ${PROJECT_BINARY_DIR}/include/kstone/core/export.hpp)
if(NOT BUILD_SHARED_LIBS)
  target_compile_definitions(kstone PUBLIC KSTONE_STATIC_DEFINE)
endif()
target_include_directories(kstone PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
# Public headers need only Eigen; Pinocchio stays an implementation detail (see kstone/model/pinocchio.hpp).
target_link_libraries(kstone PUBLIC Eigen3::Eigen PRIVATE pinocchio::pinocchio Threads::Threads ${CMAKE_DL_LIBS})
target_include_directories(kstone SYSTEM PRIVATE ${KSTONE_THIRD_PARTY_DIR}/daqp/include
  ${KSTONE_THIRD_PARTY_DIR}/ruckig/include
  ${KSTONE_THIRD_PARTY_DIR}/yaml-cpp/include)
target_compile_definitions(kstone PRIVATE YAML_CPP_STATIC_DEFINE)
# Pinocchio's targets carry OpenMP; Eigen's threaded GEMM costs ~10x per cycle on medium robots.
target_compile_definitions(kstone PRIVATE EIGEN_DONT_PARALLELIZE)
if(KSTONE_URDF_HAS_CAPSULE)
  target_compile_definitions(kstone PRIVATE KSTONE_URDF_HAS_CAPSULE)
endif()
if(UNIX)
  target_link_libraries(kstone PRIVATE m)
endif()
if(BUILD_SHARED_LIBS AND UNIX AND NOT APPLE)
  # Export only kstone:: so bundled/third-party template instances stay private.
  target_link_options(kstone PRIVATE "LINKER:--version-script=${PROJECT_SOURCE_DIR}/cmake/kstone.map")
  set_property(TARGET kstone APPEND PROPERTY LINK_DEPENDS ${PROJECT_SOURCE_DIR}/cmake/kstone.map)
endif()
if(BUILD_SHARED_LIBS)
  # The installed libkstone.so still has to find pip Pinocchio.
  # Build and install RPATH match, so skip the rewrite and its cwd-meaning empty padding.
  set_target_properties(kstone PROPERTIES INSTALL_RPATH_USE_LINK_PATH ON BUILD_WITH_INSTALL_RPATH ON)
  if(KSTONE_PIP_PREFIX)
    set_target_properties(kstone PROPERTIES INSTALL_RPATH "${KSTONE_PIP_PREFIX}/lib")
  endif()
endif()

# --- install / export ---------------------------------------------------------
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
set(KSTONE_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/kstone)

install(TARGETS kstone kstone_robot_plugin EXPORT kstoneTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/kstone ${PROJECT_BINARY_DIR}/include/kstone
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
install(EXPORT kstoneTargets NAMESPACE kstone:: DESTINATION ${KSTONE_CMAKE_DIR})

if(BUILD_SHARED_LIBS)
  set(KSTONE_CONFIG_STATIC OFF)
else()
  set(KSTONE_CONFIG_STATIC ON)
endif()
configure_package_config_file(${PROJECT_SOURCE_DIR}/cmake/kstoneConfig.cmake.in
  ${PROJECT_BINARY_DIR}/kstoneConfig.cmake INSTALL_DESTINATION ${KSTONE_CMAKE_DIR})
write_basic_package_version_file(${PROJECT_BINARY_DIR}/kstoneConfigVersion.cmake
  COMPATIBILITY SameMinorVersion)
install(FILES ${PROJECT_BINARY_DIR}/kstoneConfig.cmake ${PROJECT_BINARY_DIR}/kstoneConfigVersion.cmake
  DESTINATION ${KSTONE_CMAKE_DIR})

# pkg-config: shared builds only; static consumers need Pinocchio's full link line, use CMake instead.
if(BUILD_SHARED_LIBS)
  configure_file(${PROJECT_SOURCE_DIR}/cmake/kstone.pc.in ${PROJECT_BINARY_DIR}/kstone.pc @ONLY)
  install(FILES ${PROJECT_BINARY_DIR}/kstone.pc DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
endif()
