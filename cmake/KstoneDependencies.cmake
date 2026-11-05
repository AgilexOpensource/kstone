# Resolves every third-party dependency into imported targets:
#   Eigen3::Eigen pinocchio::pinocchio coal::coal kstone_daqp kstone_ruckig kstone_yaml (OBJECT) doctest::doctest
include_guard(GLOBAL)

set(KSTONE_THIRD_PARTY_DIR ${PROJECT_SOURCE_DIR}/third_party)

function(kstone_require_submodule name probe)
  if(NOT EXISTS ${KSTONE_THIRD_PARTY_DIR}/${name}/${probe})
    message(FATAL_ERROR "third_party/${name} is missing. Run: git submodule update --init --recursive")
  endif()
endfunction()

# --- Pinocchio / Coal / Eigen -------------------------------------------------
# AUTO:   SYSTEM first, then PIP if its Pinocchio or Coal is missing or too old.
# SYSTEM: plain find_package, i.e. a source or ROS install reached via CMAKE_PREFIX_PATH.
# PIP:    pin/coal wheels (site-packages/cmeel.prefix) of the active Python; the pip build uses this.
set(KSTONE_PINOCCHIO_SOURCE AUTO CACHE STRING "Where Pinocchio/Coal come from: AUTO, SYSTEM or PIP")
set_property(CACHE KSTONE_PINOCCHIO_SOURCE PROPERTY STRINGS AUTO SYSTEM PIP)
if(NOT KSTONE_PINOCCHIO_SOURCE MATCHES "^(AUTO|PIP|SYSTEM)$")
  message(FATAL_ERROR "KSTONE_PINOCCHIO_SOURCE must be AUTO, SYSTEM or PIP, got '${KSTONE_PINOCCHIO_SOURCE}'")
endif()

set(KSTONE_MIN_PINOCCHIO_VERSION 4.0.0)
set(KSTONE_MIN_COAL_VERSION 3.0.3)
set(Python3_FIND_VIRTUALENV FIRST)

# Lists every candidate without loading any (impossible version), then picks the first acceptable one.
function(kstone_pick_config pkg min source out)
  find_package(${pkg} 999999 CONFIG QUIET ${ARGN})
  set(_pick "")
  set(_trace ${_kstone_trace})
  if(NOT ${pkg}_CONSIDERED_CONFIGS)
    list(APPEND _trace "  [${source}] ${pkg}: no config found")
  endif()
  set(_seen "")
  foreach(_cfg _ver IN ZIP_LISTS ${pkg}_CONSIDERED_CONFIGS ${pkg}_CONSIDERED_VERSIONS)
    if(_cfg IN_LIST _seen)
      continue()
    endif()
    list(APPEND _seen "${_cfg}")
    get_filename_component(_dir "${_cfg}" DIRECTORY)
    set(_why "")
    if(_pick)
      continue()
    elseif(source STREQUAL "system" AND _dir MATCHES "/cmeel\\.prefix/")
      set(_why "pip wheel, only used by the pip source")
    elseif(_ver STREQUAL "" OR _ver STREQUAL "unknown")
      set(_why "no version file")
    elseif(_ver VERSION_LESS min)
      set(_why "needs >= ${min}")
    endif()
    if(_why)
      list(APPEND _trace "  [${source}] ${pkg} ${_ver} at ${_dir}: rejected, ${_why}")
    else()
      list(APPEND _trace "  [${source}] ${pkg} ${_ver} at ${_dir}: ok")
      set(_pick "${_dir}")
    endif()
  endforeach()
  set(${out} "${_pick}" PARENT_SCOPE)
  set(_kstone_trace ${_trace} PARENT_SCOPE)
endfunction()

# A source is taken only when both Pinocchio and Coal from it are acceptable.
macro(kstone_try_system)
  kstone_pick_config(pinocchio ${KSTONE_MIN_PINOCCHIO_VERSION} system _pin_dir)
  kstone_pick_config(coal ${KSTONE_MIN_COAL_VERSION} system _coal_dir)
endmacro()

macro(kstone_try_pip)
  set(_pin_dir "")
  set(_coal_dir "")
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(NOT Python3_Interpreter_FOUND)
    list(APPEND _kstone_trace "  [pip] no Python 3 interpreter found")
  else()
    execute_process(COMMAND "${Python3_EXECUTABLE}" -c
                    "import os,sys;print(next((p for p in (os.path.join(r,'cmeel.prefix') for r in sys.path if r) if os.path.isdir(os.path.join(p,'lib','cmake','pinocchio'))),''))"
                    OUTPUT_VARIABLE KSTONE_PIP_PREFIX OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(NOT KSTONE_PIP_PREFIX)
      list(APPEND _kstone_trace "  [pip] no pin wheel (site-packages/cmeel.prefix) for ${Python3_EXECUTABLE}")
    else()
      kstone_pick_config(pinocchio ${KSTONE_MIN_PINOCCHIO_VERSION} pip _pin_dir PATHS ${KSTONE_PIP_PREFIX} NO_DEFAULT_PATH)
      kstone_pick_config(coal ${KSTONE_MIN_COAL_VERSION} pip _coal_dir PATHS ${KSTONE_PIP_PREFIX} NO_DEFAULT_PATH)
    endif()
  endif()
endmacro()

# AUTO remembers what it picked so a reconfigure does not mix the two sources' cached deps.
set(_pin_mode ${KSTONE_PINOCCHIO_SOURCE})
if(_pin_mode STREQUAL "AUTO" AND KSTONE_PINOCCHIO_RESOLVED)
  set(_pin_mode ${KSTONE_PINOCCHIO_RESOLVED})
endif()
set(KSTONE_PIP_PREFIX "")
set(_kstone_trace "")
set(_pin_dir "")
set(_coal_dir "")
if(_pin_mode MATCHES "^(AUTO|SYSTEM)$")
  kstone_try_system()
  if(_pin_dir AND _coal_dir)
    set(_pin_mode SYSTEM)
  endif()
endif()
if(_pin_mode MATCHES "^(AUTO|PIP)$")
  kstone_try_pip()
  if(_pin_dir AND _coal_dir)
    set(_pin_mode PIP)
  endif()
endif()
string(REPLACE ";" "\n" _trace_text "${_kstone_trace}")
if(NOT _pin_dir OR NOT _coal_dir)
  message(FATAL_ERROR "No usable Pinocchio >= ${KSTONE_MIN_PINOCCHIO_VERSION} with Coal >= "
          "${KSTONE_MIN_COAL_VERSION}. KSTONE_PINOCCHIO_SOURCE=${KSTONE_PINOCCHIO_SOURCE}, searched:\n"
          "${_trace_text}\n"
          "Fix with one of:\n"
          "  source: install Pinocchio >= ${KSTONE_MIN_PINOCCHIO_VERSION} and Coal >= ${KSTONE_MIN_COAL_VERSION}, then -DCMAKE_PREFIX_PATH=<their prefix>\n"
          "  ROS:    sudo apt install ros-humble-pinocchio, then source /opt/ros/humble/setup.bash\n"
          "  pip:    python3 -m pip install 'pin>=4.0.0' 'coal>=3.0.3' cmeel-urdfdom-headers cmeel-eigen\n"
          "AUTO tries the system first, then pip. If it keeps reusing an old pick, delete CMakeCache.txt.")
endif()
if(_kstone_trace MATCHES "rejected|no ")
  message(STATUS "kstone: dependency search\n${_trace_text}")
endif()
if(KSTONE_PINOCCHIO_SOURCE STREQUAL "AUTO")
  set(KSTONE_PINOCCHIO_RESOLVED ${_pin_mode} CACHE INTERNAL "Source AUTO picked")
else()
  unset(KSTONE_PINOCCHIO_RESOLVED CACHE)
endif()
set(pinocchio_DIR "${_pin_dir}" CACHE PATH "" FORCE)
set(coal_DIR "${_coal_dir}" CACHE PATH "" FORCE)

if(_pin_mode STREQUAL "PIP")
  set(_pin_source "pip wheels")
  # A sourced ROS would otherwise leak its own Pinocchio/urdfdom/Boost into the search.
  if(DEFINED ENV{AMENT_PREFIX_PATH} AND NOT "$ENV{AMENT_PREFIX_PATH}" STREQUAL "")
    string(REPLACE ":" ";" _ros "$ENV{AMENT_PREFIX_PATH}")
    list(REMOVE_ITEM CMAKE_PREFIX_PATH ${_ros})
    string(REPLACE ":" ";" _env "$ENV{CMAKE_PREFIX_PATH}")
    list(REMOVE_ITEM _env ${_ros})
    string(REPLACE ";" ":" _env "${_env}")
    set(ENV{CMAKE_PREFIX_PATH} "${_env}")
    set(CMAKE_FIND_USE_SYSTEM_ENVIRONMENT_PATH OFF)
  endif()
  list(PREPEND CMAKE_PREFIX_PATH ${KSTONE_PIP_PREFIX})
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # DT_RPATH outranks LD_LIBRARY_PATH, so a sourced ROS cannot swap the pip sonames at run time.
    add_link_options(-Wl,--disable-new-dtags)
    # cmeel libs use RUNPATH $ORIGIN, which LD_LIBRARY_PATH outranks; direct NEEDED puts every one under our RPATH.
    add_link_options(-Wl,--no-as-needed)
  endif()
else()
  set(_pin_source "system")
  set(KSTONE_PIP_PREFIX "")
endif()

# FindBoost in CMake < 3.30 does not know Boost 1.90 (pulled in by coal) and warns about every component.
set(Boost_NO_WARN_NEW_VERSIONS ON)
# Pinocchio's own config resets policies, so set the default rather than the policy itself.
set(CMAKE_POLICY_DEFAULT_CMP0167 NEW)
# coal first, so Pinocchio's own find_dependency(coal) reuses the picked one.
find_package(coal CONFIG REQUIRED PATHS ${_coal_dir} NO_DEFAULT_PATH)
find_package(pinocchio CONFIG REQUIRED PATHS ${_pin_dir} NO_DEFAULT_PATH)
# urdfdom_headers < 2 (Ubuntu 22.04 apt, ROS Humble) has no capsule.
include(CheckCXXSourceCompiles)
set(CMAKE_REQUIRED_LIBRARIES pinocchio::pinocchio)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
check_cxx_source_compiles("#include <urdf_parser/urdf_parser.h>
int f() { return ::urdf::Geometry::CAPSULE; }" KSTONE_URDF_HAS_CAPSULE)
unset(CMAKE_REQUIRED_LIBRARIES)
unset(CMAKE_TRY_COMPILE_TARGET_TYPE)
# Must be the Eigen Pinocchio was built with, so reuse the one its config found.
find_package(Eigen3 CONFIG REQUIRED)
find_package(Threads REQUIRED)
if(Eigen3_VERSION VERSION_LESS 3.4)
  message(FATAL_ERROR "kstone needs Eigen >= 3.4, found ${Eigen3_VERSION} at ${Eigen3_DIR}")
endif()
message(STATUS "kstone: Eigen ${Eigen3_VERSION}, Pinocchio ${pinocchio_VERSION}, Coal ${coal_VERSION} "
        "from ${_pin_source} (${pinocchio_DIR})")

# --- DAQP (C, vendored as a static library) -----------------------------------
kstone_require_submodule(daqp src/api.c)
file(GLOB _daqp_sources CONFIGURE_DEPENDS ${KSTONE_THIRD_PARTY_DIR}/daqp/src/*.c)
# OBJECT library: its objects are compiled into kstone itself (see src/CMakeLists.txt), so nothing extra is exported.
add_library(kstone_daqp OBJECT ${_daqp_sources})
set_target_properties(kstone_daqp PROPERTIES POSITION_INDEPENDENT_CODE ON C_VISIBILITY_PRESET hidden)
target_include_directories(kstone_daqp PRIVATE ${KSTONE_THIRD_PARTY_DIR}/daqp/include)

# --- Ruckig / yaml-cpp (static, private to kstone) --------------------------
kstone_require_submodule(ruckig CMakeLists.txt)
kstone_require_submodule(yaml-cpp src/parse.cpp)
set(_saved_shared ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS OFF)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARK OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "" FORCE)
add_subdirectory(${KSTONE_THIRD_PARTY_DIR}/ruckig ${PROJECT_BINARY_DIR}/third_party/ruckig EXCLUDE_FROM_ALL)
# kstone compiles the community Ruckig sources into itself (like DAQP), so a static libkstone exports nothing
# extra. The cloud client is left out; the sources build as C++17.
file(GLOB _ruckig_sources CONFIGURE_DEPENDS ${KSTONE_THIRD_PARTY_DIR}/ruckig/src/ruckig/*.cpp)
list(FILTER _ruckig_sources EXCLUDE REGEX "cloud_client\\.cpp$")
add_library(kstone_ruckig OBJECT ${_ruckig_sources})
set_target_properties(kstone_ruckig PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON CXX_EXTENSIONS OFF)
target_compile_features(kstone_ruckig PRIVATE cxx_std_17)
target_include_directories(kstone_ruckig SYSTEM PRIVATE ${KSTONE_THIRD_PARTY_DIR}/ruckig/include)
# yaml-cpp is compiled into kstone the same way, so neither an exported nor an installed target is needed.
file(GLOB _yaml_sources CONFIGURE_DEPENDS ${KSTONE_THIRD_PARTY_DIR}/yaml-cpp/src/*.cpp)
add_library(kstone_yaml OBJECT ${_yaml_sources})
set_target_properties(kstone_yaml PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON CXX_EXTENSIONS OFF)
target_compile_features(kstone_yaml PRIVATE cxx_std_17)
target_compile_definitions(kstone_yaml PRIVATE YAML_CPP_STATIC_DEFINE)
# yaml-cpp 0.8.0 emitterutils.cpp lacks <cstdint> (fixed upstream in 7b469b4); GCC 15 no longer pulls it in.
target_compile_options(kstone_yaml PRIVATE -include cstdint)
target_include_directories(kstone_yaml SYSTEM PRIVATE ${KSTONE_THIRD_PARTY_DIR}/yaml-cpp/include)
set(BUILD_SHARED_LIBS ${_saved_shared})
set_target_properties(ruckig PROPERTIES POSITION_INDEPENDENT_CODE ON CXX_VISIBILITY_PRESET hidden)

# --- doctest (header only, tests) ---------------------------------------------
if(KSTONE_BUILD_TESTS)
  kstone_require_submodule(doctest doctest/doctest.h)
  add_library(kstone_doctest INTERFACE)
  add_library(doctest::doctest ALIAS kstone_doctest)
  target_include_directories(kstone_doctest SYSTEM INTERFACE ${KSTONE_THIRD_PARTY_DIR}/doctest)
  include(${KSTONE_THIRD_PARTY_DIR}/doctest/scripts/cmake/doctest.cmake)
endif()
