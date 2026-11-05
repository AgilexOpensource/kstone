include_guard(GLOBAL)

set(KSTONE_SANITIZE "" CACHE STRING "Sanitizer build: '', address (ASan+UBSan) or thread (TSan)")
set_property(CACHE KSTONE_SANITIZE PROPERTY STRINGS "" address thread)
option(KSTONE_WERROR "Treat warnings as errors in kstone targets" OFF)

if(KSTONE_SANITIZE STREQUAL "address")
  set(_san -fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer)
elseif(KSTONE_SANITIZE STREQUAL "thread")
  set(_san -fsanitize=thread -fno-omit-frame-pointer)
elseif(NOT KSTONE_SANITIZE STREQUAL "")
  message(FATAL_ERROR "Unknown KSTONE_SANITIZE='${KSTONE_SANITIZE}'")
endif()
if(_san)
  # Global so third-party code (ruckig, yaml-cpp, daqp) is instrumented too; TSan needs full coverage.
  add_compile_options(${_san})
  add_link_options(${_san})
endif()
if(KSTONE_SANITIZE STREQUAL "address")
  # Eigen switches to its handmade aligned allocator under __SANITIZE_ADDRESS__, which is ABI-incompatible
  # with prebuilt (non-ASan) Pinocchio: memory allocated there and freed here would be misinterpreted.
  add_compile_definitions(EIGEN_MALLOC_ALREADY_ALIGNED=1)
endif()

# Applied only to kstone's own targets.
function(kstone_target_defaults target)
  target_compile_features(${target} PUBLIC cxx_std_17)
  set_target_properties(${target} PROPERTIES CXX_EXTENSIONS OFF POSITION_INDEPENDENT_CODE ON)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive- $<$<BOOL:${KSTONE_WERROR}>:/WX>)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wnon-virtual-dtor
      -Wold-style-cast -Wcast-align -Woverloaded-virtual -Wconversion -Wsign-conversion
      -Wdouble-promotion -Wformat=2 -Wimplicit-fallthrough
      $<$<BOOL:${KSTONE_WERROR}>:-Werror>)
    # GCC 15 reports ~400 false null-dereferences inside inlined Eigen code; GCC 11 is clean, 12-14 untested.
    if(NOT (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 12))
      target_compile_options(${target} PRIVATE -Wnull-dereference)
    endif()
    # GCC 13 with sanitizers reports false maybe-uninitialized inside libstdc++ <regex>.
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND NOT KSTONE_SANITIZE STREQUAL "")
      target_compile_options(${target} PRIVATE -Wno-maybe-uninitialized)
    endif()
  endif()
endfunction()
