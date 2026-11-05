# Usage examples; each also runs as a ctest so it keeps building and working.
add_executable(kstone_example_ik cpp/ik.cpp)
add_executable(kstone_example_engine cpp/engine.cpp)
foreach(t kstone_example_ik kstone_example_engine)
  kstone_target_defaults(${t})
  target_link_libraries(${t} PRIVATE kstone::kstone)
endforeach()

if(KSTONE_BUILD_TESTS)
  set(prefix)
  if(KSTONE_SANITIZE STREQUAL "thread" AND KSTONE_SETARCH)
    set(prefix ${KSTONE_SETARCH} ${CMAKE_SYSTEM_PROCESSOR} -R)
  endif()
  add_test(NAME example/cpp_ik
           COMMAND ${prefix} $<TARGET_FILE:kstone_example_ik> ${CMAKE_CURRENT_SOURCE_DIR}/omni_dual_arm/omni_dual_arm.urdf)
  set_tests_properties(example/cpp_ik PROPERTIES PASS_REGULAR_EXPRESSION "converged in")
  add_test(NAME example/cpp_engine
           COMMAND ${prefix} $<TARGET_FILE:kstone_example_engine> ${PROJECT_SOURCE_DIR}/tests/fixtures/arm.urdf)
  set_tests_properties(example/cpp_engine PROPERTIES PASS_REGULAR_EXPRESSION "reached after")
endif()
