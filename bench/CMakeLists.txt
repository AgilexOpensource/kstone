add_executable(kstone_bench solve.cpp)
kstone_target_defaults(kstone_bench)
target_link_libraries(kstone_bench PRIVATE kstone::kstone)
target_compile_definitions(kstone_bench PRIVATE KSTONE_FIXTURES="${PROJECT_SOURCE_DIR}/tests/fixtures")

add_executable(kstone_cycle_bench cycle.cpp)
kstone_target_defaults(kstone_cycle_bench)
target_link_libraries(kstone_cycle_bench PRIVATE kstone::kstone)
target_compile_definitions(kstone_cycle_bench PRIVATE KSTONE_FIXTURES="${PROJECT_SOURCE_DIR}/tests/fixtures")

add_executable(kstone_collision_bench collision.cpp)
kstone_target_defaults(kstone_collision_bench)
target_link_libraries(kstone_collision_bench PRIVATE kstone::kstone)
target_compile_definitions(kstone_collision_bench PRIVATE KSTONE_FIXTURES="${PROJECT_SOURCE_DIR}/tests/fixtures")
