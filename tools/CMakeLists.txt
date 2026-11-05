include(GNUInstallDirs)
add_executable(kstone_cli main.cpp cli.cpp)
kstone_target_defaults(kstone_cli)
set_target_properties(kstone_cli PROPERTIES OUTPUT_NAME kstone)
target_link_libraries(kstone_cli PRIVATE kstone::kstone)
# Keep external link dirs (pip Pinocchio) in the installed RPATH; otherwise the binary cannot start.
set_target_properties(kstone_cli PROPERTIES INSTALL_RPATH_USE_LINK_PATH ON)
# USE_LINK_PATH skips dirs inside the source tree, such as a repo-local .venv.
if(KSTONE_PIP_PREFIX)
  set_property(TARGET kstone_cli APPEND PROPERTY INSTALL_RPATH "${KSTONE_PIP_PREFIX}/lib")
endif()
if(BUILD_SHARED_LIBS)
  set_property(TARGET kstone_cli APPEND PROPERTY INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
else()
  # Build and install RPATH are identical; skipping the rewrite avoids CMake's empty padding entry, which means cwd.
  set_target_properties(kstone_cli PROPERTIES BUILD_WITH_INSTALL_RPATH ON)
endif()
install(TARGETS kstone_cli RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
