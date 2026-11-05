# Python package `kstone` (native part `kstone._core`); needs pybind11 (pip install pybind11, then -Dpybind11_DIR=$(python -m pybind11 --cmakedir)).
find_package(Python 3.8 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 2.10 CONFIG REQUIRED)

pybind11_add_module(kstone_python MODULE src/kstone.cpp ${PROJECT_SOURCE_DIR}/tools/cli.cpp)
target_include_directories(kstone_python PRIVATE ${PROJECT_SOURCE_DIR}/tools)
set_target_properties(kstone_python PROPERTIES OUTPUT_NAME _core
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/kstone)
set(KSTONE_PYTHON_FILES __init__.py __init__.pyi py.typed)
foreach(f IN LISTS KSTONE_PYTHON_FILES)
  configure_file(kstone/${f} ${CMAKE_CURRENT_BINARY_DIR}/kstone/${f} COPYONLY)
endforeach()
kstone_target_defaults(kstone_python)
target_link_libraries(kstone_python PRIVATE kstone::kstone)
# Keep strict warnings on our code only.
target_include_directories(kstone_python SYSTEM PRIVATE ${pybind11_INCLUDE_DIRS} ${Python_INCLUDE_DIRS})

if(SKBUILD)
  # The wheel installs into site-packages next to the pin/coal wheels' cmeel.prefix.
  set_target_properties(kstone_python PROPERTIES INSTALL_RPATH "$ORIGIN/../cmeel.prefix/lib")
  install(TARGETS kstone_python LIBRARY DESTINATION kstone COMPONENT python)
  list(TRANSFORM KSTONE_PYTHON_FILES PREPEND kstone/)
  install(FILES ${KSTONE_PYTHON_FILES} DESTINATION kstone COMPONENT python)
endif()

if(KSTONE_BUILD_TESTS AND KSTONE_SANITIZE STREQUAL "")
  add_test(NAME python/binding COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_binding.py
           ${PROJECT_SOURCE_DIR}/tests/fixtures)
  set_tests_properties(python/binding PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR};KSTONE_TOKEN=")
  add_test(NAME example/py_ik COMMAND ${Python_EXECUTABLE} ${PROJECT_SOURCE_DIR}/examples/python/ik.py
           ${PROJECT_SOURCE_DIR}/tests/fixtures/arm.urdf)
  add_test(NAME example/py_engine COMMAND ${Python_EXECUTABLE} ${PROJECT_SOURCE_DIR}/examples/python/engine.py
           ${PROJECT_SOURCE_DIR}/tests/fixtures/arm_loopback.yaml)
  set_tests_properties(example/py_ik example/py_engine PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}")
  # Checks __init__.pyi against the module; skipped when mypy is not installed.
  add_test(NAME python/stubs COMMAND ${Python_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/check_stubs.py)
  set_tests_properties(python/stubs PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}" SKIP_RETURN_CODE 77)
endif()
