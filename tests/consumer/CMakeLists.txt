cmake_minimum_required(VERSION 3.22)
project(kstone_consumer LANGUAGES CXX)

find_package(kstone REQUIRED)

add_executable(consumer main.cpp)
target_compile_features(consumer PRIVATE cxx_std_17)
target_link_libraries(consumer PRIVATE kstone::kstone)
