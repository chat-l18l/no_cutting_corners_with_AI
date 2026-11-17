cmake_minimum_required(VERSION 3.16)
project(rbt_mpc C)

#
# Sampling-based MPC voor kleine voertuigen — zie README.md en
# docs/paper_no_cutting_corners.md.
#
# Bouwen en testen:
#     cmake -B build-native -S . -DCMAKE_BUILD_TYPE=Release
#     cmake --build build-native -j
#     ctest --test-dir build-native --output-on-failure
#
# De gedeelde bibliotheek (librbt_mpc.so) komt in de wortel van de builddir,
# want daar zoekt tools/viz/rbt_mpc_ffi.py hem: de visualisatie draait
# letterlijk dezelfde code als de tests.
#

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

enable_testing()

add_subdirectory(src/mpc)
add_subdirectory(test/native)
