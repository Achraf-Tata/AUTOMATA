cmake_minimum_required(VERSION 3.20)
project(circular_building VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/O2 /arch:AVX2 /W4 /MP /utf-8 /permissive-)
    add_compile_definitions(_USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
    add_link_options(/STACK:8388608)
else()
    add_compile_options(-O3 -march=native -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# OpenMP Multi-Core CPU Parallelization
# ---------------------------------------------------------------------------
find_package(OpenMP)

# ---------------------------------------------------------------------------
# Core Library: building_core
# ---------------------------------------------------------------------------
set(CORE_SOURCES
    src/thermal_network.cpp
    src/contract_abstraction.cpp
    src/contract_solver.cpp
    src/monolithic_baseline.cpp
    src/app_state.cpp
    src/simulation_engine.cpp
)

add_library(building_core STATIC ${CORE_SOURCES})
target_include_directories(building_core PUBLIC include)

if(OpenMP_CXX_FOUND)
    target_link_libraries(building_core PUBLIC OpenMP::OpenMP_CXX)
endif()

# ---------------------------------------------------------------------------
# Console Executable: circular_building (Slide 15-17 Benchmarks & Trajectory)
# ---------------------------------------------------------------------------
add_executable(circular_building
    src/main_building.cpp
)
target_link_libraries(circular_building PRIVATE building_core)

# ---------------------------------------------------------------------------
# Test Executable: test_building (10 Verification Unit Tests)
# ---------------------------------------------------------------------------
add_executable(test_building
    tests/test_building.cpp
)
target_link_libraries(test_building PRIVATE building_core)
