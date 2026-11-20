cmake_minimum_required(VERSION 3.20)
project(AcademicPrototype LANGUAGES CXX)

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

set(PROTOTYPE_SOURCES
    src/interval.cpp
    src/bounding_box.cpp
    src/grid_partition.cpp
    src/dfa.cpp
    src/product_automaton.cpp
    src/unicycle_model.cpp
    src/transition_system.cpp
    src/backward_reachability.cpp
    src/safety_solver.cpp
    src/grid_quantizer.cpp
    src/simulator.cpp
)

add_library(prototype_core STATIC ${PROTOTYPE_SOURCES})
target_include_directories(prototype_core PUBLIC include)

# Optional multi-core CPU parallelization via OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(prototype_core PUBLIC OpenMP::OpenMP_CXX)
endif()

add_executable(prototype_navigation src/main.cpp)
target_link_libraries(prototype_navigation PRIVATE prototype_core)

add_executable(test_prototype
    tests/test_prototype.cpp
    src/simulation_engine.cpp
    src/app_state.cpp
)
target_link_libraries(test_prototype PRIVATE prototype_core)
