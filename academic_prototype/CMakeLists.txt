cmake_minimum_required(VERSION 3.20)
project(AcademicPrototype LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /permissive- /utf-8 /O2)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

set(PROTOTYPE_SOURCES
    src/interval.cpp
    src/bounding_box.cpp
    src/grid_partition.cpp
    src/unicycle_model.cpp
    src/transition_system.cpp
    src/backward_reachability.cpp
    src/simulator.cpp
)

add_library(prototype_core STATIC ${PROTOTYPE_SOURCES})
target_include_directories(prototype_core PUBLIC include)

add_executable(prototype_navigation src/main.cpp)
target_link_libraries(prototype_navigation PRIVATE prototype_core)

add_executable(test_prototype tests/test_prototype.cpp)
target_link_libraries(test_prototype PRIVATE prototype_core)

