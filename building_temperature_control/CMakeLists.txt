cmake_minimum_required(VERSION 3.20)
project(circular_building VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/O2 /arch:AVX2 /W4 /MP)
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
    add_compile_options(-O3 -march=native -Wall)
endif()

# ---------------------------------------------------------------------------
# OpenMP
# ---------------------------------------------------------------------------
find_package(OpenMP REQUIRED)

# ---------------------------------------------------------------------------
# Main Executable: circular_building
# ---------------------------------------------------------------------------
add_executable(circular_building
    src/main_building.cpp
    src/contract_solver.cpp
    src/monolithic_baseline.cpp
)
target_include_directories(circular_building PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(circular_building PRIVATE
    OpenMP::OpenMP_CXX
)

# ---------------------------------------------------------------------------
# Test Executable: test_building
# ---------------------------------------------------------------------------
add_executable(test_building
    tests/test_building.cpp
    src/contract_solver.cpp
    src/monolithic_baseline.cpp
)
target_include_directories(test_building PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_building PRIVATE
    OpenMP::OpenMP_CXX
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS circular_building test_building RUNTIME DESTINATION bin)
