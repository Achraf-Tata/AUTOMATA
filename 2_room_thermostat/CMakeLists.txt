cmake_minimum_required(VERSION 3.20)
project(room_thermostat VERSION 1.0 LANGUAGES CXX)

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
# Main Executable: room_thermostat
# ---------------------------------------------------------------------------
add_executable(room_thermostat
    src/main_thermostat.cpp
    src/safety_solver.cpp
)
target_include_directories(room_thermostat PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---------------------------------------------------------------------------
# Test Executable: test_thermostat
# ---------------------------------------------------------------------------
add_executable(test_thermostat
    tests/test_thermostat.cpp
    src/safety_solver.cpp
)
target_include_directories(test_thermostat PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS room_thermostat test_thermostat RUNTIME DESTINATION bin)
