cmake_minimum_required(VERSION 3.20)
project(robot_navigation VERSION 1.0 LANGUAGES CXX)

# ---------------------------------------------------------------------------
# C++ standard
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------
if(MSVC)
    add_compile_options(/O2 /arch:AVX2 /W4 /MP)
    add_compile_definitions(_USE_MATH_DEFINES _CRT_SECURE_NO_WARNINGS)
else()
    add_compile_options(-O3 -march=native -Wall)
endif()

# ---------------------------------------------------------------------------
# OpenMP (required)
# ---------------------------------------------------------------------------
find_package(OpenMP REQUIRED)

# ---------------------------------------------------------------------------
# OpenGL (required)
# ---------------------------------------------------------------------------
find_package(OpenGL REQUIRED)

# ---------------------------------------------------------------------------
# FetchContent
# ---------------------------------------------------------------------------
include(FetchContent)

# -- GLFW --
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.4
)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# -- Dear ImGui --
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.5
)
FetchContent_MakeAvailable(imgui)

# ImGui has no CMakeLists.txt — build it as a static library manually.
add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui_lib PUBLIC glfw OpenGL::GL)

# ---------------------------------------------------------------------------
# Executable
# ---------------------------------------------------------------------------
add_executable(robot_navigation
    src/main_robot.cpp
    src/product_automaton.cpp
    src/reachability_cpu.cpp
)

set_target_properties(robot_navigation PROPERTIES OUTPUT_NAME robot_navigation)

target_include_directories(robot_navigation PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(robot_navigation PRIVATE
    OpenGL::GL
    glfw
    imgui_lib
    OpenMP::OpenMP_CXX
)

# Windows: explicitly link opengl32
if(WIN32)
    target_link_libraries(robot_navigation PRIVATE opengl32)
endif()

# ---------------------------------------------------------------------------
# Test Executable
# ---------------------------------------------------------------------------
add_executable(test_robot
    tests/test_robot.cpp
    src/product_automaton.cpp
    src/reachability_cpu.cpp
)
target_include_directories(test_robot PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_robot PRIVATE
    OpenMP::OpenMP_CXX
)

# ---------------------------------------------------------------------------
# Optional CUDA block
# ---------------------------------------------------------------------------
if(CUDA IN_LIST LANGUAGES)
    # TODO: add reachability_cuda.cu here when CUDA toolkit is installed
endif()

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS robot_navigation RUNTIME DESTINATION bin)
