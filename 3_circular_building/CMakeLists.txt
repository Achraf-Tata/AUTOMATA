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
# Console Executable: circular_building
# ---------------------------------------------------------------------------
add_executable(circular_building
    src/main_building.cpp
)
target_link_libraries(circular_building PRIVATE building_core)

# ---------------------------------------------------------------------------
# Test Executable: test_building
# ---------------------------------------------------------------------------
add_executable(test_building
    tests/test_building.cpp
)
target_link_libraries(test_building PRIVATE building_core)

# ---------------------------------------------------------------------------
# GUI Dependencies: OpenGL, GLFW, Dear ImGui
# ---------------------------------------------------------------------------
find_package(OpenGL REQUIRED)

include(FetchContent)

set(LOCAL_GLFW_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/glfw-src")
set(LOCAL_IMGUI_DIR "${CMAKE_CURRENT_BINARY_DIR}/_deps/imgui-src")

# GLFW
if(EXISTS "${LOCAL_GLFW_DIR}/include/GLFW/glfw3.h")
    FetchContent_Declare(glfw SOURCE_DIR "${LOCAL_GLFW_DIR}")
else()
    FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG        3.4
        GIT_SHALLOW    TRUE
    )
endif()
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# Dear ImGui
if(EXISTS "${LOCAL_IMGUI_DIR}/imgui.h")
    FetchContent_Declare(imgui SOURCE_DIR "${LOCAL_IMGUI_DIR}")
else()
    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG        v1.91.5
        GIT_SHALLOW    TRUE
    )
endif()
FetchContent_MakeAvailable(imgui)

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
# Interactive GUI Executable: building_gui
# ---------------------------------------------------------------------------
set(GUI_SOURCES
    src/main_gui.cpp
    src/ui_theme.cpp
    src/building_view.cpp
    src/control_panel.cpp
    src/popup_dialogs.cpp
)

add_executable(building_gui
    ${GUI_SOURCES}
)

set_target_properties(building_gui PROPERTIES
    OUTPUT_NAME building_gui
    WIN32_EXECUTABLE TRUE
)

target_include_directories(building_gui PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(building_gui PRIVATE
    building_core
    OpenGL::GL
    glfw
    imgui_lib
)

if(WIN32 AND MSVC)
    target_link_libraries(building_gui PRIVATE opengl32)
    target_link_options(building_gui PRIVATE /SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup)
endif()

# ---------------------------------------------------------------------------
# Install Targets
# ---------------------------------------------------------------------------
install(TARGETS circular_building test_building building_gui RUNTIME DESTINATION bin)
