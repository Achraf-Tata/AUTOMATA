cmake_minimum_required(VERSION 3.20)
project(BuildingThermostat LANGUAGES CXX)
set(CMAKE_CXX_EXTENSIONS OFF)

option(THERMOSTAT_BUILD_GUI "Build the GLFW and Dear ImGui desktop frontend" ON)

add_library(thermostat_core STATIC src/abstraction.cpp src/controller.cpp src/building.cpp)
target_include_directories(thermostat_core PUBLIC include)
target_compile_features(thermostat_core PUBLIC cxx_std_20)

add_executable(thermostat_cli src/main.cpp)
target_link_libraries(thermostat_cli PRIVATE thermostat_core)
if(WIN32)
    target_link_libraries(thermostat_cli PRIVATE psapi)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(thermostat_tests tests/thermostat_tests.cpp)
    target_link_libraries(thermostat_tests PRIVATE thermostat_core)
    add_test(NAME thermostat_core COMMAND thermostat_tests)
endif()

if(THERMOSTAT_BUILD_GUI)
    include(FetchContent)
    find_package(OpenGL REQUIRED)

    set(GLFW_BUILD_DOCS OFF)
    set(GLFW_BUILD_TESTS OFF)
    set(GLFW_BUILD_EXAMPLES OFF)
    set(GLFW_INSTALL OFF)
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
        set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "Build GLFW with Wayland support")
    endif()

    FetchContent_Declare(glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.4
        GIT_SHALLOW TRUE
    )
    FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.91.5
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(glfw imgui)

    # Embed ImGui's bundled Roboto so the font is available from any working directory.
    set(thermostat_font "${imgui_SOURCE_DIR}/misc/fonts/Roboto-Medium.ttf")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${thermostat_font}")
    file(READ "${thermostat_font}" thermostat_font_hex HEX)
    string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," thermostat_font_bytes "${thermostat_font_hex}")
    string(REPEAT "0x[0-9a-f][0-9a-f]," 16 thermostat_font_line)
    string(REGEX REPLACE "(${thermostat_font_line})" "\\1\n" thermostat_font_bytes "${thermostat_font_bytes}")
    file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/thermostat_font.hpp"
        CONTENT "#pragma once\ninline unsigned char thermostat_font[] = {\n${thermostat_font_bytes}\n};\n")

    add_executable(thermostat_gui
        src/gui.cpp
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/misc/cpp/imgui_stdlib.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )
    target_include_directories(thermostat_gui PRIVATE
        "${imgui_SOURCE_DIR}"
        "${imgui_SOURCE_DIR}/backends"
        "${imgui_SOURCE_DIR}/misc/cpp"
        "${CMAKE_CURRENT_BINARY_DIR}"
    )
    if(MSVC)
        target_compile_options(thermostat_gui PRIVATE /utf-8)
    endif()
    target_link_libraries(thermostat_gui PRIVATE thermostat_core glfw OpenGL::GL ${CMAKE_DL_LIBS})
endif()
