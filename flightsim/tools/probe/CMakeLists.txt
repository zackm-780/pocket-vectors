# Flight Sim device probe: a standalone SDL2 + GLES2 (+ optional JSBSim) program.
#
# Desktop:   cmake -S tools/probe -B build-probe -G Ninja && cmake --build build-probe
# Device:    tools/build_device.sh (adds -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-aarch64.cmake)
#
# This project is deliberately independent of the main app's CMake so it can be built, packaged and run
# on the device before (or without) the game itself.
cmake_minimum_required(VERSION 3.16)
project(flightsim_probe LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(PROBE_WITH_JSBSIM "Build the JSBSim CPU benchmark (fetches JSBSim v1.3.3)" ON)
option(PROBE_WERROR "Treat warnings as errors" OFF)
set(PROBE_JSBSIM_SOURCE_DIR "" CACHE PATH "Use this JSBSim checkout instead of fetching it (offline builds)")
set(PROBE_JSBSIM_TAG "v1.3.3" CACHE STRING "JSBSim git tag (decision D3)")

find_package(SDL2 REQUIRED)
if(TARGET SDL2::SDL2)
  set(PROBE_SDL_TARGET SDL2::SDL2)
else()
  add_library(probe_sdl2 INTERFACE)
  target_include_directories(probe_sdl2 INTERFACE ${SDL2_INCLUDE_DIRS})
  target_link_libraries(probe_sdl2 INTERFACE ${SDL2_LIBRARIES})
  set(PROBE_SDL_TARGET probe_sdl2)
endif()

add_executable(flightsim_probe
  main.cpp
  Report.cpp
  SysInfo.cpp
  Gl.cpp
  Context.cpp
  InputTest.cpp
  GpuBench.cpp
  CpuBench.cpp)
target_link_libraries(flightsim_probe PRIVATE ${PROBE_SDL_TARGET} ${CMAKE_DL_LIBS})
target_compile_features(flightsim_probe PRIVATE cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(flightsim_probe PRIVATE -Wall -Wextra)
  if(PROBE_WERROR)
    target_compile_options(flightsim_probe PRIVATE -Werror)
  endif()
endif()

if(PROBE_WITH_JSBSIM)
  include(FetchContent)
  if(POLICY CMP0169)
    cmake_policy(SET CMP0169 OLD)  # keep using FetchContent_Populate() with CMake >= 3.30
  endif()
  if(PROBE_JSBSIM_SOURCE_DIR)
    set(FETCHCONTENT_SOURCE_DIR_JSBSIM "${PROBE_JSBSIM_SOURCE_DIR}" CACHE PATH "" FORCE)
  endif()
  FetchContent_Declare(jsbsim
    GIT_REPOSITORY https://github.com/JSBSim-Team/jsbsim.git
    GIT_TAG ${PROBE_JSBSIM_TAG}
    GIT_SHALLOW TRUE)
  FetchContent_GetProperties(jsbsim)
  if(NOT jsbsim_POPULATED)
    FetchContent_Populate(jsbsim)
    # JSBSim is LGPL-2.1: always a shared library, shipped next to the binary (decision D3).
    set(BUILD_SHARED_LIBS ON)
    set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
    set(BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(SYSTEM_EXPAT OFF CACHE BOOL "" FORCE)
    # EXCLUDE_FROM_ALL: only libJSBSim is built (the JSBSim command line tool is not needed).
    add_subdirectory(${jsbsim_SOURCE_DIR} ${jsbsim_BINARY_DIR} EXCLUDE_FROM_ALL)
    unset(BUILD_SHARED_LIBS)
  endif()
  target_link_libraries(flightsim_probe PRIVATE libJSBSim)
  # JSBSim headers are third-party: keep their warnings out of our -Wall -Wextra output.
  target_include_directories(flightsim_probe SYSTEM PRIVATE "${jsbsim_SOURCE_DIR}/src")
  target_compile_definitions(flightsim_probe PRIVATE PROBE_HAVE_JSBSIM=1)

  # Runtime data next to the binary: <build>/jsbsim/{aircraft/{c172p,737},engine,systems}.
  add_custom_command(TARGET flightsim_probe POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E remove_directory "$<TARGET_FILE_DIR:flightsim_probe>/jsbsim"
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:flightsim_probe>/jsbsim/aircraft"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${jsbsim_SOURCE_DIR}/aircraft/c172p"
            "$<TARGET_FILE_DIR:flightsim_probe>/jsbsim/aircraft/c172p"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${jsbsim_SOURCE_DIR}/aircraft/737"
            "$<TARGET_FILE_DIR:flightsim_probe>/jsbsim/aircraft/737"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${jsbsim_SOURCE_DIR}/engine"
            "$<TARGET_FILE_DIR:flightsim_probe>/jsbsim/engine"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${jsbsim_SOURCE_DIR}/systems"
            "$<TARGET_FILE_DIR:flightsim_probe>/jsbsim/systems"
    COMMENT "Copying JSBSim aircraft c172p and 737, engine/ and systems/ next to the probe")
  # JSBSim's licence text, shipped with the port.
  add_custom_command(TARGET flightsim_probe POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${jsbsim_SOURCE_DIR}/COPYING"
            "$<TARGET_FILE_DIR:flightsim_probe>/JSBSim-COPYING.txt"
    COMMENT "Copying the JSBSim licence")
endif()
