cmake_minimum_required(VERSION 3.18)  # 3.18 is JSBSim's own minimum
project(flightsim VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(FSIM_WERROR "Treat warnings in our own code as errors (CI sets this ON)" OFF)
option(FSIM_BUILD_TESTS "Build the flightsim_tests target" ON)

# Shared libraries (libJSBSim.so) and the executable all land in the build root so the binary runs
# in place and finds them through $ORIGIN.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_BUILD_RPATH "$ORIGIN")
set(CMAKE_INSTALL_RPATH "$ORIGIN")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(Dependencies)   # JSBSim, doctest, stb_image_write, SDL2

# ---- our code ----------------------------------------------------------------------------------
# Sources are globbed (CONFIGURE_DEPENDS) so parallel work packages can add files without editing
# this list. Everything except main.cpp goes into the static library fsim_lib, which both the
# game and the tests link.
file(GLOB_RECURSE FSIM_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(REMOVE_ITEM FSIM_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp)

function(fsim_set_warnings target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra)
    if(FSIM_WERROR)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  elseif(MSVC)
    target_compile_options(${target} PRIVATE /W4)
    if(FSIM_WERROR)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  endif()
endfunction()

find_package(Threads REQUIRED)
add_library(fsim_lib STATIC ${FSIM_SOURCES})
# About page: project version plus the git commit at configure time.
find_package(Git QUIET)
set(FSIM_GIT_COMMIT "unknown")
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short=12 HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE FSIM_GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
endif()
if(FSIM_GIT_COMMIT STREQUAL "")
  set(FSIM_GIT_COMMIT "unknown")
endif()
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/src/ui/BuildInfo.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/generated/BuildInfo.h @ONLY)
target_include_directories(fsim_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src
                                           ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(fsim_lib PUBLIC fsim_jsbsim fsim_stb SDL2::SDL2 Threads::Threads
                                      ${CMAKE_DL_LIBS})
fsim_set_warnings(fsim_lib)

add_executable(flightsim src/main.cpp)
target_link_libraries(flightsim PRIVATE fsim_lib)
fsim_set_warnings(flightsim)

# ---- runtime layout: <build>/jsbsim and <build>/assets -----------------------------------------
include(RuntimeLayout)
fsim_add_runtime_layout(flightsim)

# ---- tests -------------------------------------------------------------------------------------
if(FSIM_BUILD_TESTS)
  enable_testing()
  file(GLOB_RECURSE FSIM_TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp)
  add_executable(flightsim_tests ${FSIM_TEST_SOURCES})
  target_link_libraries(flightsim_tests PRIVATE fsim_lib fsim_doctest)
  target_include_directories(flightsim_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
  fsim_set_warnings(flightsim_tests)
  add_dependencies(flightsim_tests fsim_runtime)
  # Test cases are named "<module>: ..."; the ones that fly JSBSim use the prefix "sim:".
  # Tests run with the build dir as working directory (that is where jsbsim/ and assets/ live).
  add_test(NAME unit_tests
           COMMAND flightsim_tests --test-case-exclude=sim:*
           WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
  add_test(NAME sim_tests
           COMMAND flightsim_tests --test-case=sim:*
           WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
  add_test(NAME headless_scenario
           COMMAND flightsim --headless --scenario assets/scenarios/c172_runway_xtst.ini
                   --duration 20 --trace ${CMAKE_BINARY_DIR}/ctest_trace.csv
           WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
endif()
