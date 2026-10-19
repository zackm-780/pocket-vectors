# Third-party dependencies (PLAN.md decision D17): JSBSim, doctest, stb_image_write, SDL2.
include(FetchContent)

# ---- JSBSim v1.3.3, shared library (D3) --------------------------------------------------------
set(BUILD_PYTHON_MODULE    OFF CACHE BOOL "" FORCE)
set(BUILD_DOCS             OFF CACHE BOOL "" FORCE)
set(BUILD_JULIA_PACKAGE    OFF CACHE BOOL "" FORCE)
set(BUILD_MATLAB_SFUNCTION OFF CACHE BOOL "" FORCE)
set(CMAKE_DISABLE_FIND_PACKAGE_CxxTest ON)   # no JSBSim unit tests

FetchContent_Declare(jsbsim
  GIT_REPOSITORY https://github.com/JSBSim-Team/jsbsim.git
  GIT_TAG        v1.3.3
  GIT_SHALLOW    TRUE
  EXCLUDE_FROM_ALL)          # honoured by CMake >= 3.28; older versions use the fallback below

set(_fsim_saved_shared ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS ON)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.28)
  FetchContent_MakeAvailable(jsbsim)
else()
  FetchContent_GetProperties(jsbsim)
  if(NOT jsbsim_POPULATED)
    FetchContent_Populate(jsbsim)
    add_subdirectory(${jsbsim_SOURCE_DIR} ${jsbsim_BINARY_DIR} EXCLUDE_FROM_ALL)
  endif()
endif()
set(BUILD_SHARED_LIBS ${_fsim_saved_shared})

# Wrapper target: JSBSim's headers are SYSTEM includes so its warnings never reach our -Werror.
get_target_property(_jsb_inc libJSBSim INTERFACE_INCLUDE_DIRECTORIES)
set_target_properties(libJSBSim PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_jsb_inc}")
add_library(fsim_jsbsim INTERFACE)
target_link_libraries(fsim_jsbsim INTERFACE libJSBSim)
set(FSIM_JSBSIM_SOURCE_DIR ${jsbsim_SOURCE_DIR})

# ---- single-header deps, pinned by URL + SHA256 ------------------------------------------------
FetchContent_Declare(doctest_hdr
  URL      https://raw.githubusercontent.com/doctest/doctest/v2.4.11/doctest/doctest.h
  URL_HASH SHA256=44faa038e9c3f9728efbda143748d01124ea0a27f4bf78f35a15d8fab2e039fb
  DOWNLOAD_NO_EXTRACT TRUE)
FetchContent_Declare(stb_hdr
  URL      https://raw.githubusercontent.com/nothings/stb/2c980bb59875b0d32144a71867fbdebb2f77cd20/stb_image_write.h
  URL_HASH SHA256=cbd5f0ad7a9cf4468affb36354a1d2338034f2c12473cf1a8e32053cb6914a05
  DOWNLOAD_NO_EXTRACT TRUE)
FetchContent_MakeAvailable(doctest_hdr stb_hdr)

# Expose the headers under stable sub-paths: #include <doctest/doctest.h>, <stb/stb_image_write.h>.
configure_file(${doctest_hdr_SOURCE_DIR}/doctest.h
               ${CMAKE_BINARY_DIR}/third_party/doctest/doctest.h COPYONLY)
configure_file(${stb_hdr_SOURCE_DIR}/stb_image_write.h
               ${CMAKE_BINARY_DIR}/third_party/stb/stb_image_write.h COPYONLY)
add_library(fsim_doctest INTERFACE)
target_include_directories(fsim_doctest SYSTEM INTERFACE ${CMAKE_BINARY_DIR}/third_party)
add_library(fsim_stb INTERFACE)
target_include_directories(fsim_stb SYSTEM INTERFACE ${CMAKE_BINARY_DIR}/third_party)

# ---- SDL2 (system package; libsdl2-dev) --------------------------------------------------------
find_package(SDL2 REQUIRED CONFIG)
if(NOT TARGET SDL2::SDL2)   # some older SDL2 config packages only provide variables
  add_library(SDL2::SDL2 INTERFACE IMPORTED)
  target_include_directories(SDL2::SDL2 INTERFACE ${SDL2_INCLUDE_DIRS})
  target_link_libraries(SDL2::SDL2 INTERFACE ${SDL2_LIBRARIES})
endif()
