cmake_minimum_required(VERSION 3.16)
project(Kinetix LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

option(KINETIX_BUILD_2D "" ON)
option(KINETIX_BUILD_3D "" ON)
option(KINETIX_BUILD_AI "" ON)
option(KINETIX_BUILD_TESTBED "Visual/headless physics testbed (needs SDL2)" ON)
option(KINETIX_BUILD_TESTS "Regression tests (CTest)" OFF)

add_library(ct INTERFACE)
target_include_directories(ct INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/external/containers/include)

set(MATHC_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MATHC_BUILD_BENCH OFF CACHE BOOL "" FORCE)
add_subdirectory(external/math)

add_subdirectory(core)

if(KINETIX_BUILD_2D)
    add_subdirectory(2d)
endif()

if(KINETIX_BUILD_3D)
    add_subdirectory(3d)
endif()

if(KINETIX_BUILD_AI)
    add_subdirectory(ai)
endif()

if(KINETIX_BUILD_TESTBED)
    add_subdirectory(testbed)
endif()

if(KINETIX_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
