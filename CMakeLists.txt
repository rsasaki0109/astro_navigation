cmake_minimum_required(VERSION 3.20)

project(
  astro_localization
  VERSION 0.1.0
  DESCRIPTION "Reproducible lunar visual localization and TRN building blocks"
  LANGUAGES CXX)

option(ASTRO_BUILD_APPS "Build command line applications" ON)
option(ASTRO_ENABLE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED COMPONENTS core features2d calib3d imgcodecs imgproc highgui)
find_package(Eigen3 REQUIRED)

add_library(astro_project_options INTERFACE)
target_compile_features(astro_project_options INTERFACE cxx_std_20)

add_library(astro_project_warnings INTERFACE)
target_compile_options(
  astro_project_warnings
  INTERFACE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic -Wconversion>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>)

if(ASTRO_ENABLE_WARNINGS_AS_ERRORS)
  target_compile_options(
    astro_project_warnings
    INTERFACE
      $<$<CXX_COMPILER_ID:GNU,Clang>:-Werror>
      $<$<CXX_COMPILER_ID:MSVC>:/WX>)
endif()

add_subdirectory(core)
add_subdirectory(localization)
add_subdirectory(terrain)
add_subdirectory(crater)
add_subdirectory(visualization)

if(ASTRO_BUILD_APPS)
  add_subdirectory(apps)
endif()

