cmake_minimum_required(VERSION 3.16...3.31)
project(
  BananasArucoDetector
  VERSION 1.0
  DESCRIPTION "Try to find boxes using ArUco markers"
  LANGUAGES CXX)

# OpenCV depends on the deprecated FindCUDA for some reason
cmake_policy(SET CMP0146 OLD)

include(FetchContent)
FetchContent_Declare(
  GSL
  GIT_REPOSITORY "https://github.com/microsoft/GSL"
  GIT_TAG "v4.1.0"
  GIT_SHALLOW ON)
FetchContent_MakeAvailable(GSL)

FetchContent_Declare(
  tinygltf
  GIT_REPOSITORY "https://github.com/syoyo/tinygltf"
  GIT_TAG "a5e653e46cca64bbc0ecee7ede007347a94c39b0"
  GIT_SHALLOW ON)
FetchContent_MakeAvailable(tinygltf)

find_package(Eigen3 3.4 REQUIRED CONFIG)

find_package(
  OpenCV
  CONFIG
  REQUIRED
  core
  objdetect
  videoio
  highgui
  calib3d
  imgcodecs)

find_package(OGRE REQUIRED COMPONENTS Bites CONFIG)

# In case someone out there is still using a system with libm
find_library(MATH_LIBRARY m)

add_subdirectory(src)
add_subdirectory(apps)
