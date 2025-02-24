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
  URL "https://github.com/microsoft/GSL/archive/87f9d768866548b5b86e72be66c60c5abd4d9b37.zip")
FetchContent_MakeAvailable(GSL)

FetchContent_Declare(
  tinygltf
  URL https://github.com/syoyo/tinygltf/archive/a5e653e46cca64bbc0ecee7ede007347a94c39b0.zip)
FetchContent_MakeAvailable(tinygltf)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip)
FetchContent_MakeAvailable(googletest)

find_package(nlohmann_json 3.11.3 REQUIRED CONFIG)

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

add_subdirectory(src)
add_subdirectory(apps)

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  include(CTest)
endif()

if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
  include(GoogleTest)
  add_subdirectory(tests)
endif()
