cmake_minimum_required(VERSION 3.18)
project(mrcal_jni LANGUAGES CXX VERSION 1.0.0)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set (WITH_ASAN OFF)

if (WITH_ASAN)
    add_compile_options(-fsanitize=address -g -Wall -fsanitize=undefined)
endif ()

# Grab vnlog
include(ExternalProject)
find_program(MAKE_EXECUTABLE NAMES gmake make mingw32-make REQUIRED)
ExternalProject_add(
  vnlog
  GIT_REPOSITORY https://github.com/dkogan/vnlog
  GIT_TAG HEAD
  UPDATE_DISCONNECTED true
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ${MAKE_EXECUTABLE} <SOURCE_DIR>
  INSTALL_COMMAND ""
  UPDATE_COMMAND ""
  #BUILD_BYPRODUCTS <SOURCE_DIR>/libvnlog.so
)
#FetchContent_MakeAvailable(vnlog)
ExternalProject_Get_property(vnlog SOURCE_DIR)
SET(VNLOG_INCLUDE_DIR ${SOURCE_DIR})
# message(STATUS MAKE IS ${MAKE_EXECUTABLE})

add_library(libvnlog SHARED IMPORTED)
set_target_properties(libvnlog PROPERTIES IMPORTED_LOCATION ${SOURCE_DIR}/libvnlog.so)


find_package(JNI)
if (JNI_FOUND)
    # Fixes odd AWT dependency
    set (JNI_INCLUDE_DIRS ${JAVA_INCLUDE_PATH} ${JAVA_INCLUDE_PATH2})
    message (STATUS "JNI_INCLUDE_DIRS=${JNI_INCLUDE_DIRS}")
endif()


set(OPENCV_YEAR "frc2023")
set(OPENCV_VERSION "4.6.0-5")
if(NOT OPENCV_ARCH)
    set(OPENCV_ARCH "linuxx86-64") # Set by Actions usually
endif()

# type can be "", "debug", "static", or "staticdebug"
set(OPENCV_TYPE "")

# Download opencv, and save the path
include(FetchContent)
FetchContent_Declare(
    opencv_lib
    URL https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/thirdparty/${OPENCV_YEAR}/opencv/opencv-cpp/${OPENCV_VERSION}/opencv-cpp-${OPENCV_VERSION}-${OPENCV_ARCH}${OPENCV_TYPE}.zip
)
FetchContent_MakeAvailable(opencv_lib)

# download OpenCV headers
FetchContent_Declare(
    opencv_header
    URL https://frcmaven.wpi.edu/artifactory/release/edu/wpi/first/thirdparty/${OPENCV_YEAR}/opencv/opencv-cpp/${OPENCV_VERSION}/opencv-cpp-${OPENCV_VERSION}-headers.zip
)
FetchContent_MakeAvailable(opencv_header)

# This probably doesn't work great with shared libraries, but I don't care about those right now
file(GLOB_RECURSE OPENCV_LIB_PATH
  "${opencv_lib_SOURCE_DIR}/**/*"
)
SET(OPENCV_INCLUDE_PATH ${opencv_header_SOURCE_DIR})


# headers and sources
set(
    INCLUDE_HPP
)

set(
    SRC_HPP
)
set(
    SRC_CPP
    mrcal_jni.h
    mrcal_wrapper.cpp
    mrcal_jni.cpp
)

# ignore opencv spam
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -Wno-deprecated-enum-enum-conversion")

# Create shared library
add_library(mrcal_jni SHARED ${INCLUDE_HPP} ${SRC_HPP} ${SRC_CPP})
target_include_directories(mrcal_jni PUBLIC ${JNI_INCLUDE_DIRS} ${OPENCV_INCLUDE_PATH})
target_link_libraries(mrcal_jni ${OPENCV_LIB_PATH} mrcal)

SET_TARGET_PROPERTIES(mrcal_jni PROPERTIES LINK_FLAGS "-Wl,--version-script=\"${CMAKE_CURRENT_SOURCE_DIR}/linux/exports.txt\"")

# Test script for checking our linker
add_executable(mrcal_jni_test mrcal_test.cpp mrcal_wrapper.cpp)
target_link_libraries(mrcal_jni_test PUBLIC mrcal_jni)
target_include_directories(mrcal_jni_test PRIVATE ${VNLOG_INCLUDE_DIR})
add_dependencies(mrcal_jni_test vnlog)
target_link_libraries(mrcal_jni_test PRIVATE
    ${OpenCV_LIBS}
    mrcal
    libvnlog
)

if (WITH_ASAN)
    target_link_libraries(mrcal_jni_test PRIVATE -fsanitize=address -fsanitize=undefined)
endif ()

unset(OPENCV_ARCH CACHE) # Set by Actions usually
