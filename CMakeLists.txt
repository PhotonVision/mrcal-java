cmake_minimum_required(VERSION 3.18)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(mrcal_jni LANGUAGES C CXX VERSION 1.0.0)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set (WITH_ASAN OFF)

if (WITH_ASAN)
    add_compile_options(-fsanitize=address -g -Wall -fsanitize=undefined)
endif ()

find_package(JNI)
if (JNI_FOUND)
    # Fixes odd AWT dependency
    set (JNI_INCLUDE_DIRS ${JAVA_INCLUDE_PATH} ${JAVA_INCLUDE_PATH2})
    message (STATUS "JNI_INCLUDE_DIRS=${JNI_INCLUDE_DIRS}")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Keep this in sync with build.gradle and with 
# https://github.com/PhotonVision/photonvision/blob/main/build.gradle
set(OPENCV_YEAR "frc2025")
set(OPENCV_VERSION "4.10.0-3")

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
  "${opencv_lib_SOURCE_DIR}/**/*.lib"
  "${opencv_lib_SOURCE_DIR}/**/*.so*"
)
SET(OPENCV_INCLUDE_PATH ${opencv_header_SOURCE_DIR})
message("Depending on opencv ${OPENCV_LIB_PATH}")

# headers and sources
set(
    INCLUDE_HPP
)

set(
    SRC_HPP
)
set(
    SRC_CPP
    src/mrcal_jni.h
    src/mrcal_wrapper.cpp
    src/mrcal_jni.cpp
    libdogleg/dogleg.c
    mrcal/mrcal.c mrcal/cahvore.cc mrcal/poseutils-opencv.c mrcal/poseutils-uses-autodiff.cc mrcal/poseutils.c mrcal/poseutils-opencv.c mrcal/mrcal-opencv.c mrcal/triangulation.cc
)


# ignore opencv spam
# set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -Wno-deprecated-enum-enum-conversion")


# Gennerate minimath
find_package( Perl REQUIRED )
add_custom_target(
 generate_minimath ALL
 COMMAND ${PERL_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/mrcal/minimath/minimath_generate.pl > ${CMAKE_CURRENT_SOURCE_DIR}/mrcal/minimath/minimath_generated.h
 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
 BYPRODUCTS ${CMAKE_CURRENT_SOURCE_DIR}/mrcal/minimath/minimath_generated.h
 COMMENT "Generating minimath"
)

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib )

# Create shared library
add_library(mrcal_jni SHARED ${INCLUDE_HPP} ${SRC_HPP} ${SRC_CPP})
target_include_directories(mrcal_jni SYSTEM PUBLIC ${JNI_INCLUDE_DIRS} ${OPENCV_INCLUDE_PATH} mrcal libdogleg)
add_dependencies(mrcal_jni generate_minimath)

IF (WIN32)
    find_package(CHOLMOD REQUIRED)
    find_package(SuiteSparse_config REQUIRED)
    
    target_link_libraries(mrcal_jni 
        ${OPENCV_LIB_PATH}
        SuiteSparse::CHOLMOD suitesparseconfig lapack openblas)

    # Add SuiteSparse include directories
    target_include_directories(mrcal_jni PUBLIC "${CMAKE_BINARY_DIR}/vcpkg_installed/x64-windows/include/")
    target_include_directories(mrcal_jni PUBLIC "${CMAKE_BINARY_DIR}/vcpkg_installed/x64-windows/include/suitesparse")

    # Another hack wtf
    target_link_directories(mrcal_jni PRIVATE "${CMAKE_BINARY_DIR}/vcpkg_installed/x64-windows/lib")
ELSE()
    target_link_libraries(mrcal_jni ${OPENCV_LIB_PATH} suitesparseconfig cholmod)
    target_include_directories(mrcal_jni PUBLIC "/usr/include/suitesparse")
ENDIF()


# Test script for checking our linker
add_executable(mrcal_jni_test src/mrcal_test.cpp)
target_link_libraries(mrcal_jni_test PUBLIC mrcal_jni)
target_include_directories(mrcal_jni_test SYSTEM PRIVATE ${PROJECT_SOURCE_DIR}/vnlog)
# add_dependencies(mrcal_jni_test )
target_link_libraries(mrcal_jni_test PRIVATE
    ${OpenCV_LIBS}
    ${PROJECT_SOURCE_DIR}/vnlog/libvnlog.so
)

if (WITH_ASAN)
    target_link_libraries(mrcal_jni_test PRIVATE -fsanitize=address -fsanitize=undefined)
endif ()

unset(OPENCV_ARCH CACHE) # Set by Actions usually
