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

# # Grab mrcal
# include(FetchContent)
# FetchContent_Declare(
#   mrcal
#   GIT_REPOSITORY https://github.com/dkogan/mrcal
#   GIT_TAG b79b1d42f658ff5bdafe81bd2b0bacb4b5a62b86
# )
# FetchContent_MakeAvailable(mrcal)
# SET(MRCAL_INCLUDE_DIR ${mrcal_SOURCE_DIR})

# OpenCV
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
include_directories("${OpenCV_INCLUDE_DIRS}")

find_package(JNI)
if (JNI_FOUND)
    # Fixes odd AWT dependency
    set (JNI_INCLUDE_DIRS ${JAVA_INCLUDE_PATH} ${JAVA_INCLUDE_PATH2})
    message (STATUS "JNI_INCLUDE_DIRS=${JNI_INCLUDE_DIRS}")
endif()

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

# Create shared library
add_library(mrcal_jni SHARED ${INCLUDE_HPP} ${SRC_HPP} ${SRC_CPP})
target_include_directories(mrcal_jni PUBLIC ${JNI_INCLUDE_DIRS} )
target_include_directories(mrcal_jni PRIVATE ../mrcal)
target_link_libraries(mrcal_jni ${OpenCV_LIBS} /home/matt/github/mrcal/libmrcal.so)

SET_TARGET_PROPERTIES(mrcal_jni PROPERTIES LINK_FLAGS "-Wl,--version-script=\"${CMAKE_CURRENT_SOURCE_DIR}/linux/exports.txt\"")

# Test script for checking our linker
add_executable(mrcal_jni_test mrcal_test.cpp mrcal_wrapper.cpp)
target_link_libraries(mrcal_jni_test PUBLIC mrcal_jni)
target_include_directories(mrcal_jni_test PRIVATE ../mrcal)
target_link_libraries(mrcal_jni_test PRIVATE
    ${OpenCV_LIBS}
    /home/matt/github/mrcal/libmrcal.so
    /home/matt/github/vnlog/libvnlog.so
)

if (WITH_ASAN)
    target_link_libraries(mrcal_jni_test PRIVATE -fsanitize=address -fsanitize=undefined)
endif ()
