cmake_minimum_required(VERSION 3.18)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(mrcal_jni LANGUAGES C CXX VERSION 1.0.0)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set (WITH_ASAN OFF)
if (WITH_ASAN)
    add_compile_options(-fsanitize=address -g -Wall -fsanitize=undefined)
endif ()

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

# Openblas/suitesparse/friends
# From https://github.com/wpilibsuite/thirdparty-ceres/blob/main/CMakeLists.txt
set(BUILD_SHARED_LIBS OFF)
set(BUILD_TESTING OFF)
set(BUILD_EXAMPLES OFF)

set(NO_CBLAS ON)
set(NO_LAPACKE ON)
set(C_LAPACK ON)
set(USE_LOCKING ON)
set(USE_THREAD OFF)
set(NOFORTRAN ON)

# We need PIC for this to work as a shared library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(BLA_VENDOR OpenBLAS)
fetchcontent_declare(
    BLAS
    GIT_REPOSITORY https://github.com/OpenMathLib/OpenBLAS.git
    GIT_TAG v0.3.30
    OVERRIDE_FIND_PACKAGE
)
fetchcontent_makeavailable(BLAS)

if(TARGET openblas)
    set(OPENBLAS_TARGET openblas)
    message(STATUS "Using openblas target")
elseif(TARGET OpenBLAS)
    set(OPENBLAS_TARGET OpenBLAS)
    message(STATUS "Using OpenBLAS target")
elseif(TARGET libopenblas)
    set(OPENBLAS_TARGET libopenblas)
    message(STATUS "Using libopenblas target")
else()
    # Fallback: find the library file manually
    find_library(OPENBLAS_LIB
        NAMES openblas libopenblas
        PATHS 
            ${openblas_external_BINARY_DIR}
            ${openblas_external_BINARY_DIR}/lib
            ${openblas_external_BINARY_DIR}/lib/Release
        NO_DEFAULT_PATH
        REQUIRED
    )
    message(STATUS "Using OpenBLAS library file: ${OPENBLAS_LIB}")
    add_library(openblas_imported STATIC IMPORTED)
    set_target_properties(openblas_imported PROPERTIES IMPORTED_LOCATION ${OPENBLAS_LIB})
    set(OPENBLAS_TARGET openblas_imported)
endif()

# Configure SuiteSparse to use BLAS
set(SUITESPARSE_ENABLE_PROJECTS cholmod)
set(SUITESPARSE_USE_CUDA OFF)
set(SUITESPARSE_USE_FORTRAN OFF)
set(SUITESPARSE_USE_OPENMP OFF)
set(SUITESPARSE_USE_PYTHON OFF)
set(CHOLMOD_GPL OFF)
set(SUITESPARSE_USE_BLAS ON)

fetchcontent_declare(
    SuiteSparse
    GIT_REPOSITORY https://github.com/DrTimothyAldenDavis/SuiteSparse.git
    GIT_TAG v7.11.0
    OVERRIDE_FIND_PACKAGE
)
fetchcontent_makeavailable(SuiteSparse)

get_target_property(CHOLMOD_LIBS SuiteSparse::CHOLMOD_static INTERFACE_LINK_LIBRARIES)
message(STATUS "CHOLMOD links to: ${CHOLMOD_LIBS}")
message(STATUS "BLAS is at: ${OPENBLAS_TARGET}")

# And pull in JNI
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

target_link_libraries(mrcal_jni 
    ${OPENCV_LIB_PATH}
    SuiteSparse::CHOLMOD_static
    SuiteSparse::SuiteSparseConfig_static
    ${OPENBLAS_TARGET}
    lapack
)

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
