cmake_minimum_required(VERSION 3.24)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(mrcal_jni LANGUAGES C CXX VERSION 1.0.0)

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# C++
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(WITH_ASAN OFF)
if(WITH_ASAN)
    add_compile_options(-fsanitize=address -g -Wall -fsanitize=undefined)
endif()

# GCC-specific flags to catch uninitialized variables and undefined behavior
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    add_compile_options(-Wuninitialized -Werror=uninitialized -fno-common)
endif()

# Include FetchContent module for downloading dependencies
include(FetchContent)

# OpenCV configuration
# Keep this in sync with build.gradle and with
# https://github.com/PhotonVision/photonvision/blob/main/build.gradle
set(OPENCV_VERSION "2027-4.13.0-3")
set(WPIMATH_VERSION "2027.0.0-alpha-5")

# type can be "", "debug", "static", or "staticdebug"
set(OPENCV_TYPE "")

message(STATUS "Using FRC OpenCV for architecture: ${OPENCV_ARCH}")

FetchContent_Declare(
    opencv_lib
    URL
        https://frcmaven.wpi.edu/artifactory/release/org/wpilib/thirdparty/opencv/opencv-cpp/${OPENCV_VERSION}/opencv-cpp-${OPENCV_VERSION}-${OPENCV_ARCH}${OPENCV_TYPE}.zip
)
FetchContent_MakeAvailable(opencv_lib)

# download OpenCV headers
FetchContent_Declare(
    opencv_header
    URL
        https://frcmaven.wpi.edu/artifactory/release/org/wpilib/thirdparty/opencv/opencv-cpp/${OPENCV_VERSION}/opencv-cpp-${OPENCV_VERSION}-headers.zip
)
FetchContent_MakeAvailable(opencv_header)

file(
    GLOB_RECURSE OPENCV_LIB_PATH
    "${opencv_lib_SOURCE_DIR}/**/*.lib"
    "${opencv_lib_SOURCE_DIR}/**/*.so*"
    "${opencv_lib_SOURCE_DIR}/**/*.*.dylib"
)
set(OPENCV_INCLUDE_PATH ${opencv_header_SOURCE_DIR})
set(OPENCV_LIBRARIES ${OPENCV_LIB_PATH})
message(STATUS "Using FRC OpenCV libraries: ${OPENCV_LIB_PATH}")

# Also download wpimath
FetchContent_Declare(
    wpimath_header
    URL
        https://frcmaven.wpi.edu/artifactory/release-2027/org/wpilib/wpimath/wpimath-cpp/${WPIMATH_VERSION}/wpimath-cpp-${WPIMATH_VERSION}-headers.zip
)
FetchContent_MakeAvailable(wpimath_header)
set(WPIMATH_INCLUDE_PATH ${wpimath_header_SOURCE_DIR})

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

# Silence OpenBLAS/LAPACK warnings
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -w")

if(NOT APPLE)
    set(BLA_VENDOR OpenBLAS)

    if(ARM64_BUILD)
        # ----------------------------------------------------------------
        # Variables for subdirectory builds must be set in the parent cache 
        # BEFORE FetchContent_MakeAvailable.
        #
        # We also force CMAKE_CROSSCOMPILING so OpenBLAS skips its getarch
        # host-CPU detection (which would detect the build machine's CPU,
        # potentially a Neoverse/SVE host, and silently override TARGET).
        # Without this, TARGET=ARMV8 is ignored when not cross-compiling.
        # ----------------------------------------------------------------

        # Force OpenBLAS to treat this as a cross-compile so it respects TARGET
        # and does not run getarch to auto-detect the build host's CPU.
        set(CMAKE_CROSSCOMPILING TRUE CACHE BOOL "" FORCE)

        # TARGET=ARMV8 selects the generic AArch64 kernel (NEON/ASIMD only).
        # getarch2.c confirms ARMV8 sets: HAVE_NEON, HAVE_VFP, HAVE_VFPV3,
        # HAVE_VFPV4 — and critically does NOT set HAVE_SVE.
        # Do NOT use: ARMV8SVE, NEOVERSEN1, NEOVERSEN2, NEOVERSEV1, CORTEXA510,
        # CORTEXX2, ARMV9 — all of those enable SVE kernels.
        set(TARGET "ARMV8" CACHE STRING "" FORCE)

        # Disable dynamic multi-arch dispatch. If left ON, OpenBLAS compiles
        # kernels for every supported core including SVE targets and picks one
        # at runtime — your RK3588 could still land on an SVE kernel.
        set(DYNAMIC_ARCH OFF CACHE BOOL "" FORCE)
        set(DYNAMIC_OLDER OFF CACHE BOOL "" FORCE)

        # Lock the compiler to ARMv8-A baseline. This prevents the compiler
        # from emitting SVE intrinsics even if OpenBLAS assembly kernels
        # happen to include SVE .S files guarded by preprocessor.
        set(CMAKE_C_FLAGS_OPENBLAS "-march=armv8-a" CACHE STRING "" FORCE)
        set(COMMON_OPT "-march=armv8-a" CACHE STRING "" FORCE)

        # Explicitly set ARCH so OpenBLAS does not re-detect it.
        set(ARCH "aarch64" CACHE STRING "" FORCE)
    endif()

    FetchContent_Declare(
        BLAS
        GIT_REPOSITORY https://github.com/OpenMathLib/OpenBLAS.git
        GIT_TAG v0.3.30
        OVERRIDE_FIND_PACKAGE
        # NOTE: CMAKE_ARGS is intentionally absent — it does nothing for
        # FetchContent subdirectory builds. All config is via cache vars above.
    )
    FetchContent_MakeAvailable(BLAS)

    if(TARGET openblas)
        set(OPENBLAS_TARGET openblas)
        message(STATUS "Using openblas target")
    elseif(TARGET OpenBLAS)
        set(OPENBLAS_TARGET OpenBLAS)
        message(STATUS "Using OpenBLAS target")
    endif()
else()
    set(BLA_VENDOR Apple)
    add_compile_definitions(ACCELERATE_NEW_LAPACK=1)
    find_package(BLAS)
endif()

# Configure SuiteSparse to use BLAS
set(SUITESPARSE_ENABLE_PROJECTS cholmod)
set(SUITESPARSE_USE_CUDA OFF)
set(SUITESPARSE_USE_FORTRAN OFF)
set(SUITESPARSE_USE_OPENMP OFF)
set(SUITESPARSE_USE_PYTHON OFF)
set(CHOLMOD_GPL OFF)
set(SUITESPARSE_USE_BLAS ON)

FetchContent_Declare(
    SuiteSparse
    GIT_REPOSITORY https://github.com/DrTimothyAldenDavis/SuiteSparse.git
    GIT_TAG v7.11.0
    OVERRIDE_FIND_PACKAGE
)
FetchContent_MakeAvailable(SuiteSparse)

get_target_property(
    CHOLMOD_LIBS
    SuiteSparse::CHOLMOD_static
    INTERFACE_LINK_LIBRARIES
)
message(STATUS "CHOLMOD links to: ${CHOLMOD_LIBS}")
if(NOT APPLE)
    message(STATUS "BLAS is at: ${OPENBLAS_TARGET}")
else()
    message(STATUS "BLAS is using Apple Accelerate")
endif()

# And pull in JNI
find_package(JNI REQUIRED COMPONENTS JVM)

# headers and sources
set(INCLUDE_HPP)

set(SRC_HPP)
set(SRC_CPP
    src/mrcal_jni.h
    src/mrcal_wrapper.cpp
    src/mrcal-uncertainty.cpp
    src/mrcal_jni.cpp
    libdogleg/dogleg.c
    mrcal/mrcal.c
    mrcal/cahvore.cc
    mrcal/poseutils-opencv.c
    mrcal/poseutils-uses-autodiff.cc
    mrcal/poseutils.c
    mrcal/poseutils-opencv.c
    mrcal/opencv.c
    mrcal/triangulation.cc
)

# ignore opencv spam
# set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Werror -Wno-deprecated-enum-enum-conversion")

# Gennerate minimath
find_package(Perl REQUIRED)
add_custom_target(
    generate_minimath
    ALL
    COMMAND
        ${PERL_EXECUTABLE}
        ${CMAKE_CURRENT_SOURCE_DIR}/mrcal/minimath/minimath_generate.pl >
        ${CMAKE_CURRENT_SOURCE_DIR}/mrcal/minimath/minimath_generated.h
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    BYPRODUCTS ${CMAKE_CURRENT_SOURCE_DIR}/mrcal/minimath/minimath_generated.h
    COMMENT "Generating minimath"
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Create shared library
add_library(mrcal_jni SHARED ${INCLUDE_HPP} ${SRC_HPP} ${SRC_CPP})
target_include_directories(
    mrcal_jni
    SYSTEM
    PUBLIC
        ${JNI_INCLUDE_DIRS}
        ${OPENCV_INCLUDE_PATH}
        mrcal
        libdogleg
        ${WPIMATH_INCLUDE_PATH}
)
add_dependencies(mrcal_jni generate_minimath)

target_link_libraries(
    mrcal_jni
    PUBLIC
        ${OPENCV_LIBRARIES}
        SuiteSparse::CHOLMOD_static
        SuiteSparse::SuiteSparseConfig_static
        ${OPENBLAS_TARGET}
        lapack
)

# vnlog for the test script
set(VNLOG_SRC_CPP vnlog/b64_cencode.c vnlog/vnlog-parser.c vnlog/vnlog.c)
add_library(vnlog STATIC ${VNLOG_SRC_CPP})
target_include_directories(vnlog SYSTEM PUBLIC ${PROJECT_SOURCE_DIR}/vnlog)

# Test script for checking our linker

add_executable(mrcal_jni_test src/mrcal_test.cpp)

target_link_libraries(mrcal_jni_test PUBLIC mrcal_jni)

target_include_directories(
    mrcal_jni_test
    SYSTEM
    PRIVATE ${PROJECT_SOURCE_DIR}/vnlog
)
add_dependencies(mrcal_jni_test mrcal_jni vnlog)
target_link_libraries(mrcal_jni_test PRIVATE ${OPENCV_LIBRARIES} vnlog)

if(WITH_ASAN)
    target_link_libraries(
        mrcal_jni_test
        PRIVATE -fsanitize=address -fsanitize=undefined
    )
endif()

unset(OPENCV_ARCH CACHE) # Set by Actions usually
