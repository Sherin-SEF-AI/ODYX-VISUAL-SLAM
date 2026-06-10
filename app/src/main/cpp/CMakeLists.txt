# ===========================================================================
# ODYX native estimator :: libodyx.so
#
# Layer C of the architecture. Cross-compiled for arm64-v8a via the NDK
# toolchain (Gradle externalNativeBuild passes the toolchain + ABI).
#
# Dependencies (all permissive licenses — see README LICENSE INVENTORY):
#   Eigen 3.4.0   MPL2     header-only        third_party/eigen
#   GTSAM 4.2.0   BSD-3    ExternalProject    third_party/gtsam   -> static
#   Boost 1.84    BSL-1.0  header-only subset  third_party/boost
#   DBoW3         BSD-3    ExternalProject    third_party/DBoW3   -> static
#   RTKLIB 2.4.3  BSD-2    compiled in-tree    third_party/rtklib (subset)
#   OpenCV 4.10   Apache-2 official Android SDK (prebuilt, linked)
#
# NO GPL code. The estimator algorithms are reimplemented from publications.
# ===========================================================================
cmake_minimum_required(VERSION 3.22.1)
project(odyx LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT DEFINED ODYX_NATIVE_JOBS)
    set(ODYX_NATIVE_JOBS 4)
endif()

set(TP "${CMAKE_CURRENT_SOURCE_DIR}/third_party")

# ---------------------------------------------------------------------------
# Sanity checks on fetched sources.
# ---------------------------------------------------------------------------
foreach(dep eigen gtsam boost DBoW3 rtklib)
    if(NOT EXISTS "${TP}/${dep}")
        message(FATAL_ERROR
            "Missing third_party/${dep}. Run scripts/fetch_deps.sh first.")
    endif()
endforeach()

# Eigen is installed (header-only) so GTSAM's find_package(Eigen3 CONFIG) works.
set(EIGEN_INCLUDE_DIR "${TP}/eigen-install/include/eigen3")
set(EIGEN3_CONFIG_DIR "${TP}/eigen-install/share/eigen3/cmake")
if(NOT EXISTS "${EIGEN3_CONFIG_DIR}/Eigen3Config.cmake")
    message(FATAL_ERROR
        "Eigen not installed. Run: cmake -S third_party/eigen -B /tmp/eigen-build "
        "-DCMAKE_INSTALL_PREFIX=third_party/eigen-install && cmake --install /tmp/eigen-build")
endif()
set(BOOST_INCLUDE_DIR "${TP}/boost")

# GTSAM 4.2 requires COMPILED Boost components. They are cross-compiled for
# arm64 by scripts/build_boost_android.sh into boost-android/{include,lib}.
set(BOOST_ANDROID "${TP}/boost-android")
if(NOT EXISTS "${BOOST_ANDROID}/lib/libboost_serialization.a")
    message(FATAL_ERROR
        "Cross-compiled Boost not found. Run scripts/build_boost_android.sh first.")
endif()
# Static Boost libs that GTSAM pulls in (link order: after gtsam).
set(BOOST_ANDROID_LIBS
    "${BOOST_ANDROID}/lib/libboost_serialization.a"
    "${BOOST_ANDROID}/lib/libboost_filesystem.a"
    "${BOOST_ANDROID}/lib/libboost_thread.a"
    "${BOOST_ANDROID}/lib/libboost_date_time.a"
    "${BOOST_ANDROID}/lib/libboost_timer.a"
    "${BOOST_ANDROID}/lib/libboost_chrono.a"
    "${BOOST_ANDROID}/lib/libboost_regex.a"
    "${BOOST_ANDROID}/lib/libboost_system.a"
    "${BOOST_ANDROID}/lib/libboost_atomic.a")

# ---------------------------------------------------------------------------
# OpenCV (official Android SDK, prebuilt). ODYX_OPENCV_SDK points at
# <sdk>/sdk/native/jni which contains OpenCVConfig.cmake.
# ---------------------------------------------------------------------------
if(NOT DEFINED ODYX_OPENCV_SDK OR ODYX_OPENCV_SDK STREQUAL "")
    message(FATAL_ERROR "ODYX_OPENCV_SDK not set (see gradle.properties odyx.opencv.sdk).")
endif()
set(OpenCV_DIR "${ODYX_OPENCV_SDK}")
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc features2d calib3d video flann)
message(STATUS "ODYX: OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")

# Forward the NDK toolchain settings to the ExternalProject sub-builds so they
# cross-compile identically to this build.
set(XC_TOOLCHAIN_ARGS
    "-DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}"
    "-DANDROID_ABI=${ANDROID_ABI}"
    "-DANDROID_PLATFORM=${ANDROID_PLATFORM}"
    "-DANDROID_STL=${ANDROID_STL}"
    "-DCMAKE_BUILD_TYPE=Release"
    "-DCMAKE_POSITION_INDEPENDENT_CODE=ON"
    # Propagate the Ninja binary AGP uses; the sub-builds inherit the Ninja
    # generator but not CMAKE_MAKE_PROGRAM, so pass it explicitly.
    "-DCMAKE_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}"
)

# Force the ExternalProjects to use the same generator as this build.
set(XC_GENERATOR "${CMAKE_GENERATOR}")

include(ExternalProject)

# ---------------------------------------------------------------------------
# GTSAM 4.2.0 — static cross-compile. Highest-risk dependency.
#
# Flag rationale:
#   GTSAM_BUILD_WITH_MARCH_NATIVE=OFF  -> march=native is invalid when cross
#                                          compiling and breaks the build.
#   GTSAM_USE_SYSTEM_EIGEN=ON          -> use our fetched Eigen 3.4 (matches
#                                          the one we link), avoid GTSAM's copy.
#   GTSAM_ENABLE_BOOST_SERIALIZATION=OFF
#   GTSAM_USE_BOOST_FEATURES=OFF       -> minimise the Boost dependency to the
#                                          header subset (no Boost libs to build).
#   GTSAM_WITH_TBB=OFF                 -> no TBB on Android; single/std-thread.
# ---------------------------------------------------------------------------
set(GTSAM_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/gtsam-build")
set(GTSAM_INSTALL "${GTSAM_PREFIX}/install")

ExternalProject_Add(gtsam_ext
    SOURCE_DIR "${TP}/gtsam"
    PREFIX "${GTSAM_PREFIX}"
    INSTALL_DIR "${GTSAM_INSTALL}"
    CMAKE_GENERATOR "${XC_GENERATOR}"
    CMAKE_ARGS
        ${XC_TOOLCHAIN_ARGS}
        "-DCMAKE_INSTALL_PREFIX=${GTSAM_INSTALL}"
        "-DGTSAM_USE_SYSTEM_EIGEN=ON"
        "-DEIGEN3_INCLUDE_DIR=${EIGEN_INCLUDE_DIR}"
        "-DEigen3_DIR=${EIGEN3_CONFIG_DIR}"
        "-DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=BOTH"
        # cross-compiled static Boost
        "-DBOOST_ROOT=${BOOST_ANDROID}"
        "-DBOOST_INCLUDEDIR=${BOOST_ANDROID}/include"
        "-DBOOST_LIBRARYDIR=${BOOST_ANDROID}/lib"
        "-DBoost_USE_STATIC_LIBS=ON"
        "-DBoost_NO_SYSTEM_PATHS=ON"
        "-DBoost_NO_BOOST_CMAKE=ON"
        # let FindBoost see libs/headers outside the NDK sysroot
        "-DCMAKE_FIND_ROOT_PATH_MODE_LIBRARY=BOTH"
        "-DCMAKE_FIND_ROOT_PATH_MODE_INCLUDE=BOTH"
        "-DGTSAM_WITH_TBB=OFF"
        "-DGTSAM_BUILD_TESTS=OFF"
        "-DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF"
        # IncrementalFixedLagSmoother lives in gtsam_unstable -> must be ON.
        "-DGTSAM_BUILD_UNSTABLE=ON"
        "-DGTSAM_UNSTABLE_BUILD_PYTHON=OFF"
        "-DGTSAM_BUILD_DOCS=OFF"
        "-DGTSAM_BUILD_PYTHON=OFF"
        "-DGTSAM_BUILD_STATIC_LIBRARY=ON"
        "-DBUILD_SHARED_LIBS=OFF"
        "-DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF"
        "-DGTSAM_WITH_EIGEN_MKL=OFF"
        "-DGTSAM_ENABLE_BOOST_SERIALIZATION=OFF"
        "-DGTSAM_USE_BOOST_FEATURES=OFF"
        "-DGTSAM_USE_QUATERNIONS=OFF"
        "-DGTSAM_POSE3_EXPMAP=ON"
        "-DGTSAM_ROT3_EXPMAP=ON"
        "-DGTSAM_DT_MERGING=ON"
        "-DGTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR=ON"
    BUILD_BYPRODUCTS
        "${GTSAM_INSTALL}/lib/libgtsam.a"
        "${GTSAM_INSTALL}/lib/libgtsam_unstable.a"
        "${GTSAM_INSTALL}/lib/libmetis-gtsam.a"
    BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release -j ${ODYX_NATIVE_JOBS}
)

# Imported targets for GTSAM + unstable (FixedLagSmoother) + bundled metis.
add_library(gtsam STATIC IMPORTED GLOBAL)
set_target_properties(gtsam PROPERTIES
    IMPORTED_LOCATION "${GTSAM_INSTALL}/lib/libgtsam.a")
add_dependencies(gtsam gtsam_ext)

add_library(gtsam_unstable STATIC IMPORTED GLOBAL)
set_target_properties(gtsam_unstable PROPERTIES
    IMPORTED_LOCATION "${GTSAM_INSTALL}/lib/libgtsam_unstable.a")
add_dependencies(gtsam_unstable gtsam_ext)

add_library(metis-gtsam STATIC IMPORTED GLOBAL)
set_target_properties(metis-gtsam PROPERTIES
    IMPORTED_LOCATION "${GTSAM_INSTALL}/lib/libmetis-gtsam.a")
add_dependencies(metis-gtsam gtsam_ext)

set(GTSAM_INCLUDE_DIRS
    "${GTSAM_INSTALL}/include"
    "${EIGEN_INCLUDE_DIR}"
    "${BOOST_INCLUDE_DIR}")

# ---------------------------------------------------------------------------
# DBoW3 — static cross-compile (needs OpenCV).
# ---------------------------------------------------------------------------
set(DBOW3_PREFIX "${CMAKE_CURRENT_BINARY_DIR}/dbow3-build")
set(DBOW3_INSTALL "${DBOW3_PREFIX}/install")

ExternalProject_Add(dbow3_ext
    SOURCE_DIR "${TP}/DBoW3"
    PREFIX "${DBOW3_PREFIX}"
    INSTALL_DIR "${DBOW3_INSTALL}"
    CMAKE_GENERATOR "${XC_GENERATOR}"
    CMAKE_ARGS
        ${XC_TOOLCHAIN_ARGS}
        "-DCMAKE_INSTALL_PREFIX=${DBOW3_INSTALL}"
        "-DBUILD_SHARED_LIBS=OFF"
        "-DBUILD_UTILS=OFF"
        "-DUSE_CONTRIB=OFF"
        "-DOpenCV_DIR=${OpenCV_DIR}"
    BUILD_BYPRODUCTS "${DBOW3_INSTALL}/lib/libDBoW3.a"
    BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release -j ${ODYX_NATIVE_JOBS}
)

add_library(DBoW3 STATIC IMPORTED GLOBAL)
set_target_properties(DBoW3 PROPERTIES
    IMPORTED_LOCATION "${DBOW3_INSTALL}/lib/libDBoW3.a")
add_dependencies(DBoW3 dbow3_ext)
set(DBOW3_INCLUDE_DIR "${DBOW3_INSTALL}/include")

# ---------------------------------------------------------------------------
# RTKLIB (BSD-2) — compiled in-tree as a static object subset. RTKLIB has no
# first-class CMake; we compile only the translation units the tight-coupled
# GNSS path needs (common math, ephemeris -> sat pos/clock, RINEX-free path).
# ---------------------------------------------------------------------------
set(RTKLIB_SRC "${TP}/rtklib/src")
set(RTKLIB_SOURCES
    "${RTKLIB_SRC}/rtkcmn.c"
    "${RTKLIB_SRC}/ephemeris.c"
    "${RTKLIB_SRC}/preceph.c"
    "${RTKLIB_SRC}/pntpos.c"
    "${RTKLIB_SRC}/rcvraw.c"
    "${RTKLIB_SRC}/sbas.c"
    "${RTKLIB_SRC}/ionex.c"
    "${RTKLIB_SRC}/tides.c"
    "${RTKLIB_SRC}/geoid.c"
)
# Only include the ones that exist (RTKLIB layout varies by tag).
set(RTKLIB_EXISTING "")
foreach(s ${RTKLIB_SOURCES})
    if(EXISTS "${s}")
        list(APPEND RTKLIB_EXISTING "${s}")
    endif()
endforeach()
if(NOT RTKLIB_EXISTING)
    message(FATAL_ERROR "No RTKLIB sources found under ${RTKLIB_SRC}. Check fetch_deps.sh.")
endif()

add_library(rtklib STATIC ${RTKLIB_EXISTING}
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/rtklib_rcv_stubs.c")
target_include_directories(rtklib PUBLIC "${RTKLIB_SRC}")
target_compile_definitions(rtklib PRIVATE
    TRACE              # enable but we route trace to no-op
    ENAGLO ENAGAL ENAQZS ENACMP   # enable GNSS constellations
    NFREQ=3 NEXOBS=0)
target_compile_options(rtklib PRIVATE
    -w                 # RTKLIB is C89-ish; silence its warnings
    -Wno-implicit-function-declaration)

# ---------------------------------------------------------------------------
# ODYX native core library.
# ---------------------------------------------------------------------------
file(GLOB_RECURSE ODYX_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/common/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/core/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/sync/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/frontend/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/init/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/backend/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/gnss/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/loop/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/map/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/reloc/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/jni/*.cpp"
)

add_library(odyx SHARED ${ODYX_SOURCES})

target_include_directories(odyx PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    ${GTSAM_INCLUDE_DIRS}
    "${DBOW3_INCLUDE_DIR}"
    "${RTKLIB_SRC}"
    ${OpenCV_INCLUDE_DIRS}
)

target_compile_options(odyx PRIVATE
    -O3 -ffast-math -funsafe-math-optimizations
    -Wall -Wextra -Wno-unused-parameter)

# NEON is baseline on arm64-v8a; expose a macro for hand-tuned paths.
target_compile_definitions(odyx PRIVATE ODYX_HAVE_NEON=1)

find_library(log-lib log)
find_library(android-lib android)

target_link_libraries(odyx
    PRIVATE
        # static link order: dependents before dependencies
        gtsam_unstable
        gtsam
        metis-gtsam
        ${BOOST_ANDROID_LIBS}
        DBoW3
        rtklib
        ${OpenCV_LIBS}
        ${log-lib}
        ${android-lib}
)

# Ensure the heavy ExternalProjects build before we try to compile/link.
add_dependencies(odyx gtsam_ext dbow3_ext)
